#ifdef CONFIG_RELAY_RESCUE_DIAGNOSTICS
#include "frame_ledger.h"
#endif
/* Receiver-side relay coordinator. Pure policy; see relay_coord.h. */
#include "relay_coord.h"

#include <string.h>
#ifdef __ZEPHYR__
#include <zephyr/toolchain.h>
#include <zephyr/irq.h>
#else
#define __ramfunc
#endif

#define RC_BIT(i) ((uint16_t)(1U << (i)))
#define RC_DEFAULT_PAIR_SCORE 500U
#define RC_UNKNOWN_RSSI 70U

static bool rc_after(uint32_t now, uint32_t then, uint32_t interval)
{
	return (uint32_t)(now - then) >= interval;
}

void relay_coord_fast_reset(struct relay_coord *rc)
{
	memset(rc->watch, 0, sizeof(rc->watch));
	memset(rc->receipt, 0, sizeof(rc->receipt));
	rc->published_valid = 0;
	memset(rc->choice, 0, sizeof(rc->choice));
	memset(rc->peer, 0, sizeof(rc->peer));
	rc->learn_members = 0;
	memset(rc->report_ms, 0, sizeof(rc->report_ms));
	rc->report_next = rc->survey_next = 0;
}

void relay_coord_receive(struct relay_coord *rc, uint8_t id, uint32_t frame)
{
#ifdef __ZEPHYR__
	unsigned key = irq_lock();
#endif
	struct rc_receipt *r = &rc->receipt[frame % RELAY_CACHE_FRAMES];
	if (r->frame != frame) {
		r->frame = frame;
		r->received = 0;
	}
	r->received |= 1U << id;
#ifdef __ZEPHYR__
	irq_unlock(key);
#endif
}

/* Repair records are at most two frames old. Retain 32 publication bits so
 * a previously delivered older frame remains distinguishable from a unique
 * frame overtaken by a newer publication. Early radio receipts are separate. */
bool relay_coord_was_published(const struct relay_coord *rc, uint8_t id, uint32_t frame)
{
	uint32_t age = rc->published[id] - frame;
	return (rc->published_valid & (1U << id)) && age < 32 && (rc->published_history[id] & (1U << age));
}

bool relay_coord_publish(struct relay_coord *rc, uint8_t id, uint32_t frame)
{
	relay_coord_receive(rc, id, frame);
	if ((rc->published_valid & (1U << id)) && (int32_t)(frame - rc->published[id]) <= 0) {
		return false;
	}
	uint32_t advance = frame - rc->published[id];
	if (!(rc->published_valid & (1U << id)) || advance >= 32) {
		rc->published_history[id] = 1;
	} else {
		rc->published_history[id] = (rc->published_history[id] << advance) | 1U;
	}
	rc->published_valid |= 1U << id;
	rc->published[id] = frame;
	return true;
}

uint16_t relay_coord_peer_ratio(const struct rc_peer *p)
{
	return p->expected ? (uint32_t)p->heard * 1000U / p->expected : 0;
}

/* Directed evidence older than 10 s says nothing about the current layout. */
static void rc_peer_expire(struct rc_peer *p, uint32_t now)
{
	if ((int32_t)(now - p->evidence_ms) >= 10000) {
		memset(p, 0, sizeof(*p));
	}
}

/* Caller serializes this single live cell against incoming reports. */
void relay_coord_expire_peer(struct relay_coord *rc, unsigned helper, unsigned child, uint32_t now)
{
	rc_peer_expire(&rc->peer[helper][child], now);
}

void relay_coord_capture(struct relay_coord *rc, uint8_t h, uint8_t child, uint8_t rssi, uint32_t now)
{
	struct rc_peer *p = &rc->peer[h][child];
	rc_peer_expire(p, now);
	if (rssi) {
		p->rssi_x8 = relay_rssi_avg_x8(p->rssi_x8, rssi);
	}
	p->evidence_ms = now;
}

bool relay_coord_report(
	struct relay_coord *rc,
	const struct relay_control *c,
	unsigned slot,
	const struct relay_peer_report *r,
	uint32_t now
)
{
	if (r->session != c->session || r->epoch != c->epoch || slot >= c->repair_count
		|| c->grants[slot].owner != r->helper || c->grants[slot].children || !(c->members & RC_BIT(r->helper))) {
		return false;
	}
	for (unsigned i = 0; i < r->count; i++) {
		const struct relay_peer_observation *e = &r->entries[i];
		relay_coord_capture(rc, r->helper, e->child, e->heard ? e->rssi : 0, now);
		struct rc_peer *p = &rc->peer[r->helper][e->child];
		p->expected += e->expected;
		p->heard += e->heard;
		if (p->expected >= 240U) {
			p->expected /= 2U;
			p->heard /= 2U;
		}
	}
	return true;
}

static bool rc_listener(const struct relay_coord *rc, uint8_t child, uint8_t h)
{
	const struct rc_peer *p = &rc->peer[h][child];
	/* Unknown peers may collect provisional evidence. A measured silent
	 * link must yield its repair slots; surveys can establish recovery. */
	return relay_can_overhear(rc->learn_members, h, child) && rc->t[h].direct_permille >= RC_HEAD_MIN_PERMILLE
		&& !(p->expected >= 30U && p->heard == 0);
}

static bool rc_better(const struct relay_coord *rc, uint8_t child, uint8_t a, uint8_t b, const uint8_t *load)
{
	const struct rc_peer *pa = &rc->peer[a][child], *pb = &rc->peer[b][child];
	int diff = (int)relay_coord_peer_ratio(pa) - relay_coord_peer_ratio(pb);
	if (diff > 50 || diff < -50) {
		return diff > 0;
	}
	unsigned ra = pa->rssi_x8 ? pa->rssi_x8 : (rc->t[a].rssi ? 8U * rc->t[a].rssi : 2040U);
	unsigned rb = pb->rssi_x8 ? pb->rssi_x8 : (rc->t[b].rssi ? 8U * rc->t[b].rssi : 2040U);
	if (ra != rb) {
		return ra < rb;
	}
	return load[a] != load[b] ? load[a] < load[b] : a < b;
}

static void rc_learn_preferences(struct relay_coord *rc, uint32_t now)
{
	for (uint8_t c = 0; c < RC_MAX; c++) {
		uint8_t load[RC_MAX] = {0};
		for (unsigned k = 0; k < 2; k++) {
			struct rc_choice *q = &rc->choice[c][k];
			q->valid = q->valid && rc_listener(rc, c, q->helper);
			if (q->valid) {
				load[q->helper]++;
			}
		}
		for (unsigned k = 0; k < 2; k++) {
			struct rc_choice *q = &rc->choice[c][k], *other = &rc->choice[c][1U - k];
			if (q->valid) {
				load[q->helper]--;
			}
			int best = -1;
			for (uint8_t h = 0; h < RC_MAX; h++) {
				if (!rc_listener(rc, c, h) || (k && other->valid && other->helper == h)) {
					continue;
				}
				if (best < 0 || rc_better(rc, c, h, best, load)) {
					best = h;
				}
			}
			bool replace = !q->valid;
			if (q->valid && best >= 0 && best != q->helper) {
				const struct rc_peer *old = &rc->peer[q->helper][c], *p = &rc->peer[best][c];
				int margin = (int)relay_coord_peer_ratio(p) - relay_coord_peer_ratio(old);
				bool beats
					= margin >= 50 || (margin >= -50 && margin <= 50 && p->rssi_x8 && old->rssi_x8 >= p->rssi_x8 + 48U);
				bool failed = old->expected >= 30 && relay_coord_peer_ratio(old) < 900;
				if (failed) {
					replace = true;
				} else if (beats && rc_after(now, q->since_ms, 1000U)) {
					q->streak = q->streak && q->challenger == best ? q->streak + 1U : 1U;
					q->challenger = best;
					replace = q->streak >= 2;
				} else {
					q->streak = 0;
				}
			} else {
				q->streak = 0;
			}
			if (replace) {
				if (other->valid && best == other->helper) {
					load[other->helper]--;
					*other = (struct rc_choice){.valid = q->valid, .helper = q->helper, .since_ms = now};
					if (other->valid) {
						load[other->helper]++;
					}
				}
				*q = (struct rc_choice){.valid = best >= 0, .helper = best < 0 ? 0 : best, .since_ms = now};
			}
			if (q->valid) {
				load[q->helper]++;
			}
		}
	}
}

/* Bounded alternating-path search. Each helper is visited once and each
 * existing choice queued at most once. No recursion on the stats stack. */
static bool rc_cover(
	struct relay_coord *rc,
	struct rc_choice preferred[RC_MAX][2],
	uint8_t *load,
	uint8_t child,
	unsigned kind,
	uint32_t now
)
{
	uint8_t queue[2 * RC_MAX], parent[2 * RC_MAX];
	unsigned count = 1;
	uint16_t visited = 0;
	queue[0] = 2U * child + kind;
	parent[queue[0]] = 0xff;
	for (unsigned at = 0; at < count; at++) {
		unsigned node = queue[at], c = node / 2U, k = node % 2U;
		uint16_t tried = 0;
		for (unsigned n = 0; n < RC_MAX; n++) {
			int best = -1;
			for (uint8_t h = 0; h < RC_MAX; h++) {
				if ((tried & RC_BIT(h)) || (visited & RC_BIT(h)) || !rc_listener(rc, c, h)
					|| (rc->choice[c][1U - k].valid && rc->choice[c][1U - k].helper == h)) {
					continue;
				}
				if (best < 0 || rc_better(rc, c, h, best, load)) {
					best = h;
				}
			}
			const struct rc_choice *want = &preferred[c][k];
			if (!n && want->valid && !(visited & RC_BIT(want->helper)) && rc_listener(rc, c, want->helper)
				&& !(rc->choice[c][1U - k].valid && rc->choice[c][1U - k].helper == want->helper)) {
				best = want->helper;
			}
			if (best < 0) {
				break;
			}
			tried |= RC_BIT(best);
			visited |= RC_BIT(best);
			if (load[best] < RELAY_MAX_CHILDREN) {
				/* Rotate assignments back to the newly covered root. */
				for (unsigned move = node; move != 0xff; move = parent[move]) {
					struct rc_choice *q = &rc->choice[move / 2U][move % 2U];
					const struct rc_choice *pref = &preferred[move / 2U][move % 2U];
					unsigned old = q->helper;
					if (q->valid) {
						load[old]--;
					}
					*q = pref->valid && pref->helper == best
						   ? *pref
						   : (struct rc_choice){.valid = true, .helper = best, .since_ms = now};
					load[best]++;
					best = old;
				}
				return true;
			}
			for (unsigned next = 0; next < 2U * RC_MAX; next++) {
				const struct rc_choice *q = &rc->choice[next / 2U][next % 2U];
				if (q->valid && q->helper == best) {
					parent[next] = node;
					queue[count++] = next;
				}
			}
		}
	}
	return false;
}

void relay_coord_learn_tick(struct relay_coord *rc, uint32_t now)
{
	uint8_t load[RC_MAX] = {0};
	for (unsigned h = 0; h < RC_MAX; h++) {
		for (unsigned c = 0; c < RC_MAX; c++) {
			rc_peer_expire(&rc->peer[h][c], now);
		}
	}
	rc_learn_preferences(rc, now);
	struct rc_choice preferred[RC_MAX][2];
	memcpy(preferred, rc->choice, sizeof(preferred));
	memset(rc->choice, 0, sizeof(rc->choice));
	/* Weak primaries, weak backups, healthy primaries, healthy backups.
	 * Augmentation retains all coverage already allocated at higher priority. */
	for (unsigned phase = 0; phase < 4; phase++) {
		for (uint8_t c = 0; c < RC_MAX; c++) {
			bool weak = rc->t[c].direct_permille < RC_HEAD_MIN_PERMILLE;
			if (weak != (phase < 2)) {
				continue;
			}
			(void)rc_cover(rc, preferred, load, c, phase % 2, now);
		}
	}
}

void relay_coord_plan_watch(struct relay_coord *rc, struct relay_control *c)
{
	rc->learn_members = c->members;
	c->primary_valid = c->backup_valid = 0;
	memset(c->helpers, 0, sizeof(c->helpers));
	for (uint8_t child = 0; child < RC_MAX; child++) {
		for (unsigned k = 0; k < 2; k++) {
			struct rc_choice *q = &rc->choice[child][k];
			if (!q->valid || !rc_listener(rc, child, q->helper)) {
				continue;
			}
			c->helpers[child] |= q->helper << (k * 4U);
			if (k) {
				c->backup_valid |= RC_BIT(child);
			} else {
				c->primary_valid |= RC_BIT(child);
			}
		}
	}
}

static uint16_t rc_watch_helpers(const struct rc_watch *w, unsigned child)
{
	if (w->survey & RC_BIT(child)) {
		return w->members;
	}
	uint16_t helpers = 0;
	if (w->primary_valid & RC_BIT(child)) {
		helpers |= RC_BIT(w->helpers[child] & 15U);
	}
	if (w->backup_valid & RC_BIT(child)) {
		helpers |= RC_BIT(w->helpers[child] >> 4);
	}
	return helpers;
}

static bool rc_rescue_better(
	const struct relay_coord *rc,
	uint16_t tried,
	unsigned child,
	unsigned h,
	int incumbent,
	const uint8_t *load
)
{
	if (incumbent < 0) {
		return true;
	}
	bool old = (tried & RC_BIT(incumbent)) != 0;
	bool retry = (tried & RC_BIT(h)) != 0;
	return old != retry ? old : rc_better(rc, child, h, incumbent, load);
}

/* Capture history is permission to listen, not proof of capture. Try up to two eligible
 * alternates for a missing record, oldest first, never inferred from RSSI alone.
 * The frozen step rechecks that the selected record is still missing. */
static void rc_plan_rescue(const struct relay_coord *rc, const struct relay_control *c, struct rc_grant_plan *p)
{
	for (unsigned remaining = RELAY_CACHE_FRAMES; remaining > 0; remaining--) {
		unsigned age = remaining - 1U;
		uint32_t frame = c->frame - age;
		const struct rc_watch *w = &rc->watch[frame % RELAY_CACHE_FRAMES];
		const struct rc_receipt *r = &rc->receipt[frame % RELAY_CACHE_FRAMES];
		if (w->frame != frame || w->session != c->session || w->epoch != c->epoch || w->members != c->members) {
			continue;
		}
		uint16_t missing = c->expected & ~(r->frame == frame ? r->received : 0);
		while (missing) {
			unsigned child = __builtin_ctz((unsigned)missing);
			missing &= missing - 1U;
			uint16_t listeners = w->listeners[child] | rc_watch_helpers(w, child);
			int best = -1, second = -1;
			uint16_t tried = w->rescue_tried[child];
			const uint8_t load[RC_MAX] = {0};
			for (unsigned h = 0; h < RC_MAX; h++) {
				if (!(listeners & RC_BIT(h)) || ((w->primary_valid & RC_BIT(child)) && (w->helpers[child] & 15U) == h)
					|| ((w->backup_valid & RC_BIT(child)) && (w->helpers[child] >> 4) == h)
					|| !relay_can_overhear(w->members, h, child) || !rc_listener(rc, child, h)) {
					continue;
				}
				if (rc_rescue_better(rc, tried, child, h, best, load)) {
					second = best;
					best = h;
				} else if (rc_rescue_better(rc, tried, child, h, second, load)) {
					second = h;
				}
			}
			if (best >= 0) {
				p->rescue_child = RC_BIT(child);
				p->rescue_helper = best;
				p->rescue_second = second < 0 ? RELAY_OWNER_IDLE : (unsigned)second;
				p->rescue_age = age;
				return;
			}
		}
	}
}

/* Diagnostic only, before freeze. Flags: listener, primary, backup,
 * topology, uplink, silent, tried, eligible, selected, other choice. Rank
 * includes ineligible peers so the historical ordering can be reconstructed. */
#ifdef CONFIG_RELAY_RESCUE_DIAGNOSTICS
static void
rc_rescue_diagnose(const struct relay_coord *rc, const struct relay_control *c, const struct rc_grant_plan *p)
{
	const uint8_t load[RC_MAX] = {0};
	for (unsigned age = 0; age < RELAY_CACHE_FRAMES; age++) {
		uint32_t frame = c->frame - age;
		const struct rc_watch *w = &rc->watch[frame % RELAY_CACHE_FRAMES];
		const struct rc_receipt *r = &rc->receipt[frame % RELAY_CACHE_FRAMES];
		if (w->frame != frame || w->session != c->session || w->epoch != c->epoch || w->members != c->members) {
			continue;
		}
		uint16_t missing = c->expected & ~(r->frame == frame ? r->received : 0);
		for (unsigned child = 0; child < RC_MAX; child++) {
			if (!(missing & RC_BIT(child)) || !ledger_child_enabled(child)) {
				continue;
			}
			uint16_t listeners = w->listeners[child] | rc_watch_helpers(w, child);
			uint16_t tried = w->rescue_tried[child];
			for (unsigned h = 0; h < RC_MAX; h++) {
				bool listener = listeners & RC_BIT(h);
				bool primary = (w->primary_valid & RC_BIT(child)) && (w->helpers[child] & 15U) == h;
				bool backup = (w->backup_valid & RC_BIT(child)) && (w->helpers[child] >> 4) == h;
				bool topology
					= relay_can_overhear(w->members, h, child) && relay_can_overhear(rc->learn_members, h, child);
				bool uplink = rc->t[h].direct_permille >= RC_HEAD_MIN_PERMILLE;
				bool silent = rc->peer[h][child].expected >= 30U && rc->peer[h][child].heard == 0;
				bool eligible = listener && !primary && !backup && topology && uplink && !silent;
				bool choice = p->rescue_child == RC_BIT(child) && p->rescue_age == age;
				unsigned flags = listener | (primary << 1) | (backup << 2) | (topology << 3) | (uplink << 4)
							   | (silent << 5) | (!!(tried & RC_BIT(h)) << 6) | (eligible << 7)
							   | (choice && (p->rescue_helper == h || p->rescue_second == h) ? 256U : 0U)
							   | (p->rescue_child && !choice ? 512U : 0U);
				unsigned rank = 0;
				for (unsigned j = 0; j < RC_MAX; j++) {
					if (j != h && rc_rescue_better(rc, tried, child, j, h, load)) {
						rank++;
					}
				}
				ledger_note(c->session, frame, child, LEDGER_CANDIDATE_FLAGS, flags, h, age);
				ledger_note(c->session, frame, child, LEDGER_CANDIDATE_UPLINK, rc->t[h].direct_permille, h, age);
				ledger_note(c->session, frame, child, LEDGER_CANDIDATE_RANK, rank, h, age);
			}
		}
	}
}
#endif

/* Selection scans run before the mask freeze. Ties retain rotation order. */
void relay_coord_plan_grants(struct relay_coord *rc, struct relay_control *c, struct rc_grant_plan *p, uint32_t now_ms)
{
	for (unsigned i = 0; i < c->repair_count; i++) {
		c->grants[i] = (struct relay_grant){.owner = RELAY_OWNER_IDLE};
	}
	c->maintenance_owner = RELAY_OWNER_IDLE;
	c->flags &= ~RELAY_CONTROL_MAINTENANCE;
	*p = (struct rc_grant_plan){.now_ms = now_ms, .maintenance = -1, .reporter = -1};
	uint32_t oldest_m = 0, oldest_r = 0;
	for (unsigned n = 0; n < RC_MAX; n++) {
		unsigned id = (rc->maintenance_next + n) % RC_MAX;
		if (rc->t[id].direct_permille < RC_HEAD_MIN_PERMILLE) {
			p->fresh_survey |= RC_BIT(id);
		}
		uint32_t age = now_ms - rc->maintenance_ms[id];
		if ((c->members & RC_BIT(id)) && age >= RC_MAINTENANCE_MS && (p->maintenance < 0 || age > oldest_m)) {
			p->maintenance = id;
			oldest_m = age;
		}
		id = (rc->report_next + n) % RC_MAX;
		age = now_ms - rc->report_ms[id];
		if ((c->members & RC_BIT(id)) && age >= 1000U && (p->reporter < 0 || age > oldest_r)) {
			p->reporter = id;
			oldest_r = age;
		}
	}
	/* A tracker that misses control frame can still listen under either of the preceding
	 * two control frame plans. Match its unchanged three-frame control lifetime exactly.
	 * Keep raw plans separate so inherited listeners cannot extend themselves. */
	for (unsigned child = 0; child < RC_MAX; child++) {
		if (c->primary_valid & RC_BIT(child)) {
			p->next_listeners[child] |= RC_BIT(c->helpers[child] & 15U);
		}
		if (c->backup_valid & RC_BIT(child)) {
			p->next_listeners[child] |= RC_BIT(c->helpers[child] >> 4);
		}
	}
	for (unsigned age = 0; age < RELAY_CACHE_FRAMES - 1U; age++) {
		uint32_t frame = c->frame - age;
		const struct rc_watch *w = &rc->watch[frame % RELAY_CACHE_FRAMES];
		if (w->frame != frame || w->session != c->session || w->epoch != c->epoch || w->members != c->members) {
			continue;
		}
		for (unsigned child = 0; child < RC_MAX; child++) {
			p->next_listeners[child] |= rc_watch_helpers(w, child);
		}
	}
	/* A repaired weak uplink still needs alternate copies on the next frame.
	 * Keep the existing two-survey cap; precompute the fresh mask before freeze. */
	p->fresh_survey |= ~(c->primary_valid | c->backup_valid);
	p->fresh_survey &= c->members;
	rc_plan_rescue(rc, c, p);
#ifdef CONFIG_RELAY_RESCUE_DIAGNOSTICS
	rc_rescue_diagnose(rc, c, p);
#endif
	p->urgent = oldest_m >= RC_MAINTENANCE_URGENT_MS;
	if (c->frame % 10U == 0) {
		for (unsigned n = 0; n < RC_MAX; n++) {
			unsigned child = (rc->survey_next + n) % RC_MAX;
			if (c->members & RC_BIT(child)) {
				p->survey = RC_BIT(child);
				p->survey_next = (child + 1U) % RC_MAX;
				break;
			}
		}
	} else {
		/* Reactive surveys begin after a miss. Keep extra listeners ready
		 * for the next isolated miss on an intermittent weak direct path.
		 * One preplanned survey leaves the existing second survey available
		 * for missing records. Periodic exploration keeps its own cursor. */
		unsigned weakest = RC_HEAD_MIN_PERMILLE;
		for (unsigned n = 0; n < RC_MAX; n++) {
			unsigned child = (rc->survey_next + n) % RC_MAX;
			unsigned direct = rc->t[child].direct_permille;
			if ((c->members & RC_BIT(child)) && (direct || (c->expected & RC_BIT(child))) && direct < weakest) {
				weakest = direct;
				p->survey = RC_BIT(child);
			}
		}
		p->survey_next = rc->survey_next;
		/* A fully blocked but active source still needs listening coverage.
		 * Rotate zero-direct ties so simultaneous drops share that coverage. */
		if (p->survey && weakest == 0) {
			p->survey_next = (__builtin_ctz((unsigned)p->survey) + 1U) % RC_MAX;
		}
	}
	for (unsigned age = 0; age < RELAY_CACHE_FRAMES; age++) {
		uint32_t frame = c->frame - age;
		const struct rc_watch *w = &rc->watch[frame % RELAY_CACHE_FRAMES];
		p->receipt_index[age] = frame % RELAY_CACHE_FRAMES;
		if (w->frame == frame) {
			p->watch[age] = *w;
			p->valid |= 1U << age;
			for (unsigned copy = 0; copy < 2; copy++) {
				uint16_t children = copy ? w->backup_valid : w->primary_valid;
				while (children) {
					unsigned child = __builtin_ctz((unsigned)children);
					children &= children - 1U;
					/* On an aged miss, alternate the lead lane when both original
					 * reservations exist. The primary then gets a later slot
					 * instead of repeatedly racing R0 with retained records.
					 * Even frames lead with backup, odd frames with primary.
					 * Fresh repairs and single-helper records keep their order. */
					unsigned selected_copy
						= age && !(c->frame & 1U) && (w->primary_valid & w->backup_valid & RC_BIT(child)) ? 1U - copy
																										  : copy;
					unsigned h = (w->helpers[child] >> (selected_copy * 4U)) & 15U;
					/* The planned alternate must not wait behind another failed
					 * reservation for this exact retained frame. Reuse the one
					 * existing rescue search; keep the other original lane. */
					if (age && !copy && p->rescue_age == age && p->rescue_child == RC_BIT(child)
						&& (w->rescue_tried[child] & RC_BIT(h))
						&& !(w->rescue_tried[child] & RC_BIT(p->rescue_helper))) {
						h = p->rescue_helper;
					}
					unsigned position = (copy * 3U + 2U - age) * 16U + child;
					p->helper[position / 32U][position % 32U] = h;
					p->children[position / 32U][h] |= 1U << (position % 32U);
					p->candidates[position / 32U] |= 1U << (position % 32U);
				}
			}
		}
	}
}

static void rc_apply_maintenance(struct relay_control *c, const struct rc_grant_plan *p)
{
	c->maintenance_owner = p->maintenance;
	c->flags |= RELAY_CONTROL_MAINTENANCE;
	c->grants[0] = c->grants[1] = (struct relay_grant){.owner = RELAY_OWNER_MAINTENANCE};
}

/* Attribute provenance: Frozen allocator is included in linked 10,000-payload equivalence and round 11 encode-cycle
 * evidence. Retain its call/placement boundary. */
static inline __attribute__((always_inline)) uint16_t
rc_take_records(uint32_t *pending, uint32_t candidates, unsigned *capacity)
{
	uint32_t records = *pending & candidates;
	uint32_t rest = records;
	while (rest && *capacity) {
		rest &= rest - 1U;
		(*capacity)--;
	}
	uint32_t selected = records ^ rest;
	*pending &= ~selected;
	return selected | (selected >> 16);
}

#if defined(__GNUC__) && !defined(__clang__)
/* Attribute provenance: Frozen allocator is included in linked 10,000-payload equivalence and round 11 encode-cycle
 * evidence. Retain its call/placement boundary. */
__attribute__((optimize("O3")))
#endif
__ramfunc void relay_coord_finish_frozen(struct relay_control *c, const struct rc_grant_plan *p, uint8_t *wire)
{

	unsigned idle = 0;
	if (p->urgent) {
		rc_apply_maintenance(c, p);
		wire[RELAY_CONTROL_OFF_OWNERS] = wire[RELAY_CONTROL_OFF_OWNER_SECOND] = RELAY_OWNER_MAINTENANCE;
		idle = 2;
	}

	/* Ordered candidates: primary oldest first, then backup oldest first.
	 * Fill one helper's next three records ahead of the scan. Removing those
	 * records leaves exactly the next slot-opening record of the single-pass
	 * algorithm, including a fourth record for the same helper. */
	uint32_t a = (c->missing[2] | ((uint32_t)c->missing[1] << 16)) & p->candidates[0];
	uint32_t b = (c->missing[0] | ((uint32_t)c->missing[2] << 16)) & p->candidates[1];
	uint32_t d = (c->missing[1] | ((uint32_t)c->missing[0] << 16)) & p->candidates[2];
	while (idle < c->repair_count && (a | b | d)) {
		unsigned helper = a ? p->helper[0][__builtin_ctz(a)]
						: b ? p->helper[1][__builtin_ctz(b)]
							: p->helper[2][__builtin_ctz(d)];
		unsigned capacity = RELAY_MAX_CHILDREN;
		uint16_t children = rc_take_records(&a, p->children[0][helper], &capacity);
		if (capacity) {
			children |= rc_take_records(&b, p->children[1][helper], &capacity);
		}
		if (capacity) {
			children |= rc_take_records(&d, p->children[2][helper], &capacity);
		}
		wire[RELAY_CONTROL_OFF_OWNERS + idle] = helper;
		relay_put_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2U * idle, children);
		idle++;
	}

	unsigned repair_end = idle;
	if (p->maintenance >= 0 && idle == 0) {
		rc_apply_maintenance(c, p);
		wire[RELAY_CONTROL_OFF_OWNERS] = wire[RELAY_CONTROL_OFF_OWNER_SECOND] = RELAY_OWNER_MAINTENANCE;
		idle = 2;
	}
	c->survey = p->survey;
	uint16_t failed = c->missing[0] & p->fresh_survey;
	for (unsigned age = 1; age < RELAY_CACHE_FRAMES; age++) {
		failed |= c->missing[age];
	}
	failed &= c->members;
	unsigned surveys = c->survey != 0;
	failed &= ~c->survey;
	while (failed && surveys < 2) {
		unsigned child = __builtin_ctz((unsigned)failed);
		failed &= failed - 1U;
		surveys++;
		c->survey |= RC_BIT(child);
	}
	if (p->reporter >= 0 && idle < c->repair_count) {
		wire[RELAY_CONTROL_OFF_OWNERS + idle++] = p->reporter;
	}
	/* Prefer a surveyed alternate over a same-helper retry in otherwise idle
	 * capacity. Helpers retain refused records but suppress successful ones.
	 * All distinct repairs, maintenance and reports keep their priority. */
	unsigned retry = p->urgent ? 2U : 0U;
	if (idle < c->repair_count && (c->missing[p->rescue_age] & p->rescue_child)) {
		wire[RELAY_CONTROL_OFF_OWNERS + idle] = p->rescue_helper;
		relay_put_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2U * idle, p->rescue_child);
		/* A second distinct survey copy uses only another otherwise idle slot. */
		if (++idle < c->repair_count && p->rescue_second < RC_MAX) {
			wire[RELAY_CONTROL_OFF_OWNERS + idle] = p->rescue_second;
			relay_put_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2U * idle, p->rescue_child);
		}
	} else if (idle < c->repair_count && retry < repair_end) {
		wire[RELAY_CONTROL_OFF_OWNERS + idle] = wire[RELAY_CONTROL_OFF_OWNERS + retry];
		wire[RELAY_CONTROL_OFF_CHILDREN + 2U * idle] = wire[RELAY_CONTROL_OFF_CHILDREN + 2U * retry];
		wire[RELAY_CONTROL_OFF_CHILDREN_LOW + 2U * idle] = wire[RELAY_CONTROL_OFF_CHILDREN_LOW + 2U * retry];
	}
}

void relay_coord_read_grants(struct relay_control *c, const uint8_t *wire)
{
	for (unsigned i = 0; i < c->repair_count; i++) {
		c->grants[i].owner = wire[RELAY_CONTROL_OFF_OWNERS + i];
		c->grants[i].children = relay_get_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2U * i);
	}
}

void relay_coord_commit_grants(struct relay_coord *rc, const struct relay_control *c, const struct rc_grant_plan *p)
{
	if (p->survey) {
		rc->survey_next = p->survey_next;
	}
	if (c->flags & RELAY_CONTROL_MAINTENANCE) {
		rc->maintenance_ms[p->maintenance] = p->now_ms;
		rc->maintenance_next = (p->maintenance + 1U) % RC_MAX;
	}
	for (unsigned slot = 0; p->reporter >= 0 && slot < c->repair_count; slot++) {
		if (c->grants[slot].owner == p->reporter && !c->grants[slot].children) {
			rc->report_ms[p->reporter] = p->now_ms;
			rc->report_next = (p->reporter + 1U) % RC_MAX;
			break;
		}
	}
	/* Grants authorize a child, not a frame. Every still-missing record in
	 * that child's cache had this opportunity, including younger records than
	 * the planner's oldest choice. Record only actual frozen reservations,
	 * independently per child, so persistent drops rotate across helpers. */
	for (unsigned age = 0; age < RELAY_CACHE_FRAMES; age++) {
		uint32_t frame = c->frame - age;
		struct rc_watch *w = &rc->watch[frame % RELAY_CACHE_FRAMES];
		if (w->frame != frame || w->session != c->session || w->epoch != c->epoch || w->members != c->members) {
			continue;
		}
		for (unsigned slot = 0; slot < c->repair_count; slot++) {
			const struct relay_grant *g = &c->grants[slot];
			if (g->owner >= RC_MAX) {
				continue;
			}
			uint16_t children = g->children & c->missing[age];
			while (children) {
				unsigned child = __builtin_ctz((unsigned)children);
				children &= children - 1U;
				w->rescue_tried[child] |= RC_BIT(g->owner);
			}
		}
	}
	struct rc_watch *next = &rc->watch[(c->frame + 1U) % RELAY_CACHE_FRAMES];
	*next = (struct rc_watch){
		.frame = c->frame + 1U,
		.session = c->session,
		.epoch = c->epoch,
		.members = c->members,
		.survey = c->survey,
		.primary_valid = c->primary_valid,
		.backup_valid = c->backup_valid
	};
	memcpy(next->helpers, c->helpers, sizeof(next->helpers));
	for (unsigned child = 0; child < RC_MAX; child++) {
		next->listeners[child] = (c->survey & RC_BIT(child)) ? c->members : p->next_listeners[child];
	}
}
