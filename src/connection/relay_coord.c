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

void relay_coord_init(struct relay_coord *rc)
{
	memset(rc, 0, sizeof(*rc));
	rc->enabled = true;
	for (unsigned c = 0; c < RC_MAX; c++) {
		for (unsigned h = 0; h < RC_MAX; h++) {
			rc->pair_score[c][h] = RC_DEFAULT_PAIR_SCORE;
		}
	}
}

void relay_coord_note_caps(struct relay_coord *rc, uint8_t id, uint8_t caps)
{
	if (id < RC_MAX) {
		uint8_t version = caps >> RELAY_CAP_VERSION_SHIFT;
		rc->t[id].protocol_version = version;
		rc->t[id].capable = (caps & RELAY_CAP_HELPER) != 0
			&& (version == RELAY_PROTOCOL_VERSION || version == RELAY_V3_VERSION);
		rc->t[id].fwd_requested = rc->t[id].capable
			&& (caps & RELAY_CAP_FORWARD_REQUEST) != 0;
	}
}

static void rc_release_child(struct relay_coord *rc, uint8_t child)
{
	struct rc_tracker *c = &rc->t[child];
	if (c->role != RC_ROLE_CHILD) {
		return;
	}
	struct rc_tracker *h = &rc->t[c->head];
	h->children &= (uint16_t)~RC_BIT(child);
	if (h->children == 0 && h->role == RC_ROLE_HEAD) {
		h->role = RC_ROLE_NONE;
	}
	c->role = RC_ROLE_NONE;
	rc->relayed_mask &= (uint16_t)~RC_BIT(child);
}

static void rc_release_head(struct relay_coord *rc, uint8_t head)
{
	uint16_t children = rc->t[head].children;
	for (uint8_t c = 0; c < RC_MAX; c++) {
		if (children & RC_BIT(c)) {
			rc_release_child(rc, c);
		}
	}
	rc->t[head].role = RC_ROLE_NONE;
	rc->t[head].children = 0;
}

void relay_coord_forget(struct relay_coord *rc, uint8_t id, uint32_t now_ms)
{
	if (id >= RC_MAX) {
		return;
	}
	rc_release_child(rc, id);
	rc_release_head(rc, id);
	rc->published_valid &= ~(1U << id);
	for (unsigned i = 0; i < RELAY_CACHE_MAX_FRAMES; i++) {
		rc->receipt[i].received &= ~(1U << id);
	}
	bool fwd = rc->t[id].fwd_wanted || rc->t[id].fwd_revoking;
	memset(&rc->t[id], 0, sizeof(rc->t[id]));
	if (fwd) {
		/* Keep the slot until the layout drops it; the tracker is gone. */
		rc->t[id].fwd_revoking = true;
		rc->t[id].fwd_revoke_ms = now_ms;
	}
	memset(&rc->w[id], 0, sizeof(rc->w[id]));
	for (unsigned i = 0; i < RC_MAX; i++) {
		rc->ban_until[id][i] = rc->ban_until[i][id] = 0;
		rc->pair_score[id][i] = rc->pair_score[i][id] = RC_DEFAULT_PAIR_SCORE;
		memset(&rc->peer[id][i], 0, sizeof(rc->peer[id][i]));
		memset(&rc->peer[i][id], 0, sizeof(rc->peer[i][id]));
	}
}

void relay_coord_note_frame(struct relay_coord *rc, uint8_t id, enum rc_path path,
	int seq_result, uint8_t gaps, uint8_t rssi, uint32_t now_ms)
{
	if (id >= RC_MAX) {
		return;
	}
	struct rc_tracker *t = &rc->t[id];
	struct rc_window *w = &rc->w[id];
	bool accepted = seq_result == RC_SEQ_NORMAL || seq_result == RC_SEQ_GAP || seq_result == RC_SEQ_RESTART;
	t->last_any_ms = now_ms;
	t->seen = true;
	if (path == RC_DIRECT) {
		t->last_direct_ms = now_ms;
		t->direct_seen = true;
		if (rssi) {
			w->rssi_sum += rssi;
			w->rssi_count++;
		}
		if (accepted) {
			w->direct++;
		} else if (seq_result == RC_SEQ_DUP) {
			w->direct_dup++;
		}
	} else if (accepted) {
		w->relayed++;
		if (t->role == RC_ROLE_CHILD) {
			t->eval_relayed++;
		}
	} else if (seq_result == RC_SEQ_DUP) {
		w->relayed_dup++;
	}
	if (accepted && seq_result != RC_SEQ_RESTART) {
		/* A gap is a frame lost on both paths: a relayed copy always arrives
		 * before the child's next frame, so it would have filled the gap. */
		w->gaps += gaps;
		if (t->role == RC_ROLE_CHILD) {
			t->eval_gaps += gaps;
		}
	}
}

void relay_coord_note_bundle(struct relay_coord *rc, uint8_t head, bool overflow)
{
	if (head < RC_MAX && overflow) {
		rc->w[head].overflow++;
	}
}

void relay_coord_note_trailer_sent(struct relay_coord *rc, uint8_t head, uint16_t children, uint32_t now_ms)
{
	if (RC_ID_OUTSIDE(head)) {
		return;
	}
	for (uint8_t c = 0; c < RC_MAX; c++) {
		struct rc_tracker *t = &rc->t[c];
		if ((children & RC_BIT(c)) && t->role == RC_ROLE_CHILD && t->head == head && !t->delivered) {
			t->delivered = true;
			t->delivered_ms = now_ms;
			t->eval_relayed = 0;
			t->eval_gaps = 0;
		}
	}
}

static uint8_t rc_count(uint16_t mask)
{
	return relay_mask_count(mask);
}

static uint8_t rc_head_count(const struct relay_coord *rc)
{
	uint8_t heads = 0;
	for (unsigned i = 0; i < RC_MAX; i++) {
		heads += rc->t[i].role == RC_ROLE_HEAD;
	}
	return heads;
}

struct rc_judgement {
	uint16_t expected;
	uint16_t permille;
};

static struct rc_judgement rc_judge(const struct rc_window *w)
{
	struct rc_judgement j;
	j.expected = (uint16_t)(w->direct + w->relayed + w->gaps);
	j.permille = j.expected ? (uint16_t)((uint32_t)w->direct * 1000U / j.expected) : 1000U;
	return j;
}

/* Judge once RC_MIN_EXPECTED frames have accumulated; expected == 0 means
 * no judgement this window. */
static struct rc_judgement rc_accumulate(struct rc_tracker *t, const struct rc_window *w)
{
	struct rc_judgement in = rc_judge(w);
	struct rc_judgement out = {0, 1000U};
	t->acc_direct = (uint16_t)(t->acc_direct + w->direct);
	t->acc_expected = (uint16_t)(t->acc_expected + in.expected);
	t->acc_windows++;
	if (t->acc_expected >= RC_MIN_EXPECTED) {
		out.expected = t->acc_expected;
		out.permille = (uint16_t)((uint32_t)t->acc_direct * 1000U / t->acc_expected);
	}
	if (out.expected || t->acc_windows >= RC_JUDGE_MAX_WINDOWS) {
		t->acc_direct = 0;
		t->acc_expected = 0;
		t->acc_windows = 0;
	}
	return out;
}

/* Missed about RC_MIN_EXPECTED frames at the tracker's own rate. The rate is
 * learned from windows that carried frames, so it does not decay while the
 * tracker is obstructed; an unknown rate never counts as silence. */
static bool rc_direct_quiet(const struct rc_tracker *t, uint32_t now)
{
	if (!t->direct_seen || !rc_after(now, t->last_direct_ms, RC_SILENT_MIN_MS - 1U)) {
		return false;
	}
	uint32_t gap = now - t->last_direct_ms;
	return (uint64_t)gap * t->rate_x16 >= (uint64_t)RC_MIN_EXPECTED * 16U * RC_WINDOW_MS;
}

static bool rc_head_ok(const struct relay_coord *rc, uint8_t id, uint16_t members, uint32_t now)
{
	const struct rc_tracker *t = &rc->t[id];
	const struct rc_window *w = &rc->w[id];
	struct rc_judgement j = rc_judge(w);
	if (!t->capable || !(members & RC_BIT(id)) || t->role == RC_ROLE_CHILD || !t->direct_seen
	    || rc_after(now, t->last_direct_ms, 2U * RC_WINDOW_MS) || w->direct == 0) {
		return false;
	}
	return j.expected < RC_MIN_EXPECTED || j.permille >= RC_HEAD_MIN_PERMILLE;
}

static int rc_pick_head(const struct relay_coord *rc, uint8_t child, uint16_t members, uint32_t now)
{
	int best = -1;
	int32_t best_score = 0;
	uint8_t heads = rc_head_count(rc);
	for (uint8_t h = 0; h < RC_MAX; h++) {
		const struct rc_tracker *t = &rc->t[h];
		if (h == child || !rc_head_ok(rc, h, members, now)) {
			continue;
		}
		if (t->role == RC_ROLE_HEAD ? rc_count(t->children) >= RELAY_MAX_CHILDREN : heads >= RELAY_MAX_HEADS) {
			continue;
		}
		uint32_t ban = rc->ban_until[child][h];
		if (ban && (int32_t)(ban - now) > 0) {
			continue;
		}
		uint8_t rssi = t->rssi ? t->rssi : RC_UNKNOWN_RSSI;
		int32_t score = (int32_t)rc->pair_score[child][h]
			+ ((rc->pinned & RC_BIT(h)) ? 4000 : 0)
			+ (t->role == RC_ROLE_HEAD ? 300 : 0)
			- (int32_t)rssi * 5;
		if (best < 0 || score > best_score) {
			best = h;
			best_score = score;
		}
	}
	return best;
}

static void rc_ban(struct relay_coord *rc, uint8_t child, uint8_t head, uint32_t now, uint32_t ms)
{
	rc->ban_until[child][head] = (now + ms) | 1U; /* 0 means none */
}

uint16_t relay_coord_forward_wanted(const struct relay_coord *rc, uint16_t member_mask)
{
	uint8_t room = RC_MAX_TOTAL_SLOTS > rc_count(member_mask)
		? (uint8_t)(RC_MAX_TOTAL_SLOTS - rc_count(member_mask)) : 0;
	uint16_t mask = 0;
	/* A slot being withdrawn stays until its wait elapses: never shrink a
	 * slot a head may still transmit in. Then wanted slots, by id. */
	for (uint8_t pass = 0; pass < 2; pass++) {
		for (uint8_t h = 0; h < RC_MAX && room; h++) {
			const struct rc_tracker *t = &rc->t[h];
			bool take = pass == 0 ? t->fwd_revoking && (rc->forward_mask & RC_BIT(h))
				: t->fwd_wanted && !t->fwd_revoking && (member_mask & RC_BIT(h));
			if (take && !(mask & RC_BIT(h))) {
				mask |= RC_BIT(h);
				room--;
			}
		}
	}
	return mask;
}

uint16_t relay_coord_tick(struct relay_coord *rc, uint16_t member_mask, uint32_t now)
{
	struct rc_judgement j[RC_MAX];
	struct rc_judgement jd[RC_MAX]; /* accumulated, see rc_accumulate() */
	bool weak[RC_MAX] = {false};

	for (uint8_t i = 0; i < RC_MAX; i++) {
		struct rc_tracker *t = &rc->t[i];
		struct rc_window *w = &rc->w[i];
		j[i] = rc_judge(w);
		if (w->rssi_count) {
			uint8_t avg = (uint8_t)(w->rssi_sum / w->rssi_count);
			t->rssi = t->rssi ? (uint8_t)((3U * t->rssi + avg) / 4U) : avg;
		}
		if (j[i].expected) {
			uint32_t sample = 16U * j[i].expected;
			uint32_t rate = t->rate_x16 ? (3U * t->rate_x16 + sample) / 4U : sample;
			t->rate_x16 = (uint16_t)(rate > UINT16_MAX ? UINT16_MAX : rate);
		}
		jd[i] = rc_accumulate(t, w);
		bool member = (member_mask & RC_BIT(i)) != 0;
		if (jd[i].expected) {
			t->direct_permille = jd[i].permille;
			if (jd[i].permille < RC_WEAK_PERMILLE) {
				if (t->weak_windows < UINT8_MAX) {
					t->weak_windows++;
				}
			} else {
				t->weak_windows = 0;
			}
			/* Direct recovery after a lost head, with no child role to
			 * release, ends the relay need too. */
			if (t->role == RC_ROLE_NONE && jd[i].permille >= RC_GOOD_PERMILLE) {
				t->relay_needed = false;
			}
		}
		bool loss = t->weak_windows >= RC_WEAK_WINDOWS;
		/* Direct silence of a member that was sending directly: obstructed
		 * (gaps are only counted once a later frame arrives). */
		bool quiet = w->direct == 0 && rc_direct_quiet(t, now);
		bool silent = quiet && !rc_after(now, t->last_direct_ms, RC_SILENT_WEAK_MS);
		/* Losing the only working path reveals no sequence gap. Preserve
		 * eligibility long enough to try a different head, including when
		 * the old head disappears after minutes of successful relay. */
		bool retry = t->relay_needed && quiet
			&& !rc_after(now, t->last_any_ms, RC_CHILD_RECOVERY_MS);
		weak[i] = member && t->seen && (loss || silent || retry);
	}

	/* 1. Heads must stay healthy members, or they lose their children. */
	for (uint8_t h = 0; h < RC_MAX; h++) {
		struct rc_tracker *t = &rc->t[h];
		if (t->role != RC_ROLE_HEAD) {
			continue;
		}
		bool member = (member_mask & RC_BIT(h)) != 0;
		bool lossy = j[h].expected >= RC_MIN_EXPECTED && j[h].permille < RC_HEAD_DROP_PERMILLE;
		if (!member || !rc->enabled || !t->capable || lossy || rc->w[h].direct == 0) {
			rc_release_head(rc, h);
			rc->rotations++;
		}
	}

	/* 2. Children: release, rotate or keep. */
	for (uint8_t c = 0; c < RC_MAX; c++) {
		struct rc_tracker *t = &rc->t[c];
		if (t->role != RC_ROLE_CHILD) {
			continue;
		}
		if (!(member_mask & RC_BIT(c))) {
			t->relay_needed = false;
			rc_release_child(rc, c);
			rc->releases++;
			continue;
		}
		/* Count silence since this attempt began, rather than immediately
		 * expiring each replacement against the same old last_any_ms. */
		if (rc_after(now, t->last_any_ms, RC_CHILD_SILENT_MS)
		    && rc_after(now, t->assigned_ms, RC_CHILD_SILENT_MS)) {
			rc_ban(rc, c, t->head, now, RC_SILENT_BAN_MS);
			rc_release_child(rc, c);
			rc->releases++;
			continue;
		}
		if (!t->delivered) {
			if (rc_after(now, t->assigned_ms, RC_DELIVERY_TIMEOUT_MS)) {
				rc_ban(rc, c, t->head, now, RC_BAN_MS / 4U);
				rc_release_child(rc, c);
				rc->rotations++;
				continue;
			}
		} else if (rc_after(now, t->delivered_ms, RC_EVAL_AFTER_MS)) {
			uint16_t lost = (uint16_t)(t->eval_relayed + t->eval_gaps);
			if (lost >= RC_EVAL_MIN_LOST) {
				uint16_t share = (uint16_t)((uint32_t)t->eval_relayed * 1000U / lost);
				uint16_t *score = &rc->pair_score[c][t->head];
				*score = (uint16_t)((3U * *score + share) / 4U);
				t->eval_relayed = 0;
				t->eval_gaps = 0;
				if (share < RC_EVAL_FAIL_PERMILLE) {
					rc_ban(rc, c, t->head, now, RC_BAN_MS);
					rc_release_child(rc, c);
					rc->rotations++;
					continue;
				}
			}
		}
		if (jd[c].expected && jd[c].permille >= RC_GOOD_PERMILLE) {
			if (t->good_windows < UINT8_MAX) {
				t->good_windows++;
			}
		} else if (jd[c].expected) {
			t->good_windows = 0;
		}
		if (t->good_windows >= RC_GOOD_WINDOWS && rc_after(now, t->assigned_ms, RC_MIN_HOLD_MS)) {
			t->relay_needed = false;
			rc_release_child(rc, c);
			rc->releases++;
		}
	}

	/* 3. Assign heads to weak trackers without one. */
	for (uint8_t c = 0; rc->enabled && c < RC_MAX; c++) {
		struct rc_tracker *t = &rc->t[c];
		if (!weak[c] || t->role != RC_ROLE_NONE) {
			continue;
		}
		int h = rc_pick_head(rc, c, member_mask, now);
		if (h < 0) {
			continue;
		}
		t->role = RC_ROLE_CHILD;
		t->relay_needed = true;
		t->head = (uint8_t)h;
		t->assigned_ms = now;
		t->delivered = false;
		t->eval_relayed = 0;
		t->eval_gaps = 0;
		t->good_windows = 0;
		rc->t[h].role = RC_ROLE_HEAD;
		rc->t[h].children |= RC_BIT(c);
		rc->assignments++;
	}

	/* 4. Delivery feedback for children (overheard in every extended PONG). */
	rc->relayed_mask = 0;
	for (uint8_t c = 0; c < RC_MAX; c++) {
		if (rc->t[c].role == RC_ROLE_CHILD && rc->w[c].relayed) {
			rc->relayed_mask |= RC_BIT(c);
		}
	}

	/* 5. Forwarding slots: wanted when one bundle per frame cannot carry a
	 * head's children; withdrawn in two phases. */
	for (uint8_t h = 0; h < RC_MAX; h++) {
		struct rc_tracker *t = &rc->t[h];
		if (rc->w[h].overflow) {
			if (t->overflow_windows < UINT8_MAX) {
				t->overflow_windows++;
			}
		} else {
			t->overflow_windows = 0;
		}
		bool needed = t->role == RC_ROLE_HEAD
			&& (rc_count(t->children) >= 2 || t->fwd_requested
			    || t->overflow_windows >= RC_FWD_OVERFLOW_WINDOWS);
		if (needed) {
			t->fwd_idle_windows = 0;
			if (!t->fwd_wanted && !t->fwd_revoking) {
				t->fwd_wanted = true;
			}
		} else if (t->fwd_wanted) {
			bool idle = t->role != RC_ROLE_HEAD || ++t->fwd_idle_windows >= RC_FWD_IDLE_WINDOWS;
			if (idle) {
				t->fwd_wanted = false;
				t->fwd_revoking = (rc->forward_mask & RC_BIT(h)) != 0;
				t->fwd_revoke_ms = now;
			}
		}
		if (t->fwd_revoking && rc_after(now, t->fwd_revoke_ms, RC_FWD_REVOKE_WAIT_MS)) {
			t->fwd_revoking = false;
		}
	}

	memset(rc->w, 0, sizeof(rc->w));
	return relay_coord_forward_wanted(rc, member_mask);
}

void relay_coord_set_forward_layout(struct relay_coord *rc, uint16_t forward_mask)
{
	rc->forward_mask = forward_mask;
	for (uint8_t h = 0; h < RC_MAX; h++) {
		if (!(forward_mask & RC_BIT(h))) {
			/* Not in the layout: nothing to withdraw any more. */
			rc->t[h].fwd_revoking = false;
		}
	}
}

uint8_t relay_coord_forward_slot(const struct relay_coord *rc, uint8_t head, uint16_t member_mask)
{
	if (head >= RC_MAX || !(rc->forward_mask & RC_BIT(head))) {
		return RELAY_NO_SLOT;
	}
	const struct rc_tracker *t = &rc->t[head];
	if (!rc->enabled || t->role != RC_ROLE_HEAD || !t->fwd_wanted || t->fwd_revoking) {
		return RELAY_NO_SLOT;
	}
	uint8_t index = rc_count(member_mask);
	for (uint8_t i = 0; i < head; i++) {
		index += (rc->forward_mask >> i) & 1U;
	}
	return index < RC_MAX_TOTAL_SLOTS ? index : RELAY_NO_SLOT;
}

void relay_coord_trailer(const struct relay_coord *rc, uint8_t id, uint8_t epoch,
	uint16_t member_mask, struct relay_pong_trailer *out)
{
	const struct rc_tracker *t = id < RC_MAX ? &rc->t[id] : NULL;
	out->epoch = epoch;
	out->children_mask = t && rc->enabled && t->role == RC_ROLE_HEAD ? t->children : 0;
	out->forward_slot = t ? relay_coord_forward_slot(rc, id, member_mask) : RELAY_NO_SLOT;
	out->member_mask = member_mask;
	out->relayed_mask = rc->relayed_mask;
}

bool relay_coord_is_relayed(const struct relay_coord *rc, uint8_t id)
{
	return id < RC_MAX && rc->t[id].role == RC_ROLE_CHILD;
}

bool relay_coord_direct_ready(const struct relay_coord *rc, uint8_t id, uint32_t now_ms)
{
	/* Use the coordinator's own weak verdict (RC_WEAK_WINDOWS lossy windows
	 * with enough expected frames), not one window's share: at idle rates a
	 * 500 ms window holds a handful of frames and one loss refuses OTA. */
	return id < RC_MAX && rc->t[id].direct_seen
		&& !rc_after(now_ms, rc->t[id].last_direct_ms, RC_OTA_DIRECT_FRESH_MS)
		&& rc->t[id].weak_windows < RC_WEAK_WINDOWS && !rc->t[id].relay_needed;
}

bool relay_coord_is_capable(const struct relay_coord *rc, uint8_t id)
{
	return id < RC_MAX && rc->t[id].capable;
}

void relay_coord_set_enabled(struct relay_coord *rc, bool enabled)
{
	rc->enabled = enabled;
	if (!enabled) {
		for (uint8_t i = 0; i < RC_MAX; i++) {
			rc_release_head(rc, i);
		}
		rc->relayed_mask = 0;
	}
}

void relay_coord_pin(struct relay_coord *rc, uint8_t id, bool pinned)
{
	if (id < RC_MAX) {
		if (pinned) {
			rc->pinned |= RC_BIT(id);
		} else {
			rc->pinned &= (uint16_t)~RC_BIT(id);
		}
	}
}

bool relay_coord_v3_ready(const struct relay_coord *rc, uint16_t members)
{
	if (!rc->enabled || !rc->v3_requested || !members || (members & ~RC_MEMBER_MASK)) {
		return false;
	}
	for (uint8_t id = 0; id < RC_MAX; id++) {
		if ((members & RC_BIT(id))
		    && (!rc->t[id].capable || rc->t[id].protocol_version != RELAY_V3_VERSION)) {
			return false;
		}
	}
	return true;
}

/* Frame order, rather than the wrapping native sequence, governs HID in v3. */
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
	if (RC_ID_OUTSIDE(id)) {
		return;
	}
#ifdef __ZEPHYR__
	unsigned key = irq_lock();
#endif
	struct rc_receipt *r = &rc->receipt[frame % (rc->cache_frames ? rc->cache_frames : 3U)];
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
	if (RC_ID_OUTSIDE(id)) {
		return false;
	}
	uint32_t age = rc->published[id] - frame;
	return (rc->published_valid & (1U << id)) && age < 32 &&
		(rc->published_history[id] & (1U << age));
}

/* Publication is in frame order, so a skipped frame can never be published
 * later: it was lost on both paths. Jumps over 80 frames are a restart, as in
 * check_packet_sequence(), and add no gaps. */
uint8_t relay_coord_skipped(const struct relay_coord *rc, uint8_t id, uint32_t frame)
{
	if (id >= RC_MAX || !(rc->published_valid & (1U << id))) {
		return 0;
	}
	int32_t advance = (int32_t)(frame - rc->published[id]);
	return advance > 1 && advance <= 80 ? (uint8_t)(advance - 1) : 0;
}

bool relay_coord_publish(struct relay_coord *rc, uint8_t id, uint32_t frame)
{
	if (RC_ID_OUTSIDE(id)) {
		return false;
	}
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
void relay_coord_expire_peer(struct relay_coord *rc, unsigned helper,
	unsigned child, uint32_t now)
{
	if (RC_ID_OUTSIDE(helper) || RC_ID_OUTSIDE(child)) {
		return;
	}
	rc_peer_expire(&rc->peer[helper][child], now);
}

void relay_coord_capture(struct relay_coord *rc, uint8_t h, uint8_t child, uint8_t rssi, uint32_t now)
{
	if (RC_ID_OUTSIDE(h) || RC_ID_OUTSIDE(child)) {
		return;
	}
	struct rc_peer *p = &rc->peer[h][child];
	rc_peer_expire(p, now);
	if (rssi) {
		p->rssi_x8 = relay_rssi_avg_x8(p->rssi_x8, rssi);
	}
	p->evidence_ms = now;
}

bool relay_coord_report(struct relay_coord *rc, const struct relay_control *c, unsigned slot,
	const struct relay_peer_report *r, uint32_t now)
{
	if (RC_ID_OUTSIDE(r->helper) || r->session != c->session || r->epoch != c->epoch || slot >= c->repair_count ||
	    c->grants[slot].owner != r->helper || c->grants[slot].children ||
	    !(c->members & RC_BIT(r->helper))) {
		return false;
	}
	for (unsigned i = 0; i < r->count; i++) {
		const struct relay_peer_observation *e = &r->entries[i];
		if (RC_ID_OUTSIDE(e->child)) continue;
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

static bool rc_listener(const struct rc_learn_snapshot *rc, uint8_t child, uint8_t h)
{
	if (RC_ID_OUTSIDE(h) || RC_ID_OUTSIDE(child)) return false;
	const struct rc_peer *p = &rc->peer[h][child];
	/* Unknown peers may collect provisional evidence. A measured silent
	 * link must yield its repair slots; surveys can establish recovery. */
	return relay_can_overhear(rc->learn_members, h, child) && rc->t[h].direct_permille >= RC_HEAD_MIN_PERMILLE &&
		!(p->expected >= 30U && p->heard == 0);
}

static bool rc_better(const struct rc_learn_snapshot *rc, uint8_t child, uint8_t a, uint8_t b,
	const uint8_t *load)
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

static void rc_learn_preferences(struct rc_learn_snapshot *rc, uint32_t now)
{
	for (uint8_t c = 0; c < RC_MAX; c++) {
		uint8_t load[RC_MAX] = {0};
		for (unsigned k = 0; k < 2; k++) {
			struct rc_choice *q = &rc->choice[c][k];
			q->valid = q->valid && rc_listener(rc, c, q->helper);
			if (q->valid) load[q->helper]++;
		}
		for (unsigned k = 0; k < 2; k++) {
			struct rc_choice *q = &rc->choice[c][k], *other = &rc->choice[c][1U-k];
			if (q->valid) {
				load[q->helper]--;
			}
			int best = -1;
			for (uint8_t h = 0; h < RC_MAX; h++) {
				if (!rc_listener(rc, c, h) ||
				    (k && other->valid && other->helper == h)) {
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
				bool beats = margin >= 50 || (margin >= -50 && margin <= 50 &&
					p->rssi_x8 && old->rssi_x8 >= p->rssi_x8 + 48U);
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
					if (other->valid) load[other->helper]++;
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
static bool rc_cover(struct rc_learn_snapshot *rc, struct rc_choice preferred[RC_MAX][2],
	uint8_t *load, uint8_t child, unsigned kind, uint32_t now)
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
				if ((tried & RC_BIT(h)) || (visited & RC_BIT(h)) ||
				    !rc_listener(rc, c, h) ||
				    (rc->choice[c][1U-k].valid && rc->choice[c][1U-k].helper == h)) continue;
				if (best < 0 || rc_better(rc, c, h, best, load)) best = h;
			}
			const struct rc_choice *want = &preferred[c][k];
			if (!n && want->valid && !(visited & RC_BIT(want->helper)) &&
			    rc_listener(rc, c, want->helper) &&
			    !(rc->choice[c][1U-k].valid && rc->choice[c][1U-k].helper == want->helper)) best = want->helper;
			if (best < 0) break;
			tried |= RC_BIT(best);
			visited |= RC_BIT(best);
			if (load[best] < RELAY_MAX_CHILDREN) {
				/* Rotate assignments back to the newly covered root. */
				for (unsigned move = node; move != 0xff; move = parent[move]) {
					struct rc_choice *q = &rc->choice[move/2U][move%2U];
					const struct rc_choice *pref = &preferred[move/2U][move%2U];
					unsigned old = q->helper;
					if (q->valid) load[old]--;
					*q = pref->valid && pref->helper == best ? *pref :
						(struct rc_choice){.valid=true, .helper=best, .since_ms=now};
					load[best]++;
					best = old;
				}
				return true;
			}
			for (unsigned next = 0; next < 2U * RC_MAX; next++) {
				const struct rc_choice *q = &rc->choice[next/2U][next%2U];
				if (q->valid && q->helper == best) {
					parent[next] = node;
					queue[count++] = next;
				}
			}
		}
	}
	return false;
}

void relay_learn_snapshot_tick(struct rc_learn_snapshot *rc, uint32_t now)
{
	uint8_t load[RC_MAX] = {0};
	for (unsigned h = 0; h < RC_MAX; h++)
		for (unsigned c = 0; c < RC_MAX; c++) rc_peer_expire(&rc->peer[h][c], now);
	rc_learn_preferences(rc, now);
	struct rc_choice preferred[RC_MAX][2];
	memcpy(preferred, rc->choice, sizeof(preferred));
	memset(rc->choice, 0, sizeof(rc->choice));
	/* Weak primaries, weak backups, healthy primaries, healthy backups.
	 * Augmentation retains all coverage already allocated at higher priority. */
	for (unsigned phase = 0; phase < 4; phase++) {
		for (uint8_t c = 0; c < RC_MAX; c++) {
			bool weak = rc->t[c].direct_permille < RC_HEAD_MIN_PERMILLE;
			if (weak != (phase < 2)) continue;
			(void)rc_cover(rc, preferred, load, c, phase % 2, now);
		}
	}
}

void relay_plan_snapshot_watch(struct rc_plan_snapshot *rc, struct relay_control *c)
{
	rc->learn.learn_members = c->members;
	c->primary_valid = c->backup_valid = 0;
	memset(c->helpers, 0, sizeof(c->helpers));
	for (uint8_t child = 0; child < RC_MAX; child++) {
		for (unsigned k = 0; k < 2; k++) {
			struct rc_choice *q = &rc->learn.choice[child][k];
			if (!q->valid || !rc_listener(&rc->learn, child, q->helper)) {
				continue;
			}
			c->helpers[child] |= q->helper << (k * 4U);
			if (k) c->backup_valid |= RC_BIT(child);
			else c->primary_valid |= RC_BIT(child);
		}
	}
}

static uint16_t rc_watch_helpers(const struct rc_watch *w, unsigned child)
{
	if (w->survey & RC_BIT(child)) return w->members;
	uint16_t helpers = 0;
	if (w->primary_valid & RC_BIT(child)) helpers |= RC_BIT(w->helpers[child] & 15U);
	if (w->backup_valid & RC_BIT(child)) helpers |= RC_BIT(w->helpers[child] >> 4);
	return helpers;
}

static bool rc_narrow_pool(const struct relay_control *c)
{
	return c->repair_count <= 3U;
}

static bool rc_rescue_better(
	const struct rc_plan_snapshot *rc,
	bool narrow,
	uint16_t tried,
	unsigned child,
	unsigned h,
	int incumbent,
	const uint8_t *load
)
{
	if (incumbent < 0) return true;
	bool old = (tried & RC_BIT(incumbent)) != 0;
	bool retry = (tried & RC_BIT(h)) != 0;
	if (old != retry) {
		return old;
	}
	if (narrow) {
		/* Rescue capacity needs the best measured copy probability. Do not trade
		 * several percent of capture for a small RSSI advantage at last chance. */
		unsigned candidate = relay_coord_peer_ratio(&rc->learn.peer[h][child]);
		unsigned previous = relay_coord_peer_ratio(&rc->learn.peer[incumbent][child]);
		if (candidate != previous) {
			return candidate > previous;
		}
	}
	return rc_better(&rc->learn, child, h, incumbent, load);
}

/* Capture history is permission to listen, not proof of capture. Try up to two eligible
 * alternates for a missing record, oldest first, never inferred from RSSI alone.
 * The frozen step rechecks that the selected record is still missing. */
static void rc_plan_rescue(const struct rc_plan_snapshot *rc, const struct relay_control *c,
	struct rc_grant_plan *p)
{
	for (unsigned remaining = relay_control_cache_frames(c); remaining > 0; remaining--) {
		unsigned age = remaining - 1U;
		uint32_t frame = c->frame - age;
		const struct rc_watch *w = &rc->watch[frame % (rc->cache_frames ? rc->cache_frames : 3U)];
		const struct rc_receipt *r = &rc->receipt[frame % (rc->cache_frames ? rc->cache_frames : 3U)];
		if (w->frame != frame || w->session != c->session || w->epoch != c->epoch ||
		    w->members != c->members) continue;
		uint16_t missing = c->expected & RC_MEMBER_MASK & ~(r->frame == frame ? r->received : 0);
		unsigned first = 0;
		uint16_t tail = missing;
		if (rc_narrow_pool(c)) {
			/* A narrow pool cannot leave the same later-ID child behind on every
			 * tied deadline. Rotate the first rescue among the missing children. */
			uint16_t pivot = missing;
			unsigned rank = missing ? c->frame % relay_mask_count(missing) : 0;
			while (rank--) {
				pivot &= pivot - 1U;
			}
			first = pivot ? __builtin_ctz((unsigned)pivot) : 0;
			tail = missing & (uint16_t)(0xffffU << first);
		}
		while (missing) {
			unsigned child;
			if (rc_narrow_pool(c)) {
				child = __builtin_ctz((unsigned)(tail ? tail : missing));
				missing &= ~RC_BIT(child);
				tail &= ~RC_BIT(child);
			} else {
				child = __builtin_ctz((unsigned)missing);
				missing &= missing - 1U;
			}
			uint16_t listeners = w->listeners[child] | rc_watch_helpers(w, child);
			int best = -1, second = -1;
			uint16_t tried = w->rescue_tried[child];
			const uint8_t load[RC_MAX] = {0};
			for (unsigned h = 0; h < RC_MAX; h++) {
				if (!(listeners & RC_BIT(h)) ||
				    ((w->primary_valid & RC_BIT(child)) && (w->helpers[child] & 15U) == h) ||
				    ((w->backup_valid & RC_BIT(child)) && (w->helpers[child] >> 4) == h) ||
				    !relay_can_overhear(w->members, h, child) || !rc_listener(&rc->learn, child, h)) continue;
				if (rc_rescue_better(rc, rc_narrow_pool(c), tried, child, h, best, load)) {
					second = best;
					best = h;
				} else if (rc_rescue_better(rc, rc_narrow_pool(c), tried, child, h, second, load)) {
					second = h;
				}
			}
			if (best >= 0) {
				p->rescue_child = RC_BIT(child);
				p->rescue_helper = best;
				p->rescue_second = second < 0 ? RELAY_OWNER_IDLE : second;
				p->rescue_age = age;
				return;
			}
		}
	}
}

/* Reserve a late opportunity for the oldest debt. A holder that previously
 * raced R0 gets a retry after preparation time; otherwise try the qualified
 * alternate already selected by rescue planning. Coverage is permission from
 * the capture-frame watch, never an assumption that the cache holds a copy. */
static void rc_plan_emergency(const struct rc_plan_snapshot *rc, const struct relay_control *c, struct rc_grant_plan *p)
{
	const unsigned age = relay_control_cache_frames(c) - 1U;
	const struct rc_watch *w = &rc->watch[(c->frame - age) % (rc->cache_frames ? rc->cache_frames : 3U)];
	if (!p->rescue_child || p->rescue_age != age) {
		return;
	}
	unsigned child = __builtin_ctz((unsigned)p->rescue_child);
	unsigned helper = p->rescue_helper;
	int retry = -1;
	const uint8_t load[RC_MAX] = {0};
	for (unsigned h = 0; h < RC_MAX; h++) {
		if (!(w->r0_only[child] & RC_BIT(h)) || !relay_can_overhear(w->members, h, child)
			|| !rc_listener(&rc->learn, child, h)) {
			continue;
		}
		if (retry < 0 || rc_better(&rc->learn, child, h, retry, load)) {
			retry = h;
		}
	}
	if (retry >= 0) {
		helper = retry;
	}
	p->emergency_child = p->rescue_child;
	p->emergency_helper = helper;
	for (unsigned ch = 0; ch < RC_MAX; ch++) {
		if (!(c->expected & RC_BIT(ch)) || !relay_can_overhear(w->members, helper, ch)
			|| !rc_listener(&rc->learn, ch, helper)) {
			continue;
		}
		if ((w->listeners[ch] | rc_watch_helpers(w, ch)) & RC_BIT(helper)) {
			p->emergency_coverage |= RC_BIT(ch);
		}
	}
	p->emergency_coverage |= p->emergency_child;
}

/* Diagnostic only, before freeze. Flags: listener, primary, backup,
 * topology, uplink, silent, tried, eligible, selected, other choice. Rank
 * includes ineligible peers so the historical ordering can be reconstructed. */
#ifdef CONFIG_RELAY_RESCUE_DIAGNOSTICS
static void
rc_rescue_diagnose(const struct rc_plan_snapshot *rc, const struct relay_control *c, const struct rc_grant_plan *p)
{
	const uint8_t load[RC_MAX] = {0};
	for (unsigned age = 0; age < relay_control_cache_frames(c); age++) {
		uint32_t frame = c->frame - age;
		const struct rc_watch *w = &rc->watch[frame % (rc->cache_frames ? rc->cache_frames : 3U)];
		const struct rc_receipt *r = &rc->receipt[frame % (rc->cache_frames ? rc->cache_frames : 3U)];
		if (w->frame != frame || w->session != c->session || w->epoch != c->epoch || w->members != c->members) {
			continue;
		}
		uint16_t missing = c->expected & RC_MEMBER_MASK & ~(r->frame == frame ? r->received : 0);
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
					= relay_can_overhear(w->members, h, child) && relay_can_overhear(rc->learn.learn_members, h, child);
				bool uplink = rc->learn.t[h].direct_permille >= RC_HEAD_MIN_PERMILLE;
				bool silent = rc->learn.peer[h][child].expected >= 30U && rc->learn.peer[h][child].heard == 0;
				bool eligible = listener && !primary && !backup && topology && uplink && !silent;
				bool choice = p->rescue_child == RC_BIT(child) && p->rescue_age == age;
				unsigned flags = listener | (primary << 1) | (backup << 2) | (topology << 3) | (uplink << 4)
							   | (silent << 5) | (!!(tried & RC_BIT(h)) << 6) | (eligible << 7)
							   | (choice && (p->rescue_helper == h || p->rescue_second == h) ? 256U : 0U)
							   | (p->rescue_child && !choice ? 512U : 0U);
				unsigned rank = 0;
				for (unsigned j = 0; j < RC_MAX; j++) {
					if (j != h && rc_rescue_better(rc, rc_narrow_pool(c), tried, child, j, h, load)) {
						rank++;
					}
				}
				ledger_note(c->session, frame, child, LEDGER_CANDIDATE_FLAGS, flags, h, age);
				ledger_note(c->session, frame, child, LEDGER_CANDIDATE_UPLINK, rc->learn.t[h].direct_permille, h, age);
				ledger_note(c->session, frame, child, LEDGER_CANDIDATE_RANK, rank, h, age);
			}
		}
	}
}
#endif

static void rc_finish_demand(struct relay_control *c, const struct rc_grant_plan *p, uint8_t *wire);
/* Selection scans run before the mask freeze. Ties retain rotation order. */
void relay_plan_snapshot_grants(struct rc_plan_snapshot *rc, struct relay_control *c,
	struct rc_grant_plan *p, uint32_t now_ms)
{
	for (unsigned i = 0; i < RELAY_V3_REPAIR_MAX; i++) {
		c->grants[i] = (struct relay_grant){.owner = RELAY_OWNER_IDLE};
	}
	c->maintenance_owner = RELAY_OWNER_IDLE;
	c->flags &= ~RELAY_CONTROL_MAINTENANCE;
	*p = (struct rc_grant_plan){.now_ms = now_ms, .maintenance = -1, .reporter = -1};
	uint32_t oldest_m = 0, oldest_r = 0;
	for (unsigned n = 0; n < RC_MAX; n++) {
		unsigned id = (rc->maintenance_next + n) % RC_MAX;
		if (rc->learn.t[id].direct_permille < RC_HEAD_MIN_PERMILLE) p->fresh_survey |= RC_BIT(id);
		uint32_t age = now_ms - rc->maintenance_ms[id];
		if ((c->members & RC_BIT(id)) && age >= RC_MAINTENANCE_MS &&
		    (p->maintenance < 0 || age > oldest_m)) {
			p->maintenance = id;
			oldest_m = age;
		}
		id = (rc->report_next + n) % RC_MAX;
		age = now_ms - rc->report_ms[id];
		if ((c->members & RC_BIT(id)) && age >= 1000U &&
		    (p->reporter < 0 || age > oldest_r)) {
			p->reporter = id;
			oldest_r = age;
		}
	}
	/* A tracker that misses B can still listen under either of the preceding
	 * two B plans. Match its unchanged three-frame control lifetime exactly.
	 * Keep raw plans separate so inherited listeners cannot extend themselves. */
	for (unsigned child = 0; child < RC_MAX; child++) {
		if (c->primary_valid & RC_BIT(child))
			p->next_listeners[child] |= RC_BIT(c->helpers[child] & 15U);
		if (c->backup_valid & RC_BIT(child))
			p->next_listeners[child] |= RC_BIT(c->helpers[child] >> 4);
	}
	for (unsigned age = 0; age < 2U; age++) {
		uint32_t frame = c->frame - age;
		const struct rc_watch *w = &rc->watch[frame % (rc->cache_frames ? rc->cache_frames : 3U)];
		if (w->frame != frame || w->session != c->session || w->epoch != c->epoch ||
		    w->members != c->members) continue;
		for (unsigned child = 0; child < RC_MAX; child++)
			p->next_listeners[child] |= rc_watch_helpers(w, child);
	}
	/* A repaired weak uplink still needs alternate copies on the next frame.
	 * Keep the existing two-survey cap; precompute the fresh mask before freeze. */
	p->fresh_survey |= ~(c->primary_valid | c->backup_valid);
	p->fresh_survey &= c->members;
	rc_plan_rescue(rc, c, p);
	if (rc_narrow_pool(c)) {
		rc_plan_emergency(rc, c, p);
	}
#ifdef CONFIG_RELAY_RESCUE_DIAGNOSTICS
	rc_rescue_diagnose(rc, c, p);
#endif
	if (c->repair_count < 2U) {
		p->maintenance = -1;
	}
	p->urgent = p->maintenance >= 0
			 && (oldest_m >= RC_MAINTENANCE_URGENT_MS || (c->on_demand && c->repair_count >= 2 && c->frame % 8U == 0));
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
			unsigned direct = rc->learn.t[child].direct_permille;
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
	for (unsigned age = 0; age < relay_control_cache_frames(c); age++) {
		uint32_t frame = c->frame - age;
		const struct rc_watch *w = &rc->watch[frame % (rc->cache_frames ? rc->cache_frames : 3U)];
		p->receipt_index[age] = frame % (rc->cache_frames ? rc->cache_frames : 3U);
		if (w->frame == frame) {
			*relay_plan_watch(p, age) = *w;
			if (c->on_demand) {
				for (unsigned ch = 0; ch < RC_MAX; ch++) {
					uint16_t holders = w->listeners[ch] | rc_watch_helpers(w, ch);
					for (unsigned h = 0; h < RC_MAX; h++) {
						if ((holders & RC_BIT(h)) && rc_listener(&rc->learn, ch, h)) {
							p->demand_eligible[age] |= RC_BIT(ch);
							if (!age) {
								p->fresh_helper[ch] = h;
								p->fresh_candidates |= RC_BIT(ch);
								p->fresh_coverage[h] |= RC_BIT(ch);
							}
							break;
						}
					}
				}
			}

			p->valid |= 1U << age;
			for (unsigned copy = 0; copy < 2; copy++) {
				uint16_t children = (copy ? w->backup_valid : w->primary_valid) & RC_MEMBER_MASK;
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
					if (RC_ID_OUTSIDE(h)) continue;
					if (c->on_demand) {
						unsigned lane = 4U - age, bit = copy * 16U + child;
						if (!age && rc_listener(&rc->learn, child, h)) {
							/* Replace the provisional listener with the actual
							 * static primary lane, then retain its backup too. */
							if (!copy && (p->fresh_candidates & RC_BIT(child))) {
								p->fresh_coverage[p->fresh_helper[child]] &= ~RC_BIT(child);
							}
							p->fresh_helper[bit] = h;
							p->fresh_candidates |= 1U << bit;
							p->fresh_coverage[h] |= 1U << bit;
						}
						p->demand_helper[lane][bit] = h;
						p->demand_children[lane][h] |= 1U << bit;
						p->demand_candidates[lane] |= 1U << bit;
						continue;
					}
					unsigned position;
					if (rc_narrow_pool(c)) {
						/* A narrow pool must try the backup of an expiring record
						 * before spending its remaining capacity on younger work. */
						position = ((2U - age) * 2U + copy) * RELAY_MAX_TRACKERS + child;
					} else {
						position = (copy * 3U + 2U - age) * 16U + child;
					}
					p->helper[position / 32U][position % 32U] = h;
					p->children[position / 32U][h] |= 1U << (position % 32U);
					p->candidates[position / 32U] |= 1U << (position % 32U);
				}
			}
		}
	}
	if (c->on_demand) {
		for (unsigned age = 0; age < 5; age++) {
			const struct rc_receipt *r = &rc->receipt[p->receipt_index[age]];
			relay_control_missing(c, age) = c->expected & ~(r->frame == c->frame - age ? r->received : 0);
		}
		/* A two-slot maintenance exchange must not consume the last repair
		 * capacity for retained qualified poses. It resumes once both fit. */
		uint16_t aged = 0;
		for (unsigned age = 1; age < 5; age++) {
			aged |= relay_control_missing(c, age) & p->demand_eligible[age];
		}
		if (aged && c->repair_count < relay_mask_count(aged) + 2U) {
			p->maintenance = -1;
			p->urgent = false;
		}

		uint8_t wire[72];
		relay_control_encode(wire, c);
		rc_finish_demand(c, p, wire);
		relay_coord_read_grants(c, wire);
		for (unsigned i = 0; i < c->repair_count; i++) {
			unsigned h = c->grants[i].owner;
			uint32_t children = c->grants[i].children;
			p->planned_fresh |= h < RC_MAX ? p->fresh_coverage[h] & (children | (children << 16)) : 0;
		}
	}
}

static void rc_apply_maintenance(struct relay_control *c,
	const struct rc_grant_plan *p)
{
	c->maintenance_owner = p->maintenance;
	c->flags |= RELAY_CONTROL_MAINTENANCE;
	c->grants[0] = (struct relay_grant){.owner = RELAY_OWNER_MAINTENANCE};
	c->grants[1] = c->grants[0];
}

/* Attribute provenance: Frozen allocator is included in linked 10,000-payload equivalence and round 11 encode-cycle evidence. Retain its call/placement boundary. */
static inline __attribute__((always_inline)) uint16_t rc_take_records(uint32_t *pending,
	uint32_t candidates, unsigned *capacity)
{
	uint32_t records = *pending & candidates;
	uint32_t rest = records;
	/* All callers start with at most three records. Spell out that bound
	 * so the frozen encoder pays no loop-back branch for each record. */
	if (rest && *capacity) {
		rest &= rest - 1U;
		(*capacity)--;
	}
	if (rest && *capacity) {
		rest &= rest - 1U;
		(*capacity)--;
	}
	if (rest && *capacity) {
		rest &= rest - 1U;
		(*capacity)--;
	}
	uint32_t selected = records ^ rest;
	*pending &= ~selected;
	return selected | (selected >> 16);
}

#if defined(__GNUC__) && !defined(__clang__)
/* Attribute provenance: Frozen allocator is included in linked 10,000-payload equivalence and round 11 encode-cycle evidence. Retain its call/placement boundary. */
__attribute__((optimize("O3")))
#endif
/* Keep the full-pool load placement at its measured baseline. */
#define RC_GRANT_LIMIT limit
static RELAY_RAMFUNC void rc_finish_full(struct relay_control *c, const struct rc_grant_plan *p, uint8_t *wire)
{

	unsigned idle = 0;
	if (p->urgent) {
		rc_apply_maintenance(c, p);
		wire[RELAY_CONTROL_OFF_OWNERS] = wire[RELAY_CONTROL_OFF_OWNER_SECOND] = RELAY_OWNER_MAINTENANCE;
		idle = 2;
	}

	/* Full pool: primary oldest first, then backup oldest first.
	 * Narrow pool: both lanes of an older deadline precede younger work.
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
	for (unsigned age = 1; age < RELAY_V3_CACHE_FRAMES; age++) {
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

#if defined(__GNUC__) && !defined(__clang__)
/* Match the measured frozen encoder's optimization on both runtime paths. */
__attribute__((optimize("O3")))
#endif
static RELAY_RAMFUNC void rc_finish_narrow(struct relay_control *c, const struct rc_grant_plan *p, uint8_t *wire)
{
	unsigned limit = c->repair_count;
	bool emergency = false;
	uint32_t a, b, d;

	emergency = c->repair_count > 0 && (c->missing[2] & p->emergency_child) != 0;
	if (emergency) {
		limit--;
		wire[RELAY_CONTROL_OFF_OWNERS + limit] = p->emergency_helper;
		relay_put_be16(
			wire + RELAY_CONTROL_OFF_CHILDREN + 2U * limit,
			p->emergency_coverage & (c->missing[0] | c->missing[1] | c->missing[2])
		);
	}

	unsigned idle = 0;
	if (p->urgent && c->repair_count >= 2U) {
		rc_apply_maintenance(c, p);

		wire[RELAY_CONTROL_OFF_OWNERS] = wire[RELAY_CONTROL_OFF_OWNER_SECOND] = RELAY_OWNER_MAINTENANCE;
		/* A two-slot maintenance reservation can replace the emergency
		 * repair in R1. Its old child mask must not remain on the wire. */
		if (c->repair_count == 2U) {
			relay_put_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2U, 0);
		}

		idle = 2;
	}

	/* Full pool: primary oldest first, then backup oldest first.
	 * Narrow pool: both lanes of an older deadline precede younger work.
	 * Fill one helper's next three records ahead of the scan. Removing those
	 * records leaves exactly the next slot-opening record of the single-pass
	 * algorithm, including a fourth record for the same helper. */

	a = (c->missing[2] | ((uint32_t)c->missing[2] << 16)) & p->candidates[0];
	b = (c->missing[1] | ((uint32_t)c->missing[1] << 16)) & p->candidates[1];
	d = (c->missing[0] | ((uint32_t)c->missing[0] << 16)) & p->candidates[2];

	while (idle < RC_GRANT_LIMIT && (a | b | d)) {
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
	if (p->maintenance >= 0 && idle == 0 && c->repair_count >= 2U) {
		rc_apply_maintenance(c, p);

		wire[RELAY_CONTROL_OFF_OWNERS] = wire[RELAY_CONTROL_OFF_OWNER_SECOND] = RELAY_OWNER_MAINTENANCE;
		/* A two-slot maintenance reservation can replace the emergency
		 * repair in R1. Its old child mask must not remain on the wire. */
		if (c->repair_count == 2U) {
			relay_put_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2U, 0);
		}

		idle = 2;
	}
	c->survey = p->survey;
	uint16_t failed = c->missing[0] & p->fresh_survey;
	for (unsigned age = 1; age < RELAY_V3_CACHE_FRAMES; age++) {
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
	if (p->reporter >= 0 && idle < RC_GRANT_LIMIT) {
		wire[RELAY_CONTROL_OFF_OWNERS + idle++] = p->reporter;
	}
	/* Prefer a surveyed alternate over a same-helper retry in otherwise idle
	 * capacity. Helpers retain refused records but suppress successful ones.
	 * All distinct repairs, maintenance and reports keep their priority. */
	unsigned retry = p->urgent ? 2U : 0U;
	if (idle < RC_GRANT_LIMIT && (c->missing[p->rescue_age] & p->rescue_child)) {
		wire[RELAY_CONTROL_OFF_OWNERS + idle] = p->rescue_helper;
		relay_put_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2U * idle, p->rescue_child);
		/* A second distinct survey copy uses only another otherwise idle slot. */
		if (++idle < RC_GRANT_LIMIT && p->rescue_second < RC_MAX) {
			wire[RELAY_CONTROL_OFF_OWNERS + idle] = p->rescue_second;
			relay_put_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2U * idle, p->rescue_child);
		}
	} else if (idle < RC_GRANT_LIMIT && retry < repair_end) {
		wire[RELAY_CONTROL_OFF_OWNERS + idle] = wire[RELAY_CONTROL_OFF_OWNERS + retry];
		wire[RELAY_CONTROL_OFF_CHILDREN + 2U * idle] = wire[RELAY_CONTROL_OFF_CHILDREN + 2U * retry];
		wire[RELAY_CONTROL_OFF_CHILDREN_LOW + 2U * idle] = wire[RELAY_CONTROL_OFF_CHILDREN_LOW + 2U * retry];
	}

	/* A retained copy must not repeatedly race R0. Keep all grant masks and
	 * maintenance reservations; only rotate physical repair opportunities. */
	if (!emergency && c->repair_count == 3 && !(c->flags & RELAY_CONTROL_MAINTENANCE)) {
		uint8_t owners[3];
		uint16_t children[3];
		for (unsigned i = 0; i < 3; i++) {
			owners[i] = wire[RELAY_CONTROL_OFF_OWNERS + i];
			children[i] = relay_get_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2U * i);
		}
		unsigned first = c->frame % 3U;
		for (unsigned i = 0; i < 3; i++) {
			unsigned from = (i + first) % 3U;
			wire[RELAY_CONTROL_OFF_OWNERS + i] = owners[from];
			relay_put_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2U * i, children[from]);
		}
	}
}

/* Five-age path. Planning stores oldest-first lanes; freeze only removes receipts. */
static void rc_finish_demand(struct relay_control *c, const struct rc_grant_plan *p, uint8_t *wire)
{
	uint32_t pending[5];
	for (unsigned i = 0; i < 5; i++) {
		pending[i] = (relay_control_missing(c, 4 - i) | ((uint32_t)relay_control_missing(c, 4 - i) << 16))
				   & p->demand_candidates[i];
	}
	unsigned slot = 0;
	if (p->urgent && c->repair_count >= 2) {
		rc_apply_maintenance(c, p);
		wire[46] = wire[47] = RELAY_OWNER_MAINTENANCE;
		slot = 2;
	}
	unsigned passes = rc_narrow_pool(c) ? 1U : 2U;
	for (unsigned pass = 0; pass < passes && slot < c->repair_count; pass++) {
		uint32_t lane_mask = passes == 1 ? UINT32_MAX : pass ? 0xffff0000U : 0xffffU;
		for (unsigned lane = 0; lane < 5 && slot < c->repair_count; lane++) {
			while ((pending[lane] & lane_mask) && slot < c->repair_count) {
				unsigned h = p->demand_helper[lane][__builtin_ctz(pending[lane] & lane_mask)];
				unsigned capacity = RELAY_MAX_CHILDREN;
				uint16_t children = 0;
				for (unsigned group = 0; group < passes && capacity; group++) {
					uint32_t mask = passes == 1 ? UINT32_MAX : group ? 0xffff0000U : 0xffffU;
					for (unsigned k = 0; k < 5 && capacity; k++) {
						children |= rc_take_records(&pending[k], p->demand_children[k][h] & mask, &capacity);
					}
				}
				wire[46 + slot] = h;
				relay_put_be16(wire + 51 + 2 * slot, children);
				slot++;
			}
		}
	}
	if (p->maintenance >= 0 && slot == 0 && c->repair_count >= 2) {
		rc_apply_maintenance(c, p);
		wire[46] = wire[47] = RELAY_OWNER_MAINTENANCE;
		slot = 2;
	}
	/* Apply spare-slot policy once, after frozen receipts are known. */
	c->survey = p->survey;
}

RELAY_RAMFUNC void relay_coord_finish_static_frozen(struct relay_control *c, const struct rc_grant_plan *p, uint8_t *wire)
{

	if (rc_narrow_pool(c)) {
		if (!c->repair_count) {
			return;
		}
		rc_finish_narrow(c, p, wire);
	} else {
		rc_finish_full(c, p, wire);
	}
}

#if defined(__GNUC__) && !defined(__clang__)
/* Match the measured O3 placement of the static full/narrow allocators. */
__attribute__((optimize("O3")))
#endif
RELAY_RAMFUNC void relay_coord_finish_frozen(struct relay_control *c, const struct rc_grant_plan *p, uint8_t *wire)
{
	if (c->on_demand) {
		uint16_t missing = c->missing[0] | c->missing[1] | c->missing[2] | c->missing_extra[0] | c->missing_extra[1];
		/* Every preplanned owner/child pair retains its reservation when
		 * that child is missing at freeze. Clear the exact covered lanes
		 * once; a delivered child has no bit in fresh. */
		uint32_t fresh = (c->missing[0] | ((uint32_t)c->missing[0] << 16)) & p->fresh_candidates & ~p->planned_fresh;
		unsigned available = 0, repair_slots = 0;
		for (unsigned i = 0; i < c->repair_count; i++) {
			uint16_t children = c->grants[i].children & missing;
			relay_put_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2 * i, children);
			if (children) {
				repair_slots |= 1U << i;
			} else {
				available |= 1U << i;
			}
		}
		if (c->flags & RELAY_CONTROL_MAINTENANCE) {
			available &= ~3U;
		}
		while (fresh && available) {
			unsigned i = __builtin_ctz(available);
			available &= available - 1U;
			unsigned h = p->fresh_helper[__builtin_ctz(fresh)];
			uint32_t records = fresh & p->fresh_coverage[h], rest = records;
			rest &= rest - 1U;
			rest &= rest - 1U;
			rest &= rest - 1U;
			records ^= rest;
			fresh &= ~records;
			wire[RELAY_CONTROL_OFF_OWNERS + i] = h;
			relay_put_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2 * i, records | (records >> 16));
			repair_slots |= 1U << i;
		}
		/* Match static full-pool spare-slot priority: report, surveyed
		 * alternate(s), otherwise retry the first repair reservation. */
		if (available && p->reporter >= 0) {
			unsigned i = __builtin_ctz(available);
			available &= available - 1U;
			wire[RELAY_CONTROL_OFF_OWNERS + i] = p->reporter;
		}
		if (available && (relay_control_missing(c, p->rescue_age) & p->rescue_child)) {
			unsigned i = __builtin_ctz(available);
			available &= available - 1U;
			wire[RELAY_CONTROL_OFF_OWNERS + i] = p->rescue_helper;
			relay_put_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2 * i, p->rescue_child);
			if (available && p->rescue_second < RC_MAX) {
				i = __builtin_ctz(available);
				available &= available - 1U;
				wire[RELAY_CONTROL_OFF_OWNERS + i] = p->rescue_second;
				relay_put_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2 * i, p->rescue_child);
			}
		} else if (available && repair_slots) {
			unsigned i = __builtin_ctz(available), first = __builtin_ctz(repair_slots);
			available &= available - 1U;
			wire[RELAY_CONTROL_OFF_OWNERS + i] = wire[RELAY_CONTROL_OFF_OWNERS + first];
			relay_put_be16(
				wire + RELAY_CONTROL_OFF_CHILDREN + 2 * i,
				relay_get_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2 * first)
			);
		}
		while (available) {
			unsigned i = __builtin_ctz(available);
			available &= available - 1U;
			wire[RELAY_CONTROL_OFF_OWNERS + i] = RELAY_OWNER_IDLE;
		}
		uint16_t failed = (c->missing[0] & p->fresh_survey) | c->missing[1] | c->missing[2] | c->missing_extra[0]
						| c->missing_extra[1];
		failed &= c->members & ~c->survey;
		unsigned surveys = c->survey != 0;
		while (failed && surveys < 2) {
			unsigned child = __builtin_ctz((unsigned)failed);
			failed &= failed - 1U;
			c->survey |= RC_BIT(child);
			surveys++;
		}

		return;
	}

	relay_coord_finish_static_frozen(c, p, wire);
}

#undef RC_GRANT_LIMIT

void relay_coord_read_grants(struct relay_control *c, const uint8_t *wire)
{
	for (unsigned i = 0; i < c->repair_count; i++) {
		c->grants[i].owner = wire[RELAY_CONTROL_OFF_OWNERS + i];
		c->grants[i].children = relay_get_be16(wire + RELAY_CONTROL_OFF_CHILDREN + 2U * i);
	}
}



void relay_coord_commit_grants(struct relay_coord *rc, const struct relay_control *c,
	const struct rc_grant_plan *p)
{
	if (p->survey) rc->survey_next = p->survey_next;
	if ((c->flags & RELAY_CONTROL_MAINTENANCE) && !RC_ID_OUTSIDE(p->maintenance)) {
		rc->maintenance_ms[p->maintenance] = p->now_ms;
		rc->maintenance_next = (p->maintenance + 1U) % RC_MAX;
	}
	for (unsigned slot = 0; p->reporter >= 0 && !RC_ID_OUTSIDE(p->reporter) && slot < c->repair_count; slot++) {
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
	for (unsigned age = 0; age < relay_control_cache_frames(c); age++) {
		uint32_t frame = c->frame - age;
		struct rc_watch *w = &rc->watch[frame % (rc->cache_frames ? rc->cache_frames : 3U)];
		if (w->frame != frame || w->session != c->session || w->epoch != c->epoch || w->members != c->members) {
			continue;
		}
		for (unsigned slot = 0; slot < c->repair_count; slot++) {
			const struct relay_grant *g = &c->grants[slot];
			if (g->owner >= RC_MAX) {
				continue;
			}
			uint16_t children = g->children & RC_MEMBER_MASK & relay_control_missing(c, age);
			while (children) {
				unsigned child = __builtin_ctz((unsigned)children);
				children &= children - 1U;
				if (rc_narrow_pool(c)) {
					if (!slot && !(w->rescue_tried[child] & RC_BIT(g->owner))) {
						w->r0_only[child] |= RC_BIT(g->owner);
					}
					if (slot) {
						w->r0_only[child] &= ~RC_BIT(g->owner);
					}
				}
				w->rescue_tried[child] |= RC_BIT(g->owner);
			}
		}
	}
	struct rc_watch *next = &rc->watch[(c->frame + 1U) % (rc->cache_frames ? rc->cache_frames : 3U)];
	*next = (struct rc_watch){.frame = c->frame + 1U,
		.session = c->session, .epoch = c->epoch, .members = c->members, .survey = c->survey,
		.primary_valid = c->primary_valid, .backup_valid = c->backup_valid};
	memcpy(next->helpers, c->helpers, sizeof(next->helpers));
	for (unsigned child = 0; child < RC_MAX; child++)
		next->listeners[child] = (c->survey & RC_BIT(child)) ? c->members : p->next_listeners[child];
}

/* Convenience adapters for serialized pure-policy callers. The receiver's
 * concurrent threads capture their own snapshots in bounded locked rows. */
static void rc_copy_learn(struct rc_learn_snapshot *s, const struct relay_coord *rc)
{
	memcpy(s->peer, rc->peer, sizeof(s->peer));
	memcpy(s->choice, rc->choice, sizeof(s->choice));
	for (unsigned i = 0; i < RC_MAX; i++) {
		s->t[i].direct_permille = rc->t[i].direct_permille;
		s->t[i].rssi = rc->t[i].rssi;
	}
	s->learn_members = rc->learn_members;
}
static void rc_copy_plan(struct rc_plan_snapshot *s, const struct relay_coord *rc)
{
	rc_copy_learn(&s->learn, rc);
	memcpy(s->watch, rc->watch, sizeof(s->watch));
	memcpy(s->receipt, rc->receipt, sizeof(s->receipt));
	memcpy(s->maintenance_ms, rc->maintenance_ms, sizeof(s->maintenance_ms));
	memcpy(s->report_ms, rc->report_ms, sizeof(s->report_ms));
	s->maintenance_next = rc->maintenance_next;
	s->report_next = rc->report_next;
	s->survey_next = rc->survey_next;
	s->cache_frames = rc->cache_frames;
}
void relay_coord_learn_tick(struct relay_coord *rc, uint32_t now)
{
	struct rc_learn_snapshot s;
	rc_copy_learn(&s, rc);
	relay_learn_snapshot_tick(&s, now);
	memcpy(rc->peer, s.peer, sizeof(rc->peer));
	memcpy(rc->choice, s.choice, sizeof(rc->choice));
}
void relay_coord_plan_watch(struct relay_coord *rc, struct relay_control *c)
{
	struct rc_plan_snapshot s;
	rc_copy_plan(&s, rc);
	relay_plan_snapshot_watch(&s, c);
	rc->learn_members = s.learn.learn_members;
}
void relay_coord_plan_grants(struct relay_coord *rc, struct relay_control *c,
	struct rc_grant_plan *p, uint32_t now)
{
	struct rc_plan_snapshot s;
	rc_copy_plan(&s, rc);
	relay_plan_snapshot_grants(&s, c, p, now);
}
