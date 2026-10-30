#include "auto_channel.h"
#include "auto_channel_policy.h"
#include "connection/esb.h"
#include "connection/rssi_scan.h"
#include "connection/rssi_scan_fem.h"
#include "thread_priority.h"
#include <esb.h>
#include <hal/nrf_radio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(auto_channel, LOG_LEVEL_INF);

/* Reserve this family before any tracker dispatch, even when malformed or
 * automatic avoidance is disabled. Pairing has a checksum, not a type byte. */
bool auto_channel_is_beacon(const uint8_t *data, uint8_t length)
{
	return length >= 3 && !memcmp(data, "SAC", 3);
}

#if defined(CONFIG_RADIO_AUTO_CHANNEL)
static struct ac_policy policy;
static uint8_t receiver_identity[6];
static atomic_t initialized;
static atomic_t enabled;
static atomic_t generation;
static K_SEM_DEFINE(started, 0, 1);
static int16_t scores[101];
#define AC_SCAN_ROUNDS 12
#define AC_CONFIRM_ROUNDS 24
static struct auto_channel_energy_status energy[101];
static struct auto_channel_snapshot published;
static struct auto_channel_decision_status last_decision;
static uint32_t tx_attempts, tx_completed;
static uint16_t sequence;
static bool radio_fault;
static bool entropy_fault;
static uint32_t random_state;
struct observation {
	uint8_t frame[AC_FRAME_SIZE];
	uint8_t home;
	uint32_t generation;
	int64_t received;
};
K_MSGQ_DEFINE(observations, sizeof(struct observation), 16, 4);
static struct {
	unsigned candidate, rounds, count;
	int64_t deadline;
	uint8_t order[46];
	uint64_t finalists;
	bool active, confirming;
} sweep;
static int64_t next_beacon, next_scan;
static uint8_t observed_home = 255;
static uint32_t observed_generation;

#define AC_EVIDENCE_MS 1000
#define AC_WINDOW_MS 10000
#define AC_WINDOW_SAMPLES 10
#define AC_MIN_INFERRED_SAMPLES 100
enum ac_diagnostic { AC_MONITOR, AC_SCANNING, AC_WAIT_COMMIT, AC_WAIT_RECOVERY, AC_POSTCHECK };
static enum ac_diagnostic diagnostic;
static atomic_t interference_latched;
static int64_t next_evidence;
static uint32_t migration_mask, recovery_seen;
static uint8_t requested_home, requested_target;
struct migration_result {
	uint32_t generation;
	uint8_t old_home, new_home;
	bool success;
};
static struct migration_result completion;
static bool completion_pending;
static struct esb_auto_loss_snapshot recovery;
struct loss_interval {
	uint64_t received, gaps;
	uint32_t contributors;
	int64_t elapsed;
};
static struct {
	struct esb_auto_loss_snapshot previous;
	struct loss_interval interval[AC_WINDOW_SAMPLES];
	int64_t sampled;
	unsigned count, cursor;
	bool valid, resuming;
	struct esb_auto_loss_snapshot scan_baseline;
} loss;

void auto_channel_migration_complete(uint32_t token, uint8_t old_home,
				    uint8_t new_home, bool success)
{
	unsigned key = irq_lock();
	/* Only the current automatic transaction may replace the mailbox. */
	if (token == auto_channel_generation()) {
		completion = (struct migration_result){token, old_home, new_home, success};
		completion_pending = true;
	}
	irq_unlock(key);
}

static void loss_baseline(const struct esb_auto_loss_snapshot *s, int64_t now, bool resuming)
{
	loss.previous = *s;
	loss.sampled = now;
	loss.count = loss.cursor = 0;
	loss.valid = true;
	loss.resuming = resuming;
}

static bool discontinuity(const struct esb_auto_loss_snapshot *a,
			  const struct esb_auto_loss_snapshot *b)
{
	if (a->active_mask != b->active_mask || a->reset_epoch != b->reset_epoch ||
	    a->layout_epoch != b->layout_epoch || a->cap_pending != b->cap_pending ||
	    a->home != b->home || a->eligible != b->eligible) return true;
	for (unsigned i = 0; i < ESB_AUTO_LOSS_TRACKERS; i++) {
		if (!(b->active_mask & (1u << i))) continue;
		if (a->tracker[i].restarts != b->tracker[i].restarts ||
		    b->tracker[i].received < a->tracker[i].received ||
		    b->tracker[i].gaps < a->tracker[i].gaps) return true;
	}
	return false;
}

/* -1 unknown, 0 clean, 1 >20%. Ten fixed-grid intervals form a nominal
 * ten-second packet-weighted window, with at most 500ms endpoint jitter.
 * Never qualify before a full 10000ms or convert a scheduling stall to data.
 * Counters are normal unique frames only, never PING, duplicate or USB totals. */
static int loss_sample(const struct esb_auto_loss_snapshot *s, int64_t now)
{
	if (!loss.valid || !s->eligible || !s->active_mask || s->cap_pending ||
	    (s->ready_mask & s->active_mask) != s->active_mask ||
	    now - loss.sampled < AC_EVIDENCE_MS - 500 ||
	    now - loss.sampled > AC_EVIDENCE_MS + 500 ||
	    discontinuity(&loss.previous, s)) {
		loss_baseline(s, now, false);
		return -1;
	}
	struct loss_interval item = {.elapsed = now - loss.sampled};
	for (unsigned i = 0; i < ESB_AUTO_LOSS_TRACKERS; i++) {
		if (!(s->active_mask & (1u << i))) continue;
		uint32_t received = s->tracker[i].received - loss.previous.tracker[i].received;
		item.received += received;
		item.gaps += s->tracker[i].gaps - loss.previous.tracker[i].gaps;
		if (received) item.contributors |= 1u << i;
	}
	if (loss.resuming) {
		/* After a whole idle window, discard the first returning interval
		 * because its sequence gap spans that unknown period. */
		loss_baseline(s, now, !item.received);
		return -1;
	}
	loss.previous = *s;
	loss.sampled = now;
	loss.interval[loss.cursor] = item;
	loss.cursor = (loss.cursor + 1) % AC_WINDOW_SAMPLES;
	if (loss.count < AC_WINDOW_SAMPLES) loss.count++;
	if (loss.count < AC_WINDOW_SAMPLES) return -1;
	uint64_t received = 0, gaps = 0;
	uint32_t contributors = 0;
	int64_t elapsed = 0;
	for (unsigned i = 0; i < AC_WINDOW_SAMPLES; i++) {
		received += loss.interval[i].received;
		gaps += loss.interval[i].gaps;
		contributors |= loss.interval[i].contributors;
		elapsed += loss.interval[i].elapsed;
	}
	if (elapsed > AC_WINDOW_MS + 500) {
		loss_baseline(s, now, false);
		return -1;
	}
	if (elapsed >= AC_WINDOW_MS && !received) {
		loss_baseline(s, now, true);
		return -1;
	}
	if (elapsed < AC_WINDOW_MS || contributors != s->active_mask ||
	    received + gaps < AC_MIN_INFERRED_SAMPLES) return -1;
	return gaps * 100 > (received + gaps) * 20;
}

static void recovery_sample(const struct esb_auto_loss_snapshot *s, int64_t now)
{
	/* Never shrink the original participants to a later reduced roster. */
	if (!s->eligible || !migration_mask || s->cap_pending ||
	    (s->active_mask & migration_mask) != migration_mask ||
	    (s->ready_mask & migration_mask) != migration_mask ||
	    discontinuity(&recovery, s) ||
	    now - loss.sampled > AC_EVIDENCE_MS + 500) {
		recovery = *s;
		recovery_seen = 0;
		loss_baseline(s, now, false);
		return;
	}
	recovery_seen = s->ready_mask & migration_mask;
	/* All original participants have normal anchors in the same stable layout.
	 * The common rebase fences each NEXT edge; sequence gaps that began before
	 * this boundary cannot leak into the fresh ten-second window. */
	esb_auto_loss_rebase(&recovery);
	loss_baseline(&recovery, now, false);
	if (recovery.eligible && !recovery.cap_pending &&
	    (recovery.ready_mask & migration_mask) == migration_mask &&
	    (recovery.active_mask & migration_mask) == migration_mask) {
		diagnostic = AC_POSTCHECK;
	}
}

bool auto_channel_receive(const uint8_t *data, uint8_t length, uint8_t pipe, uint8_t home)
{
	if (!auto_channel_is_beacon(data, length)) return false;
	static const uint8_t zero[6];
	if (pipe || length != AC_FRAME_SIZE || memcmp(data, "SAC\x01", 4) ||
	    data[13] || data[14] || data[15] || home > 100 || data[10] != home ||
	    !atomic_get(&initialized) || !memcmp(data + 4, zero, 6) ||
	    !memcmp(data + 4, receiver_identity, 6)) return true;
	/* No policy mutation, logging, storage, or waiting in the event IRQ. */
	unsigned key = irq_lock();
	if (atomic_get(&enabled)) {
		struct observation item = {.home = home, .generation = auto_channel_generation(),
			.received = k_uptime_get()};
		memcpy(item.frame, data, AC_FRAME_SIZE);
		(void)k_msgq_put(&observations, &item, K_NO_WAIT);
	}
	irq_unlock(key);
	return true;
}

static uint32_t random_next(void)
{
	/* Hardware-seeded, noncryptographic phase jitter. Zero is a valid seed. */
	random_state = random_state * 1664525u + 1013904223u;
	return random_state;
}

static uint8_t effective_channel(void)
{
	uint8_t channel = esb_get_receiver_channel();
	return channel == 0xFF ? CONFIG_RADIO_RF_CHANNEL : channel;
}

static void reset_sweep(void)
{
	for (unsigned ch = 0; ch < 101; ch++) {
		scores[ch] = -1;
		energy[ch] = (struct auto_channel_energy_status){.peak = -1, .quiet_floor = -1};
	}
	memset(&sweep, 0, sizeof(sweep));
}

static unsigned required_visits(void)
{
	return AC_SCAN_ROUNDS + (sweep.confirming ? AC_CONFIRM_ROUNDS : 0);
}

static bool survey_channel(uint8_t channel, uint8_t home)
{
	return !sweep.confirming || channel == home ||
		(channel <= 88 && !(channel & 1) && (sweep.finalists & (UINT64_C(1) << (channel / 2))));
}

static void shuffle_sweep(uint8_t home)
{
	sweep.count = 0;
	for (unsigned ch = 0; ch <= 88; ch += 2)
		if (survey_channel(ch, home) && energy[ch].visits < required_visits())
			sweep.order[sweep.count++] = ch;
	if ((home > 88 || (home & 1)) &&
	    energy[home].visits < required_visits())
		sweep.order[sweep.count++] = home;
	for (unsigned n = sweep.count; n > 1; n--) {
		unsigned j = random_next() % n;
		uint8_t value = sweep.order[n - 1];
		sweep.order[n - 1] = sweep.order[j];
		sweep.order[j] = value;
	}
}

static bool begin_confirmation(uint8_t home, int64_t now)
{
	/* First all 45 destinations have equal survey coverage. Only the three
	 * strongest eligible finalists advance, together with the same HOME
	 * reference, to a second matched temporal budget. Unconfirmed destinations
	 * cannot be selected merely because the finalist later reveals a burst. */
	int chosen[3], values[3];
	unsigned count = 0;
	for (; count < 3; count++) {
		int channel = ac_choose(&policy, home, scores, now);
		if (channel < 0) break;
		chosen[count] = channel;
		values[count] = scores[channel];
		scores[channel] = -1;
		sweep.finalists |= UINT64_C(1) << (channel / 2);
	}
	if (!count) return false;
	for (unsigned ch = 0; ch <= 88; ch += 2) if (ch != home) scores[ch] = -1;
	for (unsigned i = 0; i < count; i++) scores[chosen[i]] = values[i];
	sweep.confirming = true;
	shuffle_sweep(home);
	return true;
}

static void publish_status(uint8_t home, uint32_t token)
{
	/* Static staging avoids consuming the small worker stack. Only this worker
	 * mutates it. IRQ lock covers publication/copy, never formatting. */
	static struct auto_channel_snapshot next;
	static const char *const phases[] = {"monitor", "scanning", "wait-commit", "wait-recovery", "postcheck"};
	memset(&next, 0, sizeof(next));
	next.initialized = atomic_get(&initialized);
	next.enabled = atomic_get(&enabled);
	next.interference_latched = atomic_get(&interference_latched);
	next.radio_fault = radio_fault;
	next.entropy_fault = entropy_fault;
	next.pairing = esb_pairing_is_active();
	memcpy(next.identity, receiver_identity, 6);
	next.home = home;
	next.generation = token;
	next.now_ms = k_uptime_get();
	next.phase = diagnostic == AC_SCANNING && sweep.confirming ? "confirming finalists" : phases[diagnostic];
	next.tx_attempts = tx_attempts;
	next.tx_completed = tx_completed;
	next.required_mask = migration_mask;
	next.recovered_mask = recovery_seen;
	struct esb_auto_loss_snapshot snapshot;
	esb_get_auto_loss_snapshot(&snapshot);
	next.ready_mask = snapshot.ready_mask;
	next.layout_epoch = snapshot.layout_epoch;
	next.cap_pending = snapshot.cap_pending;
	for (unsigned i = 0; i < AC_PEERS; i++) {
		const struct ac_peer *peer = &policy.peers[i];
		if (!peer->observations) continue;
		struct auto_channel_peer_status *out = &next.peers[next.peer_count++];
		memcpy(out->identity, peer->id, 6);
		out->home = peer->channel;
		out->observations = peer->observations;
		out->sequence = peer->sequence;
		out->last_seen_ms = peer->seen;
		out->fresh = next.now_ms >= peer->seen && next.now_ms - peer->seen < AC_EXPIRY_MS;
	}
	unsigned fresh = 0;
	for (unsigned i = 0; i < next.peer_count; i++) fresh += next.peers[i].fresh;
	next.table_full = fresh == AC_PEERS;
	for (unsigned ch = 0; ch <= 88; ch += 2)
		if (ac_occupied(&policy, ch, next.now_ms)) next.excluded |= UINT64_C(1) << (ch / 2);
	next.decision = last_decision;
	unsigned key = irq_lock();
	if (auto_channel_generation() == token && effective_channel() == home &&
	    snapshot.home == home) published = next;
	irq_unlock(key);
}

void auto_channel_get_snapshot(struct auto_channel_snapshot *out)
{
	if (!out) return;
	unsigned key = irq_lock();
	*out = published;
	irq_unlock(key);
}

/* Quiesce PTX as well as raw measurement hardware before restoring PRX. A
 * failed restore retains ownership: no subsequent caller may touch this radio. */
static int restore(void)
{
	int err = rssi_scan_fem_is_safe() ? esb_radio_quiesce() : -EIO;
	if (!err) {
		esb_set_addr_unified();
		err = esb_initialize_quiet(false);
		if (!err) err = esb_start_rx();
	}
	if (err) {
		radio_fault = true;
		LOG_ERR("Home radio restore failed: %d; retaining owner", err);
	}
	return err;
}

/* Worker owns policy, evidence and diagnostic state. Every wake includes a
 * separate 1s evidence deadline, independent of the 2..4s HOME beacon. */
static int64_t auto_step(void)
{
	int64_t now = k_uptime_get();
	uint8_t home = effective_channel();
	uint32_t token = auto_channel_generation();
	if (!next_beacon) next_beacon = now + 2000 + random_next() % 2001;
	struct migration_result result;
	unsigned key = irq_lock();
	bool completed = completion_pending;
	result = completion;
	completion_pending = false;
	irq_unlock(key);
	bool committed = completed && diagnostic == AC_WAIT_COMMIT &&
		result.generation == token && token == observed_generation &&
		result.old_home == requested_home && result.new_home == requested_target &&
		result.success && home == result.new_home;
	if (home != observed_home || token != observed_generation) {
		memset(policy.peers, 0, sizeof(policy.peers));
		reset_sweep();
		loss.valid = false;
		if (!committed) diagnostic = AC_MONITOR;
		policy.cooldown = 0;
		observed_home = home;
		observed_generation = token;
		next_evidence = now;
	}
	if (completed && diagnostic == AC_WAIT_COMMIT && result.generation == token &&
	    result.old_home == requested_home && (!result.success || result.new_home == requested_target)) {
		loss.valid = false;
		next_evidence = now;
		if (committed) {
			diagnostic = AC_WAIT_RECOVERY;
			esb_get_auto_loss_snapshot(&recovery);
			recovery_seen = 0;
			loss_baseline(&recovery, now, false);
			policy.cooldown = now + AC_COOLDOWN_MS;
			next_evidence = now + AC_EVIDENCE_MS;
		} else {
			/* Accepted announcement was not a committed move. No diagnostic
			 * post-window and no rapid retry: require ten new seconds. */
			diagnostic = AC_MONITOR;
		}
	}
	struct observation item;
	while (!k_msgq_get(&observations, &item, K_NO_WAIT)) {
		if (atomic_get(&enabled) && !atomic_get(&interference_latched) &&
		    item.home == home && item.generation == token &&
		    now >= item.received && now - item.received < AC_EXPIRY_MS &&
		    ac_observe(&policy, item.frame, item.received)) {
			for (unsigned i = 0; i < AC_PEERS; i++) {
				const struct ac_peer *peer = &policy.peers[i];
				if (peer->observations == 1 && !memcmp(peer->id, item.frame + 4, 6)) {
					LOG_INF("Heard receiver %02X%02X%02X%02X%02X%02X on HOME %u (local observation; foreign channels unknown)",
						peer->id[5], peer->id[4], peer->id[3], peer->id[2], peer->id[1], peer->id[0], home);
					break;
				}
			}
		}
	}
	bool available = auto_channel_decision_valid(token) && !esb_pairing_is_active();
	bool qualified = false;
	if (now >= next_evidence) {
		struct esb_auto_loss_snapshot snapshot;
		esb_get_auto_loss_snapshot(&snapshot);
		snapshot.eligible = snapshot.eligible && available;
		bool missed_grid = now - next_evidence >= AC_EVIDENCE_MS;
		next_evidence += ((now - next_evidence) / AC_EVIDENCE_MS + 1) * AC_EVIDENCE_MS;
		if (missed_grid) {
			loss.valid = false;
			if (diagnostic == AC_WAIT_RECOVERY) {
				recovery = snapshot;
				recovery_seen = 0;
			}
		}
		if (diagnostic == AC_WAIT_RECOVERY) {
			recovery_sample(&snapshot, now);
		} else if (diagnostic == AC_SCANNING) {
			/* Scan outages are fenced by ESB and are not loss-learning time.
			 * Structural changes still invalidate incomparable RF evidence. */
			if (!snapshot.eligible || snapshot.cap_pending ||
			    discontinuity(&loss.scan_baseline, &snapshot)) {
				reset_sweep();
				diagnostic = AC_MONITOR;
				loss.valid = false;
			}
		} else if (diagnostic == AC_POSTCHECK &&
			   (!snapshot.eligible || snapshot.cap_pending ||
			    (snapshot.active_mask & migration_mask) != migration_mask ||
			    (snapshot.ready_mask & migration_mask) != migration_mask ||
			    discontinuity(&loss.previous, &snapshot))) {
			diagnostic = AC_WAIT_RECOVERY;
			recovery = snapshot;
			recovery_seen = 0;
			loss_baseline(&snapshot, now, false);
		} else if (diagnostic != AC_WAIT_COMMIT) {
			int evidence = loss_sample(&snapshot, now);
			if (!loss.count) {
				/* Common baseline fences crossing edges for every participant,
				 * not only those that happened to report before this wake. */
				esb_auto_loss_rebase(&snapshot);
				loss.previous = snapshot;
			}
			if (diagnostic == AC_POSTCHECK && evidence >= 0) {
				if (evidence) {
					unsigned latch_key = irq_lock();
					bool current = auto_channel_decision_valid(token);
					if (current) atomic_set(&interference_latched, 1);
					irq_unlock(latch_key);
					available = false;
					if (current)
						LOG_WRN("Possible host USB/RF interference: loss remains >20%% after channel trial; use a USB extension cable and move receiver away from host. Automatic radio actions stopped until explicit channelauto on; reboot leaves avoidance disabled");
				} else {
					diagnostic = AC_MONITOR;
				}
			} else if (diagnostic == AC_MONITOR) {
				qualified = evidence == 1 && now >= policy.cooldown;
			}
		}
	}
	if (!available) {
		reset_sweep();
		if (diagnostic == AC_SCANNING) diagnostic = AC_MONITOR;
		loss.valid = false;
	}
	bool beacon_due = now >= next_beacon;
	bool scan_due = sweep.active && now >= next_scan;
	bool start_sweep = qualified && !sweep.active;
	if (!available || (!beacon_due && !scan_due && !start_sweep)) goto schedule;
	if (esb_radio_operation_begin()) {
		if (diagnostic == AC_SCANNING) diagnostic = AC_MONITOR;
		reset_sweep();
		loss.valid = false;
		goto schedule;
	}
	if (!auto_channel_decision_valid(token) || esb_pairing_is_active() || effective_channel() != home) {
		reset_sweep();
		if (diagnostic == AC_SCANNING) diagnostic = AC_MONITOR;
		loss.valid = false;
		esb_radio_operation_end();
		goto schedule;
	}
	if (start_sweep) {
		sweep.active = true;
		sweep.deadline = now + 60000;
		diagnostic = AC_SCANNING;
		loss.scan_baseline = loss.previous;
		migration_mask = loss.previous.active_mask;
		shuffle_sweep(home);
		/* Identity perturbs hardware-seeded contention jitter, never vetoes
		 * a lossy receiver merely because its peer has a higher identity. */
		next_scan = now + 40 + (random_next() ^ policy.identity[0]) % 21;
	}
	if (!beacon_due && !scan_due) {
		esb_radio_operation_end();
		goto schedule;
	}
	/* The worker deadline is millisecond-quantized. Dither within that quantum
	 * while HOME is STILL receiving, so all samples do not share one sub-ms
	 * phase. This is not extra RF blackout and is not a periodic-burst guarantee. */
	if (scan_due) k_busy_wait(random_next() % 1000);
	if (!auto_channel_decision_valid(token) || esb_pairing_is_active() || effective_channel() != home) {
		reset_sweep();
		if (diagnostic == AC_SCANNING) diagnostic = AC_MONITOR;
		loss.valid = false;
		esb_radio_operation_end();
		goto schedule;
	}
	int err = esb_radio_operation_stop();
	if (!err && beacon_due) {
		uint8_t frame[AC_FRAME_SIZE];
		ac_frame(frame, policy.identity, home, sequence++);
		tx_attempts++;
		err = esb_home_beacon_send(frame);
		if (!err) tx_completed++;
	}
	if (!err && scan_due && auto_channel_decision_valid(token) && effective_channel() == home) {
		uint8_t channel = sweep.order[sweep.candidate];
		energy[channel].attempts++;
		struct rssi_scan_observation observation;
		err = rssi_scan_measure_bounded(channel, &observation);
		if (!err && NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) err = -EIO;
		if (!err) {
			struct auto_channel_energy_status *stat = &energy[channel];
			if (stat->peak < 0 || observation.peak < stat->peak) stat->peak = observation.peak;
			if (observation.quiet_floor > stat->quiet_floor) stat->quiet_floor = observation.quiet_floor;
			stat->samples += observation.samples;
			stat->coverage_us += observation.coverage_us;
			stat->visits++;
			scores[channel] = stat->peak;
		} else if (err == -EAGAIN) {
			/* Paired-address activity censors this WHOLE visit, including its
			 * coverage denominator. Retry in a separated randomized round. */
			err = 0;
		}
	}
	int restored = restore();
	if (!radio_fault) esb_radio_operation_end();
	if (err || restored || !auto_channel_decision_valid(token) || effective_channel() != home) {
		reset_sweep();
		if (diagnostic == AC_SCANNING) diagnostic = AC_MONITOR;
		loss.valid = false;
		goto schedule;
	}
	if (scan_due) {
		next_scan = k_uptime_get() + 40 + random_next() % 21;
		bool expired = k_uptime_get() >= sweep.deadline;
		if (++sweep.candidate == sweep.count || expired) {
			sweep.candidate = 0;
			sweep.rounds++;
			bool complete = true;
			bool attempts_exhausted = false;
			for (unsigned ch = 0; ch <= 88; ch += 2) {
				if (!survey_channel(ch, home)) continue;
				complete &= energy[ch].visits == required_visits();
				attempts_exhausted |= energy[ch].attempts >= 384 && energy[ch].visits < required_visits();
			}
			complete &= energy[home].visits == required_visits();
			attempts_exhausted |= energy[home].attempts >= 384 && energy[home].visits < required_visits();
			if (complete && !expired && !sweep.confirming && begin_confirmation(home, k_uptime_get()))
				goto schedule;
			if (complete || attempts_exhausted || expired) {
				int target = complete && !expired && sweep.confirming ?
					ac_choose(&policy, home, scores, k_uptime_get()) : -1;
				last_decision = (struct auto_channel_decision_status){
					.valid = true, .time_ms = k_uptime_get(), .generation = token,
					.home = home, .target = target < 0 ? 255 : target,
					.reference = energy[home],
					.candidate = target < 0 ? (struct auto_channel_energy_status){.peak=-1,.quiet_floor=-1} : energy[target],
					.reason = expired ? "scan deadline; incomplete evidence" :
						!complete ? "incomplete/paired-address ambiguity" :
						target < 0 ? "no strict measured-energy improvement or peer exclusion" : "guarded energy trial; RF delivery unknown"
				};
				for (unsigned ch = 0; ch <= 88; ch += 2)
					if (ac_occupied(&policy, ch, last_decision.time_ms))
						last_decision.excluded |= UINT64_C(1) << (ch / 2);
				reset_sweep();
				loss.valid = false;
				diagnostic = AC_MONITOR;
				if (target >= 0) {
					requested_home = home;
					requested_target = target;
					diagnostic = AC_WAIT_COMMIT;
					if (esb_auto_channel_request(target, home, token)) {
						diagnostic = AC_MONITOR;
						last_decision.reason = "trial request rejected";
					} else {
						LOG_INF(">20%% normal sequence loss: requested trial channel %u -> %d (matched timed energy; RF delivery unknown)", home, target);
					}
				}
			} else {
				shuffle_sweep(home);
			}
		}
	}
 schedule:
	if (beacon_due) next_beacon = k_uptime_get() + 2000 + random_next() % 2001;
	int64_t deadline = next_evidence < next_beacon ? next_evidence : next_beacon;
	if (sweep.active && next_scan < deadline) deadline = next_scan;
	publish_status(home, token);
	return deadline;
}

static void auto_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	k_sem_take(&started, K_FOREVER);
	const struct device *rng = DEVICE_DT_GET(DT_NODELABEL(rng));
	if (!device_is_ready(rng) || entropy_get_entropy(rng, (uint8_t *)&random_state, sizeof(random_state))) {
		entropy_fault = true;
		LOG_ERR("Automatic channel avoidance stopped: hardware entropy seed unavailable");
		publish_status(effective_channel(), auto_channel_generation());
		return;
	}
	while (true) {
		int64_t deadline = auto_step();
		int64_t delay = deadline - k_uptime_get();
		if (delay > 0) k_msleep(delay);
	}
}
K_THREAD_DEFINE(auto_channel_worker, 2048, auto_thread, NULL, NULL, NULL,
		AUTO_CHANNEL_THREAD_PRIORITY, 0, 0);

void auto_channel_init(const uint8_t identity[6])
{
	memcpy(receiver_identity, identity, 6);
	memcpy(policy.identity, identity, 6);
	policy.cooldown = 0;
	atomic_set(&initialized, 1);
	k_sem_give(&started);
}
void auto_channel_set_enabled(bool value)
{
	unsigned key = irq_lock();
	atomic_set(&enabled, value);
	atomic_inc(&generation);
	if (value) atomic_set(&interference_latched, 0);
	/* Publish a coherent empty new epoch immediately; never relabel old peers
	 * or decisions with the new enable generation. Worker later fills it. */
	memset(&published, 0, sizeof(published));
	published.initialized = atomic_get(&initialized);
	published.enabled = value;
	published.interference_latched = atomic_get(&interference_latched);
	published.radio_fault = radio_fault;
	published.entropy_fault = entropy_fault;
	published.pairing = esb_pairing_is_active();
	published.generation = auto_channel_generation();
	published.home = effective_channel();
	published.now_ms = k_uptime_get();
	published.phase = "awaiting worker";
	memcpy(published.identity, receiver_identity, 6);
	irq_unlock(key);
	LOG_INF("Automatic channel avoidance %s", value ? "enabled" : "disabled by manual channel override");
}
uint32_t auto_channel_generation(void)
{
	return (uint32_t)atomic_get(&generation);
}
bool auto_channel_decision_valid(uint32_t expected_generation)
{
	return atomic_get(&enabled) && !atomic_get(&interference_latched) &&
		!radio_fault && !entropy_fault && auto_channel_generation() == expected_generation;
}
const char *auto_channel_status(void)
{
	if (radio_fault) return "fault (radio owner retained)";
	if (entropy_fault) return "fault (hardware entropy seed unavailable)";
	if (atomic_get(&interference_latched)) return "stopped (possible host USB/RF interference; explicit channel auto on to reset)";
	return atomic_get(&enabled) ? "active (defers during radio busy/pairing)" : "off/pinned (channelauto on to enable; reboot stays off)";
}
#else
bool auto_channel_receive(const uint8_t *data, uint8_t length, uint8_t pipe, uint8_t home)
{
	ARG_UNUSED(pipe); ARG_UNUSED(home);
	return auto_channel_is_beacon(data, length);
}
void auto_channel_init(const uint8_t identity[6]) { ARG_UNUSED(identity); }
void auto_channel_set_enabled(bool value) { ARG_UNUSED(value); }
const char *auto_channel_status(void) { return "disabled by Kconfig"; }
void auto_channel_get_snapshot(struct auto_channel_snapshot *out)
{
	if (out) *out = (struct auto_channel_snapshot){.phase = "disabled by Kconfig"};
}
uint32_t auto_channel_generation(void) { return 0; }
bool auto_channel_decision_valid(uint32_t expected_generation) { ARG_UNUSED(expected_generation); return false; }
void auto_channel_migration_complete(uint32_t token, uint8_t old_home,
				    uint8_t new_home, bool success)
{
	ARG_UNUSED(token); ARG_UNUSED(old_home); ARG_UNUSED(new_home); ARG_UNUSED(success);
}
#endif
