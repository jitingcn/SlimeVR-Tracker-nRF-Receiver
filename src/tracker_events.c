#include "tracker_events.h"
#include "connection/tracker_event_protocol.h"
#include "hid.h"
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tracker_events, LOG_LEVEL_INF);

#define OP_COUNT 8
#define RX_COUNT 32
/* Receipt-time context, not proof of entry: 5 s WOM lead + 10 s calibration
 * silence + 5 s scheduling margin. Repeated notices never renew this budget. */
#define POWER_INTENT_RELEVANCE_MS 20000U
#define POWER_INTENT_ACTIVITY_MS 10000U
struct event_cache {
	struct tracker_event event;
	uint32_t last_rx, version, notified_generation, order, first_order;
	bool used, terminal, stale;
};
struct power_intent {
	uint32_t received_at, order, watermark_order;
	uint16_t sequence;
	uint8_t phase;
	bool ordered;
};
struct tracker_cache {
	uint32_t pairing_generation, nonce, retired_nonce, retired_at, high_at, order;
	uint16_t high;
	uint64_t seen;
	bool initialized;
	struct event_cache operations[OP_COUNT], rest[2];
	struct power_intent power;
};
struct rx_event {
	struct tracker_event event;
	uint32_t received_at, pairing_generation, subscription_generation;
};
struct delivery {
	struct tracker_event event;
	uint32_t generation, received_at, cache_version;
	uint8_t opcode;
};
static struct tracker_cache trackers[TRACKER_EVENT_MAX_TRACKERS];
static uint32_t pairing_generations[TRACKER_EVENT_MAX_TRACKERS];
static struct k_spinlock event_lock;
static char __noinit __aligned(4) _k_fifo_buf_event_rx_queue[RX_COUNT * sizeof(struct rx_event)];
static STRUCT_SECTION_ITERABLE(k_msgq, event_rx_queue) =
	Z_MSGQ_INITIALIZER(event_rx_queue, _k_fifo_buf_event_rx_queue, sizeof(struct rx_event), RX_COUNT);
static uint32_t usb_generation = 1, subscription_generation = 1;
static uint32_t lease_started, dropped;
static uint8_t subscription_filter = 255, subscription_mask;
static bool subscribed;

static uint32_t next_generation(uint32_t value)
{
	return ++value ? value : 1;
}

static bool lease_active(uint32_t now)
{
	return subscribed && (uint32_t)(now - lease_started) < TRACKER_EVENT_LEASE_MS;
}

static bool matches(uint8_t tracker, uint8_t kind, uint32_t now)
{
	return lease_active(now) && (subscription_filter == 255 || subscription_filter == tracker)
	       && (subscription_mask & tracker_event_kind_filter(kind));
}

uint32_t tracker_events_usb_generation(void)
{
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	uint32_t generation = usb_generation;
	k_spin_unlock(&event_lock, key);
	return generation;
}

void tracker_events_usb_reset(void)
{
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	usb_generation = next_generation(usb_generation);
	subscription_generation = next_generation(subscription_generation);
	subscribed = false;
	k_spin_unlock(&event_lock, key);
}

int tracker_events_control(const uint8_t *args, size_t len, uint32_t generation, uint8_t result[12])
{
	if (!args || !result || len < 4) return -EINVAL;
	if (args[0] != TRACKER_EVENT_VERSION) return -ENOTSUP;
	uint8_t action = args[1], filter = args[2], mask = args[3];
	if (action > 3 || (filter >= TRACKER_EVENT_MAX_TRACKERS && filter != 255)
	    || ((action == 1 || action == 2) ? (!mask || mask > 31) : mask != 0)) return -EINVAL;
	for (size_t i = 4; i < len; ++i) if (args[i]) return -EINVAL;
	uint32_t now = k_uptime_get_32();
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	if (generation != usb_generation) {
		k_spin_unlock(&event_lock, key);
		return -ENOTCONN;
	}
	if (action == 2 && (!lease_active(now) || filter != subscription_filter || mask != subscription_mask)) {
		k_spin_unlock(&event_lock, key);
		return -ENOENT;
	}
	if (action == 1) {
		subscription_generation = next_generation(subscription_generation);
		subscription_filter = filter;
		subscription_mask = mask;
		subscribed = true;
		lease_started = now;
	} else if (action == 2) {
		lease_started = now;
	} else if (action == 3) {
		subscribed = false;
		subscription_generation = next_generation(subscription_generation);
	}
	memset(result, 0, 12);
	result[0] = TRACKER_EVENT_VERSION;
	result[1] = 0x3f;
	result[2] = TRACKER_EVENT_MAX_REPEATS;
	result[3] = TRACKER_EVENT_SPACING_MS / 10;
	tracker_event_put16(result + 4, TRACKER_EVENT_CAL_HEARTBEAT_MS);
	tracker_event_put16(result + 6, TRACKER_EVENT_CAL_SILENCE_MS);
	tracker_event_put16(result + 8, TRACKER_EVENT_LEASE_MS);
	result[10] = subscription_filter;
	result[11] = (lease_active(now) ? 1 : 0) | (subscription_mask << 1);
	k_spin_unlock(&event_lock, key);
	return 0;
}

bool tracker_events_hid_valid(const uint8_t record[16], uint32_t generation, uint32_t now)
{
	if (!record || record[0] != 251 || record[1] != 0
	    || (record[2] != RCV_HID_OP_TRACKER_EVENT && record[2] != RCV_HID_OP_TRACKER_OBSERVATION)) return false;
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	bool valid = generation == subscription_generation && matches(record[4], record[13], now);
	k_spin_unlock(&event_lock, key);
	return valid;
}

bool tracker_events_receive(const uint8_t *packet, size_t len, uint32_t now)
{
	struct rx_event rx;
	if (!tracker_event_decode(packet, len, &rx.event)) {
		k_spinlock_key_t key = k_spin_lock(&event_lock);
		dropped++;
		k_spin_unlock(&event_lock, key);
		return false;
	}
	rx.received_at = now;
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	rx.pairing_generation = pairing_generations[rx.event.tracker_id];
	rx.subscription_generation = matches(rx.event.tracker_id, rx.event.kind, now) ? subscription_generation : 0;
	bool queued = k_msgq_put(&event_rx_queue, &rx, K_NO_WAIT) == 0;
	if (!queued) dropped++;
	k_spin_unlock(&event_lock, key);
	return queued;
}

void tracker_events_pairing_invalidate(uint32_t mask)
{
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	for (unsigned t = 0; t < TRACKER_EVENT_MAX_TRACKERS; ++t)
		if (mask & BIT(t)) pairing_generations[t] = next_generation(pairing_generations[t]);
	/* Retire queued HID actions before the pairing slot can be reused. */
	subscription_generation = next_generation(subscription_generation);
	k_spin_unlock(&event_lock, key);
}

static void reset_pairing_cache(unsigned tracker)
{
	if (trackers[tracker].pairing_generation == pairing_generations[tracker]) return;
	memset(&trackers[tracker], 0, sizeof(trackers[tracker]));
	trackers[tracker].pairing_generation = pairing_generations[tracker];
}

void tracker_events_pairing_cleanup(void)
{
	/* One cache per critical section; never called under the store IRQ lock.
	 * RX can initialize the new generation first without being erased here. */
	for (unsigned t = 0; t < TRACKER_EVENT_MAX_TRACKERS; ++t) {
		k_spinlock_key_t key = k_spin_lock(&event_lock);
		reset_pairing_cache(t);
		k_spin_unlock(&event_lock, key);
	}
}

static bool same_event(const struct tracker_event *a, const struct tracker_event *b)
{
	return a->nonce == b->nonce && a->event_seq == b->event_seq && a->operation_id == b->operation_id
	       && a->tracker_id == b->tracker_id && a->kind == b->kind && a->event == b->event
	       && a->outcome == b->outcome && a->phase == b->phase && a->detail == b->detail;
}

static bool newer(uint16_t a, uint16_t b)
{
	return (int16_t)(a - b) > 0;
}

static bool unseen(struct tracker_cache *tracker, uint16_t seq, uint32_t now)
{
	if (!tracker->initialized || (uint32_t)(now - tracker->high_at) >= TRACKER_EVENT_TTL_MS) {
		tracker->order += 65536U;
		/* Rebase, rather than discard, the power watermark. A delayed notice
		 * older than a cancellation remains older in the new coordinate. */
		if (tracker->power.ordered)
			tracker->power.watermark_order = tracker->order + (int16_t)(tracker->power.sequence - seq);
		tracker->initialized = true;
		tracker->high = seq;
		tracker->seen = 1;
		tracker->high_at = now;
		return true;
	}
	if (newer(seq, tracker->high)) {
		uint16_t shift = seq - tracker->high;
		tracker->order += shift;
		tracker->seen = shift >= 64 ? 1 : (tracker->seen << shift) | 1;
		tracker->high = seq;
		tracker->high_at = now;
		return true;
	}
	uint16_t age = tracker->high - seq;
	if (age >= 64 || (tracker->seen & (UINT64_C(1) << age))) return false;
	tracker->seen |= UINT64_C(1) << age;
	return true;
}

static struct event_cache *find_operation(struct tracker_cache *tracker, const struct tracker_event *event)
{
	for (unsigned i = 0; i < OP_COUNT; ++i) {
		struct event_cache *op = &tracker->operations[i];
		if (op->used && op->event.operation_id == event->operation_id && op->event.kind == event->kind) return op;
	}
	return NULL;
}

static struct event_cache *allocate_operation(struct tracker_cache *tracker, uint32_t now)
{
	struct event_cache *oldest = NULL;
	for (unsigned i = 0; i < OP_COUNT; ++i) {
		struct event_cache *op = &tracker->operations[i];
		if (!op->used) return op;
		if (!oldest || (op->terminal && !oldest->terminal)
		    || (op->terminal == oldest->terminal && (uint32_t)(now - op->last_rx) > (uint32_t)(now - oldest->last_rx))) oldest = op;
	}
	return oldest;
}

static struct delivery observation(const struct event_cache *cache, uint8_t reason, uint32_t now)
{
	struct delivery out = {.event = cache->event, .opcode = RCV_HID_OP_TRACKER_OBSERVATION,
		.received_at = now};
	out.event.outcome = CAL_OUTCOME_UNKNOWN;
	if (out.event.event == CAL_EVENT_STATE) {
		out.event.phase = out.event.kind == TRACKER_EVENT_KIND_FUSION_REST
			? FUSION_REST_UNKNOWN : TRACKER_REST_UNKNOWN;
	} else {
		out.event.event = CAL_EVENT_END;
		out.event.detail = reason;
	}
	out.generation = matches(out.event.tracker_id, out.event.kind, now) ? subscription_generation : 0;
	return out;
}

static void power_notice(struct tracker_cache *tracker, const struct tracker_event *event,
			 uint32_t order, uint32_t now)
{
	struct power_intent *power = &tracker->power;
	/* Expanded order tolerates continuous progress beyond the wire half-range.
	 * unseen() preserves this watermark across its quiet-interval reanchor. */
	if (power->ordered && (int32_t)(order - power->watermark_order) <= 0) return;
	power->ordered = true;
	power->sequence = event->event_seq;
	power->watermark_order = order;
	if (power->phase && (uint32_t)(now - power->received_at) >= POWER_INTENT_RELEVANCE_MS)
		power->phase = 0;
	switch (event->phase) {
	case POWER_WILL_WOM:
	case POWER_WILL_SHUTDOWN:
	case POWER_WILL_REBOOT:
		if (power->phase != event->phase) {
			power->received_at = now;
			power->order = order;
		}
		power->phase = event->phase;
		break;
	case POWER_WOM_CANCELLED:
		if (power->phase == POWER_WILL_WOM) power->phase = 0;
		break;
	case POWER_BOOT:
	case POWER_WAKE:
	case POWER_WATCHDOG_RESET:
		power->phase = 0;
		break;
	}
}

static void power_operation_evidence(struct tracker_cache *tracker,
				    const struct tracker_event *event, uint32_t order, uint32_t now)
{
	struct power_intent *power = &tracker->power;
	if (!power->phase) return;
	if ((uint32_t)(now - power->received_at) >= POWER_INTENT_RELEVANCE_MS
	    || ((int32_t)(order - power->order) > 0
		&& (event->event == CAL_EVENT_ACCEPTED || event->event == CAL_EVENT_BEGIN))
	    || (uint32_t)(now - power->received_at) > POWER_INTENT_ACTIVITY_MS) {
		/* A new operation, or ongoing operation evidence beyond the entry
		 * window, makes this old intention unsuitable as silence context. */
		power->phase = 0;
	}
}

static uint8_t silence_reason(struct tracker_cache *tracker, const struct event_cache *cache, uint32_t now)
{
	struct power_intent *power = &tracker->power;
	if (power->phase && (uint32_t)(now - power->received_at) >= POWER_INTENT_RELEVANCE_MS)
		power->phase = 0;
	if (!power->phase || (int32_t)(cache->first_order - power->order) > 0)
		return CAL_REASON_TRANSPORT_SILENCE;
	return power->phase == POWER_WILL_REBOOT ? CAL_REASON_RESET : CAL_REASON_POWER_DOWN;
}

static const char *lookup(const char *const *names, size_t count, uint8_t value)
{
	return value < count ? names[value] : "unknown";
}

static void log_delivery(const struct delivery *out)
{
	static const char *const kinds[] = {"none", "IMU_ZRO", "ACCEL_POSES", "MAG_MANUAL", "MAG_ONLINE", "GYRO_SENS", "TCAL_BOOT", "TCAL_RUNTIME"};
	static const char *const events[] = {"NONE", "ACCEPTED", "BEGIN", "STEP", "END", "REJECTED", "STATE", "NOTICE"};
	static const char *const outcomes[] = {"NONE", "SUCCESS", "FAILED", "CANCELLED", "SKIPPED", "UNKNOWN"};
	static const char *const phases[] = {"NONE", "IDENTIFY", "WAIT_STILL", "SENSOR_RETRIM", "COLLECT", "WAIT_POSE", "CAPTURE_POSE", "POSE_DONE", "RETRY", "WAIT_ROTATION", "RECORD_ROTATION", "FIT", "FREEZE", "VALIDATE", "PROBATION", "APPLY_PENDING", "APPLIED", "CONFIRM", "STORAGE", "COVERAGE"};
	static const char *const reasons[] = {"NONE", "BUSY", "UNSUPPORTED", "INVALID_ARGUMENT", "SENSOR_UNAVAILABLE", "MOTION", "SAMPLE_TIMEOUT", "INSUFFICIENT_SAMPLES", "POSE_TIMEOUT", "FIT_ERROR", "INVALID_MODEL", "QUALITY", "NO_BENEFIT", "ENVIRONMENT_ONLY", "DISABLED", "REPLACED", "POWER_DOWN", "RESET", "STORAGE_ERROR", "CANDIDATE_REJECTED", "NO_TCAL_COVERAGE", "START_TIMEOUT", "RECORD_TIMEOUT", "TEMPERATURE", "RADIAL", "DIP", "COVERAGE", "EXPIRED", "INVALID_SAMPLE", "OVERFLOW", "TRANSPORT_SILENCE", "SESSION_CHANGED", "PARTIAL"};
	static const char *const rest_phases[] = {"NOT_REST", "REST", "UNKNOWN", "UNAVAILABLE"};
	static const char *const fusion_phases[] = {"NOT_REST_DETECTED", "REST_DETECTED", "UNKNOWN", "UNAVAILABLE"};
	static const char *const rest_reasons[] = {"OBSERVED", "RESET", "SUSPENDED", "NO_FRESH_FRAME", "INITIALIZING"};
	static const char *const backends[] = {"UNKNOWN", "VQF", "EQF"};
	static const char *const power_reasons[] = {"UNKNOWN", "WOM_NORMAL", "WOM_FORCED"};
	static const char *const power_phases[] = {"NONE", "WILL_WOM", "WILL_SHUTDOWN", "BOOT", "WAKE", "WILL_REBOOT", "WOM_CANCELLED", "WATCHDOG_RESET"};
	const struct tracker_event *e = &out->event;
	bool cal = tracker_event_is_calibration(e->kind);
	const char *kind, *phase, *detail = "value";
	if (cal) {
		kind = lookup(kinds, sizeof(kinds) / sizeof(kinds[0]), e->kind & TRACKER_EVENT_KIND_MASK);
		phase = lookup(phases, sizeof(phases) / sizeof(phases[0]), e->phase);
		if (e->event == CAL_EVENT_END || e->event == CAL_EVENT_REJECTED || e->phase == CAL_PHASE_RETRY || e->phase == CAL_PHASE_STORAGE)
			detail = lookup(reasons, sizeof(reasons) / sizeof(reasons[0]), e->detail);
		else if (e->phase >= CAL_PHASE_WAIT_POSE && e->phase <= CAL_PHASE_POSE_DONE) detail = "pose";
		else if (e->phase == CAL_PHASE_WAIT_ROTATION || e->phase == CAL_PHASE_RECORD_ROTATION) detail = "axis";
		else if (e->phase == CAL_PHASE_COVERAGE) detail = "percent";
	} else if (e->kind == TRACKER_EVENT_KIND_TRACKER_REST) {
		kind = "TRACKER_REST"; phase = lookup(rest_phases, 4, e->phase); detail = lookup(rest_reasons, 5, e->detail);
	} else if (e->kind == TRACKER_EVENT_KIND_FUSION_REST) {
		kind = "FUSION_REST"; phase = lookup(fusion_phases, 4, e->phase); detail = lookup(backends, 3, e->detail);
	} else if (e->kind == TRACKER_EVENT_KIND_POWER) {
		kind = "POWER"; phase = lookup(power_phases, 8, e->phase);
		detail = e->phase == POWER_WILL_WOM || e->phase == POWER_WOM_CANCELLED
			? lookup(power_reasons, 3, e->detail) : "NONE";
	} else {
		kind = "BUTTON"; phase = "CLICK_GROUP"; detail = e->detail == 255 ? "count_at_least" : "count";
	}
	LOG_INF("EVT%s trk=%u boot=%08x op=%u seq=%u kind=%s origin=%s event=%s phase=%s(%u) outcome=%s detail=%s(%u) rx_ms=%u",
		out->opcode == RCV_HID_OP_TRACKER_OBSERVATION ? " observation" : "", e->tracker_id,
		e->nonce, e->operation_id, e->event_seq, kind, cal ? (e->kind & CAL_EVENT_ORIGIN_AUTO ? "auto" : "user") : "na",
		lookup(events, 8, e->event), phase, e->phase, lookup(outcomes, 6, e->outcome), detail, e->detail, out->received_at);
}

/* No owner lock is held across logging or HID FIFO admission. A rest receipt is
 * committed only if both the cache version and subscription still match. */
static void deliver(const struct delivery *out, bool log_event)
{
	if (log_event) log_delivery(out);
	if (!out->generation) return;
	uint8_t record[TRACKER_EVENT_HID_LEN];
	tracker_event_encode_hid(record, &out->event, out->opcode);
	if (!tracker_events_hid_valid(record, out->generation, k_uptime_get_32())
	    || !hid_write_tracker_event(record, out->generation)) return;
	if (out->event.event == CAL_EVENT_STATE && out->event.outcome == CAL_OUTCOME_NONE) {
		k_spinlock_key_t key = k_spin_lock(&event_lock);
		struct event_cache *rest = &trackers[out->event.tracker_id].rest[out->event.kind == TRACKER_EVENT_KIND_FUSION_REST];
		if (rest->used && !rest->stale && rest->version == out->cache_version
		    && out->generation == subscription_generation && same_event(&rest->event, &out->event))
			rest->notified_generation = out->generation;
		k_spin_unlock(&event_lock, key);
	}
}

static void process_rx(const struct rx_event *rx)
{
	struct delivery output[OP_COUNT + 3];
	unsigned count = 0;
	const struct tracker_event *e = &rx->event;
	k_spinlock_key_t key = k_spin_lock(&event_lock);
	struct tracker_cache *tracker = &trackers[e->tracker_id];
	if (rx->pairing_generation != pairing_generations[e->tracker_id]) goto done;
	reset_pairing_cache(e->tracker_id);
	if (tracker->nonce != e->nonce) {
		if (e->nonce == tracker->retired_nonce && (uint32_t)(rx->received_at - tracker->retired_at) < TRACKER_EVENT_TTL_MS) goto done;
		if (tracker->nonce) {
			for (unsigned i = 0; i < OP_COUNT; ++i)
				if (tracker->operations[i].used && !tracker->operations[i].terminal)
					output[count++] = observation(&tracker->operations[i], CAL_REASON_SESSION_CHANGED, rx->received_at);
			for (unsigned i = 0; i < 2; ++i)
				if (tracker->rest[i].used && !tracker->rest[i].stale)
					output[count++] = observation(&tracker->rest[i], CAL_REASON_SESSION_CHANGED, rx->received_at);
			tracker->retired_nonce = tracker->nonce;
			tracker->retired_at = rx->received_at;
		}
		tracker->nonce = e->nonce;
		tracker->initialized = false;
		memset(tracker->operations, 0, sizeof(tracker->operations));
		memset(tracker->rest, 0, sizeof(tracker->rest));
		memset(&tracker->power, 0, sizeof(tracker->power));
	}
	bool state = e->event == CAL_EVENT_STATE;
	bool storage = tracker_event_is_calibration(e->kind) && e->event == CAL_EVENT_STEP && e->phase == CAL_PHASE_STORAGE;
	struct event_cache *cache = state ? &tracker->rest[e->kind == TRACKER_EVENT_KIND_FUSION_REST] : find_operation(tracker, e);
	/* Long-lived heartbeats must be recognized before the half-range window. */
	if (cache && cache->used && same_event(&cache->event, e)) {
		if (tracker_event_is_calibration(e->kind))
			power_operation_evidence(tracker, e, cache->order, rx->received_at);
		cache->last_rx = rx->received_at;
		if (cache->stale) {
			cache->stale = false;
			if (state) {
				cache->notified_generation = 0;
				output[count++] = (struct delivery){.event = *e, .opcode = RCV_HID_OP_TRACKER_OBSERVATION,
					.generation = rx->subscription_generation, .received_at = rx->received_at,
					.cache_version = cache->version};
			}
		}
		goto done;
	}
	if (!unseen(tracker, e->event_seq, rx->received_at)) goto done;
	uint32_t order = tracker->order - (uint16_t)(tracker->high - e->event_seq);
	if (state && cache->used && (int32_t)(order - cache->order) <= 0) goto done;
	output[count++] = (struct delivery){.event = *e, .opcode = RCV_HID_OP_TRACKER_EVENT,
		.generation = rx->subscription_generation, .received_at = rx->received_at};
	if (e->kind == TRACKER_EVENT_KIND_POWER) power_notice(tracker, e, order, rx->received_at);
	if (state || (tracker_event_is_calibration(e->kind) && !storage)) {
		bool replacing = cache == NULL;
		if (replacing) cache = allocate_operation(tracker, rx->received_at);
		bool terminal = e->event == CAL_EVENT_END || e->event == CAL_EVENT_REJECTED;
		/* An out-of-order historical start/step remains observable, but cannot
		 * roll back a terminal result or the newest phase of an operation. */
		if (replacing || !cache->used || ((int32_t)(order - cache->order) > 0 && (!cache->terminal || terminal))) {
			uint32_t version = next_generation(cache->version);
			if (!state) power_operation_evidence(tracker, e, order, rx->received_at);
			uint32_t first_order = replacing || !cache->used ? order : cache->first_order;
			*cache = (struct event_cache){.event = *e, .last_rx = rx->received_at, .version = version,
				.order = order, .first_order = first_order, .used = true, .terminal = terminal};
			output[count - 1].cache_version = version;
		}
	}
 done:
	k_spin_unlock(&event_lock, key);
	for (unsigned i = 0; i < count; ++i) deliver(&output[i], true);
}

void tracker_events_process(uint32_t now)
{
	struct rx_event rx;
	for (unsigned i = 0; i < RX_COUNT && k_msgq_get(&event_rx_queue, &rx, K_NO_WAIT) == 0; ++i) process_rx(&rx);
	for (unsigned t = 0; t < TRACKER_EVENT_MAX_TRACKERS; ++t) {
		for (unsigned i = 0; i < OP_COUNT + 2; ++i) {
			struct delivery out;
			bool send = false, log_event = false;
			k_spinlock_key_t key = k_spin_lock(&event_lock);
			if (trackers[t].pairing_generation != pairing_generations[t]) {
				k_spin_unlock(&event_lock, key);
				continue;
			}
			bool state = i >= OP_COUNT;
			struct event_cache *cache = state ? &trackers[t].rest[i - OP_COUNT] : &trackers[t].operations[i];
			uint32_t timeout = state ? TRACKER_EVENT_REST_STALE_MS : TRACKER_EVENT_CAL_SILENCE_MS;
			if (cache->used && !cache->terminal && !cache->stale && (uint32_t)(now - cache->last_rx) >= timeout) {
				cache->stale = true;
				out = observation(cache, state ? CAL_REASON_TRANSPORT_SILENCE
					: silence_reason(&trackers[t], cache, now), now);
				send = log_event = true;
			} else if (state && cache->used && !cache->stale && matches(t, cache->event.kind, now)
			           && cache->notified_generation != subscription_generation) {
				out = (struct delivery){.event = cache->event, .opcode = RCV_HID_OP_TRACKER_OBSERVATION,
					.generation = subscription_generation, .received_at = cache->last_rx,
					.cache_version = cache->version};
				send = true;
			}
			k_spin_unlock(&event_lock, key);
			if (send) deliver(&out, log_event);
		}
	}
	static uint32_t last_diagnostic, reported_drops;
	if ((uint32_t)(now - last_diagnostic) >= 10000) {
		k_spinlock_key_t key = k_spin_lock(&event_lock);
		uint32_t total = dropped;
		k_spin_unlock(&event_lock, key);
		if (total != reported_drops) LOG_WRN("EVT ingress drops=%u", total);
		reported_drops = total;
		last_diagnostic = now;
	}
}
