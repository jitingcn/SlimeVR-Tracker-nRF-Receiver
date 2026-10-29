#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "connection/tracker_event_protocol.h"

#define MAX_TRACKERS 16
#define BIT(n) (1U << (n))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ARG_UNUSED(x) (void)(x)
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define TDMA_BASE_SLOT_TICKS 16
#define TDMA_SLOT_TICKS_MAX_LEVEL0 64
#define TDMA_SLOT_ROTATION 0
static void log_message(const char *format, ...) { (void)format; }
#define LOG_INF(...) log_message(__VA_ARGS__)
#define LOG_WRN(...) log_message(__VA_ARGS__)
#define LOG_DBG(...) log_message(__VA_ARGS__)
#define LOG_ERR(...) log_message(__VA_ARGS__)
#define TPS_CALCULATION_INTERVAL_MS 1000
#define ATOMIC_INIT(x) (x)
#define TDMA_ENABLED 0
#define ESB_COMPOSITE_TYPE 0xfe
#define TEST_ALL_JOIN_CONFIG_GRACE_MS 1500
#define K_SEM_DEFINE(name, initial, maximum) struct fixture_sem name = {initial}

typedef int32_t atomic_t;
typedef int32_t atomic_val_t;
struct fixture_sem { unsigned count; };


static int64_t now = 1000;
static uint32_t tdma_active_mask = BIT(3);
static int test_all_state_valid;
static int test_all_enabled;
static bool collecting;
static bool batching;
static bool ota;
static uint8_t last_packet_sequence[MAX_TRACKERS];
static bool sequence_initialized[MAX_TRACKERS];
static int tdma_rx_discontinuity_pending, channel_change_pending;
static int atomic_set(int *value, int next) { int old = *value; *value = next; return old; }
struct esb_payload { uint8_t data[64]; uint8_t length, pipe; int8_t rssi; };
static int atomic_or(int *value, int bits) { int old = *value; *value |= bits; return old; }
static int atomic_and(int *value, int bits) { int old = *value; *value &= bits; return old; }
static unsigned sched_depth;
static void k_sched_lock(void) { assert(sched_depth++ == 0); }
static void k_sched_unlock(void) { assert(sched_depth-- == 1); }
static uint8_t last_ping_counter[MAX_TRACKERS];
static uint8_t last_pong_queued_counter[MAX_TRACKERS];

static int64_t k_uptime_get(void)
{
	return now;
}
static uint32_t k_uptime_get_32(void) { return (uint32_t)now; }


static int atomic_get(const int *value)
{
	return *value;
}

static bool data_collect_is_active(void)
{
	return collecting;
}

static bool data_collect_batch_is_active(void)
{
	return batching;
}

static bool esb_ota_relay_is_active(void)
{
	return ota;
}

static void test_all_invalidate_tracker(uint8_t id, int64_t at)
{
	ARG_UNUSED(id);
	ARG_UNUSED(at);
}

#define BUILD_ASSERT(x) _Static_assert(x, #x)
#define ESB_RF_CHANNEL_DEFAULT 255
#define ESB_RF_CHANNEL_ZERO_ENC 128
#define RADIO_RF_CHANNEL 22
static bool self_ota, esb_pairing, esb_clearing, esb_radio_fault, busy;
static bool abort_pending[MAX_TRACKERS], ping_counter_initialized[MAX_TRACKERS];
static uint8_t receiver_rf_channel = ESB_RF_CHANNEL_DEFAULT;
static uint32_t auto_loss_reset_epoch;
static uint8_t tdma_config_epoch, tdma_dynamic_slot_ticks, tdma_dynamic_active_count;
static uint32_t tdma_config_packed[MAX_TRACKERS];
static uint8_t stored_trackers = 4;
static void test_all_note_layout(uint32_t old, uint32_t next, int64_t at) {}
static void test_all_update_effective(uint8_t count, int64_t at, bool reset) {}
static bool receiver_ota_is_active(void) { return self_ota; }
static bool esb_channel_is_busy(void) { return busy; }
static bool esb_ota_relay_abort_pending(uint8_t id) { return abort_pending[id]; }
static unsigned irq_depth;
static unsigned irq_lock(void) { return irq_depth++; }
static void irq_unlock(unsigned key) { assert(irq_depth == key + 1); irq_depth = key; }
static void k_sem_give(struct fixture_sem *sem) { assert(!irq_depth); sem->count = 1; }
static void hid_reset_rssi_smooth(uint8_t id) { assert(irq_depth == 0); }
static void tdma_sync_stats_reset(void) {}
struct tdma_stats { uint32_t count; };
static struct tdma_stats g_tdma_stats[MAX_TRACKERS];
static int32_t g_last_ping_rx_time_diff_ticks[MAX_TRACKERS];
static bool g_last_ping_rx_time_diff_valid[MAX_TRACKERS];
static uint32_t g_ping_isr_rx_ticks[MAX_TRACKERS];
static bool g_ping_isr_rx_ticks_valid[MAX_TRACKERS];
static uint8_t g_bias_ppb_valid[MAX_TRACKERS], g_ping_dirty_streak[MAX_TRACKERS];
static int32_t g_bias_ppb[MAX_TRACKERS], g_bias_ref_offset[MAX_TRACKERS];
static uint32_t g_bias_ref_ticks[MAX_TRACKERS], g_last_ping_isr_rx_ticks_raw[MAX_TRACKERS];
static uint32_t g_ping_reject_count[MAX_TRACKERS], pipe_mismatch_count[MAX_TRACKERS], crc_error_count[MAX_TRACKERS];
static uint64_t last_ping_time[MAX_TRACKERS];
static atomic_t test_all_confirmed_mask, test_all_ready_after_ms[MAX_TRACKERS];
static uint8_t tracker_remote_command[MAX_TRACKERS], pending_cmd_arg[MAX_TRACKERS];
static uint16_t tracker_test_on_tps[MAX_TRACKERS], metadata_active_token[MAX_TRACKERS];
static uint8_t metadata_active_mask[MAX_TRACKERS], metadata_active_chunk[MAX_TRACKERS];
static void (*remote_confirm_cb)(uint8_t, uint8_t);
static uint32_t sys_get_be32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static uint8_t crc8_ccitt(uint8_t crc, const uint8_t *p, size_t n)
{
	while (n--) {
		crc ^= *p++;
		for (unsigned i = 0; i < 8; ++i)
			crc = (uint8_t)((crc << 1) ^ ((crc & 0x80) ? 0x07 : 0));
	}
	return crc;
}
static uint32_t k_ticks_to_us_floor32(uint32_t ticks) { return ticks; }
static uint16_t test_all_wire_tps(void) { return 0; }
static void remote_wom_receive(uint8_t id, uint8_t flag, const uint8_t *data, uint8_t counter) {}
static const char *esb_pong_flag_name(uint8_t flag) { return "command"; }
static void tdma_check_slot(uint8_t id, uint32_t ticks, int8_t rssi) {}
static bool tracker_events_receive(const uint8_t *data, uint8_t length, uint32_t at) { return false; }
static unsigned fixture_pose_count;
static void hid_write_packet_n(const uint8_t *packet, int8_t rssi) { ++fixture_pose_count; }


/* PRODUCTION BODIES */

static void tick(uint32_t received, uint32_t gaps)
{
	decision_received[3] += received;
	decision_gaps[3] += gaps;
	decision_ready_mask = tdma_active_mask;
	now += 1000;
	tdma_loss_controller_tick(now);
}

static void repeat(unsigned ticks, uint32_t received, uint32_t gaps)
{
	for (unsigned i = 0; i < ticks; i++) {
		tick(received, gaps);
	}
}

static void fast_recovery(void)
{
	repeat(9, 200, 0);
	assert(tdma_cap_level == 3);
	tick(200, 0);
	assert(tdma_cap_level == 2);
}

static void slow_probe(void)
{
	repeat(59, 185, 15);
	assert(tdma_cap_level == 3);
	tick(185, 15);
	assert(tdma_cap_level == 2);
}

static void sparse_recovery(void)
{
	for (unsigned i = 0; i < 59; i++) {
		tick(23, 1 + i % 2);
	}
	assert(tdma_cap_level == 3);
	tick(23, 2);
	assert(tdma_cap_level == 2);
	tdma_published_cap_level = 2;

	repeat(59, 37, 3);
	assert(tdma_cap_level == 2);
	tick(37, 3);
	assert(tdma_cap_level == 1);
}

static void idle_breaks_evidence(void)
{
	repeat(59, 185, 15);
	repeat(180, 0, 0);
	assert(tdma_cap_level == 3);
	tick(185, 15);
	assert(tdma_cap_level == 3);
	repeat(59, 185, 15);
	assert(tdma_cap_level == 2);
}

static void loss_blocks_probe(void)
{
	repeat(59, 185, 15);
	tick(150, 50);
	repeat(59, 185, 15);
	assert(tdma_cap_level == 3);
	repeat(4, 150, 50);
	assert(tdma_cap_level == 3);
	tick(150, 50);
	assert(tdma_cap_level == 4);
}

static void threshold_boundaries(void)
{
	repeat(60, 160, 40);
	assert(tdma_cap_level == 3);
	tdma_loss_reset_windows();
	repeat(60, 180, 20);
	assert(tdma_cap_level == 3);
	tdma_loss_reset_windows();
	repeat(60, 170, 30);
	assert(tdma_cap_level == 3);

	tdma_loss_reset_windows();
	repeat(10, 190, 10);
	assert(tdma_cap_level == 3);
	repeat(49, 190, 10);
	assert(tdma_cap_level == 3);
	tick(190, 10);
	assert(tdma_cap_level == 2);

	tdma_cap_level = tdma_published_cap_level = 3;
	tdma_loss_reset_windows();
	repeat(9, 1901, 99);
	assert(tdma_cap_level == 3);
	tick(1901, 99);
	assert(tdma_cap_level == 2);

	tdma_cap_level = tdma_published_cap_level = 3;
	tdma_loss_reset_windows();
	repeat(59, 1801, 199);
	assert(tdma_cap_level == 3);
	tick(1801, 199);
	assert(tdma_cap_level == 2);

	tdma_cap_level = tdma_published_cap_level = 3;
	tdma_loss_reset_windows();
	repeat(4, 1599, 401);
	assert(tdma_cap_level == 3);
	tick(1599, 401);
	assert(tdma_cap_level == 4);
}

static void counter_reset(void)
{
	repeat(9, 200, 0);
	decision_received[3] = 0;
	tick(200, 0);
	assert(tdma_cap_level == 3);
	repeat(9, 200, 0);
	assert(tdma_cap_level == 3);
	tick(200, 0);
	assert(tdma_cap_level == 2);
}

static void counter_wrap(void)
{
	decision_received[3] = UINT32_MAX - 100;
	tdma_prev_recv[3] = decision_received[3];
	tick(200, 0); /* Counter wrap is a discarded discontinuity, not huge loss. */
	assert(tdma_cap_level == 3);
	repeat(9, 200, 0);
	assert(tdma_cap_level == 3);
	tick(200, 0);
	assert(tdma_cap_level == 2);
}

static void restart_breaks_evidence(void)
{
	repeat(9, 200, 0);
	tracker_stats[3].restart_events++;
	tick(200, 0);
	assert(tdma_cap_level == 3);
	repeat(9, 200, 0);
	assert(tdma_cap_level == 3);
	tick(200, 0);
	assert(tdma_cap_level == 2);
}

static void ping_restart_sequence(void)
{
	assert(check_packet_sequence(3, 230) == 0);
	restart_ping(3);
	assert(check_packet_sequence(3, 0) == 0);
	assert(tracker_stats[3].total_gaps == 0);
	assert(tracker_stats[3].total_received == 2);
	assert(tdma_cap_level == 3); /* Reboot is not permission to raise rate. */
}

static void sequence_wrap(void)
{
	assert(check_packet_sequence(3, 254) == 0);
	assert(check_packet_sequence(3, 255) == 0);
	assert(check_packet_sequence(3, 0) == 0);
	assert(check_packet_sequence(3, 0) == 4);
	assert(check_packet_sequence(3, 255) == 2);
	assert(check_packet_sequence(3, 2) == 1);
	assert(tracker_stats[3].total_gaps == 1);
	assert(tracker_stats[3].restart_events == 0);
}

static void pending_publication(void)
{
	repeat(60, 185, 15);
	assert(tdma_cap_level == 2);
	repeat(180, 185, 15);
	assert(tdma_cap_level == 2);
	tdma_published_cap_level = 2;
	repeat(59, 185, 15);
	assert(tdma_cap_level == 2);
	tick(185, 15);
	assert(tdma_cap_level == 1);
}

static void excluded_modes(void)
{
	repeat(59, 185, 15);
	collecting = true;
	repeat(180, 185, 15);
	collecting = false;
	batching = true;
	repeat(180, 185, 15);
	batching = false;
	ota = true;
	repeat(180, 185, 15);
	ota = false;
	test_all_state_valid = 1;
	test_all_enabled = 1;
	repeat(180, 185, 15);
	test_all_enabled = 0;
	assert(tdma_cap_level == 3);
	tick(0, 0); /* Common baseline after leaving the excluded mode. */
	repeat(59, 185, 15);
	assert(tdma_cap_level == 3);
	tick(185, 15);
	assert(tdma_cap_level == 2);
}

static void auto_snapshot(void)
{
	struct esb_auto_loss_snapshot before, after;
	esb_get_auto_loss_snapshot(&before);
	assert(before.eligible && before.home == 22 && before.active_mask == BIT(3));
	check_packet_sequence(3, 10);
	check_packet_sequence(3, 11);
	check_packet_sequence(3, 15);
	check_packet_sequence(3, 15); /* duplicate */
	check_packet_sequence(3, 14); /* out of order */
	esb_get_auto_loss_snapshot(&after);
	assert(after.tracker[3].received == 2 && after.tracker[3].gaps == 3);
	assert(after.tracker[3].restarts == 0);
	check_packet_sequence(3, 110); /* accepted restart, not inferred gaps */
	esb_get_auto_loss_snapshot(&after);
	assert(after.tracker[3].received == 2 && after.tracker[3].gaps == 3);
	assert(after.tracker[3].restarts == 1);
	restart_ping(3);
	esb_get_auto_loss_snapshot(&after);
	assert(after.tracker[3].received == 2 && after.tracker[3].restarts == 2);
	before = after;
	esb_rx_discontinuity();
	for (unsigned i = 0; i < 20; ++i) check_packet_sequence(3, i);
	esb_get_auto_loss_snapshot(&after);
	assert(after.tracker[3].received > before.tracker[3].received);
	assert(after.reset_epoch != before.reset_epoch);
	before = after;
	esb_reset_all_stats();
	for (unsigned i = 20; i < 50; ++i) check_packet_sequence(3, i);
	esb_get_auto_loss_snapshot(&after);
	assert(after.tracker[3].received > before.tracker[3].received);
	assert(after.reset_epoch != before.reset_epoch);
	bool *excluded[] = {&collecting, &batching, &ota, &self_ota, &esb_pairing,
		&esb_clearing, &esb_radio_fault, &busy, &abort_pending[15]};
	for (unsigned i = 0; i < ARRAY_SIZE(excluded); ++i) {
		*excluded[i] = true;
		esb_get_auto_loss_snapshot(&after);
		assert(!after.eligible);
		*excluded[i] = false;
	}
	test_all_state_valid = test_all_enabled = 1;
	esb_get_auto_loss_snapshot(&after);
	assert(!after.eligible);
	test_all_enabled = 0;
	esb_get_auto_loss_snapshot(&after);
	assert(after.eligible);
	before = after;
	/* A whole excluded session between snapshots still destroys its baseline. */
	esb_auto_loss_invalidate();
	esb_get_auto_loss_snapshot(&after);
	assert(after.eligible && after.reset_epoch != before.reset_epoch);
	tdma_shadow_desired_mask |= BIT(4);
	esb_get_auto_loss_snapshot(&after);
	assert(!after.eligible && after.active_mask == BIT(3));
	tdma_active_mask |= BIT(4);
	receiver_rf_channel = ESB_RF_CHANNEL_ZERO_ENC;
	esb_get_auto_loss_snapshot(&after);
	assert(after.eligible && after.home == 0 && after.active_mask == (BIT(3) | BIT(4)));
	assert(irq_depth == 0);
}

static void sequence_lifetime(void)
{
	for (unsigned i = 0; i < 768; ++i)
		assert(check_packet_sequence(0, (uint8_t)i) == 0);
	assert(check_packet_sequence(0, 255) == 4);
	assert(check_packet_sequence(0, 254) == 2);
	assert(check_packet_sequence(0, 0) == 0);
	assert(tracker_stats[0].total_received == 769);
	assert(tracker_stats[0].total_gaps == 0);
	assert(tracker_stats[0].restart_events == 0);
}

static void blackout_reanchor(void)
{
	stored_trackers = MAX_TRACKERS;
	const unsigned offsets[] = {0, 1, 80, 81, 192, 193, 220, 255, 256, 257};
	for (unsigned o = 0; o < ARRAY_SIZE(offsets); ++o) {
		for (unsigned t = 0; t < MAX_TRACKERS; ++t) {
			sequence_initialized[t] = false;
			assert(check_packet_sequence(t, 100) == 0);
		}
		uint32_t epoch = auto_loss_reset_epoch;
		esb_rx_discontinuity();
		assert(auto_loss_reset_epoch != epoch);
		for (unsigned t = 0; t < MAX_TRACKERS; ++t) {
			uint8_t seq = (uint8_t)(100 + offsets[o]);
			struct esb_payload p = {.length = 20, .pipe = 1 + t % 7};
			p.data[0] = ESB_COMPOSITE_TYPE; p.data[1] = t; p.data[2] = 2;
			p.data[3] = 0; p.data[17] = 0x99;
			p.data[t & 1 ? 19 : 16] = seq;
			bool (*admit)(uint8_t, struct esb_payload) = t & 1 ? composite_admit : normal_admit;
			uint32_t gaps = tracker_stats[t].total_gaps;
			uint32_t restarts = tracker_stats[t].restart_events;
			assert(admit(t, p));
			assert(!admit(t, p));
			p.data[t & 1 ? 19 : 16] = (uint8_t)(seq - 1);
			assert(!admit(t, p));
			p.data[t & 1 ? 19 : 16] = (uint8_t)(seq + 1);
			assert(admit(t, p));
			assert(tracker_stats[t].total_gaps == gaps);
			assert(tracker_stats[t].restart_events == restarts);
		}
	}
	esb_rx_discontinuity();
	assert(check_packet_sequence(0, 100) == 0);
	esb_rx_discontinuity();
	for (unsigned seq = 64; seq <= 101; ++seq)
		assert(check_packet_sequence(0, seq) == 0);
}

static void ingress_membership(void)
{
	struct esb_payload p = {.length = 20, .pipe = 1};
	p.data[16] = 10;
	assert(normal_admit(1, p)); /* Forwarded, but wrong-pipe evidence is ignored. */
	assert(!atomic_get(&tdma_shadow_pending_join_mask));
	assert(!atomic_get(&tdma_shadow_live_evidence_mask));
	p.pipe = 2;
	p.data[16] = 11;
	assert(normal_admit(1, p));
	assert((uint32_t)atomic_get(&tdma_shadow_pending_join_mask) == BIT(1));
	assert(tdma_membership_deadline_armed && tdma_membership_wake.count == 1);
	uint32_t deadline = tdma_membership_deadline_ms;
	now += 10;
	p.data[0] = ESB_COMPOSITE_TYPE; p.data[1] = 2; p.data[2] = 2;
	p.data[3] = 0; p.data[17] = 0x99;
	p.pipe = 1;
	p.data[19] = 40;
	assert(composite_admit(2, p));
	assert((uint32_t)atomic_get(&tdma_shadow_pending_join_mask) == BIT(1));
	p.pipe = 3;
	p.data[19] = 41;
	assert(composite_admit(2, p));
	assert((uint32_t)atomic_get(&tdma_shadow_pending_join_mask) == (BIT(1) | BIT(2)));
	assert((uint32_t)atomic_get(&tdma_shadow_last_seen_ms[2]) == (uint32_t)now);
	assert(tdma_membership_deadline_ms == deadline);
}

static void blackout_loss_evidence(void)
{
	repeat(9, 200, 0);
	esb_rx_discontinuity();
	tick(200, 0);
	assert(tdma_cap_level == 3);
	repeat(9, 200, 0);
	assert(tdma_cap_level == 3);
	tick(200, 0);
	assert(tdma_cap_level == 2);
	tdma_published_cap_level = 2;
	channel_change_pending = 1;
	repeat(150, 150, 50);
	assert(tdma_cap_level == 2);
	channel_change_pending = 0;
	esb_rx_discontinuity();
	tick(150, 50);
	repeat(4, 150, 50);
	assert(tdma_cap_level == 2);
	tick(150, 50);
	assert(tdma_cap_level == 3);
}

static void decision_common_boundary(void)
{
	struct esb_auto_loss_snapshot before, after;
	tdma_active_mask = tdma_shadow_desired_mask = 15;
	esb_rx_discontinuity();
	assert(check_packet_sequence(0, 10) == 0);
	now += 1000; assert(check_packet_sequence(1, 10) == 0);
	now += 1000; assert(check_packet_sequence(2, 10) == 0);
	esb_get_auto_loss_snapshot(&before);
	assert(before.ready_mask == 7); /* D cannot silently disappear. */
	now += 1000; assert(check_packet_sequence(3, 10) == 0);
	esb_auto_loss_rebase(&before);
	assert(before.ready_mask == 15);
	assert(check_packet_sequence(0, 10) == 4);
	assert(check_packet_sequence(0, 9) == 2);
	assert(check_packet_sequence(0, 31) == 1);
	esb_get_auto_loss_snapshot(&after);
	assert(tracker_stats[0].total_gaps == 20);
	assert(after.tracker[0].gaps == before.tracker[0].gaps);
	assert(after.tracker[0].received == before.tracker[0].received);
	assert(check_packet_sequence(0, 52) == 1); /* Entirely after baseline. */
	esb_get_auto_loss_snapshot(&after);
	assert(after.tracker[0].gaps - before.tracker[0].gaps == 20);
	assert(after.tracker[0].received - before.tracker[0].received == 1);
}

static void decision_short_boundary(void)
{
	struct esb_auto_loss_snapshot before, after;
	assert(check_packet_sequence(3, 10) == 0);
	esb_get_auto_loss_snapshot(&before);
	tdma_cap_level = tdma_published_cap_level = 0;
	for (unsigned t = 0; t < 8; ++t) {
		esb_auto_loss_fence();
		uint8_t seq = last_packet_sequence[3];
		assert(check_packet_sequence(3, seq) == 4);
		assert(check_packet_sequence(3, (uint8_t)(seq - 1)) == 2);
		assert(check_packet_sequence(3, (uint8_t)(seq + 21)) == 1);
		for (unsigned n = 0; n < 60; ++n)
			assert(check_packet_sequence(3, (uint8_t)(last_packet_sequence[3] + 1)) == 0);
		now += 1000;
		tdma_loss_controller_tick(now);
	}
	esb_get_auto_loss_snapshot(&after);
	assert(after.reset_epoch == before.reset_epoch);
	assert(after.ready_mask == BIT(3));
	assert(after.tracker[3].gaps == 0);
	assert(tracker_stats[3].total_gaps == 160);
	assert(tracker_stats[3].restart_events == 0);
	assert(tdma_cap_level == 0);
}

static void decision_layout_boundary(void)
{
	struct esb_auto_loss_snapshot before, after;
	tdma_active_mask = tdma_shadow_desired_mask = 15;
	tdma_cap_level = tdma_published_cap_level = 0;
	for (unsigned i = 0; i < 4; ++i) check_packet_sequence(i, 10);
	tdma_dynamic_slot_ticks = tdma_slot_ticks_for(4);
	tdma_dynamic_active_count = 4;
	tdma_config_epoch = 1;
	for (unsigned i = 0; i < 4; ++i)
		tdma_config_packed[i] = tdma_pack_config(i, 4, tdma_dynamic_slot_ticks, tdma_config_epoch);
	tdma_last_reconfig_time = now;
	tdma_last_reconfig_valid = true;
	esb_get_auto_loss_snapshot(&before);
	tdma_cap_level = 1;
	tdma_recalculate(false); /* Debounced: pending must be visible. */
	esb_get_auto_loss_snapshot(&after);
	assert(after.cap_pending && after.reset_epoch == before.reset_epoch);
	now += TDMA_LOSS_RECONFIG_MIN_MS;
	tdma_recalculate(false); /* Same roster, real physical width change. */
	esb_get_auto_loss_snapshot(&after);
	assert(after.active_mask == before.active_mask);
	assert(after.layout_epoch != before.layout_epoch);
	assert(after.reset_epoch != before.reset_epoch);
	assert(!after.cap_pending && after.ready_mask == 0);
	assert(tdma_cap_level == 1); /* Invalidation retains cap. */
	assert(check_packet_sequence(0, 10) == 4);
	assert(check_packet_sequence(0, 9) == 2);
	esb_get_auto_loss_snapshot(&after);
	assert(after.ready_mask == 0);
	assert(check_packet_sequence(0, 31) == 1);
	esb_get_auto_loss_snapshot(&after);
	assert(after.ready_mask == 1 && after.tracker[0].gaps == 0);
	assert(tracker_stats[0].total_gaps == 20);
	/* Normal data proves an anchor, not delivery of the new NORMAL PONG. */
	now += 1000; tdma_loss_controller_tick(now);
	assert(tdma_loss_trigger_streak_ticks == 0);
}

static void decision_pending_membership(void)
{
	struct esb_auto_loss_snapshot snapshot;
	stored_trackers = 5;
	tdma_shadow_observe(4, TDMA_SHADOW_EVIDENCE_DATA);
	assert((uint32_t)atomic_get(&tdma_shadow_pending_join_mask) == BIT(4));
	esb_get_auto_loss_snapshot(&snapshot);
	assert(!snapshot.eligible);
	tdma_shadow_update((uint32_t)now);
	assert(!atomic_get(&tdma_shadow_pending_join_mask));
	assert((uint32_t)atomic_get(&tdma_shadow_desired_mask) == (BIT(3) | BIT(4)));
	assert(tdma_active_mask == BIT(3));
	esb_get_auto_loss_snapshot(&snapshot);
	assert(!snapshot.eligible); /* Merged desired membership is not yet published. */
	unsigned int key = irq_lock();
	tdma_shadow_forget_tracker(4);
	irq_unlock(key);
	tdma_shadow_pending_leave_mask = BIT(3);
	esb_get_auto_loss_snapshot(&snapshot);
	assert(!snapshot.eligible);
	tdma_shadow_pending_leave_mask = 0;
	esb_get_auto_loss_snapshot(&snapshot);
	assert(snapshot.eligible);
}

static void decision_genuine_loss(void)
{
	/* Ten full seconds; exact ratio survives the common-boundary fence.
	 * The auto policy itself is exercised by its worker integration suite. */
	for (unsigned lost = 60; lost <= 61; ++lost) {
		struct esb_auto_loss_snapshot before, after;
		esb_auto_loss_rebase(&before);
		check_packet_sequence(3, (uint8_t)(last_packet_sequence[3] + 1));
		esb_get_auto_loss_snapshot(&before);
		for (unsigned second = 0; second < 10; ++second) {
			assert(check_packet_sequence(3, (uint8_t)(last_packet_sequence[3] + lost + 1)) == 1);
			for (unsigned n = 1; n < 1000 - lost; ++n)
				assert(check_packet_sequence(3, (uint8_t)(last_packet_sequence[3] + 1)) == 0);
			now += 1000;
		}
		esb_get_auto_loss_snapshot(&after);
		uint32_t received = after.tracker[3].received - before.tracker[3].received;
		uint32_t gaps = after.tracker[3].gaps - before.tracker[3].gaps;
		assert(gaps == lost * 10 && received + gaps == 10000);
		assert(((uint64_t)gaps * 100 > (uint64_t)(received + gaps) * 6) == (lost == 61));
	}
}

static const struct {
	const char *name;
	void (*run)(void);
} scenarios[] = {
	{"fast-recovery", fast_recovery},
	{"slow-probe", slow_probe},
	{"sparse-recovery", sparse_recovery},
	{"idle-breaks-evidence", idle_breaks_evidence},
	{"loss-blocks-probe", loss_blocks_probe},
	{"threshold-boundaries", threshold_boundaries},
	{"counter-reset", counter_reset},
	{"counter-wrap", counter_wrap},
	{"restart-breaks-evidence", restart_breaks_evidence},
	{"ping-restart-sequence", ping_restart_sequence},
	{"sequence-wrap", sequence_wrap},
	{"pending-publication", pending_publication},
	{"excluded-modes", excluded_modes},
	{"auto-snapshot", auto_snapshot},
	{"sequence-lifetime", sequence_lifetime},
	{"blackout-reanchor", blackout_reanchor},
	{"ingress-membership", ingress_membership},
	{"blackout-loss-evidence", blackout_loss_evidence},
	{"decision-common-boundary", decision_common_boundary},
	{"decision-short-boundary", decision_short_boundary},
	{"decision-layout-boundary", decision_layout_boundary},
	{"decision-pending-membership", decision_pending_membership},
	{"decision-genuine-loss", decision_genuine_loss},
};

int main(int argc, char **argv)
{
	assert(argc == 2);
	atomic_set(&tdma_shadow_desired_mask, BIT(3));
	tdma_cap_level = 3;
	tdma_published_cap_level = 3;
	tdma_decision_ready = true;

	for (unsigned i = 0; i < ARRAY_SIZE(scenarios); i++) {
		if (strcmp(argv[1], scenarios[i].name) == 0) {
			scenarios[i].run();
			return 0;
		}
	}

	fprintf(stderr, "Unknown case: %s\n", argv[1]);
	return 2;
}
