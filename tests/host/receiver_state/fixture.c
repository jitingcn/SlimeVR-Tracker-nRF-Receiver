#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include "connection/tracker_roster.h"
#include "system/system.h"

typedef int32_t atomic_t;
typedef int32_t atomic_val_t;
#define ATOMIC_INIT(x) (x)
#define BIT(x) (1u << (x))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define K_FOREVER 0
static void log_message(const char *format, ...) { (void)format; }
#define LOG_INF(...) log_message(__VA_ARGS__)
#define LOG_WRN(...) log_message(__VA_ARGS__)
#define LOG_ERR(...) log_message(__VA_ARGS__)
static atomic_val_t atomic_get(const atomic_t *v) { return *v; }
static void atomic_set(atomic_t *v, atomic_val_t x) { *v = x; }
static void atomic_or(atomic_t *v, atomic_val_t x) { *v |= x; }
static void atomic_and(atomic_t *v, atomic_val_t x) { *v &= x; }
static void atomic_inc(atomic_t *v) { *v = (uint32_t)*v + 1u; }
static unsigned irq_depth, mutex_depth;
static uint64_t now_ms;
static void preempt(void);
static unsigned irq_lock(void) { preempt(); unsigned key = irq_depth; irq_depth++; return key; }
static void irq_unlock(unsigned key) { assert(irq_depth == key + 1); irq_depth = key; preempt(); }
static int tracker_store_lock, esb_pairing_msgq;
static int tdma_membership_wake;
static unsigned membership_wakes;
static void k_sem_give(int *sem) { assert(sem == &tdma_membership_wake && !irq_depth); membership_wakes++; }
static void k_mutex_lock(int *lock, int timeout) { (void)lock; (void)timeout; assert(!irq_depth && !mutex_depth); mutex_depth++; }
static void k_mutex_unlock(int *lock) { (void)lock; assert(mutex_depth == 1 && !irq_depth); mutex_depth--; }
static int64_t k_uptime_get(void) { return now_ms; }
static uint32_t k_uptime_get_32(void) { return now_ms; }
static void k_msleep(unsigned ms) { assert(!irq_depth); now_ms += ms; }
static void k_msgq_purge(int *q) { (void)q; assert(!irq_depth); }
static uint64_t mock_device_addr;
struct mock_ficr { uint64_t DEVICEADDR[1]; };
#define NRF_FICR ((struct mock_ficr *)&mock_device_addr)
static uint8_t crc8_ccitt(uint8_t seed, const uint8_t *p, size_t n) { (void)p; (void)n; return seed; }
static const char *esb_pong_flag_name(uint8_t flag) { (void)flag; return "fixture"; }
uint8_t stored_trackers;
uint64_t stored_tracker_addr[MAX_TRACKERS];
static unsigned rssi[MAX_TRACKERS], ota[MAX_TRACKERS], collection[MAX_TRACKERS];
static unsigned raw_arq, invalidated, cleanup_calls, recalc_calls;
static int collection_target = -1;
static void hid_reset_rssi_smooth(uint8_t id) { assert(irq_depth); rssi[id] = 0; }
static void esb_ota_forget_tracker(uint8_t id) { assert(!irq_depth && mutex_depth); ota[id] = 0; }
static void data_collect_forget_tracker(uint8_t id) { assert(!irq_depth && mutex_depth); collection[id] = 0; }
static bool data_collect_is_target(uint8_t id) { return collection_target == id; }
static void raw_arq_reset(void) { raw_arq = 0; }
static void esb_auto_loss_invalidate(void) { assert(irq_depth); invalidated++; }
static void tracker_events_pairing_invalidate(uint32_t bits) { assert(irq_depth); (void)bits; }
static void tracker_events_pairing_cleanup(void) { assert(!irq_depth); cleanup_calls++; }
static void tdma_recalculate(bool forced) { assert(forced && !irq_depth && mutex_depth); recalc_calls++; }
static void test_all_invalidate(void);

/* PRODUCTION_DECLARATIONS */

static void test_all_invalidate(void) { atomic_set(&test_all_state_valid, 0); }
struct tuple { int16_t sens[3]; uint8_t axis, command, arg; uint16_t turns; };
static struct tuple old_tuple, new_tuple;
static bool observe_tuple;
static unsigned preempt_checks;
static struct tuple capture(void)
{
	struct tuple t = {0};
	for (unsigned i = 0; i < 3; i++) t.sens[i] = pending_sens_data[0][i];
	t.axis = pending_sens_auto_axis[0]; t.turns = pending_sens_auto_revolutions[0];
	t.command = tracker_remote_command[0]; t.arg = pending_cmd_arg[0];
	return t;
}
static bool same_tuple(struct tuple a, struct tuple b)
{
	return memcmp(a.sens, b.sens, sizeof(a.sens)) == 0 && a.axis == b.axis &&
		a.turns == b.turns && a.command == b.command && a.arg == b.arg;
}
static void preempt(void)
{
	if (observe_tuple && !irq_depth) {
		struct tuple t = capture();
		assert(same_tuple(t, old_tuple) || same_tuple(t, new_tuple));
		preempt_checks++;
	}
}
static uint8_t durable[98];
static bool have_record;
static int save_error, save_calls;
int sys_write_checked(uint16_t key, const void *data, size_t len)
{
	assert(key == TRACKER_ROSTER && len == sizeof(durable));
	assert(mutex_depth && !irq_depth);
	save_calls++;
	if (save_error) return save_error;
	memcpy(durable, data, len); have_record = true;
	return 0;
}
int sys_read_checked(uint16_t key, void *data, size_t len)
{
	assert(key == TRACKER_ROSTER && len == sizeof(durable) && have_record);
	memcpy(data, durable, len);
	return (int)len;
}

/* PRODUCTION_FUNCTIONS */

static void test_shadow(void)
{
	stored_trackers = 1;
	tdma_active_mask = 0;
	const uint32_t starts[] = {UINT32_MAX, 0, 1, INT32_MAX - 1u};
	for (unsigned n = 0; n < sizeof(starts) / sizeof(starts[0]); n++) {
		unsigned int key = irq_lock();
		tdma_shadow_forget_tracker(0);
		irq_unlock(key);
		now_ms = starts[n];
		unsigned wakes_before = membership_wakes;
		tdma_shadow_observe(0, TDMA_SHADOW_EVIDENCE_PING);
		assert(atomic_get(&tdma_shadow_pending_join_mask) == 1);
		assert(membership_wakes == wakes_before + 1);
		assert(tdma_membership_deadline_armed);
		assert(tdma_membership_deadline_ms == (uint32_t)now_ms + 101u);
		tdma_shadow_update((uint32_t)now_ms);
		assert(atomic_get(&tdma_shadow_desired_mask) == 1);
		assert(!atomic_get(&tdma_shadow_pending_join_mask));
		assert(tdma_membership_deadline_armed);
		tdma_active_mask = 1;
		tdma_shadow_update((uint32_t)now_ms + TDMA_SHADOW_FRESH_MS);
		assert(atomic_get(&tdma_shadow_pending_leave_mask) == 1);
		tdma_shadow_update((uint32_t)now_ms + TDMA_SHADOW_LEAVE_GRACE_MS - 1);
		assert(atomic_get(&tdma_shadow_desired_mask) == 1);
		tdma_shadow_update((uint32_t)now_ms + TDMA_SHADOW_LEAVE_GRACE_MS);
		assert(atomic_get(&tdma_shadow_desired_mask) == 0);
		assert(tdma_membership_deadline_armed);
		tdma_active_mask = 0;
		tdma_shadow_update((uint32_t)now_ms + TDMA_SHADOW_LEAVE_GRACE_MS);
		assert(!tdma_membership_deadline_armed);
		/* A sampled future tick is still fresh across uint32 wrap. */
		tdma_shadow_observe(0, TDMA_SHADOW_EVIDENCE_DATA);
		tdma_shadow_update((uint32_t)now_ms - 1);
		assert(atomic_get(&tdma_shadow_desired_mask) == 1);
		key = irq_lock();
		tdma_shadow_forget_tracker(0);
		irq_unlock(key);
	}
}

static void test_tuples(void)
{
	stored_trackers = 1; stored_tracker_addr[0] = 123;
	now_ms = 10000; tracker_stats[0].last_packet_time = now_ms;
	old_tuple = capture(); new_tuple = old_tuple;
	new_tuple.sens[0] = 100; new_tuple.sens[1] = 200; new_tuple.sens[2] = 300;
	new_tuple.command = ESB_PONG_FLAG_SENS_SET;
	observe_tuple = true;
	esb_send_remote_command_sens(0, 1, 2, 3);
	assert(same_tuple(capture(), new_tuple));
	old_tuple = capture(); new_tuple = old_tuple;
	new_tuple.axis = 2; new_tuple.turns = 17; new_tuple.command = ESB_PONG_FLAG_SENS_AUTO;
	assert(esb_send_remote_command_sens_auto(0, 2, 17));
	assert(same_tuple(capture(), new_tuple));
	old_tuple = capture(); new_tuple = old_tuple;
	new_tuple.axis = 1; new_tuple.turns = 19;
	assert(esb_send_remote_command_sens_auto_all(1, 19) == 1);
	assert(same_tuple(capture(), new_tuple));
	old_tuple = capture(); new_tuple = old_tuple;
	new_tuple.arg = 37; new_tuple.command = ESB_PONG_FLAG_DATA_COLLECT_ON;
	last_raw_valid[0] = true; raw_arq = 99;
	esb_send_remote_command_arg(0, new_tuple.command, new_tuple.arg);
	assert(same_tuple(capture(), new_tuple) && !last_raw_valid[0] && raw_arq == 0);
	assert(preempt_checks >= 8);
	observe_tuple = false;
}

#ifndef STATE_ONLY_LEGACY
/* GENERATED_SLOT_CHECKS */
static void test_slots(void)
{
	stored_trackers = 0; memset(stored_tracker_addr, 0, sizeof(stored_tracker_addr));
	tracker_store_ready = false;
	assert(esb_add_pair(123, false) == -EIO && esb_pop_pair() == -EIO);
	assert(save_calls == 0 && stored_trackers == 0);
	tracker_store_ready = true;
	assert(esb_add_pair(123, false) == 0);
	seed_slot(0);
	/* Explicitly exercise failed persistence without any live-state reset. */
	save_error = -EIO;
	assert(esb_pop_pair() == -EIO);
	assert(stored_trackers == 1 && stored_tracker_addr[0] == 123);
	assert(last_ping_time[0] != 0 && pending_sens_data[0][0] != 0 && rssi[0] != 0);
	assert(esb_clear() == -EIO);
	assert(stored_trackers == 1 && stored_tracker_addr[0] == 123);
	assert(esb_add_pair(456, false) == -EIO && stored_trackers == 1);
	save_error = 0;
	collection_target = 0; raw_arq = 99;
	assert(esb_pop_pair() == 0 && stored_trackers == 0);
	check_slot(0);
	assert(!ota[0] && !collection[0] && !raw_arq);
	assert(esb_add_pair(456, false) == 0);
	check_slot(0);
	assert(stored_tracker_addr[0] == 456);
	/* More than the removed async queue's capacity cannot silently drop saves. */
	for (unsigned n = 0; n < 35; n++) {
		assert(esb_pop_pair() == 0);
		assert(esb_add_pair(1000 + n, false) == 0);
	}
	uint8_t count; uint64_t recovered[MAX_TRACKERS];
	assert(tracker_roster_load(&count, recovered) == 0);
	assert(count == 1 && recovered[0] == 1034);
	for (unsigned id = 0; id < MAX_TRACKERS; id++) seed_slot(id);
	test_all_state_valid = test_all_enabled = 1;
	test_all_requested_tps = test_all_effective_tps = 200;
	int generation = test_all_generation;
	assert(esb_clear() == 0);
	assert(stored_trackers == 0 && !test_all_state_valid && !test_all_enabled);
	assert(!test_all_requested_tps && !test_all_effective_tps && test_all_generation == generation + 1);
	for (unsigned id = 0; id < MAX_TRACKERS; id++) {
		check_slot(id); assert(!ota[id] && !collection[id]);
	}
	assert(tracker_roster_load(&count, recovered) == 0 && count == 0);
	assert(save_calls > 70 && recalc_calls > 35 && !irq_depth && !mutex_depth);
}
#endif

int main(int argc, char **argv)
{
	assert(argc == 2);
	if (!strcmp(argv[1], "shadow")) test_shadow();
	else if (!strcmp(argv[1], "tuples")) test_tuples();
#ifndef STATE_ONLY_LEGACY
	else if (!strcmp(argv[1], "slots")) test_slots();
#endif
	else assert(!"unknown case");
	puts("receiver state case passed");
	return 0;
}
