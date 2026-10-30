#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

struct k_spinlock { unsigned char unused; };
typedef struct { unsigned int value; } k_spinlock_key_t;

/* PRODUCTION DECLARATIONS */

void esb_set_stats_detailed(uint32_t duration_seconds);
bool esb_get_stats_detailed_enabled(void);
static void expire_once(uint64_t now);

enum preemption_action {
	OBSERVE_PAIR,
	EXPIRE_DISPLAY,
	ENABLE_TIMED,
};

enum {
	ENABLED_FIELD = 1,
	DEADLINE_FIELD = 2,
};

static unsigned lock_depth;
static int64_t clock_ms;
static unsigned clock_reads;
static unsigned auto_disable_logs;
static bool preemption_armed;
static bool preemption_pending;
static unsigned attempted_fields;
static unsigned deferred_fields;
static unsigned unlocked_fields;
static unsigned preemptions;
static bool observed_enabled;
static int64_t observed_deadline;
static enum preemption_action preemption_action;
static uint64_t preemption_time;
static uint32_t preemption_duration;

static void run_preemption(void);

static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock)
{
	assert(lock == &stats_display_lock);
	assert(lock_depth == 0);
	lock_depth = 1;
	return (k_spinlock_key_t){.value = 0x51};
}

static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
	assert(lock == &stats_display_lock);
	assert(key.value == 0x51 && lock_depth == 1);
	lock_depth = 0;
	if (preemption_armed && preemption_pending) {
		run_preemption();
	}
}

static void run_preemption(void)
{
	assert(preemption_armed && lock_depth == 0);
	preemption_armed = false;
	preemption_pending = false;
	preemptions++;

	k_spinlock_key_t key = k_spin_lock(&stats_display_lock);
	observed_enabled = stats_detailed_enabled;
	observed_deadline = stats_detailed_end_time;
	k_spin_unlock(&stats_display_lock, key);

	if (preemption_action == EXPIRE_DISPLAY) {
		expire_once(preemption_time);
	} else if (preemption_action == ENABLE_TIMED) {
		esb_set_stats_detailed(preemption_duration);
	}
}

static void attempt_preemption(unsigned field)
{
	if (!preemption_armed) {
		return;
	}
	attempted_fields |= field;
	if (lock_depth) {
		deferred_fields |= field;
		preemption_pending = true;
	} else {
		unlocked_fields |= field;
		run_preemption();
	}
}

static bool *instrument_enabled(void)
{
	attempt_preemption(ENABLED_FIELD);
	return &stats_detailed_enabled;
}

static int64_t *instrument_deadline(void)
{
	attempt_preemption(DEADLINE_FIELD);
	return &stats_detailed_end_time;
}

static int64_t k_uptime_get(void)
{
	assert(lock_depth == 0);
	clock_reads++;
	return clock_ms;
}

static void log_info(const char *message)
{
	assert(lock_depth == 0);
	assert(strcmp(message, "Detailed stats auto-disabled") == 0);
	auto_disable_logs++;
}
#define LOG_INF(...) log_info(__VA_ARGS__)

/* PRODUCTION FUNCTIONS */

static void expire_once(uint64_t now)
{
/* PRODUCTION EXPIRATION */
}

static void reset_fixture(void)
{
	assert(lock_depth == 0 && !preemption_armed && !preemption_pending);
	k_spinlock_key_t key = k_spin_lock(&stats_display_lock);
	stats_detailed_enabled = false;
	stats_detailed_end_time = 0;
	k_spin_unlock(&stats_display_lock, key);
	clock_ms = 0;
	clock_reads = 0;
	auto_disable_logs = 0;
}

static void expect_state(bool enabled, int64_t deadline)
{
	assert(esb_get_stats_detailed_enabled() == enabled);
	k_spinlock_key_t key = k_spin_lock(&stats_display_lock);
	assert(stats_detailed_enabled == enabled);
	assert(stats_detailed_end_time == deadline);
	k_spin_unlock(&stats_display_lock, key);
}

static void arm_preemption(enum preemption_action action, uint64_t now, uint32_t duration)
{
	assert(lock_depth == 0 && !preemption_armed && !preemption_pending);
	preemption_action = action;
	preemption_time = now;
	preemption_duration = duration;
	attempted_fields = 0;
	deferred_fields = 0;
	unlocked_fields = 0;
	preemptions = 0;
	observed_enabled = false;
	observed_deadline = -1;
	preemption_armed = true;
}

static void expect_preemption(unsigned fields, bool enabled, int64_t deadline)
{
	assert(!preemption_armed && !preemption_pending && lock_depth == 0);
	assert(attempted_fields == fields);
	assert(deferred_fields == fields);
	assert(unlocked_fields == 0);
	assert(preemptions == 1);
	assert(observed_enabled == enabled);
	assert(observed_deadline == deadline);
}

static void test_toggles(void)
{
	reset_fixture();
	assert(esb_get_stats_detailed_enabled() == false);
	assert(esb_toggle_stats_detailed() == true);
	expect_state(true, 0);
	assert(esb_toggle_stats_detailed() == false);
	expect_state(false, 0);
	esb_set_stats_detailed(0);
	expect_state(true, 0);
	esb_set_stats_detailed(0);
	expect_state(false, 0);
	assert(clock_reads == 0);
	assert(auto_disable_logs == 0);
}

static void test_timed_expiry(void)
{
	reset_fixture();
	clock_ms = 1234;
	esb_set_stats_detailed(6);
	expect_state(true, 7234);
	assert(clock_reads == 1);
	expire_once(7233);
	expect_state(true, 7234);
	assert(auto_disable_logs == 0);
	expire_once(7234);
	expect_state(false, 0);
	assert(auto_disable_logs == 1);
	expire_once(7235);
	expect_state(false, 0);
	assert(auto_disable_logs == 1);
}

static void test_refresh_and_indefinite(void)
{
	reset_fixture();
	clock_ms = 1000;
	esb_set_stats_detailed(2);
	expect_state(true, 3000);
	clock_ms = 1500;
	esb_set_stats_detailed(6);
	expect_state(true, 7500);
	expire_once(3000);
	expect_state(true, 7500);
	assert(esb_toggle_stats_detailed() == false);
	expect_state(false, 0);
	assert(esb_toggle_stats_detailed() == true);
	expect_state(true, 0);
	expire_once(UINT64_C(100000000000));
	expect_state(true, 0);
	esb_set_stats_detailed(0);
	expect_state(false, 0);
	esb_set_stats_detailed(0);
	expire_once(UINT64_C(100000000000));
	expect_state(true, 0);
	assert(clock_reads == 2);
	assert(auto_disable_logs == 0);
}

static void test_large_duration(void)
{
	reset_fixture();
	clock_ms = 17;
	esb_set_stats_detailed(UINT32_MAX);
	expect_state(true, INT64_C(4294967295017));
	assert(clock_reads == 1);
	assert(auto_disable_logs == 0);
}

static void test_attempted_preemption(void)
{
	reset_fixture();
	clock_ms = 1000;
	esb_set_stats_detailed(2);
	clock_ms = 3000;
	arm_preemption(EXPIRE_DISPLAY, 3000, 0);
	esb_set_stats_detailed(6);
	expect_preemption(3, true, 9000);
	expect_state(true, 9000);
	assert(auto_disable_logs == 0);

	arm_preemption(OBSERVE_PAIR, 0, 0);
	assert(esb_get_stats_detailed_enabled() == true);
	expect_preemption(1, true, 9000);

	clock_ms = 4000;
	arm_preemption(ENABLE_TIMED, 0, 7);
	assert(esb_toggle_stats_detailed() == false);
	expect_preemption(3, false, 0);
	expect_state(true, 11000);

	clock_ms = 11000;
	arm_preemption(ENABLE_TIMED, 0, 7);
	expire_once(11000);
	expect_preemption(3, false, 0);
	expect_state(true, 18000);
	assert(auto_disable_logs == 1);

	arm_preemption(OBSERVE_PAIR, 0, 0);
	esb_set_stats_detailed(0);
	expect_preemption(3, false, 0);
	expect_state(false, 0);

	clock_ms = 20000;
	arm_preemption(OBSERVE_PAIR, 0, 0);
	esb_set_stats_detailed(2);
	expect_preemption(3, true, 22000);
	expect_state(true, 22000);
	assert(clock_reads == 5);
	assert(auto_disable_logs == 1);
}

int main(void)
{
	test_toggles();
	test_timed_expiry();
	test_refresh_and_indefinite();
	test_large_duration();
	test_attempted_preemption();
	puts("receiver detailed-stats display tests passed");
	return 0;
}
