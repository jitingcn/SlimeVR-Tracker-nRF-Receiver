#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MAX_TRACKERS 16
#define BIT(n) (1U << (n))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define ATOMIC_INIT(x) (x)
#define K_FOREVER 0
static void log_message(const char *format, ...) { (void)format; }
#define LOG_INF(...) log_message(__VA_ARGS__)
#define LOG_WRN(...) log_message(__VA_ARGS__)
#define LOG_DBG(...) log_message(__VA_ARGS__)
#define LOG_ERR(...) log_message(__VA_ARGS__)
typedef int32_t atomic_t;
typedef int32_t atomic_val_t;
static atomic_val_t atomic_get(const atomic_t *v) { return *v; }
static void atomic_set(atomic_t *v, atomic_val_t n) { *v = n; }
static void atomic_or(atomic_t *v, atomic_val_t n) { *v |= n; }
static void atomic_and(atomic_t *v, atomic_val_t n) { *v &= n; }

static uint64_t now_ms;
static unsigned irq_depth, mutex_depth;
static unsigned irq_lock(void) { return irq_depth++; }
static void irq_unlock(unsigned key) { assert(irq_depth == key + 1); irq_depth = key; }
struct fixture_sem { unsigned count; };
static struct fixture_sem tdma_initial_wake;
static unsigned wake_signals;
static void k_sem_give(struct fixture_sem *sem)
{
    assert(!irq_depth);
    sem->count = 1;
    ++wake_signals;
}
static int tracker_store_lock;
static void k_mutex_lock(int *lock, int timeout)
{
    (void)lock; (void)timeout;
    assert(!irq_depth && !mutex_depth);
    ++mutex_depth;
}
static void k_mutex_unlock(int *lock)
{
    (void)lock;
    assert(!irq_depth && mutex_depth == 1);
    --mutex_depth;
}
static int64_t k_uptime_get(void) { return (int64_t)now_ms; }
static uint32_t k_uptime_get_32(void) { return (uint32_t)now_ms; }
static uint8_t stored_trackers, saved_count;
static uint64_t stored_tracker_addr[MAX_TRACKERS], saved_addr[MAX_TRACKERS];
static int restore_error;
static int tracker_roster_load(uint8_t *count, uint64_t *addresses)
{
    assert(mutex_depth == 1 && !irq_depth);
    if (restore_error) return restore_error;
    *count = saved_count;
    memcpy(addresses, saved_addr, sizeof(saved_addr));
    return 0;
}
static bool data_collect_is_active(void) { return false; }
static bool data_collect_batch_is_active(void) { return false; }
static void tdma_sync_stats_reset(void) { assert(!irq_depth); }
static unsigned loss_invalidations, membership_notifications, effective_updates;
static uint32_t last_notified_mask;
static uint8_t last_effective_slots;
static void esb_auto_loss_invalidate(void) { assert(irq_depth); ++loss_invalidations; }
static void test_all_note_layout(uint32_t old_mask, uint32_t new_mask, int64_t at)
{
    (void)old_mask; (void)at;
    assert(!irq_depth);
    ++membership_notifications;
    last_notified_mask = new_mask;
}
static void test_all_update_effective(uint8_t slots, int64_t at, bool layout_changed)
{
    (void)at; (void)layout_changed;
    assert(!irq_depth);
    ++effective_updates;
    last_effective_slots = slots;
}
static struct { uint8_t data[13], length, pipe; } rx_payload;
static uint32_t pipe_mismatch_count[MAX_TRACKERS], crc_error_count[MAX_TRACKERS];
static uint8_t crc8_ccitt(uint8_t crc, const uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* PRODUCTION DECLARATIONS */
/* PRODUCTION FUNCTIONS */

static void reset_fixture(uint8_t count, uint64_t time)
{
    assert(!irq_depth && !mutex_depth && count <= MAX_TRACKERS);
    now_ms = time; restore_error = 0; saved_count = count;
    for (unsigned i = 0; i < MAX_TRACKERS; ++i) {
        saved_addr[i] = i < count ? 1000 + i : 0;
        stored_tracker_addr[i] = 0;
        tdma_config_packed[i] = 0;
        tdma_shadow_forget_tracker(i);
        tdma_initial_ping_at_ms[i] = 0;
    }
    stored_trackers = 0; tracker_store_ready = false;
    tdma_config_epoch = tdma_dynamic_active_count = tdma_dynamic_slot_ticks = 0;
    tdma_active_mask = 0; tdma_last_reconfig_time = 0;
    tdma_initial_allowed_mask = tdma_initial_ping_mask = tdma_initial_deadline_ms = 0;
    tdma_initial_started = tdma_initial_done = false;
    tdma_cap_level = tdma_published_cap_level = 0;
    tdma_loss_reset_windows(); atomic_set(&test_all_enabled, 0);
    atomic_set(&tdma_shadow_desired_changed_at_ms, 0);
    tdma_initial_wake.count = wake_signals = 0;
    loss_invalidations = membership_notifications = effective_updates = 0;
    last_notified_mask = 0; last_effective_slots = 0;
    memset(pipe_mismatch_count, 0, sizeof(pipe_mismatch_count));
    memset(crc_error_count, 0, sizeof(crc_error_count));
}

static uint8_t slot(uint32_t cfg) { return (uint8_t)(cfg >> 24); }
static uint8_t total(uint32_t cfg) { return (uint8_t)(cfg >> 16); }
static uint8_t width(uint32_t cfg) { return (uint8_t)(cfg >> 8); }
static uint8_t epoch_of(uint32_t cfg) { return (uint8_t)cfg; }

static bool ping(uint8_t id, bool valid_crc)
{
    memset(&rx_payload, 0, sizeof(rx_payload));
    rx_payload.length = ESB_PING_LEN; rx_payload.data[0] = ESB_PING_TYPE;
    rx_payload.data[1] = id; rx_payload.pipe = 1 + (id % 7);
    rx_payload.data[ESB_PING_LEN - 1] = crc8_ccitt(0x07, rx_payload.data, ESB_PING_LEN - 1);
    if (!valid_crc) rx_payload.data[ESB_PING_LEN - 1] ^= 1;
    return host_validated_ping();
}

static void process_initial(void)
{
    assert(tdma_initial_due((uint32_t)now_ms));
    k_mutex_lock(&tracker_store_lock, K_FOREVER);
    tdma_initial_process((uint32_t)now_ms);
    k_mutex_unlock(&tracker_store_lock);
}

static void all_rosters(uint64_t base)
{
    static const uint8_t widths[] = {0, 64, 64, 48, 36, 28, 24, 20, 18,
                                     16, 16, 16, 16, 16, 16, 16, 16};
    for (uint8_t n = 1; n <= MAX_TRACKERS; ++n) {
        reset_fixture(n, base);
        assert(tracker_store_restore() == 0 && tracker_store_ready);
        assert(!tdma_initial_started && !tdma_initial_done && !tdma_dynamic_active_count);
        assert(ping(0, true));
        uint32_t deadline = (uint32_t)(base + TDMA_INITIAL_COHORT_MS + 1U);
        assert(tdma_initial_deadline_ms == deadline && wake_signals == 1);
        assert(tdma_initial_wait_ms(500) == TDMA_INITIAL_COHORT_MS + 1U);
        for (uint8_t id = 1; id < n; ++id) {
            now_ms = base + ((uint32_t)id * TDMA_INITIAL_COHORT_MS) / (n - 1);
            assert(ping(id, true));
        }
        now_ms = base + TDMA_INITIAL_COHORT_MS;
        assert(ping(0, true)); /* duplicate does not defer deadline */
        assert(tdma_initial_deadline_ms == deadline && wake_signals == 1);
        assert(!tdma_initial_due((uint32_t)now_ms));
        assert(tdma_initial_wait_ms(500) == 1);
        assert(!tdma_dynamic_active_count && !tdma_active_mask);
        now_ms = base + TDMA_INITIAL_COHORT_MS + 1U;
        assert(tdma_initial_wait_ms(500) == 0);
        process_initial();
        assert(tdma_initial_done && !tdma_initial_started);
        assert(tdma_dynamic_active_count == n && tdma_active_mask == BIT(n) - 1U);
        assert((uint32_t)atomic_get(&tdma_shadow_desired_mask) == BIT(n) - 1U);
        assert(!atomic_get(&tdma_shadow_pending_join_mask));
        assert(last_notified_mask == BIT(n) - 1U && last_effective_slots == n);
        assert(tdma_dynamic_slot_ticks == widths[n]);
        for (uint8_t id = 0; id < n; ++id) {
            uint32_t cfg = tdma_config_packed[id];
            assert(slot(cfg) == id && total(cfg) == n && width(cfg) == widths[n]);
            assert(epoch_of(cfg) == tdma_config_epoch);
        }
        uint8_t epoch = tdma_config_epoch;
        now_ms += 1000; tdma_shadow_update((uint32_t)now_ms); tdma_recalculate();
        assert(tdma_config_epoch == epoch); /* no saved-roster -> compact handoff */
    }
}

static void eligibility_and_stale(void)
{
    reset_fixture(2, 0); assert(tracker_store_restore() == 0);
    assert(!ping(0, false));
    assert(!ping(16, true));
    assert(ping(2, true)); /* valid packet, but not a paired roster ID */
    memset(&rx_payload, 0, sizeof(rx_payload));
    rx_payload.length = ESB_PING_LEN; rx_payload.data[0] = ESB_PING_TYPE;
    rx_payload.data[1] = 0; rx_payload.pipe = 2; /* ID 0 belongs on pipe 1. */
    rx_payload.data[ESB_PING_LEN - 1] = crc8_ccitt(0x07, rx_payload.data, ESB_PING_LEN - 1);
    assert(host_validated_ping()); /* normal diagnostics retain this behavior */
    tdma_shadow_observe(1, TDMA_SHADOW_EVIDENCE_DATA);
    assert(!tdma_initial_started && !tdma_initial_ping_mask && !wake_signals);
    now_ms = 10000; assert(!tdma_initial_due((uint32_t)now_ms));
    assert(!tdma_config_epoch && !tdma_dynamic_active_count);
    assert(ping(1, true) && tdma_initial_started);
    now_ms += TDMA_SHADOW_FRESH_MS + 1;
    process_initial(); /* stale cohort does not promote a slot */
    assert(!tdma_initial_done && !tdma_initial_started && !tdma_dynamic_active_count);
    assert(ping(1, true) && tdma_initial_started && wake_signals == 2);
    now_ms += TDMA_INITIAL_COHORT_MS + 1;
    process_initial();
    assert(tdma_active_mask == BIT(1) && slot(tdma_config_packed[1]) == 0);
}

static void later_join_and_removal(void)
{
    reset_fixture(4, 1000); assert(tracker_store_restore() == 0);
    assert(ping(0, true));
    now_ms = 1101; process_initial();
    uint8_t epoch = tdma_config_epoch;
    now_ms = 1102; assert(ping(1, true)); /* outside initial window */
    tdma_shadow_update((uint32_t)now_ms); tdma_recalculate();
    assert(tdma_config_epoch == epoch && slot(tdma_config_packed[1]) == 0xff);
    now_ms = 3102; assert(ping(1, true));
    tdma_shadow_update((uint32_t)now_ms); tdma_recalculate();
    assert(tdma_config_epoch == epoch); /* ordinary 5 s layout debounce */
    now_ms = 6101; assert(ping(1, true));
    tdma_shadow_update((uint32_t)now_ms); tdma_recalculate();
    assert(tdma_config_epoch == (uint8_t)(epoch + 1));
    assert(tdma_active_mask == (BIT(0) | BIT(1)) && total(tdma_config_packed[1]) == 2);

    reset_fixture(3, 0); assert(tracker_store_restore() == 0);
    assert(ping(0, true) && ping(2, true));
    host_forget_initial_id(2); stored_trackers = 2;
    /* A new tracker reusing the ID is not in the saved-at-boot cohort. */
    stored_trackers = 3; assert(ping(2, true));
    assert(!(tdma_initial_allowed_mask & BIT(2)) && !(tdma_initial_ping_mask & BIT(2)));
    now_ms = 101; process_initial();
    assert(tdma_active_mask == BIT(0) && slot(tdma_config_packed[2]) == 0xff);

    reset_fixture(2, 0); assert(tracker_store_restore() == 0);
    assert(ping(0, true));
    host_forget_initial_id(0); host_forget_initial_id(1); stored_trackers = 0;
    now_ms = 101; process_initial();
    assert(!tdma_active_mask && !tdma_dynamic_active_count && !tdma_config_epoch);

    /* If the regular owner publishes first, the pending fast window cannot
     * seed another layout or turn the unassigned peer into an active member. */
    reset_fixture(2, 1000); assert(tracker_store_restore() == 0);
    assert(ping(0, true) && tdma_initial_started);
    atomic_set(&tdma_shadow_desired_mask, BIT(1)); /* ordinary confirmed evidence */
    now_ms = 1050; tdma_recalculate();
    assert(tdma_initial_done && !tdma_initial_started && tdma_active_mask == BIT(1));
    epoch = tdma_config_epoch;
    now_ms = 1101;
    assert(!tdma_initial_due((uint32_t)now_ms));
    tdma_initial_process((uint32_t)now_ms);
    assert(tdma_config_epoch == epoch && tdma_active_mask == BIT(1));
}

static void restore_and_empty_roster(void)
{
    reset_fixture(3, 25); restore_error = -EIO;
    assert(tracker_store_restore() == -EIO && !tracker_store_ready);
    assert(!tdma_initial_started && !tdma_initial_allowed_mask);
    now_ms = 1025; restore_error = 0;
    assert(tracker_store_restore() == 0 && !tdma_initial_done);
    assert(tdma_initial_allowed_mask == 7 && !tdma_dynamic_active_count);
    assert(tracker_store_restore() == 0 && tdma_initial_allowed_mask == 7);
    reset_fixture(0, 500);
    assert(tracker_store_restore() == 0 && tracker_store_ready && tdma_initial_done);
    stored_trackers = 1; assert(ping(0, true)); /* paired after cold restore */
    assert(!tdma_initial_started && !tdma_initial_ping_mask && !tdma_config_epoch);
    tdma_shadow_update((uint32_t)now_ms); tdma_recalculate();
    now_ms += TDMA_SHADOW_JOIN_CONFIRM_MS; assert(ping(0, true));
    tdma_shadow_update((uint32_t)now_ms); tdma_recalculate();
    assert(tdma_active_mask == BIT(0) && tdma_config_epoch != 0);
}

int main(void)
{
    all_rosters(0);
    all_rosters((uint64_t)UINT32_MAX - 50U);
    eligibility_and_stale();
    later_join_and_removal();
    restore_and_empty_roster();
    puts("Receiver initial cohort cases passed");
    return 0;
}
