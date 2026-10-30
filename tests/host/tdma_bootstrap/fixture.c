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
#define ARG_UNUSED(x) ((void)(x))
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
static struct fixture_sem tdma_membership_wake;
static unsigned wake_signals;
static void k_sem_give(struct fixture_sem *sem)
{
    assert(!irq_depth);
    sem->count = 1;
    ++wake_signals;
}
static int tracker_store_lock;
static void (*mutex_hook)(void);
static void k_mutex_lock(int *lock, int timeout)
{
    (void)lock; (void)timeout;
    assert(!irq_depth && !mutex_depth);
    ++mutex_depth;
    if (mutex_hook) {
        void (*hook)(void) = mutex_hook;
        mutex_hook = NULL;
        hook();
    }
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
static void (*sync_hook)(void);
static void tdma_sync_stats_reset(void)
{
    assert(!irq_depth);
    if (sync_hook) {
        void (*hook)(void) = sync_hook;
        sync_hook = NULL;
        hook();
    }
}
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
static struct esb_payload { uint8_t data[13], length, pipe; } rx_payload;
static uint8_t crc8_ccitt(uint8_t crc, const uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}
/* DIAGNOSTIC DECLARATIONS */
static uint32_t decision_ready_mask, decision_fence_mask;
static atomic_t test_all_state_valid, test_all_confirmed_mask;
static void (*remote_confirm_cb)(uint8_t, uint8_t);
static uint32_t k_ticks_to_us_floor32(uint32_t ticks) { return ticks; }
/* SDK conversion and console policy leaves; admission cases keep detail off. */
static uint64_t k_ticks_to_us_floor64(uint64_t ticks) { return ticks * 1000000 / 32768; }
static bool esb_get_stats_detailed_enabled(void) { return false; }
static void test_all_invalidate_tracker(uint8_t id, int64_t at) { (void)id; (void)at; }
static uint16_t test_all_wire_tps(void) { return 0; }
static void remote_wom_receive(uint8_t id, uint8_t flag, const uint8_t *data, uint8_t counter)
{ (void)id; (void)flag; (void)data; (void)counter; }
static const char *esb_pong_flag_name(uint8_t flag) { (void)flag; return "command"; }


/* PRODUCTION DECLARATIONS */
/* PRODUCTION FUNCTIONS */

static void reset_fixture(uint8_t count, uint64_t time)
{
    assert(!irq_depth && !mutex_depth && count <= MAX_TRACKERS);
    now_ms = time; restore_error = 0; saved_count = count;
    tdma_active_mask = 0;
    unsigned int key = irq_lock();
    for (unsigned i = 0; i < MAX_TRACKERS; ++i) {
        saved_addr[i] = i < count ? 1000 + i : 0;
        stored_tracker_addr[i] = 0;
        tracker_slots[i].tdma_config_packed = 0;
        tdma_shadow_forget_tracker(i);
    }
    irq_unlock(key);
    stored_trackers = 0; tracker_store_ready = false;
    tdma_config_epoch = tdma_dynamic_active_count = tdma_dynamic_slot_ticks = 0;
    tdma_last_reconfig_time = 0; tdma_last_reconfig_valid = false;
    tdma_membership_deadline_ms = 0; tdma_membership_deadline_armed = false;
    tdma_cap_level = tdma_published_cap_level = 0;
    tdma_loss_reset_windows(); atomic_set(&test_all_enabled, 0);
    atomic_set(&tdma_shadow_desired_changed_at_ms, 0);
    tdma_membership_wake.count = wake_signals = 0;
    sync_hook = NULL; mutex_hook = NULL;
    loss_invalidations = membership_notifications = effective_updates = 0;
    last_notified_mask = 0; last_effective_slots = 0;
    for (unsigned i = 0; i < MAX_TRACKERS; ++i) {
        tracker_slots[i].pipe_mismatch_count = 0;
        tracker_slots[i].ping_counter_initialized = false;
        tracker_slots[i].last_ping_counter = 0;
        tracker_slots[i].last_ping_time = 0;
        tracker_slots[i].last_pong_queued_counter = 0;
        tracker_slots[i].crc_error_count = 0;
#if defined(CONFIG_TDMA_DIAGNOSTICS)
        tracker_diags[i].g_last_ping_rx_time_diff_valid = false;
        tracker_diags[i].g_ping_isr_rx_ticks_valid = false;
        tracker_diags[i].g_bias_ppb_valid = 0;
        tracker_diags[i].g_ping_dirty_streak = 0;
#endif
    }
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

static void service(void)
{
    k_mutex_lock(&tracker_store_lock, K_FOREVER);
    tdma_shadow_update((uint32_t)now_ms);
    tdma_recalculate(false);
    k_mutex_unlock(&tracker_store_lock);
}

static void force_layout(void)
{
    k_mutex_lock(&tracker_store_lock, K_FOREVER);
    tdma_recalculate(true);
    k_mutex_unlock(&tracker_store_lock);
}

static void all_rosters(uint64_t base)
{
    static const uint8_t widths[] = {0, 64, 64, 48, 36, 28, 24, 20, 18,
                                     16, 16, 16, 16, 16, 16, 16, 16};
    for (uint8_t n = 1; n <= MAX_TRACKERS; ++n) {
        reset_fixture(n, base);
        assert(tracker_store_restore() == 0 && tracker_store_ready);
        assert(ping(0, true));
        assert(ping(0, true) && wake_signals == 1);
        assert(tdma_membership_deadline_ms == (uint32_t)(base + 101));
        assert(tdma_membership_wait_ms(500) == 101);
        service();
        assert((uint32_t)atomic_get(&tdma_shadow_desired_mask) == BIT(0));
        assert(!tdma_active_mask && !atomic_get(&tdma_shadow_pending_join_mask));
        for (uint8_t id = 1; id < n; ++id) {
            now_ms = base + ((uint32_t)id * 100U) / (n - 1);
            assert(ping(id, true));
        }
        now_ms = base + 100;
        assert(ping(0, true));
        service();
        assert(tdma_membership_deadline_ms == (uint32_t)(base + 101));
        assert(tdma_membership_wait_ms(500) == 1);
        assert(!tdma_active_mask && !tdma_last_reconfig_valid);
        assert((uint32_t)atomic_get(&tdma_shadow_desired_mask) == BIT(n) - 1U);
        now_ms = base + 101;
        assert(tdma_membership_wait_ms(500) == 0);
        service();
        assert(tdma_dynamic_active_count == n && tdma_active_mask == BIT(n) - 1U);
        assert(!tdma_membership_deadline_armed && !atomic_get(&tdma_shadow_pending_join_mask));
        assert(tdma_last_reconfig_valid && tdma_last_reconfig_time == (int64_t)now_ms);
        assert(last_notified_mask == BIT(n) - 1U && last_effective_slots == n);
        assert(tdma_dynamic_slot_ticks == widths[n]);
        for (uint8_t id = 0; id < n; ++id) {
            uint32_t cfg = tracker_slots[id].tdma_config_packed;
            assert(slot(cfg) == id && total(cfg) == n && width(cfg) == widths[n]);
            assert(epoch_of(cfg) == tdma_config_epoch);
        }
        uint8_t epoch = tdma_config_epoch;
        now_ms += 1000;
        service();
        assert(tdma_config_epoch == epoch && tdma_last_reconfig_time == (int64_t)(base + 101));
    }
}

static void invalid_and_stale_evidence(void)
{
    reset_fixture(2, 0); assert(tracker_store_restore() == 0);
    assert(!ping(0, false));
    assert(!ping(16, true));
    assert(ping(2, true)); /* Valid PING but not a stored ID. */
    memset(&rx_payload, 0, sizeof(rx_payload));
    rx_payload.length = ESB_PING_LEN; rx_payload.data[0] = ESB_PING_TYPE;
    rx_payload.data[1] = 0; rx_payload.pipe = 2;
    rx_payload.data[ESB_PING_LEN - 1] = crc8_ccitt(0x07, rx_payload.data, ESB_PING_LEN - 1);
    rx_payload.length = ESB_PING_LEN - 1;
    assert(!host_validated_ping());
    rx_payload.length = ESB_PING_LEN; rx_payload.data[0] ^= 1;
    assert(!host_validated_ping());
    rx_payload.data[0] = ESB_PING_TYPE;
    assert(host_validated_ping()); /* The rest of the packet path is unchanged. */
    assert(!atomic_get(&tdma_shadow_live_evidence_mask));
    assert(!tdma_membership_deadline_armed && !wake_signals);
    service();
    assert(!tdma_active_mask && !tdma_last_reconfig_valid);

    assert(ping(1, true));
    assert(atomic_get(&tdma_shadow_pending_join_mask) == BIT(1));
    now_ms = TDMA_SHADOW_FRESH_MS;
    service();
    assert(!tdma_active_mask && !atomic_get(&tdma_shadow_live_evidence_mask));
    assert(!tdma_membership_deadline_armed && !atomic_get(&tdma_shadow_pending_join_mask));
    assert(ping(1, true));
    assert(wake_signals == 2 && tdma_membership_deadline_ms == TDMA_SHADOW_FRESH_MS + 101);
    service();
    assert((uint32_t)atomic_get(&tdma_shadow_desired_mask) == BIT(1) && !tdma_active_mask);
    now_ms += 100;
    service();
    assert(!tdma_active_mask && tdma_membership_wait_ms(500) == 1);
    now_ms++;
    service();
    assert(tdma_active_mask == BIT(1) && slot(tracker_slots[1].tdma_config_packed) == 0);
    uint32_t last_seen = (uint32_t)atomic_get(&tdma_shadow_last_seen_ms[1]);
    now_ms++;
    rx_payload.pipe = 1; /* ID 1 belongs on pipe 2. */
    assert(host_validated_ping());
    assert((uint32_t)atomic_get(&tdma_shadow_last_seen_ms[1]) == last_seen);

    reset_fixture(1, 0); assert(tracker_store_restore() == 0);
    assert(ping(0, true)); service();
    assert((uint32_t)atomic_get(&tdma_shadow_desired_mask) == 1 && !tdma_active_mask);
    now_ms = TDMA_SHADOW_FRESH_MS; service();
    assert(!atomic_get(&tdma_shadow_desired_mask) && !tdma_membership_deadline_armed);
    assert(ping(0, true)); service();
    now_ms += 101; service();
    assert(tdma_active_mask == 1);
}

static void fresh_joins_and_spacing(void)
{
    reset_fixture(0, 1600); assert(tracker_store_restore() == 0);
    stored_trackers = 4; /* IDs paired after the empty roster was restored. */
    assert(ping(0, true));
    service();
    assert(tdma_active_mask == 0 && (uint32_t)atomic_get(&tdma_shadow_desired_mask) == BIT(0));
    now_ms = 1700;
    service();
    assert(!tdma_active_mask && tdma_membership_wait_ms(500) == 1);
    now_ms = 1701;
    service();
    assert(tdma_active_mask == BIT(0) && tdma_last_reconfig_time == 1701);

    now_ms = 1702; assert(ping(1, true));
    assert(atomic_get(&tdma_shadow_pending_join_mask) == BIT(1));
    service();
    assert(!atomic_get(&tdma_shadow_pending_join_mask));
    assert((uint32_t)atomic_get(&tdma_shadow_desired_mask) == 3 && tdma_active_mask == 1);
    assert(slot(tracker_slots[1].tdma_config_packed) == 0xff);
    uint32_t first_deadline = tdma_membership_deadline_ms;
    now_ms = 1750; assert(ping(2, true));
    service();
    assert(tdma_membership_deadline_ms == first_deadline && first_deadline == 1803);
    now_ms = 2700;
    service();
    assert(tdma_active_mask == 1 && tdma_membership_wait_ms(500) == 1);
    now_ms = 2701;
    service();
    assert(tdma_active_mask == 7 && tdma_last_reconfig_time == 2701);
    assert(slot(tracker_slots[1].tdma_config_packed) == 1 && slot(tracker_slots[2].tdma_config_packed) == 2);
    uint8_t epoch = tdma_config_epoch;

    /* Evidence after the snapshot starts a new batch even in the same tick. */
    assert(ping(3, true));
    assert(tdma_membership_deadline_ms == 2802);
    service();
    assert(tdma_active_mask == 7 && (uint32_t)atomic_get(&tdma_shadow_desired_mask) == 15);
    now_ms = 3700;
    service();
    assert(tdma_active_mask == 7 && tdma_membership_wait_ms(500) == 1);
    now_ms = 3701;
    service();
    assert(tdma_active_mask == 15 && tdma_config_epoch == (uint8_t)(epoch + 1));
}

static void deadline_anchors(void)
{
    reset_fixture(1, 1600); assert(tracker_store_restore() == 0);
    assert(ping(0, true));
    now_ms = 1800;
    service();
    assert(tdma_active_mask == 1 && tdma_last_reconfig_time == 1800);

    reset_fixture(2, 899); assert(tracker_store_restore() == 0);
    assert(ping(0, true)); service();
    now_ms = 1000; service();
    assert(tdma_last_reconfig_time == 1000);
    now_ms = 1600; assert(ping(1, true)); service();
    assert(tdma_membership_deadline_ms == 1701);
    now_ms = 1999; service();
    assert(tdma_active_mask == 1 && tdma_membership_wait_ms(500) == 1);
    now_ms = 2000; service();
    assert(tdma_active_mask == 3 && tdma_last_reconfig_time == 2000);

    reset_fixture(2, 0); assert(tracker_store_restore() == 0);
    assert(ping(0, true)); service(); force_layout();
    assert(tdma_active_mask == 1 && tdma_last_reconfig_valid && tdma_last_reconfig_time == 0);
    now_ms = 100; assert(ping(1, true)); service();
    now_ms = 999; service();
    assert(tdma_active_mask == 1 && tdma_membership_wait_ms(500) == 1);
    now_ms = 1000; service();
    assert(tdma_active_mask == 3 && tdma_last_reconfig_time == 1000);

    reset_fixture(1, (uint64_t)UINT32_MAX - 100); assert(tracker_store_restore() == 0);
    assert(ping(0, true)); service();
    assert(tdma_membership_deadline_armed && tdma_membership_deadline_ms == 0);
    assert(tdma_membership_wait_ms(500) == 101);
    now_ms = (uint64_t)UINT32_MAX + 1; service();
    assert(tdma_active_mask == 1 && tdma_last_reconfig_time == (int64_t)now_ms);
}

static void inject_before_snapshot(void)
{
    assert(ping(1, true));
}

static void expire_before_snapshot(void)
{
    now_ms += TDMA_SHADOW_FRESH_MS;
}

static void snapshot_and_reuse(void)
{
    reset_fixture(3, 10); assert(tracker_store_restore() == 0);
    assert(ping(0, true)); service();
    now_ms = 111; sync_hook = inject_before_snapshot; service();
    assert(tdma_active_mask == 3 && slot(tracker_slots[1].tdma_config_packed) == 1);
    assert(ping(2, true)); service();
    assert(tdma_active_mask == 3 && (uint32_t)atomic_get(&tdma_shadow_desired_mask) == 7);
    now_ms = 1111; service();
    assert(tdma_active_mask == 7);

    reset_fixture(1, 0); assert(tracker_store_restore() == 0);
    assert(ping(0, true)); service();
    now_ms = 101; sync_hook = expire_before_snapshot; service();
    assert(!tdma_active_mask && !atomic_get(&tdma_shadow_desired_mask));
    assert(!tdma_membership_deadline_armed && !tdma_last_reconfig_valid);
    assert(ping(0, true)); service();
    now_ms += 101; service();
    assert(tdma_active_mask == 1);

    reset_fixture(3, 10); assert(tracker_store_restore() == 0);
    assert(ping(0, true) && ping(1, true) && ping(2, true)); service();
    now_ms = 111; service();
    assert(tdma_active_mask == 7);

    now_ms = 120;
    stored_trackers = 2; host_forget_id(2); force_layout();
    assert(tdma_active_mask == 3 && tdma_last_reconfig_time == 120);
    assert(slot(tracker_slots[2].tdma_config_packed) == 0xff && total(tracker_slots[2].tdma_config_packed) == 2);
    stored_trackers = 3; now_ms = 130;
    assert(ping(2, true));
    stored_trackers = 2; host_forget_id(2);
    assert(!atomic_get(&tdma_shadow_pending_join_mask) && !tdma_membership_deadline_armed);
    stored_trackers = 3; now_ms = 140;
    assert(ping(2, true)); service();
    assert(tdma_membership_deadline_ms == 241);
    now_ms = 1119; service();
    assert(tdma_active_mask == 3 && tdma_membership_wait_ms(500) == 1);
    now_ms = 1120; service();
    assert(tdma_active_mask == 7 && slot(tracker_slots[2].tdma_config_packed) == 2);
}

static void empty_and_returning(void)
{
    reset_fixture(1, 0); assert(tracker_store_restore() == 0);
    assert(ping(0, true)); service();
    now_ms = 101; service();
    uint8_t epoch = tdma_config_epoch;
    now_ms = TDMA_SHADOW_LEAVE_GRACE_MS - 1; service();
    assert(tdma_active_mask == 1 && atomic_get(&tdma_shadow_pending_leave_mask) == 1);
    now_ms = TDMA_SHADOW_LEAVE_GRACE_MS; service();
    assert(!tdma_active_mask && !tdma_dynamic_active_count);
    assert(tdma_config_epoch == epoch && tdma_last_reconfig_time == (int64_t)now_ms);
    for (unsigned i = 0; i < MAX_TRACKERS; ++i) {
        assert(slot(tracker_slots[i].tdma_config_packed) == 0xff && total(tracker_slots[i].tdma_config_packed) == 0);
    }
    now_ms++; service();
    assert(tdma_last_reconfig_time == TDMA_SHADOW_LEAVE_GRACE_MS);
    assert(ping(0, true)); service();
    assert((uint32_t)atomic_get(&tdma_shadow_desired_mask) == 1 && !tdma_active_mask);
    now_ms = TDMA_SHADOW_LEAVE_GRACE_MS + 999; service();
    assert(!tdma_active_mask && tdma_membership_wait_ms(500) == 1);
    now_ms++; service();
    assert(tdma_active_mask == 1 && tdma_config_epoch == (uint8_t)(epoch + 1));
}

static void refresh_while_store_locked(void)
{
    assert(mutex_depth == 1);
    now_ms = 1700;
    assert(ping(0, true));
    now_ms = 1800;
}

static void expire_while_store_locked(void)
{
    assert(mutex_depth == 1);
    now_ms = 15100;
}

static void owner_store_lock_delay(void)
{
    reset_fixture(1, 1600); assert(tracker_store_restore() == 0);
    assert(ping(0, true));
    mutex_hook = refresh_while_store_locked;
    host_early_owner_wake(1600, 2000);
    assert(tdma_active_mask == 1 && tdma_last_reconfig_time == 1800);
    assert((uint32_t)atomic_get(&tdma_shadow_last_seen_ms[0]) == 1700);

    reset_fixture(1, 0); assert(tracker_store_restore() == 0);
    assert(ping(0, true)); service();
    now_ms = 101; service();
    assert(tdma_active_mask == 1);
    now_ms = 14900; mutex_hook = expire_while_store_locked;
    host_early_owner_wake(14900, 15000);
    assert(!tdma_active_mask && tdma_last_reconfig_time == 15100);
}

static void restore_retry(void)
{
    reset_fixture(3, 25); restore_error = -EIO;
    assert(tracker_store_restore() == -EIO && !tracker_store_ready);
    assert(!tdma_membership_deadline_armed);
    now_ms = 1025; restore_error = 0;
    assert(tracker_store_restore() == 0 && tracker_store_ready && stored_trackers == 3);
    assert(tracker_store_restore() == 0 && stored_trackers == 3);
    assert(!tdma_active_mask && !tdma_last_reconfig_valid);
}

int main(int argc, char **argv)
{
    if (argc == 2) {
        assert(!strcmp(argv[1], "owner-store-lock-delay"));
        owner_store_lock_delay();
        puts("Receiver owner lock-delay cases passed");
        return 0;
    }
    assert(argc == 1);
    all_rosters(0);
    all_rosters((uint64_t)UINT32_MAX - 50U);
    invalid_and_stale_evidence();
    fresh_joins_and_spacing();
    deadline_anchors();
    snapshot_and_reuse();
    empty_and_returning();
    owner_store_lock_delay();
    restore_retry();
    puts("Receiver membership admission cases passed");
    return 0;
}
