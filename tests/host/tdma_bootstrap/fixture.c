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
#if defined(CONFIG_TDMA_DIAGNOSTICS)
struct k_spinlock { bool held; };
typedef unsigned int k_spinlock_key_t;
static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock)
{
    k_spinlock_key_t key = irq_lock();
    assert(!lock->held);
    lock->held = true;
    return key;
}
static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
    assert(lock->held);
    lock->held = false;
    irq_unlock(key);
}
#endif
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
/* 32768 Hz kernel ticks, as on the receiver. */
static int64_t k_uptime_ticks(void) { return (int64_t)now_ms * 32768 / 1000; }
static uint32_t k_ticks_to_ms_ceil32(uint32_t ticks)
{
	return (uint32_t)(((uint64_t)ticks * 1000 + 32767) / 32768);
}
static uint32_t k_ms_to_ticks_ceil32(uint32_t ms) { return (uint32_t)(((uint64_t)ms * 32768 + 999) / 1000); }
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
static bool tracker_ota, channel_busy, abort_pending;
static bool esb_pairing, esb_clearing, esb_radio_fault;
static bool esb_ota_relay_is_active(void) { return tracker_ota; }
static bool esb_channel_is_busy(void) { return channel_busy; }
static bool esb_ota_relay_abort_pending(uint8_t id) { (void)id; return abort_pending; }
static bool self_ota;
static bool receiver_ota_is_active(void) { return self_ota; }
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
    relay_user_mode = RELAY_MODE_AUTO;
    relay_coord_set_enabled(&relay_coord, relay_user_mode != RELAY_MODE_OFF);
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
    relay_query_until = 0; relay_query_protected_mask = 0;
    relay_query_recovery_started_ms = 0; relay_query_recovering = relay_query_suspending = false;
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

/* Staged layout activation (relay_protocol.h): with every affected tracker
 * relay-capable, a layout change is announced and applied at its activation
 * tick; otherwise, or when forced, it is published immediately. */
static uint32_t ticks_at(uint64_t ms) { return (uint32_t)(ms * 32768 / 1000); }

static void two_member_layout(uint8_t count, bool capable_joiner)
{
    reset_fixture(count, 0); assert(tracker_store_restore() == 0);
    relay_coord_init(&relay_coord);
    for (uint8_t id = 0; id < count; ++id) {
        if (id < 2 || capable_joiner) relay_coord_note_caps(&relay_coord, id, RELAY_INFO_CAPS);
    }
    assert(ping(0, true) && ping(1, true)); service();
    now_ms = 101; service();
    assert(tdma_active_mask == 3 && !tdma_staged.valid); /* from empty: immediate */
    now_ms = 2000; assert(ping(0, true) && ping(1, true) && ping(2, true)); service();
    now_ms = 2101; service();
}

static void staged_publication(void)
{
    two_member_layout(3, true);
    uint8_t old_epoch = epoch_of(tracker_slots[0].tdma_config_packed);
    uint32_t old0 = tracker_slots[0].tdma_config_packed;
    assert(tdma_staged.valid && tdma_active_mask == 3); /* announced, not applied */
    assert(tdma_staged.p.new_mask == 7 && tdma_staged.p.epoch == (uint8_t)(old_epoch + 1));
    assert(tdma_staged.activation_ticks == ticks_at(2101) + k_ms_to_ticks_ceil32(RELAY_LAYOUT_ACTIVATION_MS));
    assert(tracker_slots[0].tdma_config_packed == old0 && slot(tracker_slots[2].tdma_config_packed) == 0xff);
    /* Before activation the PONG tuple is the current layout. */
    uint32_t before = tdma_staged.activation_ticks - 1;
    assert(tdma_advertised_config(0, before) == old0);
    assert(slot(tdma_advertised_config(2, before)) == 0xff);
    /* Serialized: nothing else is published while the stage is pending. */
    now_ms = 4000; service();
    assert(tdma_staged.valid && tdma_active_mask == 3);
    /* At the activation tick PONGs already carry the new tuple and wake the
     * stats thread, which then applies it. */
    tdma_membership_wake.count = 0;
    uint32_t at = tdma_staged.activation_ticks;
    uint32_t cfg2 = tdma_advertised_config(2, at);
    assert(slot(cfg2) == 2 && total(cfg2) == 3 && epoch_of(cfg2) == (uint8_t)(old_epoch + 1));
    assert(tdma_membership_wake.count == 1);
    now_ms = 2101 + RELAY_LAYOUT_ACTIVATION_MS + 1; service();
    assert(!tdma_staged.valid && tdma_active_mask == 7);
    assert(tracker_slots[2].tdma_config_packed == cfg2);
    assert(tdma_advertised_config(2, ticks_at(now_ms)) == cfg2);

    /* A tracker without staged-activation firmware forces immediate mode. */
    two_member_layout(3, false);
    assert(!tdma_staged.valid && tdma_active_mask == 7);

    /* A forced recalculation still honors the advertised activation tick. */
    two_member_layout(3, true);
    assert(tdma_staged.valid);
    uint32_t activation = tdma_staged.activation_ticks;
    force_layout();
    assert(tdma_staged.valid && tdma_active_mask == 3);
    assert(tdma_staged.activation_ticks == activation);
    now_ms = 2101 + RELAY_LAYOUT_ACTIVATION_MS + 1;
    service();
    assert(!tdma_staged.valid && tdma_active_mask == 7);
    relay_coord_init(&relay_coord);
    tdma_staged.valid = false;
}

static void runtime_settings_publication(void)
{
	two_member_layout(3, true);
	now_ms = 6000;
	service();
	assert(tdma_active_mask == 7 && !tdma_staged.valid);
	relay_settings = (struct relay_runtime_settings){2, 1, 16, 100, 120};
	relay_settings_generation++;
	relay_coord.enabled = true;
	relay_coord.auto_engaged = true;
	force_layout();
	assert(!tdma_staged.valid && !relay_profile && tdma_active_mask == 7);
	esb_relay_set_mode(RELAY_MODE_FORCE);
	force_layout();
	assert(!tdma_staged.valid && !relay_profile && tdma_active_mask == 3);
	now_ms += TDMA_RECONFIG_MIN_MS;
	assert(ping(0, true) && ping(1, true) && ping(2, true));
	service();
	assert(tdma_staged.valid && !relay_profile && tdma_staged.p.profile == 1);
	assert(tdma_staged.p.new_mask == 3 && tdma_staged.p.geometry.direct == 2);
	uint32_t requested = relay_settings_generation;
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1;
	service();
	assert(relay_profile == 1 && tdma_active_mask == 3 && relay_geometry_current()->direct == 2);
	assert(relay_applied_generation == requested && relay_applied_settings.max_repairs == 1);
	uint32_t frame = relay_timeline_active.frame, start;
	assert(relay_timeline_start(&relay_timeline_active, frame, &start));
	assert(start == tdma_staged.activation_ticks);
	assert(ping(0, true) && ping(1, true) && ping(2, true));
	service();
	relay_settings.max_trackers = 3;
	relay_settings.max_repairs = 5;
	relay_settings.repair_floor = 5;
	relay_settings_generation++;
	force_layout();
	assert(tdma_staged.valid && tdma_staged.p.settings.max_repairs == 5 && tdma_staged.p.geometry.direct == 3);
	assert(relay_applied_settings.max_repairs == 1 && relay_geometry_current()->direct == 2);
	relay_settings.max_repairs = 3;
	relay_settings.repair_floor = 3;
	relay_settings_generation++;
	force_layout();
	assert(tdma_staged.p.settings.max_repairs == 5); /* An in-flight publication is immutable. */
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1;
	service();
	assert(relay_applied_settings.max_repairs == 5 && relay_geometry_current()->direct == 3);
	assert(tdma_staged.valid && tdma_staged.p.settings.max_repairs == 3);
	assert(relay_timeline_active.frame > frame);
	frame = relay_timeline_active.frame;
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1;
	service();
	assert(relay_applied_settings.max_repairs == 3 && relay_applied_generation == relay_settings_generation);
	assert(!tdma_staged.valid && relay_timeline_active.frame > frame);
	/* The new R must seed the first interval, even when occupied time
	 * dominates the cap and the old R would make a shorter frame. */
	relay_applied_settings.max_repairs = 1;
	struct tdma_publication full
		= {.new_mask = 0xffff,
		   .active_count = 16,
		   .profile = 1,
		   .slot_ticks = 16,
		   .epoch = (uint8_t)(relay_epoch + 1),
		   .settings = {16, 3, 16, 100, 120, 3},
		   .settings_generation = relay_settings_generation + 1};
	assert(relay_geometry_make(&full.geometry, 16, 3, 16, 120));
	tdma_staged.activation_ticks = ticks_at(now_ms) + 1000;
	unsigned key = irq_lock();
	tdma_apply_locked(&full, now_ms);
	irq_unlock(key);
	uint32_t next;
	assert(relay_timeline_start(&relay_timeline_active, relay_timeline_active.frame, &start));
	assert(relay_timeline_start(&relay_timeline_active, relay_timeline_active.frame + 1, &next));
	assert(next - start == 324 && relay_applied_settings.max_repairs == 3);
	/* Exercise actual publication across every threshold in both directions.
	 * The old applied queue stays intact while the next publication is staged. */
	for (unsigned target = 100; target <= 120; target += 20) {
		for (unsigned pass = 0; pass < 2; pass++) {
			for (unsigned i = 1; i <= 16; i++) {
				unsigned n = pass ? 17 - i : i;
				struct relay_runtime_settings desired = {16, 5, 16, target, 120, 5, 0};
				struct relay_runtime_settings effective = relay_runtime_for_members(desired, n);
				struct tdma_publication pending
					= {.new_mask = (1U << n) - 1U,
					   .active_count = n,
					   .profile = 1,
					   .slot_ticks = 16,
					   .epoch = (uint8_t)(relay_epoch + 1),
					   .settings = desired,
					   .settings_generation = ++relay_settings_generation};
				assert(relay_geometry_make_control(&pending.geometry, n, effective.max_repairs, 16, 120, 0));
				tdma_staged.p = pending;
				tdma_staged.valid = true;
				unsigned previous = relay_applied_settings.max_repairs;
				assert(tdma_staged.p.settings.max_repairs == 5 && relay_applied_settings.max_repairs == previous);
				tdma_staged.activation_ticks = ticks_at(now_ms) + 1000;
				key = irq_lock();
				tdma_apply_locked(&tdma_staged.p, now_ms);
				irq_unlock(key);
				assert(relay_applied_settings.max_repairs == effective.max_repairs);
				assert(relay_applied_settings.repair_floor == effective.max_repairs);
				assert(relay_timeline_start(&relay_timeline_active, relay_timeline_active.frame, &start));
				assert(relay_timeline_start(&relay_timeline_active, relay_timeline_active.frame + 1, &next));
				assert(
					next - start
					== relay_runtime_period(
						relay_timeline_active.frame,
						n,
						relay_frame_repairs(relay_timeline_active.frame),
						16,
						120,
						relay_applied_settings.control_repeat_mode
					)
				);
				tdma_staged.valid = false;
			}
		}
	}
	puts("PASS runtime settings activation: ranked member cap, immutable pending settings and monotonic frame epochs");
}

/* Phase-2 receiver hang: a live member above max_trackers kept the raw
 * desired mask different from the admitted one, re-arming an expired
 * membership deadline forever, and a pending stage was waited for with zero
 * timeouts. Either made the stats owner spin and starve the console. */
static void member_cap_owner_blocks(void)
{
	two_member_layout(3, true);
	now_ms = 6000;
	service();
	assert(tdma_active_mask == 7 && !tdma_staged.valid);
	relay_settings = (struct relay_runtime_settings){2, 1, 16, 100, 120, 0};
	relay_settings_generation++;
	esb_relay_set_mode(RELAY_MODE_FORCE);
	force_layout();
	assert(!tdma_staged.valid && !relay_profile && tdma_active_mask == 3);
	now_ms += TDMA_RECONFIG_MIN_MS;
	assert(ping(0, true) && ping(1, true) && ping(2, true));
	service();
	assert(tdma_staged.valid && tdma_staged.p.new_mask == 3);
	for (unsigned ms = 0; ms + 10 < RELAY_LAYOUT_ACTIVATION_MS; ms += 10) {
		now_ms += 10;
		assert(ping(0, true) && ping(1, true) && ping(2, true));
		service();
		assert(tdma_staged.valid);
		uint32_t wait = tdma_membership_wait_ms(500);
		assert(wait > 0 && wait <= RELAY_LAYOUT_ACTIVATION_MS - ms);
	}
	now_ms += 20;
	service();
	assert(!tdma_staged.valid && relay_profile == 1 && tdma_active_mask == 3);
	for (unsigned ms = 0; ms < 2000; ms += 10) {
		now_ms += 10;
		assert(ping(0, true) && ping(1, true) && ping(2, true));
		service();
		assert(tdma_active_mask == 3 && !tdma_membership_deadline_armed);
		assert(tdma_membership_wait_ms(500) == 500);
	}
	relay_coord.enabled = false;
	puts("PASS member cap: owner sleeps while staged and with live members above max_trackers");
}

static void member_cap_native_auto(void)
{
	relay_settings = (struct relay_runtime_settings){2, 1, 16, 100, 120, 0};
	relay_settings_generation++;
	for (unsigned capable = 0; capable < 2; capable++) {
		two_member_layout(3, capable);
		now_ms = 6000;
		service();
		assert(tdma_active_mask == 7 && !relay_profile && slot(tracker_slots[2].tdma_config_packed) != 0xff);
		relay_coord.enabled = true;
		relay_coord.auto_engaged = true; /* Simulate pending demand from the first two trackers. */
		service();
		assert(tdma_active_mask == 7 && !relay_profile && !tdma_staged.valid);
	}
	puts("PASS member cap: auto keeps all three direct trackers with a cap of two, including a legacy joiner");
}

static void relay_entry_fences(void)
{
 for (unsigned fence=0; fence<3; fence++) {
  tdma_staged.valid=false;relay_profile=0;
  two_member_layout(2, true);now_ms=6000;service();
  relay_coord.enabled=true;relay_coord.auto_engaged=true;
  relay_control_ready=fence!=0;radio_operation_busy=fence==1;self_ota=fence==2;
  force_layout();assert(!relay_profile && !(tdma_staged.valid && tdma_staged.p.profile));
  relay_control_ready=true;radio_operation_busy=0;self_ota=false;
  force_layout();assert(tdma_staged.valid && tdma_staged.p.profile);
  assert(esb_relay_blocks_ota(0));
  tdma_staged.valid=false;relay_coord.auto_engaged=false;
 }
}

static void automatic_profile_publication(void)
{
	two_member_layout(2, true);
	now_ms = 6000;
	service();
	relay_coord.enabled = true;
	force_layout();
	assert(!relay_profile && !tdma_staged.valid);
	/* A pending cap must not impose the ladder's 15 s delay on a profile switch. */
	tdma_cap_level = 3;
	tdma_published_cap_level = 0;
	tdma_last_reconfig_time = now_ms;
	relay_coord.auto_engaged = true;
	now_ms += TDMA_RECONFIG_MIN_MS;
	service();
	assert(tdma_staged.valid && tdma_staged.p.profile == 1 && !relay_profile);
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1;
	service();
	assert(relay_profile == 1 && !tdma_staged.valid);
	/* Admit a relay join while engaged. Its staged publication retains cap 3. */
	stored_trackers = 3;
	relay_coord_note_caps(&relay_coord, 2, RELAY_INFO_CAPS);
	now_ms += TDMA_RECONFIG_MIN_MS;
	assert(ping(0, true) && ping(1, true) && ping(2, true)); service();
	now_ms += 101; service();
	assert(tdma_staged.valid && tdma_staged.p.new_mask == 7);
	assert(tdma_cap_level == 3 && tdma_staged.p.cap_level == 3);
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1; service();
	assert(tdma_published_cap_level == 3);
	relay_coord.auto_engaged = false; /* recovery decision, permission stays on */
	now_ms += TDMA_RECONFIG_MIN_MS;
	service();
	assert(tdma_staged.valid && !tdma_staged.p.profile && relay_profile == 1);
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1;
	service();
	assert(!relay_profile && !tdma_staged.valid && relay_coord.enabled);
	assert(tdma_cap_level == 3 && tdma_published_cap_level == 3);
	puts("PASS auto profile: permission stays direct, entry and recovery use staged activation");
}

/* Both members keep pinging, so membership never changes. */
static void live(void)
{
	assert(ping(0, true) && ping(1, true));
}

/* Hardware run 3 (H3): the session is drawn once per boot, and trackers keep
 * its last control frame across a profile exit, dropping any control frame that does not
 * advance it (pr-tracker esb_relay_control_receive). A second engagement
 * restarted control frame at frame 0, so no control frame was accepted until the 3 s layout grant
 * expired, no repairs flowed, and a direct-blocked member was evicted. */
static void automatic_reentry_continues_frames(void)
{
	two_member_layout(2, true);
	now_ms = 6000;
	service();
	relay_coord.enabled = true;
	relay_timeline_clear(&relay_timeline_active); /* first engagement since boot */
	relay_coord.auto_engaged = true;
	now_ms += TDMA_RECONFIG_MIN_MS; live(); service();
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1; live(); service();
	assert(relay_profile == 1 && relay_timeline_active.frame == 0);
	uint32_t session = relay_session, start;
	/* The control frame thread announces each frame before it sends that frame's control frame. */
	for (uint32_t frame = 1; frame <= 600; frame++) {
		assert(relay_timeline_start(&relay_timeline_active, frame, &start));
		relay_frame_announce(frame, start);
	}
	uint32_t last_b = relay_timeline_active.frame;
	relay_coord.auto_engaged = false;
	now_ms += TDMA_RECONFIG_MIN_MS; live(); service();
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1; live(); service();
	assert(!relay_profile && !tdma_staged.valid);
	relay_coord.auto_engaged = true;
	now_ms += TDMA_RECONFIG_MIN_MS; live(); service();
	assert(tdma_staged.valid && tdma_staged.p.profile == 1);
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1; live(); service();
	assert(relay_profile == 1 && relay_session == session);
	/* The first control frame of the new engagement advances past the tracker's last. */
	assert((int32_t)(relay_timeline_active.frame - last_b) > 0);
	assert(relay_timeline_start(&relay_timeline_active, relay_timeline_active.frame, &start));
	assert(start == tdma_staged.activation_ticks);
	relay_coord.auto_engaged = false;
	relay_coord.enabled = false;
	puts("PASS auto re-entry: control frames continue past the previous engagement");
}

/* Hardware run 8: `relay off` cleared the permission that also gated control frame, so
 * control frame stopped while the applied profile 1 still ran for the staged exit.
 * Trackers' control frame-derived direct anchors expire ~25 ms after the last control frame, so the
 * fleet lost TX admission (Total TPS 0) until the exit activated. */
static void permission_off_keeps_control_through_exit(void)
{
	two_member_layout(2, true);
	now_ms = 6000;
	service();
	esb_relay_set_mode(RELAY_MODE_AUTO);
	relay_coord.auto_engaged = true;
	now_ms += TDMA_RECONFIG_MIN_MS; live(); service();
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1; live(); service();
	assert(relay_profile == 1 && relay_control_eligible());
	for (unsigned reenable = 0; reenable < 2; reenable++) {
		esb_relay_set_mode(RELAY_MODE_OFF);
		assert(!relay_coord.enabled && !relay_coord.auto_engaged);
		assert(!relay_coord_ready(&relay_coord, (uint16_t)tdma_active_mask)); /* future entry stays gated */
		assert(relay_control_eligible());
		now_ms += TDMA_RECONFIG_MIN_MS; live(); service();
		assert(tdma_staged.valid && !tdma_staged.p.profile && relay_profile == 1);
		uint64_t activation = now_ms + RELAY_LAYOUT_ACTIVATION_MS;
		for (unsigned step = 0; now_ms + 10 < activation; step++) {
			/* Permission returning mid-exit is not an engagement: the exit stays staged. */
			if (reenable && step == 100) {
				esb_relay_set_mode(RELAY_MODE_AUTO);
			}
			now_ms += 10; live(); service();
			assert(tdma_staged.valid && relay_profile == 1 && relay_control_eligible());
		}
		now_ms = activation + 1; live(); service();
		assert(!tdma_staged.valid && !relay_profile && !relay_control_eligible());
		assert(relay_coord.enabled == (reenable != 0));
		if (!reenable) {
			esb_relay_set_mode(RELAY_MODE_AUTO);
			relay_coord.auto_engaged = true;
			now_ms += TDMA_RECONFIG_MIN_MS; live(); service();
			assert(tdma_staged.valid && tdma_staged.p.profile == 1 && !relay_control_eligible());
			now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1; live(); service();
			assert(relay_profile == 1 && relay_control_eligible());
		}
	}
	relay_coord.auto_engaged = false;
	relay_coord.enabled = false;
	puts("PASS permission off: control frame maintains applied profile 1 until the staged exit activates");
}

/* `relay off` and pairing while profile 1 is applied: control frame keeps running until a
 * staged exit activates; the exit re-anchors the native sequence. */
static void relay_off_and_pairing_keep_b(void)
{
	two_member_layout(2, true);
	now_ms = 6000;
	service();
	esb_relay_set_mode(RELAY_MODE_AUTO);
	relay_coord.auto_engaged = true;
	now_ms += TDMA_RECONFIG_MIN_MS; live(); service();
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1; live(); service();
	assert(relay_profile == 1 && relay_control_eligible());
	/* Pairing pauses automatic decisions only; the private layout keeps its control frame. */
	esb_pairing = true;
	for (unsigned step = 0; step < 50; step++) {
		now_ms += 100; live(); service();
		assert(relay_profile == 1 && !tdma_staged.valid && relay_control_eligible());
	}
	esb_pairing = false;
	/* Native base from before entry; the tracker keeps counting in profile 1. */
	tracker_slots[0].sequence_initialized = tracker_slots[1].sequence_initialized = true;
	tracker_slots[0].last_packet_sequence = 10;
	esb_relay_set_mode(RELAY_MODE_OFF);
	assert(!relay_coord_ready(&relay_coord, (uint16_t)tdma_active_mask));
	assert(relay_control_eligible());
	now_ms += TDMA_RECONFIG_MIN_MS; live(); service();
	assert(tdma_staged.valid && !tdma_staged.p.profile && relay_profile == 1);
	uint64_t activation = now_ms + RELAY_LAYOUT_ACTIVATION_MS;
	while (now_ms + 10 < activation) {
		now_ms += 10; live(); service();
		assert(tdma_staged.valid && relay_profile == 1 && relay_control_eligible());
	}
	now_ms = activation + 1; live(); service();
	assert(!tdma_staged.valid && !relay_profile && !relay_control_eligible());
	/* 4091 packets later the sequence sits 5 behind the old base; without a
	 * re-anchor this and the next 58 packets would be dropped as old. */
	assert(!tracker_slots[0].sequence_initialized && !tracker_slots[1].sequence_initialized);
	assert(check_packet_sequence(0, 5) == 0 && check_packet_sequence(0, 6) == 0);
	esb_relay_set_mode(RELAY_MODE_AUTO);
	relay_coord.auto_engaged = false;
	relay_coord.enabled = false;
	puts("PASS relay off and pairing: control frame maintains applied profile 1, exit re-anchors native sequence");
}

/* QUERY_INFO needs a reliable 66-byte reply, which no private slot fits: once
 * the tracker confirms the command, the receiver stages profile 0 for the
 * hold, then automatic entry resumes. */
static void query_info_holds_profile0(void)
{
	two_member_layout(2, true);
	now_ms = 6000;
	service();
	esb_relay_set_mode(RELAY_MODE_AUTO);
	relay_coord.auto_engaged = true;
	now_ms += TDMA_RECONFIG_MIN_MS; live(); service();
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1; live(); service();
	assert(relay_profile == 1);
	/* The tracker's PING echoes QUERY_INFO, here delivered while profile 1 runs. */
	memset(&rx_payload, 0, sizeof(rx_payload));
	rx_payload.length = ESB_PING_LEN; rx_payload.data[0] = ESB_PING_TYPE;
	rx_payload.data[1] = 0; rx_payload.pipe = 1;
	rx_payload.data[2] = (uint8_t)(tracker_slots[0].last_ping_counter + 1); /* next PING */
	rx_payload.data[7] = ESB_PONG_FLAG_OTA_QUERY_INFO;
	tracker_slots[0].tracker_remote_command = ESB_PONG_FLAG_OTA_QUERY_INFO;
	tracker_slots[0].query_advertised = true;
	tracker_slots[0].query_advertised_ticks = k_uptime_ticks() - 1;
	rx_payload.data[ESB_PING_LEN - 1] = crc8_ccitt(0x07, rx_payload.data, ESB_PING_LEN - 1);
	assert(host_validated_ping());
	assert(relay_query_until == (int64_t)now_ms + 15000);
	now_ms += TDMA_RECONFIG_MIN_MS; live(); service();
	assert(tdma_staged.valid && !tdma_staged.p.profile && relay_control_eligible());
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1; live(); service();
	assert(!relay_profile && !tdma_staged.valid);
	while (now_ms + 500 < (uint64_t)relay_query_until) {
		now_ms += 500; live(); service();
		assert(!relay_profile && !(tdma_staged.valid && tdma_staged.p.profile));
	}
	now_ms += TDMA_RECONFIG_MIN_MS; live(); service();
	assert(tdma_staged.valid && tdma_staged.p.profile == 1);
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1; live(); service();
	assert(relay_profile == 1);
	relay_coord.auto_engaged = false;
	relay_coord.enabled = false;
	relay_query_until = 0;
	puts("PASS query info: profile 0 held for the reply, then automatic entry resumes");
}

/* These cases run the production PING, membership merge and staged layout
 * functions with logical time. They do not model radio delivery. */
static void query_case_ping(uint8_t id, uint8_t flag)
{
	if (flag == ESB_PONG_FLAG_OTA_QUERY_INFO) {
		tracker_slots[id].tracker_remote_command = flag;
		tracker_slots[id].query_advertised = true;
		tracker_slots[id].query_advertised_ticks = k_uptime_ticks() - 1;
	}
	memset(&rx_payload, 0, sizeof(rx_payload));
	rx_payload.length = ESB_PING_LEN;
	rx_payload.data[0] = ESB_PING_TYPE;
	rx_payload.data[1] = id;
	rx_payload.data[2] = (uint8_t)(tracker_slots[id].last_ping_counter + 1);
	rx_payload.data[7] = flag;
	rx_payload.pipe = 1 + id % 7;
	rx_payload.data[ESB_PING_LEN - 1] = crc8_ccitt(0x07, rx_payload.data, ESB_PING_LEN - 1);
	assert(host_validated_ping());
}

static void query_case_live_until(uint64_t end)
{
	assert(end >= now_ms);
	while (now_ms < end) {
		now_ms = MIN(now_ms + 100, end);
		query_case_ping(0, ESB_PONG_FLAG_NORMAL);
		query_case_ping(1, ESB_PONG_FLAG_NORMAL);
		service();
	}
}

static void query_case_setup(uint64_t base, bool engage)
{
	relay_profile = 0;
	tdma_staged.valid = false;
	relay_coord_init(&relay_coord);
	relay_settings = (struct relay_runtime_settings){16, 5, 16, 120, 120, 5, 0};
	relay_settings_generation++;
	reset_fixture(5, base);
	assert(tracker_store_restore() == 0);
	for (uint8_t id = 0; id < 4; ++id) {
		relay_coord_note_caps(&relay_coord, id, RELAY_INFO_CAPS);
		if (id < 3) {
			query_case_ping(id, ESB_PONG_FLAG_NORMAL);
		}
	}
	service();
	now_ms = base + 101; service();
	assert(tdma_active_mask == 7);
	esb_relay_set_mode(RELAY_MODE_AUTO);
	if (engage) {
		relay_coord.auto_engaged = true;
		now_ms = base + 2000;
		for (uint8_t id = 0; id < 3; ++id) query_case_ping(id, ESB_PONG_FLAG_NORMAL);
		service();
		assert(tdma_staged.valid && tdma_staged.p.profile == 1);
		now_ms = base + 2000 + RELAY_LAYOUT_ACTIVATION_MS + 1;
		for (uint8_t id = 0; id < 3; ++id) query_case_ping(id, ESB_PONG_FLAG_NORMAL);
		service();
		assert(relay_profile == 1 && !tdma_staged.valid && tdma_active_mask == 7);
	} else {
		now_ms = base + 2000 + RELAY_LAYOUT_ACTIVATION_MS + 1;
		for (uint8_t id = 0; id < 3; ++id) query_case_ping(id, ESB_PONG_FLAG_NORMAL);
		service();
		assert(!relay_profile && !tdma_staged.valid);
	}
}

static void relay_mode_service(void)
{
	for (uint8_t id = 0; id < 3; ++id) {
		query_case_ping(id, ESB_PONG_FLAG_NORMAL);
	}
	service();
}

static void relay_mode_transitions(void)
{
	query_case_setup(0, false);
	assert(relay_user_mode == RELAY_MODE_AUTO && relay_coord.enabled);
	assert(!relay_coord.auto_engaged && !relay_profile && !tdma_staged.valid);

	atomic_set(&radio_operation_busy, 1);
	esb_relay_set_mode(RELAY_MODE_FORCE);
	relay_mode_service();
	assert(!relay_profile && !tdma_staged.valid);
	atomic_set(&radio_operation_busy, 0);
	tracker_ota = true;
	relay_mode_service();
	assert(!relay_profile && !tdma_staged.valid);
	tracker_ota = false;
	self_ota = true;
	relay_mode_service();
	assert(!relay_profile && !tdma_staged.valid);
	self_ota = false;
	abort_pending = true;
	relay_mode_service();
	assert(!relay_profile && !tdma_staged.valid);
	abort_pending = false;
	relay_coord_note_caps(&relay_coord, 2, 0);
	relay_mode_service();
	assert(!relay_profile && !tdma_staged.valid);
	relay_coord_note_caps(&relay_coord, 2, RELAY_INFO_CAPS);
	relay_mode_service();
	assert(!relay_profile && tdma_staged.valid && tdma_staged.p.profile);
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1U;
	relay_mode_service();
	assert(relay_profile && !tdma_staged.valid && relay_control_eligible());

	/* Clean quality can withdraw auto demand while forced relay stays applied. */
	for (unsigned tick = 0; tick < 8; ++tick) {
		now_ms += RC_WINDOW_MS;
		live();
		query_case_ping(2, ESB_PONG_FLAG_NORMAL);
		for (uint8_t id = 0; id < 3; ++id) {
			for (unsigned n = 0; n < 60; ++n) {
				relay_coord_note_frame(&relay_coord, id, RC_DIRECT, RC_SEQ_NORMAL, 0, 60, now_ms);
			}
		}
		relay_coord_tick(&relay_coord, tdma_active_mask, now_ms);
		relay_coord_auto_update(&relay_coord, tdma_active_mask, now_ms, true, true);
		service();
		assert(relay_profile && !tdma_staged.valid);
	}
	assert(!relay_coord.auto_engaged);

	self_ota = true;
	relay_mode_service();
	assert(relay_profile && tdma_staged.valid && !tdma_staged.p.profile);
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1U;
	relay_mode_service();
	assert(relay_user_mode == RELAY_MODE_FORCE && !relay_profile && !tdma_staged.valid);
	self_ota = false;
	relay_mode_service();
	assert(tdma_staged.valid && tdma_staged.p.profile);
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1U;
	relay_mode_service();
	assert(relay_profile && !tdma_staged.valid);

	esb_relay_set_mode(RELAY_MODE_AUTO);
	service();
	assert(relay_profile && tdma_staged.valid && !tdma_staged.p.profile && relay_control_eligible());
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1U;
	relay_mode_service();
	assert(!relay_profile && !tdma_staged.valid && relay_coord.enabled);

	esb_relay_set_mode(RELAY_MODE_FORCE);
	service();
	assert(tdma_staged.valid && tdma_staged.p.profile);
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1U;
	relay_mode_service();
	assert(relay_profile && !tdma_staged.valid);
	relay_coord.auto_engaged = true; /* Auto demand has returned during force. */
	esb_relay_set_mode(RELAY_MODE_AUTO);
	service();
	assert(relay_profile && !tdma_staged.valid);
	esb_relay_set_mode(RELAY_MODE_OFF);
	service();
	assert(!relay_coord.enabled && !relay_coord.auto_engaged);
	assert(relay_profile && tdma_staged.valid && !tdma_staged.p.profile && relay_control_eligible());
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1U;
	relay_mode_service();
	assert(!relay_profile && !tdma_staged.valid && !relay_control_eligible());
	puts("PASS relay modes: auto default, force holds, auto follows demand, off stages exit, OTA and channel exclusion");
}

static void dense_relay_exit_announcement_lead(uint16_t cap)
{
	relay_coord_init(&relay_coord);
	reset_fixture(RELAY_MAX_TRACKERS, 0);
	relay_settings = (struct relay_runtime_settings){16, 1, 16, cap, cap, 1, 0};
	relay_settings_generation++;
	assert(tracker_store_restore() == 0);
	for (uint8_t id = 0; id < RELAY_MAX_TRACKERS; ++id) {
		relay_coord_note_caps(&relay_coord, id, RELAY_INFO_CAPS);
		query_case_ping(id, ESB_PONG_FLAG_NORMAL);
	}
	service();
	now_ms = 101;
	service();
	assert(tdma_active_mask == UINT16_MAX && !relay_profile);

	now_ms = 2000;
	esb_relay_set_mode(RELAY_MODE_FORCE);
	service();
	assert(tdma_staged.valid && tdma_staged.p.profile);
	assert(tdma_staged.activation_ticks == ticks_at(now_ms) + k_ms_to_ticks_ceil32(RELAY_LAYOUT_ACTIVATION_MS));

	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1U;
	for (uint8_t id = 0; id < RELAY_MAX_TRACKERS; ++id) {
		query_case_ping(id, ESB_PONG_FLAG_NORMAL);
	}
	service();
	assert(relay_profile && relay_applied_settings.repair_floor == 1);

	/* Seven R1 frames and one R2 maintenance frame per opportunity. */
	uint32_t round_ticks = RELAY_MAX_TRACKERS * (
		7U * relay_runtime_period(cap - 1U, 16, 1, 16, cap, 0)
		+ relay_runtime_period(cap - 1U, 16, 2, 16, cap, 0));
	uint32_t lead = relay_stage_lead_ticks(&relay_applied_settings, RELAY_MAX_TRACKERS);
	assert(lead > 3U * round_ticks + k_ms_to_ticks_ceil32(150));
	esb_relay_set_mode(RELAY_MODE_OFF);
	service();
	assert(tdma_staged.valid && !tdma_staged.p.profile);
	assert(tdma_staged.activation_ticks == ticks_at(now_ms) + lead);
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1U;
	service();
	assert(relay_profile && relay_control_eligible() && tdma_staged.valid);
	now_ms += k_ticks_to_ms_ceil32(lead) - RELAY_LAYOUT_ACTIVATION_MS;
	service();
	assert(!relay_profile && !tdma_staged.valid);
	printf("PASS T16/R1 at %u TPS: three maintenance rounds fit the %u ms staged lead\n",
		cap, k_ticks_to_ms_ceil32(lead));
}

static void relay_admission_requires_a_representable_exit(void)
{
	query_case_setup(0, false);
	relay_settings.tps_min = relay_settings.tps_max = 1;
	relay_settings_generation++;
	relay_coord.auto_engaged = true;
	force_layout();
	assert(!relay_profile && !tdma_staged.valid);
	esb_relay_set_mode(RELAY_MODE_FORCE);
	force_layout();
	assert(!relay_profile && !tdma_staged.valid);
	puts("PASS relay admission: a layout whose exit exceeds the PONG delta stays native");
}

static void legacy_join_waits_for_relay_exit(void)
{
	for (unsigned cap = 2; cap <= 3; cap++) {
		reset_fixture(3, 0);
		assert(tracker_store_restore() == 0);
		relay_coord_init(&relay_coord);
		relay_settings = (struct relay_runtime_settings){cap, 1, 16, 100, 120, 0};
		relay_settings_generation++;
		relay_coord_note_caps(&relay_coord, 0, RELAY_INFO_CAPS);
		relay_coord_note_caps(&relay_coord, 2, RELAY_INFO_CAPS);
		assert(ping(0, true) && ping(2, true));
		service();
		now_ms = 101;
		service();
		const uint32_t v5_mask = BIT(0) | BIT(2);
		assert(tdma_active_mask == v5_mask && !tdma_staged.valid);
		relay_coord.enabled = true;
		relay_coord.auto_engaged = true;
		now_ms += TDMA_RECONFIG_MIN_MS;
		service();
		assert(tdma_staged.valid && tdma_staged.p.profile);
		now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1;
		service();
		assert(relay_profile && tdma_active_mask == v5_mask);

		now_ms += TDMA_RECONFIG_MIN_MS;
		assert(ping(0, true) && ping(1, true) && ping(2, true));
		service();
		now_ms += TDMA_MEMBERSHIP_COHORT_MS;
		service();
		assert(tdma_staged.valid && !tdma_staged.p.profile && tdma_staged.p.new_mask == 7);
		assert(relay_profile && relay_control_eligible() && tdma_active_mask == v5_mask);
		uint32_t activation = tdma_staged.activation_ticks;
		assert(slot(tdma_advertised_config(1, activation - 1)) == 0xff);
		assert(slot(tdma_advertised_config(2, activation - 1)) == 1);
		uint32_t v5_config = tdma_advertised_config(2, activation);
		uint32_t legacy_config = tdma_advertised_config(1, activation);
		assert(slot(v5_config) == 2 && slot(legacy_config) == 1);
		assert(total(v5_config) == 3 && total(legacy_config) == 3);
		assert(epoch_of(v5_config) == epoch_of(legacy_config));
		now_ms += k_ticks_to_ms_ceil32(activation - ticks_at(now_ms));
		service();
		assert(!tdma_staged.valid && !relay_profile && !relay_control_eligible());
		assert(tdma_active_mask == 7 && tdma_config_epoch == epoch_of(v5_config));
		assert(tracker_slots[1].tdma_config_packed == legacy_config);
		assert(tracker_slots[2].tdma_config_packed == v5_config);
	}
	puts("PASS legacy join: lower ID remains unassigned until the announced native roster activates");
}

/* Last modeled repair before the query-triggered native activation. */
#define QUERY_CASE_LAST_REPAIR_MS (6000U + RELAY_LAYOUT_ACTIVATION_MS - 10U)

static void query_case_begin(uint64_t base)
{
	now_ms = base + 6000;
	query_case_ping(0, ESB_PONG_FLAG_OTA_QUERY_INFO);
	service();
	assert(tdma_staged.valid && !tdma_staged.p.profile && relay_profile == 1);
	/* Accepted repair just before the staged native activation: this invokes
	 * the production membership side effect, not a simulated RF decoder. */
	now_ms = base + QUERY_CASE_LAST_REPAIR_MS;
	assert(relay_profile && !tdma_staged_due_locked((uint32_t)k_uptime_ticks()));
	tdma_shadow_observe(2, TDMA_SHADOW_EVIDENCE_DATA);
	now_ms = base + 6000 + RELAY_LAYOUT_ACTIVATION_MS + 1;
	query_case_ping(0, ESB_PONG_FLAG_NORMAL);
	query_case_ping(1, ESB_PONG_FLAG_NORMAL);
	service();
	assert(!relay_profile && tdma_active_mask == 7);
}

static void query_membership_recovery(uint64_t base, bool repeated)
{
	query_case_setup(base, true);
	query_case_begin(base);
	if (repeated) {
		query_case_live_until(base + 12000);
		query_case_ping(1, ESB_PONG_FLAG_OTA_QUERY_INFO);
		service();
		assert(relay_query_until == (int64_t)(base + 27000));
	}
	/* A newcomer was not in the query roster and must still expire normally. */
	query_case_live_until(base + 13000);
	query_case_ping(3, ESB_PONG_FLAG_NORMAL);
	service();
	uint32_t child_seen = (uint32_t)(base + QUERY_CASE_LAST_REPAIR_MS);
	if (!repeated) {
		/* An intermittent genuine direct observation during the pause does
		 * not consume the opportunity to recover after relay resumes. */
		query_case_live_until(base + 17000);
		query_case_ping(2, ESB_PONG_FLAG_NORMAL);
		service();
		child_seen = (uint32_t)now_ms;
	}
	query_case_live_until(base + 25100);
	assert(((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)) != 0);
	assert((uint32_t)atomic_get(&tdma_shadow_last_seen_ms[2]) == child_seen);
	assert(!((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(4)));
	if (repeated) {
		assert(!relay_profile && now_ms < (uint64_t)relay_query_until);
	}
	query_case_live_until(base + 32500);
	assert(relay_profile == 1);
	assert(((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)) != 0);
	assert(!((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(3)));
	assert((uint32_t)atomic_get(&tdma_shadow_last_seen_ms[2]) == child_seen);

	/* A real direct PING after activation ends the special lease for this
	 * member; its unchanged 15 s inactivity rule is measured from this PING. */
	query_case_ping(2, ESB_PONG_FLAG_NORMAL);
	service();
	uint64_t resumed_observation = now_ms;
	assert((uint32_t)atomic_get(&tdma_shadow_last_seen_ms[2]) == (uint32_t)now_ms);
	query_case_live_until(resumed_observation + TDMA_SHADOW_LEAVE_GRACE_MS - 1);
	assert(((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)) != 0);
	assert(!(tdma_active_mask & BIT(3)));
	query_case_live_until(resumed_observation + TDMA_SHADOW_LEAVE_GRACE_MS);
	assert(!((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)));
}

static void query_serialized_entry(void)
{
	query_case_setup(0, true);
	query_case_begin(0);
	query_case_live_until(21000);
	assert(tdma_staged.valid && tdma_staged.p.profile && !relay_profile);
	query_case_live_until(24000);
	query_case_ping(1, ESB_PONG_FLAG_OTA_QUERY_INFO);
	service();
	assert(relay_query_until == 39000);
	/* This immutable pre-query entry still activates, then stages another
	 * exit. It is not the post-hold re-entry. */
	query_case_live_until(24501);
	assert(relay_profile && tdma_staged.valid && !tdma_staged.p.profile);
	query_case_live_until(28002);
	assert(!relay_profile);
	query_case_live_until(40000);
	assert(((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)) != 0);
	assert(!relay_profile && tdma_staged.valid && tdma_staged.p.profile);
	query_case_live_until(42600);
	assert(relay_profile && ((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)));
}

static void query_forget_and_inability(void)
{
	query_case_setup(0, true);
	now_ms = 6000;
	query_case_ping(0, ESB_PONG_FLAG_OTA_QUERY_INFO);
	service();
	host_forget_id(2);
	now_ms++;
	query_case_ping(1, ESB_PONG_FLAG_OTA_QUERY_INFO);
	service();
	assert(!((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)));
	query_case_live_until(15000);
	assert(!(tdma_active_mask & BIT(2)));

	/* Reuse a forgotten slot while the old published layout still contains
	 * its bit. A repeated query cannot grant the new session the old lease. */
	query_case_setup(0, true);
	now_ms = 6000;
	query_case_ping(0, ESB_PONG_FLAG_OTA_QUERY_INFO);
	service();
	host_forget_id(2);
	now_ms = 6001;
	query_case_ping(2, ESB_PONG_FLAG_NORMAL);
	service();
	assert(((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)) != 0);
	now_ms = 6002;
	query_case_ping(1, ESB_PONG_FLAG_OTA_QUERY_INFO);
	service();
	query_case_live_until(21001);
	assert(!((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)));
	assert(now_ms < (uint64_t)relay_query_until);

	for (unsigned reason = 0; reason < 4; ++reason) {
		query_case_setup(0, true);
		query_case_begin(0);
		query_case_live_until(12000);
		query_case_ping(1, ESB_PONG_FLAG_OTA_QUERY_INFO);
		service();
		query_case_live_until(13000);
		if (reason == 0) {
			esb_relay_set_mode(RELAY_MODE_OFF);
		} else if (reason == 1) {
			relay_coord.auto_engaged = false;
		} else if (reason == 2) {
			unsigned key = irq_lock();
			relay_coord_forget(&relay_coord, 1, (uint32_t)now_ms);
			irq_unlock(key);
		} else {
			relay_settings.max_trackers = 2; /* Auto cannot resume with three live members. */
		}
		service();
		query_case_live_until(QUERY_CASE_LAST_REPAIR_MS + TDMA_SHADOW_LEAVE_GRACE_MS - 1);
		assert(((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)) != 0);
		query_case_live_until(QUERY_CASE_LAST_REPAIR_MS + TDMA_SHADOW_LEAVE_GRACE_MS);
		assert(!((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)));
		assert(now_ms < (uint64_t)relay_query_until);
	}
}

static void query_does_not_revive_expired_or_native_members(void)
{
	query_case_setup(0, true);
	/* The published layout is stale because its owner has not yet merged the
	 * timeout. A query must not revive that already-expired member. */
	now_ms = 2000 + RELAY_LAYOUT_ACTIVATION_MS + 1 + TDMA_SHADOW_LEAVE_GRACE_MS;
	query_case_ping(0, ESB_PONG_FLAG_NORMAL);
	query_case_ping(1, ESB_PONG_FLAG_NORMAL);
	assert(tdma_active_mask & BIT(2));
	query_case_ping(0, ESB_PONG_FLAG_OTA_QUERY_INFO);
	service();
	assert(!((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)));
	assert(!((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(3)));

	query_case_setup(0, false);
	now_ms = 6000;
	query_case_ping(0, ESB_PONG_FLAG_OTA_QUERY_INFO);
	service();
	query_case_live_until(12000);
	query_case_ping(1, ESB_PONG_FLAG_OTA_QUERY_INFO);
	service();
	query_case_live_until(2000 + RELAY_LAYOUT_ACTIVATION_MS + 1 + TDMA_SHADOW_LEAVE_GRACE_MS);
	assert(!((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)));
	assert(!relay_profile && now_ms < (uint64_t)relay_query_until);
}

static void query_dead_after_resume(void)
{
	query_case_setup(0, true);
	query_case_begin(0);
	query_case_live_until(21000);
	assert(tdma_staged.valid && tdma_staged.p.profile);
	query_case_live_until(24500);
	assert(relay_profile);
	uint64_t resumed = relay_query_recovery_started_ms;
	query_case_live_until(resumed + TDMA_SHADOW_LEAVE_GRACE_MS - 1);
	assert(((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)) != 0);
	query_case_live_until(resumed + TDMA_SHADOW_LEAVE_GRACE_MS);
	assert(!((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)));

	/* A new echo at the expiry boundary precedes the owner's merge; it must
	 * not revive an already-expired recovery lease. */
	query_case_setup(0, true);
	query_case_begin(0);
	query_case_live_until(21000);
	query_case_live_until(24500);
	assert(relay_profile);
	query_case_live_until(relay_query_recovery_started_ms + TDMA_SHADOW_LEAVE_GRACE_MS - 1);
	now_ms++;
	query_case_ping(0, ESB_PONG_FLAG_OTA_QUERY_INFO);
	service();
	assert(!((uint32_t)atomic_get(&tdma_shadow_desired_mask) & BIT(2)));
}

/* Reproduce the observed applied profile with only ID 6, without assuming
 * why automatic relay was engaged. All later owner passes use its real
 * coordinator with the selected eligibility, followed by the real merge and
 * layout publication. */
static void query_single_setup(uint64_t base)
{
	relay_profile = 0;
	tdma_staged.valid = false;
	relay_coord_init(&relay_coord);
	relay_settings = (struct relay_runtime_settings){16, 5, 16, 120, 120, 5, 0};
	relay_settings_generation++;
	reset_fixture(7, base);
	assert(tracker_store_restore() == 0);
	relay_coord_note_caps(&relay_coord, 6, RELAY_INFO_CAPS);
	query_case_ping(6, ESB_PONG_FLAG_NORMAL);
	service();
	now_ms = base + 101;
	service();
	assert(tdma_active_mask == BIT(6) && !relay_profile);
	esb_relay_set_mode(RELAY_MODE_AUTO);
	relay_coord.auto_engaged = true;
	now_ms = base + 2000;
	query_case_ping(6, ESB_PONG_FLAG_NORMAL);
	service();
	assert(tdma_staged.valid && tdma_staged.p.profile == 1);
	now_ms = base + 2000 + RELAY_LAYOUT_ACTIVATION_MS + 1;
	query_case_ping(6, ESB_PONG_FLAG_NORMAL);
	service();
	assert(tdma_active_mask == BIT(6) && relay_profile == 1 && !tdma_staged.valid);
}

static void query_single_owner(bool eligible)
{
	unsigned key = irq_lock();
	relay_coord_tick(&relay_coord, (uint16_t)tdma_active_mask, (uint32_t)now_ms);
	relay_coord_auto_update(&relay_coord, (uint16_t)tdma_active_mask,
				(uint32_t)now_ms, relay_profile != 0, eligible);
	irq_unlock(key);
	service();
}

static void query_force_unstageable_reentry_releases_member(void)
{
	query_single_setup(0);
	esb_relay_set_mode(RELAY_MODE_FORCE);
	now_ms = 6000;
	query_case_ping(6, ESB_PONG_FLAG_OTA_QUERY_INFO);
	service();
	assert(relay_query_suspending && relay_query_protected_mask == BIT(6));
	assert(relay_profile && tdma_staged.valid && !tdma_staged.p.profile);
	now_ms += RELAY_LAYOUT_ACTIVATION_MS + 1;
	service();
	assert(!relay_profile && !tdma_staged.valid && tdma_active_mask == BIT(6));

	/* Settings can change while the receiver is in the query's native hold. */
	relay_settings.tps_min = relay_settings.tps_max = 1;
	relay_settings_generation++;
	assert(relay_runtime_validate(&relay_settings, false));
	assert(!relay_roster_eligible((uint32_t)atomic_get(&tdma_shadow_desired_mask)));
	force_layout();
	now_ms = 6000 + TDMA_SHADOW_LEAVE_GRACE_MS - 1;
	service();
	assert(tdma_active_mask == BIT(6) && !relay_profile);
	now_ms++;
	service();
	assert(!(atomic_get(&tdma_shadow_desired_mask) & BIT(6)));
	assert(!tdma_active_mask && !tdma_staged.valid);
	assert(!relay_query_protected_mask && !relay_query_suspending && !relay_query_recovering);
	puts("PASS force query: unstageable re-entry releases the silent member at ordinary expiry");
}

static uint64_t query_single_missed_hold(uint64_t base, bool eligible)
{
	query_single_setup(base);
	now_ms = base + 6000;
	query_case_ping(6, ESB_PONG_FLAG_OTA_QUERY_INFO);
	assert(relay_query_suspending && relay_query_protected_mask == BIT(6));
	assert(!relay_query_recovering && !tdma_staged.valid);
	/* No owner pass occurs during the entire hold, so no native exit is staged. */
	now_ms = base + 6000 + TDMA_SHADOW_LEAVE_GRACE_MS + 501;
	query_single_owner(eligible);
	assert(relay_coord.auto_engaged && relay_profile == 1 && !tdma_staged.valid);
	assert(tdma_active_mask == BIT(6) && relay_query_protected_mask == BIT(6));
	assert(!relay_query_suspending && relay_query_recovering);
	assert(relay_query_recovery_started_ms == (uint32_t)now_ms);
	assert((uint32_t)atomic_get(&tdma_shadow_last_seen_ms[6]) == (uint32_t)(base + 6000));
	return now_ms;
}

static void query_single_bounded_recovery(uint64_t base, bool eligible)
{
	uint64_t resumed = query_single_missed_hold(base, eligible);
	/* Repeated owner passes cannot renew the single recovery window. */
	for (now_ms = resumed + 500; now_ms < resumed + TDMA_SHADOW_LEAVE_GRACE_MS; now_ms += 500) {
		query_single_owner(eligible);
		assert(relay_coord.auto_engaged && relay_profile == 1 && tdma_active_mask == BIT(6));
		assert(relay_query_recovering && !relay_query_suspending);
		assert(relay_query_recovery_started_ms == (uint32_t)resumed);
		assert((uint32_t)atomic_get(&tdma_shadow_last_seen_ms[6]) == (uint32_t)(base + 6000));
	}
	now_ms = resumed + TDMA_SHADOW_LEAVE_GRACE_MS - 1;
	query_single_owner(eligible);
	assert(relay_query_recovering && tdma_active_mask == BIT(6));
	now_ms++;
	query_single_owner(eligible);
	assert(!(atomic_get(&tdma_shadow_desired_mask) & BIT(6)));
	assert(!tdma_active_mask && !relay_profile && !tdma_staged.valid);
	assert(!relay_query_protected_mask && !relay_query_recovering && !relay_query_suspending);
	now_ms += 500;
	query_single_owner(eligible);
	assert(!tdma_active_mask && !relay_profile && !relay_query_recovering);
}

static void query_single_no_query_control(uint64_t base, bool eligible)
{
	query_single_setup(base);
	now_ms = base + 6000;
	query_case_ping(6, ESB_PONG_FLAG_NORMAL);
	/* Same outage and owner gap, but no echo and therefore no special lease. */
	now_ms = base + 6000 + TDMA_SHADOW_LEAVE_GRACE_MS + 501;
	query_single_owner(eligible);
	assert(!(atomic_get(&tdma_shadow_desired_mask) & BIT(6)));
	assert(!tdma_active_mask && !relay_profile && !tdma_staged.valid);
	assert(!relay_query_protected_mask && !relay_query_recovering && !relay_query_suspending);
}

static void query_single_pending_native_exit(uint64_t base, bool eligible)
{
	query_single_setup(base);
	now_ms = base + 6000;
	query_case_ping(6, ESB_PONG_FLAG_OTA_QUERY_INFO);
	query_single_owner(eligible);
	assert(relay_profile && tdma_staged.valid && !tdma_staged.p.profile);
	assert(relay_query_suspending && !relay_query_recovering);

	/* The owner returns after the hold, with the old native exit still
	 * pending. Its first merge must not mistake profile 1 for a re-entry. */
	now_ms = base + 6000 + TDMA_SHADOW_LEAVE_GRACE_MS + 1;
	query_single_owner(eligible);
	assert(!relay_profile && tdma_staged.valid && tdma_staged.p.profile);
	assert(relay_query_suspending && !relay_query_recovering);
	assert(tdma_active_mask == BIT(6) && relay_query_protected_mask == BIT(6));
	uint64_t exit_applied = now_ms;
	now_ms = exit_applied + RELAY_LAYOUT_ACTIVATION_MS - 1;
	query_single_owner(eligible);
	assert(!relay_profile && tdma_staged.valid && !relay_query_recovering);
	now_ms = exit_applied + RELAY_LAYOUT_ACTIVATION_MS + 1;
	query_single_owner(eligible);
	assert(relay_profile && !tdma_staged.valid && relay_query_recovering);
	assert(!relay_query_suspending && relay_query_recovery_started_ms == (uint32_t)now_ms);
	uint64_t reentry = now_ms;
	now_ms = reentry + TDMA_SHADOW_LEAVE_GRACE_MS - 1;
	query_single_owner(eligible);
	assert(tdma_active_mask == BIT(6) && relay_query_recovering);
	now_ms++;
	query_single_owner(eligible);
	assert(!tdma_active_mask && !relay_profile && !relay_query_recovering);
}

static void query_single_late_echo(uint64_t base, bool eligible)
{
	uint64_t resumed = query_single_missed_hold(base, eligible);
	now_ms = resumed + TDMA_SHADOW_LEAVE_GRACE_MS;
	/* Another stored tracker sends a real query before the owner's expiry
	 * merge. Its new receipt is legitimate, but must not revive silent ID 6. */
	query_case_ping(0, ESB_PONG_FLAG_OTA_QUERY_INFO);
	assert(!(relay_query_protected_mask & BIT(6)));
	query_single_owner(eligible);
	assert(!(atomic_get(&tdma_shadow_desired_mask) & BIT(6)));
	/* The legitimate new join has its own batching deadline. */
	now_ms += TDMA_MEMBERSHIP_COHORT_MS;
	query_single_owner(eligible);
	assert(!(tdma_active_mask & BIT(6)));
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
    assert(relay_user_mode == RELAY_MODE_AUTO && relay_coord.enabled); /* production boot init */
    all_rosters(0);
    all_rosters((uint64_t)UINT32_MAX - 50U);
    invalid_and_stale_evidence();
    fresh_joins_and_spacing();
    deadline_anchors();
    snapshot_and_reuse();
    empty_and_returning();
    owner_store_lock_delay();
    restore_retry();
    staged_publication();
	relay_entry_fences();
	automatic_profile_publication();
	automatic_reentry_continues_frames();
	permission_off_keeps_control_through_exit();
	relay_off_and_pairing_keep_b();
	query_info_holds_profile0();
	runtime_settings_publication();
	member_cap_owner_blocks();
	member_cap_native_auto();
	relay_mode_transitions();
	relay_admission_requires_a_representable_exit();
	legacy_join_waits_for_relay_exit();
	dense_relay_exit_announcement_lead(100);
	dense_relay_exit_announcement_lead(120);
	query_membership_recovery(0, false);
	query_membership_recovery(0, true);
	query_membership_recovery((uint64_t)UINT32_MAX - 20000U, true);
	query_serialized_entry();
	query_forget_and_inability();
	query_does_not_revive_expired_or_native_members();
	query_dead_after_resume();
	query_force_unstageable_reentry_releases_member();
	const uint64_t query_bases[] = {0, (uint64_t)UINT32_MAX - 10000U};
	for (unsigned b = 0; b < ARRAY_SIZE(query_bases); ++b) {
		for (unsigned eligible = 0; eligible < 2; ++eligible) {
			query_single_no_query_control(query_bases[b], eligible != 0);
			query_single_bounded_recovery(query_bases[b], eligible != 0);
			query_single_pending_native_exit(query_bases[b], eligible != 0);
			query_single_late_echo(query_bases[b], eligible != 0);
		}
	}
	puts("PASS query membership: suspension, serialized re-entry, recovery and expiry");
	puts("PASS staged layout publication: announce, serialize, activate, legacy and forced");
	puts("Receiver membership admission cases passed");
	return 0;
}
