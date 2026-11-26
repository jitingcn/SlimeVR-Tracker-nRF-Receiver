#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define snprintk snprintf
#include <string.h>
#include <errno.h>
#define MAX_TRACKERS 16
#define BIT(i) (1u << (i))
#define K_FOREVER 0
static void host_log(const char *format, ...) { (void)format; }
#define LOG_INF(...) host_log(__VA_ARGS__)
#define LOG_DBG(...) host_log(__VA_ARGS__)
#define LOG_ERR(...) host_log(__VA_ARGS__)
static void host_warning(const char *format, ...) { (void)format; }
#define LOG_WRN(...) host_warning(__VA_ARGS__)
#define ARG_UNUSED(x) ((void)(x))
#define TPS_CALCULATION_INTERVAL_MS 1000
#define ESB_RF_CHANNEL_DEFAULT 255
#define ESB_RF_CHANNEL_ZERO_ENC 128
#define ESB_PONG_FLAG_NORMAL 0
#define ESB_PONG_FLAG_SET_CHANNEL 10
#define ESB_PONG_FLAG_CLEAR_CHANNEL 11
#define ESB_PONG_FLAG_DATA_COLLECT_ON 34
#define ESB_PONG_FLAG_TEST_MODE_ON 31
#define ESB_PONG_FLAG_TEST_MODE_OFF 32
#define ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON 52
#define RF_CHANNEL 1
#define RADIO_RF_CHANNEL 22
#define SYS_STATUS_CONNECTION_ERROR 1
#define CONFIG_DATA_COLLECT 1
#define RCV_HID_ST_OK 0
#define RCV_HID_ST_EINVAL 1
#define RCV_HID_ST_ENOENT 2
#define RCV_HID_ST_EBUSY 3
#define ESB_PONG_FLAG_DATA_COLLECT_OFF 35
#define ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF 53
typedef int atomic_t;
typedef int atomic_val_t;
typedef void (*esb_channel_change_done_cb_t)(bool);
typedef void (*esb_remote_confirm_cb_t)(uint8_t, uint8_t, uint16_t);
static int atomic_get(atomic_t *p) { return *p; }
static void atomic_set(atomic_t *p, int v) { *p = v; }
static int atomic_inc(atomic_t *p) { int old = *p; ++*p; return old; }
static bool atomic_cas(atomic_t *p, int old, int value) {
    if (*p != old) return false;
    *p = value;
    return true;
}
static int atomic_or(atomic_t *p, int v) { int old = *p; *p |= v; return old; }
static int tracker_store_lock;
static void k_mutex_lock(int *m, int timeout) { assert(!*m); *m = 1; }
static void k_mutex_unlock(int *m) { assert(*m); *m = 0; }
static int64_t now;
static int64_t k_uptime_get(void) { return now; }
static uint32_t k_uptime_get_32(void) { return (uint32_t)now; }
#define MIN(a, b) ((a) < (b) ? (a) : (b))
static void k_msleep(int32_t ms) { now += ms; }
#define REMOTE_COMMAND_ACTIVE_SCAN_MS 1000
static bool ota, self_ota, collecting, batch, abort_pending[MAX_TRACKERS];
static bool esb_ota_relay_is_active(void) { return ota; }
static bool receiver_ota_is_active(void) { return self_ota; }
static bool data_collect_is_active(void) { return collecting; }
static bool data_collect_batch_is_active(void) { return batch; }
static bool esb_ota_relay_abort_pending(uint8_t id) { return abort_pending[id]; }
static uint8_t collecting_id;
static uint32_t collecting_mask;
static void data_collect_start(uint8_t id) { batch = false; collecting = true; collecting_id = id; }
static void data_collect_stop(void) { collecting = false; }
static uint8_t data_collect_get_target_id(void) { return collecting_id; }
static void data_collect_batch_start(uint32_t mask, uint16_t rate) { collecting = false; batch = true; collecting_mask = mask; }
static void data_collect_batch_stop(void) { batch = false; }
static bool data_collect_batch_is_target(uint8_t id) { return (collecting_mask & BIT(id)) != 0; }
static uint8_t stored_trackers, receiver_rf_channel, physical_channel, durable_channel;
static uint64_t stored_tracker_addr[MAX_TRACKERS];
/* SLOT DECLARATIONS */
static void raw_arq_reset(void) {}
static bool rx_running, fault_status, auto_enabled;
static bool esb_pairing;
static uint32_t auto_generation;
static uint32_t auto_loss_reset_epoch;
static atomic_t test_all_generation, test_all_broadcasting;
static uint16_t metadata_next_token;
static atomic_t tdma_rx_discontinuity_pending;
static bool stale_fifo;
static int esb_flush_rx(void) { assert(!rx_running); stale_fifo = false; return 0; }
static bool auto_channel_decision_valid(uint32_t generation) {
    return auto_enabled && auto_generation == generation;
}
static unsigned irq_depth, radio_attempts, radio_deferred, radio_deliveries;
static bool radio_pending, radio_running;
static void (*radio_hook)(void);
static void host_deliver_radio(void) {
    assert(!irq_depth && radio_hook && !radio_running);
    radio_running = true;
    ++radio_deliveries;
    radio_hook();
    radio_running = false;
}
static void host_radio_preempt(void) {
    if (!radio_hook) return;
    ++radio_attempts;
    if (irq_depth || radio_running) {
        ++radio_deferred;
        radio_pending = true;
    } else {
        host_deliver_radio();
    }
}
static unsigned int irq_lock(void) { return irq_depth++; }
static void irq_unlock(unsigned int key) {
    assert(irq_depth == key + 1);
    irq_depth = key;
    if (!irq_depth && radio_pending && !radio_running) {
        radio_pending = false;
        host_deliver_radio();
    }
}
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
#define TEST_ALL_RATE_QUANTUM_TPS 10U
#define TDMA_BASE_SLOT_TICKS 16
#define MIN(a, b) ((a) < (b) ? (a) : (b))
static int test_all_mutex;
static atomic_t test_all_state_valid, test_all_enabled, test_all_confirmed_mask;
static atomic_t test_all_ready_after_ms[MAX_TRACKERS];
static uint16_t test_all_requested_tps, test_all_effective_tps;
static uint16_t test_all_remote_cmd_token;
static uint32_t tdma_active_mask;
static uint8_t tdma_dynamic_active_count, tdma_dynamic_slot_ticks;
static int fail_stop, fail_set, fail_start, fail_store;
static int esb_stop_rx(void) {
    if (fail_stop) { fail_stop--; return -EIO; }
    rx_running = false; return 0;
}
static int esb_set_rf_channel(uint8_t c) {
    assert(!rx_running);
    if (fail_set) { fail_set--; return -EIO; }
    physical_channel = c; return 0;
}
static int esb_start_rx(void) {
    assert(!stale_fifo);
    if (fail_start) { fail_start--; return -EIO; }
    rx_running = true; return 0;
}
static int sys_write_checked(uint16_t id, const void *data, size_t len) {
    assert(id == RF_CHANNEL && len == 1 && rx_running);
    if (fail_store) { fail_store--; return -EIO; }
    durable_channel = *(const uint8_t *)data;
    return 0;
}
static void set_status(int status, bool value) { fault_status = value; }
static void auto_channel_set_enabled(bool value) { auto_enabled = value; }
static unsigned auto_completed;
static uint32_t completed_generation;
static uint8_t completed_old_home, completed_new_home;
static bool auto_succeeded;
static void auto_channel_migration_complete(uint32_t generation, uint8_t old_home,
                                            uint8_t new_home, bool success) {
    auto_completed++;
    completed_generation = generation;
    completed_old_home = old_home;
    completed_new_home = new_home;
    auto_succeeded = success;
    if (success) assert(rx_running && physical_channel == new_home &&
                        (durable_channel == 128 ? 0 : durable_channel) == new_home);
}
struct payload { uint8_t data[13]; };
struct esb_payload { uint8_t data[64]; uint8_t length, pipe; bool noack; };
static uint16_t wom_next_token;
/* DIAGNOSTIC DECLARATIONS */
static uint8_t echo_counter[MAX_TRACKERS];
static esb_remote_confirm_cb_t remote_confirm_cb;
enum tdma_shadow_evidence { TDMA_SHADOW_EVIDENCE_PING, TDMA_SHADOW_EVIDENCE_DATA };
static void tdma_shadow_observe(uint8_t id, enum tdma_shadow_evidence evidence) {}
static uint32_t k_ticks_to_us_floor32(uint32_t ticks) { return ticks; }
/* SDK conversion and console policy leaves; channel cases keep detail off. */
static uint64_t k_ticks_to_us_floor64(uint64_t ticks) { return ticks * 1000000 / 32768; }
static bool esb_get_stats_detailed_enabled(void) { return false; }
static void test_all_invalidate_tracker(uint8_t id, int64_t at) {}
static const char *esb_pong_flag_name(uint8_t flag) { return "command"; }
static uint32_t sys_get_be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static atomic_t pairing_new_devices_blocked;
static bool ota_target, ota_payload;
static uint8_t receiver_device_addr[6] = {1, 2, 3, 4, 5, 6};
static int channel_get_error;
static uint32_t now_ticks;
static uint32_t k_uptime_ticks(void) { return now_ticks; }
static uint32_t k_ticks_to_ms_ceil32(uint32_t ticks)
{
    return (uint32_t)(((uint64_t)ticks * 1000 + 32767) / 32768);
}
static int esb_get_rf_channel(uint32_t *channel) {
    *channel = physical_channel;
    return channel_get_error;
}
static bool auto_channel_is_beacon(const uint8_t *p, uint8_t len) { return false; }
static bool data_collect_is_target(uint8_t id) { return id == collecting_id; }
static bool esb_ota_relay_is_target(uint8_t id) { return ota_target; }
static int esb_find_tracker(uint64_t addr) {
    for (unsigned i = 0; i < stored_trackers; ++i)
        if (stored_tracker_addr[i] == addr) return i;
    return -1;
}
static uint8_t crc8_ccitt(uint8_t crc, const uint8_t *data, size_t len) {
    while (len--) {
        crc ^= *data++;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80) ? 0x07 : 0));
    }
    return crc;
}
static uint16_t sys_get_be16(const uint8_t *p) { return (p[0] << 8) | p[1]; }
static void raw_arq_process_isr(uint16_t seq, struct esb_payload *p, bool *has) {}
static void esb_ota_relay_fill_ack(uint8_t id, uint32_t pipe, struct esb_payload *p,
                                 bool *has, const uint8_t *data, uint8_t len) {
    *has = ota_payload;
    if (*has) { p->length = 17; p->data[0] = 0xAB; }
}
/* The timer records its due time. Tests run the production work handler. */
struct k_work { int unused; };
enum { REMOTE_HID_IDLE = 0, REMOTE_HID_ACTIVE, REMOTE_HID_FINISHING };
enum {
    REMOTE_HID_JOB_NONE = 0,
    REMOTE_HID_JOB_FLAG_ALL,
    REMOTE_HID_JOB_SENS_AUTO_ALL,
    REMOTE_HID_JOB_TEST_ON_ALL,
    REMOTE_HID_JOB_TEST_OFF_ALL,
    REMOTE_HID_JOB_WOM_COMMAND,
};
static int remote_hid_job, remote_hid_pending, remote_hid_expected, remote_hid_confirmed;
static uint8_t remote_hid_seq, remote_hid_opcode, remote_hid_flag;
static uint8_t remote_hid_job_flag, remote_hid_job_axis;
static uint16_t remote_hid_job_rev, remote_hid_job_tps;
static uint16_t remote_hid_next_token, remote_hid_cmd_token;
static uint16_t remote_wom_token, remote_wom_minutes;
static int64_t remote_hid_deadline;
static union { uint32_t wom_failed; uint32_t stage_budget_ms; } remote_hid_wait;
#define TEST_REMOTE_HID_STAGE
static int tdma_membership_wake;
static void k_sem_give(int *sem) { (*sem)++; }
#define RCV_HID_ST_STARTED 7
#define RCV_HID_TARGET_ALL 0xff
#define REMOTE_HID_CONFIRM_TIMEOUT_MS 5000
#define K_MSEC(ms) (ms)
static int remote_hid_timeout_work, remote_hid_job_work, rssi_wq;
static int hid_status;
static unsigned hid_completions;
static uint8_t hid_completed_seq, hid_completed_opcode;
static int64_t hid_timer_due;
static void (*hid_cancel_hook)(void), (*hid_completion_hook)(void);
static int k_work_cancel_delayable(int *w) {
    ARG_UNUSED(w);
    if (hid_cancel_hook) { void (*hook)(void) = hid_cancel_hook; hid_cancel_hook = NULL; hook(); }
    hid_timer_due = -1;
    return 0;
}
static int k_work_reschedule(int *w, int delay) { ARG_UNUSED(w); hid_timer_due = now + delay; return 0; }
static int k_work_submit_to_queue(int *q, int *w) { ARG_UNUSED(q); ARG_UNUSED(w); return 0; }
static void remote_wom_poll(void) { assert(false); }
static void remote_hid_send_completion(uint8_t status) {
    hid_status = status;
    hid_completed_seq = remote_hid_seq;
    hid_completed_opcode = remote_hid_opcode;
    ++hid_completions;
    if (hid_completion_hook) { void (*hook)(void) = hid_completion_hook; hid_completion_hook = NULL; hook(); }
}
static bool rcv_hid_opcode_is_pong_flag(uint8_t flag) { return flag != ESB_PONG_FLAG_NORMAL; }
static uint8_t rcv_cmd_remote_flag(uint8_t target_id, uint8_t pong_flag) {
    ARG_UNUSED(target_id);
    ARG_UNUSED(pong_flag);
    return RCV_HID_ST_EINVAL;
}
/* PRODUCTION BODIES */
static unsigned completed;
static bool succeeded;
static void done(bool success) {
    assert(esb_channel_is_busy());
    assert(esb_set_all_trackers_channel(7) == (esb_radio_fault ? -EIO : -EBUSY));
    completed++;
    succeeded = success;
}
static void reset(unsigned count) {
    radio_operation_busy = channel_change_pending = 0;
    radio_startup_state = ESB_RADIO_READY;
    tracker_store_lock = test_all_mutex = 0;
    test_all_state_valid = test_all_enabled = test_all_broadcasting = 0;
    test_all_confirmed_mask = test_all_generation = 0;
    test_all_requested_tps = test_all_effective_tps = 0;
    test_all_remote_cmd_token = 0;
    memset(test_all_ready_after_ms, 0, sizeof(test_all_ready_after_ms));
    tdma_active_mask = count ? ((1u << count) - 1u) : 0;
    tdma_dynamic_active_count = count;
    tdma_dynamic_slot_ticks = TDMA_BASE_SLOT_TICKS;
    stored_trackers = count;
    relay_coord_init(&relay_coord);
    relay_profile = 0;
    memset(&tdma_staged, 0, sizeof(tdma_staged));
    relay_query_until = relay_query_protected_mask = 0;
    relay_query_recovering = relay_query_suspending = false;
    tdma_shadow_live_evidence_mask = tdma_shadow_desired_mask = 0;
    memset(stored_tracker_addr, 0, sizeof(stored_tracker_addr));
    for (unsigned i = 0; i < MAX_TRACKERS; ++i) {
        tracker_slots[i].tracker_remote_command = 0;
        tracker_slots[i].query_advertised = false;
        tracker_slots[i].remote_cmd_token = 0;
        tracker_slots[i].metadata_active_mask = 0;
        tracker_slots[i].metadata_pending_mask = 0;
        tracker_slots[i].ping_counter_initialized = false;
        tracker_slots[i].last_ping_time = 0;
        tracker_slots[i].tdma_config_packed = 0;
#if defined(CONFIG_TDMA_DIAGNOSTICS)
        tracker_diags[i].g_last_ping_rx_time_diff_valid = false;
        tracker_diags[i].g_bias_ppb_valid = 0;
        tracker_diags[i].g_ping_dirty_streak = 0;
#endif
    }
    memset(abort_pending, 0, sizeof(abort_pending));
    radio_hook = NULL;
    radio_pending = radio_running = false;
    radio_attempts = radio_deferred = radio_deliveries = 0;
    assert(!irq_depth);
    remote_hid_pending = REMOTE_HID_IDLE;
    remote_hid_expected = remote_hid_confirmed = 0;
    remote_hid_job = REMOTE_HID_JOB_NONE;
    hid_status = -1;
    hid_completions = 0;
    hid_timer_due = -1;
    hid_cancel_hook = hid_completion_hook = NULL;
    remote_confirm_cb = NULL;
    memset(echo_counter, 0, sizeof(echo_counter));
    ota_target = ota_payload = false;
    atomic_set(&pairing_new_devices_blocked, 0);
    channel_get_error = 0;
    for (unsigned i = 0; i < count; i++) stored_tracker_addr[i] = 100 + i;
    receiver_rf_channel = physical_channel = durable_channel = 22;
    rx_running = auto_enabled = true;
    fault_status = ota = self_ota = collecting = batch = false;
    fail_stop = fail_set = fail_start = fail_store = 0;
    now = 100;
    now_ticks = 0x12345678;
    completed = 0;
    auto_completed = 0;
    esb_radio_fault = esb_pairing = false;
    succeeded = false;
    channel_change_done_cb = done;
}
static void deadline(void) { now = channel_change_timeout; channel_change_poll(); }
static void assert_wire(uint8_t id, uint8_t flag, uint8_t channel) {
    struct payload p = {0};
    assert(pong(id, &p) == flag);
    if (flag == ESB_PONG_FLAG_SET_CHANNEL) {
        assert(p.data[8] == 0 && p.data[9] == 0 && p.data[10] == 0 && p.data[11] == channel);
    }
}
static struct esb_payload ping_ack(uint8_t tracker_id, uint8_t counter, uint8_t flags, uint8_t clock_valid) {
    uint8_t ping[ESB_PING_LEN] = {ESB_PING_TYPE, tracker_id, counter, clock_valid};
    ping[7] = flags;
    ping[12] = crc8_ccitt(0x07, ping, 12);
    struct esb_payload ack = {0};
    bool has = false, suppress = false;
    esb_ack_handler_cb(ping, sizeof(ping), 1 + tracker_id % 7, &ack, &has, &suppress);
    assert(has && !suppress);
    assert(ack.data[0] == ESB_PONG_TYPE && ack.data[1] == tracker_id && ack.data[2] == counter);
    assert(ack.data[12] == crc8_ccitt(0x07, ack.data, 12));
    return ack;
}
static struct esb_payload test_tracker_pong(uint8_t tracker_id, uint8_t flags) {
    struct esb_payload ack = ping_ack(tracker_id, 7, flags, 0);
    assert(ack.length == ESB_PONG_LEN);
    return ack;
}
static struct esb_payload test_pong(uint8_t flags) { return test_tracker_pong(0, flags); }
static void echo_ack(uint8_t tracker_id, const struct esb_payload *ack) {
    struct esb_payload ping = {.length = ESB_PING_LEN, .pipe = 1 + tracker_id % 7};
    ping.data[0] = ESB_PING_TYPE;
    ping.data[1] = tracker_id;
    ping.data[2] = ++echo_counter[tracker_id];
    memcpy(ping.data + 3, ack->data + 3, 4);
    ping.data[7] = ack->data[7];
    memcpy(ping.data + 8, ack->data + 8, 4);
    ping.data[ESB_PING_LEN - 1] = crc8_ccitt(0x07, ping.data, ESB_PING_LEN - 1);
    esb_receive_ping(&ping, (uint32_t)now);
}
static uint8_t publication_old[9], publication_new[9];
static void observe_publication(void) {
    struct esb_payload ack = test_pong(0);
    assert(!memcmp(ack.data + 3, publication_old, sizeof(publication_old)) ||
           !memcmp(ack.data + 3, publication_new, sizeof(publication_new)));
}
static void arm_publication(const uint8_t old[9], const uint8_t next[9]) {
    struct esb_payload ack = test_pong(0);
    assert(!memcmp(ack.data + 3, old, sizeof(publication_old)));
    memcpy(publication_old, old, sizeof(publication_old));
    memcpy(publication_new, next, sizeof(publication_new));
    radio_attempts = radio_deferred = radio_deliveries = 0;
    radio_hook = observe_publication;
}
static void finish_publication(unsigned stores) {
    assert(!irq_depth && !radio_pending);
    assert(radio_attempts == stores && radio_deferred == stores && radio_deliveries == 1);
    radio_hook = NULL;
    struct esb_payload ack = test_pong(0);
    assert(!memcmp(ack.data + 3, publication_new, sizeof(publication_new)));
}
static void test_command_publication(void) {
    static const uint8_t sens_old[9] = {
        0, 100, 0, 200, ESB_PONG_FLAG_SENS_SET, 1, 44, 0, 0,
    };
    static const uint8_t sens_new[9] = {
        1, 144, 0xfe, 0x0c, ESB_PONG_FLAG_SENS_SET, 2, 88, 0, 0,
    };
    static const uint8_t auto_old[9] = {
        1, 0x12, 0x34, 0, ESB_PONG_FLAG_SENS_AUTO, 0, 0, 0, 0,
    };
    static const uint8_t auto_new[9] = {
        2, 0xab, 0xcd, 0, ESB_PONG_FLAG_SENS_AUTO, 0, 0, 0, 0,
    };
    static const uint8_t tps_old[9] = {
        0x12, 0x34, 0x56, 0x78, ESB_PONG_FLAG_TEST_MODE_ON, 0x12, 0x34, 0, 0,
    };
    static const uint8_t tps_new[9] = {
        0x12, 0x34, 0x56, 0x78, ESB_PONG_FLAG_TEST_MODE_ON, 0x24, 0x68, 0, 0,
    };
    static const uint8_t tps_off[9] = {
        0x12, 0x34, 0x56, 0x78, ESB_PONG_FLAG_TEST_MODE_OFF, 0, 0, 0, 0,
    };
    static const uint8_t normal[9] = {
        0x12, 0x34, 0x56, 0x78, ESB_PONG_FLAG_NORMAL, 0, 0, 0, 0,
    };
    static const uint8_t metadata_first[9] = {
        0x12, 0x34, 0x56, 0x78, ESB_PONG_FLAG_METADATA_REQUEST, 1, 2, 0x12, 0x34,
    };
    static const uint8_t metadata_second[9] = {
        0x12, 0x34, 0x56, 0x78, ESB_PONG_FLAG_METADATA_REQUEST, 4, 5, 0x12, 0x35,
    };

    reset(1);
    esb_send_remote_command_sens(0, 1, 2, 3);
    arm_publication(sens_old, sens_new);
    esb_send_remote_command_sens(0, 4, -5, 6);
    finish_publication(5);

    reset(1);
    tracker_slots[0].stats.last_packet_time = now;
    assert(esb_send_remote_command_sens_auto(0, 1, 0x1234));
    arm_publication(auto_old, auto_new);
    assert(esb_send_remote_command_sens_auto(0, 2, 0xabcd));
    finish_publication(4);

    reset(1);
    tracker_slots[0].stats.last_packet_time = now;
    assert(esb_send_remote_command_sens_auto(0, 1, 0x1234));
    arm_publication(auto_old, auto_new);
    assert(esb_send_remote_command_sens_auto_all(2, 0xabcd) == BIT(0));
    finish_publication(4);

    reset(1);
    assert(esb_send_remote_command_test_on(0, 0x1234) == BIT(0));
    arm_publication(tps_old, tps_new);
    assert(esb_send_remote_command_test_on(0, 0x2468) == BIT(0));
    finish_publication(3);
    arm_publication(tps_new, tps_off);
    assert(esb_send_remote_command_test_off(0) == BIT(0));
    finish_publication(3);

    reset(1);
    tracker_slots[0].stats.last_packet_time = now;
    metadata_next_token = 0x1233;
    arm_publication(normal, metadata_first);
    assert(esb_request_metadata(0, 1, 2));
    finish_publication(3);
    assert(tracker_slots[0].metadata_active_mask == 1 &&
           tracker_slots[0].metadata_active_chunk == 2 &&
           tracker_slots[0].metadata_active_token == 0x1234 &&
           tracker_slots[0].metadata_pending_mask == 0);
    arm_publication(metadata_first, metadata_first);
    assert(esb_request_metadata(0, 4, 5));
    finish_publication(3);
    assert(tracker_slots[0].metadata_active_mask == 1 &&
           tracker_slots[0].metadata_active_chunk == 2 &&
           tracker_slots[0].metadata_active_token == 0x1234 &&
           tracker_slots[0].metadata_pending_mask == 4 &&
           tracker_slots[0].metadata_pending_chunk == 5 &&
           tracker_slots[0].metadata_pending_token == 0x1235);

    struct esb_payload echo_packet = {.length = ESB_PING_LEN, .pipe = 1};
    echo_packet.data[0] = ESB_PING_TYPE;
    echo_packet.data[1] = 0;
    echo_packet.data[2] = 1;
    echo_packet.data[7] = ESB_PONG_FLAG_METADATA_REQUEST;
    echo_packet.data[8] = 1;
    echo_packet.data[9] = 2;
    echo_packet.data[10] = 0x12;
    echo_packet.data[11] = 0x35;
    echo_packet.data[12] = crc8_ccitt(0x07, echo_packet.data, 12);
    esb_receive_ping(&echo_packet, 0);
    assert(tracker_slots[0].metadata_active_mask == 1 &&
           tracker_slots[0].metadata_pending_mask == 4);
    echo_packet.data[2] = 2;
    echo_packet.data[11] = 0x34;
    echo_packet.data[12] = crc8_ccitt(0x07, echo_packet.data, 12);
    esb_receive_ping(&echo_packet, 0);
    assert(tracker_slots[0].metadata_active_mask == 0 &&
           tracker_slots[0].metadata_pending_mask == 4);
    struct esb_payload ack = test_pong(0);
    assert(!memcmp(ack.data + 3, metadata_second, sizeof(metadata_second)));
    assert(tracker_slots[0].metadata_active_mask == 4 &&
           tracker_slots[0].metadata_active_chunk == 5 &&
           tracker_slots[0].metadata_active_token == 0x1235 &&
           tracker_slots[0].metadata_pending_mask == 0);
}
static void observe_retired_timeout_pong(void) {
    assert(!irq_depth && remote_hid_pending == REMOTE_HID_FINISHING);
    assert(test_pong(0).data[7] == ESB_PONG_FLAG_NORMAL);
    assert(hid_status == -1);
}
static void test_hid_timeout_cancellation(void) {
    reset(3);
    remote_hid_pending = remote_hid_expected = remote_hid_confirmed = 0;
    remote_hid_job = REMOTE_HID_JOB_NONE;
    remote_hid_next_token = remote_hid_cmd_token = 0;
    remote_confirm_cb = on_remote_confirm;

    /* 1. Ordinary 5s timeout clears matching unconfirmed command. */
    hid_status = -1;
    assert(rcv_cmd_remote_sens_set_hid(1, ESB_PONG_FLAG_SENS_SET, 0, 1, 2, 3) == RCV_HID_ST_STARTED);
    uint16_t first_token = remote_hid_cmd_token;
    assert(first_token != 0 && tracker_slots[0].remote_cmd_token == first_token);
    assert(hid_timer_due == now + REMOTE_HID_CONFIRM_TIMEOUT_MS);
    assert(test_pong(0).data[7] == ESB_PONG_FLAG_SENS_SET);
    radio_hook = observe_retired_timeout_pong;
    now += REMOTE_HID_CONFIRM_TIMEOUT_MS;
    remote_hid_timeout_handler(NULL);
    assert(radio_attempts == 1 && radio_deferred == 1 && radio_deliveries == 1);
    radio_hook = NULL;
    assert(!remote_hid_pending && hid_status == RCV_HID_ST_ENOENT);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_NORMAL);
    assert(tracker_slots[0].remote_cmd_token == 0);
    assert(test_pong(0).data[7] == ESB_PONG_FLAG_NORMAL);

    /* 1b. Targeted generic flag HID timeout also clears pending command. */
    hid_status = -1;
    assert(rcv_cmd_remote_flag_hid(2, ESB_PONG_FLAG_MEOW, 0, ESB_PONG_FLAG_MEOW) == RCV_HID_ST_STARTED);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_MEOW);
    assert(tracker_slots[0].remote_cmd_token == remote_hid_cmd_token && remote_hid_cmd_token != 0);
    now += REMOTE_HID_CONFIRM_TIMEOUT_MS;
    remote_hid_timeout_handler(NULL);
    assert(!remote_hid_pending && hid_status == RCV_HID_ST_ENOENT);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_NORMAL);
    assert(tracker_slots[0].remote_cmd_token == 0);

    /* 2. Later same-flag console command (token 0) is preserved on HID timeout. */
    hid_status = -1;
    assert(rcv_cmd_remote_sens_set_hid(3, ESB_PONG_FLAG_SENS_SET, 0, 1, 2, 3) == RCV_HID_ST_STARTED);
    assert(tracker_slots[0].remote_cmd_token == remote_hid_cmd_token);
    esb_send_remote_command_sens(0, 4, 5, 6);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_SENS_SET);
    assert(tracker_slots[0].remote_cmd_token == 0);
    struct esb_payload console_ack = test_pong(0);
    assert(console_ack.data[7] == ESB_PONG_FLAG_SENS_SET);
    assert(console_ack.data[3] == 1 && console_ack.data[4] == 144 &&
           console_ack.data[5] == 1 && console_ack.data[6] == 244 &&
           console_ack.data[8] == 2 && console_ack.data[9] == 88);
    unsigned previous_completions = hid_completions;
    echo_ack(0, &console_ack);
    assert(remote_hid_pending == REMOTE_HID_ACTIVE && hid_status == -1);
    assert(hid_completions == previous_completions);
    esb_send_remote_command_sens(0, 4, 5, 6);
    now += REMOTE_HID_CONFIRM_TIMEOUT_MS;
    remote_hid_timeout_handler(NULL);
    assert(!remote_hid_pending && hid_status == RCV_HID_ST_ENOENT);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_SENS_SET);
    assert(tracker_slots[0].pending_sens_data[0] == 400 &&
           tracker_slots[0].pending_sens_data[1] == 500 &&
           tracker_slots[0].pending_sens_data[2] == 600);
    assert(test_pong(0).data[7] == ESB_PONG_FLAG_SENS_SET);
    /* Direct cancellation with token 0 never clears console commands. */
    unsigned key = irq_lock();
    esb_finish_remote_command_locked(0, true);
    irq_unlock(key);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_SENS_SET);

    /* 3. Later different-flag command is also preserved on HID timeout. */
    hid_status = -1;
    assert(rcv_cmd_remote_sens_set_hid(4, ESB_PONG_FLAG_SENS_SET, 0, 1, 2, 3) == RCV_HID_ST_STARTED);
    esb_send_remote_command(0, ESB_PONG_FLAG_MEOW);
    now += REMOTE_HID_CONFIRM_TIMEOUT_MS;
    remote_hid_timeout_handler(NULL);
    assert(!remote_hid_pending && hid_status == RCV_HID_ST_ENOENT);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_MEOW);

    /* 4. Stale token never cancels a newer HID transaction with the same flag. */
    hid_status = -1;
    assert(rcv_cmd_remote_sens_set_hid(5, ESB_PONG_FLAG_SENS_SET, 0, 7, 8, 9) == RCV_HID_ST_STARTED);
    uint16_t live_token = remote_hid_cmd_token;
    assert(live_token != first_token);
    key = irq_lock();
    esb_finish_remote_command_locked(first_token, true);
    irq_unlock(key);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_SENS_SET);
    assert(tracker_slots[0].remote_cmd_token == live_token);
    echo(0, ESB_PONG_FLAG_SENS_SET);
    assert(!remote_hid_pending && hid_status == RCV_HID_ST_OK);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_NORMAL);
    assert(tracker_slots[0].remote_cmd_token == 0);

    /* 5. Multi-target timeout clears unconfirmed slots without touching a confirmed slot's new command. */
    hid_status = -1;
    assert(rcv_cmd_remote_sens_set_hid(6, ESB_PONG_FLAG_SENS_SET, RCV_HID_TARGET_ALL, 1, 2, 3) == RCV_HID_ST_STARTED);
    echo(0, ESB_PONG_FLAG_SENS_SET);
    assert(remote_hid_pending && hid_status == -1);
    esb_send_remote_command_sens(0, 9, 9, 9);
    now += REMOTE_HID_CONFIRM_TIMEOUT_MS;
    remote_hid_timeout_handler(NULL);
    assert(!remote_hid_pending && hid_status == RCV_HID_ST_ENOENT);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_SENS_SET);
    assert(tracker_slots[1].tracker_remote_command == ESB_PONG_FLAG_NORMAL);
    assert(tracker_slots[1].remote_cmd_token == 0);
    assert(tracker_slots[2].tracker_remote_command == ESB_PONG_FLAG_NORMAL);
    assert(tracker_slots[2].remote_cmd_token == 0);

    /* 6. Token wraparound skips 0 and targeted TEST_MODE_ON timeout clears slot. */
    remote_hid_next_token = ESB_REMOTE_CMD_TOKEN_MAX;
    hid_status = -1;
    assert(rcv_cmd_remote_test_on_hid(7, ESB_PONG_FLAG_TEST_MODE_ON, 0, 250) == RCV_HID_ST_STARTED);
    assert(remote_hid_cmd_token == 1);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_TEST_MODE_ON);
    assert(tracker_slots[0].remote_cmd_token == 1);
    now += REMOTE_HID_CONFIRM_TIMEOUT_MS;
    remote_hid_timeout_handler(NULL);
    assert(!remote_hid_pending && hid_status == RCV_HID_ST_ENOENT);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_NORMAL);
    assert(tracker_slots[0].remote_cmd_token == 0);

    /* Sticky all-target TEST_MODE_ON/OFF retain the receiver policy while
     * carrying an independently retireable HID completion subscriber. */
    reset(2);
    remote_hid_pending = remote_hid_expected = remote_hid_confirmed = 0;
    remote_hid_job = REMOTE_HID_JOB_NONE;
    hid_status = -1;
    tracker_slots[0].stats.last_packet_time = now + REMOTE_COMMAND_ACTIVE_SCAN_MS;
    tracker_slots[1].stats.last_packet_time = now + REMOTE_COMMAND_ACTIVE_SCAN_MS;
    assert(remote_hid_begin(8, ESB_PONG_FLAG_TEST_MODE_ON, ESB_PONG_FLAG_TEST_MODE_ON) == RCV_HID_ST_STARTED);
    uint16_t all_token = remote_hid_cmd_token;
    uint32_t all_mask = esb_send_remote_command_test_on_all_tx(250, all_token);
    assert(all_mask == (BIT(0) | BIT(1)));
    assert(test_all_state_valid && test_all_enabled && test_all_requested_tps == 250 && test_all_effective_tps == 250);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_TEST_MODE_ON &&
           tracker_slots[0].remote_cmd_token == (all_token | ESB_REMOTE_CMD_RETAIN_BIT));
    assert(tracker_slots[1].tracker_remote_command == ESB_PONG_FLAG_TEST_MODE_ON &&
           tracker_slots[1].remote_cmd_token == (all_token | ESB_REMOTE_CMD_RETAIN_BIT));
    remote_hid_arm_expected(all_mask, all_token);
    now += REMOTE_HID_CONFIRM_TIMEOUT_MS;
    remote_hid_timeout_handler(NULL);
    assert(!remote_hid_pending && hid_status == RCV_HID_ST_ENOENT);
    assert(test_all_state_valid && test_all_enabled && test_all_requested_tps == 250 && test_all_effective_tps == 250);
    assert(test_all_remote_cmd_token == 0);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_TEST_MODE_ON && tracker_slots[0].remote_cmd_token == 0);
    assert(tracker_slots[1].tracker_remote_command == ESB_PONG_FLAG_TEST_MODE_ON && tracker_slots[1].remote_cmd_token == 0);
    /* Reconciliation still delivers the sticky policy to late/idle trackers. */
    tracker_slots[1].tracker_remote_command = ESB_PONG_FLAG_NORMAL;
    test_all_reconcile(now + 1000);
    assert(tracker_slots[1].tracker_remote_command == ESB_PONG_FLAG_TEST_MODE_ON && tracker_slots[1].remote_cmd_token == 0);

    /* Sticky TEST_MODE_OFF all timeout also preserves policy and pending slots. */
    reset(2);
    tracker_slots[0].stats.last_packet_time = now + REMOTE_COMMAND_ACTIVE_SCAN_MS;
    tracker_slots[1].stats.last_packet_time = 0;
    remote_hid_pending = remote_hid_expected = remote_hid_confirmed = 0;
    hid_status = -1;
    assert(remote_hid_begin(9, ESB_PONG_FLAG_TEST_MODE_OFF, ESB_PONG_FLAG_TEST_MODE_OFF) == RCV_HID_ST_STARTED);
    uint16_t off_token = remote_hid_cmd_token;
    uint32_t off_mask = esb_send_remote_command_test_off_all_tx(off_token);
    assert(off_mask == BIT(0) && test_all_state_valid && !test_all_enabled);
    assert(tracker_slots[0].remote_cmd_token == (off_token | ESB_REMOTE_CMD_RETAIN_BIT));
    remote_hid_arm_expected(off_mask, off_token);
    now += REMOTE_HID_CONFIRM_TIMEOUT_MS;
    remote_hid_timeout_handler(NULL);
    assert(!remote_hid_pending && hid_status == RCV_HID_ST_ENOENT);
    assert(test_all_state_valid && !test_all_enabled);
    assert(test_all_remote_cmd_token == 0);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_TEST_MODE_OFF && tracker_slots[0].remote_cmd_token == 0);

    remote_confirm_cb = NULL;
    puts("PASS HID timeout cancellation: clears expired one-shot commands, preserves console/newer/sticky-all commands, skips token 0");
}
static void test_receiver_owned_ota_coalescing(void) {
    for (unsigned receiver_all = 0; receiver_all < 2; ++receiver_all) {
        for (unsigned hid_all = 0; hid_all < 2; ++hid_all) {
            reset(2);
            remote_confirm_cb = on_remote_confirm;
            tracker_slots[0].stats.last_packet_time = now + 10000;
            tracker_slots[1].stats.last_packet_time = now + 10000;
            if (receiver_all) {
                assert(esb_send_remote_command_all(ESB_PONG_FLAG_OTA_UNSUPPRESS) == (BIT(0) | BIT(1)));
            } else {
                esb_send_remote_command(0, ESB_PONG_FLAG_OTA_UNSUPPRESS);
            }
            assert(test_pong(0).data[7] == ESB_PONG_FLAG_OTA_UNSUPPRESS);
            assert(tracker_slots[0].remote_cmd_token == 0);

            assert(rcv_cmd_remote_flag_hid(10, ESB_PONG_FLAG_OTA_UNSUPPRESS,
                                          hid_all ? RCV_HID_TARGET_ALL : 0,
                                          ESB_PONG_FLAG_OTA_UNSUPPRESS) == RCV_HID_ST_STARTED);
            if (hid_all) remote_hid_job_handler(NULL);
            uint16_t token = remote_hid_cmd_token;
            assert(token && tracker_slots[0].remote_cmd_token == (token | ESB_REMOTE_CMD_RETAIN_BIT));
            assert(hid_timer_due == now + REMOTE_HID_CONFIRM_TIMEOUT_MS);
            now = hid_timer_due;
            remote_hid_timeout_handler(NULL);
            assert(remote_hid_pending == REMOTE_HID_IDLE && hid_status == RCV_HID_ST_ENOENT);
            assert(hid_completions == 1 && hid_completed_seq == 10);
            assert(test_pong(0).data[7] == ESB_PONG_FLAG_OTA_UNSUPPRESS);
            assert(tracker_slots[0].remote_cmd_token == 0);
            if (receiver_all) {
                assert(test_tracker_pong(1, 0).data[7] == ESB_PONG_FLAG_OTA_UNSUPPRESS);
                assert(tracker_slots[1].remote_cmd_token == 0);
            } else if (hid_all) {
                assert(test_tracker_pong(1, 0).data[7] == ESB_PONG_FLAG_NORMAL);
                assert(tracker_slots[1].remote_cmd_token == 0);
            }
        }
    }
    puts("PASS receiver-owned OTA unsuppress survives coalesced HID timeout for targeted and broadcast writers");
}

static void test_hid_then_console_coalescing(void) {
    reset(1);
    remote_confirm_cb = on_remote_confirm;
    assert(rcv_cmd_remote_flag_hid(20, ESB_PONG_FLAG_MEOW, 0, ESB_PONG_FLAG_MEOW) == RCV_HID_ST_STARTED);
    esb_send_remote_command(0, ESB_PONG_FLAG_MEOW);
    struct esb_payload ack = test_pong(ESB_PONG_FLAG_NORMAL);
    assert(ack.data[7] == ESB_PONG_FLAG_MEOW);
    echo_ack(0, &ack);
    assert(hid_completions == 1 && hid_completed_seq == 20 && hid_status == RCV_HID_ST_OK);

    reset(1);
    assert(rcv_cmd_remote_sens_set_hid(21, ESB_PONG_FLAG_SENS_SET, 0, 1, 2, 3) == RCV_HID_ST_STARTED);
    esb_send_remote_command_sens(0, 1, 2, 3);
    ack = test_pong(ESB_PONG_FLAG_NORMAL);
    assert(ack.data[7] == ESB_PONG_FLAG_SENS_SET);
    now = hid_timer_due;
    remote_hid_timeout_handler(NULL);
    assert(hid_completions == 1 && hid_completed_seq == 21 && hid_status == RCV_HID_ST_ENOENT);
    struct esb_payload after = test_pong(ESB_PONG_FLAG_NORMAL);
    assert(memcmp(ack.data, after.data, ESB_PONG_LEN) == 0);
    puts("PASS identical console publication preserves HID completion and survives HID timeout");
}

static unsigned early_publications;
static void confirm_during_flag_publication(void) {
    assert(remote_hid_pending == REMOTE_HID_ACTIVE && remote_hid_expected == 0);
    for (uint8_t id = 0; id < 2; ++id) {
        if (tracker_slots[id].tracker_remote_command != ESB_PONG_FLAG_MEOW) continue;
        assert(tracker_slots[id].remote_cmd_token == remote_hid_cmd_token);
        struct esb_payload ack = test_tracker_pong(id, 0);
        assert(ack.data[7] == ESB_PONG_FLAG_MEOW);
        echo_ack(id, &ack);
        ++early_publications;
    }
    if (early_publications == 2) radio_hook = NULL;
    assert(remote_hid_pending == REMOTE_HID_ACTIVE && hid_completions == 0);
}

static void test_broadcast_publication_confirmations(void) {
    reset(2);
    remote_confirm_cb = on_remote_confirm;
    tracker_slots[0].stats.last_packet_time = now + REMOTE_COMMAND_ACTIVE_SCAN_MS;
    tracker_slots[1].stats.last_packet_time = now + REMOTE_COMMAND_ACTIVE_SCAN_MS;
    early_publications = 0;
    assert(rcv_cmd_remote_flag_hid(19, ESB_PONG_FLAG_MEOW, RCV_HID_TARGET_ALL,
                                   ESB_PONG_FLAG_MEOW) == RCV_HID_ST_STARTED);
    radio_hook = confirm_during_flag_publication;
    remote_hid_job_handler(NULL);
    radio_hook = NULL;
    assert(early_publications == 2 && radio_attempts == 2 &&
           radio_deferred == 2 && radio_deliveries == 2);
    assert(remote_hid_pending == REMOTE_HID_IDLE && hid_status == RCV_HID_ST_OK);
    assert(hid_completions == 1 && hid_completed_seq == 19);
    assert(remote_hid_expected == (BIT(0) | BIT(1)) &&
           remote_hid_confirmed == (BIT(0) | BIT(1)));
    puts("PASS broadcast echoes between slot publications are retained until scan-mask arming");
}

static void confirm_sticky_before_arming(void) {
    radio_hook = NULL;
    assert(remote_hid_pending == REMOTE_HID_ACTIVE && remote_hid_expected == 0);
    uint16_t token = remote_hid_cmd_token;
    on_remote_confirm(0, remote_hid_flag, 0);
    on_remote_confirm(0, remote_hid_flag, token == ESB_REMOTE_CMD_TOKEN_MAX ? 1 : token + 1);
    assert(remote_hid_confirmed == 0);
    for (uint8_t id = 0; id < 2; ++id) {
        assert(tracker_slots[id].remote_cmd_token == (token | ESB_REMOTE_CMD_RETAIN_BIT));
        struct esb_payload ack = test_tracker_pong(id, 0);
        assert(ack.data[7] == remote_hid_flag);
        echo_ack(id, &ack);
    }
    assert(remote_hid_pending == REMOTE_HID_ACTIVE && remote_hid_expected == 0);
    assert(remote_hid_confirmed == (BIT(0) | BIT(1)) && hid_completions == 0);
}

static void test_sticky_early_confirmations(void) {
    for (unsigned enabled = 0; enabled < 2; ++enabled) {
        reset(2);
        remote_confirm_cb = on_remote_confirm;
        tracker_slots[0].stats.last_packet_time = now + REMOTE_COMMAND_ACTIVE_SCAN_MS;
        tracker_slots[1].stats.last_packet_time = now + REMOTE_COMMAND_ACTIVE_SCAN_MS;
        uint8_t flag = enabled ? ESB_PONG_FLAG_TEST_MODE_ON : ESB_PONG_FLAG_TEST_MODE_OFF;
        uint8_t status = enabled
            ? rcv_cmd_remote_test_on_hid(20, flag, RCV_HID_TARGET_ALL, 250)
            : rcv_cmd_remote_test_off_hid(20, flag, RCV_HID_TARGET_ALL);
        assert(status == RCV_HID_ST_STARTED);
        radio_hook = confirm_sticky_before_arming;
        remote_hid_job_handler(NULL);
        radio_hook = NULL;
        assert(radio_attempts == 1 && radio_deliveries == 1);
        assert(remote_hid_pending == REMOTE_HID_IDLE && hid_status == RCV_HID_ST_OK);
        assert(hid_completions == 1 && hid_completed_seq == 20 && hid_completed_opcode == flag);
        assert(remote_hid_expected == (BIT(0) | BIT(1)));
        assert(test_all_state_valid && !!test_all_enabled == !!enabled);
        assert(test_all_remote_cmd_token == 0);
        assert((test_all_confirmed_mask & (BIT(0) | BIT(1))) == (BIT(0) | BIT(1)));
        if (enabled) assert(test_all_requested_tps == 250 && test_all_effective_tps == 250);
        else assert(test_all_effective_tps == 0);
    }
    puts("PASS sticky HID confirmations arriving before scan-mask arming complete exactly once");
}

static void test_sticky_armed_confirmations(void) {
    for (unsigned enabled = 0; enabled < 2; ++enabled) {
        reset(2);
        remote_confirm_cb = on_remote_confirm;
        tracker_slots[0].stats.last_packet_time = now + REMOTE_COMMAND_ACTIVE_SCAN_MS;
        tracker_slots[1].stats.last_packet_time = now + REMOTE_COMMAND_ACTIVE_SCAN_MS;
        uint8_t flag = enabled ? ESB_PONG_FLAG_TEST_MODE_ON : ESB_PONG_FLAG_TEST_MODE_OFF;
        uint8_t status = enabled
            ? rcv_cmd_remote_test_on_hid(21, flag, RCV_HID_TARGET_ALL, 250)
            : rcv_cmd_remote_test_off_hid(21, flag, RCV_HID_TARGET_ALL);
        assert(status == RCV_HID_ST_STARTED);
        remote_hid_job_handler(NULL);
        int64_t due = now + REMOTE_HID_CONFIRM_TIMEOUT_MS;
        assert(remote_hid_pending == REMOTE_HID_ACTIVE &&
               remote_hid_expected == (BIT(0) | BIT(1)) && hid_timer_due == due);
        struct esb_payload first = test_tracker_pong(0, 0);
        assert(first.data[7] == flag);
        if (enabled) {
            struct esb_payload wrong_tps = first;
            wrong_tps.data[9] ^= 1;
            echo_ack(0, &wrong_tps);
            assert(remote_hid_confirmed == 0 && tracker_slots[0].tracker_remote_command == flag);
        }
        echo_ack(0, &first);
        assert(remote_hid_pending == REMOTE_HID_ACTIVE && remote_hid_confirmed == BIT(0));
        assert(hid_timer_due == due && hid_completions == 0);
        struct esb_payload second = test_tracker_pong(1, 0);
        assert(second.data[7] == flag);
        echo_ack(1, &second);
        assert(remote_hid_pending == REMOTE_HID_IDLE && hid_status == RCV_HID_ST_OK);
        assert(hid_completions == 1 && hid_completed_seq == 21 && hid_timer_due == -1);
        assert(test_all_state_valid && !!test_all_enabled == !!enabled && test_all_remote_cmd_token == 0);
        assert((test_all_confirmed_mask & (BIT(0) | BIT(1))) == (BIT(0) | BIT(1)));
    }
    puts("PASS sticky HID completion waits for matching armed confirmations");
}

static unsigned finishing_rejections;
static void try_request_while_finishing(void) {
    assert(!irq_depth && remote_hid_pending == REMOTE_HID_FINISHING);
    assert(remote_hid_seq == 30 && remote_hid_opcode == ESB_PONG_FLAG_MEOW);
    assert(rcv_cmd_remote_sens_set_hid(31, ESB_PONG_FLAG_SENS_SET, 0, 1, 2, 3) == RCV_HID_ST_EBUSY);
    ++finishing_rejections;
}

static void test_empty_scan_finish_isolation(void) {
    reset(1);
    tracker_slots[0].stats.last_packet_time = 0;
    remote_confirm_cb = on_remote_confirm;
    finishing_rejections = 0;
    assert(rcv_cmd_remote_flag_hid(30, ESB_PONG_FLAG_MEOW, RCV_HID_TARGET_ALL,
                                   ESB_PONG_FLAG_MEOW) == RCV_HID_ST_STARTED);
    hid_cancel_hook = try_request_while_finishing;
    hid_completion_hook = try_request_while_finishing;
    remote_hid_job_handler(NULL);
    assert(finishing_rejections == 2 && remote_hid_pending == REMOTE_HID_IDLE);
    assert(hid_completions == 1 && hid_completed_seq == 30 &&
           hid_completed_opcode == ESB_PONG_FLAG_MEOW && hid_status == RCV_HID_ST_ENOENT);
    assert(hid_timer_due == -1);

    assert(rcv_cmd_remote_sens_set_hid(32, ESB_PONG_FLAG_SENS_SET, 0, 1, 2, 3) == RCV_HID_ST_STARTED);
    int64_t due = now + REMOTE_HID_CONFIRM_TIMEOUT_MS;
    assert(hid_timer_due == due && test_pong(0).data[7] == ESB_PONG_FLAG_SENS_SET);
    now = due - 1;
    assert(hid_timer_due > now && remote_hid_pending == REMOTE_HID_ACTIVE && hid_completions == 1);
    now = due;
    assert(now == hid_timer_due);
    remote_hid_timeout_handler(NULL);
    assert(remote_hid_pending == REMOTE_HID_IDLE && hid_timer_due == -1);
    assert(hid_completions == 2 && hid_completed_seq == 32 &&
           hid_completed_opcode == ESB_PONG_FLAG_SENS_SET && hid_status == RCV_HID_ST_ENOENT);
    assert(test_pong(0).data[7] == ESB_PONG_FLAG_NORMAL);
    puts("PASS empty-scan completion retains ownership through cancellation and reply");
}

static void assert_channel_confirm(struct esb_payload p, uint8_t channel) {
    assert(p.data[7] == ESB_PONG_FLAG_CHANNEL_CONFIRM);
    assert(p.data[8] == channel && p.data[9] == ESB_CHANNEL_CONFIRM_VERSION);
    assert(p.data[10] == 0 && p.data[11] == 0);
    assert(p.data[3] == 0x12 && p.data[4] == 0x34 &&
           p.data[5] == 0x56 && p.data[6] == 0x78);
}
static void test_query_confirmation(void)
{
    reset(1);
    relay_query_until = 0;
    const uint8_t query = ESB_PONG_FLAG_OTA_QUERY_INFO;
    echo(0, query);
    assert(!relay_query_until);
    esb_send_remote_command_arg(0, query, 0);
    echo(0, query);
    assert(!relay_query_until);
    assert(test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM).data[7] == ESB_PONG_FLAG_CHANNEL_CONFIRM);
    assert(!tracker_slots[0].query_advertised);
    now_ticks = (uint32_t)now;
    uint32_t received = now_ticks;
    assert(test_pong(query).data[7] == query);
    now_ticks = (uint32_t)++now;
    echo_at(0, query, received);
    assert(!relay_query_until); /* This PING predates its own ACK. */
    now_ticks = (uint32_t)++now;
    struct esb_payload wrong_pipe = {.length = ESB_PING_LEN, .pipe = 2};
    wrong_pipe.data[0] = ESB_PING_TYPE;
    wrong_pipe.data[2] = ++echo_counter[0];
    wrong_pipe.data[7] = query;
    wrong_pipe.data[12] = crc8_ccitt(0x07, wrong_pipe.data, 12);
    esb_receive_ping(&wrong_pipe, (uint32_t)now);
    assert(!relay_query_until && tracker_slots[0].tracker_remote_command == query);
    now_ticks = (uint32_t)++now;
    const uint64_t advertised = tracker_slots[0].query_advertised_ticks;
    esb_send_remote_command(0, query);
    esb_publish_flag_mask(query, BIT(0), 0);
    assert(tracker_slots[0].query_advertised && tracker_slots[0].query_advertised_ticks == advertised);
    echo(0, query);
    const int64_t deadline = now + 15000;
    assert(relay_query_until == deadline);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_NORMAL);
    now_ticks = (uint32_t)++now;
    echo(0, query);
    assert(relay_query_until == deadline);

    reset(2);
    now_ticks = (uint32_t)now;
    esb_send_remote_command_arg_tx(0, query, 0, 11);
    assert(test_pong(0).data[7] == query && tracker_slots[0].query_advertised);
    esb_publish_flag_mask(query, BIT(0) | BIT(1), 12);
    assert(!tracker_slots[0].query_advertised && !tracker_slots[1].query_advertised);
    now_ticks = (uint32_t)++now;
    echo(0, query);
    echo(1, query);
    assert(!relay_query_until);
    assert(test_pong(0).data[7] == query && tracker_slots[0].query_advertised);
    unsigned int key = irq_lock();
    esb_finish_remote_command_locked(12, true);
    irq_unlock(key);
    assert(!tracker_slots[0].query_advertised && !tracker_slots[1].query_advertised);
    now_ticks = (uint32_t)++now;
    echo(0, query);
    assert(!relay_query_until && test_pong(0).data[7] == ESB_PONG_FLAG_NORMAL);
}

static void test_returning_pong_capability(void)
{
    reset(2);
    tdma_shadow_live_evidence_mask = BIT(0) | BIT(1);
    tdma_shadow_last_seen_ms[0] = tdma_shadow_last_seen_ms[1] = 100;
    tracker_slots[0].tdma_config_packed = tdma_pack_config(0, 2, 16, 3);
    tracker_slots[1].tdma_config_packed = tdma_pack_config(1, 2, 16, 3);
    tracker_slots[0].last_ping_counter = 31;
    tracker_slots[0].ping_counter_initialized = true;
    relay_coord_note_caps(&relay_coord, 0, RELAY_INFO_CAPS);
    relay_coord_note_caps(&relay_coord, 1, RELAY_INFO_CAPS);

    assert(relay_pong_is_extended(ping_ack(0, 32, 0, 1).length));
    assert(ping_ack(0, 0, ESB_PING_FLAG_CHANNEL_CONFIRM, 0).length == ESB_PONG_LEN);
    struct esb_payload native = ping_ack(0, 1, 0, 1);
    assert(native.length == ESB_PONG_LEN && native.data[7] == ESB_PONG_FLAG_NORMAL && native.data[8] == 0);
    assert(relay_coord_is_capable(&relay_coord, 1));
    uint8_t info[16] = {[RELAY_INFO_CAPS_INDEX] = RELAY_INFO_CAPS};
    relay_take_caps(0, info);
    assert(relay_pong_is_extended(ping_ack(0, 2, 0, 1).length));

    /* Legacy may have queued several off-channel startup PINGs before
     * finding us. Retransmissions must not consume the fallback budget. */
    for (uint8_t counter = 41; counter < 48; counter++) {
        assert(relay_pong_is_extended(ping_ack(0, counter, 0, 0).length));
        assert(relay_pong_is_extended(ping_ack(0, counter, 0, 0).length));
    }
    assert(ping_ack(0, 48, 0, 0).length == ESB_PONG_LEN);
    info[RELAY_INFO_CAPS_INDEX] = RELAY_INFO_CAPS;
    relay_take_caps(0, info);

    relay_profile = RELAY_PROFILE_120HZ;
    now += TDMA_SHADOW_LEAVE_GRACE_MS;
    relay_query_protected_mask = BIT(1);
    assert(relay_pong_is_extended(ping_ack(1, 36, ESB_PING_FLAG_CHANNEL_CONFIRM, 0).length));
    struct esb_payload returned = ping_ack(0, 37, 0, 1);
    assert(returned.length == ESB_PONG_LEN && returned.data[7] == ESB_PONG_FLAG_NORMAL && returned.data[8] == 0xff);
    assert(!relay_coord_is_capable(&relay_coord, 0) && relay_coord_is_capable(&relay_coord, 1));
    relay_profile = 0;
    returned = ping_ack(0, 38, 0, 1);
    assert(returned.length == ESB_PONG_LEN && returned.data[8] == 0);
}

static void test_channel_pongs(void) {
    reset(2);
    /* Released trackers see the exact legacy tuple at every physical channel. */
    for (unsigned channel = 0; channel <= 100; ++channel) {
        physical_channel = channel;
        for (unsigned total = 1; total <= 16; ++total) {
            for (unsigned slot = 0; slot < total; ++slot) {
                tracker_slots[0].tdma_config_packed = slot << 24 | total << 16 | 32 << 8 | 0xFE;
                struct esb_payload p = test_pong(0);
                uint8_t legacy[13] = {
                    ESB_PONG_TYPE, 0, 7, 0x12, 0x34, 0x56, 0x78,
                    ESB_PONG_FLAG_NORMAL, slot, total, 32, 0xFE, 0,
                };
                legacy[12] = crc8_ccitt(0x07, legacy, 12);
                assert(!memcmp(p.data, legacy, sizeof(legacy)));
            }
        }
        assert_channel_confirm(test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM), channel);
    }
    uint32_t inactive[] = {0, 0xFF000007, 0xFF102007};
    for (unsigned i = 0; i < sizeof(inactive) / sizeof(inactive[0]); ++i) {
        tracker_slots[0].tdma_config_packed = inactive[i];
        struct esb_payload p = test_pong(0);
        assert(p.data[8] == (uint8_t)(inactive[i] >> 24));
        assert(p.data[9] == (uint8_t)(inactive[i] >> 16));
        assert(p.data[10] == (uint8_t)(inactive[i] >> 8));
        assert(p.data[11] == (uint8_t)inactive[i]);
    }
    /* Actual84, not stored default255 or migration destination85. */
    physical_channel = 84;
    receiver_rf_channel = ESB_RF_CHANNEL_DEFAULT;
    assert(esb_set_all_trackers_channel(85) == 0);
    tracker_slots[0].tracker_remote_command = ESB_PONG_FLAG_MEOW;
    tracker_slots[0].metadata_pending_mask = 1;
    assert_channel_confirm(test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM), 84);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_MEOW);
    assert(tracker_slots[0].metadata_pending_mask == 1 && tracker_slots[0].metadata_active_mask == 0);
    struct esb_payload p = test_pong(0);
    assert(p.data[7] == ESB_PONG_FLAG_SET_CHANNEL);
    assert(p.data[8] == 0 && p.data[9] == 0 && p.data[10] == 0 && p.data[11] == 85);
    /* Flagged echoes retain real low-seven-bit acknowledgements only. */
    echo(0, ESB_PING_FLAG_CHANNEL_CONFIRM);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_MEOW);
    echo(0, ESB_PING_FLAG_CHANNEL_CONFIRM | ESB_PONG_FLAG_MEOW);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_NORMAL);
    reset(1);
    physical_channel = 84;
    abort_pending[0] = true;
    ota = ota_target = ota_payload = true;
    p = test_pong(0);
    assert(p.data[7] == ESB_PONG_FLAG_OTA_ABORT);
    assert(p.data[8] == 0 && p.data[9] == 0 && p.data[10] == 0 && p.data[11] == 0);
    assert_channel_confirm(test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM), 84);
    assert(abort_pending[0]); /* Not consumed by a channel probe. */
    abort_pending[0] = false;
    assert_channel_confirm(test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM), 84);
    /* The next unflagged PING still reaches the existing OTA BEGIN overlay. */
    uint8_t ping[13] = {ESB_PING_TYPE, 0, 7};
    ping[12] = crc8_ccitt(0x07, ping, 12);
    struct esb_payload begin = {0};
    bool has_begin = false, suppress_begin = false;
    esb_ack_handler_cb(ping, sizeof(ping), 1, &begin, &has_begin, &suppress_begin);
    assert(has_begin && !suppress_begin && begin.length == 17 && begin.data[0] == 0xAB);
    /* Controls keep their established argument layouts and queue state. */
    reset(1);
    physical_channel = 84;
    tracker_slots[0].tracker_remote_command = ESB_PONG_FLAG_OTA_QUERY_INFO;
    assert_channel_confirm(test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM), 84);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_OTA_QUERY_INFO);
    assert(test_pong(0).data[7] == ESB_PONG_FLAG_OTA_QUERY_INFO);
    tracker_slots[0].tracker_remote_command = ESB_PONG_FLAG_SENS_SET;
    tracker_slots[0].pending_sens_data[0] = 0x1234;
    tracker_slots[0].pending_sens_data[1] = 0x5678;
    tracker_slots[0].pending_sens_data[2] = 0x2345;
    p = test_pong(0);
    assert(p.data[7] == ESB_PONG_FLAG_SENS_SET);
    assert(p.data[3] == 0x12 && p.data[4] == 0x34);
    assert(p.data[5] == 0x56 && p.data[6] == 0x78);
    assert(p.data[8] == 0x23 && p.data[9] == 0x45);
    assert_channel_confirm(test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM), 84);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_SENS_SET);
    tracker_slots[0].tracker_remote_command = ESB_PONG_FLAG_NORMAL;
    tracker_slots[0].metadata_pending_mask = 3;
    tracker_slots[0].metadata_pending_chunk = 4;
    tracker_slots[0].metadata_pending_token = 0x5678;
    assert_channel_confirm(test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM), 84);
    assert(tracker_slots[0].metadata_pending_mask == 3 && tracker_slots[0].metadata_active_mask == 0);
    p = test_pong(0);
    assert(p.data[7] == ESB_PONG_FLAG_METADATA_REQUEST);
    assert(p.data[8] == 3 && p.data[9] == 4 && p.data[10] == 0x56 && p.data[11] == 0x78);
    assert_channel_confirm(test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM), 84);
    assert(tracker_slots[0].metadata_active_mask == 3 && tracker_slots[0].metadata_active_chunk == 4 &&
           tracker_slots[0].metadata_active_token == 0x5678);
    p = test_pong(0);
    assert(p.data[7] == ESB_PONG_FLAG_METADATA_REQUEST);
    assert(p.data[8] == 3 && p.data[9] == 4 && p.data[10] == 0x56 && p.data[11] == 0x78);
    /* The unchanged eight-byte pairing ACK is still provisional. */
    reset(1);
    esb_pairing = true;
    uint8_t pair[8] = {0, 1, 100};
    pair[0] = crc8_ccitt(0x07, pair + 2, 6);
    if (!pair[0]) pair[0] = 8;
    struct esb_payload ack = {0};
    bool has = false, suppress = false;
    esb_ack_handler_cb(pair, sizeof(pair), 0, &ack, &has, &suppress);
    assert(has && !suppress && ack.length == 8 && ack.data[1] == 0);
    assert(ack.data[0] == pair[0] && !memcmp(ack.data + 2, receiver_device_addr, 6));
    atomic_set(&pairing_new_devices_blocked, 1);
    esb_ack_handler_cb(pair, sizeof(pair), 0, &ack, &has, &suppress);
    assert(has && !suppress && ack.length == 8 && ack.data[1] == 0);
    assert(ack.data[0] == pair[0] && !memcmp(ack.data + 2, receiver_device_addr, 6));
    pair[1] = 0;
    pair[2] = 101;
    pair[0] = crc8_ccitt(0x07, pair + 2, 6);
    if (!pair[0]) pair[0] = 8;
    esb_ack_handler_cb(pair, sizeof(pair), 0, &ack, &has, &suppress);
    assert(!has && suppress);
    atomic_set(&pairing_new_devices_blocked, 0);
    esb_ack_handler_cb(pair, sizeof(pair), 0, &ack, &has, &suppress);
    assert(!has && !suppress);

    /* Full roster: known ID 15 still gets the same ACK, unknowns do not. */
    reset(MAX_TRACKERS);
    esb_pairing = true;
    pair[1] = 1;
    pair[2] = 100 + MAX_TRACKERS - 1;
    pair[0] = crc8_ccitt(0x07, pair + 2, 6);
    if (!pair[0]) pair[0] = 8;
    ack = (struct esb_payload){.noack = true};
    esb_ack_handler_cb(pair, sizeof(pair), 0, &ack, &has, &suppress);
    assert(has && !suppress && ack.length == 8 && ack.pipe == 0 && !ack.noack);
    assert(ack.data[1] == MAX_TRACKERS - 1);
    atomic_set(&pairing_new_devices_blocked, 1);
    esb_ack_handler_cb(pair, sizeof(pair), 0, &ack, &has, &suppress);
    assert(has && !suppress && ack.length == 8 && ack.data[1] == MAX_TRACKERS - 1);
    pair[2] = 100 + MAX_TRACKERS;
    pair[0] = crc8_ccitt(0x07, pair + 2, 6);
    if (!pair[0]) pair[0] = 8;
    esb_ack_handler_cb(pair, sizeof(pair), 0, &ack, &has, &suppress);
    assert(!has && suppress);
    pair[1] = 0;
    esb_ack_handler_cb(pair, sizeof(pair), 0, &ack, &has, &suppress);
    assert(!has && suppress);
    atomic_set(&pairing_new_devices_blocked, 0);
    esb_ack_handler_cb(pair, sizeof(pair), 0, &ack, &has, &suppress);
    assert(!has && !suppress);
    pair[1] = 1;
    esb_ack_handler_cb(pair, sizeof(pair), 0, &ack, &has, &suppress);
    assert(!has && suppress);
    pair[2] = 100 + MAX_TRACKERS - 1;
    pair[0] = crc8_ccitt(0x07, pair + 2, 6);
    if (!pair[0]) pair[0] = 8;
    esb_pairing = false;
    esb_ack_handler_cb(pair, sizeof(pair), 0, &ack, &has, &suppress);
    assert(!has && suppress);
}
static void test_wom_timeouts(void) {
    reset(3);
    uint16_t token, next, minutes = 999, error = 999;
    assert(esb_send_remote_wom_command(0, 35792, &token) == 0);
    assert(esb_send_remote_wom_command(16, 10, &token) == 0);
    assert(esb_send_remote_wom_command(3, 10, &token) == 0);
    assert(esb_send_remote_wom_command(0, 10, &token) == BIT(0) && token != 0);
    struct esb_payload p = test_pong(0);
    assert(p.data[7] == ESB_PONG_FLAG_WOM_CONTROL);
    assert(p.data[3] == 0x12 && p.data[4] == 0x34 && p.data[5] == 0x56 && p.data[6] == 0x78);
    assert(p.data[8] == 0 && p.data[9] == 10 && p.data[10] == (token >> 8) && p.data[11] == (token & 255));
    assert(test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM).data[7] == ESB_PONG_FLAG_CHANNEL_CONFIRM);
    assert(tracker_slots[0].wom_token == token && tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_WOM_CONTROL);
    uint8_t reply[4] = {0, 10, token >> 8, token & 255};
    echo(0, ESB_PONG_FLAG_WOM_CONTROL);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_WOM_CONTROL);
    remote_wom_receive(0, ESB_PONG_FLAG_WOM_CONTROL, reply, 8);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    reply[1] = 0;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    reply[1] = 10;
    reply[3] ^= 1;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    reply[3] ^= 1; reply[1] = 11;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    reply[1] = 10;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 7);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
    assert(esb_get_remote_wom_result(0, token, &minutes, &error) && minutes == 10 && error == 0);
    reply[1] = EIO;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_ERROR, reply, 8);
    assert(esb_get_remote_wom_result(0, token, &minutes, &error) && minutes == 10 && error == 0);
    assert(esb_send_remote_wom_command(0, 0, &next) == BIT(0) && next != token);
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
    assert(!esb_get_remote_wom_result(0, next, &minutes, &error));
    esb_cancel_remote_wom_command(BIT(0), token);
    assert(tracker_slots[0].wom_token == next);
    test_pong(0);
    reply[0] = 0; reply[1] = 0; reply[2] = next >> 8; reply[3] = next & 255;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
    remote_wom_receive(0, ESB_PING_FLAG_WOM_ERROR, reply, 8);
    assert(!esb_get_remote_wom_result(0, next, &minutes, &error));
    reply[0] = 0x8b; reply[1] = 0xd0;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
    assert(!esb_get_remote_wom_result(0, next, &minutes, &error));
    reply[0] = 0x8b; reply[1] = 0xcf; reply[2] = next >> 8; reply[3] = next & 255;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
    assert(esb_get_remote_wom_result(0, next, &minutes, &error) && minutes == 35791 && error == 0);
    assert(esb_send_remote_wom_command(0, 1, &token) == BIT(0));
    test_pong(0);
    reply[0] = 0; reply[1] = EIO; reply[2] = token >> 8; reply[3] = token & 255;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_ERROR, reply, 8);
    assert(esb_get_remote_wom_result(0, token, &minutes, &error) && minutes == 0 && error == EIO);
    assert(esb_send_remote_wom_command(0, 1, &token) == BIT(0));
    esb_send_remote_command(0, ESB_PONG_FLAG_MEOW);
    assert(esb_get_remote_wom_result(0, token, &minutes, &error) && error == ECANCELED);
    esb_cancel_remote_wom_command(BIT(0), token);
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_MEOW && tracker_slots[0].wom_token == 0);
    wom_next_token = UINT16_MAX;
    assert(esb_send_remote_wom_command(0, 1, &token) == BIT(0) && token == 1);
    tracker_slots[0].stats.last_packet_time = now + 1000;
    tracker_slots[1].stats.last_packet_time = now - 1;
    tracker_slots[2].stats.last_packet_time = now + 1000;
    assert(esb_send_remote_wom_command(255, 0, &token) == (BIT(0) | BIT(2)));
    assert(tracker_slots[0].wom_token == token && tracker_slots[2].wom_token == token);
    esb_cancel_remote_wom_command(BIT(0) | BIT(2), token);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    /* A rebooted receiver can reuse a cached tracker token. The first incoming
     * RESULT predates the request carried by its own ACK: it is not proof. */
    wom_next_token = 0;
    assert(esb_send_remote_wom_command(0, 0, &token) == BIT(0) && token == 1);
    uint8_t stale[13] = {ESB_PING_TYPE, 0, 41, 0, 0, 0, 0,
                        ESB_PING_FLAG_WOM_RESULT, 0, 10, 0, 1, 0};
    remote_wom_receive(0, stale[7], &stale[8], stale[2]);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM);
    assert(!tracker_slots[0].wom_advertised);
    abort_pending[0] = true;
    test_pong(0);
    assert(!tracker_slots[0].wom_advertised);
    abort_pending[0] = false;
    struct esb_payload ack = {0};
    bool has = false, suppress = false;
    uint8_t ota_ping[13] = {ESB_PING_TYPE, 0, 40};
    ota_ping[12] = crc8_ccitt(0x07, ota_ping, 12);
    ota = ota_target = ota_payload = true;
    esb_ack_handler_cb(ota_ping, sizeof(ota_ping), 1, &ack, &has, &suppress);
    assert(has && ack.length == 17 && ack.data[0] == 0xAB && !tracker_slots[0].wom_advertised);
    ota = ota_target = ota_payload = false;
    stale[12] = crc8_ccitt(0x07, stale, 12);
    esb_ack_handler_cb(stale, sizeof(stale), 1, &ack, &has, &suppress);
    assert(has && ack.data[7] == ESB_PONG_FLAG_NORMAL && !tracker_slots[0].wom_advertised);
    remote_wom_receive(0, stale[7], &stale[8], stale[2]);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    stale[7] = ESB_PING_FLAG_WOM_ERROR;
    stale[12] = crc8_ccitt(0x07, stale, 12);
    esb_ack_handler_cb(stale, sizeof(stale), 1, &ack, &has, &suppress);
    assert(ack.data[7] == ESB_PONG_FLAG_NORMAL && !tracker_slots[0].wom_advertised);
    remote_wom_receive(0, stale[7], &stale[8], stale[2]);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    stale[7] = ESB_PONG_FLAG_NORMAL;
    stale[12] = crc8_ccitt(0x07, stale, 12);
    esb_ack_handler_cb(stale, sizeof(stale), 1, &ack, &has, &suppress);
    assert(ack.data[7] == ESB_PONG_FLAG_WOM_CONTROL && tracker_slots[0].wom_first_ping_counter == 41);
    stale[7] = ESB_PING_FLAG_WOM_RESULT;
    remote_wom_receive(0, stale[7], &stale[8], stale[2]);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    /* Repeated request encoding must retain the FIRST counter. */
    stale[2]++;
    stale[9] = 20;
    stale[12] = crc8_ccitt(0x07, stale, 12);
    esb_ack_handler_cb(stale, sizeof(stale), 1, &ack, &has, &suppress);
    assert(tracker_slots[0].wom_first_ping_counter == 41);
    remote_wom_receive(0, stale[7], &stale[8], stale[2]);
    assert(esb_get_remote_wom_result(0, token, &minutes, &error) && minutes == 20 && !error);
    assert(esb_send_remote_wom_command(0, ESB_WOM_TIMEOUT_CLEAR, &token) == BIT(0));
    reply[0] = reply[1] = 0; reply[2] = token >> 8; reply[3] = token & 255;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    p = test_pong(0);
    assert(p.data[8] == 255 && p.data[9] == 255);
    remote_wom_receive(0, ESB_PONG_FLAG_WOM_CONTROL, reply, 8);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    reply[1] = 10;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    reply[1] = 0; reply[3] ^= 1;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    reply[3] ^= 1;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 7);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
    assert(esb_get_remote_wom_result(0, token, &minutes, &error) && !minutes && !error);
    assert(esb_send_remote_wom_command(0, ESB_WOM_TIMEOUT_CLEAR, &token) == BIT(0));
    test_pong(0);
    reply[1] = EIO; reply[2] = token >> 8; reply[3] = token & 255;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_ERROR, reply, 8);
    assert(esb_get_remote_wom_result(0, token, &minutes, &error) && !minutes && error == EIO);
    assert(esb_send_remote_wom_command(255, ESB_WOM_TIMEOUT_CLEAR, &token) == (BIT(0) | BIT(2)));
    assert(tracker_slots[0].wom_minutes == ESB_WOM_TIMEOUT_CLEAR &&
           tracker_slots[2].wom_minutes == ESB_WOM_TIMEOUT_CLEAR);
}
static void test_wom_staging_deadline(void)
{
    reset(RELAY_MAX_TRACKERS);
    relay_profile = RELAY_PROFILE_120HZ;
    tdma_active_mask = UINT16_MAX;
    relay_applied_settings = (struct relay_runtime_settings){16, 1, 16, 60, 60, 1, 0};
    relay_coord_note_caps(&relay_coord, 0, RELAY_INFO_CAPS);
    relay_coord_note_caps(&relay_coord, 2, RELAY_INFO_CAPS);
    uint16_t token, earlier, unblocked;
    assert(esb_send_remote_wom_command(2, 0, &earlier) == BIT(2));

    now += 500;
    now_ticks += 32768 / 2;
    uint32_t lead = relay_stage_lead_ticks(&relay_applied_settings, 16);
    uint32_t lead_ms = k_ticks_to_ms_ceil32(lead);
    tdma_staged.p = (struct tdma_publication){.new_mask = UINT16_MAX, .active_count = 16, .slot_ticks = 16};
    tdma_staged.activation_ticks = now_ticks + lead;
    tdma_staged.valid = true;
    unsigned key = irq_lock();
    tdma_defer_wom_locked(UINT32_MAX);
    irq_unlock(key);
    int64_t activation_ms = now + lead_ms;
    assert(lead_ms > ESB_WOM_COMMAND_TIMEOUT_MS);

    now += 1000;
    now_ticks += 32768;
    assert(esb_send_remote_wom_command(0, 0, &token) == BIT(0));
    assert(esb_send_remote_wom_command(1, 0, &unblocked) == BIT(1));
    now += ESB_WOM_COMMAND_TIMEOUT_MS;
    now_ticks += 32768 * ESB_WOM_COMMAND_TIMEOUT_MS / 1000;
    assert(now < activation_ms);
    assert(esb_remote_wom_waiting(0, token) && esb_remote_wom_waiting(2, earlier));
    assert(!esb_remote_wom_waiting(1, unblocked));

    uint8_t ping[ESB_PING_LEN] = {ESB_PING_TYPE, 0, 7};
    ping[12] = crc8_ccitt(0x07, ping, 12);
    struct esb_payload ack = {0};
    bool has = false, suppress = false;
    esb_ack_handler_cb(ping, sizeof(ping), 1, &ack, &has, &suppress);
    assert(has && !suppress && ack.data[7] == ESB_PONG_FLAG_NORMAL);
    assert(!tracker_slots[0].wom_advertised);

    now = activation_ms;
    now_ticks = tdma_staged.activation_ticks;
    esb_ack_handler_cb(ping, sizeof(ping), 1, &ack, &has, &suppress);
    assert(has && !suppress && ack.data[7] == ESB_PONG_FLAG_WOM_CONTROL);
    assert(tracker_slots[0].wom_advertised);
    tdma_staged.valid = false;
    relay_profile = 0;
    now += ESB_WOM_COMMAND_TIMEOUT_MS - 501;
    assert(esb_remote_wom_waiting(2, earlier));
    now++;
    assert(!esb_remote_wom_waiting(2, earlier) && esb_remote_wom_waiting(0, token));
    now = activation_ms + ESB_WOM_COMMAND_TIMEOUT_MS - 1;
    assert(esb_remote_wom_waiting(0, token));
    now++;
    assert(!esb_remote_wom_waiting(0, token));
    /* Already-advertised WOM whose PONG may have been lost still gets a bounded stage extension. */
    assert(esb_send_remote_wom_command(0, 0, &token) == BIT(0));
    uint32_t wom_start = (uint32_t)now;
    struct esb_payload adv = {0};
    esb_ack_handler_cb(ping, sizeof(ping), 1, &adv, &has, &suppress);
    assert(has && adv.data[7] == ESB_PONG_FLAG_WOM_CONTROL && tracker_slots[0].wom_advertised);
    now += 2000;
    now_ticks += 32768 * 2;
    for (unsigned s = 0; s < 3; ++s) {
        tdma_staged.p = (struct tdma_publication){.new_mask = UINT16_MAX, .active_count = 16, .slot_ticks = 16, .profile = 1};
        tdma_staged.activation_ticks = now_ticks + ((RELAY_LAYOUT_ACTIVATION_MS * 32768U + 999U) / 1000U);
        tdma_staged.valid = true;
        key = irq_lock();
        tdma_defer_wom_locked(UINT32_MAX);
        irq_unlock(key);
        if (s < 2) {
            now += RELAY_LAYOUT_ACTIVATION_MS;
            now_ticks = tdma_staged.activation_ticks;
            tdma_staged.valid = false;
            relay_profile = (s == 0) ? 1 : 0;
            esb_ack_handler_cb(ping, sizeof(ping), 1, &adv, &has, &suppress);
            assert(has && adv.data[7] == ESB_PONG_FLAG_WOM_CONTROL && esb_remote_wom_waiting(0, token));
        }
    }
    assert(tracker_slots[0].wom_deadline_ms == wom_start + ESB_WOM_COMMAND_TIMEOUT_MS + RC_LAYOUT_MAX_ACTIVATION_MS);
    assert(tracker_slots[0].wom_stage_budget_ms == 0);
    tdma_staged.valid = false;
    relay_profile = 0;
    esb_cancel_remote_wom_command(BIT(0), token);
    relay_coord_note_caps(&relay_coord, 0, 0);
    relay_coord_note_caps(&relay_coord, 2, 0);
    tdma_active_mask = 0;
    printf("PASS WOM staging: first advertisement follows the %u ms stage and the unblocked timeout is %u ms\n",
        lead_ms, ESB_WOM_COMMAND_TIMEOUT_MS);
}

static void test_wom_enter(void) {
    reset(3);
    uint16_t token, value, error;
    assert(esb_send_remote_wom_command(0, ESB_WOM_ENTER, &token) == BIT(0));
    uint8_t reply[4] = {0xff, 0xfe, token >> 8, token & 255};
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
    assert(!esb_get_remote_wom_result(0, token, &value, &error));
    struct esb_payload p = test_pong(0);
    assert(p.data[7] == ESB_PONG_FLAG_WOM_CONTROL && p.data[8] == 0xff && p.data[9] == 0xfe);
    remote_wom_receive(0, ESB_PONG_FLAG_WOM_CONTROL, reply, 8);
    assert(!esb_get_remote_wom_result(0, token, &value, &error));
    const uint16_t wrong[] = {0, 1, 10, 35791, ESB_WOM_TIMEOUT_CLEAR};
    for (unsigned i = 0; i < sizeof(wrong) / sizeof(wrong[0]); i++) {
        reply[0] = wrong[i] >> 8; reply[1] = wrong[i] & 255;
        remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
        assert(!esb_get_remote_wom_result(0, token, &value, &error));
    }
    reply[0] = 0xff; reply[1] = 0xfe; reply[3] ^= 1;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
    assert(!esb_get_remote_wom_result(0, token, &value, &error));
    reply[3] ^= 1;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 7);
    assert(!esb_get_remote_wom_result(0, token, &value, &error));
    remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
    assert(esb_get_remote_wom_result(0, token, &value, &error) && value == ESB_WOM_ENTER && !error);
    /* ENTER must never be interpreted as a timeout query/set/clear result. */
    const uint16_t operations[] = {0, 10, ESB_WOM_TIMEOUT_CLEAR};
    for (unsigned i = 0; i < sizeof(operations) / sizeof(operations[0]); i++) {
        assert(esb_send_remote_wom_command(0, operations[i], &token) == BIT(0));
        test_pong(0);
        reply[2] = token >> 8; reply[3] = token & 255;
        remote_wom_receive(0, ESB_PING_FLAG_WOM_RESULT, reply, 8);
        assert(!esb_get_remote_wom_result(0, token, &value, &error));
    }
    assert(esb_send_remote_wom_command(0, ESB_WOM_ENTER, &token) == BIT(0));
    test_pong(0);
    reply[0] = 0; reply[1] = EBUSY; reply[2] = token >> 8; reply[3] = token & 255;
    remote_wom_receive(0, ESB_PING_FLAG_WOM_ERROR, reply, 8);
    assert(esb_get_remote_wom_result(0, token, &value, &error) && !value && error == EBUSY);
    tracker_slots[0].stats.last_packet_time = now + 1000;
    tracker_slots[1].stats.last_packet_time = now - 1;
    tracker_slots[2].stats.last_packet_time = now + 1000;
    assert(esb_send_remote_wom_command(255, ESB_WOM_ENTER, &token) == (BIT(0) | BIT(2)));
    assert(tracker_slots[0].wom_minutes == ESB_WOM_ENTER && tracker_slots[2].wom_minutes == ESB_WOM_ENTER);
}
static void test_hid_reserved_pongs(bool arm_first)
{
    reset(16);
    now = now_ticks = 0;
    relay_profile = 1;
    tdma_active_mask = UINT16_MAX;
    relay_applied_settings = (struct relay_runtime_settings){16, 1, 16, 120, 120, 1, 0};
    relay_coord_note_caps(&relay_coord, 0, RELAY_INFO_CAPS);
    remote_confirm_cb = on_remote_confirm;
    assert(remote_hid_begin(0, ESB_PONG_FLAG_SENS_SET, ESB_PONG_FLAG_SENS_SET) == RCV_HID_ST_STARTED);
    uint16_t token = remote_hid_cmd_token;
    hid_status = -1;
    if (arm_first) {
        remote_hid_arm_expected(BIT(0), token);
        assert(hid_timer_due == 5000);
    }
    tdma_staged.valid = true;
    tdma_staged.p = (struct tdma_publication){.new_mask = UINT16_MAX, .active_count = 16, .slot_ticks = 16};
    tdma_staged.activation_ticks = relay_stage_lead_ticks(&relay_applied_settings, 16);
    if (arm_first) rcv_cmd_hid_stage_started();
    else remote_hid_arm_expected(BIT(0), token);
    esb_send_remote_command_sens_tx(0, 1, 2, 3, token);
    for (unsigned ms = 500; ms <= 4500; ms += 1000) {
        now = ms;
        now_ticks = ms * 32768 / 1000;
        uint8_t ping[ESB_PING_LEN] = {ESB_PING_TYPE, 0, 7};
        ping[12] = crc8_ccitt(0x07, ping, 12);
        struct esb_payload ack = {0};
        bool has = false, suppress = false;
        esb_ack_handler_cb(ping, sizeof(ping), 1, &ack, &has, &suppress);
        assert(has && !suppress && ack.data[7] == (ms == 4500 ? ESB_PONG_FLAG_SENS_SET : ESB_PONG_FLAG_NORMAL));
    }
    now = 5000;
    if (hid_timer_due == now) remote_hid_timeout_handler(NULL);
    assert(hid_status == -1 && hid_timer_due == 5000 + k_ticks_to_ms_ceil32(tdma_staged.activation_ticks));
    now = 5500;
    echo(0, ESB_PONG_FLAG_SENS_SET);
    assert(hid_status == RCV_HID_ST_OK && !remote_hid_pending);
    tdma_staged.valid = false;
    assert(remote_hid_begin(0, ESB_PONG_FLAG_SENS_SET, ESB_PONG_FLAG_SENS_SET) == RCV_HID_ST_STARTED);
    remote_hid_arm_expected(BIT(0), remote_hid_cmd_token);
    now += 5000;
    remote_hid_timeout_handler(NULL);
    assert(hid_status == RCV_HID_ST_ENOENT && !remote_hid_pending);
    remote_confirm_cb = NULL;
    tdma_active_mask = relay_profile = 0;
    relay_coord_note_caps(&relay_coord, 0, 0);
}

static void test_consecutive_hid_stages(void)
{
    reset(16);
    now = now_ticks = 0;
    relay_profile = 1;
    tdma_active_mask = UINT16_MAX;
    relay_coord_note_caps(&relay_coord, 0, RELAY_INFO_CAPS);
    relay_applied_settings = (struct relay_runtime_settings){16, 1, 16, 120, 120, 1, 0};
    uint32_t lead = relay_stage_lead_ticks(&relay_applied_settings, 16);
    uint32_t bound = REMOTE_HID_CONFIRM_TIMEOUT_MS + RC_LAYOUT_MAX_ACTIVATION_MS;
    tdma_staged.valid = true;
    tdma_staged.p = (struct tdma_publication){.new_mask = UINT16_MAX, .profile = 1};
    tdma_staged.activation_ticks = lead;
    hid_status = -1;
    assert(rcv_cmd_remote_sens_set_hid(1, ESB_PONG_FLAG_SENS_SET, 0, 1, 2, 3) == RCV_HID_ST_STARTED);
    /* Start another stage at each activation and service the timer when due. */
    for (uint32_t ms = 1; ms <= bound; ++ms) {
        now = ms;
        now_ticks = ms * 32768 / 1000;
        if (now_ticks >= tdma_staged.activation_ticks) {
            tdma_staged.activation_ticks = now_ticks + lead;
            unsigned key = irq_lock();
            rcv_cmd_hid_stage_started();
            irq_unlock(key);
        }
        if (hid_timer_due == now) remote_hid_timeout_handler(NULL);
        if (now < bound) assert(remote_hid_pending && hid_status == -1);
    }
    assert(!remote_hid_pending && hid_status == RCV_HID_ST_ENOENT);
    /* The expired HID command is withdrawn rather than left for a late PING. */
    assert(tracker_slots[0].tracker_remote_command == ESB_PONG_FLAG_NORMAL);
    assert(rcv_cmd_remote_sens_set_hid(2, ESB_PONG_FLAG_SENS_SET, 0, 4, 5, 6) == RCV_HID_ST_STARTED);
    assert(hid_timer_due == now + REMOTE_HID_CONFIRM_TIMEOUT_MS + esb_reserved_pong_ms(BIT(0)));
    remote_hid_abort();
    tdma_staged.valid = false;
    tdma_active_mask = relay_profile = 0;
    relay_coord_note_caps(&relay_coord, 0, 0);
    printf("PASS consecutive HID stages: ENOENT at %u ms, next command STARTED\n", bound);
}

static void test_blocked_pose_receipt(void)
{
    reset(2);
    relay_profile = 1;
    tdma_active_mask = BIT(0);
    relay_origin = relay_frame_anchor = relay_start_anchor = 0;
    now_ticks = RELAY_AIR_TICKS(17);
    uint8_t pose[17] = {1, 0};
    struct esb_payload ack = {0};
    bool has = false, suppress = false;
    relay_coord.receipt[0] = (struct rc_receipt){0};
    assert(esb_relay_block_mask(BIT(0), 1));
    esb_ack_handler_cb(pose, sizeof(pose), 1, &ack, &has, &suppress);
    assert(!(relay_coord.receipt[0].received & BIT(0)));
    uint32_t frame = 150; /* Same pose on a valid slot after the one-second block. */
    now_ticks = relay_frame_start(relay_origin, frame) + RELAY_AIR_TICKS(sizeof(pose));
    now += k_ticks_to_ms_ceil32(now_ticks);
    esb_ack_handler_cb(pose, sizeof(pose), 1, &ack, &has, &suppress);
    assert(relay_coord.receipt[frame % 3].frame == frame &&
           (relay_coord.receipt[frame % 3].received & BIT(0)));
    /* Partial-loss injector: kept sequences still record early receipt without bumping blocked. */
    relay_block_id = 0;
    relay_block_percent = 50;
    uint32_t blocked_before = relay_rx_stats.blocked;
    bool saw_kept = false, saw_dropped = false;
    for (unsigned seq = 0; seq < 32 && !(saw_kept && saw_dropped); ++seq) {
        frame++;
        now_ticks = relay_frame_start(relay_origin, frame) + RELAY_AIR_TICKS(sizeof(pose));
        pose[16] = (uint8_t)seq;
        relay_coord.receipt[frame % 3] = (struct rc_receipt){0};
        esb_ack_handler_cb(pose, sizeof(pose), 1, &ack, &has, &suppress);
        bool kept = !relay_block_would_drop(0, (uint8_t)seq);
        assert(((relay_coord.receipt[frame % 3].received & BIT(0)) != 0) == kept);
        saw_kept |= kept;
        saw_dropped |= !kept;
    }
    assert(saw_kept && saw_dropped && relay_rx_stats.blocked == blocked_before);
    relay_block_id = 0xFF;
    relay_block_percent = 0;
    relay_profile = tdma_active_mask = 0;
    assert(esb_relay_block_mask(0, 0));
}

int main(void) {
    test_blocked_pose_receipt();
    test_consecutive_hid_stages();
    test_hid_reserved_pongs(false);
    test_hid_reserved_pongs(true);
    puts("PASS HID staging: four reserved PONGs, success at 5500 ms, ordinary timeout at 5000 ms");
    test_command_publication();
    test_wom_enter();
    test_wom_timeouts();
    test_wom_staging_deadline();
    test_channel_pongs();
    test_query_confirmation();
    test_returning_pong_capability();
    /* Production channel_apply discards queued old-home packets before
     * resetting admission, including storage-error rollback. */
    for (unsigned rollback = 0; rollback < 2; ++rollback) {
        reset(2);
        for (unsigned i = 0; i < MAX_TRACKERS; ++i) {
            tracker_slots[i].sequence_initialized = false;
            memset(&tracker_slots[i].stats, 0, sizeof(tracker_slots[i].stats));
        }
        assert(check_packet_sequence(0, 100) == 0);
        assert(check_packet_sequence(1, 255) == 0);
        assert(esb_set_all_trackers_channel(26) == 0);
        stale_fifo = true;
        fail_store = rollback;
        deadline();
        assert(!stale_fifo && rx_running);
        assert(physical_channel == (rollback ? 22 : 26));
        assert(check_packet_sequence(0, 64) == 0);
        assert(check_packet_sequence(0, 64) == 4);
        assert(check_packet_sequence(0, 63) == 2);
        assert(check_packet_sequence(1, 220) == 0);
        assert(check_packet_sequence(1, 220) == 4);
        assert(tracker_slots[0].stats.total_received == 2);
        assert(tracker_slots[0].stats.total_gaps == 0 && tracker_slots[0].stats.restart_events == 0);
        assert(tracker_slots[1].stats.total_received == 2);
    }
    /* No live traffic, partial traffic, all traffic: delivery is not a barrier. */
    for (unsigned count = 0; count <= 3; count++) {
        for (unsigned online = 0; online <= count; online++) {
            reset(count);
            assert(esb_set_all_trackers_channel(100) == 0);
            assert(!auto_enabled);
            for (unsigned i = 0; i < online; i++) assert_wire(i, 10, 100);
            now = channel_change_timeout - 1;
            channel_change_poll();
            assert(!completed && physical_channel == 22);
            deadline();
            assert(completed == 1 && succeeded && physical_channel == 100 && durable_channel == 100);
            assert(auto_completed == 0);
            assert(!esb_channel_is_busy() && !channel_change_pending && rx_running);
            for (unsigned i = 0; i < count; i++) assert_wire(i, 0, 0);
            channel_change_poll();
            assert(completed == 1);
        }
    }
    /* Generic pending commands are neither consumed nor zero-channel corrupted. */
    reset(3);
    esb_send_remote_command_arg(0, 4, 77);
    assert(esb_set_all_trackers_channel(31) == 0);
    assert(esb_set_all_trackers_channel(32) == -EBUSY);
    assert(esb_set_receiver_channel(32) == -EBUSY);
    assert(esb_clear_receiver_channel() == -EBUSY);
    esb_send_remote_command_arg(1, 7, 0);
    esb_send_remote_command_arg(0, 10, 0); /* Reject raw channel opcode injection. */
    assert_wire(0, 10, 31);
    assert_wire(1, 10, 31);
    assert(tracker_slots[0].tracker_remote_command == 4 && tracker_slots[0].pending_cmd_arg == 77);
    echo(0, ESB_PONG_FLAG_SET_CHANNEL); /* No echo can end the announcement. */
    channel_change_poll();
    assert(completed == 0 && physical_channel == 22);
    deadline();
    assert_wire(0, 4, 0);
    assert_wire(1, 7, 0);
    /* Repeated destination/new destination/clear do not reuse prior completion. */
    assert(esb_set_all_trackers_channel(31) == 0);
    assert_wire(0, 10, 31);
    channel_change_poll();
    assert(completed == 1);
    deadline();
    assert(completed == 2 && succeeded);
    assert(esb_set_all_trackers_channel(0) == 0);
    echo(0, ESB_PONG_FLAG_SET_CHANNEL); /* Late echo from the old transaction. */
    assert_wire(0, 10, 0);
    assert_wire(2, 10, 0);
    deadline();
    assert(physical_channel == 0 && durable_channel == 128);
    assert(esb_clear_all_trackers_channel() == 0);
    assert_wire(2, 11, 0);
    deadline();
    assert(physical_channel == RADIO_RF_CHANNEL && durable_channel == 255);
    /* Snapshot doesn't grow/shrink with stored count while announcing. */
    reset(2);
    assert(esb_set_all_trackers_channel(55) == 0);
    stored_trackers = 3;
    stored_tracker_addr[2] = 200;
    assert_wire(2, 0, 0);
    stored_trackers = 0;
    deadline();
    assert(succeeded && physical_channel == 55);
    /* First failed stop, tuning, start or persistence never reports success. */
    for (unsigned failure = 0; failure < 4; failure++) {
        reset(2);
        assert(esb_set_all_trackers_channel(1) == 0);
        if (failure == 0) fail_stop = 1;
        if (failure == 1) fail_set = 1;
        if (failure == 2) fail_start = 1;
        if (failure == 3) fail_store = 1;
        deadline();
        assert(completed == 1 && !succeeded && !esb_channel_is_busy());
        assert(physical_channel == 22 && durable_channel == 22 && receiver_rf_channel == 22 && rx_running);
        assert_wire(0, 0, 0);
    }
    reset(1);
    assert(esb_set_all_trackers_channel(1) == 0);
    fail_set = 2;
    deadline();
    assert(!succeeded && fault_status && !rx_running);
    /* All incompatible transfer/owner states exclude admission. */
    reset(1);
    bool *busy[] = {&ota, &self_ota, &collecting, &batch, &abort_pending[0]};
    for (unsigned i = 0; i < sizeof(busy) / sizeof(*busy); i++) {
        *busy[i] = true;
        assert(esb_set_all_trackers_channel(3) == -EBUSY);
        assert(esb_set_receiver_channel(3) == -EBUSY);
        *busy[i] = false;
    }
    assert(esb_radio_operation_begin() == 0);
    assert(esb_set_all_trackers_channel(3) == -EBUSY);
    esb_radio_operation_end();
    assert(esb_set_all_trackers_channel(101) == -EINVAL);
    auto_enabled = false;
    assert(esb_auto_channel_request(88, 22, auto_generation) == -EBUSY);
    auto_enabled = true;
    assert(esb_auto_channel_request(88, 22, auto_generation + 1) == -EBUSY);
    assert(esb_auto_channel_request(88, 23, auto_generation) == -EBUSY);
    esb_pairing = true;
    assert(esb_auto_channel_request(88, 22, auto_generation) == -EBUSY);
    esb_pairing = false;
    for (unsigned ch = 0; ch <= 100; ch++) {
        if ((ch & 1) || ch > 88)
            assert(esb_auto_channel_request(ch, 22, auto_generation) == -EINVAL);
    }
    /* 90 is even but outside the automatic destination range; 88 is the highest. */
    assert(esb_auto_channel_request(90, 22, auto_generation) == -EINVAL);
    assert(esb_auto_channel_request(89, 22, auto_generation) == -EINVAL);
    assert(esb_auto_channel_request(88, 22, auto_generation) == 0 && auto_enabled);
    assert(auto_completed == 0);
    deadline();
    assert(succeeded && esb_get_receiver_channel() == 88);
    assert(auto_completed == 1 && auto_succeeded);
    assert(completed_generation == auto_generation && completed_old_home == 22 && completed_new_home == 88);
    assert(esb_auto_channel_request(0, 88, auto_generation) == 0);
    deadline();
    assert(succeeded && durable_channel == 128);
    assert(esb_set_receiver_channel(99) == 0);
    assert(esb_get_receiver_channel() == 99);
    assert(esb_set_receiver_channel(0) == 0 && !auto_enabled);
    assert(esb_get_receiver_channel() == 0 && durable_channel == 128);
    assert(esb_clear_receiver_channel() == 0 && durable_channel == 255);
    /* No-op automatic requests never count as useful moves. */
    reset(1);
    assert(esb_auto_channel_request(22, 22, auto_generation) == 0);
    deadline();
    assert(succeeded && auto_completed == 1 && !auto_succeeded);
    /* Failure (including failed rollback) publishes failure with original tag. */
    for (unsigned failure = 0; failure < 5; ++failure) {
        reset(1);
        assert(esb_auto_channel_request(88, 22, auto_generation) == 0);
        if (failure == 0) fail_stop = 1;
        if (failure == 1) fail_set = 1;
        if (failure == 2) fail_start = 1;
        if (failure == 3) fail_store = 1;
        if (failure == 4) fail_set = 2;
        deadline();
        assert(auto_completed == 1 && !auto_succeeded && !succeeded);
        assert(completed_old_home == 22 && completed_new_home == 22);
        assert(esb_radio_fault == (failure == 4));
        assert(esb_channel_is_busy() == (failure == 4));
        if (failure == 4) assert(esb_radio_operation_begin() == -EIO);
    }
    /* Cancellation during an accepted announcement keeps the original tag;
     * the worker can reject it rather than associating it with the new intent. */
    reset(1);
    uint32_t original_generation = auto_generation;
    assert(esb_auto_channel_request(88, 22, original_generation) == 0);
    auto_channel_set_enabled(false);
    auto_generation++;
    deadline();
    assert(auto_completed == 1 && completed_generation == original_generation);
    assert(esb_set_all_trackers_channel(40) == 0);
    deadline();
    assert(auto_completed == 1 && physical_channel == 40);
    reset(3);
    assert(rcv_cmd_collect_start(0) == RCV_HID_ST_OK && collecting && !batch);
    assert(auto_loss_reset_epoch != 0);
    assert(esb_set_all_trackers_channel(33) == -EBUSY);
    ota = true;
    assert(rcv_cmd_collect_start(1) == RCV_HID_ST_EBUSY);
    assert(collecting && collecting_id == 0); /* failed replacement preserves old session */
    ota = false;
    assert(rcv_cmd_collect_batch_start(50) == RCV_HID_ST_OK && !collecting && batch);
    assert(esb_set_receiver_channel(33) == -EBUSY);
    assert(rcv_cmd_collect_batch_stop() == RCV_HID_ST_OK && !batch);
    assert(esb_set_all_trackers_channel(33) == 0);
    assert(rcv_cmd_collect_start(0) == RCV_HID_ST_EBUSY && !collecting);
    assert(rcv_cmd_collect_batch_start(50) == RCV_HID_ST_EBUSY && !batch);
    deadline();
    /* Whole raw/batch sessions without advancing time invalidate evidence. */
    uint32_t epoch = auto_loss_reset_epoch;
    assert(rcv_cmd_collect_start(0) == RCV_HID_ST_OK);
    assert(rcv_cmd_collect_stop() == RCV_HID_ST_OK);
    assert(!collecting && auto_loss_reset_epoch != epoch);
    epoch = auto_loss_reset_epoch;
    assert(rcv_cmd_collect_batch_start(50) == RCV_HID_ST_OK);
    assert(rcv_cmd_collect_batch_stop() == RCV_HID_ST_OK);
    assert(!batch && auto_loss_reset_epoch != epoch);
    for (unsigned mode = 0; mode < 4; mode++) {
        reset(3);
        tdma_staged.native_followup = false;
        tdma_staged.native_guard_until_ms = 0;
        relay_profile = (mode == 0);
        tdma_staged.valid = (mode == 1 || mode == 2);
        tdma_staged.p.profile = (mode == 1);
        tdma_staged.native_followup = (mode == 2);
        tdma_staged.native_guard_until_ms = (mode == 3) ? now + RELAY_LAYOUT_ACTIVATION_MS : 0;
        if (mode < 2) {
            uint32_t before = auto_loss_reset_epoch;
            assert(rcv_cmd_collect_start(0) == RCV_HID_ST_EBUSY);
            assert(rcv_cmd_collect_batch_start(50) == RCV_HID_ST_EBUSY);
            assert(!radio_operation_busy && !collecting && !batch && auto_loss_reset_epoch == before);
            assert(test_tracker_pong(1, 0).data[7] == ESB_PONG_FLAG_NORMAL);
        }
        assert(esb_set_all_trackers_channel(33) == -EBUSY);
        assert(auto_enabled && !radio_operation_busy && !channel_change_pending);
        relay_profile = 0;
        tdma_staged.valid = false;
        tdma_staged.native_followup = false;
        tdma_staged.native_guard_until_ms = 0;
        assert(esb_set_all_trackers_channel(33) == 0);
        assert(!auto_enabled);
        deadline();
        assert(completed == 1 && succeeded && physical_channel == 33);
        assert(!esb_channel_is_busy());
    }
    test_hid_timeout_cancellation();
    test_receiver_owned_ota_coalescing();
    test_hid_then_console_coalescing();
    test_broadcast_publication_confirmations();
    test_sticky_early_confirmations();
    test_sticky_armed_confirmations();
    test_empty_scan_finish_isolation();
    puts("PASS receiver channel migration: offline/partial/all, fixed deadline, pending command isolation, snapshot, repeat/clear/zero, RF/storage rollback and exclusion");
    return 0;
}
