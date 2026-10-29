#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#define MAX_TRACKERS 16
#define BIT(i) (1u << (i))
#define K_FOREVER 0
#define LOG_INF(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define TDMA_ENABLED 0
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
typedef void (*esb_channel_change_done_cb_t)(bool);
typedef void (*esb_remote_confirm_cb_t)(uint8_t, uint8_t);
static int atomic_get(atomic_t *p) { return *p; }
static void atomic_set(atomic_t *p, int v) { *p = v; }
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
static void data_collect_start(uint8_t id) { collecting = true; collecting_id = id; }
static void data_collect_stop(void) { collecting = false; }
static uint8_t data_collect_get_target_id(void) { return collecting_id; }
static void data_collect_batch_start(uint32_t mask, uint16_t rate) { batch = true; collecting_mask = mask; }
static void data_collect_batch_stop(void) { batch = false; }
static bool data_collect_batch_is_target(uint8_t id) { return (collecting_mask & BIT(id)) != 0; }
static uint8_t stored_trackers, receiver_rf_channel, physical_channel, durable_channel;
static uint64_t stored_tracker_addr[MAX_TRACKERS];
static uint8_t tracker_remote_command[MAX_TRACKERS], pending_cmd_arg[MAX_TRACKERS];
static bool last_raw_valid[MAX_TRACKERS];
static void raw_arq_reset(void) {}
static bool rx_running, fault_status, auto_enabled;
static bool esb_pairing;
static uint32_t auto_generation;
static uint32_t auto_loss_reset_epoch;
static bool sequence_initialized[MAX_TRACKERS];
static uint8_t last_packet_sequence[MAX_TRACKERS];
static atomic_t tdma_rx_discontinuity_pending;
static bool stale_fifo;
static int esb_flush_rx(void) { assert(!rx_running); stale_fifo = false; return 0; }
static bool auto_channel_decision_valid(uint32_t generation) {
    return auto_enabled && auto_generation == generation;
}
static unsigned int irq_lock(void) { return 0; }
static void irq_unlock(unsigned int key) {}
static uint16_t tracker_test_on_tps[MAX_TRACKERS];
static atomic_t test_all_state_valid, test_all_enabled, test_all_confirmed_mask;
static uint16_t test_all_wire_tps(void) { return 0; }
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
static uint32_t tdma_config_packed[MAX_TRACKERS];
static uint8_t metadata_active_mask[MAX_TRACKERS], metadata_pending_mask[MAX_TRACKERS];
static uint8_t metadata_active_chunk[MAX_TRACKERS], metadata_pending_chunk[MAX_TRACKERS];
static uint16_t metadata_active_token[MAX_TRACKERS], metadata_pending_token[MAX_TRACKERS];
static bool wom_advertised[MAX_TRACKERS];
static uint8_t wom_first_ping_counter[MAX_TRACKERS];
static uint16_t wom_next_token, wom_token[MAX_TRACKERS], wom_minutes[MAX_TRACKERS], wom_error[MAX_TRACKERS];
static bool wom_complete[MAX_TRACKERS];
static int16_t pending_sens_data[MAX_TRACKERS][3];
static uint8_t pending_sens_auto_axis[MAX_TRACKERS];
static uint16_t pending_sens_auto_revolutions[MAX_TRACKERS];
static uint32_t g_last_isr_rx_ticks[MAX_TRACKERS], g_ping_isr_rx_ticks[MAX_TRACKERS];
static bool g_last_isr_rx_valid[MAX_TRACKERS], g_ping_isr_rx_ticks_valid[MAX_TRACKERS];
struct tdma_stats { uint32_t count; };
static struct tdma_stats g_tdma_stats[MAX_TRACKERS];
static int32_t g_last_ping_rx_time_diff_ticks[MAX_TRACKERS];
static bool g_last_ping_rx_time_diff_valid[MAX_TRACKERS];
static uint8_t g_bias_ppb_valid[MAX_TRACKERS], g_ping_dirty_streak[MAX_TRACKERS];
static int32_t g_bias_ppb[MAX_TRACKERS], g_bias_ref_offset[MAX_TRACKERS];
static uint32_t g_bias_ref_ticks[MAX_TRACKERS], g_last_ping_isr_rx_ticks_raw[MAX_TRACKERS];
static uint32_t g_ping_reject_count[MAX_TRACKERS], pipe_mismatch_count[MAX_TRACKERS], crc_error_count[MAX_TRACKERS];
static uint8_t last_ping_counter[MAX_TRACKERS], last_pong_queued_counter[MAX_TRACKERS], echo_counter[MAX_TRACKERS];
static bool ping_counter_initialized[MAX_TRACKERS];
static uint64_t last_ping_time[MAX_TRACKERS];
static esb_remote_confirm_cb_t remote_confirm_cb;
enum tdma_shadow_evidence { TDMA_SHADOW_EVIDENCE_PING, TDMA_SHADOW_EVIDENCE_DATA };
static void tdma_shadow_observe(uint8_t id, enum tdma_shadow_evidence evidence) {}
static uint32_t k_ticks_to_us_floor32(uint32_t ticks) { return ticks; }
static void test_all_invalidate_tracker(uint8_t id, int64_t at) {}
static const char *esb_pong_flag_name(uint8_t flag) { return "command"; }
static uint32_t sys_get_be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static bool pairing_new_devices_blocked, ota_target, ota_payload;
static uint8_t receiver_device_addr[6] = {1, 2, 3, 4, 5, 6};
static int channel_get_error;
static uint32_t k_uptime_ticks(void) { return 0x12345678; }
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
    tracker_store_lock = 0;
    stored_trackers = count;
    memset(stored_tracker_addr, 0, sizeof(stored_tracker_addr));
    memset(tracker_remote_command, 0, sizeof(tracker_remote_command));
    memset(abort_pending, 0, sizeof(abort_pending));
    memset(metadata_active_mask, 0, sizeof(metadata_active_mask));
    memset(metadata_pending_mask, 0, sizeof(metadata_pending_mask));
    memset(echo_counter, 0, sizeof(echo_counter));
    memset(ping_counter_initialized, 0, sizeof(ping_counter_initialized));
    memset(last_ping_time, 0, sizeof(last_ping_time));
    memset(g_last_ping_rx_time_diff_valid, 0, sizeof(g_last_ping_rx_time_diff_valid));
    memset(g_bias_ppb_valid, 0, sizeof(g_bias_ppb_valid));
    memset(g_ping_dirty_streak, 0, sizeof(g_ping_dirty_streak));
    memset(tdma_config_packed, 0, sizeof(tdma_config_packed));
    ota_target = ota_payload = pairing_new_devices_blocked = false;
    channel_get_error = 0;
    for (unsigned i = 0; i < count; i++) stored_tracker_addr[i] = 100 + i;
    receiver_rf_channel = physical_channel = durable_channel = 22;
    rx_running = auto_enabled = true;
    fault_status = ota = self_ota = collecting = batch = false;
    fail_stop = fail_set = fail_start = fail_store = 0;
    now = 100;
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
static struct esb_payload test_pong(uint8_t flags) {
    uint8_t ping[ESB_PING_LEN] = {ESB_PING_TYPE, 0, 7};
    ping[7] = flags;
    ping[12] = crc8_ccitt(0x07, ping, 12);
    struct esb_payload ack = {0};
    bool has = false, suppress = false;
    esb_ack_handler_cb(ping, sizeof(ping), 1, &ack, &has, &suppress);
    assert(has && !suppress);
    assert(ack.length == 13 && ack.data[0] == ESB_PONG_TYPE);
    assert(ack.data[1] == 0 && ack.data[2] == 7);
    assert(ack.data[12] == crc8_ccitt(0x07, ack.data, 12));
    return ack;
}
static void assert_channel_confirm(struct esb_payload p, uint8_t channel) {
    assert(p.data[7] == ESB_PONG_FLAG_CHANNEL_CONFIRM);
    assert(p.data[8] == channel && p.data[9] == ESB_CHANNEL_CONFIRM_VERSION);
    assert(p.data[10] == 0 && p.data[11] == 0);
    assert(p.data[3] == 0x12 && p.data[4] == 0x34 &&
           p.data[5] == 0x56 && p.data[6] == 0x78);
}
static void test_channel_pongs(void) {
    reset(2);
    /* Released trackers see the exact legacy tuple at every physical channel. */
    for (unsigned channel = 0; channel <= 100; ++channel) {
        physical_channel = channel;
        for (unsigned total = 1; total <= 16; ++total) {
            for (unsigned slot = 0; slot < total; ++slot) {
                tdma_config_packed[0] = slot << 24 | total << 16 | 32 << 8 | 0xFE;
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
        tdma_config_packed[0] = inactive[i];
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
    tracker_remote_command[0] = ESB_PONG_FLAG_MEOW;
    metadata_pending_mask[0] = 1;
    assert_channel_confirm(test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM), 84);
    assert(tracker_remote_command[0] == ESB_PONG_FLAG_MEOW);
    assert(metadata_pending_mask[0] == 1 && metadata_active_mask[0] == 0);
    struct esb_payload p = test_pong(0);
    assert(p.data[7] == ESB_PONG_FLAG_SET_CHANNEL);
    assert(p.data[8] == 0 && p.data[9] == 0 && p.data[10] == 0 && p.data[11] == 85);
    /* Flagged echoes retain real low-seven-bit acknowledgements only. */
    echo(0, ESB_PING_FLAG_CHANNEL_CONFIRM);
    assert(tracker_remote_command[0] == ESB_PONG_FLAG_MEOW);
    echo(0, ESB_PING_FLAG_CHANNEL_CONFIRM | ESB_PONG_FLAG_MEOW);
    assert(tracker_remote_command[0] == ESB_PONG_FLAG_NORMAL);
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
    tracker_remote_command[0] = ESB_PONG_FLAG_OTA_QUERY_INFO;
    assert_channel_confirm(test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM), 84);
    assert(tracker_remote_command[0] == ESB_PONG_FLAG_OTA_QUERY_INFO);
    assert(test_pong(0).data[7] == ESB_PONG_FLAG_OTA_QUERY_INFO);
    tracker_remote_command[0] = ESB_PONG_FLAG_SENS_SET;
    pending_sens_data[0][0] = 0x1234;
    pending_sens_data[0][1] = 0x5678;
    pending_sens_data[0][2] = 0x2345;
    p = test_pong(0);
    assert(p.data[7] == ESB_PONG_FLAG_SENS_SET);
    assert(p.data[3] == 0x12 && p.data[4] == 0x34);
    assert(p.data[5] == 0x56 && p.data[6] == 0x78);
    assert(p.data[8] == 0x23 && p.data[9] == 0x45);
    assert_channel_confirm(test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM), 84);
    assert(tracker_remote_command[0] == ESB_PONG_FLAG_SENS_SET);
    tracker_remote_command[0] = ESB_PONG_FLAG_NORMAL;
    metadata_pending_mask[0] = 3;
    metadata_pending_chunk[0] = 4;
    metadata_pending_token[0] = 0x5678;
    assert_channel_confirm(test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM), 84);
    assert(metadata_pending_mask[0] == 3 && metadata_active_mask[0] == 0);
    p = test_pong(0);
    assert(p.data[7] == ESB_PONG_FLAG_METADATA_REQUEST);
    assert(p.data[8] == 3 && p.data[9] == 4 && p.data[10] == 0x56 && p.data[11] == 0x78);
    assert_channel_confirm(test_pong(ESB_PING_FLAG_CHANNEL_CONFIRM), 84);
    assert(metadata_active_mask[0] == 3 && metadata_active_chunk[0] == 4 &&
           metadata_active_token[0] == 0x5678);
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
    assert(wom_token[0] == token && tracker_remote_command[0] == ESB_PONG_FLAG_WOM_CONTROL);
    uint8_t reply[4] = {0, 10, token >> 8, token & 255};
    echo(0, ESB_PONG_FLAG_WOM_CONTROL);
    assert(tracker_remote_command[0] == ESB_PONG_FLAG_WOM_CONTROL);
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
    assert(wom_token[0] == next);
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
    assert(tracker_remote_command[0] == ESB_PONG_FLAG_MEOW && wom_token[0] == 0);
    wom_next_token = UINT16_MAX;
    assert(esb_send_remote_wom_command(0, 1, &token) == BIT(0) && token == 1);
    tracker_stats[0].last_packet_time = now + 1000;
    tracker_stats[1].last_packet_time = now - 1;
    tracker_stats[2].last_packet_time = now + 1000;
    assert(esb_send_remote_wom_command(255, 0, &token) == (BIT(0) | BIT(2)));
    assert(wom_token[0] == token && wom_token[2] == token);
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
    assert(!wom_advertised[0]);
    abort_pending[0] = true;
    test_pong(0);
    assert(!wom_advertised[0]);
    abort_pending[0] = false;
    struct esb_payload ack = {0};
    bool has = false, suppress = false;
    uint8_t ota_ping[13] = {ESB_PING_TYPE, 0, 40};
    ota_ping[12] = crc8_ccitt(0x07, ota_ping, 12);
    ota = ota_target = ota_payload = true;
    esb_ack_handler_cb(ota_ping, sizeof(ota_ping), 1, &ack, &has, &suppress);
    assert(has && ack.length == 17 && ack.data[0] == 0xAB && !wom_advertised[0]);
    ota = ota_target = ota_payload = false;
    stale[12] = crc8_ccitt(0x07, stale, 12);
    esb_ack_handler_cb(stale, sizeof(stale), 1, &ack, &has, &suppress);
    assert(has && ack.data[7] == ESB_PONG_FLAG_NORMAL && !wom_advertised[0]);
    remote_wom_receive(0, stale[7], &stale[8], stale[2]);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    stale[7] = ESB_PING_FLAG_WOM_ERROR;
    stale[12] = crc8_ccitt(0x07, stale, 12);
    esb_ack_handler_cb(stale, sizeof(stale), 1, &ack, &has, &suppress);
    assert(ack.data[7] == ESB_PONG_FLAG_NORMAL && !wom_advertised[0]);
    remote_wom_receive(0, stale[7], &stale[8], stale[2]);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    stale[7] = ESB_PONG_FLAG_NORMAL;
    stale[12] = crc8_ccitt(0x07, stale, 12);
    esb_ack_handler_cb(stale, sizeof(stale), 1, &ack, &has, &suppress);
    assert(ack.data[7] == ESB_PONG_FLAG_WOM_CONTROL && wom_first_ping_counter[0] == 41);
    stale[7] = ESB_PING_FLAG_WOM_RESULT;
    remote_wom_receive(0, stale[7], &stale[8], stale[2]);
    assert(!esb_get_remote_wom_result(0, token, &minutes, &error));
    /* Repeated request encoding must retain the FIRST counter. */
    stale[2]++;
    stale[9] = 20;
    stale[12] = crc8_ccitt(0x07, stale, 12);
    esb_ack_handler_cb(stale, sizeof(stale), 1, &ack, &has, &suppress);
    assert(wom_first_ping_counter[0] == 41);
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
    assert(wom_minutes[0] == ESB_WOM_TIMEOUT_CLEAR && wom_minutes[2] == ESB_WOM_TIMEOUT_CLEAR);
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
    tracker_stats[0].last_packet_time = now + 1000;
    tracker_stats[1].last_packet_time = now - 1;
    tracker_stats[2].last_packet_time = now + 1000;
    assert(esb_send_remote_wom_command(255, ESB_WOM_ENTER, &token) == (BIT(0) | BIT(2)));
    assert(wom_minutes[0] == ESB_WOM_ENTER && wom_minutes[2] == ESB_WOM_ENTER);
}
int main(void) {
    test_wom_enter();
    test_wom_timeouts();
    test_channel_pongs();
    /* Production channel_apply discards queued old-home packets before
     * resetting admission, including storage-error rollback. */
    for (unsigned rollback = 0; rollback < 2; ++rollback) {
        reset(2);
        memset(sequence_initialized, 0, sizeof(sequence_initialized));
        memset(tracker_stats, 0, sizeof(tracker_stats));
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
        assert(tracker_stats[0].total_received == 2);
        assert(tracker_stats[0].total_gaps == 0 && tracker_stats[0].restart_events == 0);
        assert(tracker_stats[1].total_received == 2);
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
    assert(tracker_remote_command[0] == 4 && pending_cmd_arg[0] == 77);
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
    puts("PASS receiver channel migration: offline/partial/all, fixed deadline, pending command isolation, snapshot, repeat/clear/zero, RF/storage rollback and exclusion");
    return 0;
}
