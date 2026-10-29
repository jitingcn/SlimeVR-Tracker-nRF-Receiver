#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_TRACKERS 16
#define BIT(n) (1u << (n))
#define ARG_UNUSED(x) ((void)(x))
#define TRACKER_EVENT_ESB_TYPE 0xe0
#define TRACKER_EVENT_ESB_LEN 17
#define ESB_EVENT_TX_SUCCESS 1
#define ESB_EVENT_TX_FAILED 2
#define ESB_EVENT_RX_RECEIVED 3
#define TDMA_ENABLED 1
#define PING_TIMEOUT_MS 5000
#define PING_CLEAN_REACQUIRE_STREAK 8u

/* PROTOCOL CONSTANTS */

typedef int32_t atomic_t;
static int atomic_get(const atomic_t *p) { return *p; }
static void atomic_set(atomic_t *p, int v) { *p = v; }
static void atomic_or(atomic_t *p, int v) { *p |= v; }
static void log_message(const char *fmt, ...) { (void)fmt; }
#define LOG_DBG(...) log_message(__VA_ARGS__)
#define LOG_WRN(...) log_message(__VA_ARGS__)
#define LOG_ERR(...) log_message(__VA_ARGS__)
#define LOG_INF(...) log_message(__VA_ARGS__)

struct esb_payload { uint8_t length, pipe; bool noack; uint8_t data[64]; };
struct esb_evt { int evt_id; unsigned tx_attempts; };
struct tdma_stats {
    uint32_t count, violations;
    int64_t sum_offset;
    uint64_t sum_sq_offset;
    bool phase_initialized;
    int32_t phase_ema_q8, min_offset, max_offset;
};
static struct tdma_stats g_tdma_stats[MAX_TRACKERS];
static int32_t g_last_ping_rx_time_diff_ticks[MAX_TRACKERS];
static bool g_last_ping_rx_time_diff_valid[MAX_TRACKERS];
static volatile uint32_t g_ping_isr_rx_ticks[MAX_TRACKERS];
static volatile bool g_ping_isr_rx_ticks_valid[MAX_TRACKERS];
static volatile uint32_t g_last_isr_rx_ticks[MAX_TRACKERS];
static volatile bool g_last_isr_rx_valid[MAX_TRACKERS];
static uint8_t g_bias_ppb_valid[MAX_TRACKERS];
static int32_t g_bias_ppb[MAX_TRACKERS], g_bias_ref_offset[MAX_TRACKERS];
static uint32_t g_bias_ref_ticks[MAX_TRACKERS], g_last_ping_isr_rx_ticks_raw[MAX_TRACKERS];
static uint8_t g_ping_dirty_streak[MAX_TRACKERS];
static uint32_t g_ping_reject_count[MAX_TRACKERS];
static uint8_t pipe_mismatch_count[MAX_TRACKERS], crc_error_count[MAX_TRACKERS];
static uint8_t last_ping_counter[MAX_TRACKERS], last_pong_queued_counter[MAX_TRACKERS];
static bool ping_counter_initialized[MAX_TRACKERS], sequence_initialized[MAX_TRACKERS];
static uint64_t last_ping_time[MAX_TRACKERS];
static uint32_t decision_ready_mask, decision_fence_mask;
static struct { uint32_t restart_events; } tracker_stats[MAX_TRACKERS];
static atomic_t test_all_state_valid, test_all_enabled, test_all_confirmed_mask;
static atomic_t home_beacon_tx_result;
static uint8_t tracker_remote_command[MAX_TRACKERS], pending_cmd_arg[MAX_TRACKERS];
static uint16_t tracker_test_on_tps[MAX_TRACKERS];
static uint8_t metadata_active_mask[MAX_TRACKERS], metadata_pending_mask[MAX_TRACKERS];
static uint8_t metadata_active_chunk[MAX_TRACKERS], metadata_pending_chunk[MAX_TRACKERS];
static uint16_t metadata_active_token[MAX_TRACKERS], metadata_pending_token[MAX_TRACKERS];
static int16_t pending_sens_data[MAX_TRACKERS][3];
static uint8_t pending_sens_auto_axis[MAX_TRACKERS];
static uint16_t pending_sens_auto_revolutions[MAX_TRACKERS];
static uint16_t wom_minutes[MAX_TRACKERS], wom_token[MAX_TRACKERS];
static bool wom_advertised[MAX_TRACKERS];
static uint8_t wom_first_ping_counter[MAX_TRACKERS], pending_channel;
static volatile uint32_t tdma_config_packed[MAX_TRACKERS];
static uint8_t receiver_device_addr[6], stored_trackers = MAX_TRACKERS;
static bool esb_pairing, pairing_new_devices_blocked;
static struct esb_payload rx_payload, incoming;
static bool incoming_ready;
static uint32_t now_ticks;
static unsigned shadow_count, ota_count, confirmations;
static uint8_t confirmed_id, confirmed_flag;
static void (*remote_confirm_cb)(uint8_t, uint8_t);

enum tdma_shadow_evidence { TDMA_SHADOW_EVIDENCE_PING, TDMA_SHADOW_EVIDENCE_DATA };
static void tdma_shadow_observe(uint8_t id, enum tdma_shadow_evidence evidence)
{
    assert(id < MAX_TRACKERS && evidence == TDMA_SHADOW_EVIDENCE_PING);
    ++shadow_count;
}
static uint32_t k_uptime_ticks(void) { return now_ticks; }
static uint64_t k_uptime_get(void) { return 1000; }
static uint32_t k_uptime_get_32(void) { return 1000; }
static uint32_t k_ticks_to_us_floor32(uint32_t n) { return (uint32_t)((uint64_t)n * 1000000 / 32768); }
static uint64_t k_ticks_to_us_floor64(uint64_t n) { return n * 1000000 / 32768; }
static bool esb_get_stats_detailed_enabled(void) { return false; }
static unsigned irq_lock(void) { return 0; }
static void irq_unlock(unsigned key) { (void)key; }
static uint32_t sys_get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static uint8_t crc8_ccitt(uint8_t crc, const uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}
static int esb_read_rx_payload(struct esb_payload *p)
{
    if (!incoming_ready) return -ENODATA;
    *p = incoming;
    incoming_ready = false;
    return 0;
}
static bool auto_channel_is_beacon(const uint8_t *p, uint8_t n) { (void)p; (void)n; return false; }
static bool auto_channel_receive(const uint8_t *p, uint8_t n, uint8_t pipe, uint8_t home)
{ (void)p; (void)n; (void)pipe; (void)home; return false; }
static int esb_get_rf_channel(uint32_t *channel) { *channel = 42; return 0; }
static int esb_find_tracker(uint64_t addr) { (void)addr; return -1; }
static uint8_t channel_command_for_tracker(uint8_t id) { (void)id; return ESB_PONG_FLAG_NORMAL; }
static bool data_collect_is_active(void) { return false; }
static bool data_collect_batch_is_active(void) { return false; }
static bool data_collect_is_target(uint8_t id) { (void)id; return false; }
static bool esb_ota_relay_is_active(void) { return false; }
static bool esb_ota_relay_is_target(uint8_t id) { (void)id; return false; }
static bool esb_ota_relay_abort_pending(uint8_t id) { (void)id; return false; }
static uint8_t esb_rf_channel_decode(uint8_t channel) { return channel; }
static void esb_ota_relay_fill_ack(uint8_t id, uint32_t pipe, struct esb_payload *ack,
                                   bool *has_ack, const uint8_t *p, uint8_t n)
{ (void)id; (void)pipe; (void)ack; (void)has_ack; (void)p; (void)n; }
static void esb_ota_relay_process_tracker_packet(const uint8_t *p, uint8_t n)
{ (void)p; (void)n; ++ota_count; }
static void tracker_events_receive(const uint8_t *p, uint8_t n, uint32_t now)
{ (void)p; (void)n; (void)now; }
static void test_all_invalidate_tracker(uint8_t id, int64_t now) { (void)id; (void)now; }
static uint16_t test_all_wire_tps(void) { return 220; }
static void remote_wom_receive(uint8_t id, uint8_t flag, const uint8_t *data, uint8_t counter)
{ (void)id; (void)flag; (void)data; (void)counter; }
static const char *esb_pong_flag_name(uint8_t flag) { (void)flag; return "command"; }

/* ACTUAL PRODUCTION FUNCTIONS */

struct clock_state {
    int32_t bias[MAX_TRACKERS], ppb[MAX_TRACKERS], ref_offset[MAX_TRACKERS];
    uint32_t ref_ticks[MAX_TRACKERS], raw[MAX_TRACKERS], radio_ticks[MAX_TRACKERS];
    uint32_t rejects[MAX_TRACKERS];
    uint8_t bias_valid[MAX_TRACKERS], ppb_valid[MAX_TRACKERS], dirty[MAX_TRACKERS];
    uint8_t radio_valid[MAX_TRACKERS];
};
static struct clock_state snapshot(void)
{
    struct clock_state s = {0};
    for (unsigned i = 0; i < MAX_TRACKERS; ++i) {
        s.bias[i] = g_last_ping_rx_time_diff_ticks[i];
        s.bias_valid[i] = g_last_ping_rx_time_diff_valid[i];
        s.ppb[i] = g_bias_ppb[i];
        s.ppb_valid[i] = g_bias_ppb_valid[i];
        s.ref_offset[i] = g_bias_ref_offset[i];
        s.ref_ticks[i] = g_bias_ref_ticks[i];
        s.raw[i] = g_last_ping_isr_rx_ticks_raw[i];
        s.dirty[i] = g_ping_dirty_streak[i];
        s.rejects[i] = g_ping_reject_count[i];
        s.radio_ticks[i] = g_ping_isr_rx_ticks[i];
        s.radio_valid[i] = g_ping_isr_rx_ticks_valid[i];
    }
    return s;
}
static void equal_state(const struct clock_state *expected)
{
    struct clock_state actual = snapshot();
    assert(memcmp(actual.bias, expected->bias, sizeof(actual.bias)) == 0);
    assert(memcmp(actual.bias_valid, expected->bias_valid, sizeof(actual.bias_valid)) == 0);
    assert(memcmp(actual.ppb, expected->ppb, sizeof(actual.ppb)) == 0);
    assert(memcmp(actual.ppb_valid, expected->ppb_valid, sizeof(actual.ppb_valid)) == 0);
    assert(memcmp(actual.ref_offset, expected->ref_offset, sizeof(actual.ref_offset)) == 0);
    assert(memcmp(actual.ref_ticks, expected->ref_ticks, sizeof(actual.ref_ticks)) == 0);
    assert(memcmp(actual.raw, expected->raw, sizeof(actual.raw)) == 0);
    assert(memcmp(actual.dirty, expected->dirty, sizeof(actual.dirty)) == 0);
    assert(memcmp(actual.rejects, expected->rejects, sizeof(actual.rejects)) == 0);
    assert(memcmp(actual.radio_ticks, expected->radio_ticks, sizeof(actual.radio_ticks)) == 0);
    assert(memcmp(actual.radio_valid, expected->radio_valid, sizeof(actual.radio_valid)) == 0);
}
static void seed(void)
{
    for (unsigned i = 0; i < MAX_TRACKERS; ++i) {
        g_last_ping_rx_time_diff_ticks[i] = 8;
        g_last_ping_rx_time_diff_valid[i] = true;
        g_bias_ppb_valid[i] = 1;
        g_bias_ppb[i] = 17;
        g_bias_ref_offset[i] = 5;
        g_bias_ref_ticks[i] = 100000;
        g_last_ping_isr_rx_ticks_raw[i] = 100100;
        g_ping_isr_rx_ticks[i] = 100100;
        g_ping_isr_rx_ticks_valid[i] = true;
        g_ping_dirty_streak[i] = 7;
        g_ping_reject_count[i] = 10;
    }
}
static struct esb_payload packet(uint8_t type, uint8_t id, uint8_t pipe, uint8_t counter,
                                 uint32_t expected, uint8_t flag)
{
    struct esb_payload p = {.length = ESB_PING_LEN, .pipe = pipe};
    p.data[0] = type; p.data[1] = id; p.data[2] = counter;
    p.data[3] = (uint8_t)(expected >> 24); p.data[4] = (uint8_t)(expected >> 16);
    p.data[5] = (uint8_t)(expected >> 8); p.data[6] = (uint8_t)expected;
    p.data[7] = flag;
    p.data[ESB_PING_LEN - 1] = crc8_ccitt(0x07, p.data, ESB_PING_LEN - 1);
    return p;
}
static struct esb_payload ack_for(const struct esb_payload *p, bool *has_ack)
{
    struct esb_payload ack = {0};
    bool suppress = true;
    esb_ack_handler_cb(p->data, p->length, p->pipe, &ack, has_ack, &suppress);
    assert(!suppress);
    return ack;
}
static void dispatch(const struct esb_payload *p)
{
    incoming = *p;
    incoming_ready = true;
    const struct esb_evt event = {.evt_id = ESB_EVENT_RX_RECEIVED};
    event_handler(&event);
    assert(!incoming_ready);
}
static void confirm(uint8_t id, uint8_t flag)
{
    ++confirmations;
    confirmed_id = id;
    confirmed_flag = flag;
}
static void test_invalid(const char *which)
{
    const uint8_t id = 2;
    seed();
    now_ticks = 132768;
    struct esb_payload p = packet(ESB_PING_TYPE, id, 3, 1, 0, ESB_PONG_FLAG_NORMAL);
    if (strcmp(which, "wrong-type") == 0) {
        p = packet(0x40, id, 3, 1, 0, ESB_PONG_FLAG_NORMAL);
    } else if (strcmp(which, "bad-crc") == 0) {
        p.data[ESB_PING_LEN - 1] ^= 1;
    } else if (strcmp(which, "wrong-pipe") == 0) {
        p = packet(ESB_PING_TYPE, id, 4, 1, 0, ESB_PONG_FLAG_CALIBRATE);
        tracker_remote_command[id] = ESB_PONG_FLAG_CALIBRATE;
        remote_confirm_cb = confirm;
    } else if (strcmp(which, "out-of-range") == 0) {
        p = packet(ESB_PING_TYPE, MAX_TRACKERS, 3, 1, 0, ESB_PONG_FLAG_NORMAL);
    } else {
        abort();
    }
    struct clock_state before = snapshot();
    bool has_ack = true;
    struct esb_payload ack = ack_for(&p, &has_ack);
    equal_state(&before);
    if (strcmp(which, "wrong-pipe") == 0) {
        assert(has_ack && ack.pipe == 3 && ack.data[7] == ESB_PONG_FLAG_CALIBRATE);
    } else {
        assert(!has_ack);
    }
    now_ticks = 132795;
    dispatch(&p);
    equal_state(&before);
    assert(shadow_count == 0);
    if (strcmp(which, "wrong-pipe") == 0) {
        assert(pipe_mismatch_count[id] == 1);
        assert(ping_counter_initialized[id] && last_ping_counter[id] == 1);
        assert(tracker_remote_command[id] == ESB_PONG_FLAG_NORMAL);
        assert(confirmations == 1 && confirmed_id == id && confirmed_flag == ESB_PONG_FLAG_CALIBRATE);
    }
    if (strcmp(which, "bad-crc") == 0) assert(crc_error_count[id] == 1);
}
static void test_valid(void)
{
    const uint8_t id = 2;
    seed();
    g_ping_dirty_streak[id] = 3;
    struct clock_state expected = snapshot();
    struct esb_payload p = packet(ESB_PING_TYPE, id, 3, 1, 132754, ESB_PONG_FLAG_NORMAL);
    now_ticks = 132768;
    bool has_ack = false;
    struct esb_payload ack = ack_for(&p, &has_ack);
    assert(has_ack && ack.pipe == 3 && ack.data[0] == ESB_PONG_TYPE);
    assert(sys_get_be32(&ack.data[3]) == 132768);
    expected.radio_ticks[id] = 132768;
    equal_state(&expected);
    now_ticks = 132795;
    dispatch(&p);
    expected.bias[id] = 9;
    expected.ppb[id] = 122070;
    expected.ppb_valid[id] = 2;
    expected.raw[id] = 132768;
    expected.dirty[id] = 0;
    equal_state(&expected);
    assert(shadow_count == 1 && ping_counter_initialized[id]);
}
static void test_valid_dirty(void)
{
    const uint8_t id = 2;
    seed();
    g_ping_dirty_streak[id] = 3;
    struct clock_state expected = snapshot();
    struct esb_payload p = packet(ESB_PING_TYPE, id, 3, 1, 132600, ESB_PONG_FLAG_NORMAL);
    now_ticks = 132768;
    bool has_ack = false;
    (void)ack_for(&p, &has_ack);
    assert(has_ack);
    expected.radio_ticks[id] = 132768;
    equal_state(&expected);
    now_ticks = 132795;
    dispatch(&p);
    expected.dirty[id] = 4;
    expected.rejects[id] = 11;
    equal_state(&expected);
    assert(shadow_count == 1);
}
static void test_reacquire(void)
{
    const uint8_t id = 2;
    seed();
    struct clock_state expected = snapshot();
    struct esb_payload p = packet(ESB_PING_TYPE, id, 3, 1, 132600, ESB_PONG_FLAG_NORMAL);
    now_ticks = 132768;
    bool has_ack = false;
    (void)ack_for(&p, &has_ack);
    assert(has_ack);
    expected.radio_ticks[id] = 132768;
    equal_state(&expected);
    now_ticks = 132795;
    dispatch(&p);
    expected.bias[id] = 163;
    expected.ppb_valid[id] = 0;
    expected.raw[id] = 132768;
    expected.dirty[id] = 0;
    expected.rejects[id] = 11;
    equal_state(&expected);
    assert(shadow_count == 1);
}
static void test_event_fallback(void)
{
    const uint8_t id = 2;
    seed();
    g_ping_isr_rx_ticks_valid[id] = false;
    g_ping_dirty_streak[id] = 3;
    struct clock_state expected = snapshot();
    struct esb_payload p = packet(ESB_PING_TYPE, id, 3, 1, 132754, ESB_PONG_FLAG_NORMAL);
    now_ticks = 132768;
    dispatch(&p);
    expected.bias[id] = 9;
    expected.ppb[id] = 122070;
    expected.ppb_valid[id] = 2;
    expected.raw[id] = 132768;
    expected.dirty[id] = 0;
    equal_state(&expected);
    assert(shadow_count == 1);
}
static void test_interleaved_pipe(void)
{
    const uint8_t id = 2;
    seed();
    g_ping_dirty_streak[id] = 3;
    struct clock_state expected = snapshot();
    struct esb_payload valid = packet(ESB_PING_TYPE, id, 3, 1, 132754, ESB_PONG_FLAG_NORMAL);
    struct esb_payload wrong_pipe = packet(ESB_PING_TYPE, id, 4, 2, 0, ESB_PONG_FLAG_NORMAL);
    bool has_ack = false;
    now_ticks = 132768;
    (void)ack_for(&valid, &has_ack);
    assert(has_ack);
    expected.radio_ticks[id] = 132768;
    equal_state(&expected);
    incoming = valid;
    incoming_ready = true;
    now_ticks = 133000;
    struct esb_payload ack = ack_for(&wrong_pipe, &has_ack);
    assert(has_ack && ack.pipe == 3 && ack.data[0] == ESB_PONG_TYPE);
    assert(sys_get_be32(&ack.data[3]) == 133000);
    assert(g_last_isr_rx_ticks[id] == 133000 && g_last_isr_rx_valid[id]);
    equal_state(&expected);
    now_ticks = 133010;
    const struct esb_evt event = {.evt_id = ESB_EVENT_RX_RECEIVED};
    event_handler(&event);
    assert(!incoming_ready);
    expected.bias[id] = 9;
    expected.ppb[id] = 122070;
    expected.ppb_valid[id] = 2;
    expected.raw[id] = 132768;
    expected.dirty[id] = 0;
    equal_state(&expected);
    dispatch(&wrong_pipe);
    equal_state(&expected);
    assert(shadow_count == 1 && pipe_mismatch_count[id] == 1);
    assert(last_ping_counter[id] == 2);
}
static void test_ota(uint8_t type)
{
    seed();
    struct clock_state expected = snapshot();
    struct esb_payload p = packet(type, 2, 3, 1, 0, 0);
    now_ticks = 132768;
    dispatch(&p);
    equal_state(&expected);
    assert(ota_count == 1 && shadow_count == 0);
}
static void test_valid_after_invalid(void)
{
    const uint8_t id = 2;
    seed();
    g_ping_dirty_streak[id] = 3;
    struct clock_state expected = snapshot();
    struct esb_payload p = packet(0x40, id, 3, 1, 0, ESB_PONG_FLAG_NORMAL);
    bool has_ack = true;
    now_ticks = 132000;
    (void)ack_for(&p, &has_ack);
    assert(!has_ack);
    dispatch(&p);
    equal_state(&expected);
    p = packet(ESB_PING_TYPE, id, 3, 1, 0, ESB_PONG_FLAG_NORMAL);
    p.data[ESB_PING_LEN - 1] ^= 1;
    (void)ack_for(&p, &has_ack);
    assert(!has_ack);
    dispatch(&p);
    equal_state(&expected);
    p = packet(ESB_PING_TYPE, id, 4, 1, 0, ESB_PONG_FLAG_NORMAL);
    (void)ack_for(&p, &has_ack);
    assert(has_ack);
    dispatch(&p);
    equal_state(&expected);
    p = packet(ESB_PING_TYPE, id, 3, 2, 132754, ESB_PONG_FLAG_NORMAL);
    now_ticks = 132768;
    (void)ack_for(&p, &has_ack);
    assert(has_ack);
    expected.radio_ticks[id] = 132768;
    equal_state(&expected);
    now_ticks = 132795;
    dispatch(&p);
    expected.bias[id] = 9;
    expected.ppb[id] = 122070;
    expected.ppb_valid[id] = 2;
    expected.raw[id] = 132768;
    expected.dirty[id] = 0;
    equal_state(&expected);
    assert(shadow_count == 1 && last_ping_counter[id] == 2);
}
static void test_early_sequence_exit(bool duplicate)
{
    const uint8_t id = 2;
    seed();
    g_ping_dirty_streak[id] = 3;
    ping_counter_initialized[id] = true;
    last_ping_counter[id] = 5;
    last_pong_queued_counter[id] = 5;
    struct clock_state expected = snapshot();
    struct esb_payload p = packet(ESB_PING_TYPE, id, 3, duplicate ? 5 : 4,
                                  132754, ESB_PONG_FLAG_NORMAL);
    now_ticks = 132768;
    bool has_ack = false;
    (void)ack_for(&p, &has_ack);
    assert(has_ack);
    expected.radio_ticks[id] = 132768;
    equal_state(&expected);
    now_ticks = 132795;
    dispatch(&p);
    expected.bias[id] = 9;
    expected.ppb[id] = 122070;
    expected.ppb_valid[id] = 2;
    expected.raw[id] = 132768;
    expected.dirty[id] = 0;
    equal_state(&expected);
    assert(shadow_count == 1 && last_ping_counter[id] == 5);
    assert(last_pong_queued_counter[id] == 5);
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    if (strcmp(argv[1], "valid") == 0) test_valid();
    else if (strcmp(argv[1], "valid-dirty") == 0) test_valid_dirty();
    else if (strcmp(argv[1], "reacquire") == 0) test_reacquire();
    else if (strcmp(argv[1], "event-fallback") == 0) test_event_fallback();
    else if (strcmp(argv[1], "interleaved-pipe") == 0) test_interleaved_pipe();
    else if (strcmp(argv[1], "ota-status") == 0) test_ota(ESB_OTA_STATUS_TYPE);
    else if (strcmp(argv[1], "ota-info") == 0) test_ota(ESB_OTA_FW_INFO_TYPE);
    else if (strcmp(argv[1], "valid-after-invalid") == 0) test_valid_after_invalid();
    else if (strcmp(argv[1], "duplicate") == 0) test_early_sequence_exit(true);
    else if (strcmp(argv[1], "out-of-order") == 0) test_early_sequence_exit(false);
    else test_invalid(argv[1]);
    printf("PASS %s\n", argv[1]);
    return 0;
}
