#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "connection/tracker_event_protocol.h"

#define MAX_TRACKERS 16
#define BIT(n) (1u << (n))
#define ARG_UNUSED(x) ((void)(x))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define CLAMP(v, lo, hi) MIN(MAX((v), (lo)), (hi))
#define TEST_ALL_JOIN_CONFIG_GRACE_MS 1500
#define K_NO_WAIT 0
#define AC_FRAME_SIZE 16
#define ESB_EVENT_TX_SUCCESS 1
#define ESB_EVENT_TX_FAILED 2
#define ESB_EVENT_RX_RECEIVED 3
#define PING_TIMEOUT_MS 5000
#define PING_CLEAN_REACQUIRE_STREAK 8u

/* PROTOCOL CONSTANTS */

typedef int32_t atomic_t;
typedef int32_t atomic_val_t;
static int atomic_get(const atomic_t *p) { return *p; }
static void atomic_set(atomic_t *p, int v) { *p = v; }
static void atomic_or(atomic_t *p, int v) { *p |= v; }
static void atomic_and(atomic_t *p, int v) { *p &= v; }
static void log_message(const char *fmt, ...) { (void)fmt; }
#define LOG_DBG(...) log_message(__VA_ARGS__)
static unsigned detailed_warnings;
static int64_t warning_mean;
static unsigned warning_stddev;
static uint64_t warning_rx_us;
static char warning_sign;
static void log_warning(const char *fmt, ...)
{
    if (strncmp(fmt, "TDMA Stats ID=", 14) != 0) return;
    va_list args;
    va_start(args, fmt);
    assert(va_arg(args, int) == 2);
    assert(va_arg(args, uint32_t) == 2);
    assert(va_arg(args, uint32_t) == 13);
    warning_mean = va_arg(args, int64_t);
    warning_stddev = va_arg(args, uint32_t);
    assert(va_arg(args, int) == -3);
    assert(va_arg(args, int) == -6);
    assert(va_arg(args, int) == 2);
    warning_sign = va_arg(args, char *)[0];
    warning_rx_us = va_arg(args, uint64_t);
    va_end(args);
    ++detailed_warnings;
}
#define LOG_WRN(...) log_warning(__VA_ARGS__)
static unsigned error_count;
#define LOG_ERR(...) (error_count++, log_message(__VA_ARGS__))
#define LOG_INF(...) log_message(__VA_ARGS__)

struct esb_payload { uint8_t length, pipe; bool noack; int8_t rssi; uint8_t data[64]; };
struct esb_evt { int evt_id; unsigned tx_attempts; };
/* PRODUCTION STATE */
static uint32_t decision_ready_mask, decision_fence_mask;
static atomic_t test_all_state_valid, test_all_enabled, test_all_confirmed_mask;
static atomic_t test_all_ready_after_ms[MAX_TRACKERS];
static atomic_t home_beacon_tx_result;
static uint8_t pending_channel;
static uint8_t receiver_device_addr[6], stored_trackers = MAX_TRACKERS;
static bool esb_pairing;
static atomic_t pairing_new_devices_blocked;
static struct esb_payload rx_payload, incoming;
static bool incoming_ready;
static uint32_t now_ticks;
static unsigned shadow_count, shadow_ping_count, shadow_data_count, ota_count, confirmations;
static unsigned tick_reads, radio_tick_reads, event_tick_reads, pong_tick_reads;
static unsigned sequence_calls, slot_count, pose_count, raw_count, event_count, private_count;
enum tick_context { TICK_RADIO, TICK_EVENT };
static enum tick_context tick_context;
static uint32_t slot_ticks[16];
static uint8_t poses[16][16], raw_packets[16][64], raw_lengths[16], event_packets[16][17];
static uint8_t ota_packets[16][64], ota_lengths[16];
static uint8_t sequence_last[MAX_TRACKERS];
static bool sequence_seen[MAX_TRACKERS], raw_target;
static struct esb_payload fifo[32];
static uint32_t fifo_ticks[32];
static unsigned fifo_count, fifo_next;
struct pairing_event { uint8_t packet[8]; };
static struct { unsigned count; struct pairing_event item; } esb_pairing_msgq;
static uint8_t receiver_identity[6] = {1, 2, 3, 4, 5, 6};
static atomic_t initialized = 1, enabled = 1;
struct observation { uint8_t frame[AC_FRAME_SIZE], home; uint32_t generation; int64_t received; };
static struct observation queued_observation;
static int observations;
static uint8_t confirmed_id, confirmed_flag;
static void (*remote_confirm_cb)(uint8_t, uint8_t);

enum tdma_shadow_evidence { TDMA_SHADOW_EVIDENCE_PING, TDMA_SHADOW_EVIDENCE_DATA };
static void tdma_shadow_observe(uint8_t id, enum tdma_shadow_evidence evidence)
{
    assert(id < MAX_TRACKERS);
    ++shadow_count;
    if (evidence == TDMA_SHADOW_EVIDENCE_PING) ++shadow_ping_count;
    else { assert(evidence == TDMA_SHADOW_EVIDENCE_DATA); ++shadow_data_count; }
}
static uint32_t k_uptime_ticks(void)
{
    ++tick_reads;
    if (tick_context == TICK_RADIO) ++radio_tick_reads;
    else ++event_tick_reads;
    return now_ticks;
}
static uint64_t k_uptime_get(void) { return 1000; }
static uint32_t k_uptime_get_32(void) { return 1000; }
static uint32_t k_ticks_to_us_floor32(uint32_t n) { return (uint32_t)((uint64_t)n * 1000000 / 32768); }
static uint64_t k_ticks_to_us_floor64(uint64_t n) { return n * 1000000 / 32768; }
static bool detailed_enabled;
static bool esb_get_stats_detailed_enabled(void) { return detailed_enabled; }
static unsigned irq_lock(void) { return 0; }
static void irq_unlock(unsigned key) { (void)key; }
static uint32_t sys_get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static uint16_t sys_get_be16(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }
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
    if (fifo_next < fifo_count) {
        *p = fifo[fifo_next];
        now_ticks = fifo_ticks[fifo_next++];
        return 0;
    }
    if (!incoming_ready) return -ENODATA;
    *p = incoming;
    incoming_ready = false;
    return 0;
}
static uint32_t auto_channel_generation(void) { return 7; }
static int esb_get_rf_channel(uint32_t *channel) { *channel = 42; return 0; }
static int esb_find_tracker(uint64_t addr) { (void)addr; return -1; }
static uint8_t channel_command_for_tracker(uint8_t id) { (void)id; return ESB_PONG_FLAG_NORMAL; }
static bool data_collect_is_active(void) { return false; }
static bool data_collect_batch_is_active(void) { return false; }
static bool data_collect_is_target(uint8_t id) { return raw_target && id == 2; }
static bool data_collect_batch_is_target(uint8_t id) { (void)id; return false; }
static bool esb_ota_relay_is_active(void) { return false; }
static bool esb_ota_relay_is_target(uint8_t id) { (void)id; return false; }
static bool esb_ota_relay_abort_pending(uint8_t id) { (void)id; return false; }
static uint8_t esb_rf_channel_decode(uint8_t channel) { return channel; }
static void esb_ota_relay_fill_ack(uint8_t id, uint32_t pipe, struct esb_payload *ack,
                                   bool *has_ack, const uint8_t *p, uint8_t n)
{ (void)id; (void)pipe; (void)ack; (void)has_ack; (void)p; (void)n; }
static void esb_ota_relay_process_tracker_packet(const uint8_t *p, uint8_t n)
{
    assert(ota_count < 16 && n <= 64);
    memcpy(ota_packets[ota_count], p, n);
    ota_lengths[ota_count++] = n;
}
static bool tracker_events_receive(const uint8_t *p, uint8_t n, uint32_t now)
{
    struct tracker_event decoded;
    (void)now;
    if (!tracker_event_decode(p, n, &decoded)) return false;
    assert(event_count < 16);
    memcpy(event_packets[event_count++], p, TRACKER_EVENT_ESB_LEN);
    return true;
}
static void test_all_invalidate_tracker(uint8_t id, int64_t now) { (void)id; (void)now; }
static uint16_t test_all_wire_tps(void) { return 220; }
static void remote_wom_receive(uint8_t id, uint8_t flag, const uint8_t *data, uint8_t counter)
{ (void)id; (void)flag; (void)data; (void)counter; }
static const char *esb_pong_flag_name(uint8_t flag) { (void)flag; return "command"; }
#if defined(CONFIG_TDMA_DIAGNOSTICS)
static void tdma_check_slot(uint8_t id, uint32_t ticks, int8_t rssi)
{
    (void)rssi;
    assert(id < MAX_TRACKERS && slot_count < 16);
    slot_ticks[slot_count++] = ticks;
}
#endif
static int check_packet_sequence(uint8_t id, uint8_t sequence)
{
    assert(id < MAX_TRACKERS);
    ++sequence_calls;
    if (sequence_seen[id] && sequence == sequence_last[id]) return 4;
    if (sequence_seen[id] && (uint8_t)(sequence - sequence_last[id]) > 128) return 2;
    sequence_seen[id] = true;
    sequence_last[id] = sequence;
    return 0;
}
static void hid_write_packet_n(const uint8_t *packet, int8_t rssi)
{
    (void)rssi;
    assert(pose_count < 16);
    memcpy(poses[pose_count++], packet, 16);
}
static void data_collect_write(const uint8_t *packet, uint8_t length, int8_t rssi)
{
    (void)rssi;
    assert(raw_count < 16 && length <= 64);
    memcpy(raw_packets[raw_count], packet, length);
    raw_lengths[raw_count++] = length;
}
static void esb_send_remote_command(uint8_t id, uint8_t command) { tracker_slots[id].tracker_remote_command = command; }
static int k_msgq_put(void *queue, const void *item, int timeout)
{
    (void)timeout;
    if (queue == &observations) {
        queued_observation = *(const struct observation *)item;
        ++private_count;
        return 0;
    }
    assert(queue == &esb_pairing_msgq);
    assert(esb_pairing_msgq.count == 0);
    esb_pairing_msgq.item = *(const struct pairing_event *)item;
    ++esb_pairing_msgq.count;
    return 0;
}
static int k_msgq_get(void *queue, void *item, int timeout)
{
    (void)queue; (void)item; (void)timeout;
    return -ENOMSG;
}
/* ACTUAL AUTO CHANNEL */


/* ACTUAL PRODUCTION FUNCTIONS */

#if defined(CONFIG_TDMA_DIAGNOSTICS)
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
        s.bias[i] = tracker_diags[i].g_last_ping_rx_time_diff_ticks;
        s.bias_valid[i] = tracker_diags[i].g_last_ping_rx_time_diff_valid;
        s.ppb[i] = tracker_diags[i].g_bias_ppb;
        s.ppb_valid[i] = tracker_diags[i].g_bias_ppb_valid;
        s.ref_offset[i] = tracker_diags[i].g_bias_ref_offset;
        s.ref_ticks[i] = tracker_diags[i].g_bias_ref_ticks;
        s.raw[i] = tracker_diags[i].g_last_ping_isr_rx_ticks_raw;
        s.dirty[i] = tracker_diags[i].g_ping_dirty_streak;
        s.rejects[i] = tracker_diags[i].g_ping_reject_count;
        s.radio_ticks[i] = tracker_diags[i].g_ping_isr_rx_ticks;
        s.radio_valid[i] = tracker_diags[i].g_ping_isr_rx_ticks_valid;
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
#endif
static void seed(void)
{
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    for (unsigned i = 0; i < MAX_TRACKERS; ++i) {
        tracker_diags[i].g_last_ping_rx_time_diff_ticks = 8;
        tracker_diags[i].g_last_ping_rx_time_diff_valid = true;
        tracker_diags[i].g_bias_ppb_valid = 1;
        tracker_diags[i].g_bias_ppb = 17;
        tracker_diags[i].g_bias_ref_offset = 5;
        tracker_diags[i].g_bias_ref_ticks = 100000;
        tracker_diags[i].g_last_ping_isr_rx_ticks_raw = 100100;
        tracker_diags[i].g_ping_isr_rx_ticks = 100100;
        tracker_diags[i].g_ping_isr_rx_ticks_valid = true;
        tracker_diags[i].g_ping_dirty_streak = 7;
        tracker_diags[i].g_ping_reject_count = 10;
    }
#endif
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
    tick_context = TICK_RADIO;
    esb_ack_handler_cb(p->data, p->length, p->pipe, &ack, has_ack, &suppress);
    assert(!suppress);
    if (*has_ack) {
        ++pong_tick_reads;
        assert(ack.length == ESB_PONG_LEN && ack.data[0] == ESB_PONG_TYPE);
        assert(ack.data[1] == p->data[1] && ack.data[2] == p->data[2]);
        assert(sys_get_be32(&ack.data[3]) == now_ticks);
        assert(crc8_ccitt(0x07, ack.data, ESB_PONG_LEN - 1) == ack.data[ESB_PONG_LEN - 1]);
        printf("PROTOCOL ack pipe=%u data=", ack.pipe);
        for (unsigned i = 0; i < ack.length; ++i) printf("%02x", ack.data[i]);
        putchar('\n');
    }
    tick_context = TICK_EVENT;
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
static void test_late_metadata(void)
{
    const uint8_t id = 2;
    tracker_slots[id].metadata_active_mask = 1;
    tracker_slots[id].metadata_active_chunk = 3;
    tracker_slots[id].metadata_active_token = 102;
    now_ticks = 1000;
    struct esb_payload p = packet(ESB_PING_TYPE, id, 3, 1, 990, ESB_PONG_FLAG_METADATA_REQUEST);
    p.data[8] = 1;
    p.data[9] = 3;
    p.data[10] = 0;
    p.data[11] = 101;
    p.data[ESB_PING_LEN - 1] = crc8_ccitt(0x07, p.data, ESB_PING_LEN - 1);
    dispatch(&p);
    assert(tracker_slots[id].metadata_active_mask == 1);
    assert(tracker_slots[id].metadata_active_token == 102);
    p.data[2] = 2;
    p.data[11] = 102;
    p.data[ESB_PING_LEN - 1] = crc8_ccitt(0x07, p.data, ESB_PING_LEN - 1);
    dispatch(&p);
    assert(tracker_slots[id].metadata_active_mask == 0);
    assert(tracker_slots[id].metadata_active_token == 102);
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
        tracker_slots[id].tracker_remote_command = ESB_PONG_FLAG_CALIBRATE;
        remote_confirm_cb = confirm;
    } else if (strcmp(which, "out-of-range") == 0) {
        p = packet(ESB_PING_TYPE, MAX_TRACKERS, 3, 1, 0, ESB_PONG_FLAG_NORMAL);
    } else {
        abort();
    }
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    struct clock_state before = snapshot();
#endif
    bool has_ack = true;
    struct esb_payload ack = ack_for(&p, &has_ack);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    equal_state(&before);
#endif
    if (strcmp(which, "wrong-pipe") == 0) {
        assert(has_ack && ack.pipe == 3 && ack.data[7] == ESB_PONG_FLAG_CALIBRATE);
    } else {
        assert(!has_ack);
    }
    now_ticks = 132795;
    dispatch(&p);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    equal_state(&before);
#endif
    assert(shadow_count == 0);
    if (strcmp(which, "wrong-pipe") == 0) {
        assert(tracker_slots[id].pipe_mismatch_count == 1);
        assert(tracker_slots[id].ping_counter_initialized && tracker_slots[id].last_ping_counter == 1);
        assert(tracker_slots[id].tracker_remote_command == ESB_PONG_FLAG_NORMAL);
        assert(confirmations == 1 && confirmed_id == id && confirmed_flag == ESB_PONG_FLAG_CALIBRATE);
    }
    if (strcmp(which, "bad-crc") == 0) assert(tracker_slots[id].crc_error_count == 1);
}
static void test_valid(void)
{
    const uint8_t id = 2;
    seed();
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    tracker_diags[id].g_ping_dirty_streak = 3;
    struct clock_state expected = snapshot();
#endif
    struct esb_payload p = packet(ESB_PING_TYPE, id, 3, 1, 132754, ESB_PONG_FLAG_NORMAL);
    now_ticks = 132768;
    bool has_ack = false;
    struct esb_payload ack = ack_for(&p, &has_ack);
    assert(has_ack && ack.pipe == 3 && ack.data[0] == ESB_PONG_TYPE);
    assert(sys_get_be32(&ack.data[3]) == 132768);
    const uint8_t expected_pong[12] = {0xf1, 2, 1, 0, 2, 6, 0xa0, 0, 7, 48, 3, 2};
    assert(memcmp(ack.data, expected_pong, sizeof(expected_pong)) == 0);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    expected.radio_ticks[id] = 132768;
    equal_state(&expected);
#endif
    now_ticks = 132795;
    dispatch(&p);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    expected.bias[id] = 9;
    expected.ppb[id] = 122070;
    expected.ppb_valid[id] = 2;
    expected.raw[id] = 132768;
    expected.dirty[id] = 0;
    equal_state(&expected);
#endif
    assert(shadow_count == 1 && tracker_slots[id].ping_counter_initialized);
}
static void test_valid_dirty(void)
{
    const uint8_t id = 2;
    seed();
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    tracker_diags[id].g_ping_dirty_streak = 3;
    struct clock_state expected = snapshot();
#endif
    struct esb_payload p = packet(ESB_PING_TYPE, id, 3, 1, 132600, ESB_PONG_FLAG_NORMAL);
    now_ticks = 132768;
    bool has_ack = false;
    (void)ack_for(&p, &has_ack);
    assert(has_ack);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    expected.radio_ticks[id] = 132768;
    equal_state(&expected);
#endif
    now_ticks = 132795;
    dispatch(&p);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    expected.dirty[id] = 4;
    expected.rejects[id] = 11;
    equal_state(&expected);
#endif
    assert(shadow_count == 1);
}
static void test_reacquire(void)
{
    const uint8_t id = 2;
    seed();
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    struct clock_state expected = snapshot();
#endif
    struct esb_payload p = packet(ESB_PING_TYPE, id, 3, 1, 132600, ESB_PONG_FLAG_NORMAL);
    now_ticks = 132768;
    bool has_ack = false;
    (void)ack_for(&p, &has_ack);
    assert(has_ack);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    expected.radio_ticks[id] = 132768;
    equal_state(&expected);
#endif
    now_ticks = 132795;
    dispatch(&p);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    expected.bias[id] = 163;
    expected.ppb_valid[id] = 0;
    expected.raw[id] = 132768;
    expected.dirty[id] = 0;
    expected.rejects[id] = 11;
    equal_state(&expected);
#endif
    assert(shadow_count == 1);
}
static void test_event_fallback(void)
{
    const uint8_t id = 2;
    seed();
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    tracker_diags[id].g_ping_isr_rx_ticks_valid = false;
    tracker_diags[id].g_ping_dirty_streak = 3;
    struct clock_state expected = snapshot();
#endif
    struct esb_payload p = packet(ESB_PING_TYPE, id, 3, 1, 132754, ESB_PONG_FLAG_NORMAL);
    now_ticks = 132768;
    dispatch(&p);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    expected.bias[id] = 9;
    expected.ppb[id] = 122070;
    expected.ppb_valid[id] = 2;
    expected.raw[id] = 132768;
    expected.dirty[id] = 0;
    equal_state(&expected);
#endif
    assert(shadow_count == 1);
}
static void test_interleaved_pipe(void)
{
    const uint8_t id = 2;
    seed();
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    tracker_diags[id].g_ping_dirty_streak = 3;
    struct clock_state expected = snapshot();
#endif
    struct esb_payload valid = packet(ESB_PING_TYPE, id, 3, 1, 132754, ESB_PONG_FLAG_NORMAL);
    struct esb_payload wrong_pipe = packet(ESB_PING_TYPE, id, 4, 2, 0, ESB_PONG_FLAG_NORMAL);
    bool has_ack = false;
    now_ticks = 132768;
    (void)ack_for(&valid, &has_ack);
    assert(has_ack);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    expected.radio_ticks[id] = 132768;
    equal_state(&expected);
#endif
    incoming = valid;
    incoming_ready = true;
    now_ticks = 133000;
    struct esb_payload ack = ack_for(&wrong_pipe, &has_ack);
    assert(has_ack && ack.pipe == 3 && ack.data[0] == ESB_PONG_TYPE);
    assert(sys_get_be32(&ack.data[3]) == 133000);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    assert(tracker_diags[id].g_last_isr_rx_ticks == 133000 && tracker_diags[id].g_last_isr_rx_valid);
    equal_state(&expected);
#endif
    now_ticks = 133010;
    const struct esb_evt event = {.evt_id = ESB_EVENT_RX_RECEIVED};
    event_handler(&event);
    assert(!incoming_ready);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    expected.bias[id] = 9;
    expected.ppb[id] = 122070;
    expected.ppb_valid[id] = 2;
    expected.raw[id] = 132768;
    expected.dirty[id] = 0;
    equal_state(&expected);
#endif
    dispatch(&wrong_pipe);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    equal_state(&expected);
#endif
    assert(shadow_count == 1 && tracker_slots[id].pipe_mismatch_count == 1);
    assert(tracker_slots[id].last_ping_counter == 2);
}
static void test_ota(uint8_t type)
{
    seed();
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    struct clock_state expected = snapshot();
#endif
    struct esb_payload p = packet(type, 2, 3, 1, 0, 0);
    now_ticks = 132768;
    dispatch(&p);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    equal_state(&expected);
#endif
    assert(ota_count == 1 && shadow_count == 0);
}
static void test_valid_after_invalid(void)
{
    const uint8_t id = 2;
    seed();
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    tracker_diags[id].g_ping_dirty_streak = 3;
    struct clock_state expected = snapshot();
#endif
    struct esb_payload p = packet(0x40, id, 3, 1, 0, ESB_PONG_FLAG_NORMAL);
    bool has_ack = true;
    now_ticks = 132000;
    (void)ack_for(&p, &has_ack);
    assert(!has_ack);
    dispatch(&p);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    equal_state(&expected);
#endif
    p = packet(ESB_PING_TYPE, id, 3, 1, 0, ESB_PONG_FLAG_NORMAL);
    p.data[ESB_PING_LEN - 1] ^= 1;
    (void)ack_for(&p, &has_ack);
    assert(!has_ack);
    dispatch(&p);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    equal_state(&expected);
#endif
    p = packet(ESB_PING_TYPE, id, 4, 1, 0, ESB_PONG_FLAG_NORMAL);
    (void)ack_for(&p, &has_ack);
    assert(has_ack);
    dispatch(&p);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    equal_state(&expected);
#endif
    p = packet(ESB_PING_TYPE, id, 3, 2, 132754, ESB_PONG_FLAG_NORMAL);
    now_ticks = 132768;
    (void)ack_for(&p, &has_ack);
    assert(has_ack);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    expected.radio_ticks[id] = 132768;
    equal_state(&expected);
#endif
    now_ticks = 132795;
    dispatch(&p);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    expected.bias[id] = 9;
    expected.ppb[id] = 122070;
    expected.ppb_valid[id] = 2;
    expected.raw[id] = 132768;
    expected.dirty[id] = 0;
    equal_state(&expected);
#endif
    assert(shadow_count == 1 && tracker_slots[id].last_ping_counter == 2);
}
static void test_early_sequence_exit(bool duplicate)
{
    const uint8_t id = 2;
    seed();
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    tracker_diags[id].g_ping_dirty_streak = 3;
#endif
    tracker_slots[id].ping_counter_initialized = true;
    tracker_slots[id].last_ping_counter = 5;
    tracker_slots[id].last_pong_queued_counter = 5;
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    struct clock_state expected = snapshot();
#endif
    struct esb_payload p = packet(ESB_PING_TYPE, id, 3, duplicate ? 5 : 4,
                                  132754, ESB_PONG_FLAG_NORMAL);
    now_ticks = 132768;
    bool has_ack = false;
    (void)ack_for(&p, &has_ack);
    assert(has_ack);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    expected.radio_ticks[id] = 132768;
    equal_state(&expected);
#endif
    now_ticks = 132795;
    dispatch(&p);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    expected.bias[id] = 9;
    expected.ppb[id] = 122070;
    expected.ppb_valid[id] = 2;
    expected.raw[id] = 132768;
    expected.dirty[id] = 0;
    equal_state(&expected);
#endif
    assert(shadow_count == 1 && tracker_slots[id].last_ping_counter == 5);
    assert(tracker_slots[id].last_pong_queued_counter == 5);
}
static void test_detailed_warning(void)
{
    detailed_enabled = true;
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    /* Exercise the extracted clock helper, including nonzero variance/sqrt. */
    struct tdma_stats *stats = &tracker_diags[2].stats;
    stats->count = 2;
    stats->violations = 13;
    stats->sum_offset = -4;
    stats->sum_sq_offset = 40;
    stats->phase_initialized = true;
    stats->phase_ema_q8 = -3 * 256;
    stats->min_offset = -6;
    stats->max_offset = 2;
    tracker_diags[2].g_ping_isr_rx_ticks_valid = false;
    account_ping_clock(2, 1000, 990);
    assert(detailed_warnings == 1);
    assert(warning_mean == -2 && warning_stddev == 4);
    assert(warning_sign == '-' && warning_rx_us == 305);
    account_ping_clock(2, 1000, 1010);
    assert(detailed_warnings == 2);
    assert(warning_sign == '+' && warning_rx_us == 305);
    stats->violations = 12;
    account_ping_clock(2, 1000, 1010);
    assert(detailed_warnings == 2);
    stats->violations = 13;
    detailed_enabled = false;
    account_ping_clock(2, 1000, 1010);
    assert(detailed_warnings == 2);
    detailed_enabled = true;
    stats->count = 0;
    account_ping_clock(2, 1000, 1010);
    assert(detailed_warnings == 2);
#else
    assert(detailed_warnings == 0);
#endif
}

static void fifo_push(struct esb_payload p, uint32_t ticks)
{
    assert(fifo_count < 32);
    fifo[fifo_count] = p;
    fifo_ticks[fifo_count++] = ticks;
}

static void test_drain(void)
{
    struct esb_payload p = {0};
    esb_pairing = true;
    raw_target = true;
    tracker_slots[2].stats.status_received = 7;
    tracker_slots[2].stats.status_lost = 3;
    tracker_slots[2].tracker_remote_command = ESB_PONG_FLAG_CALIBRATE;
    remote_confirm_cb = confirm;

    fifo_push(p, 1000);
    p = (struct esb_payload){.length = 16, .pipe = 0,
                             .data = {'S', 'A', 'C', 1, 9, 8, 7, 6, 5, 4, 42, 0x34, 0x12}};
    fifo_push(p, 1100);
    p = (struct esb_payload){.length = 8, .pipe = 0, .data = {0xfe, 0, 1, 2, 3, 4, 5, 4}};
    assert(crc8_ccitt(0x07, &p.data[2], 6) == 0xfe);
    fifo_push(p, 1200);

    p = packet(ESB_PING_TYPE, 2, 3, 4, 1290, ESB_PONG_FLAG_NORMAL);
    p.data[12] ^= 1;
    fifo_push(p, 1300);
    p = (struct esb_payload){.length = 4, .pipe = 3, .data = {ESB_COMPOSITE_TYPE, 2, 1, 3}};
    fifo_push(p, 1400);
    p = (struct esb_payload){.length = 5, .pipe = 3, .data = {ESB_COMPOSITE_TYPE, MAX_TRACKERS, 1, 3, 0}};
    fifo_push(p, 1500);
    fifo_push(packet(ESB_PING_TYPE, 2, 3, 5, 1590, ESB_PONG_FLAG_NORMAL), 1600);
    fifo_push(packet(ESB_PING_TYPE, 2, 3, 5, 1690, ESB_PONG_FLAG_CALIBRATE), 1700);
    fifo_push(packet(ESB_PING_TYPE, 2, 3, 4, 1790, ESB_PONG_FLAG_CALIBRATE), 1800);
    fifo_push(packet(ESB_PING_TYPE, 2, 3, 6, 1890, ESB_PONG_FLAG_CALIBRATE), 1900);

    p = (struct esb_payload){.length = ESB_PING_LEN, .pipe = 3, .data = {ESB_OTA_STATUS_TYPE, 2, 0x41}};
    fifo_push(p, 2000);
    p = (struct esb_payload){.length = 6, .pipe = 3, .data = {ESB_OTA_FW_INFO_TYPE, 2, 0x42}};
    fifo_push(p, 2100);
    p = (struct esb_payload){.length = 6, .pipe = 3, .rssi = -41,
                             .data = {ESB_RAW_IMU_TYPE, 2, 0, 11, 0x55, 0x66}};
    fifo_push(p, 2200);
    fifo_push(p, 2300);

    struct tracker_event e = {.nonce = 0x12345678, .event_seq = 1, .operation_id = 9,
                              .tracker_id = 2, .kind = CAL_KIND_IMU_ZRO,
                              .event = CAL_EVENT_BEGIN, .phase = CAL_PHASE_COLLECT};
    uint8_t event_packet[TRACKER_EVENT_ESB_LEN];
    assert(tracker_event_encode(event_packet, &e));
    p = (struct esb_payload){.length = TRACKER_EVENT_ESB_LEN, .pipe = 3};
    memcpy(p.data, event_packet, sizeof(event_packet));
    fifo_push(p, 2400);
    p.length = 6;
    fifo_push(p, 2500);

    p = (struct esb_payload){.length = 17, .pipe = 3, .rssi = -42,
                             .data = {ESB_COMPOSITE_TYPE, 2, 2, 3, 0x11, 0x22, 0x99}};
    p.data[16] = 20;
    fifo_push(p, 2600);
    p = (struct esb_payload){.length = 8, .pipe = 3,
                             .data = {ESB_COMPOSITE_TYPE, 2, 2, 3, 0x11, 0x22, 1, 21}};
    fifo_push(p, 2700);
    p = (struct esb_payload){.length = 17, .pipe = 3, .rssi = -43, .data = {1, 2, 0x33}};
    p.data[16] = 21;
    fifo_push(p, 2800);
    p = (struct esb_payload){.length = 20, .pipe = 3, .data = {ESB_COMPOSITE_TYPE, 2, 1, TRACKER_EVENT_ESB_TYPE}};
    memcpy(&p.data[4], &event_packet[2], TRACKER_EVENT_BODY_LEN);
    p.data[19] = 99;
    fifo_push(p, 2900);
    p = (struct esb_payload){.length = ESB_PING_LEN, .pipe = 3,
                             .data = {ESB_COMPOSITE_TYPE, 2, 1, 5, 0x44}};
    p.data[12] = 22;
    fifo_push(p, 3000);
    p = (struct esb_payload){.length = 5, .pipe = 3, .data = {0x77, 2}};
    fifo_push(p, 3100);
    fifo_push(packet(ESB_PING_TYPE, 3, 4, 1, 3190, ESB_PONG_FLAG_NORMAL), 3200);

    const struct esb_evt rx = {.evt_id = ESB_EVENT_RX_RECEIVED};
    event_handler(&rx);
    assert(fifo_next == 23 && fifo_count == 23);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    assert(tick_reads == 23 && event_tick_reads == 23);
#else
    assert(tick_reads == 0 && event_tick_reads == 0);
#endif
    assert(radio_tick_reads == 0 && pong_tick_reads == 0);
    assert(private_count == 1 && esb_pairing_msgq.count == 1);
    assert(queued_observation.home == 42 && queued_observation.generation == 7);
    assert(queued_observation.frame[0] == 'S' && queued_observation.frame[4] == 9);
    const uint8_t discovery[8] = {0xfe, 0, 1, 2, 3, 4, 5, 4};
    assert(memcmp(esb_pairing_msgq.item.packet, discovery, sizeof(discovery)) == 0);
    assert(tracker_slots[2].crc_error_count == 1 && tracker_slots[2].last_ping_counter == 6);
    assert(confirmations == 1 && confirmed_id == 2 && confirmed_flag == ESB_PONG_FLAG_CALIBRATE);
    assert(tracker_slots[2].tracker_remote_command == ESB_PONG_FLAG_NORMAL);
    assert(tracker_slots[3].last_ping_counter == 1);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    assert(tracker_diags[3].g_last_ping_isr_rx_ticks_raw == 3200);
    assert(tracker_diags[3].g_last_ping_rx_time_diff_ticks == 5);
#endif
    assert(shadow_ping_count == 5 && shadow_data_count == 3);
    assert(ota_count == 2 && ota_lengths[0] == 13 && ota_packets[0][2] == 0x41);
    assert(ota_lengths[1] == 6 && ota_packets[1][2] == 0x42);
    assert(raw_count == 1 && raw_lengths[0] == 6 && raw_packets[0][3] == 11 && raw_packets[0][4] == 0x55);
    assert(event_count == 2 && memcmp(event_packets[0], event_packet, 17) == 0);
    assert(memcmp(event_packets[1], event_packet, 17) == 0);
    assert(sequence_calls == 3 && sequence_last[2] == 22);
    printf("PROTOCOL fifo=%u private=%u pairing=%u crc=%u ping2=%u ping3=%u "
           "shadow=%u/%u confirm=%u ota=%u raw=%u events=%u sequence=%u last=%u errors=%u\n",
           fifo_next, private_count, esb_pairing_msgq.count, tracker_slots[2].crc_error_count,
           tracker_slots[2].last_ping_counter, tracker_slots[3].last_ping_counter,
           shadow_ping_count, shadow_data_count, confirmations,
           ota_count, raw_count, event_count, sequence_calls, sequence_last[2], error_count);
    for (unsigned i = 0; i < pose_count; ++i) {
        printf("PROTOCOL pose data=");
        for (unsigned j = 0; j < 16; ++j) printf("%02x", poses[i][j]);
        putchar('\n');
    }
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    assert(slot_count == 3 && slot_ticks[0] == 2600 && slot_ticks[1] == 2800 && slot_ticks[2] == 3000);
#else
    assert(slot_count == 0);
#endif
    assert(pose_count == 3 && poses[0][0] == 3 && poses[0][1] == 2);
    assert(poses[0][2] == 0x11 && poses[0][3] == 0x22 && poses[0][4] == 7 && poses[0][5] == 3);
    assert(poses[1][0] == 1 && poses[1][2] == 0x33);
    assert(poses[2][0] == 5 && poses[2][2] == 0x44);
    assert(tracker_slots[2].stats.status_received == 0 && tracker_slots[2].stats.status_lost == 0);
    assert(error_count == 2);
}

static void assert_tick_reads(void)
{
#if defined(CONFIG_TDMA_DIAGNOSTICS)
    assert(radio_tick_reads >= pong_tick_reads);
    printf("DIAGNOSTIC radio=%u pong=%u event=%u\n",
           radio_tick_reads - pong_tick_reads, pong_tick_reads, event_tick_reads);
#else
    assert(radio_tick_reads == pong_tick_reads);
    assert(event_tick_reads == 0);
#endif
    printf("PROTOCOL shadow=%u/%u seq=%u ping2=%u command=%u confirm=%u/%u/%u "
           "mismatch=%u crc=%u ota=%u\n",
           shadow_ping_count, shadow_data_count, sequence_calls, tracker_slots[2].last_ping_counter,
           tracker_slots[2].tracker_remote_command, confirmations, confirmed_id, confirmed_flag,
           tracker_slots[2].pipe_mismatch_count, tracker_slots[2].crc_error_count, ota_count);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    tick_context = TICK_EVENT;
    tracker_slots[2].tdma_config_packed = 2u | (3u << 8) | (48u << 16) | (7u << 24);
    if (strcmp(argv[1], "detailed-warning") == 0) {
        test_detailed_warning();
        printf("PASS %s\n", argv[1]);
        return 0;
    }
    if (strcmp(argv[1], "drain") == 0) {
        test_drain();
        assert_tick_reads();
        printf("PASS %s\n", argv[1]);
        return 0;
    }
    if (strcmp(argv[1], "valid") == 0) test_valid();
    else if (strcmp(argv[1], "valid-dirty") == 0) test_valid_dirty();
    else if (strcmp(argv[1], "reacquire") == 0) test_reacquire();
    else if (strcmp(argv[1], "event-fallback") == 0) test_event_fallback();
    else if (strcmp(argv[1], "interleaved-pipe") == 0) test_interleaved_pipe();
    else if (strcmp(argv[1], "ota-status") == 0) test_ota(ESB_OTA_STATUS_TYPE);
    else if (strcmp(argv[1], "ota-info") == 0) test_ota(ESB_OTA_FW_INFO_TYPE);
    else if (strcmp(argv[1], "valid-after-invalid") == 0) test_valid_after_invalid();
    else if (strcmp(argv[1], "late-metadata") == 0) test_late_metadata();
    else if (strcmp(argv[1], "duplicate") == 0) test_early_sequence_exit(true);
    else if (strcmp(argv[1], "out-of-order") == 0) test_early_sequence_exit(false);
    else test_invalid(argv[1]);
    assert_tick_reads();
    printf("PASS %s\n", argv[1]);
    return 0;
}
