/* SDK/HW leaves are modeled; selected production functions are inserted by
 * run_radio.py. No RF timing, RTOS scheduling or unbounded SDK wait proof. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "connection/tracker_event_protocol.h"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define ARG_UNUSED(x) ((void)(x))
#define BIT(n) (1u << (n))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define TEST_ALL_JOIN_CONFIG_GRACE_MS 1500
#define MAX_TRACKERS 8
#define K_NO_WAIT 0
#define CONFIG_RADIO_TX_POWER 0
#define RADIO_RETRANSMIT_DELAY 250
#define RADIO_RF_CHANNEL 42
#define ESB_RF_CHANNEL_DEFAULT 255
#define SYS_STATUS_CONNECTION_ERROR 1
#define RADIO_STATE_STATE_Disabled 0
#define ESB_PROTOCOL_ESB_DPL 1
#define ESB_MODE_PTX 0
#define ESB_MODE_PRX 1
#define ESB_TXMODE_MANUAL 1
#define ESB_EVENT_TX_SUCCESS 1
#define ESB_EVENT_TX_FAILED 2
#define ESB_EVENT_RX_RECEIVED 3
#define ESB_DEFAULT_CONFIG {0}
#define AC_FRAME_SIZE 16

/* PROTOCOL CONSTANTS */

typedef int atomic_t;
typedef int atomic_val_t;
static int atomic_get(const atomic_t *value) { return *value; }
static void atomic_set(atomic_t *value, int next) { *value = next; }
static void atomic_or(atomic_t *value, int bits) { *value |= bits; }
static void atomic_and(atomic_t *value, int bits) { *value &= bits; }
static bool atomic_cas(atomic_t *value, int old, int next)
{
    if (*value != old) return false;
    *value = next;
    return true;
}
struct esb_payload { uint8_t length, pipe; bool noack; int8_t rssi; uint8_t data[64]; };
struct esb_evt { int evt_id; unsigned tx_attempts; };
struct esb_config {
    int protocol, mode, tx_output_power, retransmit_delay, tx_mode;
    bool selective_auto_ack, use_fast_ramp_up;
    void (*event_handler)(const struct esb_evt *);
    void (*ack_handler)(const uint8_t *, uint8_t, uint32_t, struct esb_payload *, bool *, bool *);
};
static struct { unsigned STATE; } radio;
#define NRF_RADIO (&radio)
static atomic_t home_beacon_tx_result, radio_operation_busy;
static bool esb_initialized, esb_radio_fault;
static uint8_t receiver_rf_channel, base_addr_0[4], base_addr_1[4], addr_prefix[8];
static struct esb_payload rx_payload;
static unsigned info_logs, error_logs, warning_logs;
static void host_log(const char *format, ...) { (void)format; }
#define LOG_INF(...) (++info_logs, host_log(__VA_ARGS__))
#define LOG_ERR(...) (++error_logs, host_log(__VA_ARGS__))
#define LOG_WRN(...) (++warning_logs, host_log(__VA_ARGS__))
#define LOG_DBG(...) host_log(__VA_ARGS__)
static unsigned downstream_ack, downstream_pose, downstream_pairing, downstream_command;
static unsigned init_calls, disable_calls, write_calls, start_calls, rx_start_calls;
static unsigned sequence_clock, restored_at, released_at;
static int fail_init, fail_address, fail_channel, fail_write, fail_start, fail_rx;
static int fail_storage;
static unsigned prx_stop_calls, storage_calls;
#define RF_CHANNEL 1
static bool fail_disable, sdk_active, completion_pending, stale_on_init;
static bool ota_relay, ota_receiver, raw_active, raw_batch, abort_pending;
static int completion_event;
static uint32_t clock_us, physical_channel;
static struct esb_config last_config;
static struct esb_payload sent_payload, incoming;
static bool incoming_ready;
static const struct esb_payload *dma_payload;
static unsigned status_errors;
static unsigned irq_lock(void) { return 0; }
static void irq_unlock(unsigned key) { (void)key; }
static int64_t k_uptime_get(void) { return 1234; }
static uint32_t k_uptime_get_32(void) { return 1234; }
static int k_msgq_put(void *queue, const void *item, int timeout);
struct pairing_event { uint8_t packet[8]; };
static int esb_pairing_msgq;
static bool esb_pairing;
static atomic_t pairing_new_devices_blocked;
static uint8_t stored_trackers = MAX_TRACKERS;
/* SLOT DECLARATIONS */
/* DIAGNOSTIC DECLARATIONS */
static uint32_t decision_ready_mask, decision_fence_mask;
static atomic_t test_all_state_valid, test_all_enabled, test_all_confirmed_mask, test_all_ready_after_ms[MAX_TRACKERS];
static uint8_t last_pose[16];
static unsigned data_evidence;
static void (*remote_confirm_cb)(uint8_t, uint8_t);
enum tdma_shadow_evidence { TDMA_SHADOW_EVIDENCE_PING, TDMA_SHADOW_EVIDENCE_DATA };
static void tdma_shadow_observe(uint8_t id, enum tdma_shadow_evidence evidence)
{ (void)id; if (evidence == TDMA_SHADOW_EVIDENCE_DATA) ++data_evidence; }
static uint32_t k_ticks_to_us_floor32(uint32_t ticks) { return ticks; }
/* SDK conversion and console policy leaves; radio cases keep detail off. */
static uint64_t k_ticks_to_us_floor64(uint64_t ticks) { return ticks * 1000000 / 32768; }
static bool esb_get_stats_detailed_enabled(void) { return false; }
static uint16_t test_all_wire_tps(void) { return 0; }
static void test_all_invalidate_tracker(uint8_t id, int64_t at) { (void)id; (void)at; }
static const char *esb_pong_flag_name(uint8_t flag) { (void)flag; return "command"; }
static void remote_wom_receive(uint8_t id, uint8_t flag, const uint8_t *data, uint8_t counter)
{ (void)id; (void)flag; (void)data; (void)counter; ++downstream_command; }
static uint32_t sys_get_be32(const uint8_t *p)
{ return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static uint16_t sys_get_be16(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }
static uint8_t crc8_ccitt(uint8_t crc, const uint8_t *p, size_t n)
{
    while (n--) {
        crc ^= *p++;
        for (unsigned i = 0; i < 8; ++i)
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80) ? 0x07 : 0));
    }
    return crc;
}
static int esb_find_tracker(uint64_t address) { (void)address; return -1; }
static void tdma_check_slot(uint8_t id, uint32_t ticks, int8_t rssi) { (void)id; (void)ticks; (void)rssi; }
static int check_packet_sequence(uint8_t id, uint8_t sequence) { (void)id; (void)sequence; return 0; }
static void hid_write_packet_n(const uint8_t *packet, int8_t rssi)
{ (void)rssi; memcpy(last_pose, packet, 16); ++downstream_pose; }
static bool tracker_events_receive(const uint8_t *packet, uint8_t length, uint32_t now)
{ struct tracker_event e; (void)now; return tracker_event_decode(packet, length, &e); }
static bool data_collect_is_target(uint8_t id) { (void)id; return false; }
static bool data_collect_batch_is_target(uint8_t id) { (void)id; return false; }
static void data_collect_write(const uint8_t *packet, uint8_t length, int8_t rssi)
{ (void)packet; (void)length; (void)rssi; ++downstream_pose; }
static void esb_ota_relay_process_tracker_packet(const uint8_t *packet, uint8_t length)
{ (void)packet; (void)length; ++downstream_command; }
static void esb_send_remote_command(uint8_t id, uint8_t command)
{ tracker_slots[id].tracker_remote_command = command; ++downstream_command; }
static int k_msgq_get(void *queue, void *item, int timeout)
{ (void)queue; (void)item; (void)timeout; return -ENOMSG; }


static void event_handler(const struct esb_evt *event);
static void esb_ack_handler_cb(const uint8_t *, uint8_t, uint32_t,
                               struct esb_payload *, bool *, bool *);
static uint8_t esb_rf_channel_decode(uint8_t channel)
{
    return channel == 128 ? 0 : channel;
}
static void set_status(int status, bool value)
{
    assert(status == SYS_STATUS_CONNECTION_ERROR && value);
    status_errors++;
}
static int esb_init(const struct esb_config *config)
{
    init_calls++;
    last_config = *config;
    sdk_active = true; /* Failed init may still have partially touched hardware. */
    radio.STATE = 1;
    if (stale_on_init) {
        const struct esb_evt stale = {.evt_id = ESB_EVENT_TX_SUCCESS};
        event_handler(&stale);
    }
    return fail_init;
}
static void esb_disable(void)
{
    disable_calls++;
    if (!fail_disable) {
        radio.STATE = RADIO_STATE_STATE_Disabled;
        sdk_active = false;
        completion_pending = false; /* SDK removes pending IRQ/activity. */
    }
}
static void esb_auto_loss_fence(void) {}
static int esb_set_rf_channel(uint8_t channel)
{
    if (!fail_channel) physical_channel = channel;
    return fail_channel;
}
static int esb_get_rf_channel(uint32_t *channel) { *channel = physical_channel; return 0; }
static int esb_set_base_address_0(const uint8_t *address) { (void)address; return fail_address; }
static int esb_set_base_address_1(const uint8_t *address) { (void)address; return 0; }
static int esb_set_prefixes(const uint8_t *prefix, unsigned count)
{
    (void)prefix;
    assert(count == 8);
    return 0;
}
static int esb_write_payload(const struct esb_payload *payload)
{
    write_calls++;
    sent_payload = *payload;
    dma_payload = payload;
    return fail_write;
}
static int esb_start_tx(void)
{
    start_calls++;
    assert(last_config.mode == ESB_MODE_PTX);
    assert(last_config.tx_mode == ESB_TXMODE_MANUAL);
    if (!fail_start) completion_pending = true;
    return fail_start;
}
static int esb_start_rx(void)
{
    rx_start_calls++;
    assert(last_config.mode == ESB_MODE_PRX);
    assert(atomic_get(&radio_operation_busy) == 1);
    if (!fail_rx) restored_at = ++sequence_clock;
    return fail_rx;
}
static int esb_stop_rx(void)
{
    prx_stop_calls++;
    assert(last_config.mode == ESB_MODE_PRX);
    return 0;
}
static int esb_flush_rx(void) { incoming_ready=false; return 0; }
static void esb_rx_discontinuity(void) { assert(atomic_get(&radio_operation_busy)); }
static int sys_write_checked(int key, const void *data, size_t length)
{
    assert(key == RF_CHANNEL && data && length == 1);
    storage_calls++;
    return fail_storage;
}
static uint32_t k_cycle_get_32(void) { return clock_us; }
static uint32_t k_cyc_to_us_floor32(uint32_t cycles) { return cycles; }
static uint32_t k_uptime_ticks(void) { return clock_us; }
static void k_busy_wait(unsigned us)
{
    clock_us += us;
    if (sdk_active && completion_pending && completion_event) {
        struct esb_evt event = {.evt_id = completion_event};
        completion_pending = false;
        event_handler(&event);
    }
}
static int esb_read_rx_payload(struct esb_payload *payload)
{
    if (!incoming_ready) return -ENODATA;
    *payload = incoming;
    incoming_ready = false;
    return 0;
}
static bool esb_ota_relay_is_active(void) { return ota_relay; }
static bool receiver_ota_is_active(void) { return ota_receiver; }
static bool data_collect_is_active(void) { return raw_active; }
static bool data_collect_batch_is_active(void) { return raw_batch; }
static bool esb_ota_relay_abort_pending(uint8_t id) { (void)id; return abort_pending; }
static bool radio_fault;
static void esb_set_addr_unified(void)
{
    assert(radio.STATE == RADIO_STATE_STATE_Disabled);
    memset(base_addr_0, 0x62, sizeof(base_addr_0));
}

/* ACTUAL_COMMON */
#if TEST_FEATURE_ON
static uint8_t receiver_identity[6] = {1, 2, 3, 4, 5, 6};
static atomic_t initialized = 1, enabled = 1;
struct observation {
    uint8_t frame[AC_FRAME_SIZE], home;
    uint32_t generation;
    int64_t received;
};
static struct observation queued;
static unsigned queue_calls;
static int observations;
static bool queue_full;
static uint32_t auto_channel_generation(void) { return 7; }
/* ACTUAL_RECEIVE_ON */
#else
/* ACTUAL_RECEIVE_OFF */
#endif
static int k_msgq_put(void *queue, const void *item, int timeout)
{
    assert(timeout == K_NO_WAIT);
#if TEST_FEATURE_ON
    if (queue == &observations) {
        ++queue_calls;
        if (queue_full) return -ENOMSG;
        memcpy(&queued, item, sizeof(queued));
        return 0;
    }
#endif
    assert(queue == &esb_pairing_msgq);
    (void)item;
    ++downstream_pairing;
    return 0;
}
/* ACTUAL_ACK */
/* ACTUAL_EVENT */
/* ACTUAL_LIFECYCLE */
/* ACTUAL_RESTORE */

static void reset(void)
{
    home_beacon_tx_result = radio_operation_busy = 0;
    esb_initialized = esb_radio_fault = radio_fault = false;
    info_logs = error_logs = warning_logs = status_errors = 0;
    init_calls = disable_calls = write_calls = start_calls = rx_start_calls = 0;
    sequence_clock = restored_at = released_at = 0;
    fail_init = fail_address = fail_channel = fail_write = fail_start = fail_rx = 0;
    fail_storage = 0;
    prx_stop_calls = storage_calls = 0;
    fail_disable = sdk_active = completion_pending = stale_on_init = false;
    ota_relay = ota_receiver = raw_active = raw_batch = abort_pending = false;
    completion_event = ESB_EVENT_TX_SUCCESS;
    receiver_rf_channel = 42;
    physical_channel = 42;
    clock_us = 0;
    radio.STATE = RADIO_STATE_STATE_Disabled;
    dma_payload = NULL;
    downstream_ack = downstream_pose = downstream_pairing = downstream_command = 0;
    incoming_ready = false;
    data_evidence = 0;
}
static void frame_make(uint8_t frame[16])
{
    const uint8_t valid[16] = {'S','A','C',1,9,8,7,6,5,4,42,0x34,0x12,0,0,0};
    memcpy(frame, valid, 16);
}
static void receive_frame(const uint8_t *frame, uint8_t length, uint8_t pipe)
{
    memset(&incoming, 0, sizeof(incoming));
    memcpy(incoming.data, frame, length);
    incoming.length = length;
    incoming.pipe = pipe;
    incoming_ready = true;
    const struct esb_evt rx = {.evt_id = ESB_EVENT_RX_RECEIVED};
    event_handler(&rx);
}
static void assert_private(const uint8_t *frame, uint8_t length, uint8_t pipe)
{
    bool has_payload = true, suppress = false;
    struct esb_payload ack = {0};
    esb_ack_handler_cb(frame, length, pipe, &ack, &has_payload, &suppress);
    assert(!has_payload && suppress);
    receive_frame(frame, length, pipe);
    assert(downstream_ack == 0 && downstream_pose == 0);
    assert(downstream_pairing == 0 && downstream_command == 0);
}
static void test_ingress(void)
{
    reset();
    uint8_t frame[17];
    frame_make(frame);
    frame[16] = 0;
#if TEST_FEATURE_ON
    queue_calls = 0;
    assert_private(frame, 16, 0);
    assert(queue_calls == 1);
    assert(memcmp(queued.frame, frame, 16) == 0);
    assert(queued.home == 42 && queued.generation == 7 && queued.received == 1234);
    frame[4] ^= 1; /* The IRQ handoff owns a copy, never an RX buffer pointer. */
    assert(queued.frame[4] == 9);
    frame_make(frame);
    queue_calls = 0;
#endif
    assert_private(frame, 16, 3);
    for (unsigned length = 3; length <= 17; length++) {
        if (length != 16) assert_private(frame, length, 0);
    }
    frame[3] = 2;
    assert_private(frame, 16, 0);
    frame_make(frame);
    for (unsigned i = 13; i < 16; i++) {
        frame[i] = 1;
        assert_private(frame, 16, 0);
        frame[i] = 0;
    }
    memset(frame + 4, 0, 6);
    assert_private(frame, 16, 0);
    frame_make(frame);
    frame[10] = 40;
    assert_private(frame, 16, 0);
    frame_make(frame);
    physical_channel = 101;
    assert_private(frame, 16, 0);
    physical_channel = 42;
#if TEST_FEATURE_ON
    memcpy(frame + 4, receiver_identity, 6);
    assert_private(frame, 16, 0);
    frame_make(frame);
    initialized = 0;
    assert_private(frame, 16, 0);
    initialized = 1;
    enabled = 0;
    assert_private(frame, 16, 0);
    enabled = 1;
    assert(queue_calls == 0);
    queue_full = true;
    assert_private(frame, 16, 0);
    assert(queue_calls == 1); /* A full queue is a nonblocking drop, not dispatch. */
    queue_full = false;
#else
    assert_private(frame, 16, 0);
#endif
    /* Ordinary packets must reach the downstream boundary, not be swallowed. */
    frame[0] = 1;
    frame[1] = 0;
    frame[16] = 0x42;
    bool has_payload = true, suppress = true;
    struct esb_payload ack = {0};
    esb_ack_handler_cb(frame, 17, 1, &ack, &has_payload, &suppress);
    receive_frame(frame, 17, 1);
    assert(downstream_ack == 1 && !has_payload && !suppress);
    assert(downstream_pose == 1 && downstream_pairing == 0 && downstream_command == 0);
    assert(data_evidence == 1 && memcmp(last_pose, frame, 16) == 0);
}
static void test_commit_logging(void)
{
    reset();
    assert(esb_radio_operation_begin() == 0);
    assert(esb_initialize(false) == 0);
    unsigned before = info_logs;
    assert(channel_apply(84) == 0);
    assert(receiver_rf_channel == 84 && physical_channel == 84);
    assert(info_logs == before + 1 && storage_calls == 1);
    assert(channel_apply(84) == 0);
    assert(info_logs == before + 1);
    fail_storage = -ENOSPC;
    assert(channel_apply(80) == -ENOSPC);
    assert(receiver_rf_channel == 84 && physical_channel == 84);
    assert(info_logs == before + 1);
    fail_storage = 0;
    assert(channel_apply(ESB_RF_CHANNEL_DEFAULT) == 0);
    assert(physical_channel == RADIO_RF_CHANNEL && info_logs == before + 2);
    assert(channel_apply(RADIO_RF_CHANNEL) == 0);
    assert(info_logs == before + 2);
}
static void test_init_logging(void)
{
    reset();
    assert(esb_initialize(false) == 0);
    assert(info_logs == 2 && error_logs == 0);
    assert(esb_radio_quiesce() == 0);
    unsigned before = info_logs;
    assert(esb_initialize_quiet(false) == 0);
    assert(info_logs == before);
    assert(esb_initialize_quiet(false) == -EALREADY && warning_logs == 1);
    assert(esb_radio_quiesce() == 0);
    receiver_rf_channel = 84; /* Startup from a different saved home remains visible. */
    assert(esb_initialize(false) == 0 && physical_channel == 84);
    assert(info_logs == before + 2);
    for (unsigned quiet = 0; quiet < 2; quiet++) {
        for (unsigned failure = 0; failure < 3; failure++) {
            reset();
            if (failure == 0) fail_init = -EINVAL;
            if (failure == 1) fail_channel = -EINVAL;
            if (failure == 2) fail_address = -EINVAL;
            assert(esb_initialize_internal(false, quiet) == -EINVAL);
            assert(error_logs == 1 && status_errors == 1 && disable_calls == 1);
            assert(!esb_initialized && !sdk_active);
            if (quiet) assert(info_logs == 0);
            else assert(info_logs > 0);
        }
    }
}
static void test_transmit(void)
{
    uint8_t frame[16];
    frame_make(frame);
    for (unsigned failure = 0; failure < 6; failure++) {
        reset();
        assert(esb_radio_operation_begin() == 0);
        if (failure == 1) completion_event = ESB_EVENT_TX_FAILED;
        if (failure == 2) fail_init = -EINVAL;
        if (failure == 3) fail_write = -ENOSPC;
        if (failure == 4) fail_start = -EBUSY;
        if (failure == 5) completion_event = 0;
        const int expected[] = {0, -EIO, -EINVAL, -ENOSPC, -EBUSY, -ETIMEDOUT};
        assert(esb_home_beacon_send(frame) == expected[failure]);
        assert(atomic_get(&radio_operation_busy) == 1);
        assert(!sdk_active && !completion_pending && !esb_initialized);
        assert(disable_calls >= 1 && info_logs == 0);
        assert(prx_stop_calls == 0);
        assert(write_calls == (failure == 2 ? 0u : 1u));
        assert(start_calls == ((failure == 2 || failure == 3) ? 0u : 1u));
        if (write_calls) {
            assert(sent_payload.pipe == 0 && sent_payload.noack && sent_payload.length == 16);
            assert(memcmp(sent_payload.data, frame, 16) == 0);
            assert(dma_payload && memcmp(dma_payload->data, frame, 16) == 0);
            assert(physical_channel == 42 && last_config.selective_auto_ack);
        }
        if (failure == 5) assert(clock_us == 2000);
        fail_init = 0; /* The injected PTX initialization failure has ended. */
        assert(restore() == 0);
        assert(restored_at && radio_operation_busy == 1);
        assert(prx_stop_calls == 0 && info_logs == 0);
        esb_radio_operation_end();
        released_at = ++sequence_clock;
        assert(restored_at < released_at && radio_operation_busy == 0);
    }
    reset();
    assert(esb_home_beacon_send(NULL) == -EINVAL);
    assert(init_calls == 0 && disable_calls == 1);
    for (unsigned channel = 0; channel <= 88; channel += 44) {
        reset();
        receiver_rf_channel = channel == 0 ? 128 : (uint8_t)channel;
        assert(esb_home_beacon_send(frame) == 0 && physical_channel == channel);
    }
    reset();
    completion_event = 0;
    assert(esb_home_beacon_send(frame) == -ETIMEDOUT);
    assert(!completion_pending);
    const struct esb_evt late = {.evt_id = ESB_EVENT_TX_SUCCESS};
    event_handler(&late); /* Prior/PRX result cannot satisfy the next PTX send. */
    stale_on_init = true;
    assert(esb_home_beacon_send(frame) == -ETIMEDOUT);
    assert(clock_us == 4000 && !completion_pending);
    stale_on_init = false;
    completion_event = ESB_EVENT_TX_SUCCESS;
    assert(esb_home_beacon_send(frame) == 0);
}
static void test_restore_failure(void)
{
    for (unsigned failure = 0; failure < 3; failure++) {
        reset();
        assert(esb_radio_operation_begin() == 0);
        if (failure == 0) { radio.STATE = 1; fail_disable = true; }
        if (failure == 1) fail_init = -EINVAL;
        if (failure == 2) fail_rx = -EBUSY;
        assert(restore() < 0 && radio_fault && radio_operation_busy == 1);
        assert(error_logs > 0 && restored_at == 0);
        /* Actual auto_step decides not to release on its sticky radio_fault;
         * worker harness covers that orchestration, not a reimplementation here. */
        if (failure == 0) {
            esb_radio_operation_end();
            assert(radio_operation_busy == 1);
            fail_disable = false;
            assert(esb_radio_quiesce() == -EIO);
            assert(esb_initialize_quiet(false) == -EIO);
            assert(esb_radio_operation_begin() == -EIO);
        }
    }
    reset();
    uint8_t frame[16];
    frame_make(frame);
    assert(esb_radio_operation_begin() == 0);
    fail_disable = true;
    assert(esb_home_beacon_send(frame) == -EIO);
    assert(esb_radio_fault && dma_payload && memcmp(dma_payload->data, frame, 16) == 0);
    esb_radio_operation_end();
    assert(radio_operation_busy == 1);
    for (unsigned busy = 0; busy < 5; busy++) {
        reset();
        if (busy == 0) ota_relay = true;
        if (busy == 1) ota_receiver = true;
        if (busy == 2) raw_active = true;
        if (busy == 3) raw_batch = true;
        if (busy == 4) abort_pending = true;
        assert(esb_radio_operation_begin() == -EBUSY);
        assert(radio_operation_busy == 0 && init_calls == 0 && disable_calls == 0);
    }
    reset();
    assert(esb_radio_operation_begin() == 0);
    assert(esb_radio_operation_begin() == -EBUSY);
    assert(esb_radio_operation_stop() == 0);
}
int main(void)
{
    test_ingress();
    test_init_logging();
    test_commit_logging();
    test_transmit();
    test_restore_failure();
    printf("auto-channel actual radio lifecycle/ingress: feature=%d passed\n", TEST_FEATURE_ON);
    return 0;
}
