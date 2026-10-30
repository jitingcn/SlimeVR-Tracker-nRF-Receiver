#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CONFIG_ESB_MAX_PAYLOAD_LENGTH 64
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define __DMB() ((void)0)

#include "globals.h"

struct esb_evt;
struct esb_payload {
    uint8_t data[64];
    uint8_t length;
    uint32_t pipe;
    bool noack;
};

static unsigned irq_mask, spin_depth, deferred_irqs, delivered_irqs;
static void (*pending_irq)(void);
static void (*next_spin_unlock_irq)(void);
static void (*spin_entry_irq)(void);
static unsigned spin_entry_at, spin_entries;
static uint32_t now_ticks;

static void deliver_pending(void)
{
    if (!irq_mask && pending_irq) {
        void (*fn)(void) = pending_irq;
        pending_irq = NULL;
        delivered_irqs++;
        fn();
    }
}

static void request_interrupt(void (*fn)(void))
{
    assert(fn);
    if (irq_mask) {
        assert(!pending_irq);
        pending_irq = fn;
        deferred_irqs++;
    } else {
        delivered_irqs++;
        fn();
    }
}

static unsigned int irq_lock(void)
{
    unsigned int old = irq_mask;
    irq_mask = 1;
    return old;
}

static void irq_unlock(unsigned int key)
{
    assert(irq_mask && key <= 1);
    irq_mask = key;
    deliver_pending();
}

struct k_spinlock { bool held; };
typedef unsigned int k_spinlock_key_t;

static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock)
{
    if (spin_entry_irq && ++spin_entries == spin_entry_at) {
        void (*fn)(void) = spin_entry_irq;
        spin_entry_irq = NULL;
        request_interrupt(fn);
    }
    unsigned int key = irq_lock();
    assert(!lock->held && spin_depth == 0);
    lock->held = true;
    spin_depth++;
    return key;
}

static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
    assert(lock->held && spin_depth == 1 && irq_mask);
    lock->held = false;
    spin_depth--;
    if (next_spin_unlock_irq) {
        void (*fn)(void) = next_spin_unlock_irq;
        next_spin_unlock_irq = NULL;
        request_interrupt(fn);
    }
    irq_unlock(key);
}

static void external_call(void) { assert(spin_depth == 0); }

static uint32_t k_uptime_ticks(void)
{
    external_call();
    return now_ticks;
}

static int64_t k_uptime_get(void) { external_call(); return now_ticks; }
static uint32_t k_ms_to_ticks_floor32(uint32_t ms) { return ms; }

struct k_work { void (*handler)(struct k_work *); };
#define K_WORK_DEFINE(name, function) struct k_work name = { .handler = function }
static struct k_work *submitted_work;
static unsigned work_submits, sleeps, log_calls, print_calls;
static unsigned confirmed_type, confirmed_tracker, confirmed_sends;
static int k_work_submit(struct k_work *work)
{
    external_call();
    submitted_work = work;
    work_submits++;
    return 0;
}
static void k_msleep(int ms) { external_call(); assert(ms == 2); sleeps++; }
static void test_log(const char *format, ...)
{
    external_call();
    log_calls++;
    if (strstr(format, "confirmed by tracker")) {
        va_list args;
        va_start(args, format);
        confirmed_type = (unsigned)va_arg(args, int);
        confirmed_tracker = (unsigned)va_arg(args, int);
        confirmed_sends = va_arg(args, unsigned);
        va_end(args);
    }
}
#define LOG_MODULE_REGISTER(...)
#define LOG_INF(...) test_log(__VA_ARGS__)
#define LOG_WRN(...) test_log(__VA_ARGS__)
#define LOG_ERR(...) test_log(__VA_ARGS__)
#define LOG_DBG(...) test_log(__VA_ARGS__)
static int printk(const char *format, ...) { external_call(); (void)format; print_calls++; return 0; }

static uint16_t sys_get_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}
static uint32_t sys_get_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static void sys_put_be16(uint16_t n, uint8_t *p) { p[0] = n >> 8; p[1] = n; }
static void sys_put_be32(uint32_t n, uint8_t *p)
{
    p[0] = n >> 24; p[1] = n >> 16; p[2] = n >> 8; p[3] = n;
}
static void sys_put_le32(uint32_t n, uint8_t *p)
{
    p[0] = n; p[1] = n >> 8; p[2] = n >> 16; p[3] = n >> 24;
}
static uint8_t crc8_ccitt(uint8_t crc, const void *buffer, size_t size)
{
    const uint8_t *p = buffer;
    while (size--) {
        crc ^= *p++;
        for (unsigned i = 0; i < 8; i++) {
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80) ? 0x07 : 0));
        }
    }
    return crc;
}

uint8_t stored_trackers = 4;
typedef int atomic_t;
static atomic_t radio_operation_busy;
static bool esb_radio_fault, receiver_ota_active, collect_active, collect_batch_active;
#define radio_owned (radio_operation_busy != 0)
static unsigned radio_begins, radio_ends, loss_invalidations, abort_clears;
static unsigned radio_claims, radio_releases;
static void (*radio_begin_hook)(void);
static struct { uint8_t id, flag; } commands[256];
static size_t command_count;
static uint8_t reports[512][16];
static size_t report_count;

bool esb_ota_relay_is_active(void);
static bool receiver_ota_is_active(void) { return receiver_ota_active; }
static bool data_collect_is_active(void) { return collect_active; }
static bool data_collect_batch_is_active(void) { return collect_batch_active; }
static bool atomic_cas(atomic_t *value, int old, int next)
{
    assert(value == &radio_operation_busy && old == 0 && next == 1);
    if (*value != old) return false;
    *value = next;
    radio_claims++;
    return true;
}
static void atomic_set(atomic_t *value, int next)
{
    assert(value == &radio_operation_busy && *value == 1 && next == 0);
    *value = next;
    radio_releases++;
}
bool esb_ota_relay_abort_pending(uint8_t tracker_id);
void esb_send_remote_command(uint8_t id, uint8_t flag)
{
    external_call();
    assert(command_count < sizeof(commands) / sizeof(commands[0]));
    commands[command_count].id = id;
    commands[command_count++].flag = flag;
    if (flag == 0x31) {
        assert(esb_ota_relay_abort_pending(id));
    }
}
void esb_clear_remote_ota_abort(uint8_t id)
{
    external_call();
    assert(irq_mask);
    assert(!esb_ota_relay_abort_pending(id));
    abort_clears++;
}
#define esb_radio_operation_begin production_esb_radio_operation_begin
#define esb_radio_operation_end production_esb_radio_operation_end
#include RADIO_SOURCE
#undef esb_radio_operation_end
#undef esb_radio_operation_begin

int esb_radio_operation_begin(void)
{
    external_call();
    radio_begins++;
    if (radio_begin_hook) {
        void (*fn)(void) = radio_begin_hook;
        radio_begin_hook = NULL;
        fn();
    }
    return production_esb_radio_operation_begin();
}
void esb_radio_operation_end(void)
{
    external_call();
    assert(radio_owned);
    production_esb_radio_operation_end();
    radio_ends++;
}
void esb_auto_loss_invalidate(void) { external_call(); loss_invalidations++; }
void hid_write_packet_n(const uint8_t *data, uint8_t rssi)
{
    external_call();
    assert(rssi == 0 && report_count < sizeof(reports) / sizeof(reports[0]));
    memcpy(reports[report_count++], data, 16);
}

static void *watched_copy;
static const void *watched_source;
static size_t watched_byte;
static void (*copy_irq)(void);
static bool copy_fired;
static uintptr_t tracked_start, tracked_end;
static void *watched_fields[4];
static unsigned store_target, store_seen, store_fields;
static void (*store_irq)(void);
static bool store_fired;

static void *test_memcpy(void *destination, const void *source, size_t length)
{
    uint8_t *out = destination;
    const uint8_t *in = source;
    for (size_t i = 0; i < length; i++) {
        out[i] = in[i];
        if (((watched_copy && destination == watched_copy) ||
             (watched_source && source == watched_source)) &&
            i + 1 == watched_byte && copy_irq) {
            void (*fn)(void) = copy_irq;
            copy_irq = NULL;
            copy_fired = true;
            request_interrupt(fn);
        }
    }
    return destination;
}

static void *test_memset(void *destination, int value, size_t length)
{
    uint8_t *out = destination;
    for (size_t i = 0; i < length; i++) {
        out[i] = (uint8_t)value;
        if (watched_copy && destination == watched_copy &&
            i + 1 == watched_byte && copy_irq) {
            void (*fn)(void) = copy_irq;
            copy_irq = NULL;
            copy_fired = true;
            request_interrupt(fn);
        }
    }
    return destination;
}

static void test_after_store(void *address)
{
    uintptr_t p = (uintptr_t)address;
    if (p >= tracked_start && p < tracked_end) assert(spin_depth == 1);
    for (unsigned i = 0; i < 4; i++) {
        if (address == watched_fields[i] && store_target) {
            store_fields |= 1u << i;
            store_seen++;
            if (store_seen == store_target) {
                void (*fn)(void) = store_irq;
                store_target = 0;
                store_fired = true;
                request_interrupt(fn);
            }
            break;
        }
    }
}

#define memcpy test_memcpy
#define memset test_memset
#include RELAY_SOURCE
#undef memset
#undef memcpy

static const uint8_t begin_a[64] = {
    [0] = 0xf2, [2] = 0x40, [6] = 0x44, [7] = 0x33,
    [8] = 0x22, [9] = 0x11, [11] = 1, [12] = 1,
    [13] = 'A', [61] = 0x34, [62] = 0x12,
};
static const uint8_t begin_b[64] = {
    [0] = 0xf2, [2] = 0x78, [3] = 0x56, [4] = 0x34, [5] = 0x12,
    [6] = 0xef, [7] = 0xcd, [8] = 0xab, [9] = 0x90,
    [10] = 1, [11] = 0x23, [12] = 1, [13] = 'X',
    [61] = 0xcd, [62] = 0xab,
};
static const uint8_t begin_ack_a[64] = {
    [0] = 0x23, [2] = 0x40, [6] = 0x44, [7] = 0x33,
    [8] = 0x22, [9] = 0x11, [11] = 1, [12] = 1,
    [13] = 'A', [61] = 0x34, [62] = 0x12, [63] = 0x86,
};
static const uint8_t begin_ack_a_one[64] = {
    [0] = 0x23, [1] = 1, [2] = 0x40, [6] = 0x44, [7] = 0x33,
    [8] = 0x22, [9] = 0x11, [11] = 1, [12] = 1,
    [13] = 'A', [61] = 0x34, [62] = 0x12, [63] = 0xbb,
};
static const uint8_t begin_ack_b[64] = {
    [0] = 0x23, [2] = 0x78, [3] = 0x56, [4] = 0x34, [5] = 0x12,
    [6] = 0xef, [7] = 0xcd, [8] = 0xab, [9] = 0x90,
    [10] = 1, [11] = 0x23, [12] = 1, [13] = 'X',
    [61] = 0xcd, [62] = 0xab, [63] = 0x9d,
};
static const uint8_t verify_ack[4] = {0x24, 0, 0, 0};
static const uint8_t activate_ack[4] = {0x25, 0, 0, 0};
static const uint8_t data_a[7] = {0x20, 0, 0, 0, 0xa1, 0xb2, 0xc3};
static const uint8_t data_b[7] = {0x20, 0, 0, 0, 0xd4, 0xe5, 0xf6};
static const uint8_t abort_at_100[13] = {0xf1, 0, 0, 0, 0, 0, 100, 0x31, 0, 0, 0, 0, 0x21};

static void reset_world(void)
{
    assert(!irq_mask && !spin_depth && !pending_irq && !radio_owned);
    memset(&ota_relay, 0, sizeof(ota_relay));
    memset(ota_abort_valid, 0, sizeof(ota_abort_valid));
    memset(ota_abort_expiry_ticks, 0, sizeof(ota_abort_expiry_ticks));
    memset(&fw_info_pending, 0, sizeof(fw_info_pending));
#if !EXPECT_UNSAFE
    memset(&ota_relay_lock, 0, sizeof(ota_relay_lock));
#endif
    now_ticks = 100;
    stored_trackers = 4;
    esb_radio_fault = receiver_ota_active = collect_active = collect_batch_active = false;
    radio_begins = radio_ends = loss_invalidations = abort_clears = 0;
    radio_claims = radio_releases = 0;
    command_count = report_count = 0;
    deferred_irqs = delivered_irqs = spin_entries = 0;
    work_submits = sleeps = log_calls = print_calls = 0;
    confirmed_type = confirmed_tracker = confirmed_sends = 0;
    submitted_work = NULL;
    watched_copy = NULL;
    watched_source = NULL;
    copy_irq = NULL;
    copy_fired = false;
    radio_begin_hook = pending_irq = next_spin_unlock_irq = spin_entry_irq = NULL;
    store_target = store_seen = store_fields = 0;
    store_irq = NULL;
    store_fired = false;
    tracked_start = (uintptr_t)&ota_relay;
    tracked_end = tracked_start + sizeof(ota_relay);
    memset(watched_fields, 0, sizeof(watched_fields));
}

static void begin(uint8_t id, const uint8_t packet[64])
{
    uint8_t hid[64];
    memcpy(hid, packet, sizeof(hid));
    hid[1] = id;
    esb_ota_relay_process_hid(hid, sizeof(hid));
}
static void simple(uint8_t type, uint8_t id)
{
    uint8_t hid[2] = {type, id};
    esb_ota_relay_process_hid(hid, sizeof(hid));
}
static void status(uint8_t id, uint8_t state, uint16_t seq, uint32_t written)
{
    uint8_t packet[9] = {0x21, id, state, (uint8_t)(seq >> 8), (uint8_t)seq,
                         (uint8_t)written, (uint8_t)(written >> 8),
                         (uint8_t)(written >> 16), (uint8_t)(written >> 24)};
    esb_ota_relay_process_tracker_packet(packet, sizeof(packet));
}
static void data(uint16_t seq, uint8_t a, uint8_t b, uint8_t c)
{
    uint8_t hid[7] = {0xf3, 0, (uint8_t)(seq >> 8), (uint8_t)seq, a, b, c};
    esb_ota_relay_process_hid(hid, sizeof(hid));
}
struct result { bool has; struct esb_payload packet; };
static struct result ack_with_rx(uint8_t id, const uint8_t *rx, uint8_t length)
{
    struct result out = {0};
    out.packet.noack = true;
    esb_ota_relay_fill_ack(id, 3, &out.packet, &out.has, rx, length);
    return out;
}
static struct result ack(uint8_t id, uint16_t seq)
{
    uint8_t rx[5] = {0x21, id, 0x02, (uint8_t)(seq >> 8), (uint8_t)seq};
    return ack_with_rx(id, rx, sizeof(rx));
}
static void expect_ack(struct result r, const uint8_t *expected, size_t length)
{
    assert(r.has && r.packet.pipe == 3 && !r.packet.noack);
    assert(r.packet.length == length && memcmp(r.packet.data, expected, length) == 0);
}
static void setup_ready(uint8_t id)
{
    begin(id, begin_a);
    status(id, 0x01, 0, 0);
}

static struct result preempt_ack;
static void take_ack(void) { preempt_ack = ack(0, 0); }
static void cancel_zero(void) { simple(0xf7, 0); }
static void confirm_verify(void) { status(0, 0x03, 0, 0); }

static void test_publication_copy(void)
{
    for (size_t byte = 1; byte <= 64; byte++) {
        reset_world();
        begin(0, begin_a);
        expect_ack(ack(0, 0), begin_ack_a, 64);
        watched_copy = ota_relay.tracker[0].pending_cmd_data;
        watched_byte = byte;
        copy_irq = take_ack;
        begin(0, begin_b);
        assert(copy_fired && deferred_irqs == 1 && delivered_irqs == 1);
        expect_ack(preempt_ack, begin_ack_b, 64);
        expect_ack(ack(0, 0), begin_ack_b, 64);
        assert(radio_begins == 1 && radio_ends == 1);

        reset_world();
        setup_ready(0);
        simple(0xf5, 0);
        watched_copy = ota_relay.tracker[0].pending_cmd_data;
        watched_byte = byte;
        copy_irq = take_ack;
        begin(0, begin_b);
        assert(copy_fired && deferred_irqs == 1 && delivered_irqs == 1);
        expect_ack(preempt_ack, begin_ack_b, 64);
    }
    for (size_t byte = 1; byte <= 4; byte++) {
        reset_world();
        setup_ready(0);
        simple(0xf5, 0);
        expect_ack(ack_with_rx(0, NULL, 0), verify_ack, 4);
        watched_copy = ota_relay.tracker[0].pending_cmd_data;
        watched_byte = byte;
        copy_irq = take_ack;
        simple(0xf6, 0);
        assert(copy_fired && deferred_irqs == 1 && delivered_irqs == 1);
        expect_ack(preempt_ack, activate_ack, 4);

        reset_world();
        setup_ready(0);
        simple(0xf5, 0);
        watched_copy = ota_relay.tracker[0].pending_cmd_data;
        watched_byte = byte;
        copy_irq = confirm_verify;
        simple(0xf6, 0);
        assert(copy_fired && deferred_irqs == 1 && delivered_irqs == 1);
        expect_ack(ack_with_rx(0, NULL, 0), activate_ack, 4);

        reset_world();
        begin(0, begin_a);
        watched_copy = ota_relay.tracker[0].pending_cmd_data;
        watched_byte = byte;
        copy_irq = take_ack;
        simple(0xf5, 0);
        assert(copy_fired && deferred_irqs == 1 && delivered_irqs == 1);
        expect_ack(preempt_ack, verify_ack, 4);
    }
}

static void test_publication_stores(void)
{
#if INSTRUMENTED
    unsigned seen_fields = 0, points = 0;
    for (unsigned point = 1; point < 32; point++) {
        reset_world();
        setup_ready(0);
        simple(0xf5, 0);
        watched_fields[0] = &ota_relay.tracker[0].pending_cmd_type;
        watched_fields[1] = &ota_relay.tracker[0].pending_cmd_len;
        watched_fields[2] = &ota_relay.tracker[0].pending_cmd_set_ticks;
        watched_fields[3] = &ota_relay.tracker[0].pending_cmd_send_count;
        store_target = point;
        store_irq = take_ack;
        simple(0xf6, 0);
        seen_fields |= store_fields;
        if (!store_fired) break;
        points++;
        assert(deferred_irqs == 1 && delivered_irqs == 1);
        expect_ack(preempt_ack, activate_ack, 4);
        expect_ack(ack_with_rx(0, NULL, 0), activate_ack, 4);
    }
    assert(points >= 4 && seen_fields == 0x0f);
#endif
}

static void test_confirmation_preemption(void)
{
    reset_world();
    setup_ready(0);
    simple(0xf5, 0);
    next_spin_unlock_irq = take_ack;
    status(0, 0x03, 0, 0);
    assert(!next_spin_unlock_irq && deferred_irqs == 1 && delivered_irqs == 1);
    assert(!preempt_ack.has);
    simple(0xf6, 0);
    expect_ack(ack_with_rx(0, NULL, 0), activate_ack, 4);
#if INSTRUMENTED
    reset_world();
    setup_ready(0);
    simple(0xf5, 0);
    watched_fields[0] = &ota_relay.tracker[0].pending_cmd_type;
    store_target = 1;
    store_irq = take_ack;
    status(0, 0x03, 0, 0);
    assert(store_fired && deferred_irqs == 1 && delivered_irqs == 1);
    assert(!preempt_ack.has);
#endif
}

static void test_confirmations(void)
{
    static const struct { uint8_t state; bool terminal; } states[] = {
        {0x00, false}, {0x01, false}, {0x02, false}, {0x03, false},
        {0x04, true}, {0x05, false}, {0x06, true}, {0x10, true},
        {0x11, true}, {0x12, true}, {0x13, true}, {0x14, true},
        {0x15, true}, {0x7f, false},
    };
    for (unsigned command = 0; command < 3; command++) {
        for (unsigned i = 0; i < sizeof(states) / sizeof(states[0]); i++) {
            reset_world();
            if (command == 0) {
                begin(0, begin_a);
            } else {
                setup_ready(0);
                simple(command == 1 ? 0xf5 : 0xf6, 0);
            }
            uint8_t s = states[i].state;
            bool confirmed = command == 0 ? s != 0 :
                             command == 1 ? (s == 3 || s == 4) : (s == 5 || s == 6);
            status(0, s, 0, 0);
            if (states[i].terminal) {
                assert(!esb_ota_relay_is_target(0));
                assert(ota_relay.tracker[0].pending_cmd_type ==
                       (confirmed ? 0 : (uint8_t)(0x23 + command)));
                assert(!ack(0, 0).has);
            } else if (confirmed) {
                assert(esb_ota_relay_is_target(0) && !ack(0, 0).has);
            } else if (command == 0) {
                expect_ack(ack(0, 0), begin_ack_a, 64);
            } else {
                expect_ack(ack(0, 0), command == 1 ? verify_ack : activate_ack, 4);
            }
        }
    }

    reset_world();
    setup_ready(0);
    simple(0xf5, 0);
    expect_ack(ack_with_rx(0, NULL, 0), verify_ack, 4);
    expect_ack(ack_with_rx(0, NULL, 0), verify_ack, 4);
    simple(0xf6, 0);
    expect_ack(ack_with_rx(0, NULL, 0), activate_ack, 4);
    status(0, 0x05, 0, 0);
    assert(confirmed_type == 0x25 && confirmed_tracker == 0 && confirmed_sends == 1);
}

static void test_timeout_and_deferral(void)
{
    reset_world();
    begin(0, begin_a);
    data(0, 0xa1, 0xb2, 0xc3);
    expect_ack(ack(0, 0), begin_ack_a, 64);
    now_ticks = 10100;
    expect_ack(ack(0, 0), begin_ack_a, 64);
    now_ticks = 10101;
    assert(!ack(0, 0).has);
    expect_ack(ack(0, 0), data_a, 7);

    reset_world();
    now_ticks = 0xfffffff0u;
    begin(0, begin_a);
    now_ticks = 9984;
    expect_ack(ack(0, 0), begin_ack_a, 64);
    now_ticks = 9985;
    assert(!ack(0, 0).has);

    for (unsigned command = 0; command < 2; command++) {
        reset_world();
        setup_ready(0);
        data(0, 0xa1, 0xb2, 0xc3);
        simple(command ? 0xf6 : 0xf5, 0);
        const uint8_t *cmd = command ? activate_ack : verify_ack;
        expect_ack(ack_with_rx(0, NULL, 0), cmd, 4);
        uint8_t wrong[5] = {0x20, 0, 0, 0, 0};
        expect_ack(ack_with_rx(0, wrong, sizeof(wrong)), cmd, 4);
        uint8_t short_status[4] = {0x21, 0, 2, 0};
        expect_ack(ack_with_rx(0, short_status, sizeof(short_status)), cmd, 4);
        expect_ack(ack(0, 1), cmd, 4);
        expect_ack(ack(0, 0), data_a, 7);
        expect_ack(ack(0, 0), data_a, 7);
        now_ticks = 10100;
        expect_ack(ack(0, 0), data_a, 7);
        now_ticks = 10101;
        assert(!ack(0, 0).has);
        expect_ack(ack(0, 0), data_a, 7);
    }
}

static void test_ring(void)
{
    for (size_t byte = 1; byte <= 3; byte++) {
        reset_world();
        setup_ready(0);
        data(0, 0xa1, 0xb2, 0xc3);
        expect_ack(ack(0, 0), data_a, 7);
        watched_copy = &ota_relay.ring[0].data[4];
        watched_byte = byte;
        copy_irq = take_ack;
        data(0, 0xd4, 0xe5, 0xf6);
        assert(copy_fired && deferred_irqs == 1 && delivered_irqs == 1);
        expect_ack(preempt_ack, data_b, 7);
        expect_ack(ack(0, 0), data_b, 7);
    }
    reset_world();
    setup_ready(0);
    setup_ready(1);
    assert(!ack(0, 0).has);
    for (unsigned i = 0; i < 128; i++) data((uint16_t)i, (uint8_t)i, 0x55, 0x66);
    data(128, 0xaa, 0xbb, 0xcc);
    assert(!ack(0, 128).has);
    const uint8_t first[7] = {0x20, 1, 0, 0, 0, 0x55, 0x66};
    expect_ack(ack(1, 0), first, 7);
    const uint8_t last[7] = {0x20, 1, 0, 127, 127, 0x55, 0x66};
    expect_ack(ack(1, 127), last, 7);
    expect_ack(ack(1, 0), first, 7);
    simple(0xf7, 1);
    assert(esb_ota_relay_is_active() && !esb_ota_relay_is_target(1));
    watched_copy = &ota_relay.ring[0].data[4];
    watched_byte = 1;
    copy_irq = take_ack;
    data(128, 0xaa, 0xbb, 0xcc);
    assert(copy_fired && deferred_irqs == 1 && delivered_irqs == 1);
    assert(!preempt_ack.has);
    const uint8_t wrapped[7] = {0x20, 0, 0, 128, 0xaa, 0xbb, 0xcc};
    expect_ack(ack(0, 128), wrapped, 7);
    expect_ack(ack(0, 128), wrapped, 7);
    assert(!ack(0, 0).has);

    reset_world();
    setup_ready(0);
    watched_copy = &ota_relay.ring[0].data[4];
    watched_byte = 2;
    copy_irq = cancel_zero;
    data(0, 0xa1, 0xb2, 0xc3);
    assert(copy_fired && deferred_irqs == 1 && delivered_irqs == 1);
    assert(!esb_ota_relay_is_active() && esb_ota_relay_abort_pending(0));
    begin(0, begin_a);
    assert(!esb_ota_relay_is_active() && esb_ota_relay_abort_pending(0));
    expect_ack(ack(0, 0), abort_at_100, sizeof(abort_at_100));
    now_ticks = 10100;
    begin(0, begin_a);
    status(0, 0x01, 0, 0);
    assert(!ack(0, 0).has);
}

static void fill_capacity(void)
{
    for (uint8_t id = 0; id < 4; id++) begin(id, begin_a);
}
static void begin_one(void) { begin(1, begin_a); }
static void begin_two(void) { begin(2, begin_a); }

static void test_reservation(void)
{
    reset_world();
    radio_operation_busy = 1;
    begin(0, begin_a);
    assert(!esb_ota_relay_is_active() && radio_begins == 1 && radio_ends == 0);
    assert(radio_owned && loss_invalidations == 0 && abort_clears == 0 && !ack(0, 0).has);
    radio_operation_busy = 0;

    reset_world();
    simple(0xf7, 0);
    begin(0, begin_a);
    assert(!esb_ota_relay_is_active() && esb_ota_relay_abort_pending(0));
    expect_ack(ack(0, 0), abort_at_100, sizeof(abort_at_100));
    assert(abort_clears == 0 && loss_invalidations == 0);
    assert(radio_begins == 1 && radio_ends == 0 && !radio_owned);
    now_ticks = 10100;
    begin(0, begin_a);
    assert(esb_ota_relay_is_target(0) && !esb_ota_relay_abort_pending(0));
    assert(radio_begins == 2 && radio_ends == 1 && !radio_owned);
    expect_ack(ack(0, 0), begin_ack_a, 64);

    reset_world();
    simple(0xf7, 15);
    begin(0, begin_a);
    assert(!esb_ota_relay_is_active() && esb_ota_relay_abort_pending(15));
    assert(radio_begins == 1 && radio_ends == 0 && !radio_owned && abort_clears == 0);
    assert(!ack(0, 0).has);
    now_ticks = 10100;
    begin(0, begin_a);
    assert(esb_ota_relay_is_target(0) && !esb_ota_relay_abort_pending(15));
    assert(radio_begins == 2 && radio_ends == 1 && !radio_owned);
    expect_ack(ack(0, 0), begin_ack_a, 64);

    for (unsigned gate = 0; gate < 4; gate++) {
        reset_world();
        esb_radio_fault = gate == 0;
        receiver_ota_active = gate == 1;
        collect_active = gate == 2;
        collect_batch_active = gate == 3;
        begin(0, begin_a);
        assert(!esb_ota_relay_is_active() && !ack(0, 0).has);
        assert(radio_begins == 1 && radio_ends == 0 && !radio_owned);
        assert(loss_invalidations == 0 && abort_clears == 0);
        esb_radio_fault = receiver_ota_active = collect_active = collect_batch_active = false;
        begin(0, begin_a);
        assert(esb_ota_relay_is_target(0));
        assert(radio_begins == 2 && radio_ends == 1 && !radio_owned);
        expect_ack(ack(0, 0), begin_ack_a, 64);
    }

    reset_world();
    radio_begin_hook = fill_capacity;
    begin(4, begin_b);
    assert(esb_ota_relay_is_active());
    for (uint8_t id = 0; id < 4; id++) assert(esb_ota_relay_is_target(id));
    assert(!esb_ota_relay_is_target(4));
    assert(ota_relay.image_size == 64 && ota_relay.total_packets == 1);
    assert(radio_begins == 2 && radio_ends == 1 && !radio_owned);
    begin(0, begin_b);
    assert(ota_relay.num_targets == 4 && esb_ota_relay_is_target(0));
    assert(radio_begins == 2 && radio_ends == 1);
    expect_ack(ack(0, 0), begin_ack_b, 64);

    reset_world();
    radio_begin_hook = begin_one;
    begin(0, begin_b);
    assert(!esb_ota_relay_is_target(0) && esb_ota_relay_is_target(1));
    assert(ota_relay.image_size == 64 && ota_relay.total_packets == 1);
    assert(radio_begins == 2 && radio_ends == 1 && !radio_owned);
    assert(!ack(0, 0).has);
    expect_ack(ack(1, 0), begin_ack_a_one, 64);
    begin(0, begin_b);
    assert(esb_ota_relay_is_target(0) && esb_ota_relay_is_target(1));
    expect_ack(ack(0, 0), begin_ack_b, 64);

    reset_world();
    setup_ready(0);
    next_spin_unlock_irq = cancel_zero;
    begin(1, begin_a);
    assert(!next_spin_unlock_irq && deferred_irqs == 1 && delivered_irqs == 1);
    assert(!esb_ota_relay_is_active() && !esb_ota_relay_is_target(1));
    assert(esb_ota_relay_abort_pending(0) && abort_clears == 1);
    assert(radio_begins == 2 && radio_ends == 1 && !radio_owned);
    assert(radio_claims == 2 && radio_releases == 2);
    assert(loss_invalidations == 1 && !ack(1, 0).has);
    expect_ack(ack(0, 0), abort_at_100, sizeof(abort_at_100));
    now_ticks = 10100;
    begin(1, begin_a);
    assert(esb_ota_relay_is_target(1) && !esb_ota_relay_abort_pending(0));
    assert(radio_begins == 3 && radio_ends == 2 && !radio_owned);
    assert(radio_claims == 3 && radio_releases == 3);
    expect_ack(ack(1, 0), begin_ack_a_one, 64);

    for (unsigned at = 1; at <= 2; at++) {
        reset_world();
        setup_ready(0);
        spin_entries = 0;
        spin_entry_at = at;
        spin_entry_irq = cancel_zero;
        begin(1, begin_a);
        assert(!spin_entry_irq && esb_ota_relay_abort_pending(0));
        assert(!esb_ota_relay_is_target(0) && !radio_owned);
        expect_ack(ack(0, 0), abort_at_100, sizeof(abort_at_100));
        if (at == 1) {
            assert(!esb_ota_relay_is_active() && !esb_ota_relay_is_target(1));
            assert(radio_begins == 2 && radio_ends == 1 && abort_clears == 1);
            assert(!ack(1, 0).has);
        } else {
            assert(esb_ota_relay_is_target(1));
            assert(radio_begins == 1 && radio_ends == 1 && abort_clears == 2);
            expect_ack(ack(1, 0), begin_ack_a_one, 64);
        }
    }
}

static void test_cancellation(void)
{
    reset_world();
    setup_ready(0);
    simple(0xf7, 0);
    assert(!esb_ota_relay_is_active() && esb_ota_relay_abort_pending(0));
    expect_ack(ack(0, 0), abort_at_100, sizeof(abort_at_100));
    now_ticks = 10099;
    assert(esb_ota_relay_abort_pending(0) && ack(0, 0).has);
    now_ticks = 10100;
    assert(!esb_ota_relay_abort_pending(0) && !ack(0, 0).has);

    reset_world();
    now_ticks = 0xfffffff0u;
    simple(0xf7, 2);
    now_ticks = 9983;
    assert(esb_ota_relay_abort_pending(2));
    now_ticks = 9984;
    assert(!esb_ota_relay_abort_pending(2));

    reset_world();
    setup_ready(0);
    setup_ready(1);
    simple(0xf7, 0xff);
    assert(!esb_ota_relay_is_active());
    assert(esb_ota_relay_abort_pending(0) && esb_ota_relay_abort_pending(1));
    assert(ack(0, 0).has && ack(1, 0).has);
    esb_ota_forget_tracker(0);
    assert(!esb_ota_relay_abort_pending(0) && esb_ota_relay_abort_pending(1));
    begin(1, begin_a);
    assert(!esb_ota_relay_is_active() && !esb_ota_relay_is_target(1));
    assert(esb_ota_relay_abort_pending(1) && abort_clears == 2 && !radio_owned);
    assert(!ack(0, 0).has && ack(1, 0).has);
    now_ticks = 10100;
    begin(1, begin_a);
    assert(esb_ota_relay_is_target(1) && !esb_ota_relay_abort_pending(1));
    assert(abort_clears == 3);
    expect_ack(ack(1, 0), begin_ack_a_one, 64);

    reset_world();
    setup_ready(0);
    setup_ready(1);
    simple(0xf7, 0);
    assert(esb_ota_relay_is_target(1) && !esb_ota_relay_is_target(0));
    assert(esb_ota_relay_abort_pending(0));
    begin(0, begin_a);
    assert(esb_ota_relay_is_target(0) && esb_ota_relay_is_target(1));
    assert(!esb_ota_relay_abort_pending(0) && abort_clears == 3);
    assert(radio_begins == 1 && radio_ends == 1 && !radio_owned);
    expect_ack(ack(0, 0), begin_ack_a, 64);

    for (size_t byte = 1; byte <= sizeof(ota_relay.tracker[0]); byte++) {
        reset_world();
        setup_ready(0);
        simple(0xf7, 0);
        watched_copy = &ota_relay.tracker[0];
        watched_byte = byte;
        copy_irq = take_ack;
        esb_ota_forget_tracker(0);
        assert(copy_fired && deferred_irqs == 1 && delivered_irqs == 1);
        assert(!preempt_ack.has && !esb_ota_relay_abort_pending(0));
        begin(0, begin_a);
        expect_ack(ack(0, 0), begin_ack_a, 64);
    }

    reset_world();
    setup_ready(0);
    setup_ready(1);
    watched_source = ota_relay.target_ids;
    watched_byte = 1;
    copy_irq = begin_two;
    simple(0xf7, 0xff);
    assert(copy_fired && deferred_irqs == 1 && delivered_irqs == 1);
    assert(!esb_ota_relay_is_active() && !esb_ota_relay_is_target(2));
    assert(esb_ota_relay_abort_pending(0) && esb_ota_relay_abort_pending(1));
    assert(radio_begins == 2 && radio_ends == 1 && !radio_owned && abort_clears == 2);
    expect_ack(ack(0, 0), abort_at_100, sizeof(abort_at_100));
    assert(!ack(2, 0).has);
}

static void test_forwarding_and_external_calls(void)
{
    reset_world();
    setup_ready(0);
    setup_ready(1);
    data(0, 0xa1, 0xb2, 0xc3);
    data(1, 0xd4, 0xe5, 0xf6);
    status(0, 0x06, 0x0102, 0x44332211);
    const uint8_t terminal[16] = {0xf4, 0, 0x06, 0x01, 0x02, 0x11, 0x22,
                                  0x33, 0x44, 2, 2, 0, 0, 0, 0, 0};
    assert(report_count == 3 && memcmp(reports[2], terminal, 16) == 0);
    assert(!esb_ota_relay_is_target(0) && esb_ota_relay_is_target(1));
    status(1, 0x06, 0, 0);
    const uint8_t last_terminal[16] = {0xf4, 1, 0x06, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0};
    assert(report_count == 4 && memcmp(reports[3], last_terminal, 16) == 0);
    assert(!esb_ota_relay_is_active());

    reset_world();
    setup_ready(0);
    assert(report_count == 1);
    now_ticks = 129;
    status(0, 1, 0, 0);
    assert(report_count == 1);
    now_ticks = 130;
    status(0, 1, 0, 0);
    assert(report_count == 2);
    const uint8_t ready[16] = {0xf4, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0};
    assert(memcmp(reports[1], ready, 16) == 0);

    uint8_t fw[66] = {0x22, 0};
    for (unsigned i = 2; i < 66; i++) fw[i] = (uint8_t)i;
    esb_ota_relay_process_tracker_packet(fw, sizeof(fw));
    assert(work_submits == 1 && submitted_work);
    submitted_work->handler(submitted_work);
    assert(sleeps == 6 && report_count == 8);
    const uint8_t fw_first[16] = {0xf1, 0, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    const uint8_t fw_last[16] = {0xf1, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    assert(memcmp(reports[2], fw_first, 16) == 0);
    assert(memcmp(reports[7], fw_last, 16) == 0);
    esb_ota_relay_console_cmd(0, "status");
    esb_ota_relay_console_cmd(0, "info");
    assert(print_calls == 2 && commands[command_count - 1].id == 0 &&
           commands[command_count - 1].flag == 0x30);
    assert(log_calls > 0);

    unsigned int key = irq_lock();
    assert(esb_ota_relay_is_active() && esb_ota_relay_is_target(0));
    assert(!esb_ota_relay_abort_pending(0) && irq_mask);
    irq_unlock(key);
}

#if EXPECT_UNSAFE
static void test_expected_unsafe(void)
{
    static const uint8_t mixed[64] = {
        [0] = 0x24, [2] = 0x40, [6] = 0x44, [7] = 0x33,
        [8] = 0x22, [9] = 0x11, [11] = 1, [12] = 1,
        [13] = 'A', [61] = 0x34, [62] = 0x12, [63] = 0x86,
    };
    reset_world();
    begin(0, begin_a);
    expect_ack(ack(0, 0), begin_ack_a, 64);
    watched_copy = ota_relay.tracker[0].pending_cmd_data;
    watched_byte = 1;
    copy_irq = take_ack;
    simple(0xf5, 0);
    assert(copy_fired && deferred_irqs == 0 && delivered_irqs == 1);
    expect_ack(preempt_ack, mixed, sizeof(mixed));
    expect_ack(ack_with_rx(0, NULL, 0), verify_ack, 4);
}
#endif

int main(void)
{
#if EXPECT_UNSAFE
    test_expected_unsafe();
    puts("OTA relay expected-unsafe mixed ACK observed");
#else
    test_publication_copy();
    test_publication_stores();
    test_confirmation_preemption();
    test_confirmations();
    test_timeout_and_deferral();
    test_ring();
    test_reservation();
    test_cancellation();
    test_forwarding_and_external_calls();
    assert(!irq_mask && !spin_depth && !pending_irq && !radio_owned);
    puts("OTA relay production replay passed");
#endif
    return 0;
}
