#include <assert.h>
#include <errno.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ARG_UNUSED(x) (void)(x)
#define MAX_TRACKERS 16
#define CONFIG_RADIO_RF_CHANNEL 18
#define RADIO_RF_CHANNEL CONFIG_RADIO_RF_CHANNEL
#define ESB_RF_CHANNEL_DEFAULT UINT8_MAX
#define PAIRING_TIMEOUT_SECONDS 30
#define PAIRING_EXIT_DELAY_MS 2000
#define RF_CHANNEL 1
#define SYS_LED_PATTERN_SHORT 1
#define SYS_LED_PRIORITY_CONNECTION 1
#define SYS_STATUS_CONNECTION_ERROR 1
#define RADIO_STATE_STATE_Disabled 0
#define LOG_INF(...) ((void)0)

typedef int atomic_t;
static int atomic_get(const atomic_t *p) { return *p; }
static void atomic_set(atomic_t *p, int v) { *p = v; }
static bool atomic_cas(atomic_t *p, int old, int v)
{
    if (*p != old) return false;
    *p = v;
    return true;
}

static bool esb_ota_relay_is_active(void) { return false; }
static bool esb_ota_relay_abort_pending(uint8_t i) { return i >= MAX_TRACKERS; }
static bool receiver_ota_is_active(void) { return false; }
static bool data_collect_is_active(void) { return false; }
static bool data_collect_batch_is_active(void) { return false; }
static void esb_auto_loss_fence(void) {}
static void esb_auto_loss_invalidate(void) {}
static void esb_disable(void);
static void log_error(const char *format, ...);
#define LOG_ERR(...) log_error(__VA_ARGS__)
static struct { unsigned STATE, SHORTS; } radio;
#define NRF_RADIO (&radio)
/* ADMISSION */

struct k_sem { unsigned count; };
#define K_SEM_DEFINE(name, count, limit) struct k_sem name = {count}
#define K_MSEC(ms) (ms)
#define CLOCK_CONTROL_NRF_SUBSYS_HF 0
#define ONOFF_STATE_OFF 0
#define ONOFF_STATE_ON 2
#define ONOFF_STATE_TO_ON 6
#define ONOFF_STATE_ERROR 1

struct onoff_manager;
struct onoff_client;
typedef void (*notify_fn)(struct onoff_manager *, struct onoff_client *, uint32_t, int);
struct sys_notify { notify_fn callback; };
struct onoff_client { struct sys_notify notify; };
static struct onoff_manager {
    struct onoff_client *pending, *detached;
    int result;
    unsigned refs;
} clock;

enum clock_case { INLINE, ASYNC, NO_MANAGER, REQUEST_ERROR, COMPLETION_ERROR,
                  NEVER_COMPLETES, LATE_SUCCESS, LATE_FAILURE };
static enum clock_case clock_case;
static unsigned requests, cancels, release_calls, notifications, wait_ms;
static void expect_starting(void);

static void sys_notify_init_callback(struct sys_notify *n, notify_fn cb) { n->callback = cb; }
static struct onoff_manager *z_nrf_clock_control_get_onoff(int source)
{
    assert(source == CLOCK_CONTROL_NRF_SUBSYS_HF);
    return clock_case == NO_MANAGER ? NULL : &clock;
}

/* Zephyr attributes successful client refs and detaches notifications under
 * the manager lock. A cancelled pending request is absent from this list. */
static void detach_completion(int result)
{
    assert(!clock.detached);
    clock.detached = clock.pending;
    clock.pending = NULL;
    clock.result = result;
    if (clock.detached && result >= 0) clock.refs++;
}

static void notify_completion(void)
{
    if (!clock.detached) return;
    struct onoff_client *client = clock.detached;
    clock.detached = NULL;
    notifications++;
    client->notify.callback(&clock, client,
                            clock.result < 0 ? ONOFF_STATE_ERROR : ONOFF_STATE_ON, clock.result);
}

static int onoff_request(struct onoff_manager *mgr, struct onoff_client *client)
{
    assert(mgr == &clock && !clock.pending && !clock.detached);
    requests++;
    if (clock_case == REQUEST_ERROR) return -EAGAIN;
    clock.pending = client;
    if (clock_case == INLINE) {
        assert(clock.refs == 1);  /* A separate pre-existing HF user. */
        detach_completion(0);
        notify_completion();
        return ONOFF_STATE_ON;
    }
    return ONOFF_STATE_OFF;
}

static int onoff_cancel_or_release(struct onoff_manager *mgr, struct onoff_client *client)
{
    assert(mgr == &clock);
    cancels++;
    if (clock.pending) {
        assert(clock.pending == client);
        clock.pending = NULL;
        return ONOFF_STATE_TO_ON;
    }
    /* Equivalent to -EALREADY from cancel, even before callback delivery. */
    assert(clock.detached == client);
    release_calls++;
    if (clock.result < 0) return -EIO;
    assert(clock.refs > 0);
    clock.refs--;
    return ONOFF_STATE_ON;
}

static void k_sem_give(struct k_sem *sem) { assert(!sem->count); sem->count++; }
static int k_sem_take(struct k_sem *sem, int timeout)
{
    assert(timeout > 0 && timeout <= 1000);
    expect_starting();
    if (clock_case == ASYNC || clock_case == COMPLETION_ERROR) {
        detach_completion(clock_case == ASYNC ? 0 : -EIO);
        notify_completion();
    }
    if (sem->count) {
        sem->count--;
        return 0;
    }
    wait_ms += timeout;
    if (clock_case == LATE_SUCCESS || clock_case == LATE_FAILURE) {
        detach_completion(clock_case == LATE_SUCCESS ? 0 : -EIO);
    }
    return -EAGAIN;
}

static jmp_buf boot_done;
static unsigned restores, inits, rx_starts, disables, visits, auto_starts, maintenance, status_errors;
static unsigned submits, busy_messages, unavailable_messages;
static int startup_error;
static bool fail_init, fail_rx, fail_disable;
static uint8_t stored_trackers, pairing_target_count, pairing_initial_count;
static uint8_t receiver_rf_channel, receiver_device_addr[6];
static bool tracker_store_ready, esb_pairing;
static int64_t pairing_start_time, pairing_target_reached_time;
static atomic_t pairing_new_devices_blocked, rssi_scan_busy;
static const struct { uint64_t DEVICEADDR[1]; } ficr = {{UINT64_C(0x112233445566)}};
#define NRF_FICR (&ficr)

static void log_error(const char *format, ...)
{
    if (strcmp(format, "Radio startup failed: %d") == 0) {
        va_list args;
        va_start(args, format);
        startup_error = va_arg(args, int);
        va_end(args);
    }
}

static int tracker_store_restore(void) { restores++; tracker_store_ready = true; return 0; }
static void sys_read(int key, uint8_t *value, unsigned size)
{
    assert(key == RF_CHANNEL && size == 1);
    *value = ESB_RF_CHANNEL_DEFAULT;
}
static uint8_t esb_rf_channel_decode(uint8_t channel) { return channel; }
static uint8_t esb_get_receiver_channel(void) { return receiver_rf_channel; }
static uint32_t k_uptime_get_32(void) { return 100; }
static int64_t k_uptime_get(void) { return 100; }
static void set_led(int pattern, int priority) { assert(pattern == priority); }
static void set_status(int status, bool enabled)
{
    assert(status == SYS_STATUS_CONNECTION_ERROR && enabled);
    status_errors++;
}

static void esb_set_addr_unified(void) { assert(clock.refs); }
static int esb_initialize(bool tx)
{
    assert(!tx && !esb_initialized && clock.refs);
    inits++;
    if (fail_init) return -EINVAL;  /* SDK init wrapper already cleaned up safely. */
    esb_initialized = true;
    return 0;
}
static int esb_initialize_quiet(bool tx) { return esb_initialize(tx); }
static int esb_start_rx(void)
{
    assert(clock.refs && esb_initialized);
    if (radio_startup_state == ESB_RADIO_STARTING) expect_starting();
    rx_starts++;
    radio.STATE = 3;
    return fail_rx ? -EIO : 0;
}
static void esb_disable(void)
{
    assert(clock.refs);
    disables++;
    if (!fail_disable) radio.STATE = RADIO_STATE_STATE_Disabled;
}
static void esb_rx_discontinuity(void) { assert(clock.refs && radio_operation_busy); }
static void auto_channel_init(const uint8_t address[6])
{
    assert(address == receiver_device_addr && radio_startup_state == ESB_RADIO_READY);
    auto_starts++;
}
static void esb_start_pairing(void) { assert(!"unexpected roster retry"); }
static void esb_finish_pair(void) { assert(!"unexpected pairing timeout"); }
static void process_pairing_queue(void) { assert(esb_pairing); }
static void tracker_events_process(uint32_t now) { assert(now); maintenance++; }
static void channel_change_poll(void) { assert(radio_startup_state == ESB_RADIO_READY); }
static void k_msleep(unsigned ms)
{
    if (ms == 100) longjmp(boot_done, 1);  /* End first successful boot-loop iteration. */
}
static void k_yield(void) {}
/* BOOT */

static void printk(const char *format, ...)
{
    if (strstr(format, "RSSI scan busy")) busy_messages++;
    if (strstr(format, "RSSI scan unavailable")) unavailable_messages++;
}
static bool rssi_scan_fem_is_safe(void) { return true; }
static bool esb_channel_is_allowed(uint8_t channel) { return channel == RADIO_RF_CHANNEL; }
static int rssi_scan_channel_measure_min_n(uint8_t channel, uint32_t samples, uint8_t *result)
{
    assert(clock.refs && radio_operation_busy && !radio.STATE && samples && esb_channel_is_allowed(channel));
    visits++;
    *result = 90;
    return 0;
}
static struct k_work { int unused; } rssi_scan_work;
static int rssi_wq;
static void k_work_submit_to_queue(int *queue, struct k_work *work)
{
    assert(queue == &rssi_wq && work == &rssi_scan_work);
    submits++;
}
/* SCAN */

static void expect_starting(void)
{
    unsigned scans = visits, disables_before = disables, submissions = submits;
    assert(esb_channel_is_busy());
    assert(esb_radio_operation_begin() == -EBUSY && esb_radio_collection_begin() == -EBUSY);
    assert(esb_radio_self_ota_begin() == -EBUSY);
    assert(rcv_cmd_rssi_scan() == RCV_HID_ST_EBUSY);
    rssi_scan_run_and_print();
    assert(busy_messages && visits == scans && disables == disables_before && submits == submissions);
}

static void reset(enum clock_case scenario)
{
    clock_case = scenario;
    clock = (struct onoff_manager){ .refs = scenario == INLINE };
    clocks_start_client = (struct onoff_client){0};
    clocks_start_sem.count = 0;
    radio_startup_state = ESB_RADIO_STARTING;
    radio_operation_busy = rssi_scan_busy = 0;
    esb_initialized = esb_radio_fault = tracker_store_ready = esb_pairing = false;
    radio.STATE = radio.SHORTS = 0;
    stored_trackers = 0;
    requests = cancels = release_calls = notifications = wait_ms = 0;
    restores = inits = rx_starts = disables = visits = auto_starts = maintenance = status_errors = 0;
    submits = busy_messages = unavailable_messages = startup_error = 0;
    fail_init = fail_rx = fail_disable = false;
}

static void replay_boot(void)
{
    expect_starting();
    if (!setjmp(boot_done)) esb_thread();
    assert(tracker_store_ready && restores == 1);
}

static void expect_failed(int error, bool safely_disabled)
{
    assert(startup_error == error && status_errors == 1 && !esb_pairing && !auto_starts && !maintenance);
    assert(radio_startup_state == ESB_RADIO_STARTUP_FAILED);
    unsigned disables_before = disables;
    for (int i = 0; i < 2; ++i) {
        assert(esb_radio_operation_begin() == -EIO && esb_radio_collection_begin() == -EIO);
        assert(rcv_cmd_rssi_scan() == RCV_HID_ST_EBUSY);
        rssi_scan_run_and_print();
    }
    assert(unavailable_messages == 2 && !visits && !submits && disables == disables_before);
    assert(esb_radio_self_ota_begin() == (safely_disabled ? 0 : -EIO));
    if (safely_disabled) esb_radio_operation_end();
}

int main(void)
{
    for (enum clock_case scenario = INLINE; scenario <= ASYNC; scenario++) {
        reset(scenario);
        replay_boot();
        unsigned expected_refs = scenario == INLINE ? 2 : 1;
        assert(!startup_error && !status_errors && requests == 1 && clock.refs == expected_refs);
        assert(notifications == 1 && !cancels && !wait_ms);
        assert(esb_pairing && auto_starts == 1 && maintenance == 1 && !esb_channel_is_busy());
        for (unsigned i = 0; i < 2; ++i) {
            assert(rcv_cmd_rssi_scan() == RCV_HID_ST_STARTED);
            assert(rcv_cmd_rssi_scan() == RCV_HID_ST_EBUSY);
            rssi_scan_work_handler(&rssi_scan_work);
            assert(!esb_channel_is_busy() && !rssi_scan_busy && radio.STATE == 3);
            assert(requests == 1 && clock.refs == expected_refs && submits == i + 1);
        }
        assert(visits == 2 * RSSI_SCAN_SWEEPS && inits == 3 && rx_starts == 3);
    }
    for (enum clock_case scenario = NO_MANAGER; scenario <= LATE_FAILURE; scenario++) {
        reset(scenario);
        replay_boot();
        int expected = scenario == NO_MANAGER ? -ENXIO : scenario == REQUEST_ERROR ? -EAGAIN :
            scenario == COMPLETION_ERROR ? -EIO : -ETIMEDOUT;
        expect_failed(expected, true);
        assert(requests == (scenario != NO_MANAGER));
        assert(!inits && !rx_starts && !disables && !clock.refs);
        bool timed_out = scenario >= NEVER_COMPLETES;
        assert(cancels == timed_out && (wait_ms > 0) == timed_out);
        assert(release_calls == (scenario >= LATE_SUCCESS));
        if (scenario == NEVER_COMPLETES) detach_completion(0);
        if (timed_out) notify_completion();  /* After boot thread has already returned. */
        assert(notifications == (scenario == COMPLETION_ERROR || scenario >= LATE_SUCCESS));
        assert(!clock.refs && !clock.pending && !clock.detached);
        assert(esb_radio_operation_begin() == -EIO && startup_error == expected);
    }
    reset(ASYNC);
    fail_init = true;
    replay_boot();
    expect_failed(-EINVAL, true);
    assert(inits == 1 && !rx_starts && !disables && clock.refs == 1);
    for (int unsafe = 0; unsafe < 2; ++unsafe) {
        reset(ASYNC);
        fail_rx = true;
        fail_disable = unsafe;
        replay_boot();
        expect_failed(-EIO, !unsafe);
        assert(inits == 1 && rx_starts == 1 && disables == 1 && clock.refs == 1);
        assert(esb_radio_fault == unsafe && (radio.STATE != 0) == unsafe);
    }
    puts("Receiver startup: inline/async success, cancel and late notification, failure gates, repeated scans, local recovery");
    return 0;
}
