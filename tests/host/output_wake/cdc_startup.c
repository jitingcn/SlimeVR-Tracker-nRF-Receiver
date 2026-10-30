#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MAX_TRACKERS 16
#define BIT(n) (1U << (n))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ARG_UNUSED(x) (void)(x)
#define BUILD_ASSERT _Static_assert
#define LOG_MODULE_REGISTER(...)
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define printk(...) ((void)0)
#define K_NO_WAIT 0
#define K_FOREVER (-1)
#define K_MSEC(x) (x)
#define K_SECONDS(x) ((x) * 1000)
#define __noinit
#define __aligned(n) __attribute__((aligned(n)))
#define STRUCT_SECTION_ITERABLE(type, name) struct type name
#define CONFIG_CONSOLE_INPUT_MAX_LINE_LEN 128
#define CONFIG_SLIMEVR_USB_DEVICE_MANUFACTURER "test"
#define CONFIG_SLIMEVR_USB_DEVICE_PRODUCT "receiver"
#define FW_STRING "test"
#define FW_GIT_REPO_URL "test"
#define FW_GIT_BRANCH "test"
#define CONSOLE_THREAD_PRIORITY 8
#define K_THREAD_STACK_DEFINE(name, size) unsigned char name[size]
#define K_THREAD_STACK_SIZEOF(name) sizeof(name)

struct device { int kind; };
static const struct device cdc_acm_uart1 = {1};
static const struct device zephyr_console = {2};
static const struct device hid_device = {3};
#define DT_NODELABEL(node) node
#define DT_CHOSEN(node) node
#define DEVICE_DT_GET(node) (&node)
#define UART_LINE_CTRL_DTR 1
#define HID_REPORT_TYPE_OUTPUT 2

struct k_work { void (*handler)(struct k_work *); bool queued; };
#define k_work_delayable k_work
#define K_WORK_DEFINE(name, fn) struct k_work name = {.handler = fn}
#define K_WORK_DELAYABLE_DEFINE(name, fn) K_WORK_DEFINE(name, fn)
struct k_timer { void (*handler)(struct k_timer *); bool active; };
#define K_TIMER_DEFINE(name, fn, stop) struct k_timer name = {.handler = fn}
static struct k_work *scheduled[16];
static size_t scheduled_count;
static int k_work_schedule(struct k_work *work, int delay)
{
    bool known = false;
    for (size_t i = 0; i < scheduled_count; i++) known |= scheduled[i] == work;
    if (!known) {
        assert(scheduled_count < sizeof(scheduled) / sizeof(scheduled[0]));
        scheduled[scheduled_count++] = work;
    }
    work->queued = true;
    return 1;
}
static int k_work_submit(struct k_work *work) { return k_work_schedule(work, 0); }
static void k_timer_start(struct k_timer *timer, int delay, int period) { timer->active = true; }
static int64_t k_uptime_get(void) { return 1234; }
static int64_t k_uptime_ticks(void) { return 0x01020304; }
static unsigned int irq_lock(void) { return 0; }
static void irq_unlock(unsigned int key) {}

typedef int atomic_t;
static int atomic_get(const atomic_t *value) { return *value; }
static int atomic_set(atomic_t *value, int next) { int old = *value; *value = next; return old; }
static int atomic_inc(atomic_t *value) { return (*value)++; }
struct k_spinlock { bool held; };
typedef int k_spinlock_key_t;
static k_spinlock_key_t k_spin_lock(struct k_spinlock *lock)
{
    assert(!lock->held);
    lock->held = true;
    return 0;
}
static void k_spin_unlock(struct k_spinlock *lock, k_spinlock_key_t key)
{
    assert(lock->held);
    lock->held = false;
}
struct k_thread { int unused; };
typedef void (*k_thread_entry_t)(void *, void *, void *);
static unsigned console_threads;
static void k_thread_create(struct k_thread *thread, void *stack, size_t size,
                            k_thread_entry_t entry, void *a, void *b, void *c,
                            int priority, int options, int delay)
{
    console_threads++;
}
struct k_msgq { unsigned char *data; size_t size, capacity, head, count; };
#define Z_MSGQ_INITIALIZER(name, buf, size, depth) {(unsigned char *)(buf), size, depth, 0, 0}
static int k_msgq_put(struct k_msgq *queue, const void *message, int timeout)
{
    assert(timeout == K_NO_WAIT);
    if (queue->count == queue->capacity) return -ENOMSG;
    size_t slot = (queue->head + queue->count++) % queue->capacity;
    memcpy(queue->data + slot * queue->size, message, queue->size);
    return 0;
}
static int k_msgq_get(struct k_msgq *queue, void *message, int timeout)
{
    assert(timeout == K_NO_WAIT);
    if (!queue->count) return -ENOMSG;
    memcpy(message, queue->data + queue->head * queue->size, queue->size);
    queue->head = (queue->head + 1) % queue->capacity;
    queue->count--;
    return 0;
}

static bool raw_device_ready = true, raw_dtr, raw_tx_enabled, console_tx_enabled;
static int raw_callback_error;
static void (*raw_callback)(const struct device *, void *);
static void (*console_callback)(const struct device *, void *);
static uint8_t output[1024];
static size_t output_len;
static bool device_is_ready(const struct device *dev)
{
    return dev != &cdc_acm_uart1 || raw_device_ready;
}
static int uart_irq_callback_user_data_set(const struct device *dev,
                                           void (*callback)(const struct device *, void *), void *data)
{
    if (dev == &cdc_acm_uart1) {
        if (raw_callback_error) return raw_callback_error;
        raw_callback = callback;
    } else {
        assert(dev == &zephyr_console);
        console_callback = callback;
    }
    return 0;
}
static int uart_line_ctrl_get(const struct device *dev, int command, uint32_t *value)
{
    assert(dev == &cdc_acm_uart1 && command == UART_LINE_CTRL_DTR);
    *value = raw_dtr;
    return 0;
}
static void uart_irq_rx_disable(const struct device *dev) {}
static void uart_irq_rx_enable(const struct device *dev) {}
static void uart_irq_tx_disable(const struct device *dev)
{
    if (dev == &cdc_acm_uart1) raw_tx_enabled = false;
    else { assert(dev == &zephyr_console); console_tx_enabled = false; }
}
static void uart_irq_tx_enable(const struct device *dev)
{
    if (dev == &cdc_acm_uart1) raw_tx_enabled = true;
    else { assert(dev == &zephyr_console); console_tx_enabled = true; }
}
static int uart_irq_update(const struct device *dev) { return 1; }
static int uart_irq_rx_ready(const struct device *dev) { return 0; }
static int uart_irq_tx_ready(const struct device *dev)
{
    return dev == &cdc_acm_uart1 ? raw_tx_enabled : console_tx_enabled;
}
static int uart_irq_is_pending(const struct device *dev) { return uart_irq_tx_ready(dev); }
static int uart_poll_in(const struct device *dev, unsigned char *byte) { return -1; }
static int uart_fifo_read(const struct device *dev, uint8_t *data, int size) { return 0; }
static int uart_fifo_fill(const struct device *dev, const uint8_t *data, int size)
{
    if (dev == &cdc_acm_uart1) {
        assert(size >= 0 && output_len + (size_t)size <= sizeof(output));
        memcpy(output + output_len, data, (size_t)size);
        output_len += (size_t)size;
    }
    return size;
}
static void sys_put_be32(uint32_t value, uint8_t *p)
{
    p[0] = value >> 24; p[1] = value >> 16; p[2] = value >> 8; p[3] = value;
}
static uint16_t sys_get_be16(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }
static uint16_t sys_get_le16(const uint8_t *p) { return p[0] | ((uint16_t)p[1] << 8); }
static uint32_t sys_get_le32(const uint8_t *p)
{
    return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint8_t crc8_ccitt(uint8_t crc, const uint8_t *data, size_t len)
{
    while (len--) {
        crc ^= *data++;
        for (unsigned bit = 0; bit < 8; bit++)
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80) ? 0x07 : 0));
    }
    return crc;
}

#include "protocol.inc"
#if defined(CONFIG_DATA_COLLECT) && !defined(CONFIG_DATA_COLLECT_HID)
#include "data_collect.h"
static uint8_t stored_trackers = 4;
static uint64_t stored_tracker_addr[MAX_TRACKERS] = {[2] = 0x22, [3] = 0x33};
static uint8_t tracker_remote_command[MAX_TRACKERS];
static uint16_t last_raw_seq[MAX_TRACKERS];
static bool last_raw_valid[MAX_TRACKERS];
struct command_record { uint8_t tracker, flag, arg; };
static struct command_record radio_commands[32];
static size_t radio_count;
static void esb_send_remote_command_arg(uint8_t tracker, uint8_t flag, uint8_t arg)
{
    assert(tracker < MAX_TRACKERS && radio_count < 32);
    radio_commands[radio_count++] = (struct command_record){tracker, flag, arg};
    tracker_remote_command[tracker] = flag;
}
static void esb_send_remote_command(uint8_t tracker, uint8_t flag)
{
    esb_send_remote_command_arg(tracker, flag, 0);
}
static int esb_radio_collection_begin(void) { return 0; }
static void esb_radio_operation_end(void) {}
static int unexpected(void) { assert(!"unrelated command path"); return -1; }
#define tracker_events_control(...) unexpected()
#define remote_wom_begin(...) unexpected()
#define rcv_cmd_remote_sens_set_hid(...) unexpected()
#define rcv_cmd_remote_sens_auto_hid(...) unexpected()
#define rcv_cmd_remote_test_on_hid(...) unexpected()
#define rcv_cmd_remote_test_off_hid(...) unexpected()
#define rcv_cmd_remote_flag_hid(...) unexpected()
#define rcv_cmd_pair(...) unexpected()
#define rcv_cmd_exit_pair(...) unexpected()
#define rcv_cmd_clear(...) unexpected()
#define rcv_cmd_add(...) unexpected()
#define rcv_cmd_remove(...) unexpected()
#define rcv_cmd_list(...) unexpected()
#define rcv_cmd_channel_set(...) unexpected()
#define rcv_cmd_channel_clear(...) unexpected()
#define rcv_cmd_rssi_scan(...) unexpected()
#define rcv_cmd_info(...) unexpected()
#define rcv_cmd_uptime(...) unexpected()
#define rcv_cmd_stats(...) unexpected()
#define rcv_cmd_resetstats(...) unexpected()
#define rcv_cmd_collect_meta(...) unexpected()
#define rcv_cmd_reboot(...) unexpected()
#define rcv_cmd_dfu(...) unexpected()
#define rcv_cmd_tracker_channel_all(...) unexpected()
#define rcv_cmd_tracker_channel_clear_all(...) unexpected()
#define arm_ch_hid_completion(...) unexpected()
static bool esb_get_stats_detailed_enabled(void) { return false; }
static int channel_completion_lock;
static const struct device *hdev = &hid_device;
static uint8_t last_ack[16];
static unsigned ack_count;
static void hid_write_packet_n(const uint8_t *packet, int rssi)
{
    memcpy(last_ack, packet, sizeof(last_ack));
    ack_count++;
}
static uint32_t tracker_events_usb_generation(void) { return 7; }
#define RECEIVER_OTA_ID 0xff
#define receiver_ota_process_hid(...) unexpected()
#define esb_ota_relay_process_hid(...) unexpected()
struct raw_payload { uint8_t data[64]; uint8_t length, rssi; };
#else
static unsigned forbidden_initializations;
static int data_collect_init(void) { forbidden_initializations++; return -EIO; }
static void console_serial_close(void) {}
static void console_serial_stop(void) {}
#endif

static void k_mutex_lock(int *mutex, int timeout) { assert(!*mutex); *mutex = 1; }
static void k_mutex_unlock(int *mutex) { assert(*mutex); *mutex = 0; }
static int usb_serial_transition_lock, usb_ctrl_start_sem;
static int64_t usb_start_time_ms;
static void k_sem_give(int *sem) { (*sem)++; }
static bool usb_enabled;
static atomic_t usb_configured, usb_config_epoch;
static void (*receiver_usb_state_callback)(bool);
struct usbd_context { int unused; };
struct usbd_msg { int type, status; };
typedef void (*usbd_msg_cb_t)(struct usbd_context *, const struct usbd_msg *);
enum { USBD_MSG_RESET, USBD_MSG_CONFIGURATION, USBD_MSG_VBUS_READY, USBD_MSG_VBUS_REMOVED };
#define USBD_SPEED_FS 0
#define USB_BCC_MISCELLANEOUS 0xef
#define CONFIG_USBD_CDC_ACM_CLASS 1
#define IS_ENABLED(x) (x)
#define IF_ENABLED(...)
#define SYS_STATUS_SERIAL_ACTIVE 1
static struct usbd_context slimevr_usbd;
static int slimevr_lang, slimevr_mfr, slimevr_product, slimevr_fs_config;
static bool setup_entered, setup_had_raw_callback, enable_had_raw_callback;
static bool device_enabled, detect_vbus, early_vbus;
static usbd_msg_cb_t registered_callback;
static int usbd_add_descriptor(struct usbd_context *ctx, int *descriptor)
{
    if (!setup_entered) setup_had_raw_callback = raw_callback != NULL;
    setup_entered = true;
    return 0;
}
static int usbd_add_configuration(struct usbd_context *ctx, int speed, int *config) { return 0; }
static int usbd_register_all_classes(struct usbd_context *ctx, int speed, int config, void *list) { return 0; }
static int usbd_device_set_code_triple(struct usbd_context *ctx, int speed, int a, int b, int c) { return 0; }
static int usbd_msg_register_cb(struct usbd_context *ctx, usbd_msg_cb_t callback)
{
    registered_callback = callback;
    return 0;
}
static int usbd_init(struct usbd_context *ctx)
{
    if (early_vbus) {
        assert(registered_callback);
        const struct usbd_msg msg = {.type = USBD_MSG_VBUS_READY};
        registered_callback(ctx, &msg);
    }
    return 0;
}
static bool usbd_can_detect_vbus(struct usbd_context *ctx) { return detect_vbus; }
static int usbd_enable(struct usbd_context *ctx)
{
    enable_had_raw_callback = raw_callback != NULL;
    device_enabled = true;
    return 0;
}
static int usbd_disable(struct usbd_context *ctx) { device_enabled = false; return 0; }
static bool get_status(int status) { return false; }
static void set_status(int status, bool value) {}
struct log_backend { int unused; };
static struct log_backend backend;
static const struct log_backend *log_backend_get_by_name(const char *name) { return &backend; }
static void log_backend_disable(const struct log_backend *value) {}

#include "production.inc"

#if defined(CONFIG_DATA_COLLECT) && !defined(CONFIG_DATA_COLLECT_HID)
static void console_thread(void) {}
static void run_work(void)
{
    for (unsigned step = 0; step < 64; step++) {
        struct k_work *work = NULL;
        for (size_t i = 0; i < scheduled_count; i++) {
            if (scheduled[i]->queued) { work = scheduled[i]; break; }
        }
        if (work) {
            work->queued = false;
            work->handler(work);
        }
        if (raw_tx_enabled && raw_callback) raw_callback(&cdc_acm_uart1, NULL);
        if (!work && !raw_tx_enabled) return;
    }
    assert(!"work did not quiesce");
}
static void configure_usb(void)
{
    assert(device_enabled && registered_callback);
    const struct usbd_msg configured = {.type = USBD_MSG_CONFIGURATION, .status = 1};
    registered_callback(&slimevr_usbd, &configured);
    assert(usb_configured == 1);
}
static void start_single(void)
{
    const uint8_t command[16] = {254, 0x41, 214, 0, 2};
    const uint8_t ack[16] = {251, 0x41, 214, 0};
    assert(set_report_cb(hdev, HID_REPORT_TYPE_OUTPUT, 0, sizeof(command), command) == 0);
    run_work();
    assert(ack_count == 1 && memcmp(last_ack, ack, sizeof(ack)) == 0);
    assert(data_collect_is_active() && data_collect_is_target(2));
    assert(!data_collect_batch_is_active());
    assert(radio_count == 1 && radio_commands[0].tracker == 2 && radio_commands[0].flag == 0x22);
}
static void start_batch(void)
{
    const uint8_t command[16] = {254, 0x42, 221, 0, 200, 0};
    const uint8_t ack[16] = {251, 0x42, 221, 0};
    assert(set_report_cb(hdev, HID_REPORT_TYPE_OUTPUT, 0, sizeof(command), command) == 0);
    run_work();
    assert(ack_count == 1 && memcmp(last_ack, ack, sizeof(ack)) == 0);
    assert(!data_collect_is_active() && data_collect_batch_is_active());
    assert(data_collect_batch_is_target(2) && data_collect_batch_is_target(3));
    assert(!data_collect_batch_is_target(1));
    assert(radio_count == 2 && radio_commands[0].tracker == 2 && radio_commands[1].tracker == 3);
    assert(radio_commands[0].flag == 0x34 && radio_commands[1].flag == 0x34);
    assert(radio_commands[0].arg == 200 && radio_commands[1].arg == 200);
}
static void expect_output(const uint8_t *bytes, size_t size)
{
    assert(output_len == size && memcmp(output, bytes, size) == 0);
}
static void sample(uint8_t tracker, uint16_t seq)
{
    receive_raw(tracker, seq);
    run_work();
}
#endif

int main(int argc, char **argv)
{
    assert(argc == 2);
#if defined(CONFIG_DATA_COLLECT) && !defined(CONFIG_DATA_COLLECT_HID)
    const uint8_t frame2_11[] = {0xaa,0x55,0x0b,0x10,0x02,0x00,0x0b,0x55,0x66,0xd7,0x01,0x02,0x03,0x04,0x3f};
    const uint8_t frame2_12[] = {0xaa,0x55,0x0b,0x10,0x02,0x00,0x0c,0x55,0x66,0xd7,0x01,0x02,0x03,0x04,0x46};
    const uint8_t two_single[] = {
        0xaa,0x55,0x0b,0x10,0x02,0x00,0x0b,0x55,0x66,0xd7,0x01,0x02,0x03,0x04,0x3f,
        0xaa,0x55,0x0b,0x10,0x02,0x00,0x0c,0x55,0x66,0xd7,0x01,0x02,0x03,0x04,0x46};
    const uint8_t two_batch[] = {
        0xaa,0x55,0x0b,0x10,0x02,0x00,0x0b,0x55,0x66,0xd7,0x01,0x02,0x03,0x04,0x3f,
        0xaa,0x55,0x0b,0x10,0x03,0x00,0x0b,0x55,0x66,0xd7,0x01,0x02,0x03,0x04,0x57};
    raw_dtr = true;
    if (!strcmp(argv[1], "device-not-ready")) raw_device_ready = false;
    if (!strcmp(argv[1], "callback-failure")) raw_callback_error = -EIO;
    if (!strcmp(argv[1], "early-vbus")) detect_vbus = early_vbus = true;
    usb_init_thread();
    if (!strcmp(argv[1], "device-not-ready") || !strcmp(argv[1], "callback-failure")) {
        assert(!setup_entered && !registered_callback && !device_enabled && !usb_enabled);
        assert(usb_ctrl_start_sem == 0 && !dc_timer.active && !cdc_ready);
    } else {
        assert(setup_entered && device_enabled && usb_enabled && usb_ctrl_start_sem == 1);
        configure_usb();
        if (!strcmp(argv[1], "batch-console")) {
            start_batch();
            void (*saved_callback)(const struct device *, void *) = raw_callback;
            console_serial_start();
            assert(console_input.active && console_callback && data_collect_batch_is_active());
            assert(data_collect_batch_is_target(2) && data_collect_batch_is_target(3));
            sample(1, 11);
            assert(output_len == 0 && radio_count == 2);
            sample(2, 11);
            expect_output(frame2_11, sizeof(frame2_11));
            console_serial_close();
            console_serial_start();
            assert(console_input.active && console_threads == 1 && raw_callback == saved_callback);
            assert(data_collect_batch_is_target(2) && data_collect_batch_is_target(3));
            sample(3, 11);
            expect_output(two_batch, sizeof(two_batch));
            assert(radio_count == 2);
        } else {
            start_single();
            if (!strcmp(argv[1], "single-console")) {
                void (*saved_callback)(const struct device *, void *) = raw_callback;
                console_serial_start();
                assert(console_input.active && console_callback && data_collect_is_target(2));
                sample(2, 11);
                expect_output(frame2_11, sizeof(frame2_11));
                console_serial_close();
                console_serial_start();
                assert(console_input.active && console_threads == 1 && raw_callback == saved_callback);
                assert(data_collect_is_target(2));
                sample(2, 12);
                expect_output(two_single, sizeof(two_single));
                assert(radio_count == 1);
            } else if (!strcmp(argv[1], "raw-reopen")) {
                raw_dtr = false;
                sample(2, 11);
                assert(output_len == 0 && data_collect_is_target(2));
                raw_dtr = true;
                sample(2, 12);
                expect_output(frame2_12, sizeof(frame2_12));
                assert(radio_count == 1);
            } else {
                assert(!strcmp(argv[1], "raw-only") || !strcmp(argv[1], "early-vbus"));
                assert(!console_input.active && !console_callback);
                sample(2, 11);
                expect_output(frame2_11, sizeof(frame2_11));
                assert(radio_count == 1 && data_collect_is_target(2));
            }
            assert(setup_had_raw_callback && enable_had_raw_callback);
        }
    }
#else
    assert(!strcmp(argv[1], "disabled") || !strcmp(argv[1], "dedicated-hid"));
    usb_init_thread();
    assert(setup_entered && registered_callback && device_enabled && usb_enabled);
    assert(usb_ctrl_start_sem == 1 && forbidden_initializations == 0);
#endif
    puts(argv[1]);
    return 0;
}
