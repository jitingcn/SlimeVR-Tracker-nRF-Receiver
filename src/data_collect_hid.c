/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

/*
 * Data collection via dedicated HID endpoint.
 *
 * Each raw ESB packet is forwarded as a single 64-byte HID input report:
 *   [0..N-1]  ESB payload (up to 52 bytes: 48 for type 0x10, 52 for type 0x13)
 *   [N]       RSSI
 *   [N+1..N+4] rx_ticks (BE32)
 *   [N+5..63]  zero padding
 *
 * No framing protocol needed — HID guarantees packet boundaries.
 * The PC reads from the second HID interface of the same USB device.
 */

#include "data_collect.h"
#include "connection/esb.h"
#include "globals.h"
#include <zephyr/irq.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(data_collect, LOG_LEVEL_INF);

/* HID report descriptor for data collection endpoint.
 * Vendor-defined usage page to avoid conflicts with standard HID drivers. */
static const uint8_t dc_hid_report_desc[] = {
	0x06, 0x00, 0xFF,                           /* Usage Page (Vendor Defined 0xFF00) */
	HID_USAGE(0x01),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
		HID_USAGE(0x01),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(64),
		HID_INPUT(0x02),                    /* Data, Variable, Absolute */
	HID_END_COLLECTION,
};

#define DC_HID_NODE DT_NODELABEL(hid_dev_1)
#define DC_REPORT_SIZE 64U
#define DC_FIFO_SIZE 64U
#define DC_STATS_INTERVAL_MS 3000

/* Device and state */
static const struct device *dc_hdev;
static ATOMIC_DEFINE(dc_ep_busy, 1);
static bool dc_hid_ready;
static const uint8_t *dc_submitted_report;
#define DC_EP_BUSY_FLAG 0

/* Report FIFO: ISR writes here, work handler reads.
 * hid_device_submit_report() is NOT safe from ISR context (event_handler),
 * so we buffer reports and process them in the system work queue.
 * 64 slots × 64 bytes = 4 KB; at 400 Hz input rate this provides
 * ~160 ms of buffering against transient USB stalls. */
static uint8_t dc_fifo[DC_FIFO_SIZE][DC_REPORT_SIZE] __aligned(sizeof(void *));
static atomic_t dc_fifo_write;
static atomic_t dc_fifo_read;
static struct k_work dc_send_work;

static uint32_t dc_frames_sent;
static uint32_t dc_frames_dropped;
static uint32_t dc_frames_received; /* Total raw packets from ESB (before dedup) */

#include "data_collect_session.h"

/* Periodic stats */
static int64_t dc_last_stats_time;
static uint32_t dc_last_stats_sent;
static uint32_t dc_last_stats_dropped;
static uint32_t dc_last_stats_received;

static size_t dc_fifo_next(size_t index)
{
	index++;
	return (index >= DC_FIFO_SIZE) ? 0U : index;
}

static bool dc_fifo_empty(void)
{
	return atomic_get(&dc_fifo_read) == atomic_get(&dc_fifo_write);
}

static void dc_fifo_advance_read(void)
{
	size_t rd = (size_t)atomic_get(&dc_fifo_read);

	atomic_set(&dc_fifo_read, (atomic_val_t)dc_fifo_next(rd));
}

static void dc_drop_current_report(void)
{
	if (!dc_fifo_empty()) {
		dc_fifo_advance_read();
		dc_frames_dropped++;
	}
}

/* Work handler: send buffered reports from thread context */
static void dc_send_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!dc_hdev || !dc_hid_ready) return;

	while (1) {
		size_t rd = (size_t)atomic_get(&dc_fifo_read);
		size_t wr = (size_t)atomic_get(&dc_fifo_write);
		if (rd == wr) break; /* FIFO empty */

		/* Wait for endpoint to be free */
		if (atomic_test_and_set_bit(dc_ep_busy, DC_EP_BUSY_FLAG)) {
			break;
		}

		dc_submitted_report = dc_fifo[rd];
		int ret = hid_device_submit_report(dc_hdev, DC_REPORT_SIZE, dc_fifo[rd]);
		if (ret == 0) {
			break;
		}

		dc_submitted_report = NULL;
		if (ret == -EACCES) {
			atomic_clear_bit(dc_ep_busy, DC_EP_BUSY_FLAG);
			break;
		}

		dc_drop_current_report();
		atomic_clear_bit(dc_ep_busy, DC_EP_BUSY_FLAG);
		break;
	}
}

static void dc_iface_ready_cb(const struct device *dev, const bool ready)
{
	if (dev != dc_hdev) {
		return;
	}

	dc_hid_ready = ready;
	if (!ready) {
		dc_submitted_report = NULL;
	}

	atomic_clear_bit(dc_ep_busy, DC_EP_BUSY_FLAG);
	if (ready) {
		k_work_submit(&dc_send_work);
	}
}

static int dc_get_report_cb(const struct device *dev, const uint8_t type, const uint8_t id,
			    const uint16_t len, uint8_t *const buf)
{
	if (dev != dc_hdev) {
		return -ENODEV;
	}

	if (type != HID_REPORT_TYPE_INPUT || id != 0U) {
		return -ENOTSUP;
	}

	if (len < DC_REPORT_SIZE || buf == NULL) {
		return -EINVAL;
	}

	memset(buf, 0, DC_REPORT_SIZE);
	return DC_REPORT_SIZE;
}

static void dc_input_report_done_cb(const struct device *dev,
				    const uint8_t *const submitted_report)
{
	if (dev != dc_hdev) {
		return;
	}

	if (submitted_report == dc_submitted_report) {
		dc_fifo_advance_read();
		dc_frames_sent++;
		dc_submitted_report = NULL;
	} else {
		LOG_WRN("Data HID completion for stale report buffer");
	}

	atomic_clear_bit(dc_ep_busy, DC_EP_BUSY_FLAG);
	k_work_submit(&dc_send_work);
}

static const struct hid_device_ops dc_hid_ops = {
	.iface_ready = dc_iface_ready_cb,
	.get_report = dc_get_report_cb,
	.input_report_done = dc_input_report_done_cb,
};

/* Periodic timer for timeout checking (HID version has no flush timer) */
static void dc_timeout_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	int64_t now = k_uptime_get();

	if ((dc_active || dc_batch_active) && now - dc_last_stats_time >= DC_STATS_INTERVAL_MS) {
		uint32_t period_sent = dc_frames_sent - dc_last_stats_sent;
		uint32_t period_dropped = dc_frames_dropped - dc_last_stats_dropped;
		uint32_t period_received = dc_frames_received - dc_last_stats_received;

		LOG_INF("DC: rcv=%u/3s sent=%u/3s drop=%u/3s (total sent=%u drop=%u)",
			period_received, period_sent, period_dropped,
			dc_frames_sent, dc_frames_dropped);
		dc_last_stats_time = now;
		dc_last_stats_sent = dc_frames_sent;
		dc_last_stats_dropped = dc_frames_dropped;
		dc_last_stats_received = dc_frames_received;
	}

	dc_timeout_check();
}
static K_TIMER_DEFINE(dc_timeout_timer, dc_timeout_timer_handler, NULL);

int data_collect_init(void)
{
	k_work_init(&dc_send_work, dc_send_work_handler);

	/* Start periodic timeout check (1 second interval) */
	k_timer_start(&dc_timeout_timer, K_SECONDS(1), K_SECONDS(1));

	dc_hdev = DEVICE_DT_GET(DC_HID_NODE);
	if (!device_is_ready(dc_hdev)) {
		LOG_ERR("Cannot get data collection HID device");
		return -ENODEV;
	}

	int ret = hid_device_register(dc_hdev, dc_hid_report_desc,
				      sizeof(dc_hid_report_desc), &dc_hid_ops);
	if (ret) {
		LOG_ERR("Failed to register data collection HID device: %d", ret);
		return ret;
	}

	dc_active = false;
	dc_batch_active = false;
	dc_batch_mask = 0;
	dc_frames_sent = 0;
	dc_frames_dropped = 0;
	dc_frames_received = 0;
	dc_last_rx_time = 0;
	dc_last_stats_time = k_uptime_get();
	dc_last_stats_sent = 0;
	dc_last_stats_dropped = 0;
	dc_last_stats_received = 0;

	LOG_INF("Data collection HID endpoint initialized");
	return 0;
}

SYS_INIT(data_collect_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

void data_collect_write(const uint8_t *data, uint8_t len, uint8_t rssi)
{
	if (!dc_hdev || (!dc_active && !dc_batch_active)) {
		return;
	}

	dc_note_rx();
	dc_frames_received++;
	/* Stats LOG lives in send work — not EVENT IRQ. */

	if (!dc_hid_ready) {
		dc_frames_dropped++;
		return;
	}

	/* FIFO full check */
	size_t wr = (size_t)atomic_get(&dc_fifo_write);
	size_t next = dc_fifo_next(wr);
	if (next == (size_t)atomic_get(&dc_fifo_read)) {
		dc_frames_dropped++;
		return;
	}

	/* Build 64-byte HID report in FIFO slot */
	uint8_t *report = dc_fifo[wr];
	memset(report, 0, DC_REPORT_SIZE);

	/* Allow up to 52 bytes for type 0x13 (gyrQuat) packets */
	uint8_t copy_len = (len > 52) ? 52 : len;
	memcpy(report, data, copy_len);

	uint8_t pos = copy_len;
	report[pos++] = rssi;

	uint32_t rx_ticks = (uint32_t)k_uptime_ticks();
	sys_put_be32(rx_ticks, &report[pos]);

	atomic_set(&dc_fifo_write, (atomic_val_t)next);

	/* Schedule deferred send (ISR-safe) */
	k_work_submit(&dc_send_work);
}

static void dc_reset_fifo_locked(void)
{
	/* Producers are stopped by the caller while this queue ownership changes.
	 * If a report is in flight, it is always the current read slot: the send
	 * work sets dc_submitted_report from dc_fifo[dc_fifo_read], and completion
	 * advances that same slot only after pointer validation. */
	if (dc_submitted_report != NULL) {
		size_t inflight = (size_t)atomic_get(&dc_fifo_read);
		/* Keep only the in-flight slot; discard all queued old reports. */
		atomic_set(&dc_fifo_write, (atomic_val_t)dc_fifo_next(inflight));
	} else {
		atomic_set(&dc_fifo_read, atomic_get(&dc_fifo_write));
	}
}

static void dc_reset_output_locked(void)
{
	dc_reset_fifo_locked();
}

static void dc_reset_stats_locked(int64_t now)
{
	dc_frames_sent = 0;
	dc_frames_dropped = 0;
	dc_frames_received = 0;
	dc_last_stats_time = now;
	dc_last_stats_sent = 0;
	dc_last_stats_dropped = 0;
	dc_last_stats_received = 0;
}

void data_collect_forget_tracker(uint8_t tracker_id)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}
	unsigned int key = irq_lock();
	if (dc_target_tracker_id == tracker_id) {
		dc_active = false;
		dc_target_tracker_id = UINT8_MAX;
	}
	dc_batch_mask &= ~BIT(tracker_id);
	dc_batch_active = dc_batch_mask != 0;
	/* Preserve USB ownership of an in-flight report, discard queued frames. */
	dc_reset_fifo_locked();
	irq_unlock(key);
}

bool data_collect_is_active(void)
{
	return dc_active;
}

uint8_t data_collect_get_target_id(void)
{
	return dc_target_tracker_id;
}

bool data_collect_batch_is_active(void)
{
	return dc_batch_active;
}

bool data_collect_batch_is_target(uint8_t tracker_id)
{
	return dc_batch_active && tracker_id < MAX_TRACKERS && (dc_batch_mask & BIT(tracker_id)) != 0U;
}

bool data_collect_is_target(uint8_t tracker_id)
{
	return dc_active && tracker_id == dc_target_tracker_id;
}
