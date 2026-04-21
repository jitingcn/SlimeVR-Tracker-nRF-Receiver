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
 * Data collection via dedicated HID endpoint (HID_1).
 *
 * Each raw ESB packet is forwarded as a single 64-byte HID input report:
 *   [0..N-1]  ESB payload (up to 48 bytes)
 *   [N]       RSSI
 *   [N+1..N+4] rx_ticks (BE32)
 *   [N+5..63]  zero padding
 *
 * No framing protocol needed — HID guarantees packet boundaries.
 * The PC reads from the second HID interface of the same USB device.
 */

#include "globals.h"
#include "data_collect.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(data_collect, LOG_LEVEL_INF);

/* HID report descriptor for data collection endpoint.
 * Vendor-defined usage page to avoid conflicts with standard HID drivers. */
static const uint8_t dc_hid_report_desc[] = {
	0x06, 0x00, 0xFF,                           /* Usage Page (Vendor Defined 0xFF00) */
	HID_USAGE(0x01),                            /* Vendor usage 0x01 */
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
		HID_USAGE(0x01),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(64),
		HID_INPUT(0x02),                    /* Data, Variable, Absolute */
	HID_END_COLLECTION,
};

/* Device and state */
static const struct device *dc_hdev;
static ATOMIC_DEFINE(dc_ep_busy, 1);
#define DC_EP_BUSY_FLAG 0

/* Report FIFO: ISR writes here, work handler reads.
 * hid_int_ep_write() is NOT safe from ISR context (event_handler),
 * so we buffer reports and process them in the system work queue. */
#define DC_FIFO_SIZE 16
static uint8_t dc_fifo[DC_FIFO_SIZE][64];
static atomic_t dc_fifo_write;
static atomic_t dc_fifo_read;
static struct k_work dc_send_work;

/* Runtime state */
static bool dc_active;
static uint8_t dc_target_tracker_id;
static uint32_t dc_frames_sent;
static uint32_t dc_frames_dropped;
static uint32_t dc_frames_received; /* Total raw packets from ESB (before dedup) */

/* Periodic stats */
static int64_t dc_last_stats_time;
static uint32_t dc_last_stats_sent;
static uint32_t dc_last_stats_dropped;
static uint32_t dc_last_stats_received;

/* Work handler: send buffered reports from thread context */
static void dc_send_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!dc_hdev) return;

	while (1) {
		size_t rd = (size_t)atomic_get(&dc_fifo_read);
		size_t wr = (size_t)atomic_get(&dc_fifo_write);
		if (rd == wr) break; /* FIFO empty */

		/* Wait for endpoint to be free */
		if (atomic_test_and_set_bit(dc_ep_busy, DC_EP_BUSY_FLAG)) {
			/* Endpoint busy — will be rescheduled by int_in_ready_cb */
			break;
		}

		int wrote;
		int ret = hid_int_ep_write(dc_hdev, dc_fifo[rd], 64, &wrote);
		if (ret != 0) {
			atomic_clear_bit(dc_ep_busy, DC_EP_BUSY_FLAG);
			dc_frames_dropped++;
		} else {
			dc_frames_sent++;
		}

		size_t next = rd + 1;
		if (next >= DC_FIFO_SIZE) next = 0;
		atomic_set(&dc_fifo_read, next);
	}
}

/* Endpoint callbacks */
static void dc_int_in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	atomic_clear_bit(dc_ep_busy, DC_EP_BUSY_FLAG);
	/* Reschedule work to send next buffered report */
	k_work_submit(&dc_send_work);
}

static const struct hid_ops dc_hid_ops = {
	.int_in_ready = dc_int_in_ready_cb,
};

int data_collect_init(void)
{
	k_work_init(&dc_send_work, dc_send_work_handler);

	dc_hdev = device_get_binding("HID_1");
	if (dc_hdev == NULL) {
		LOG_ERR("Cannot get HID_1 device for data collection");
		return -ENODEV;
	}

	usb_hid_register_device(dc_hdev, dc_hid_report_desc,
				sizeof(dc_hid_report_desc), &dc_hid_ops);

	if (usb_hid_set_proto_code(dc_hdev, HID_BOOT_IFACE_CODE_NONE)) {
		LOG_WRN("Failed to set HID_1 Protocol Code");
	}

	int ret = usb_hid_init(dc_hdev);
	if (ret) {
		LOG_ERR("Failed to init HID_1: %d", ret);
		return ret;
	}

	dc_active = false;
	dc_frames_sent = 0;
	dc_frames_dropped = 0;

	LOG_INF("Data collection HID endpoint initialized (HID_1)");
	return 0;
}

SYS_INIT(data_collect_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

void data_collect_write(const uint8_t *data, uint8_t len, uint8_t rssi)
{
	if (!dc_hdev || !dc_active) return;

	dc_frames_received++;

	/* Periodic stats (every 3 seconds) */
	int64_t now = k_uptime_get();
	if (now - dc_last_stats_time >= 3000) {
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

	/* FIFO full check */
	size_t wr = (size_t)atomic_get(&dc_fifo_write);
	size_t next = wr + 1;
	if (next >= DC_FIFO_SIZE) next = 0;
	if (next == (size_t)atomic_get(&dc_fifo_read)) {
		dc_frames_dropped++;
		return;
	}

	/* Build 64-byte HID report in FIFO slot */
	uint8_t *report = dc_fifo[wr];
	memset(report, 0, 64);

	uint8_t copy_len = (len > 48) ? 48 : len;
	memcpy(report, data, copy_len);

	uint8_t pos = copy_len;
	report[pos++] = rssi;

	uint32_t rx_ticks = (uint32_t)k_uptime_ticks();
	sys_put_be32(rx_ticks, &report[pos]);

	atomic_set(&dc_fifo_write, next);

	/* Schedule deferred send (ISR-safe) */
	k_work_submit(&dc_send_work);
}

void data_collect_start(uint8_t tracker_id)
{
	dc_target_tracker_id = tracker_id;
	dc_active = true;
	dc_frames_sent = 0;
	dc_frames_dropped = 0;
	dc_frames_received = 0;
	dc_last_stats_time = k_uptime_get();
	dc_last_stats_sent = 0;
	dc_last_stats_dropped = 0;
	dc_last_stats_received = 0;
	/* Reset FIFO */
	atomic_set(&dc_fifo_write, 0);
	atomic_set(&dc_fifo_read, 0);
	/* Clear busy flag to ensure endpoint is ready */
	atomic_clear_bit(dc_ep_busy, DC_EP_BUSY_FLAG);
	LOG_INF("Data collection STARTED for tracker %u (HID)", tracker_id);
}

void data_collect_stop(void)
{
	dc_active = false;
	LOG_INF("Data collection STOPPED (received: %u, sent: %u, dropped: %u)",
		dc_frames_received, dc_frames_sent, dc_frames_dropped);
}

bool data_collect_is_active(void)
{
	return dc_active;
}

uint8_t data_collect_get_target_id(void)
{
	return dc_target_tracker_id;
}

bool data_collect_is_target(uint8_t tracker_id)
{
	return dc_active && tracker_id == dc_target_tracker_id;
}
