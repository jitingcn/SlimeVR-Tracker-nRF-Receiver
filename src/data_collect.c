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

#include "globals.h"
#include "data_collect.h"
#include "connection/esb.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(data_collect, LOG_LEVEL_INF);

/* Dedicated CDC ACM UART device for data collection (separate from console) */
static const struct device *cdc_dev;
static bool cdc_ready;

/* Ring buffer for CDC output to decouple ISR-context writes from USB.
 * 16 KB holds ~284 frames (57 bytes each) ≈ 716 ms at 400 Hz. */
#define DATA_COLLECT_BUF_SIZE 16384
static uint8_t dc_buf[DATA_COLLECT_BUF_SIZE];
static volatile uint32_t dc_buf_head;
static volatile uint32_t dc_buf_tail;

/* Statistics */
static uint32_t dc_frames_sent;
static uint32_t dc_frames_dropped;

/* Runtime state */
static bool dc_active;
static uint8_t dc_target_tracker_id;

/* Timeout: auto-stop if no raw data received for this long */
#define DC_TIMEOUT_MS 60000
static int64_t dc_last_rx_time;

/* CRC-8 CCITT (polynomial 0x07) - same as ESB uses */
static uint8_t crc8_ccitt(uint8_t crc, const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int j = 0; j < 8; j++) {
			if (crc & 0x80)
				crc = (crc << 1) ^ 0x07;
			else
				crc <<= 1;
		}
	}
	return crc;
}

static inline uint32_t buf_used(void)
{
	int32_t diff = (int32_t)dc_buf_head - (int32_t)dc_buf_tail;
	if (diff < 0) diff += DATA_COLLECT_BUF_SIZE;
	return (uint32_t)diff;
}

static inline uint32_t buf_free(void)
{
	return DATA_COLLECT_BUF_SIZE - 1 - buf_used();
}

static void buf_put(uint8_t byte)
{
	dc_buf[dc_buf_head] = byte;
	dc_buf_head = (dc_buf_head + 1) % DATA_COLLECT_BUF_SIZE;
}

/* Flush ring buffer to CDC UART (called from workqueue) */
static void dc_flush_work_handler(struct k_work *work);
static K_WORK_DEFINE(dc_flush_work, dc_flush_work_handler);

static void dc_flush_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!cdc_dev) return;

	/* Only send when a host has opened the port (DTR asserted) */
	uint32_t dtr = 0;
	if (uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_DTR, &dtr) != 0 || !dtr) {
		/* No host listening — silently discard buffered data to avoid
		 * unbounded ring buffer growth while nobody is connected. */
		dc_buf_tail = dc_buf_head;
		return;
	}

	while (dc_buf_tail != dc_buf_head) {
		uint8_t byte = dc_buf[dc_buf_tail];
		/* uart_poll_out blocks until the byte is accepted */
		uart_poll_out(cdc_dev, byte);
		dc_buf_tail = (dc_buf_tail + 1) % DATA_COLLECT_BUF_SIZE;
	}
}

/* Periodic flush timer - ensures data gets sent even without new packets */
static void dc_timer_handler(struct k_timer *timer);
static K_TIMER_DEFINE(dc_timer, dc_timer_handler, NULL);

/* Timeout work: stop collection and notify tracker (runs in workqueue context) */
static void dc_timeout_work_handler(struct k_work *work);
static K_WORK_DEFINE(dc_timeout_work, dc_timeout_work_handler);

static void dc_timeout_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!dc_active) return;
	uint8_t tid = dc_target_tracker_id;
	data_collect_stop();
	esb_send_remote_command(tid, ESB_PONG_FLAG_DATA_COLLECT_OFF);
	LOG_WRN("Data collection timed out (no data for %d s), sent OFF to tracker %u",
		DC_TIMEOUT_MS / 1000, tid);
}

static void dc_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_work_submit(&dc_flush_work);

	/* Check for data collection timeout */
	if (dc_active && dc_last_rx_time > 0 &&
	    (k_uptime_get() - dc_last_rx_time) > DC_TIMEOUT_MS) {
		k_work_submit(&dc_timeout_work);
	}
}

int data_collect_init(void)
{
	/* Get the dedicated data-collection CDC ACM device */
	cdc_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart1));
	if (!device_is_ready(cdc_dev)) {
		LOG_ERR("CDC ACM device not ready");
		return -ENODEV;
	}

	cdc_ready = true;
	dc_frames_sent = 0;
	dc_frames_dropped = 0;
	dc_active = false;

	/* Start periodic flush at 1ms to keep latency low */
	k_timer_start(&dc_timer, K_MSEC(1), K_MSEC(1));

	LOG_INF("Data collection subsystem initialized");
	return 0;
}

void data_collect_start(uint8_t tracker_id)
{
	dc_target_tracker_id = tracker_id;
	dc_active = true;
	dc_frames_sent = 0;
	dc_frames_dropped = 0;
	dc_last_rx_time = k_uptime_get();
	LOG_INF("Data collection STARTED for tracker %u", tracker_id);
}

void data_collect_stop(void)
{
	dc_active = false;
	LOG_INF("Data collection STOPPED (sent: %u, dropped: %u)",
		dc_frames_sent, dc_frames_dropped);
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

void data_collect_write(const uint8_t *data, uint8_t len, uint8_t rssi)
{
	if (!cdc_ready) return;

	dc_last_rx_time = k_uptime_get();

	/* Frame: [0xAA][0x55][length][payload...][rssi][rx_ticks(4)][CRC-8]
	 * Total overhead: 2 sync + 1 len + 1 rssi + 4 ticks + 1 crc = 9 bytes */
	uint32_t frame_size = 2 + 1 + len + 1 + 4 + 1;

	if (buf_free() < frame_size) {
		dc_frames_dropped++;
		return;
	}

	uint32_t rx_ticks = (uint32_t)k_uptime_ticks();

	/* Write sync marker */
	buf_put(0xAA);
	buf_put(0x55);

	/* Length byte (payload + rssi + ticks = len + 5) */
	uint8_t frame_len = len + 5;
	buf_put(frame_len);

	/* Payload (raw ESB data) */
	uint8_t crc = 0x07;
	crc = crc8_ccitt(crc, &frame_len, 1);
	for (uint8_t i = 0; i < len; i++) {
		buf_put(data[i]);
	}
	crc = crc8_ccitt(crc, data, len);

	/* RSSI */
	buf_put(rssi);
	crc = crc8_ccitt(crc, &rssi, 1);

	/* Receiver timestamp (big-endian) */
	uint8_t ticks_buf[4];
	sys_put_be32(rx_ticks, ticks_buf);
	for (int i = 0; i < 4; i++) {
		buf_put(ticks_buf[i]);
	}
	crc = crc8_ccitt(crc, ticks_buf, 4);

	/* CRC */
	buf_put(crc);

	dc_frames_sent++;

	/* Trigger flush */
	k_work_submit(&dc_flush_work);
}
