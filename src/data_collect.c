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

#include "data_collect.h"
#include "connection/esb.h"
#include "globals.h"

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
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

#include "data_collect_session.h"

/* CDC drains already queued bytes on stop/replacement; only HID resets its queue. */
static void dc_reset_output_locked(void)
{
}

static void dc_reset_stats_locked(int64_t now)
{
	ARG_UNUSED(now);
	dc_frames_sent = 0;
	dc_frames_dropped = 0;
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

static bool cdc_host_is_open(void)
{
	uint32_t dtr = 0;

	return cdc_dev != NULL &&
	       uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_DTR, &dtr) == 0 &&
	       dtr != 0;
}

static void dc_discard_buffer(void)
{
	dc_buf_tail = dc_buf_head;
}

static uint32_t dc_contiguous_len(void)
{
	uint32_t head = dc_buf_head;
	uint32_t tail = dc_buf_tail;

	if (tail == head) {
		return 0;
	}

	if (tail < head) {
		return head - tail;
	}

	return DATA_COLLECT_BUF_SIZE - tail;
}

static void dc_tx_kick_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(dc_tx_kick_work, dc_tx_kick_work_handler);

/* IRQ callbacks and producer kicks may run on different workqueues. Disable
 * first, then recheck: a producer before the disable must not lose its enable.
 * A producer after the recheck schedules its own kick.
 */
static void dc_tx_pause(const struct device *dev)
{
	uart_irq_tx_disable(dev);
	if (dc_buf_tail != dc_buf_head) {
		k_work_schedule(&dc_tx_kick_work, K_MSEC(1));
	}
}

static void dc_uart_irq_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (dev != cdc_dev) {
		return;
	}

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!uart_irq_tx_ready(dev)) {
			continue;
		}

		if (!cdc_host_is_open()) {
			dc_discard_buffer();
			dc_tx_pause(dev);
			continue;
		}

		uint32_t len = dc_contiguous_len();
		if (len == 0) {
			dc_tx_pause(dev);
			continue;
		}

		int sent = uart_fifo_fill(dev, &dc_buf[dc_buf_tail], len);
		if (sent <= 0) {
			dc_tx_pause(dev);
			continue;
		}

		dc_buf_tail = (dc_buf_tail + (uint32_t)sent) % DATA_COLLECT_BUF_SIZE;
		if (dc_buf_tail == dc_buf_head) {
			dc_tx_pause(dev);
		}
	}
}

static void dc_tx_kick_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!cdc_dev) {
		return;
	}

	if (!cdc_host_is_open()) {
		dc_discard_buffer();
		return;
	}

	if (dc_buf_tail != dc_buf_head) {
		uart_irq_tx_enable(cdc_dev);
		/* The CDC driver normally advances TX from completion/enable/resume.
		 * Keep a bounded fallback only while app bytes remain: FIFO-full can
		 * suppress callbacks, including observation of DTR closing.
		 */
		if (dc_buf_tail != dc_buf_head) {
			k_work_schedule(&dc_tx_kick_work, K_MSEC(1));
		}
	}
}

static void dc_timer_handler(struct k_timer *timer);
static K_TIMER_DEFINE(dc_timer, dc_timer_handler, NULL);

static void dc_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	dc_timeout_check();
}

int data_collect_init(void)
{
	/* Get the dedicated data-collection CDC ACM device */
	cdc_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart1));
	if (!device_is_ready(cdc_dev)) {
		LOG_ERR("CDC ACM device not ready");
		return -ENODEV;
	}

	int ret = uart_irq_callback_user_data_set(cdc_dev, dc_uart_irq_callback, NULL);
	if (ret != 0) {
		LOG_ERR("Failed to set CDC ACM IRQ callback: %d", ret);
		return ret;
	}

	cdc_ready = true;
	dc_active = false;
	dc_batch_active = false;
	dc_batch_mask = 0;
	k_timer_start(&dc_timer, K_SECONDS(1), K_SECONDS(1));

	LOG_INF("Data collection subsystem initialized");
	return 0;
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
	/* CDC frames share one byte ring; discard queued old-slot frames. */
	dc_discard_buffer();
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
	return dc_batch_active && tracker_id < MAX_TRACKERS && (dc_batch_mask & BIT(tracker_id)) != 0;
}

bool data_collect_is_target(uint8_t tracker_id)
{
	return dc_active && tracker_id == dc_target_tracker_id;
}

void data_collect_write(const uint8_t *data, uint8_t len, uint8_t rssi)
{
	if (!cdc_ready) return;

	dc_note_rx();

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

	k_work_schedule(&dc_tx_kick_work, K_NO_WAIT);
}
