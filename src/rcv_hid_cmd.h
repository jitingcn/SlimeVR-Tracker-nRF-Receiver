/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors
*/
#ifndef SLIMENRF_RCV_HID_CMD_H
#define SLIMENRF_RCV_HID_CMD_H

#include <stdbool.h>
#include <stdint.h>

#include "connection/esb.h"

/* HID report types (buf[0]). Dongle control envelope / ACK / registration. */
#define RCV_HID_TYPE_CMD_ACK     251
#define RCV_HID_TYPE_CMD         254
#define RCV_HID_TYPE_DEVICE_ADDR 255

/*
 * Opcode ranges (byte [2] inside type 254 / echoed in ACK):
 *   1–200   Tracker remote (aligned with ESB_PONG_FLAG_* where possible)
 *   201–253 Dongle-local (skip 240–247 HID OTA report types)
 *   254     Reserved (envelope type itself)
 *
 * Legacy tracker stream types 0–7 stay outside this table.
 */

/* Tracker opcodes: use ESB_PONG_FLAG_* values directly (0x01–0x33). */
#define RCV_HID_OP_TRACKER_MIN 1
#define RCV_HID_OP_TRACKER_MAX 200

/* Dongle opcodes */
#define RCV_HID_OP_DONGLE_MIN 201
#define RCV_HID_OP_DONGLE_MAX 253

#define RCV_HID_OP_PAIR           201
#define RCV_HID_OP_EXIT_PAIR      202
#define RCV_HID_OP_CLEAR          203
#define RCV_HID_OP_ADD            204
#define RCV_HID_OP_REMOVE         205
#define RCV_HID_OP_LIST           206
#define RCV_HID_OP_CHANNEL_SET    207
#define RCV_HID_OP_CHANNEL_CLEAR  208
#define RCV_HID_OP_RSSI_SCAN      209
#define RCV_HID_OP_INFO           210
#define RCV_HID_OP_UPTIME         211
#define RCV_HID_OP_STATS          212
#define RCV_HID_OP_RESETSTATS     213
#define RCV_HID_OP_COLLECT_START  214
#define RCV_HID_OP_COLLECT_STOP   215
#define RCV_HID_OP_REBOOT         216
#define RCV_HID_OP_DFU            217
#define RCV_HID_OP_TRACKER_CH_ALL 218
#define RCV_HID_OP_TRACKER_CH_CLR 219
#define RCV_HID_OP_NOP            220
#define RCV_HID_OP_COLLECT_BATCH_START 221
#define RCV_HID_OP_COLLECT_BATCH_STOP  222
#define RCV_HID_OP_COLLECT_META        223
#define RCV_HID_OP_TRACKER_EVENTS      224
#define RCV_HID_OP_TRACKER_EVENT       225 /* IN only */
#define RCV_HID_OP_TRACKER_OBSERVATION 226 /* IN only */

/* HID OTA types occupy 0xF0–0xF7; never assign dongle opcodes here. */
#define RCV_HID_OTA_TYPE_MIN 0xF0
#define RCV_HID_OTA_TYPE_MAX 0xF7

/* Request flags (byte [3]) */
#define RCV_HID_FLAG_MORE 0x01

/* Target id 0xFF = all active trackers (console "all") */
#define RCV_HID_TARGET_ALL 0xFF

/* ACK status (byte [3] of type 251) */
#define RCV_HID_ST_OK       0
#define RCV_HID_ST_EINVAL   1
#define RCV_HID_ST_ENOSPC   2
#define RCV_HID_ST_EBUSY    3
#define RCV_HID_ST_ENOENT   4
#define RCV_HID_ST_ENOTSUP  5
#define RCV_HID_ST_QUEUED   6
#define RCV_HID_ST_STARTED  7

/* Async tracker control: STARTED owns the request until its final ACK.
 * Identical queued commands can share a confirmation with the console.
 * ENOENT means no eligible targets or incomplete confirmation at the deadline.
 * It withdraws pending one-shot HID commands, not identical receiver-owned work
 * or sticky TEST_MODE_ON/OFF all-target policy. It cannot recall an emitted PONG. */

#define RCV_HID_CMD_LEN 16

/* WOM request: target [4], LE16 operation [5:7] (0=query, 1..35791=minutes,
 * ESB_WOM_TIMEOUT_CLEAR=clear saved override and restore exact firmware default,
 * ESB_WOM_ENTER=request immediate WOM admission).
 * ACK result: STARTED, kind [4]=1, id [5], LE16 value [6:8],
 * LE16 errno [8:10], LE16 receiver token [10:12].
 * Value=0 with errno=0 confirms clear; ESB_WOM_ENTER confirms entry admission,
 * not physical sleep. Otherwise value is timeout minutes; nonzero errno is failure.
 * ACK summary: OK/ENOENT, kind [4]=2, LE16 token [5:7],
 * LE32 success mask [7:11], LE32 failure mask [11:15].
 * Unused bytes are zero. All targets uses the active-tracker scan.
 */
#define RCV_HID_WOM_RESULT 1
#define RCV_HID_WOM_SUMMARY 2

static inline bool rcv_hid_opcode_is_tracker(uint8_t op)
{
	return op >= RCV_HID_OP_TRACKER_MIN && op <= RCV_HID_OP_TRACKER_MAX;
}

static inline bool rcv_hid_opcode_is_dongle(uint8_t op)
{
	if (op < RCV_HID_OP_DONGLE_MIN || op > RCV_HID_OP_DONGLE_MAX) {
		return false;
	}
	if (op >= RCV_HID_OTA_TYPE_MIN && op <= RCV_HID_OTA_TYPE_MAX) {
		return false;
	}
	return true;
}

static inline bool rcv_hid_opcode_is_pong_flag(uint8_t op)
{
	switch (op) {
	case ESB_PONG_FLAG_SHUTDOWN:
	case ESB_PONG_FLAG_CALIBRATE:
	case ESB_PONG_FLAG_CALIBRATE_ACC:
	case ESB_PONG_FLAG_MEOW:
	case ESB_PONG_FLAG_SCAN:
	case ESB_PONG_FLAG_MAG_CLEAR:
	case ESB_PONG_FLAG_REBOOT:
	case ESB_PONG_FLAG_CLEAR:
	case ESB_PONG_FLAG_DFU:
	case ESB_PONG_FLAG_SET_CHANNEL:
	case ESB_PONG_FLAG_CLEAR_CHANNEL:
	case ESB_PONG_FLAG_SENS_SET:
	case ESB_PONG_FLAG_SENS_RESET:
	case ESB_PONG_FLAG_RESET_ZRO:
	case ESB_PONG_FLAG_RESET_ACC:
	case ESB_PONG_FLAG_RESET_BAT:
	case ESB_PONG_FLAG_PING:
	case ESB_PONG_FLAG_RESET_TCAL:
	case ESB_PONG_FLAG_TCAL_AUTO_ON:
	case ESB_PONG_FLAG_TCAL_AUTO_OFF:
	case ESB_PONG_FLAG_FUSION_RESET:
	case ESB_PONG_FLAG_TCAL_BOOT_ON:
	case ESB_PONG_FLAG_TCAL_BOOT_OFF:
	case ESB_PONG_FLAG_MAG_CAL:
	case ESB_PONG_FLAG_MAG_ON:
	case ESB_PONG_FLAG_MAG_OFF:
	case ESB_PONG_FLAG_TCAL_ON:
	case ESB_PONG_FLAG_TCAL_OFF:
	case ESB_PONG_FLAG_TDMA_ON:
	case ESB_PONG_FLAG_TDMA_OFF:
	case ESB_PONG_FLAG_TEST_MODE_ON:
	case ESB_PONG_FLAG_TEST_MODE_OFF:
	case ESB_PONG_FLAG_DFU_OTA:
	case ESB_PONG_FLAG_DATA_COLLECT_ON:
	case ESB_PONG_FLAG_DATA_COLLECT_OFF:
	case ESB_PONG_FLAG_SENS_AUTO:
	case ESB_PONG_FLAG_MAG_AUTO_ON:
	case ESB_PONG_FLAG_MAG_AUTO_OFF:
	case ESB_PONG_FLAG_TCAL_HEATED_START:
	case ESB_PONG_FLAG_WOM_CONTROL:
	case ESB_PONG_FLAG_OTA_QUERY_INFO:
	case ESB_PONG_FLAG_OTA_ABORT:
	case ESB_PONG_FLAG_OTA_SUPPRESS:
	case ESB_PONG_FLAG_OTA_UNSUPPRESS:
		return true;
	default:
		return false;
	}
}

#endif
