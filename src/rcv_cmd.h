/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors
*/
#ifndef SLIMENRF_RCV_CMD_H
#define SLIMENRF_RCV_CMD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rcv_hid_cmd.h"

#if defined(CONFIG_RELAY)
/* Called once per new stage under irq_lock. */
void rcv_cmd_hid_stage_started(void);
#endif

/* Shared command core used by console and HID. Status codes match RCV_HID_ST_*. */

uint8_t rcv_cmd_pair(uint8_t count);
uint8_t rcv_cmd_exit_pair(void);
uint8_t rcv_cmd_clear(void);
uint8_t rcv_cmd_add(uint64_t addr, int8_t *slot_out);
uint8_t rcv_cmd_remove(void);
uint8_t rcv_cmd_list(void);
uint8_t rcv_cmd_channel_set(uint8_t channel);
uint8_t rcv_cmd_channel_clear(void);
uint8_t rcv_cmd_rssi_scan(void);
uint8_t rcv_cmd_info(void);
uint8_t rcv_cmd_uptime(void);
uint8_t rcv_cmd_stats(uint32_t duration_seconds);
uint8_t rcv_cmd_resetstats(void);
uint8_t rcv_cmd_collect_start(uint8_t tracker_id);
uint8_t rcv_cmd_collect_stop(void);
uint8_t rcv_cmd_collect_batch_start(uint16_t rate_hz);
uint8_t rcv_cmd_collect_batch_stop(void);
/* Queue selected metadata repair for one active tracker. */
uint8_t rcv_cmd_collect_meta(uint8_t tracker_id, uint8_t mask, uint8_t chunk);
uint8_t rcv_cmd_reboot(void);
uint8_t rcv_cmd_dfu(bool ota);
uint8_t rcv_cmd_tracker_channel_all(uint8_t channel);
uint8_t rcv_cmd_tracker_channel_clear_all(void);

/* Queue ESB PONG remote flag. target_id RCV_HID_TARGET_ALL = all active.
 * Channel/test flags require their typed APIs. Raw ON/OFF use collection
 * admission; batch ON requires the rate-bearing batch API. */
uint8_t rcv_cmd_remote_flag(uint8_t target_id, uint8_t pong_flag);
/* Queue TEST_MODE_ON with optional target TPS (0 = tracker default). */
uint8_t rcv_cmd_remote_test_on(uint8_t target_id, uint16_t tps);
/* Queue TEST_MODE_OFF. */
uint8_t rcv_cmd_remote_test_off(uint8_t target_id);
/* Async terminal WOM query (0), set (1..35791), clear (ESB_WOM_TIMEOUT_CLEAR),
 * or entry (ESB_WOM_ENTER). Success for entry means admission, not sleep.
 * Shares the HID remote job; all targets uses the active-tracker scan. */
uint8_t rcv_cmd_remote_wom_command(uint8_t target_id, uint16_t minutes);

/* sens set / sens auto helpers */
uint8_t rcv_cmd_remote_sens_set(uint8_t target_id, float x, float y, float z);
uint8_t rcv_cmd_remote_sens_auto(uint8_t target_id, uint8_t axis, uint16_t revolutions);

/*
 * Process one HID type-254 command (first 16 bytes). Writes 16-byte ACK into ack_out.
 * Returns true if ack_out is valid and should be sent.
 */
bool rcv_cmd_process_hid(const uint8_t *buf, size_t len, uint32_t usb_generation,
			 uint8_t ack_out[RCV_HID_CMD_LEN]);

/* Async completion ACKs (e.g. tracker channel-all). Registered by HID, not console. */
typedef void (*rcv_cmd_async_ack_fn)(const uint8_t ack[RCV_HID_CMD_LEN]);
void rcv_cmd_set_async_ack(rcv_cmd_async_ack_fn fn);

#endif
