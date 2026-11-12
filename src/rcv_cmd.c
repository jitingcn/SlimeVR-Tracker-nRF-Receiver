/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors
*/
#include "rcv_cmd.h"

#include "build_defines.h"
#include "connection/esb.h"
#include "connection/rssi_scan.h"
#include "data_collect.h"
#include "esb_ota.h"
#include "globals.h"
#include "system/system.h"
#include "tracker_events.h"

#include <errno.h>
#include <string.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(rcv_cmd, LOG_LEVEL_INF);

#define DFU_EXISTS (CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER || CONFIG_BOOTLOADER_MCUBOOT)

static atomic_t rssi_scan_busy = ATOMIC_INIT(0);

/* Async completion ACK for tracker channel-all (seq/opcode from STARTED request). */
static atomic_t ch_hid_pending = ATOMIC_INIT(0);
static uint8_t ch_hid_seq;
static uint8_t ch_hid_opcode;
static rcv_cmd_async_ack_fn async_ack_fn;
static K_MUTEX_DEFINE(channel_completion_lock);

enum pending_reset {
	PENDING_RESET_NONE = 0,
	PENDING_RESET_REBOOT,
	PENDING_RESET_DFU_UF2,
	PENDING_RESET_DFU_OTA,
};

static atomic_t pending_reset = ATOMIC_INIT(PENDING_RESET_NONE);

static void rssi_scan_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	rssi_scan_run_and_print();
	atomic_set(&rssi_scan_busy, 0);
}

static K_WORK_DEFINE(rssi_scan_work, rssi_scan_work_handler);

/* Own queue so multi-second scan cannot stall HID send_report on system WQ. */
static K_THREAD_STACK_DEFINE(rssi_wq_stack, 1024);
static struct k_work_q rssi_wq;

static int rssi_wq_init(void)
{
	k_work_queue_init(&rssi_wq);
	k_work_queue_start(&rssi_wq, rssi_wq_stack, K_THREAD_STACK_SIZEOF(rssi_wq_stack),
			   RSSI_WQ_PRIORITY, NULL);
	k_thread_name_set(&rssi_wq.thread, "rssi_wq");
	return 0;
}

SYS_INIT(rssi_wq_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static void reset_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	int kind = (int)atomic_get(&pending_reset);
	atomic_set(&pending_reset, PENDING_RESET_NONE);
	switch (kind) {
	case PENDING_RESET_REBOOT:
		sys_skip_dfu_marker();
		sys_request_system_reboot();
		break;
	case PENDING_RESET_DFU_UF2:
		sys_enter_dfu(false);
		break;
	case PENDING_RESET_DFU_OTA:
		sys_enter_dfu(true);
		break;
	default:
		break;
	}
}

static K_WORK_DELAYABLE_DEFINE(reset_work, reset_work_handler);

static void schedule_reset(enum pending_reset kind)
{
	atomic_set(&pending_reset, kind);
	k_work_schedule(&reset_work, K_MSEC(50));
}

void rcv_cmd_set_async_ack(rcv_cmd_async_ack_fn fn)
{
	async_ack_fn = fn;
}

uint8_t rcv_cmd_pair(uint8_t count)
{
	if (count == 0) {
		esb_start_pairing();
	} else {
		esb_start_pairing_with_count(count);
	}
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_exit_pair(void)
{
	esb_finish_pair();
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_clear(void)
{
	return esb_clear() == 0 ? RCV_HID_ST_OK : RCV_HID_ST_EBUSY;
}

uint8_t rcv_cmd_add(uint64_t addr, int8_t *slot_out)
{
	if (addr == 0) {
		return RCV_HID_ST_EINVAL;
	}
	int slot = esb_add_pair(addr, true);
	if (slot >= 0) {
		if (slot_out) {
			*slot_out = (int8_t)slot;
		}
		return RCV_HID_ST_OK;
	}
	if (slot == -ENOSPC) {
		return RCV_HID_ST_ENOSPC;
	}
	return slot == -EINVAL ? RCV_HID_ST_EINVAL : RCV_HID_ST_EBUSY;
}

uint8_t rcv_cmd_remove(void)
{
	return esb_pop_pair() == 0 ? RCV_HID_ST_OK : RCV_HID_ST_EBUSY;
}

uint8_t rcv_cmd_list(void)
{
	printk("Stored devices:\n");
	for (uint8_t i = 0; i < stored_trackers; i++) {
		printk("%012llX\n", stored_tracker_addr[i]);
	}
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_channel_set(uint8_t channel)
{
	if (channel > 100) {
		return RCV_HID_ST_EINVAL;
	}
	int err = esb_set_receiver_channel(channel);
	return err == 0 ? RCV_HID_ST_OK :
		err == -EBUSY ? RCV_HID_ST_EBUSY : RCV_HID_ST_ENOENT;
}

uint8_t rcv_cmd_channel_clear(void)
{
	int err = esb_clear_receiver_channel();
	return err == 0 ? RCV_HID_ST_OK :
		err == -EBUSY ? RCV_HID_ST_EBUSY : RCV_HID_ST_ENOENT;
}

uint8_t rcv_cmd_rssi_scan(void)
{
	if (esb_channel_is_busy()) {
		return RCV_HID_ST_EBUSY;
	}
	if (!atomic_cas(&rssi_scan_busy, 0, 1)) {
		return RCV_HID_ST_EBUSY;
	}
	k_work_submit_to_queue(&rssi_wq, &rssi_scan_work);
	return RCV_HID_ST_STARTED;
}

uint8_t rcv_cmd_info(void)
{
	printk(CONFIG_SLIMEVR_USB_DEVICE_MANUFACTURER " " CONFIG_SLIMEVR_USB_DEVICE_PRODUCT "\n");
	printk(FW_STRING);
	printk("Repo: %s | Branch: %s\n", FW_GIT_REPO_URL, FW_GIT_BRANCH);

	printk("\nBoard: " CONFIG_BOARD "\n");
	printk("SOC: " CONFIG_SOC "\n");
	printk("Target: " CONFIG_BOARD_TARGET "\n");

	printk("\nDevice address: %012llX\n", *(uint64_t *)NRF_FICR->DEVICEADDR & 0xFFFFFFFFFFFF);

	const struct sys_fatal_record *fatal = sys_last_fatal();
	if (fatal) {
		printk("Last fatal: reason=%u pc=0x%08x lr=0x%08x thread=0x%08x\n",
			fatal->reason, fatal->pc, fatal->lr, fatal->thread);
	}

	uint8_t current_channel = esb_get_receiver_channel();
	if (current_channel != ESB_RF_CHANNEL_DEFAULT) {
		printk("RF Channel: %u (custom)\n", current_channel);
	} else {
		printk("RF Channel: %u (default)\n", CONFIG_RADIO_RF_CHANNEL);
	}
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_uptime(void)
{
	int64_t uptime = k_ticks_to_us_floor64(k_uptime_ticks());

	uint32_t days = uptime / 86400000000;
	uptime %= 86400000000;
	uint8_t hours = uptime / 3600000000;
	uptime %= 3600000000;
	uint8_t minutes = uptime / 60000000;
	uptime %= 60000000;
	uint8_t seconds = uptime / 1000000;
	uptime %= 1000000;
	uint16_t milliseconds = uptime / 1000;
	uint16_t microseconds = uptime %= 1000;

	printk("Uptime: %u.%02u:%02u:%02u.%03u,%03u\n", days, hours, minutes, seconds, milliseconds,
	       microseconds);
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_stats(uint32_t duration_seconds)
{
	if (duration_seconds == 0) {
		esb_toggle_stats_detailed();
	} else {
		esb_set_stats_detailed(duration_seconds);
	}
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_resetstats(void)
{
	esb_reset_all_stats();
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_collect_start(uint8_t tracker_id)
{
#ifndef CONFIG_DATA_COLLECT
	ARG_UNUSED(tracker_id);
	return RCV_HID_ST_ENOTSUP;
#else
	if (tracker_id >= MAX_TRACKERS) {
		return RCV_HID_ST_EINVAL;
	}
	if (esb_radio_collection_begin()) {
		return RCV_HID_ST_EBUSY;
	}
	rcv_cmd_collect_batch_stop();
	rcv_cmd_collect_stop();
	data_collect_start(tracker_id);
	esb_send_remote_command(tracker_id, ESB_PONG_FLAG_DATA_COLLECT_ON);
	esb_radio_operation_end();
	return RCV_HID_ST_OK;
#endif
}

uint8_t rcv_cmd_collect_stop(void)
{
#ifndef CONFIG_DATA_COLLECT
	return RCV_HID_ST_ENOTSUP;
#else
	if (data_collect_is_active()) {
		uint8_t tid = data_collect_get_target_id();
		data_collect_stop();
		esb_send_remote_command(tid, ESB_PONG_FLAG_DATA_COLLECT_OFF);
	}
	return RCV_HID_ST_OK;
#endif
}

uint8_t rcv_cmd_collect_batch_start(uint16_t rate_hz)
{
#ifndef CONFIG_DATA_COLLECT
	ARG_UNUSED(rate_hz);
	return RCV_HID_ST_ENOTSUP;
#else
	if (rate_hz > UINT8_MAX) {
		return RCV_HID_ST_EINVAL;
	}
	uint32_t mask = 0;
	for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; i++) {
		if (stored_tracker_addr[i] != 0) {
			mask |= BIT(i);
		}
	}
	if (mask == 0) {
		return RCV_HID_ST_ENOENT;
	}
	if (esb_radio_collection_begin()) {
		return RCV_HID_ST_EBUSY;
	}
	rcv_cmd_collect_stop();
	rcv_cmd_collect_batch_stop();
	data_collect_batch_start(mask, rate_hz);
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		if (mask & BIT(i)) {
			esb_send_remote_command_arg(i, ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON, (uint8_t)rate_hz);
		}
	}
	esb_radio_operation_end();
	return RCV_HID_ST_OK;
#endif
}

uint8_t rcv_cmd_collect_batch_stop(void)
{
#ifndef CONFIG_DATA_COLLECT
	return RCV_HID_ST_ENOTSUP;
#else
	if (data_collect_batch_is_active()) {
		uint32_t mask = 0;
		for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
			if (data_collect_batch_is_target(i)) {
				mask |= BIT(i);
			}
		}
		data_collect_batch_stop();
		for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
			if (mask & BIT(i)) {
				esb_send_remote_command(i, ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF);
			}
		}
	}
	return RCV_HID_ST_OK;
#endif
}
uint8_t rcv_cmd_collect_meta(uint8_t tracker_id, uint8_t mask, uint8_t chunk)
{
	if (tracker_id >= MAX_TRACKERS || mask == 0 || (mask & ~ESB_METADATA_MASK_VALID) != 0) {
		return RCV_HID_ST_EINVAL;
	}
	return esb_request_metadata(tracker_id, mask, chunk) ? RCV_HID_ST_QUEUED : RCV_HID_ST_ENOENT;
}
uint8_t rcv_cmd_reboot(void)
{
	schedule_reset(PENDING_RESET_REBOOT);
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_dfu(bool ota)
{
#if !DFU_EXISTS
	ARG_UNUSED(ota);
	return RCV_HID_ST_ENOTSUP;
#else
	schedule_reset(ota ? PENDING_RESET_DFU_OTA : PENDING_RESET_DFU_UF2);
	return RCV_HID_ST_OK;
#endif
}

static void fill_ack(uint8_t ack[RCV_HID_CMD_LEN], uint8_t seq, uint8_t opcode, uint8_t status);

/* ---- HID remote completion (STARTED → OK/ENOENT after tracker confirm) ---- */
#define REMOTE_HID_CONFIRM_TIMEOUT_MS 5000

static atomic_t remote_hid_pending = ATOMIC_INIT(0);
static uint8_t remote_hid_seq;
static uint8_t remote_hid_opcode;
static uint8_t remote_hid_flag;
static atomic_t remote_hid_expected = ATOMIC_INIT(0);
static atomic_t remote_hid_confirmed = ATOMIC_INIT(0);
static bool remote_wom_console;
static uint16_t remote_wom_token;
static uint32_t remote_wom_failed;
static int64_t remote_wom_deadline;
static uint16_t remote_wom_minutes;

enum remote_hid_job {
	REMOTE_HID_JOB_NONE = 0,
	REMOTE_HID_JOB_FLAG_ALL,
	REMOTE_HID_JOB_SENS_AUTO_ALL,
	REMOTE_HID_JOB_TEST_ON_ALL,
	REMOTE_HID_JOB_TEST_OFF_ALL,
	REMOTE_HID_JOB_WOM_COMMAND,
};

static enum remote_hid_job remote_hid_job;
static uint8_t remote_hid_job_flag;
static uint8_t remote_hid_job_axis;
static uint16_t remote_hid_job_rev;
static uint16_t remote_hid_job_tps;

static void remote_hid_timeout_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(remote_hid_timeout_work, remote_hid_timeout_handler);
/* Runs on rssi_wq: the ~1s active-scan inside all-target jobs must not stall
 * the system workqueue; the single-threaded queue serializes jobs and scans. */
static void remote_hid_job_handler(struct k_work *work);
static K_WORK_DEFINE(remote_hid_job_work, remote_hid_job_handler);

static void remote_hid_send_completion(uint8_t status)
{
	if (!async_ack_fn) {
		return;
	}
	uint8_t ack[RCV_HID_CMD_LEN];
	fill_ack(ack, remote_hid_seq, remote_hid_opcode, status);
	LOG_INF("HID remote complete seq=%u opcode=0x%02X status=%u", remote_hid_seq, remote_hid_opcode,
		status);
	async_ack_fn(ack);
}

static void remote_hid_abort(void)
{
	atomic_set(&remote_hid_pending, 0);
	remote_hid_job = REMOTE_HID_JOB_NONE;
	(void)k_work_cancel_delayable(&remote_hid_timeout_work);
}

static void remote_hid_finish(uint8_t status)
{
	if (!atomic_cas(&remote_hid_pending, 1, 0)) {
		return;
	}
	(void)k_work_cancel_delayable(&remote_hid_timeout_work);
	remote_hid_job = REMOTE_HID_JOB_NONE;
	remote_hid_send_completion(status);
}

static void remote_hid_arm_expected(uint32_t mask)
{
	atomic_set(&remote_hid_confirmed, 0);
	atomic_set(&remote_hid_expected, (atomic_val_t)mask);
	if (mask == 0) {
		remote_hid_finish(RCV_HID_ST_ENOENT);
		return;
	}
	(void)k_work_reschedule(&remote_hid_timeout_work, K_MSEC(REMOTE_HID_CONFIRM_TIMEOUT_MS));
}

static void remote_wom_result(uint8_t id, uint16_t minutes, uint16_t error)
{
	if (remote_wom_console) {
		if (error == ETIMEDOUT) {
			printk("Tracker %u WOM command: timed out (offline or unsupported)\n", id);
		} else if (error) {
			printk("Tracker %u WOM command: error %u\n", id, error);
		} else if (minutes == ESB_WOM_ENTER) {
			printk("Tracker %u WOM entry accepted\n", id);
		} else if (minutes == 0) {
			printk("Tracker %u WOM normal timeout: restored firmware default\n", id);
		} else {
			printk("Tracker %u WOM normal timeout: %u minutes\n", id, minutes);
		}
	} else if (async_ack_fn) {
		uint8_t ack[RCV_HID_CMD_LEN];
		fill_ack(ack, remote_hid_seq, remote_hid_opcode, RCV_HID_ST_STARTED);
		ack[4] = RCV_HID_WOM_RESULT;
		ack[5] = id;
		sys_put_le16(error ? 0 : minutes, &ack[6]);
		sys_put_le16(error, &ack[8]);
		sys_put_le16(remote_wom_token, &ack[10]);
		async_ack_fn(ack);
	}
}

static void remote_wom_poll(void)
{
	uint32_t expected = (uint32_t)atomic_get(&remote_hid_expected);
	uint32_t confirmed = (uint32_t)atomic_get(&remote_hid_confirmed);
	bool expired = k_uptime_get() >= remote_wom_deadline;
	for (uint8_t id = 0; id < MAX_TRACKERS; id++) {
		uint32_t bit = BIT(id);
		if (!(expected & bit) || ((confirmed | remote_wom_failed) & bit)) {
			continue;
		}
		uint16_t minutes = 0, error = 0;
		if (!esb_get_remote_wom_result(id, remote_wom_token, &minutes, &error)) {
			if (!expired) {
				continue;
			}
			error = ETIMEDOUT;
		}
		if (error) {
			remote_wom_failed |= bit;
		} else {
			confirmed |= bit;
		}
		remote_wom_result(id, minutes, error);
	}
	atomic_set(&remote_hid_confirmed, confirmed);
	if ((confirmed | remote_wom_failed) != expected) {
		(void)k_work_reschedule(&remote_hid_timeout_work, K_MSEC(50));
		return;
	}
	esb_cancel_remote_wom_command(expected, remote_wom_token);
	if (remote_wom_console) {
		if (!expected) {
			printk("WOM command: no active trackers found\n");
		}
		printk("WOM command complete: success mask 0x%08X, failure mask 0x%08X\n",
		       confirmed, remote_wom_failed);
	} else if (async_ack_fn) {
		uint8_t ack[RCV_HID_CMD_LEN];
		fill_ack(ack, remote_hid_seq, remote_hid_opcode,
			 remote_wom_failed || !expected ? RCV_HID_ST_ENOENT : RCV_HID_ST_OK);
		ack[4] = RCV_HID_WOM_SUMMARY;
		sys_put_le16(remote_wom_token, &ack[5]);
		sys_put_le32(confirmed, &ack[7]);
		sys_put_le32(remote_wom_failed, &ack[11]);
		async_ack_fn(ack);
	}
	remote_hid_job = REMOTE_HID_JOB_NONE;
	atomic_set(&remote_hid_pending, 0);
}

static void remote_hid_timeout_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!atomic_get(&remote_hid_pending)) {
		return;
	}
	if (remote_hid_job == REMOTE_HID_JOB_WOM_COMMAND) {
		remote_wom_poll();
		return;
	}
	uint32_t expected = (uint32_t)atomic_get(&remote_hid_expected);
	uint32_t confirmed = (uint32_t)atomic_get(&remote_hid_confirmed);
	LOG_WRN("HID remote confirm timeout opcode=0x%02X confirmed=0x%08X expected=0x%08X",
		remote_hid_opcode, confirmed, expected);
	remote_hid_finish(RCV_HID_ST_ENOENT);
}

static void remote_hid_job_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!atomic_get(&remote_hid_pending)) {
		return;
	}

	uint32_t mask = 0;
	switch (remote_hid_job) {
	case REMOTE_HID_JOB_FLAG_ALL:
		mask = esb_send_remote_command_all(remote_hid_job_flag);
		break;
	case REMOTE_HID_JOB_SENS_AUTO_ALL:
		mask = esb_send_remote_command_sens_auto_all(remote_hid_job_axis, remote_hid_job_rev);
		break;
	case REMOTE_HID_JOB_TEST_ON_ALL:
		mask = esb_send_remote_command_test_on_all(remote_hid_job_tps);
		break;
	case REMOTE_HID_JOB_TEST_OFF_ALL:
		mask = esb_send_remote_command_test_off_all();
		break;
	case REMOTE_HID_JOB_WOM_COMMAND:
		mask = esb_send_remote_wom_command(RCV_HID_TARGET_ALL, remote_wom_minutes,
						   &remote_wom_token);
		atomic_set(&remote_hid_expected, mask);
		remote_wom_deadline = k_uptime_get() + REMOTE_HID_CONFIRM_TIMEOUT_MS;
		(void)k_work_reschedule(&remote_hid_timeout_work, K_MSEC(50));
		return;
	default:
		remote_hid_finish(RCV_HID_ST_EINVAL);
		return;
	}
	remote_hid_arm_expected(mask);
}

static void on_remote_confirm(uint8_t tracker_id, uint8_t flag)
{
	if (!atomic_get(&remote_hid_pending)) {
		return;
	}
	if (flag == ESB_PONG_FLAG_WOM_CONTROL) {
		return; /* A legacy flag echo is never a committed WOM result. */
	}
	if (flag != remote_hid_flag || tracker_id >= MAX_TRACKERS) {
		return;
	}

	uint32_t expected = (uint32_t)atomic_get(&remote_hid_expected);
	uint32_t bit = 1u << tracker_id;
	if (expected == 0 || (expected & bit) == 0) {
		return;
	}

	atomic_val_t old = atomic_or(&remote_hid_confirmed, (atomic_val_t)bit);
	uint32_t confirmed = (uint32_t)old | bit;
	if ((confirmed & expected) == expected) {
		remote_hid_finish(RCV_HID_ST_OK);
	}
}

static uint8_t remote_hid_begin(uint8_t seq, uint8_t opcode, uint8_t flag)
{
	if (!atomic_cas(&remote_hid_pending, 0, 1)) {
		return RCV_HID_ST_EBUSY;
	}
	remote_hid_seq = seq;
	remote_hid_opcode = opcode;
	remote_hid_flag = flag;
	atomic_set(&remote_hid_expected, 0);
	atomic_set(&remote_hid_confirmed, 0);
	return RCV_HID_ST_STARTED;
}

static uint8_t remote_wom_begin(uint8_t seq, uint8_t target, uint16_t minutes, bool console)
{
	if ((minutes > ESB_WOM_TIMEOUT_MAX_MINUTES &&
	     minutes != ESB_WOM_TIMEOUT_CLEAR && minutes != ESB_WOM_ENTER) ||
	    (target != RCV_HID_TARGET_ALL && target >= MAX_TRACKERS)) {
		return RCV_HID_ST_EINVAL;
	}
	uint8_t status = remote_hid_begin(seq, ESB_PONG_FLAG_WOM_CONTROL,
					ESB_PONG_FLAG_WOM_CONTROL);
	if (status != RCV_HID_ST_STARTED) {
		return status;
	}
	remote_hid_job = REMOTE_HID_JOB_WOM_COMMAND;
	remote_wom_console = console;
	remote_wom_failed = 0;
	remote_wom_token = 0;
	if (target == RCV_HID_TARGET_ALL) {
		remote_wom_minutes = minutes;
		k_work_submit_to_queue(&rssi_wq, &remote_hid_job_work);
		return RCV_HID_ST_STARTED;
	}
	uint32_t mask = esb_send_remote_wom_command(target, minutes, &remote_wom_token);
	if (!mask) {
		remote_hid_abort();
		return RCV_HID_ST_ENOENT;
	}
	atomic_set(&remote_hid_expected, mask);
	remote_wom_deadline = k_uptime_get() + REMOTE_HID_CONFIRM_TIMEOUT_MS;
	(void)k_work_reschedule(&remote_hid_timeout_work, K_MSEC(50));
	return RCV_HID_ST_STARTED;
}

uint8_t rcv_cmd_remote_wom_command(uint8_t target_id, uint16_t minutes)
{
	return remote_wom_begin(0, target_id, minutes, true);
}

static uint8_t rcv_cmd_remote_flag_hid(uint8_t seq, uint8_t opcode, uint8_t target_id, uint8_t pong_flag)
{
	if (!rcv_hid_opcode_is_pong_flag(pong_flag)) {
		return RCV_HID_ST_EINVAL;
	}
	if (pong_flag == ESB_PONG_FLAG_DATA_COLLECT_ON || pong_flag == ESB_PONG_FLAG_DATA_COLLECT_OFF ||
	    pong_flag == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON || pong_flag == ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF) {
		return rcv_cmd_remote_flag(target_id, pong_flag);
	}

	uint8_t st = remote_hid_begin(seq, opcode, pong_flag);
	if (st != RCV_HID_ST_STARTED) {
		return st;
	}

	if (target_id == RCV_HID_TARGET_ALL) {
		/* Defer 1s active-scan so HID IN FIFO keeps draining. */
		remote_hid_job = REMOTE_HID_JOB_FLAG_ALL;
		remote_hid_job_flag = pong_flag;
		k_work_submit_to_queue(&rssi_wq, &remote_hid_job_work);
		return RCV_HID_ST_STARTED;
	}
	if (target_id >= MAX_TRACKERS) {
		remote_hid_abort();
		return RCV_HID_ST_EINVAL;
	}

	/* Arm before queue so a fast confirm cannot race expected==0. */
	remote_hid_arm_expected(1u << target_id);
	esb_send_remote_command(target_id, pong_flag);
	return RCV_HID_ST_STARTED;
}

static uint8_t rcv_cmd_remote_sens_set_hid(uint8_t seq, uint8_t opcode, uint8_t target_id, float x,
					  float y, float z)
{
	uint8_t st = remote_hid_begin(seq, opcode, ESB_PONG_FLAG_SENS_SET);
	if (st != RCV_HID_ST_STARTED) {
		return st;
	}

	if (target_id == RCV_HID_TARGET_ALL) {
		uint32_t mask = 0;
		for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; i++) {
			mask |= (1u << i);
		}
		remote_hid_arm_expected(mask);
		if (mask == 0) {
			return RCV_HID_ST_STARTED;
		}
		for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; i++) {
			esb_send_remote_command_sens(i, x, y, z);
		}
		return RCV_HID_ST_STARTED;
	}
	if (target_id >= MAX_TRACKERS) {
		remote_hid_abort();
		return RCV_HID_ST_EINVAL;
	}
	remote_hid_arm_expected(1u << target_id);
	esb_send_remote_command_sens(target_id, x, y, z);
	return RCV_HID_ST_STARTED;
}

static uint8_t rcv_cmd_remote_sens_auto_hid(uint8_t seq, uint8_t opcode, uint8_t target_id,
					   uint8_t axis, uint16_t revolutions)
{
	if (axis > 2) {
		return RCV_HID_ST_EINVAL;
	}

	uint8_t st = remote_hid_begin(seq, opcode, ESB_PONG_FLAG_SENS_AUTO);
	if (st != RCV_HID_ST_STARTED) {
		return st;
	}

	if (target_id == RCV_HID_TARGET_ALL) {
		remote_hid_job = REMOTE_HID_JOB_SENS_AUTO_ALL;
		remote_hid_job_axis = axis;
		remote_hid_job_rev = revolutions;
		k_work_submit_to_queue(&rssi_wq, &remote_hid_job_work);
		return RCV_HID_ST_STARTED;
	}
	if (target_id >= MAX_TRACKERS) {
		remote_hid_abort();
		return RCV_HID_ST_EINVAL;
	}
	remote_hid_arm_expected(1u << target_id);
	if (!esb_send_remote_command_sens_auto(target_id, axis, revolutions)) {
		remote_hid_abort();
		return RCV_HID_ST_EINVAL;
	}
	return RCV_HID_ST_STARTED;
}

static uint8_t rcv_cmd_remote_test_on_hid(uint8_t seq, uint8_t opcode, uint8_t target_id, uint16_t tps)
{
	if (tps > 1000) {
		return RCV_HID_ST_EINVAL;
	}
	uint8_t st = remote_hid_begin(seq, opcode, ESB_PONG_FLAG_TEST_MODE_ON);
	if (st != RCV_HID_ST_STARTED) {
		return st;
	}
	if (target_id == RCV_HID_TARGET_ALL) {
		/* Defer 1s active-scan so HID IN FIFO keeps draining. */
		remote_hid_job = REMOTE_HID_JOB_TEST_ON_ALL;
		remote_hid_job_tps = tps;
		k_work_submit_to_queue(&rssi_wq, &remote_hid_job_work);
		return RCV_HID_ST_STARTED;
	}
	if (target_id >= MAX_TRACKERS) {
		remote_hid_abort();
		return RCV_HID_ST_EINVAL;
	}
	/* Arm before queue so a fast confirm cannot race expected==0. */
	remote_hid_arm_expected(1u << target_id);
	esb_send_remote_command_test_on((uint8_t)target_id, tps);
	return RCV_HID_ST_STARTED;
}

static uint8_t rcv_cmd_remote_test_off_hid(uint8_t seq, uint8_t opcode, uint8_t target_id)
{
	uint8_t st = remote_hid_begin(seq, opcode, ESB_PONG_FLAG_TEST_MODE_OFF);
	if (st != RCV_HID_ST_STARTED) {
		return st;
	}
	if (target_id == RCV_HID_TARGET_ALL) {
		remote_hid_job = REMOTE_HID_JOB_TEST_OFF_ALL;
		k_work_submit_to_queue(&rssi_wq, &remote_hid_job_work);
		return RCV_HID_ST_STARTED;
	}
	if (target_id >= MAX_TRACKERS) {
		remote_hid_abort();
		return RCV_HID_ST_EINVAL;
	}
	remote_hid_arm_expected(1u << target_id);
	esb_send_remote_command_test_off((uint8_t)target_id);
	return RCV_HID_ST_STARTED;
}

static void arm_ch_hid_completion(uint8_t seq, uint8_t opcode)
{
	ch_hid_seq = seq;
	ch_hid_opcode = opcode;
	atomic_set(&ch_hid_pending, 1);
}

static void on_tracker_channel_done(bool success)
{
	k_mutex_lock(&channel_completion_lock, K_FOREVER);
	if (atomic_cas(&ch_hid_pending, 1, 0) && async_ack_fn) {
		uint8_t ack[RCV_HID_CMD_LEN];
		fill_ack(ack, ch_hid_seq, ch_hid_opcode, success ? RCV_HID_ST_OK : RCV_HID_ST_ENOENT);
		async_ack_fn(ack);
	}
	k_mutex_unlock(&channel_completion_lock);
}

static int map_esb_channel_err(int err)
{
	if (err == 0) {
		return RCV_HID_ST_STARTED;
	}
	if (err == -EBUSY) {
		return RCV_HID_ST_EBUSY;
	}
	return RCV_HID_ST_EINVAL;
}

uint8_t rcv_cmd_tracker_channel_all(uint8_t channel)
{
	if (channel > 100) {
		return RCV_HID_ST_EINVAL;
	}
	return (uint8_t)map_esb_channel_err(esb_set_all_trackers_channel(channel));
}

uint8_t rcv_cmd_tracker_channel_clear_all(void)
{
	return (uint8_t)map_esb_channel_err(esb_clear_all_trackers_channel());
}

uint8_t rcv_cmd_remote_test_on(uint8_t target_id, uint16_t tps)
{
	if (tps > 1000) {
		return RCV_HID_ST_EINVAL;
	}
	if (target_id == RCV_HID_TARGET_ALL) {
		uint32_t mask = esb_send_remote_command_test_on_all(tps);
		return mask != 0 ? RCV_HID_ST_STARTED : RCV_HID_ST_ENOENT;
	}
	if (target_id >= MAX_TRACKERS) {
		return RCV_HID_ST_EINVAL;
	}
	esb_send_remote_command_test_on((uint8_t)target_id, tps);
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_remote_test_off(uint8_t target_id)
{
	if (target_id == RCV_HID_TARGET_ALL) {
		uint32_t mask = esb_send_remote_command_test_off_all();
		return mask != 0 ? RCV_HID_ST_STARTED : RCV_HID_ST_ENOENT;
	}
	if (target_id >= MAX_TRACKERS) {
		return RCV_HID_ST_EINVAL;
	}
	esb_send_remote_command_test_off((uint8_t)target_id);
	return RCV_HID_ST_OK;
}

uint8_t rcv_cmd_remote_flag(uint8_t target_id, uint8_t pong_flag)
{
	if (!rcv_hid_opcode_is_pong_flag(pong_flag)) {
		return RCV_HID_ST_EINVAL;
	}
	if (pong_flag == ESB_PONG_FLAG_DATA_COLLECT_ON) {
		return rcv_cmd_collect_start(target_id);
	}
	if (pong_flag == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON) {
		/* Batch admission needs its explicit rate-bearing API. */
		return RCV_HID_ST_EINVAL;
	}
	if (pong_flag == ESB_PONG_FLAG_DATA_COLLECT_OFF) {
		return rcv_cmd_collect_stop();
	}
	if (pong_flag == ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF) {
		return rcv_cmd_collect_batch_stop();
	}
	if (pong_flag == ESB_PONG_FLAG_SET_CHANNEL || pong_flag == ESB_PONG_FLAG_CLEAR_CHANNEL ||
	    pong_flag == ESB_PONG_FLAG_WOM_CONTROL ||
	    pong_flag == ESB_PONG_FLAG_TEST_MODE_ON || pong_flag == ESB_PONG_FLAG_TEST_MODE_OFF) {
		/* Test flags carry TPS state; the generic path would keep a stale
		 * rate. Callers must use rcv_cmd_remote_test_on/off. */
		return RCV_HID_ST_EINVAL;
	}
	if (target_id == RCV_HID_TARGET_ALL) {
		(void)esb_send_remote_command_all(pong_flag);
		return RCV_HID_ST_QUEUED;
	}
	if (target_id >= MAX_TRACKERS) {
		return RCV_HID_ST_EINVAL;
	}
	esb_send_remote_command(target_id, pong_flag);
	return RCV_HID_ST_QUEUED;
}

uint8_t rcv_cmd_remote_sens_set(uint8_t target_id, float x, float y, float z)
{
	if (target_id == RCV_HID_TARGET_ALL) {
		for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; i++) {
			esb_send_remote_command_sens(i, x, y, z);
		}
		return RCV_HID_ST_QUEUED;
	}
	if (target_id >= MAX_TRACKERS) {
		return RCV_HID_ST_EINVAL;
	}
	esb_send_remote_command_sens(target_id, x, y, z);
	return RCV_HID_ST_QUEUED;
}

uint8_t rcv_cmd_remote_sens_auto(uint8_t target_id, uint8_t axis, uint16_t revolutions)
{
	if (axis > 2) {
		return RCV_HID_ST_EINVAL;
	}
	if (target_id == RCV_HID_TARGET_ALL) {
		(void)esb_send_remote_command_sens_auto_all(axis, revolutions);
		return RCV_HID_ST_QUEUED;
	}
	if (target_id >= MAX_TRACKERS) {
		return RCV_HID_ST_EINVAL;
	}
	if (!esb_send_remote_command_sens_auto(target_id, axis, revolutions)) {
		return RCV_HID_ST_EINVAL;
	}
	return RCV_HID_ST_QUEUED;
}

static void fill_ack(uint8_t ack[RCV_HID_CMD_LEN], uint8_t seq, uint8_t opcode, uint8_t status)
{
	memset(ack, 0, RCV_HID_CMD_LEN);
	ack[0] = RCV_HID_TYPE_CMD_ACK;
	ack[1] = seq;
	ack[2] = opcode;
	ack[3] = status;
}

static int rcv_cmd_register_esb_cbs(void)
{
	esb_set_channel_change_done_cb(on_tracker_channel_done);
	esb_set_remote_confirm_cb(on_remote_confirm);
	return 0;
}

SYS_INIT(rcv_cmd_register_esb_cbs, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

bool rcv_cmd_process_hid(const uint8_t *buf, size_t len, uint32_t usb_generation,
			 uint8_t ack_out[RCV_HID_CMD_LEN])
{
	if (buf == NULL || ack_out == NULL || len < 4 || buf[0] != RCV_HID_TYPE_CMD) {
		return false;
	}

	uint8_t seq = buf[1];
	uint8_t opcode = buf[2];
	const uint8_t *args = &buf[4];
	size_t args_len = (len > 4) ? (len - 4) : 0;
	if (args_len > 12) {
		args_len = 12;
	}

	if (opcode == RCV_HID_OP_TRACKER_EVENTS) {
		fill_ack(ack_out, seq, opcode, RCV_HID_ST_EINVAL);
		if (buf[3] != 0) {
			return true;
		}
		/* Core checks USB generation under the same lock as lease mutation. */
		int err = tracker_events_control(args, args_len, usb_generation, &ack_out[4]);
		switch (err) {
		case 0:
			ack_out[3] = RCV_HID_ST_OK;
			break;
		case -ENOTSUP:
			ack_out[3] = RCV_HID_ST_ENOTSUP;
			break;
		case -ENOENT:
		case -ENOTCONN:
			ack_out[3] = RCV_HID_ST_ENOENT;
			break;
		default:
			ack_out[3] = RCV_HID_ST_EINVAL;
			break;
		}
		return true;
	}

	LOG_INF("HID cmd seq=%u opcode=0x%02X args_len=%u", seq, opcode, (unsigned)args_len);

	uint8_t status = RCV_HID_ST_EINVAL;
	int8_t slot = -1;

	if (rcv_hid_opcode_is_pong_flag(opcode)) {
		uint8_t target = (args_len >= 1) ? args[0] : RCV_HID_TARGET_ALL;
		if (opcode == ESB_PONG_FLAG_WOM_CONTROL) {
			bool valid = args_len >= 3 && buf[3] == 0;
			for (size_t i = 3; i < args_len; i++) {
				valid = valid && args[i] == 0;
			}
			status = valid ? remote_wom_begin(seq, target, sys_get_le16(&args[1]), false)
				       : RCV_HID_ST_EINVAL;
		} else if (opcode == ESB_PONG_FLAG_SENS_SET) {
			if (args_len < 7) {
				status = RCV_HID_ST_EINVAL;
			} else {
				int16_t xi = (int16_t)sys_get_le16(&args[1]);
				int16_t yi = (int16_t)sys_get_le16(&args[3]);
				int16_t zi = (int16_t)sys_get_le16(&args[5]);
				status = rcv_cmd_remote_sens_set_hid(seq, opcode, target, xi / 100.0f,
								     yi / 100.0f, zi / 100.0f);
			}
		} else if (opcode == ESB_PONG_FLAG_SENS_AUTO) {
			if (args_len < 4) {
				status = RCV_HID_ST_EINVAL;
			} else {
				uint8_t axis = args[1];
				uint16_t rev = sys_get_le16(&args[2]);
				status = rcv_cmd_remote_sens_auto_hid(seq, opcode, target, axis, rev);
			}
		} else if (opcode == ESB_PONG_FLAG_TEST_MODE_ON) {
			/* args[0] target; optional little-endian uint16 TPS in args[1:3].
			 * Absent (<=1 arg) = 0 (built-in default); truncated (2 args) is
			 * rejected. */
			uint16_t tps = 0;
			if (args_len == 2) {
				status = RCV_HID_ST_EINVAL;
			} else {
				if (args_len >= 3) {
					tps = sys_get_le16(&args[1]);
				}
				status = rcv_cmd_remote_test_on_hid(seq, opcode, target, tps);
			}
		} else if (opcode == ESB_PONG_FLAG_TEST_MODE_OFF) {
			status = rcv_cmd_remote_test_off_hid(seq, opcode, target);
		} else if (opcode == ESB_PONG_FLAG_SET_CHANNEL ||
			   opcode == ESB_PONG_FLAG_CLEAR_CHANNEL) {
			status = RCV_HID_ST_EINVAL;
		} else {
			status = rcv_cmd_remote_flag_hid(seq, opcode, target, opcode);
		}
	} else {
		switch (opcode) {
		case RCV_HID_OP_TRACKER_EVENT:
		case RCV_HID_OP_TRACKER_OBSERVATION:
			/* Notification opcodes have no host-to-device operation. */
			status = RCV_HID_ST_EINVAL;
			break;
		case RCV_HID_OP_NOP:
			status = RCV_HID_ST_OK;
			break;
		case RCV_HID_OP_PAIR:
			status = rcv_cmd_pair((args_len >= 1) ? args[0] : 0);
			break;
		case RCV_HID_OP_EXIT_PAIR:
			status = rcv_cmd_exit_pair();
			break;
		case RCV_HID_OP_CLEAR:
			status = rcv_cmd_clear();
			break;
		case RCV_HID_OP_ADD: {
			if (args_len < 6) {
				status = RCV_HID_ST_EINVAL;
				break;
			}
			uint64_t addr = 0;
			memcpy(&addr, args, 6);
			status = rcv_cmd_add(addr, &slot);
			break;
		}
		case RCV_HID_OP_REMOVE:
			status = rcv_cmd_remove();
			break;
		case RCV_HID_OP_LIST:
			status = rcv_cmd_list();
			break;
		case RCV_HID_OP_CHANNEL_SET:
			status = rcv_cmd_channel_set((args_len >= 1) ? args[0] : 0);
			break;
		case RCV_HID_OP_CHANNEL_CLEAR:
			status = rcv_cmd_channel_clear();
			break;
		case RCV_HID_OP_RSSI_SCAN:
			status = rcv_cmd_rssi_scan();
			break;
		case RCV_HID_OP_INFO:
			status = rcv_cmd_info();
			break;
		case RCV_HID_OP_UPTIME:
			status = rcv_cmd_uptime();
			break;
		case RCV_HID_OP_STATS: {
			uint32_t dur = 0;
			if (args_len >= 4) {
				dur = sys_get_le32(args);
			}
			status = rcv_cmd_stats(dur);
			break;
		}
		case RCV_HID_OP_RESETSTATS:
			status = rcv_cmd_resetstats();
			break;
		case RCV_HID_OP_COLLECT_START:
			status = rcv_cmd_collect_start((args_len >= 1) ? args[0] : 0);
			break;
		case RCV_HID_OP_COLLECT_STOP:
			status = rcv_cmd_collect_stop();
			break;
		case RCV_HID_OP_COLLECT_BATCH_START:
			if (args_len < 2) {
				status = RCV_HID_ST_EINVAL;
			} else {
				status = rcv_cmd_collect_batch_start(sys_get_le16(args));
			}
			break;
		case RCV_HID_OP_COLLECT_META:
			if (args_len < 3) {
				status = RCV_HID_ST_EINVAL;
			} else {
				status = rcv_cmd_collect_meta(args[0], args[1], args[2]);
			}
			break;
		case RCV_HID_OP_COLLECT_BATCH_STOP:
			status = rcv_cmd_collect_batch_stop();
			break;
		case RCV_HID_OP_REBOOT:
			status = rcv_cmd_reboot();
			break;
		case RCV_HID_OP_DFU:
			status = rcv_cmd_dfu((args_len >= 1) && args[0] != 0);
			break;
		case RCV_HID_OP_TRACKER_CH_ALL:
			k_mutex_lock(&channel_completion_lock, K_FOREVER);
			status = rcv_cmd_tracker_channel_all((args_len >= 1) ? args[0] : 0);
			if (status == RCV_HID_ST_STARTED) {
				arm_ch_hid_completion(seq, opcode);
			}
			k_mutex_unlock(&channel_completion_lock);
			break;
		case RCV_HID_OP_TRACKER_CH_CLR:
			k_mutex_lock(&channel_completion_lock, K_FOREVER);
			status = rcv_cmd_tracker_channel_clear_all();
			if (status == RCV_HID_ST_STARTED) {
				arm_ch_hid_completion(seq, opcode);
			}
			k_mutex_unlock(&channel_completion_lock);
			break;
		default:
			status = RCV_HID_ST_EINVAL;
			break;
		}
	}

	fill_ack(ack_out, seq, opcode, status);
	if (opcode == RCV_HID_OP_ADD && status == RCV_HID_ST_OK && slot >= 0) {
		ack_out[4] = (uint8_t)slot;
	}
	if (opcode == RCV_HID_OP_STATS && status == RCV_HID_ST_OK) {
		ack_out[4] = esb_get_stats_detailed_enabled() ? 1 : 0;
	}
	LOG_INF("HID ACK seq=%u opcode=0x%02X status=%u", seq, opcode, status);
	return true;
}
