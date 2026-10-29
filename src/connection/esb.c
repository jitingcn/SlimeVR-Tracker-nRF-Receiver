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
#include "esb.h"

#include <errno.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#include "globals.h"
#include "hid.h"
#include "system/system.h"
#include "data_collect.h"
#include "esb_ota.h"
#include "receiver_ota.h"
#include "connection/auto_channel.h"
#include "tracker_events.h"
#include "tracker_event_protocol.h"
#include "connection/tracker_roster.h"

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

//|type    |description
//|RX  CRC8|pairing
//|TX  CRC8|pairing

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12 |b13
//|b14     |b15     | |type    |data | |RX  CRC8|ack     |device_addr                                          |- |TX
// CRC8|ack     |recv_addr                                            |-

//|packet  |description
//|RX     1|request from tracker
//|TX     2|pairing accepted from dongle
//|TX     3|Dongle State
//|TX     4|No Windows
//|TX     5|Window Info

//|packet  |b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11 |b12
//|b13     |b14     |b15     | |RX     1|    0xCD|    0x01|    0x00|Tracker Hardware ID |Tracker Hardware ID |- |TX 2|
// 0xCD|    0x02|Trckr ID|Dongle Hardware ID                                   |Tracker Hardware ID |- |TX     3| 0xCD|
// 0x03|Dongle Hardware ID                                   |state   |channel |- |TX     4|    0xCD|    0x04|- |TX 5|
// 0xCD|    0x05|Window  |Timer                              |Packet  |-

// packet 3:
// state field bits: 9[0:0]: Accepts new trackers?; 9[1:1]: Force pair
// channel bundle field bits: 10[0:3]: Channels bundle; 10[4:7]: Next channel offset
// packet 5:
// Packet: Packet Number

#define RADIO_RETRANSMIT_DELAY CONFIG_RADIO_RETRANSMIT_DELAY
#define RADIO_RF_CHANNEL CONFIG_RADIO_RF_CHANNEL
#define PAIRING_TIMEOUT_SECONDS CONFIG_PAIRING_TIMEOUT
/* Preferred ESB channels (upstream SlimeVR selection): even channels outside
 * WiFi/BT-heavy spectrum. Reference for future auto channel-picking. */
static const uint8_t esb_allowed_channels[] = {
	0,  2,  52, 72, 74, 76, 78, 82, 84, 86, 88, 50, 24, 48, 70, 68, 46, 44, 20, 54, 56,
	28, 30, 6,  8,  10, 12, 14, 16, 18, 32, 34, 36, 38, 40, 42, 58, 60, 62, 64, 66,
};

bool esb_channel_is_allowed(uint8_t channel)
{
	for (size_t i = 0; i < ARRAY_SIZE(esb_allowed_channels); i++) {
		if (esb_allowed_channels[i] == channel) {
			return true;
		}
	}
	return false;
}

// Guarded-PING TDMA: every active tracker owns one base slot per frame. A
// synchronized PING uses its own slot plus the following slot; every tracker
// derives and observes the same guard schedule.
//
// ping_guard_v2: the slot width is adaptive instead of fixed 16 ticks. With
// few trackers the per-frame opportunity rate (32768/(sticks*N)) far exceeds
// the ~220 TPS of useful demand, so slots are widened for admission-window
// robustness; the loss controller below trades rate for stability by stepping
// the per-tracker opportunity cap down a ladder (never below ~115 TPS).
#define TDMA_ENABLED 1
#define TDMA_BASE_SLOT_TICKS 16
#define TDMA_SLOT_TICKS_MAX_LEVEL0 64
#define TDMA_TOLERANCE_TICKS 3
#define TDMA_SLOT_ROTATION 0
#define TDMA_RECONFIG_MIN_MS 1000 /* minimum interval between ordinary membership layouts */
/* Loss ladder: sustained aggregate sequence-gap loss steps the opportunity
 * cap down; sustained clean loss steps back up. Values are opportunities/s
 * per tracker; sticks is the widest even width keeping opp >= cap. */
static const uint16_t tdma_cap_ladder[] = {220, 180, 150, 130, 115};
#define TDMA_CAP_LEVEL_MAX (ARRAY_SIZE(tdma_cap_ladder) - 1U)
#define TDMA_LOSS_TRIGGER_PERMILLE 200
#define TDMA_LOSS_TRIGGER_TICKS 5      /* 5 qualified, nonempty stats ticks */
#define TDMA_LOSS_RECOVER_PERMILLE 50
#define TDMA_LOSS_RECOVER_TICKS 10
#define TDMA_LOSS_PROBE_PERMILLE 100
#define TDMA_LOSS_PROBE_TICKS 60
#define TDMA_LOSS_MIN_SAMPLES 100      /* pool sparse ticks until enough evidence */
#define TDMA_LOSS_RECONFIG_MIN_MS 15000 /* min spacing between ladder steps */
#define TDMA_SYNC_EXTRAP_MAX_TICKS (10u * 32768u) /* skip frozen-skew extrapolation beyond this PING age */
#define PING_CLEAN_REACQUIRE_STREAK 8u            /* consecutive dirty PINGs before re-baselining */

// Dynamic TDMA config packed into uint32_t for atomic ISR access (ARM Cortex-M).
// Layout: [31:24]=assigned_slot, [23:16]=total_slots, [15:8]=slot_ticks, [7:0]=epoch
static volatile uint32_t tdma_config_packed[MAX_TRACKERS];
static uint8_t tdma_config_epoch;         // wrapping counter, incremented on each recalc
static uint8_t tdma_dynamic_active_count; // current number of active trackers
static uint8_t tdma_dynamic_slot_ticks;   // current slot width in ticks
static uint32_t tdma_active_mask;         // bitmask of active trackers (for change detection)
static int64_t tdma_last_reconfig_time;
static bool tdma_last_reconfig_valid;
/* Loss-controller state, evaluated once per 1 s stats tick. */
static uint8_t tdma_cap_level;           /* index into tdma_cap_ladder */
static uint8_t tdma_published_cap_level; /* level in effect in the live layout */
static uint8_t tdma_loss_trigger_streak_ticks;
static uint8_t tdma_loss_recover_streak_ticks;
static uint8_t tdma_loss_probe_streak_ticks;
/* Samples determine when to judge a pool; its nonempty 1 s ticks credit streaks. */
static uint32_t tdma_loss_pool_received;
static uint32_t tdma_loss_pool_gaps;
static uint8_t tdma_loss_pool_ticks;
#if defined(CONFIG_TDMA_DIAGNOSTICS)
static int64_t tdma_last_loss_step_ms;   /* last applied ladder step (log only) */
static uint32_t tdma_loss_last_permille; /* last judged window (log only) */
#endif
static uint32_t tdma_prev_recv[MAX_TRACKERS];
static uint32_t tdma_prev_gaps[MAX_TRACKERS];
static uint32_t tdma_prev_restarts[MAX_TRACKERS];
static atomic_t tdma_rx_discontinuity_pending;
static bool tdma_decision_ready;

static void tdma_loss_reset_windows(void)
{
	tdma_loss_trigger_streak_ticks = 0;
	tdma_loss_recover_streak_ticks = 0;
	tdma_loss_probe_streak_ticks = 0;
	tdma_loss_pool_received = 0;
	tdma_loss_pool_gaps = 0;
	tdma_loss_pool_ticks = 0;
}


static inline uint32_t tdma_pack_config(uint8_t slot, uint8_t total, uint8_t slot_ticks, uint8_t epoch)
{
	return ((uint32_t)slot << 24) | ((uint32_t)total << 16) | ((uint32_t)slot_ticks << 8) | (uint32_t)epoch;
}

/* Forward declaration — defined after last_ping_time / PING_TIMEOUT_MS */
static void tdma_recalculate(bool forced);
static void tdma_stats_reset(void);
static void tdma_sync_stats_reset(void);

static void tdma_loss_controller_tick(int64_t now);
static struct esb_payload rx_payload;
static atomic_t home_beacon_tx_result;

struct pairing_event {
	uint8_t packet[8];
};

// Queue pairing packets from the ISR context to the pairing worker thread.
K_MSGQ_DEFINE(esb_pairing_msgq, sizeof(struct pairing_event), 8, 4);

static K_MUTEX_DEFINE(tracker_store_lock);
static bool tracker_store_ready;

static uint8_t last_packet_sequence[MAX_TRACKERS];    // Track the last packet sequence for each tracker
static uint8_t last_ping_counter[MAX_TRACKERS] = {0}; // Track the last PING counter for each tracker
static bool ping_counter_initialized[MAX_TRACKERS]
	= {false};                                      // Flag indicating if a PING has been received from this tracker
static uint64_t last_ping_time[MAX_TRACKERS] = {0}; // Track the last time a PING was received from each tracker
static uint8_t last_pong_queued_counter[MAX_TRACKERS] = {0}; // Track the last PONG counter enqueued for each tracker
static uint8_t pipe_mismatch_count[MAX_TRACKERS];
static uint8_t crc_error_count[MAX_TRACKERS];
static bool sequence_initialized[MAX_TRACKERS]; // Ordinary-data sequence anchor validity
static uint16_t last_raw_seq[MAX_TRACKERS] = {0};
static bool last_raw_valid[MAX_TRACKERS] = {false};
// Shared ACK tuples are published under irq_lock, including their command flag.
// Volatile alone does not make a multi-field command atomic.
static volatile uint8_t tracker_remote_command[MAX_TRACKERS]; // Command flag for next PONG
static volatile uint8_t pending_cmd_arg[MAX_TRACKERS];       // Optional byte carried in PONG data[8]
static volatile uint16_t tracker_test_on_tps[MAX_TRACKERS];   // Optional TPS carried by TEST_MODE_ON
static volatile int16_t pending_sens_data[MAX_TRACKERS][3];   // SENS_SET sensitivity data
static volatile uint8_t pending_sens_auto_axis[MAX_TRACKERS];
static volatile uint16_t pending_sens_auto_revolutions[MAX_TRACKERS];
/* Metadata repair requests are lower priority than any existing PONG command.
 * Active and queued request fields are separate so a retry gets a fresh token
 * without replacing collection/control commands already in flight. */
static volatile uint8_t metadata_active_mask[MAX_TRACKERS];
static volatile uint8_t metadata_active_chunk[MAX_TRACKERS];
static volatile uint16_t metadata_active_token[MAX_TRACKERS];
static volatile uint8_t metadata_pending_mask[MAX_TRACKERS];
static volatile uint8_t metadata_pending_chunk[MAX_TRACKERS];
static volatile uint16_t metadata_pending_token[MAX_TRACKERS];
static uint16_t metadata_next_token;
/* Tokens survive slot cleanup so late replies cannot confirm a replacement. */
static uint16_t wom_next_token;
static uint16_t wom_token[MAX_TRACKERS];
static uint16_t wom_minutes[MAX_TRACKERS];
static uint16_t wom_error[MAX_TRACKERS];
static bool wom_complete[MAX_TRACKERS];
static bool wom_advertised[MAX_TRACKERS];
static uint8_t wom_first_ping_counter[MAX_TRACKERS];
/* Sticky state for commands addressed to "all": newly active trackers must
 * converge to the same test on/off state. Confirmed means the tracker has
 * executed and acknowledged the current desired state. */
static atomic_t test_all_state_valid;
static atomic_t test_all_enabled;
static atomic_t test_all_broadcasting;
static atomic_t test_all_confirmed_mask;
static volatile uint16_t test_all_requested_tps;
static volatile uint16_t test_all_effective_tps;
static atomic_t test_all_ready_after_ms[MAX_TRACKERS];
static atomic_t test_all_generation; /* bumped by every test command; stale broadcasts abort */
static K_MUTEX_DEFINE(test_all_mutex); /* serializes whole all-target helper bodies */
static uint8_t receiver_rf_channel = 0xFF; // Current RF channel of the receiver, 0xFF indicates using default value

#define PING_TIMEOUT_MS 5000               // PING timeout threshold: 5 seconds
#define REMOTE_COMMAND_ACTIVE_SCAN_MS 1000 // Time window to detect trackers actively sending data

/* Valid PING/data evidence updates the desired membership immediately. The
 * published layout waits for a fixed first-evidence cohort and spacing. */
#define TDMA_SHADOW_FRESH_MS (PING_TIMEOUT_MS + 1000)
#define TDMA_SHADOW_LEAVE_GRACE_MS 15000
#define TDMA_MEMBERSHIP_COHORT_MS 101U

enum tdma_shadow_evidence {
	TDMA_SHADOW_EVIDENCE_PING,
	TDMA_SHADOW_EVIDENCE_DATA,
};

static atomic_t tdma_shadow_last_seen_ms[MAX_TRACKERS];
static atomic_t tdma_shadow_ever_ping_mask = ATOMIC_INIT(0);
static atomic_t tdma_shadow_ever_data_mask = ATOMIC_INIT(0);
static atomic_t tdma_shadow_live_evidence_mask = ATOMIC_INIT(0);
static atomic_t tdma_shadow_desired_mask = ATOMIC_INIT(0);
/* Evidence awaiting the owner's merge, not a join-confirmation stopwatch. */
static atomic_t tdma_shadow_pending_join_mask = ATOMIC_INIT(0);
static atomic_t tdma_shadow_pending_leave_mask = ATOMIC_INIT(0);
static atomic_t tdma_shadow_desired_changed_at_ms = ATOMIC_INIT(0);

/* Protected by irq_lock; the semaphore is only a coalesced notification. */
static K_SEM_DEFINE(tdma_membership_wake, 0, 1);
static uint32_t tdma_membership_deadline_ms;
static bool tdma_membership_deadline_armed;

static void tdma_shadow_observe(uint8_t tracker_id, enum tdma_shadow_evidence evidence)
{
	bool wake = false;
	unsigned int key = irq_lock();
	if (tracker_id < stored_trackers && tracker_id < MAX_TRACKERS) {
		uint32_t now_ms = k_uptime_get_32();
		uint32_t bit = BIT(tracker_id);
		/* Validity lives in the evidence mask: every timestamp is valid. */
		atomic_set(&tdma_shadow_last_seen_ms[tracker_id], (atomic_val_t)now_ms);
		if (evidence == TDMA_SHADOW_EVIDENCE_PING) {
			atomic_or(&tdma_shadow_ever_ping_mask, bit);
		} else {
			atomic_or(&tdma_shadow_ever_data_mask, bit);
		}
		atomic_or(&tdma_shadow_live_evidence_mask, bit);
		if (!((uint32_t)atomic_get(&tdma_shadow_desired_mask) & bit)) {
			uint32_t queued = (uint32_t)atomic_get(&tdma_shadow_pending_join_mask);
			atomic_or(&tdma_shadow_pending_join_mask, bit);
			if (!tdma_membership_deadline_armed) {
				tdma_membership_deadline_ms = now_ms + TDMA_MEMBERSHIP_COHORT_MS;
				tdma_membership_deadline_armed = true;
			}
			wake = !(queued & bit);
		}
	}
	irq_unlock(key);
	if (wake) {
		k_sem_give(&tdma_membership_wake);
	}
}

/* Caller holds irq_lock, including when a stored slot is reset. */
static void tdma_shadow_forget_tracker(uint8_t tracker_id)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}

	uint32_t bit = BIT(tracker_id);
	atomic_set(&tdma_shadow_last_seen_ms[tracker_id], 0);
	atomic_and(&tdma_shadow_ever_ping_mask, (atomic_val_t)~bit);
	atomic_and(&tdma_shadow_ever_data_mask, (atomic_val_t)~bit);
	atomic_and(&tdma_shadow_live_evidence_mask, (atomic_val_t)~bit);
	atomic_and(&tdma_shadow_desired_mask, (atomic_val_t)~bit);
	atomic_and(&tdma_shadow_pending_join_mask, (atomic_val_t)~bit);
	atomic_and(&tdma_shadow_pending_leave_mask, (atomic_val_t)~bit);
	if ((uint32_t)atomic_get(&tdma_shadow_desired_mask) == tdma_active_mask
	    && !atomic_get(&tdma_shadow_pending_join_mask)) {
		tdma_membership_deadline_armed = false;
	}
}

/* Caller holds irq_lock. A publication resamples here under the same lock as
 * its final tuple snapshot, so a later IRQ starts a separate batch. */
static uint32_t tdma_shadow_merge_locked(uint32_t now_ms)
{
	uint8_t tracker_count = MIN(stored_trackers, MAX_TRACKERS);
	uint32_t stored_mask = tracker_count > 0 ? BIT(tracker_count) - 1U : 0;
	uint32_t evidence_mask = (uint32_t)atomic_get(&tdma_shadow_live_evidence_mask) & stored_mask;
	uint32_t desired_mask = (uint32_t)atomic_get(&tdma_shadow_desired_mask) & stored_mask;
	uint32_t pending_leave_mask = 0;
	uint32_t evicted_mask = 0;

	for (uint8_t i = 0; i < tracker_count; i++) {
		uint32_t bit = BIT(i);
		uint32_t last_seen_ms = (uint32_t)atomic_get(&tdma_shadow_last_seen_ms[i]);
		bool has_evidence = (evidence_mask & bit) != 0;
		/* Modular age handles wrap; a concurrently sampled future tick is fresh. */
		int32_t signed_age_ms = (int32_t)(now_ms - last_seen_ms);
		uint32_t age_ms = has_evidence ? (uint32_t)MAX(signed_age_ms, 0) : UINT32_MAX;

		if (desired_mask & bit) {
			if (!(tdma_active_mask & bit) && age_ms >= TDMA_SHADOW_FRESH_MS) {
				desired_mask &= ~bit;
				atomic_and(&tdma_shadow_live_evidence_mask, (atomic_val_t)~bit);
				atomic_set(&tdma_shadow_last_seen_ms[i], 0);
			} else if (has_evidence && age_ms >= TDMA_SHADOW_LEAVE_GRACE_MS) {
				evicted_mask |= bit;
				desired_mask &= ~bit;
				atomic_and(&tdma_shadow_live_evidence_mask, (atomic_val_t)~bit);
				atomic_set(&tdma_shadow_last_seen_ms[i], 0);
			} else if (has_evidence && age_ms >= TDMA_SHADOW_FRESH_MS) {
				pending_leave_mask |= bit;
			}
		} else if (has_evidence && age_ms < TDMA_SHADOW_FRESH_MS) {
			desired_mask |= bit;
		} else if (has_evidence) {
			atomic_and(&tdma_shadow_live_evidence_mask, (atomic_val_t)~bit);
			atomic_set(&tdma_shadow_last_seen_ms[i], 0);
		}
	}

	if (desired_mask != (uint32_t)atomic_get(&tdma_shadow_desired_mask)) {
		atomic_set(&tdma_shadow_desired_changed_at_ms, (atomic_val_t)now_ms);
	}
	atomic_set(&tdma_shadow_desired_mask, (atomic_val_t)desired_mask);
	atomic_set(&tdma_shadow_pending_join_mask, 0);
	atomic_set(&tdma_shadow_pending_leave_mask, (atomic_val_t)pending_leave_mask);
	if (desired_mask == tdma_active_mask) {
		tdma_membership_deadline_armed = false;
	} else if (!tdma_membership_deadline_armed) {
		/* A departure has no admission cohort to collect. */
		tdma_membership_deadline_ms = now_ms;
		tdma_membership_deadline_armed = true;
	}
	return evicted_mask;
}

static void tdma_shadow_update(uint32_t now_ms)
{
	unsigned int key = irq_lock();
	uint32_t evicted_mask = tdma_shadow_merge_locked(now_ms);
	irq_unlock(key);
	if (evicted_mask) {
		LOG_WRN("SHADOW evict mask=0x%04x now=%u (grace=%u)",
			(unsigned int)evicted_mask, now_ms, TDMA_SHADOW_LEAVE_GRACE_MS);
	}
}

static uint32_t tdma_membership_wait_ms(uint32_t maximum_ms)
{
	unsigned int key = irq_lock();
	if (tdma_membership_deadline_armed) {
		int64_t now = k_uptime_get();
		int32_t cohort_remaining = (int32_t)(tdma_membership_deadline_ms - (uint32_t)now);
		int64_t gap_remaining = tdma_last_reconfig_valid
			? TDMA_RECONFIG_MIN_MS - (now - tdma_last_reconfig_time) : 0;
		int64_t remaining = MAX((int64_t)cohort_remaining, gap_remaining);
		maximum_ms = remaining > 0 ? MIN(maximum_ms, (uint32_t)remaining) : 0;
	}
	irq_unlock(key);
	return maximum_ms;
}


#define TEST_ALL_RATE_QUANTUM_TPS 10U
#define TEST_ALL_JOIN_CONFIG_GRACE_MS 1500

static uint16_t test_all_capacity_tps(uint8_t active_count)
{
	if (active_count == 0) {
		return 1000;
	}
	/* Capacity follows the *actual* layout: ping_guard_v2 widens slots when
	 * trackers are few, so fixed TDMA_BASE_SLOT_TICKS math would overestimate
	 * achievable TPS. Test-all forces sticks back to 16 (max rate), but during
	 * the layout transition the current width is the truth. */
	uint8_t sticks = tdma_dynamic_slot_ticks > 0 ? tdma_dynamic_slot_ticks : TDMA_BASE_SLOT_TICKS;
	uint16_t capacity = (uint16_t)(32768U / ((uint32_t)sticks * active_count));
	uint16_t clamped = (capacity / TEST_ALL_RATE_QUANTUM_TPS) * TEST_ALL_RATE_QUANTUM_TPS;
	return clamped > 0 ? clamped : 1;
}

static uint16_t test_all_requested_value(void)
{
	return test_all_requested_tps == 0 ? 100 : test_all_requested_tps;
}

static uint16_t test_all_wire_tps(void)
{
	/* Preserve zero as the protocol spelling of the built-in 100 TPS default. */
	return test_all_requested_tps == 0 ? 0 : test_all_effective_tps;
}

/* Any targeted (per-tracker) TEST_MODE_ON/OFF command supersedes sticky-all
 * policy: stop reconciling, drop stale confirmations, and release the
 * broadcast guard so an in-flight all-command cannot keep suppressing it. */
static void test_all_invalidate(void)
{
	atomic_set(&test_all_state_valid, 0);
	atomic_set(&test_all_confirmed_mask, 0);
	atomic_set(&test_all_broadcasting, 0);
}

/* After a restart or a >5s PING gap: drop the sticky confirmation, park a
 * pending test flag as NORMAL so the next PONG delivers a clean TDMA config
 * first, and arm the join grace; reconciliation re-queues afterwards. */
static void test_all_invalidate_tracker(uint8_t tracker_id, int64_t now_ms)
{
	uint8_t pending_cmd = tracker_remote_command[tracker_id];
	if (pending_cmd == ESB_PONG_FLAG_TEST_MODE_ON || pending_cmd == ESB_PONG_FLAG_TEST_MODE_OFF) {
		tracker_remote_command[tracker_id] = ESB_PONG_FLAG_NORMAL;
	}
	atomic_and(&test_all_confirmed_mask, (atomic_val_t)~BIT(tracker_id));
	atomic_set(&test_all_ready_after_ms[tracker_id], (atomic_val_t)((uint32_t)now_ms + TEST_ALL_JOIN_CONFIG_GRACE_MS));
}

static void test_all_update_effective(uint8_t active_count, int64_t now, bool layout_changed)
{
	if (!atomic_get(&test_all_state_valid) || !atomic_get(&test_all_enabled)) {
		return;
	}
	uint16_t requested = test_all_requested_value();
	uint16_t effective = MIN(requested, test_all_capacity_tps(active_count));
	if (effective == test_all_effective_tps) {
		return;
	}
	test_all_effective_tps = effective;
	uint32_t active_mask = tdma_active_mask;
	atomic_and(&test_all_confirmed_mask, (atomic_val_t)~active_mask);
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		if (!(active_mask & BIT(i))) {
			continue;
		}
		tracker_test_on_tps[i] = test_all_wire_tps();
		atomic_set(
			&test_all_ready_after_ms[i],
			(atomic_val_t)((uint32_t)now + (layout_changed ? TEST_ALL_JOIN_CONFIG_GRACE_MS : 0))
		);
	}
	LOG_INF(
		"Sticky TEST_MODE_ON reclamp: requested=%u effective=%u active=%u capacity=%u",
		requested,
		effective,
		active_count,
		test_all_capacity_tps(active_count)
	);
}

static void test_all_note_layout(uint32_t old_mask, uint32_t new_mask, int64_t now)
{
	if (!atomic_get(&test_all_state_valid)) {
		return;
	}
	uint32_t joined_mask = new_mask & ~old_mask;
	if (joined_mask == 0) {
		return;
	}
	atomic_and(&test_all_confirmed_mask, (atomic_val_t)~joined_mask);
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		if (joined_mask & BIT(i)) {
			/* Refresh the joined tracker's TPS even on a same-count swap:
			 * update_effective skips unchanged effective rates. */
			if (atomic_get(&test_all_enabled)) {
				tracker_test_on_tps[i] = test_all_wire_tps();
			}
			/* First allow a NORMAL PONG to deliver the new TDMA layout;
			 * then converge the newly active tracker to test all state. */
			atomic_set(&test_all_ready_after_ms[i], (atomic_val_t)((uint32_t)now + TEST_ALL_JOIN_CONFIG_GRACE_MS));
		}
	}
}

static void test_all_reconcile(int64_t now)
{
	if (!atomic_get(&test_all_state_valid) || atomic_get(&test_all_broadcasting)) {
		return;
	}
	bool enabled = atomic_get(&test_all_enabled) != 0;
	uint8_t command = enabled ? ESB_PONG_FLAG_TEST_MODE_ON : ESB_PONG_FLAG_TEST_MODE_OFF;
	uint32_t queued_mask = 0;
	uint16_t requested = 0;
	uint16_t effective = 0;
	/* Same critical section as the test commands: the publication decision
	 * and TPS/flag writes are atomic against targeted supersede. */
	unsigned int key = irq_lock();
	if (atomic_get(&test_all_state_valid) && !atomic_get(&test_all_broadcasting)) {
		uint32_t pending_mask = tdma_active_mask
			& ~(uint32_t)atomic_get(&test_all_confirmed_mask);
		for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
			uint32_t ready_after = (uint32_t)atomic_get(&test_all_ready_after_ms[i]);
			if (!(pending_mask & BIT(i)) || (int32_t)((uint32_t)now - ready_after) < 0
				|| tracker_remote_command[i] != ESB_PONG_FLAG_NORMAL) {
				continue;
			}
			if (enabled) {
				tracker_test_on_tps[i] = test_all_wire_tps();
			}
			__asm__ volatile("" ::: "memory"); /* TPS published before the flag the ISR acts on */
			/* Re-read right before publishing: if an EVENT-path recovery
			 * re-armed the grace during iteration, skip this cycle and let
			 * the next reconcile pass handle the tracker. */
			if ((uint32_t)atomic_get(&test_all_ready_after_ms[i]) != ready_after) {
				continue;
			}
			tracker_remote_command[i] = command;
			queued_mask |= BIT(i);
		}
		requested = test_all_requested_value();
		effective = test_all_effective_tps;
	}
	irq_unlock(key);
	if (queued_mask == 0) {
		return;
	}
	LOG_INF(
		"Queued sticky TEST_MODE_%s for trackers 0x%04x (requested=%u effective=%u)",
		enabled ? "ON" : "OFF",
		(unsigned int)queued_mask,
		enabled ? requested : 0,
		enabled ? effective : 0
	);
}

/* Widest even slot width (ticks) that keeps per-tracker opportunity rate at
 * or above the current ladder cap: sticks <= 32768/(N*cap) is equivalent to
 * 32768/(sticks*N) >= cap. Level 0 clamps width to 64 ticks so normal mode
 * always holds comfortable headroom above the ~220 TPS useful demand; deeper
 * loss levels may widen up to the protocol byte limit to deliberately trade
 * rate for stability. Test-all measures raw throughput, so it forces the
 * narrowest (max-rate) layout. */
static uint8_t tdma_slot_ticks_for(uint8_t active_count)
{
	if (active_count == 0) {
		return TDMA_BASE_SLOT_TICKS;
	}
	if (atomic_get(&test_all_enabled)) {
		return TDMA_BASE_SLOT_TICKS;
	}
	uint32_t cap = tdma_cap_ladder[tdma_cap_level];
	uint32_t max_sticks = 32768U / ((uint32_t)active_count * cap);
	uint32_t width_limit = tdma_cap_level == 0 ? TDMA_SLOT_TICKS_MAX_LEVEL0 : 255U;
	uint32_t sticks = MIN(max_sticks, width_limit) & ~1U;
	return (uint8_t)MAX(sticks, TDMA_BASE_SLOT_TICKS);
}


/* Caller holds tracker_store_lock. Forced administrative cleanup bypasses the
 * ordinary spacing, but only an actual layout change moves its anchor. */
static void tdma_recalculate(bool forced)
{
	bool diagnostics_reset = false;

	for (;;) {
		unsigned int key = irq_lock();
		int64_t now = k_uptime_get();
		if (!forced && (diagnostics_reset || atomic_get(&tdma_shadow_pending_join_mask)
		    || ((uint32_t)atomic_get(&tdma_shadow_desired_mask) & ~tdma_active_mask))) {
			(void)tdma_shadow_merge_locked((uint32_t)now);
		}
		uint32_t old_mask = tdma_active_mask;
		uint32_t stored_mask = stored_trackers >= MAX_TRACKERS
			? UINT32_MAX : (stored_trackers > 0 ? BIT(stored_trackers) - 1U : 0U);
		uint32_t new_mask = (uint32_t)atomic_get(&tdma_shadow_desired_mask) & stored_mask;
		uint8_t active_count = 0;
		for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; i++) {
			if (new_mask & BIT(i)) {
				active_count++;
			}
		}

		bool mask_changed = new_mask != old_mask;
		if (mask_changed) {
			tdma_cap_level = 0;
			tdma_loss_reset_windows();
		} else if (data_collect_is_active() || data_collect_batch_is_active()) {
			tdma_cap_level = tdma_published_cap_level;
		}
		uint8_t slot_ticks = tdma_slot_ticks_for(active_count);
		bool test_all_layout = atomic_get(&test_all_enabled) != 0;
		bool ladder_step = tdma_cap_level != tdma_published_cap_level;

		/* An initially empty layout is not a publication. */
		if (!mask_changed && active_count == 0) {
			tdma_dynamic_slot_ticks = TDMA_BASE_SLOT_TICKS;
			tdma_published_cap_level = tdma_cap_level;
			irq_unlock(key);
			return;
		}
		if (!mask_changed && slot_ticks == tdma_dynamic_slot_ticks) {
			if (!test_all_layout) {
				tdma_published_cap_level = tdma_cap_level;
			}
			irq_unlock(key);
			return;
		}

		uint32_t min_reconfig_ms = (mask_changed || test_all_layout || !ladder_step)
			? TDMA_RECONFIG_MIN_MS : TDMA_LOSS_RECONFIG_MIN_MS;
		if (!forced && ((mask_changed && tdma_membership_deadline_armed
		    && (int32_t)((uint32_t)now - tdma_membership_deadline_ms) < 0)
		    || (tdma_last_reconfig_valid && now - tdma_last_reconfig_time < min_reconfig_ms))) {
			irq_unlock(key);
			return;
		}

		/* Keep the larger diagnostic reset outside the IRQ lock, then resample
		 * evidence before publishing the final snapshot under that lock. */
		if (active_count && !diagnostics_reset) {
			irq_unlock(key);
			tdma_sync_stats_reset();
			diagnostics_reset = true;
			continue;
		}

		uint8_t epoch = tdma_config_epoch;
		if (active_count) {
			epoch = ++tdma_config_epoch;
		}
		uint8_t slot = 0;
		for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
			uint8_t assigned = 0xFF;
			if (new_mask & BIT(i)) {
				assigned = (slot++ + TDMA_SLOT_ROTATION) % active_count;
			}
			tdma_config_packed[i] = tdma_pack_config(assigned, active_count, slot_ticks, epoch);
		}
		tdma_dynamic_active_count = active_count;
		tdma_dynamic_slot_ticks = slot_ticks;
		tdma_active_mask = new_mask;
		tdma_last_reconfig_time = now;
		tdma_last_reconfig_valid = true;
		if (!atomic_get(&tdma_shadow_pending_join_mask)) {
			tdma_membership_deadline_armed = false;
		}
		if (mask_changed || !test_all_layout) {
			tdma_published_cap_level = tdma_cap_level;
		}
#if defined(CONFIG_TDMA_DIAGNOSTICS)
		if (ladder_step && !test_all_layout) {
			tdma_last_loss_step_ms = now;
		}
#endif
		esb_auto_loss_invalidate();
		irq_unlock(key);

		if (!active_count) {
			return;
		}
		test_all_note_layout(old_mask, new_mask, now);
		test_all_update_effective(active_count, now, true);
		uint32_t frame_ticks = (uint32_t)slot_ticks * active_count;
		uint32_t est_tps = 32768 / frame_ticks;
		LOG_INF(
			"TDMA reconfig: mode=ping_guard_v2 reason=%s rotation=%u active=%u slot=%u frame=%u opportunity=%u/trk aggregate=%u/s cap=%u lvl=%u/%u epoch=%u members=0x%04x",
			mask_changed ? "membership" : (test_all_layout ? "test_all" : "loss_ladder"),
			TDMA_SLOT_ROTATION,
			active_count,
			slot_ticks,
			frame_ticks,
			est_tps,
			32768 / slot_ticks,
			tdma_cap_ladder[tdma_cap_level],
			tdma_cap_level,
			(uint8_t)TDMA_CAP_LEVEL_MAX,
			epoch,
			new_mask
		);
		return;
	}
}

/* Migration owns an ACK overlay, never the generic command slots. Delivery
 * echoes are not execution barriers: a tracker may move before echoing. */
static atomic_t radio_operation_busy;
/* Sticky on an unsafe teardown: never reinitialize/release a faulted RADIO. */
static bool esb_radio_fault;
static atomic_t channel_change_pending;
static uint8_t pending_channel;
static uint8_t channel_command;
static uint32_t channel_target_mask;
static int64_t channel_change_timeout;
static bool channel_change_automatic;
static uint32_t channel_change_generation;
static uint8_t channel_change_old_home;
static esb_channel_change_done_cb_t channel_change_done_cb;
static esb_remote_confirm_cb_t remote_confirm_cb;
#define CHANNEL_CHANGE_ANNOUNCE_MS 3000

bool esb_channel_is_busy(void)
{
	return atomic_get(&radio_operation_busy) != 0;
}

static int radio_operation_begin(bool replace_collection)
{
	if (esb_radio_fault) {
		return -EIO;
	}
	if (!atomic_cas(&radio_operation_busy, 0, 1)) {
		return -EBUSY;
	}
	if (esb_ota_relay_is_active() || receiver_ota_is_active() ||
	    (!replace_collection && (data_collect_is_active() || data_collect_batch_is_active()))) {
		atomic_set(&radio_operation_busy, 0);
		return -EBUSY;
	}
	for (uint8_t i = 0; i < MAX_TRACKERS; ++i) {
		if (esb_ota_relay_abort_pending(i)) {
			atomic_set(&radio_operation_busy, 0);
			return -EBUSY;
		}
	}
	return 0;
}

int esb_radio_operation_begin(void)
{
	return radio_operation_begin(false);
}

int esb_radio_collection_begin(void)
{
	int err = radio_operation_begin(true);
	if (!err) {
		esb_auto_loss_invalidate();
	}
	return err;
}

void esb_radio_operation_end(void)
{
	if (!esb_radio_fault) {
		atomic_set(&radio_operation_busy, 0);
	}
}

static uint8_t channel_command_for_tracker(uint8_t id)
{
	return atomic_get(&channel_change_pending) && (channel_target_mask & BIT(id))
		? channel_command : ESB_PONG_FLAG_NORMAL;
}

// Pairing state (declared here for ISR access in event_handler)
static bool esb_pairing = false;
static bool esb_clearing = false;

bool esb_pairing_is_active(void)
{
	return esb_pairing;
}

// Pairing timeout and target count
static int64_t pairing_start_time = 0;
static uint8_t pairing_target_count = 0;                  // 0 = no limit, >0 = exit after N new devices
static uint8_t pairing_initial_count = 0;                 // Number of devices when pairing started
static int64_t pairing_target_reached_time = 0;           // Time when target count was reached (for delayed exit)
static volatile bool pairing_new_devices_blocked = false; // When true, only allow re-pairing of known devices
#define PAIRING_EXIT_DELAY_MS 15000                       // Delay before exiting after target count reached

// Cached receiver device address (set once at init, used by ISR for pairing responses)
static uint8_t receiver_device_addr[6] = {0};


// Packet statistics structure
struct packet_stats {
	uint32_t total_received;    // Total number of packets received (excluding duplicates)
	uint32_t normal_packets;    // Number of packets received in normal sequence
	uint32_t gap_events;        // Number of gap events (potential packet loss)
	uint32_t out_of_order;      // Number of out-of-order packets
	uint32_t duplicate_packets; // Number of duplicate packets
	uint32_t restart_events;    // Number of restart events
	uint32_t total_gaps;        // Total number of gaps (estimated packet loss)
	uint32_t last_sequence;     // Last normal sequence number
	uint64_t last_packet_time;  // Timestamp of the last received packet
	bool first_packet;          // Flag indicating if it's the first packet
	// TPS calculation related
	uint32_t packets_in_last_second; // Number of packets in the last second
	uint64_t last_tps_time;          // Last TPS calculation timestamp
	uint32_t current_tps;            // Current TPS (packets per second)
	// Per-status-period counters (filled into status packet data[4]/data[5], reset after each status packet)
	uint16_t status_received; // Packets received since last status packet
	uint16_t status_lost;     // Packets lost (gaps) since last status packet
};

static struct packet_stats tracker_stats[MAX_TRACKERS] = {0};
static uint32_t auto_loss_reset_epoch;
/* Decision evidence is distinct from lifetime/status diagnostics. A fence
 * drops only the interval crossing a known boundary, never decoder state. */
static uint32_t decision_received[MAX_TRACKERS];
static uint32_t decision_gaps[MAX_TRACKERS];
static uint32_t decision_ready_mask;
static uint32_t decision_fence_mask = UINT32_MAX;

void esb_auto_loss_fence(void)
{
	unsigned int key = irq_lock();
	decision_fence_mask = UINT32_MAX;
	irq_unlock(key);
}

static void decision_accept(uint8_t tracker_id, uint8_t gaps)
{
	uint32_t bit = BIT(tracker_id);
	if (decision_fence_mask & bit) {
		decision_fence_mask &= ~bit;
		decision_ready_mask |= bit;
		return;
	}
	decision_ready_mask |= bit;
	decision_received[tracker_id]++;
	decision_gaps[tracker_id] += gaps;
}

void esb_auto_loss_invalidate(void)
{
	unsigned int key = irq_lock();
	auto_loss_reset_epoch++;
	decision_ready_mask = 0;
	decision_fence_mask = UINT32_MAX;
	atomic_set(&tdma_rx_discontinuity_pending, 1);
	irq_unlock(key);
}

void esb_rx_discontinuity(void)
{
	/* Only deliberate long blackouts with RX quiescent; old FIFO entries
	 * must be discarded before delivery can resume. Never use for home
	 * beacons or individual automatic energy measurements. */
	unsigned int key = irq_lock();
	memset(sequence_initialized, 0, sizeof(sequence_initialized));
	esb_auto_loss_invalidate();
	irq_unlock(key);
}

void esb_get_auto_loss_snapshot(struct esb_auto_loss_snapshot *out)
{
	BUILD_ASSERT(MAX_TRACKERS == ESB_AUTO_LOSS_TRACKERS);
	unsigned int key = irq_lock();
	for (uint8_t i = 0; i < MAX_TRACKERS; ++i) {
		out->tracker[i].received = decision_received[i];
		out->tracker[i].gaps = decision_gaps[i];
		out->tracker[i].restarts = tracker_stats[i].restart_events;
	}
	out->reset_epoch = auto_loss_reset_epoch;
	out->active_mask = tdma_active_mask;
	out->ready_mask = decision_ready_mask & tdma_active_mask;
	out->layout_epoch = tdma_config_epoch;
	out->cap_pending = tdma_cap_level != tdma_published_cap_level;
	out->home = esb_rf_channel_decode(receiver_rf_channel);
	if (out->home == ESB_RF_CHANNEL_DEFAULT) {
		out->home = RADIO_RF_CHANNEL;
	}
	out->eligible = !esb_radio_fault && !esb_channel_is_busy() && !esb_pairing
		&& !esb_clearing && !esb_ota_relay_is_active() && !receiver_ota_is_active()
		&& !data_collect_is_active() && !data_collect_batch_is_active()
		&& !(atomic_get(&test_all_state_valid) && atomic_get(&test_all_enabled))
		&& out->active_mask == (uint32_t)atomic_get(&tdma_shadow_desired_mask)
		&& !atomic_get(&tdma_shadow_pending_join_mask)
		&& !atomic_get(&tdma_shadow_pending_leave_mask);
	for (uint8_t i = 0; i < MAX_TRACKERS; ++i) {
		if (esb_ota_relay_abort_pending(i)) {
			out->eligible = false;
		}
	}
	irq_unlock(key);
}

void esb_auto_loss_rebase(struct esb_auto_loss_snapshot *out)
{
	unsigned int key = irq_lock();
	esb_auto_loss_fence();
	esb_get_auto_loss_snapshot(out);
	irq_unlock(key);
}
#define STATS_PRINT_INTERVAL_MS 1000     // Print detailed statistics every 1 second (when enabled)
#define TPS_CALCULATION_INTERVAL_MS 1000 // Calculate TPS every second
#define TPS_MONITOR_INTERVAL_MS 500

// Statistics display control
static volatile bool stats_detailed_enabled = false; // Whether detailed stats are enabled
static volatile int64_t stats_detailed_end_time
	= 0;                                // Timestamp when detailed stats should auto-disable (0 = no auto-disable)
static int64_t last_tps_print_time = 0; // Last time total TPS was printed

#if defined(CONFIG_TDMA_DIAGNOSTICS)
// Receiver-side arrival phase/RSSI histograms. Updated in EVENT context only;
// health computes percentiles on demand, so the RADIO ISR remains unchanged.
#define RX_PHASE_MIN_TICKS (-64)
#define RX_PHASE_BUCKETS 128
#define RX_RSSI_MAX_DBM 127
#define RX_RSSI_BUCKETS 128

struct tdma_stats {
	int64_t sum_offset;
	uint64_t sum_sq_offset;
	int32_t min_offset;
	int32_t max_offset;
	uint32_t count;
	uint32_t violations;
	int32_t phase_ema_q8;
	bool phase_initialized;
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	uint32_t phase_hist[RX_PHASE_BUCKETS];
	uint32_t raw_phase_hist[RX_PHASE_BUCKETS];
	int32_t raw_min_offset;
	int32_t raw_max_offset;
	uint32_t rssi_hist[RX_RSSI_BUCKETS];
	uint32_t rssi_count;
#endif
};


static struct tdma_stats g_tdma_stats[MAX_TRACKERS] = {0};

static void tdma_stats_reset(void)
{
	memset(g_tdma_stats, 0, sizeof(g_tdma_stats));
}

/* Last observed time-sync error from PING (rx_ticks - expected_rx_ticks), per tracker.
 * Used only for diagnostics to understand TDMA phase offset on receiver side.
 *
 * IMPORTANT: g_last_ping_rx_time_diff_ticks is computed from the RADIO ISR timestamp
 * (g_ping_isr_rx_ticks), NOT from the EVENT_IRQ current_rx_ticks.  The EVENT_IRQ fires
 * at priority 2, which can be delayed by Zephyr kernel critical sections (spinlocks)
 * by 10-25+ ticks relative to the actual packet receipt.  Using the EVENT_IRQ timestamp
 * would inflate clock_bias by that scheduling jitter, causing data packets to appear
 * 15-20 ticks early in tdma_check_slot() — i.e. false violations with Mean ≈ −15. */
static int32_t g_last_ping_rx_time_diff_ticks[MAX_TRACKERS] = {0};
static bool g_last_ping_rx_time_diff_valid[MAX_TRACKERS] = {0};

/* RADIO ISR timestamp (priority 1) captured in esb_ack_handler_cb for each PING.
 * Written from RADIO ISR (prio 1), read from EVENT ISR (prio 2).
 * 32-bit aligned write on ARM Cortex-M4 is atomic, so no lock needed. */
static volatile uint32_t g_ping_isr_rx_ticks[MAX_TRACKERS] = {0};
static volatile bool g_ping_isr_rx_ticks_valid[MAX_TRACKERS] = {false};

/* Per-tracker RADIO ISR timestamp for ALL packets (PING + data).
 * Now works for NoACK data packets too, since the ESB library was patched
 * to always call ack_handler regardless of the no_ack flag.
 * Used by tdma_check_slot() for accurate phase measurement. */
static volatile uint32_t g_last_isr_rx_ticks[MAX_TRACKERS] = {0};
static volatile bool g_last_isr_rx_valid[MAX_TRACKERS] = {false};

/* Clock bias drift rate estimation (ppb = parts per billion).
 * Uses a long-baseline approach: hold a reference point and measure
 * total drift over time, averaging out PING retransmission noise.
 *
 * g_bias_ppb: estimated drift rate in ppb, positive = tracker clock
 *   is fast relative to receiver.
 * g_bias_ref_offset: clock_bias at the reference point.
 * g_bias_ref_ticks: receiver ticks at the reference point.
 * g_last_ping_isr_rx_ticks_raw: ticks of the most recent PING,
 *   used as elapsed-time baseline for per-packet extrapolation. */
static uint8_t g_bias_ppb_valid[MAX_TRACKERS] = {0};
static int32_t g_bias_ppb[MAX_TRACKERS] = {0};
static int32_t g_bias_ref_offset[MAX_TRACKERS] = {0};
static uint32_t g_bias_ref_ticks[MAX_TRACKERS] = {0};
static uint32_t g_last_ping_isr_rx_ticks_raw[MAX_TRACKERS] = {0};
static uint8_t g_ping_dirty_streak[MAX_TRACKERS] = {0};  /* consecutive rejected PINGs */
static uint32_t g_ping_reject_count[MAX_TRACKERS] = {0}; /* total rejected PINGs since stats reset */
static uint32_t g_extrap_skip_count[MAX_TRACKERS] = {0}; /* extrapolations skipped due to stale baseline */

#endif

static void tdma_sync_stats_reset(void)
{
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	tdma_stats_reset();
	memset(g_last_ping_rx_time_diff_valid, 0, sizeof(g_last_ping_rx_time_diff_valid));
	memset(g_bias_ppb_valid, 0, sizeof(g_bias_ppb_valid));
	memset(g_last_ping_isr_rx_ticks_raw, 0, sizeof(g_last_ping_isr_rx_ticks_raw));
	memset(g_last_ping_rx_time_diff_ticks, 0, sizeof(g_last_ping_rx_time_diff_ticks));
	memset(g_ping_dirty_streak, 0, sizeof(g_ping_dirty_streak));
	memset(g_ping_reject_count, 0, sizeof(g_ping_reject_count));
	memset(g_extrap_skip_count, 0, sizeof(g_extrap_skip_count));
#endif
}

/* Aggregate normal/composite sequence-gap loss, excluding raw capture.
 * Pool consecutive nonempty 1 s stats ticks until at least 100 samples exist;
 * classify the whole pool and credit its ticks, never an idle interval.
 * Publication and debounce stay owned by tdma_recalculate. */
static void tdma_loss_controller_update(int64_t now)
{
	ARG_UNUSED(now);

	uint32_t tick_received = 0;
	uint32_t tick_gaps = 0;
	struct esb_auto_loss_snapshot evidence;
	unsigned int key = irq_lock();
	esb_get_auto_loss_snapshot(&evidence);
	bool discontinuity = atomic_set(&tdma_rx_discontinuity_pending, 0) != 0;
	irq_unlock(key);
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		uint32_t r = evidence.tracker[i].received;
		uint32_t g = evidence.tracker[i].gaps;
		uint32_t prev_r = tdma_prev_recv[i];
		uint32_t prev_g = tdma_prev_gaps[i];
		uint32_t restarts = evidence.tracker[i].restarts;
		bool restarted = restarts != tdma_prev_restarts[i];
		tdma_prev_restarts[i] = restarts;
		tdma_prev_recv[i] = r;
		tdma_prev_gaps[i] = g;
		/* A reset/restart is not a loss window. Never carry recovery evidence
		 * across a sequence epoch, even when other trackers remain active. */
		if (!(tdma_active_mask & BIT(i))) {
			continue;
		}
		if (r < prev_r || g < prev_g || restarted) {
			discontinuity = true;
			continue;
		}
		uint32_t delta_recv = r - prev_r;
		uint32_t delta_gaps = g - prev_g;
		if (delta_recv + delta_gaps == 0) {
			continue;
		}
		tick_received += delta_recv;
		tick_gaps += delta_gaps;
	}

	/* Always advance snapshots above. Otherwise leaving an excluded mode would
	 * collapse its entire accumulated traffic into one fake 1 s loss window. */
	bool collecting = data_collect_is_active() || data_collect_batch_is_active();
	if (collecting) {
		/* Batch fusion traffic is throttled to 10 TPS; it must not drive
		 * a layout change underneath the independent raw stream. */
		tdma_cap_level = tdma_published_cap_level;
	}
	if (discontinuity || !evidence.eligible
	    || (evidence.ready_mask & tdma_active_mask) != tdma_active_mask
	    || collecting || esb_ota_relay_is_active()
	    || atomic_get(&channel_change_pending) || esb_radio_fault
	    || (atomic_get(&test_all_state_valid) && atomic_get(&test_all_enabled))) {
		tdma_decision_ready = evidence.eligible
			&& evidence.ready_mask == tdma_active_mask;
		if (tdma_decision_ready) {
			esb_auto_loss_rebase(&evidence);
		}
		tdma_loss_reset_windows();
		return;
	}
	if (!tdma_decision_ready) {
		/* Rendezvous at one common boundary, not each participant's first
		 * return. An early, then silent stream must not carry old gaps in. */
		esb_auto_loss_rebase(&evidence);
		tdma_decision_ready = true;
		tdma_loss_reset_windows();
		return;
	}

	/* A requested level must be published (or acknowledged as a physical
	 * no-op) before another step can be requested. This prevents a 15 s
	 * reconfiguration debounce from accumulating three 5 s down-steps. */
	if (tdma_cap_level != tdma_published_cap_level) {
		tdma_loss_reset_windows();
		return;
	}

	if (tick_received + tick_gaps == 0) {
		tdma_loss_reset_windows();
		return;
	}
	tdma_loss_pool_received += tick_received;
	tdma_loss_pool_gaps += tick_gaps;
	tdma_loss_pool_ticks++;
	uint32_t pool_samples = tdma_loss_pool_received + tdma_loss_pool_gaps;
	if (pool_samples < TDMA_LOSS_MIN_SAMPLES) {
		return;
	}

	/* Judge the pooled ratio once, crediting every contributing nonempty tick. */
	uint32_t pool_received = tdma_loss_pool_received;
	uint32_t pool_gaps = tdma_loss_pool_gaps;
	uint8_t credited_ticks = tdma_loss_pool_ticks;
	tdma_loss_pool_received = 0;
	tdma_loss_pool_gaps = 0;
	tdma_loss_pool_ticks = 0;

	/* Compare exact ratios; rounded permille is only for diagnostics/logging. */
	uint64_t scaled_gaps = (uint64_t)pool_gaps * 1000U;
	bool above_trigger = scaled_gaps > (uint64_t)TDMA_LOSS_TRIGGER_PERMILLE * pool_samples;
	bool below_recover = scaled_gaps < (uint64_t)TDMA_LOSS_RECOVER_PERMILLE * pool_samples;
	bool below_probe = scaled_gaps < (uint64_t)TDMA_LOSS_PROBE_PERMILLE * pool_samples;
	uint32_t loss_permille = scaled_gaps / pool_samples;
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	tdma_loss_last_permille = loss_permille;
#endif
	tdma_loss_trigger_streak_ticks
		= above_trigger ? MIN(tdma_loss_trigger_streak_ticks + credited_ticks, TDMA_LOSS_TRIGGER_TICKS) : 0;
	tdma_loss_recover_streak_ticks
		= below_recover ? MIN(tdma_loss_recover_streak_ticks + credited_ticks, TDMA_LOSS_RECOVER_TICKS) : 0;
	tdma_loss_probe_streak_ticks
		= below_probe ? MIN(tdma_loss_probe_streak_ticks + credited_ticks, TDMA_LOSS_PROBE_TICKS) : 0;

	if (tdma_loss_trigger_streak_ticks >= TDMA_LOSS_TRIGGER_TICKS && tdma_cap_level < TDMA_CAP_LEVEL_MAX) {
		tdma_cap_level++;
		tdma_loss_reset_windows();
		LOG_WRN(
			"TDMA loss ladder DOWN: lvl=%u/%u cap=%u loss=%u.%u%% recv=%u gaps=%u",
			tdma_cap_level,
			(uint8_t)TDMA_CAP_LEVEL_MAX,
			tdma_cap_ladder[tdma_cap_level],
			loss_permille / 10,
			loss_permille % 10,
			pool_received,
			pool_gaps
		);
	} else if ((tdma_loss_recover_streak_ticks >= TDMA_LOSS_RECOVER_TICKS
				|| tdma_loss_probe_streak_ticks >= TDMA_LOSS_PROBE_TICKS)
			   && tdma_cap_level > 0) {
		tdma_cap_level--;
		tdma_loss_reset_windows();
		LOG_INF(
			"TDMA loss ladder UP: lvl=%u/%u cap=%u",
			tdma_cap_level,
			(uint8_t)TDMA_CAP_LEVEL_MAX,
			tdma_cap_ladder[tdma_cap_level]
		);
	}
}

static void tdma_loss_controller_tick(int64_t now)
{
	/* Scan/migration boundaries are thread-owned. Keep their epoch changes
	 * outside the complete snapshot-to-cap decision, without masking RADIO
	 * interrupts while inspecting the bounded tracker roster. */
	k_sched_lock();
	tdma_loss_controller_update(now);
	k_sched_unlock();
}

#if defined(CONFIG_TDMA_DIAGNOSTICS)
#if TDMA_ENABLED
/**
 * Check if a packet from a tracker arrived in its assigned TDMA slot.
 * Uses dynamic parameters from tdma_config_packed[].
 *
 * @param tracker_id   The tracker's ID (0-15)
 * @param rx_ticks     Receiver time in ticks when packet arrived (EVENT_IRQ fallback)
 */
static void tdma_check_slot(uint8_t tracker_id, uint32_t rx_ticks, int8_t rssi)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}

	/* Prefer RADIO ISR timestamp (accurate) over EVENT_IRQ timestamp
	 * (delayed 10-25+ ticks by kernel scheduling).  The ISR timestamp is
	 * now available for ALL packets including NoACK data, thanks to the
	 * ESB library patch that always calls on_radio_disabled_rx_dpl(). */
	if (g_last_isr_rx_valid[tracker_id]) {
		rx_ticks = g_last_isr_rx_ticks[tracker_id];
	}

	/* Read dynamic TDMA config (atomic 32-bit load) */
	uint32_t cfg = tdma_config_packed[tracker_id];
	uint8_t expected_slot = (cfg >> 24) & 0xFF;
	uint8_t total_slots = (cfg >> 16) & 0xFF;
	uint8_t slot_ticks = (cfg >> 8) & 0xFF;

	/* Skip validation if tracker has no slot assignment or config invalid */
	if (expected_slot == 0xFF || total_slots == 0 || slot_ticks == 0) {
		return;
	}

	uint32_t frame_ticks = (uint32_t)slot_ticks * total_slots;

	/*
	 * Compensate receiver clock vs tracker clock offset.
	 * Base bias: last clean PING measurement, extrapolated forward
	 * using the measured drift rate (ppb) to keep it accurate
	 * between PING sync intervals.
	 */
	int32_t clock_bias = g_last_ping_rx_time_diff_valid[tracker_id] ? g_last_ping_rx_time_diff_ticks[tracker_id] : 0;

	/* Drift extrapolation: bias changes linearly between PINGs.
	 * Use the long-baseline ppb estimate to correct for drift
	 * since the last PING, keeping the effective bias accurate
	 * throughout the sync interval. */
	if (clock_bias && g_last_ping_isr_rx_ticks_raw[tracker_id] != 0 && g_bias_ppb_valid[tracker_id]) {
		uint32_t elapsed = rx_ticks - g_last_ping_isr_rx_ticks_raw[tracker_id];
		if (elapsed <= TDMA_SYNC_EXTRAP_MAX_TICKS) {
			int32_t drift_comp = (int32_t)((int64_t)g_bias_ppb[tracker_id] * elapsed / 1000000000LL);
			clock_bias += drift_comp;
		} else {
			g_extrap_skip_count[tracker_id]++;
		}
	}

	uint32_t adjusted_rx_ticks = (uint32_t)((int32_t)rx_ticks - clock_bias);
	uint32_t raw_frame_phase = rx_ticks % frame_ticks;
	uint32_t frame_phase = adjusted_rx_ticks % frame_ticks;

	/* Keep both receiver-clock phase and bias-corrected phase. The raw phase
	 * reveals per-tracker synchronization displacement that the corrected
	 * diagnostic intentionally removes. */
	int32_t slot_center = (int32_t)(expected_slot * slot_ticks + slot_ticks / 2);
	int32_t raw_offset = (int32_t)raw_frame_phase - slot_center;
	int32_t ref_offset = (int32_t)frame_phase - slot_center;
	if (raw_offset > (int32_t)(frame_ticks / 2)) {
		raw_offset -= frame_ticks;
	} else if (raw_offset < -(int32_t)(frame_ticks / 2)) {
		raw_offset += frame_ticks;
	}

	if (ref_offset > (int32_t)(frame_ticks / 2)) {
		ref_offset -= frame_ticks;
	} else if (ref_offset < -(int32_t)(frame_ticks / 2)) {
		ref_offset += frame_ticks;
	}

	struct tdma_stats *stats = &g_tdma_stats[tracker_id];
	stats->count++;

	/* Update receiver-time phase and RSSI distributions. */
	stats->sum_offset += ref_offset;
	stats->sum_sq_offset += ((int64_t)ref_offset * ref_offset);
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	uint32_t phase_bucket = (uint32_t)CLAMP(ref_offset - RX_PHASE_MIN_TICKS, 0, RX_PHASE_BUCKETS - 1);
	stats->phase_hist[phase_bucket]++;
	uint32_t raw_phase_bucket
		= (uint32_t)CLAMP(raw_offset - RX_PHASE_MIN_TICKS, 0, RX_PHASE_BUCKETS - 1);
	stats->raw_phase_hist[raw_phase_bucket]++;
	/* Nordic RSSISAMPLE is a positive attenuation magnitude (for example 60
	 * means -60 dBm), even though esb_payload stores it in int8_t. */
	uint32_t rssi_bucket = MIN((uint32_t)(uint8_t)rssi, RX_RSSI_MAX_DBM);
	stats->rssi_hist[rssi_bucket]++;
	stats->rssi_count++;
#endif
	ARG_UNUSED(rssi);
	ARG_UNUSED(raw_offset);

	if (stats->count == 1) {
		stats->min_offset = ref_offset;
		stats->max_offset = ref_offset;
	} else {
		stats->min_offset = MIN(stats->min_offset, ref_offset);
		stats->max_offset = MAX(stats->max_offset, ref_offset);
	}
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	if (stats->count == 1) {
		stats->raw_min_offset = raw_offset;
		stats->raw_max_offset = raw_offset;
	} else {
		stats->raw_min_offset = MIN(stats->raw_min_offset, raw_offset);
		stats->raw_max_offset = MAX(stats->raw_max_offset, raw_offset);
	}
#endif

	/*
	 * Phase-consistency violation detection.
	 *
	 * The absolute ref_offset includes a per-tracker constant bias from:
	 *   - D_est update after PONG (tracker updates offset, receiver's
	 *     clock_bias is stale until next PING)
	 *   - ISR-vs-EVENT timing differences
	 *   - Air-time asymmetry in RTT model
	 *
	 * Instead of checking absolute offset against slot boundaries, track
	 * a running mean (EMA) of each tracker's observed phase and flag
	 * DEVIATIONS from that mean.  This automatically adapts to D_est
	 * jumps and only fires for genuine TDMA failures (wrong slot, phase
	 * drift, collisions).
	 */
	if (!stats->phase_initialized) {
		stats->phase_ema_q8 = ref_offset << 8;
		stats->phase_initialized = true;
	} else {
		/* EMA alpha=1/8: converges in ~8 packets (~50ms at 170 TPS) */
		stats->phase_ema_q8 += (((ref_offset << 8) - stats->phase_ema_q8) + 4) >> 3;
	}

	int32_t phase_mean = stats->phase_ema_q8 >> 8;
	int32_t deviation = ref_offset - phase_mean;
	int32_t abs_dev = deviation < 0 ? -deviation : deviation;

	/* Violation: packet deviates from its own running mean by more than
	 * half a slot.  This catches actual TDMA failures while ignoring
	 * the constant per-tracker phase bias. */
	int32_t half_slot = (int32_t)(slot_ticks / 2);
	if (abs_dev > half_slot) {
		stats->violations++;

		if ((stats->violations & 0x0F) == 1) {
			LOG_DBG(
				"TDMA viol trk=%u dev=%d ema=%d off=%d bias=%d",
				tracker_id,
				deviation,
				phase_mean,
				ref_offset,
				clock_bias
			);
		}
	}
}
#else
/* When TDMA is disabled, provide a no-op stub */
static inline void tdma_check_slot(uint8_t tracker_id, uint32_t rx_ticks, int8_t rssi)
{
	ARG_UNUSED(tracker_id);
	ARG_UNUSED(rx_ticks);
	ARG_UNUSED(rssi);
}
#endif
#endif

static void esb_stats_thread(void);
/* The loss snapshot is inlined into this thread under LTO. TDMA log packaging
 * alone exceeds the former 512-byte stack; leave room for kernel/IRQ frames. */
K_THREAD_DEFINE(esb_stats_thread_id, 1024, esb_stats_thread, NULL, NULL, NULL, ESB_STATS_THREAD_PRIORITY, 0, 0);

static void esb_thread(void);
/* LTO startup frames plus transactional roster restore/NVS reads leave only
 * 16 bytes of the former 1024-byte stack, before deeper calls/exception entry. */
K_THREAD_DEFINE(esb_thread_id, 2048, esb_thread, NULL, NULL, NULL, ESB_THREAD_PRIORITY, 0, 0);

static bool esb_parse_pair(const uint8_t packet[8]);
static void process_pairing_queue(void);
/* Only result/error flags prove execution; legacy request echoes never do. */
static void remote_wom_receive(uint8_t id, uint8_t flag, const uint8_t data[4], uint8_t counter)
{
	if (flag != ESB_PING_FLAG_WOM_RESULT && flag != ESB_PING_FLAG_WOM_ERROR) {
		return;
	}
	uint16_t value = ((uint16_t)data[0] << 8) | data[1];
	uint16_t token = ((uint16_t)data[2] << 8) | data[3];
	unsigned int key = irq_lock();
	if (tracker_remote_command[id] == ESB_PONG_FLAG_WOM_CONTROL &&
	    token != 0 && token == wom_token[id] && !wom_complete[id] &&
	    wom_advertised[id] && counter != wom_first_ping_counter[id]) {
		bool success = flag == ESB_PING_FLAG_WOM_RESULT &&
			(wom_minutes[id] == ESB_WOM_ENTER ? value == ESB_WOM_ENTER :
			 wom_minutes[id] == ESB_WOM_TIMEOUT_CLEAR ? value == 0 :
			 (value >= 1 && value <= ESB_WOM_TIMEOUT_MAX_MINUTES &&
			  (wom_minutes[id] == 0 || wom_minutes[id] == value)));
		bool failure = flag == ESB_PING_FLAG_WOM_ERROR && value != 0;
		if (success || failure) {
			wom_minutes[id] = success ? value : 0;
			wom_error[id] = failure ? value : 0;
			wom_complete[id] = true;
			tracker_remote_command[id] = ESB_PONG_FLAG_NORMAL;
		}
	}
	irq_unlock(key);
}

static const char *esb_pong_flag_name(uint8_t flag);

/* ISR-safe roster snapshot; mutations publish address/count under irq_lock. */
static inline int esb_find_tracker(uint64_t addr)
{
	if (addr == 0) {
		return -1;
	}
	unsigned int key = irq_lock();
	uint8_t count = stored_trackers;
	for (int i = 0; i < count && i < MAX_TRACKERS; i++) {
		if (stored_tracker_addr[i] == addr) {
			irq_unlock(key);
			return i;
		}
	}
	irq_unlock(key);
	return -1;
}


static int check_packet_sequence(uint8_t tracker_id, uint8_t received_seq)
{
	if (tracker_id >= MAX_TRACKERS) {
		return 2;
	}

	struct packet_stats *stats = &tracker_stats[tracker_id];
	uint64_t current_time = k_uptime_get();
	// Update TPS calculation
	if (stats->last_tps_time == 0) {
		stats->last_tps_time = current_time;
		stats->packets_in_last_second = 0;
	} else if (current_time - stats->last_tps_time >= TPS_CALCULATION_INTERVAL_MS) {
		// Calculate TPS and reset counter
		stats->current_tps = stats->packets_in_last_second;
		stats->packets_in_last_second = 0;
		stats->last_tps_time = current_time;
	}

	// Each packet counts towards TPS calculation
	stats->packets_in_last_second++;
	stats->last_packet_time = current_time;

	// First packet, accept directly
	if (!sequence_initialized[tracker_id]) {
		// Update received packet count
		stats->total_received++;
		last_packet_sequence[tracker_id] = received_seq;
		sequence_initialized[tracker_id] = true;
		stats->normal_packets++;
		stats->last_sequence = received_seq;
		stats->first_packet = false;
		stats->status_received++;
		decision_accept(tracker_id, 0);
		LOG_DBG("First packet: tracker=%d, seq=%d", tracker_id, received_seq);
		return 0;
	}

	uint8_t last_seq = last_packet_sequence[tracker_id];
	uint8_t expected_seq = (last_seq + 1) & 0xFF;

	LOG_DBG(
		"Packet check: tracker=%d, received=%d, last=%d, expected=%d",
		tracker_id,
		received_seq,
		last_seq,
		expected_seq
	);
	// Normal next sequence number
	if (received_seq == expected_seq) {
		// Update received packet count
		stats->total_received++;
		last_packet_sequence[tracker_id] = received_seq;
		stats->normal_packets++;
		stats->last_sequence = received_seq;
		stats->status_received++;
		decision_accept(tracker_id, 0);
		LOG_DBG("Normal packet: tracker=%d, seq=%d", tracker_id, received_seq);
		return 0;
	}

	// Calculate sequence difference (considering wrap-around)
	uint8_t diff_forward = (received_seq - last_seq) & 0xFF;  // Forward difference (including wrap-around)
	uint8_t diff_backward = (last_seq - received_seq) & 0xFF; // Backward difference (including wrap-around)

	if (diff_forward == 0) {
		// Same sequence number - this is a true duplicate packet
		stats->duplicate_packets++;
		return 4; // Duplicate packet
	}

	// Backward jump (old packet) is considered out-of-order, keep current window unchanged
	// Note: Out-of-order packets are not counted in total_received as they are dropped and not forwarded to the
	// application layer Check condition: small diff_backward (< 64) indicates a true backward jump (old packet)
	if (diff_backward > 0 && diff_backward < 64) {
		stats->out_of_order++;
		// Log single out-of-order event only at DEBUG level to reduce output
		LOG_DBG(
			"Out-of-order packet dropped: tracker=%d, seq=%d (expected >%d), "
			"backward=%d",
			tracker_id,
			received_seq,
			last_seq,
			diff_backward
		);
		return 2;
	}

	// Detect large jump (possibly a restart)
	// If the jump exceeds 80, it's likely a tracker restart or wrap-around
	// In this case, a large number of gaps should not be accumulated
	if (diff_forward > 80) {
		// This is a large jump, treated as a tracker restart
		stats->total_received++;
		stats->restart_events++;
		decision_ready_mask &= ~BIT(tracker_id);
		decision_fence_mask |= BIT(tracker_id);
		last_packet_sequence[tracker_id] = received_seq;
		stats->last_sequence = received_seq;
		stats->status_received++;
		// Restart events are kept at WARNING level because this is important information
		LOG_WRN(
			"Tracker restart detected: tracker=%d, last_seq=%d, new_seq=%d, jump=%d",
			tracker_id,
			last_seq,
			received_seq,
			diff_forward
		);
		return 3; // Restart event
	}

	// Forward jump (normal packet loss range)
	if (diff_forward > 0 && diff_forward <= 80) {
		stats->total_received++;
		stats->gap_events++;
		uint8_t gaps = diff_forward - 1;
		stats->total_gaps += gaps; // Estimate number of lost packets
		stats->status_received++;
		stats->status_lost += gaps;
		decision_accept(tracker_id, gaps);
		last_packet_sequence[tracker_id] = received_seq;
		stats->last_sequence = received_seq;
		// Log single gap only at DEBUG level to reduce output (summary statistics will show total gaps)
		LOG_DBG("Gap detected: tracker=%d, seq=%d, gap=%d (forward=%d)", tracker_id, received_seq, gaps, diff_forward);
		return 1;
	}

	// Default case, should not be reached; treat as duplicate for safety
	stats->duplicate_packets++;
	return 4;
}

// Prints statistics for a specific tracker
// Prints statistics for a single tracker
static void print_tracker_stats(uint8_t tracker_id)
{
	struct packet_stats *stats = &tracker_stats[tracker_id];

	if (stats->total_received == 0 && stats->duplicate_packets == 0) {
		return;
	}

	// Total received packets (including duplicates and out-of-order)
	uint32_t total_receives = stats->total_received + stats->duplicate_packets + stats->out_of_order;

	// Calculate various rates (using permille to avoid floating-point operations)
	uint32_t duplicate_rate = 0;
	uint32_t out_of_order_rate = 0;

	if (total_receives > 0) {
		duplicate_rate = (stats->duplicate_packets * 1000) / total_receives;
		out_of_order_rate = (stats->out_of_order * 1000) / total_receives;
	}

	// Estimate packet loss rate: based on total gaps and received packets
	uint32_t estimated_sent = stats->total_received + stats->total_gaps;
	uint32_t estimated_loss_rate = 0;
	if (estimated_sent > 0) {
		estimated_loss_rate = (stats->total_gaps * 1000) / estimated_sent;
	}

	LOG_INF(
		"Tracker %d: Recv=%u(+%u dup +%u ooo), Normal=%u, EstLoss=%u.%u%% (%u gaps), "
		"Dup=%u.%u%%, OOO=%u.%u%%, Restart=%u, TPS=%u",
		tracker_id,
		stats->total_received,
		stats->duplicate_packets,
		stats->out_of_order,
		stats->normal_packets,
		estimated_loss_rate / 10,
		estimated_loss_rate % 10,
		stats->total_gaps,
		duplicate_rate / 10,
		duplicate_rate % 10,
		out_of_order_rate / 10,
		out_of_order_rate % 10,
		stats->restart_events,
		stats->current_tps
	);
}

static void print_tracker_stats_batch(void)
{
	for (int i = 0; i < MAX_TRACKERS; i++) {
		if (tracker_stats[i].total_received > 0 || tracker_stats[i].duplicate_packets > 0) {
			print_tracker_stats(i);
		}
	}
}

static void esb_stats_thread(void)
{
	last_tps_print_time = k_uptime_get();
	int64_t next_monitor_time = last_tps_print_time + TPS_MONITOR_INTERVAL_MS;

	while (1) {
		int64_t until_monitor = next_monitor_time - k_uptime_get();
		uint32_t wait_ms = until_monitor > 0
			? MIN((uint32_t)until_monitor, TPS_MONITOR_INTERVAL_MS) : 0;
		wait_ms = tdma_membership_wait_ms(wait_ms);
		(void)k_sem_take(&tdma_membership_wake, K_MSEC(wait_ms));

		uint64_t now = (uint64_t)k_uptime_get();
		if ((int64_t)now < next_monitor_time) {
			k_mutex_lock(&tracker_store_lock, K_FOREVER);
			tdma_shadow_update(k_uptime_get_32());
			tdma_recalculate(false);
			k_mutex_unlock(&tracker_store_lock);
			continue;
		}
		next_monitor_time = (int64_t)now + TPS_MONITOR_INTERVAL_MS;

		// Update TPS for each tracker
		for (int i = 0; i < MAX_TRACKERS; i++) {
			struct packet_stats *stats = &tracker_stats[i];
			if (stats->last_tps_time == 0) {
				continue;
			}
			if (now - stats->last_tps_time >= TPS_CALCULATION_INTERVAL_MS) {
				if (stats->packets_in_last_second > 0) {
					stats->current_tps = stats->packets_in_last_second;
				} else if (stats->last_packet_time && now - stats->last_packet_time >= TPS_CALCULATION_INTERVAL_MS) {
					stats->current_tps = 0;
				}
				stats->packets_in_last_second = 0;
				stats->last_tps_time = now;
			}
		}

		// Check if detailed stats should be auto-disabled
		if (stats_detailed_enabled && stats_detailed_end_time > 0) {
			if (now >= stats_detailed_end_time) {
				stats_detailed_enabled = false;
				stats_detailed_end_time = 0;
				LOG_INF("Detailed stats auto-disabled");
			}
		}

		// Print total TPS every second (always)
		if (now - last_tps_print_time >= TPS_CALCULATION_INTERVAL_MS) {
			uint32_t total_tps = 0;
			for (int i = 0; i < MAX_TRACKERS; i++) {
				total_tps += tracker_stats[i].current_tps;
			}
			LOG_INF("Total TPS: %u", total_tps);
			last_tps_print_time = now;

			k_mutex_lock(&tracker_store_lock, K_FOREVER);
			int64_t policy_now = k_uptime_get();
			tdma_shadow_update((uint32_t)policy_now);
			tdma_loss_controller_tick(policy_now);
			tdma_recalculate(false);
			test_all_reconcile(policy_now);
			k_mutex_unlock(&tracker_store_lock);
		} else {
			k_mutex_lock(&tracker_store_lock, K_FOREVER);
			tdma_shadow_update(k_uptime_get_32());
			tdma_recalculate(false);
			k_mutex_unlock(&tracker_store_lock);
		}

		// Print detailed stats only when enabled
		if (stats_detailed_enabled) {
			static int64_t last_detailed_log_time = 0;
			if (last_detailed_log_time == 0) {
				last_detailed_log_time = now;
			}

			if (now - last_detailed_log_time >= STATS_PRINT_INTERVAL_MS) {
				bool has_data = false;
				for (int i = 0; i < MAX_TRACKERS; i++) {
					if (tracker_stats[i].total_received > 0) {
						has_data = true;
						break;
					}
				}

				if (has_data) {
					LOG_INF("=== Packet Statistics ===");
					print_tracker_stats_batch();
				}
				last_detailed_log_time = now;
			}
		}
	}
}

/* ---------------------------------------------------------------------------
 * Raw data ARQ — sequence tracking and retransmit requests via ACK payload.
 *
 * All state is volatile for ISR access.  Only the target tracker is tracked.
 * Gap queue holds up to RAW_ARQ_MAX_GAPS missing sequence numbers.
 * Each ACK payload carries up to RAW_ARQ_MAX_GAPS retransmit requests.
 *
 * ACK payload format for raw data (type 0x10):
 *   [0]    = 0xAA  (marker byte)
 *   [1]    = count  (0 = all OK, 1..4 = number of seqs to retransmit)
 *   [2..3] = missing_seq_0 (BE16)
 *   [4..5] = missing_seq_1 (BE16)  (if count >= 2)
 *   ...
 * -------------------------------------------------------------------------*/
#define RAW_ARQ_MAX_GAPS 8
#define RAW_ARQ_MARKER 0xAA
/* Maximum sequence distance before a gap is considered stale and
 * unrecoverable. The tracker retains 128 packets; a 96-packet cutoff leaves
 * 32 packets of reserve for ACK processing and transport delay. */
#define RAW_ARQ_STALE_DISTANCE 96

static volatile uint16_t raw_arq_expected_seq;
static volatile bool raw_arq_seq_initialized;
static volatile uint16_t raw_arq_gap_queue[RAW_ARQ_MAX_GAPS];
static volatile uint8_t raw_arq_gap_count;
/* Counters for diagnostics (read by event_handler, not ISR-critical) */
static volatile uint32_t raw_arq_gaps_detected;
static volatile uint32_t raw_arq_retransmits_received;

/* Called from esb_ack_handler_cb (ISR context) when raw data packet arrives.
 * Updates gap queue and builds ACK payload with retransmit requests. */
static void raw_arq_process_isr(uint16_t received_seq, struct esb_payload *ack_payload, bool *has_ack_payload)
{
	if (!raw_arq_seq_initialized) {
		raw_arq_expected_seq = received_seq + 1;
		raw_arq_seq_initialized = true;
		*has_ack_payload = false;
		return;
	}

	uint16_t expected = raw_arq_expected_seq;
	int16_t diff = (int16_t)(received_seq - expected);

	if (diff == 0) {
		/* In order — advance expected */
		raw_arq_expected_seq = received_seq + 1;
	} else if (diff > 0 && diff <= 100) {
		/* Forward gap: diff packets missing */
		uint8_t remaining = RAW_ARQ_MAX_GAPS - raw_arq_gap_count;
		uint16_t to_add = (uint16_t)diff;
		if (to_add > remaining) {
			to_add = remaining;
		}
		for (uint16_t i = 0; i < to_add; i++) {
			raw_arq_gap_queue[raw_arq_gap_count++] = (uint16_t)(expected + i);
		}
		raw_arq_gaps_detected += (uint32_t)diff;
		raw_arq_expected_seq = received_seq + 1;
	} else if (diff < 0 && diff > -100) {
		/* Retransmitted (out-of-order) packet: remove from gap queue */
		for (uint8_t i = 0; i < raw_arq_gap_count; i++) {
			if (raw_arq_gap_queue[i] == received_seq) {
				/* O(1) swap-delete — ISR must stay short */
				raw_arq_gap_queue[i] = raw_arq_gap_queue[raw_arq_gap_count - 1];
				raw_arq_gap_count--;
				raw_arq_retransmits_received++;
				break;
			}
		}
		/* Don't advance expected_seq for retransmits */
	} else {
		/* Large jump — treat as reset/restart */
		raw_arq_expected_seq = received_seq + 1;
		raw_arq_gap_count = 0;
	}

	/* Evict stale gap entries that are too far behind expected_seq.
	 * These sequences have been overwritten in the tracker's ring buffer
	 * and can never be retransmitted. Without eviction, stale entries
	 * permanently occupy queue slots, blocking new gap tracking. */
	{
		uint16_t cur_expected = raw_arq_expected_seq;
		uint8_t write_idx = 0;
		for (uint8_t i = 0; i < raw_arq_gap_count; i++) {
			uint16_t age = (uint16_t)(cur_expected - raw_arq_gap_queue[i]);
			if (age < RAW_ARQ_STALE_DISTANCE) {
				raw_arq_gap_queue[write_idx++] = raw_arq_gap_queue[i];
			}
		}
		raw_arq_gap_count = write_idx;
	}

	/* Build ACK payload with pending retransmit requests */
	if (raw_arq_gap_count > 0) {
		uint8_t n = raw_arq_gap_count;
		ack_payload->pipe = 0; /* will be overridden by caller */
		ack_payload->length = 2 + n * 2;
		ack_payload->noack = false;
		ack_payload->data[0] = RAW_ARQ_MARKER;
		ack_payload->data[1] = n;
		for (uint8_t i = 0; i < n; i++) {
			sys_put_be16(raw_arq_gap_queue[i], &ack_payload->data[2 + i * 2]);
		}
		*has_ack_payload = true;
	} else {
		*has_ack_payload = false;
	}
}

static void raw_arq_reset(void)
{
	raw_arq_seq_initialized = false;
	raw_arq_gap_count = 0;
	raw_arq_gaps_detected = 0;
	raw_arq_retransmits_received = 0;
}

/* ---------------------------------------------------------------------------
 * ACK handler — runs in radio ISR context (~130 µs budget).
 * Builds the ACK payload for the *current* packet so the response is
 * returned in the same transaction rather than the next one.
 *
 * Only reads pre-computed volatile state written by event_handler / threads.
 * No blocking, no logging, no locks.
 * -------------------------------------------------------------------------*/
static void esb_ack_handler_cb(
	const uint8_t *pdu_data,
	uint8_t data_length,
	uint32_t pipe_id,
	struct esb_payload *ack_payload,
	bool *has_ack_payload,
	bool *suppress_ack
)
{
	*has_ack_payload = false;
	*suppress_ack = false;

	/* Consume the whole beacon family, even malformed/wrong-pipe frames. */
	if (auto_channel_is_beacon(pdu_data, data_length)) {
		*suppress_ack = true;
		return;
	}

	/* ---- Discovery pairing packets on pipe 0 ---- */
	if (pipe_id == 0 && data_length == 8) {
		uint8_t step = pdu_data[1];
		uint64_t raw_addr = 0;
		uint64_t found_addr = 0;
		int known_id = -1;
		uint8_t checksum = crc8_ccitt(0x07, &pdu_data[2], 6);

		if (checksum == 0) {
			checksum = 8;
		}

		memcpy(&raw_addr, pdu_data, sizeof(raw_addr));
		found_addr = (raw_addr >> 16) & 0xFFFFFFFFFFFFULL;

		if (checksum != pdu_data[0] || found_addr == 0) {
			*suppress_ack = true;
			return;
		}

		known_id = esb_find_tracker(found_addr);

		if (!esb_pairing) {
			*suppress_ack = true;
			return;
		}

		if (pairing_new_devices_blocked && known_id < 0) {
			*suppress_ack = true;
			return;
		}

		if (step == 0) {
			return;
		}

		if (step == 1) {
			if (known_id < 0) {
				*suppress_ack = true;
				return;
			}

			ack_payload->pipe = pipe_id;
			ack_payload->length = 8;
			ack_payload->noack = false;
			ack_payload->data[0] = checksum;
			ack_payload->data[1] = (uint8_t)known_id;
			memcpy(&ack_payload->data[2], receiver_device_addr, 6);
			*has_ack_payload = true;
			return;
		}

		if (step == 2) {
			if (known_id < 0) {
				*suppress_ack = true;
			}
			return;
		}

		*suppress_ack = true;
		return;
	}

#if defined(CONFIG_TDMA_DIAGNOSTICS)
	/* ---- Capture ISR timestamp for any tracker packet (non-pairing) ---- */
	if (data_length > 1 && pipe_id > 0) {
		uint8_t tid = pdu_data[1];
		if (tid < MAX_TRACKERS) {
			g_last_isr_rx_ticks[tid] = k_uptime_ticks();
			g_last_isr_rx_valid[tid] = true;
		}
	}
#endif

	/* ---- PING → PONG (immediate response) ---- */
	if (data_length == ESB_PING_LEN && pdu_data[0] == ESB_PING_TYPE) {
		uint8_t tracker_id = pdu_data[1];
		if (tracker_id >= MAX_TRACKERS) {
			return;
		}

		uint8_t crc = crc8_ccitt(0x07, pdu_data, ESB_PING_LEN - 1);
		if (pdu_data[ESB_PING_LEN - 1] != crc) {
			return;
		}
		uint8_t counter = pdu_data[2];
		bool channel_confirm = (pdu_data[7] & ESB_PING_FLAG_CHANNEL_CONFIRM) != 0;
		uint32_t rx_ticks = k_uptime_ticks();
		uint8_t cmd = ESB_PONG_FLAG_CHANNEL_CONFIRM;
		bool ota_aborting = false;
		if (!channel_confirm) {
			cmd = tracker_remote_command[tracker_id];
			/* Keep an active metadata request on duplicate PINGs until its echo
			 * arrives; channel probes must leave both pending and active tuples alone. */
			unsigned int metadata_key = irq_lock();
			if (cmd == ESB_PONG_FLAG_NORMAL && metadata_active_mask[tracker_id] != 0) {
				cmd = ESB_PONG_FLAG_METADATA_REQUEST;
			} else if (cmd == ESB_PONG_FLAG_NORMAL && metadata_pending_mask[tracker_id] != 0) {
				metadata_active_mask[tracker_id] = metadata_pending_mask[tracker_id];
				metadata_active_chunk[tracker_id] = metadata_pending_chunk[tracker_id];
				metadata_active_token[tracker_id] = metadata_pending_token[tracker_id];
				metadata_pending_mask[tracker_id] = 0;
				cmd = ESB_PONG_FLAG_METADATA_REQUEST;
			}
			irq_unlock(metadata_key);
			uint8_t migration_cmd = channel_command_for_tracker(tracker_id);
			if (migration_cmd != ESB_PONG_FLAG_NORMAL) {
				cmd = migration_cmd;
			}
			/* In single-target collection, shut down non-target trackers. */
			if (data_collect_is_active() && !data_collect_is_target(tracker_id)
				&& !data_collect_batch_is_active()) {
				cmd = ESB_PONG_FLAG_SHUTDOWN;
			}
			/* Keep rebooted non-participating trackers suppressed during OTA. */
			if (cmd == ESB_PONG_FLAG_NORMAL && esb_ota_relay_is_active() && !esb_ota_relay_is_target(tracker_id)) {
				cmd = ESB_PONG_FLAG_OTA_SUPPRESS;
			}
			ota_aborting = esb_ota_relay_abort_pending(tracker_id);
			if (ota_aborting) {
				cmd = ESB_PONG_FLAG_OTA_ABORT;
			}
			/* Drain a previous tracker outcome before issuing a new WOM
			 * transaction. In particular, receiver reboot must not reuse a
			 * token against a still-cached success or error on the tracker. */
			if (cmd == ESB_PONG_FLAG_WOM_CONTROL && !wom_advertised[tracker_id] &&
			    (pdu_data[7] & 0x7f) != ESB_PONG_FLAG_NORMAL) {
				cmd = ESB_PONG_FLAG_NORMAL;
			}
		}
#if defined(CONFIG_TDMA_DIAGNOSTICS)
		if (pipe_id == 1U + (tracker_id % 7)) {
			g_ping_isr_rx_ticks[tracker_id] = rx_ticks;
			g_ping_isr_rx_ticks_valid[tracker_id] = true;
		}
#endif

		ack_payload->pipe = 1 + (tracker_id % 7);
		ack_payload->length = ESB_PONG_LEN;
		ack_payload->noack = false;

		ack_payload->data[0] = ESB_PONG_TYPE;
		ack_payload->data[1] = tracker_id;
		ack_payload->data[2] = counter;
		ack_payload->data[7] = cmd;

		if (cmd == ESB_PONG_FLAG_SENS_SET) {
			/* SENS_SET overrides time sync bytes with sensitivity data */
			int16_t s0 = pending_sens_data[tracker_id][0];
			int16_t s1 = pending_sens_data[tracker_id][1];
			int16_t s2 = pending_sens_data[tracker_id][2];
			ack_payload->data[3] = (s0 >> 8) & 0xFF;
			ack_payload->data[4] = (s0) & 0xFF;
			ack_payload->data[5] = (s1 >> 8) & 0xFF;
			ack_payload->data[6] = (s1) & 0xFF;
			ack_payload->data[8] = (s2 >> 8) & 0xFF;
			ack_payload->data[9] = (s2) & 0xFF;
			ack_payload->data[10] = 0;
			ack_payload->data[11] = 0;
		} else if (cmd == ESB_PONG_FLAG_SENS_AUTO) {
			uint16_t rev = pending_sens_auto_revolutions[tracker_id];
			ack_payload->data[3] = pending_sens_auto_axis[tracker_id];
			ack_payload->data[4] = (rev >> 8) & 0xFF;
			ack_payload->data[5] = (rev) & 0xFF;
			ack_payload->data[6] = 0;
			ack_payload->data[8] = 0;
			ack_payload->data[9] = 0;
			ack_payload->data[10] = 0;
			ack_payload->data[11] = 0;
		} else {
			/* Normal: embed receiver timestamp for time sync */
			ack_payload->data[3] = (rx_ticks >> 24) & 0xFF;
			ack_payload->data[4] = (rx_ticks >> 16) & 0xFF;
			ack_payload->data[5] = (rx_ticks >> 8) & 0xFF;
			ack_payload->data[6] = (rx_ticks) & 0xFF;

			if (cmd == ESB_PONG_FLAG_SET_CHANNEL) {
				uint32_t ch = esb_rf_channel_decode(pending_channel);
				ack_payload->data[8] = (ch >> 24) & 0xFF;
				ack_payload->data[9] = (ch >> 16) & 0xFF;
				ack_payload->data[10] = (ch >> 8) & 0xFF;
				ack_payload->data[11] = (ch) & 0xFF;
			} else if (cmd == ESB_PONG_FLAG_WOM_CONTROL) {
				ack_payload->data[8] = wom_minutes[tracker_id] >> 8;
				ack_payload->data[9] = wom_minutes[tracker_id] & 0xff;
				ack_payload->data[10] = wom_token[tracker_id] >> 8;
				ack_payload->data[11] = wom_token[tracker_id] & 0xff;
			} else if (cmd == ESB_PONG_FLAG_TEST_MODE_ON) {
				ack_payload->data[8] = (tracker_test_on_tps[tracker_id] >> 8) & 0xFF;
				ack_payload->data[9] = tracker_test_on_tps[tracker_id] & 0xFF;
				ack_payload->data[10] = 0;
			} else if (cmd == ESB_PONG_FLAG_METADATA_REQUEST) {
				ack_payload->data[8] = metadata_active_mask[tracker_id];
				ack_payload->data[9] = metadata_active_chunk[tracker_id];
				uint16_t token = metadata_active_token[tracker_id];
				ack_payload->data[10] = (token >> 8) & 0xFF;
				ack_payload->data[11] = token & 0xFF;
			} else if (cmd == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON) {
				ack_payload->data[8] = pending_cmd_arg[tracker_id];
				ack_payload->data[9] = 0;
				ack_payload->data[10] = 0;
				ack_payload->data[11] = 0;
			} else if (cmd == ESB_PONG_FLAG_CHANNEL_CONFIRM) {
				uint32_t channel = UINT32_MAX;
				if (esb_get_rf_channel(&channel) || channel > 100) {
					return;
				}
				ack_payload->data[8] = (uint8_t)channel;
				ack_payload->data[9] = ESB_CHANNEL_CONFIRM_VERSION;
				ack_payload->data[10] = 0;
				ack_payload->data[11] = 0;
			} else if (cmd == ESB_PONG_FLAG_NORMAL) {
				/* Preserve the legacy TDMA tuple byte for byte. */
				uint32_t cfg = tdma_config_packed[tracker_id];
				ack_payload->data[8] = (cfg >> 24) & 0xFF;
				ack_payload->data[9] = (cfg >> 16) & 0xFF;
				ack_payload->data[10] = (cfg >> 8) & 0xFF;
				ack_payload->data[11] = (cfg) & 0xFF;
			} else {
				memset(&ack_payload->data[8], 0, 4);
			}
		}

		ack_payload->data[ESB_PONG_LEN - 1] = crc8_ccitt(0x07, ack_payload->data, ESB_PONG_LEN - 1);
		*has_ack_payload = true;

		/* Preserve the ordinary PONG counter and clock stamp on cancellation. */
		if (ota_aborting) {
			return;
		}

		/* If OTA session has a pending command for this tracker
		 * (e.g., BEGIN before tracker enters OTA mode), override the
		 * standard PONG with the OTA payload. */
		if (!channel_confirm && esb_ota_relay_is_active() && esb_ota_relay_is_target(tracker_id)) {
			bool ota_has_ack = false;
			esb_ota_relay_fill_ack(tracker_id, pipe_id, ack_payload, &ota_has_ack, NULL, 0);
			if (ota_has_ack) {
				*has_ack_payload = true;
			}
		}
		/* Arm only the final selected request, never a PONG replaced by OTA.
		 * The incoming PING predates this ACK and cannot confirm it. */
		if (*has_ack_payload && ack_payload->length == ESB_PONG_LEN &&
		    ack_payload->data[0] == ESB_PONG_TYPE &&
		    ack_payload->data[7] == ESB_PONG_FLAG_WOM_CONTROL &&
		    tracker_remote_command[tracker_id] == ESB_PONG_FLAG_WOM_CONTROL &&
		    wom_token[tracker_id] != 0 &&
		    (((uint16_t)ack_payload->data[8] << 8) | ack_payload->data[9]) == wom_minutes[tracker_id] &&
		    (((uint16_t)ack_payload->data[10] << 8) | ack_payload->data[11]) == wom_token[tracker_id] &&
		    !wom_advertised[tracker_id]) {
			wom_first_ping_counter[tracker_id] = counter;
			wom_advertised[tracker_id] = true;
		}
		return;
	}

	/* Other packet types: no ACK payload, handled by event_handler */

	/* ---- OTA status/poll packets from tracker in OTA mode ---- */
	if (data_length >= 2 && pdu_data[0] == ESB_OTA_STATUS_TYPE) {
		uint8_t tracker_id = pdu_data[1];
		esb_ota_relay_fill_ack(tracker_id, pipe_id, ack_payload, has_ack_payload, pdu_data, data_length);
		return;
	}

	/* ---- Raw data ARQ (type 0x10/0x13, single-target collection only) ---- */
	if (data_length >= 4 && (pdu_data[0] == ESB_RAW_IMU_TYPE || pdu_data[0] == ESB_RAW_IMU_QUAT_TYPE)) {
		uint8_t tracker_id = pdu_data[1];
		if (data_collect_is_target(tracker_id) && !data_collect_batch_is_target(tracker_id)) {
			raw_arq_process_isr(sys_get_be16(&pdu_data[2]), ack_payload, has_ack_payload);
			if (*has_ack_payload) {
				ack_payload->pipe = pipe_id;
			}
		}
	}
}

static int composite_body_length(uint8_t type)
{
	switch (type) {
	case 0: return 13; /* info */
	case 1: return 14; /* quat+accel */
	case 2: return 13; /* compact quat */
	case 3: return 2;  /* status */
	case 4: return 14; /* quat+mag */
	case 5: return 8;  /* runtime */
	case TRACKER_EVENT_ESB_TYPE: return TRACKER_EVENT_BODY_LEN;
	default: return -1;
	}
}

#if defined(CONFIG_TDMA_DIAGNOSTICS)
static void account_ping_clock(uint8_t tracker_id, uint32_t expected_rx_ticks, uint32_t current_rx_ticks)
{
	struct tdma_stats *stats = &g_tdma_stats[tracker_id];

	/* Use RADIO ISR timestamp to avoid EVENT_IRQ scheduling jitter. */
	uint32_t isr_rx_ticks
		= g_ping_isr_rx_ticks_valid[tracker_id] ? g_ping_isr_rx_ticks[tracker_id] : current_rx_ticks;
	int32_t rx_time_diff_ticks = (int32_t)(isr_rx_ticks - expected_rx_ticks);

	/* The tracker estimates TX start, but the ISR samples after the complete
	 * PING. Remove the 13-byte 2 Mbps on-air time. Retransmissions have an
	 * additional retry interval and are excluded by the clean-PING gate. */
	int32_t prev_bias = g_last_ping_rx_time_diff_valid[tracker_id]
		? g_last_ping_rx_time_diff_ticks[tracker_id] : 0;
	#define PING_TX_START_TO_RX_TICKS 5

	bool clean_ping;
	if (!g_last_ping_rx_time_diff_valid[tracker_id]) {
		clean_ping = true;
	} else if (rx_time_diff_ticks <= g_last_ping_rx_time_diff_ticks[tracker_id]
					     + PING_TX_START_TO_RX_TICKS + 2) {
		g_ping_dirty_streak[tracker_id] = 0;
		clean_ping = true;
	} else {
		g_ping_reject_count[tracker_id]++;
		if (++g_ping_dirty_streak[tracker_id] >= PING_CLEAN_REACQUIRE_STREAK) {
			g_ping_dirty_streak[tracker_id] = 0;
			g_bias_ppb_valid[tracker_id] = 0;
			prev_bias = 0;
			clean_ping = true;
		} else {
			clean_ping = false;
		}
	}
	if (clean_ping) {
		g_last_ping_rx_time_diff_ticks[tracker_id] = rx_time_diff_ticks - PING_TX_START_TO_RX_TICKS;
		g_last_ping_rx_time_diff_valid[tracker_id] = true;
	}

	if (clean_ping && prev_bias != 0) {
		if (!g_bias_ppb_valid[tracker_id]) {
			g_bias_ref_offset[tracker_id] = g_last_ping_rx_time_diff_ticks[tracker_id];
			g_bias_ref_ticks[tracker_id] = isr_rx_ticks;
			g_bias_ppb[tracker_id] = 0;
			g_bias_ppb_valid[tracker_id] = 1;
		} else {
			uint32_t elapsed = isr_rx_ticks - g_bias_ref_ticks[tracker_id];
			if (elapsed >= 32768u) {
				int32_t new_bias = g_last_ping_rx_time_diff_ticks[tracker_id];
				int64_t total_drift = (int64_t)(new_bias - g_bias_ref_offset[tracker_id]);
				int32_t raw_ppb = (int32_t)(total_drift * 1000000000LL / elapsed);
				if (g_bias_ppb_valid[tracker_id] > 1) {
					g_bias_ppb[tracker_id] += (raw_ppb - g_bias_ppb[tracker_id]) / 4;
				} else {
					g_bias_ppb[tracker_id] = raw_ppb;
					g_bias_ppb_valid[tracker_id] = 2;
				}
			}
			if (elapsed > 60u * 32768u) {
				g_bias_ref_offset[tracker_id] = g_last_ping_rx_time_diff_ticks[tracker_id];
				g_bias_ref_ticks[tracker_id] = isr_rx_ticks;
			}
		}
	}
	if (clean_ping) {
		g_last_ping_isr_rx_ticks_raw[tracker_id] = isr_rx_ticks;
	}

#if TDMA_ENABLED
	if (stats->count > 0 && esb_get_stats_detailed_enabled()) {
		uint64_t rx_time_diff_us = k_ticks_to_us_floor64(
			(rx_time_diff_ticks < 0 ? -rx_time_diff_ticks : rx_time_diff_ticks)
		);
		int64_t mean = stats->sum_offset / stats->count;
		int64_t mean_sq = mean * mean;
		uint64_t variance = 0;
		if (stats->sum_sq_offset / stats->count >= (uint64_t)mean_sq) {
			variance = (stats->sum_sq_offset / stats->count) - (uint64_t)mean_sq;
		}
		uint32_t std_dev = 0;
		if (variance > 0) {
			uint64_t s = variance / 2;
			if (s > 0) {
				uint64_t x = s;
				uint64_t y = (x + variance / x) / 2;
				while (y < x) {
					x = y;
					y = (x + variance / x) / 2;
				}
				std_dev = (uint32_t)x;
			} else {
				std_dev = (uint32_t)variance;
			}
		}

		if (stats->violations > 12) {
			LOG_WRN(
				"TDMA Stats ID=%u Count=%u Viol=%u Mean=%lld StdDev=%u EMA=%d "
				"Range=[%d,%d] RxDiff=%s%llu us",
				tracker_id,
				stats->count,
				stats->violations,
				mean,
				std_dev,
				stats->phase_initialized ? (stats->phase_ema_q8 >> 8) : 0,
				stats->min_offset,
				stats->max_offset,
				rx_time_diff_ticks >= 0 ? "+" : "-",
				rx_time_diff_us
			);
		}
	}
#else
	ARG_UNUSED(rx_time_diff_ticks);
	ARG_UNUSED(stats);
#endif
}
#endif

static void esb_receive_ping(const struct esb_payload *payload, uint32_t event_ticks)
{
	uint8_t tracker_id = payload->data[1];
	uint8_t counter = payload->data[2];

	uint8_t ping_ack_flag = payload->data[7] & ESB_PING_COMMAND_MASK;

	/* ack_handler clamps; this path must too before any [MAX_TRACKERS] index. */
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}

	if (payload->pipe != 1 + (tracker_id % 7)) {
		pipe_mismatch_count[tracker_id]++;
		LOG_DBG(
			"PING pipe mismatch (x%u): id=%u, expected "
			"pipe=%u got pipe=%u",
			pipe_mismatch_count[tracker_id],
			tracker_id,
			1 + (tracker_id % 7),
			payload->pipe
		);
	}

	// check crc for PING
	uint8_t crc_calc = crc8_ccitt(0x07, payload->data, ESB_PING_LEN - 1);
	if (payload->data[ESB_PING_LEN - 1] != crc_calc) {
		// CRC error - only log warning if consecutive errors occur
		crc_error_count[tracker_id]++;

		LOG_DBG(
			"PING CRC mismatch (x%u): id=%u expected %02X got %02X",
			crc_error_count[tracker_id],
			tracker_id,
			crc_calc,
			payload->data[ESB_PING_LEN - 1]
		);
		return;
	}
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	uint32_t expected_rx_ticks = sys_get_be32(&payload->data[3]);
#else
	ARG_UNUSED(event_ticks);
#endif
	if (payload->pipe == 1 + (tracker_id % 7)) {
#if defined(CONFIG_TDMA_DIAGNOSTICS)
		account_ping_clock(tracker_id, expected_rx_ticks, event_ticks);
#endif
		tdma_shadow_observe(tracker_id, TDMA_SHADOW_EVIDENCE_PING);
	}
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	// Calculate signed ticks difference
	int32_t ticks_diff = (int32_t)event_ticks - (int32_t)expected_rx_ticks;
	// Get absolute value for conversion to microseconds
	uint32_t ticks_diff_abs = (uint32_t)(ticks_diff < 0 ? -ticks_diff : ticks_diff);
	LOG_DBG(
		"Tracker %u PING ctr=%u ticks_offset=%s%d us",
		tracker_id,
		counter,
		ticks_diff >= 0 ? "+" : "-",          // Add sign to the log
		k_ticks_to_us_floor32(ticks_diff_abs) // Convert absolute tick difference to microseconds
	);
#endif

	// First PING received for this tracker - accept directly, no sequence check
	if (!ping_counter_initialized[tracker_id]) {
		ping_counter_initialized[tracker_id] = true;
		last_ping_counter[tracker_id] = counter;
		last_ping_time[tracker_id] = k_uptime_get();
		last_pong_queued_counter[tracker_id] = 0xFF; // Mark as not queued
		LOG_DBG("First PING from tracker %u, ctr=%u, initializing", tracker_id, counter);
		// Continue processing, send PONG
	} else {
		// Already initialized, perform sequence check
		uint64_t current_time = k_uptime_get();
		bool is_duplicate = false;
		bool is_out_of_order = false;
		bool is_large_gap = false;
		bool is_tracker_restart = false;

		// Check for timeout - if no PING received for more than 5 seconds, reset expectation
		if (last_ping_time[tracker_id] > 0
			&& (current_time - last_ping_time[tracker_id]) > PING_TIMEOUT_MS) {
			LOG_WRN(
				"PING timeout (%llu ms), resetting tracker %u counter tracking",
				current_time - last_ping_time[tracker_id],
				tracker_id
			);
			last_ping_counter[tracker_id] = counter;
			last_ping_time[tracker_id] = current_time;
			last_pong_queued_counter[tracker_id] = 0xFF;
			if (atomic_get(&test_all_state_valid)) {
				test_all_invalidate_tracker(tracker_id, (int64_t)current_time);
			}
			// Continue processing this PING
		} else {
			// Calculate difference
			int counter_diff = (int)counter - (int)last_ping_counter[tracker_id];
			if (counter_diff < 0) {
				counter_diff += 256; // Handle wrap-around
			}

			if (counter_diff == 0) {
				// Duplicate packet
				is_duplicate = true;
			} else if (counter_diff >= 1 && counter_diff <= 100) {
				// Normal progression (possible packet loss)
				if (counter_diff > 5) {
					is_large_gap = true;
				}
			} else if (counter_diff > 128) {
				// Possibly backward or wrap-around
				int backward_amount = 256 - counter_diff;

				if (backward_amount <= 10) {
					// Small backward step - truly out of order
					is_out_of_order = true;
				} else if (counter < 5) {
					// Large backward step and small counter - possibly a restart
					is_tracker_restart = true;
					LOG_DBG(
						"Tracker restart detected: id=%u old_ctr=%u new_ctr=%u (backward=%d)",
						tracker_id,
						last_ping_counter[tracker_id],
						counter,
						backward_amount
					);
				} else {
					// Large backward step but counter not small - long packet loss + wrap-around
					is_large_gap = true;
					LOG_DBG(
						"Long packet loss with wraparound: id=%u last=%u new=%u (backward=%d, "
						"accepting)",
						tracker_id,
						last_ping_counter[tracker_id],
						counter,
						backward_amount
					);
				}
			} else {
				// counter_diff is between 101-128 - large packet loss
				is_large_gap = true;
			}

			// Update timestamp
			last_ping_time[tracker_id] = current_time;
		}

		// Handle various cases
		if (is_tracker_restart) {
			// Tracker restarted, reset counter tracking
			last_ping_counter[tracker_id] = counter;
			// Reset PONG queue tracking
			last_pong_queued_counter[tracker_id] = 0xFF;
			/* The first data sequence after reboot is unrelated to
			 * the old stream; retain totals but re-anchor admission. */
			sequence_initialized[tracker_id] = false;
			decision_ready_mask &= ~BIT(tracker_id);
			decision_fence_mask |= BIT(tracker_id);
			tracker_stats[tracker_id].restart_events++;
			if (atomic_get(&test_all_state_valid)) {
				test_all_invalidate_tracker(tracker_id, (int64_t)current_time);
			}
			// Continue processing this PING, send PONG
		} else if (is_duplicate) {
			// Same counter as last time - likely a retransmit
			LOG_DBG("Duplicate PING detected: id=%u ctr=%u (retransmit or retry)", tracker_id, counter);

			// Check if we already queued a PONG for this counter
			if (counter == last_pong_queued_counter[tracker_id]) {
				LOG_DBG("PONG already queued for ctr=%u, skipping duplicate queue", counter);
				// Don't queue again, tracker will get PONG on next packet
				return;
			}
		} else if (is_out_of_order) {
			// Out-of-order PING - this is an old packet that arrived late
			// Don't process it to avoid sending stale PONG
			int backward_amount = 256 - ((int)counter - (int)last_ping_counter[tracker_id] + 256) % 256;
			LOG_WRN(
				"Out-of-order PING: id=%u ctr=%u (expected >%u, -%d backward), SKIPPING",
				tracker_id,
				counter,
				last_ping_counter[tracker_id],
				backward_amount
			);
			// Don't update last_ping_counter, don't queue PONG
			return;
		} else if (is_large_gap) {
			// Large gap detected - possible packet loss
			int counter_diff = (int)counter - (int)last_ping_counter[tracker_id];
			if (counter_diff < 0) {
				counter_diff += 256;
			}
			if (counter_diff > 10) {
				LOG_WRN(
					"Large PING counter gap: id=%u last=%u new=%u (gap=%d)",
					tracker_id,
					last_ping_counter[tracker_id],
					counter,
					counter_diff
				);
			}
		}

		// Update last seen counter (only if not out-of-order and not skipped)
		if (!is_out_of_order) {
			last_ping_counter[tracker_id] = counter;
		}
	} // End of else branch for ping_counter_initialized

	remote_wom_receive(tracker_id, ping_ack_flag, &payload->data[8], counter);
	if (ping_ack_flag == ESB_PONG_FLAG_METADATA_REQUEST) {
		uint16_t echoed_token = ((uint16_t)payload->data[10] << 8) | payload->data[11];
		bool metadata_matches = metadata_active_mask[tracker_id] != 0 &&
			payload->data[8] == metadata_active_mask[tracker_id] &&
			payload->data[9] == metadata_active_chunk[tracker_id] &&
			echoed_token == metadata_active_token[tracker_id];
		if (metadata_matches) {
			metadata_active_mask[tracker_id] = 0;
			LOG_DBG("Tracker %u confirmed metadata token=%u", tracker_id, echoed_token);
		}
	}
	if (ping_ack_flag != ESB_PONG_FLAG_NORMAL &&
	    ping_ack_flag != ESB_PONG_FLAG_WOM_CONTROL &&
	    ping_ack_flag != ESB_PING_FLAG_WOM_RESULT &&
	    ping_ack_flag != ESB_PING_FLAG_WOM_ERROR) {
		uint16_t ping_ack_tps = ping_ack_flag == ESB_PONG_FLAG_TEST_MODE_ON
			? ((uint16_t)payload->data[8] << 8) | payload->data[9] : 0;
		bool test_payload_matches = ping_ack_flag != ESB_PONG_FLAG_TEST_MODE_ON
			|| ping_ack_tps == tracker_test_on_tps[tracker_id];
		bool batch_payload_matches = ping_ack_flag != ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON
			|| payload->data[8] == pending_cmd_arg[tracker_id];
		if (tracker_remote_command[tracker_id] == ping_ack_flag
		    && test_payload_matches && batch_payload_matches) {
			tracker_remote_command[tracker_id] = ESB_PONG_FLAG_NORMAL;
			/* Confirmations are frequent under load — keep UART off EVENT IRQ. */
			LOG_DBG(
				"Tracker %u confirmed command %s (0x%02X)",
				tracker_id,
				esb_pong_flag_name(ping_ack_flag),
				ping_ack_flag
			);
			if (atomic_get(&test_all_state_valid)) {
				bool confirms_all = ping_ack_flag == ESB_PONG_FLAG_TEST_MODE_OFF
					&& !atomic_get(&test_all_enabled);
				confirms_all = confirms_all
					|| (ping_ack_flag == ESB_PONG_FLAG_TEST_MODE_ON
						&& atomic_get(&test_all_enabled)
						&& ping_ack_tps == test_all_wire_tps());
				if (confirms_all) {
					atomic_or(&test_all_confirmed_mask, BIT(tracker_id));
				}
			}
			if (remote_confirm_cb) {
				remote_confirm_cb(tracker_id, ping_ack_flag);
			}

		}
	}

	/* PONG is now built by ack_handler in radio ISR context.
	 * event_handler only needs to track sequence state. */
	last_pong_queued_counter[tracker_id] = counter;
}

static void esb_receive_composite(const struct esb_payload *payload, uint32_t event_ticks)
{
	/* Composite packet (type ESB_COMPOSITE_TYPE): variable length.
	 * Format: [ESB_COMPOSITE_TYPE][tracker_id][sub_count][sub_type0][sub_data0...]...[sequence]
	 * Each sub-packet: 1 byte type + variable data.
	 */
	if (payload->length < 5 || payload->data[0] != ESB_COMPOSITE_TYPE) {
		LOG_ERR("Wrong packet length: %d", payload->length);
		return;
	}

	uint8_t tracker_id = payload->data[1];
	uint8_t sub_count = payload->data[2];

	if (tracker_id >= stored_trackers || tracker_id >= MAX_TRACKERS) {
		return;
	}

	/* Validate known records before side effects. Unknown legacy tails
	 * stop parsing, preserving the known prefix and pose accounting.
	 * A reachable E0 must still be a valid, exact final record. */
	int scan = 3;
	int event_pos = 0;
	int frame_end = payload->length - 1;
	bool valid = sub_count != 0;
	bool unknown_tail = false;
	for (int i = 0; valid && i < sub_count; i++) {
		if (scan >= frame_end) {
			valid = false;
			break;
		}
		uint8_t type = payload->data[scan++];
		int body_len = composite_body_length(type);
		if (body_len < 0) {
			unknown_tail = true;
			break;
		}
		if (scan + body_len > frame_end) {
			valid = false;
			break;
		}
		if (type == TRACKER_EVENT_ESB_TYPE) {
			struct tracker_event decoded;
			valid = i == sub_count - 1
				&& tracker_event_decode_body(tracker_id, &payload->data[scan],
				                            body_len, &decoded);
			event_pos = scan;
		}
		scan += body_len;
	}
	if (!valid || (!unknown_tail && scan != frame_end)) {
		return;
	}
	if (event_pos != 0) {
		uint8_t event_packet[TRACKER_EVENT_ESB_LEN] = {
			TRACKER_EVENT_ESB_TYPE, tracker_id
		};
		memcpy(&event_packet[2], &payload->data[event_pos], TRACKER_EVENT_BODY_LEN);
		tracker_events_receive(event_packet, sizeof(event_packet), k_uptime_get_32());
		/* Event retries have their own sequence and are not pose evidence. */
		if (--sub_count == 0) {
			return;
		}
	}
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	tdma_check_slot(tracker_id, event_ticks, payload->rssi);
#else
	ARG_UNUSED(event_ticks);
#endif

	LOG_DBG("Received composite packet from tracker %d with %d sub-packets", tracker_id, sub_count);

	/* Sequence byte is at the very end of the composite packet */
	uint8_t received_sequence = payload->data[payload->length - 1];
	int seq_result = check_packet_sequence(tracker_id, received_sequence);
	if (payload->pipe == 1 + (tracker_id % 7)) {
		tdma_shadow_observe(tracker_id, TDMA_SHADOW_EVIDENCE_DATA);
	}
	/* seq_result: 0=normal, 1=potential loss, 2=out of order, 3=reboot, 4=duplicate */
	if (seq_result == 4) {
		LOG_WRN("TRK %d: Duplicate composite packet seq=%d, dropped", tracker_id, received_sequence);
		return; /* duplicate */
	}
	if (seq_result == 2) {
		LOG_WRN("TRK %d: Composite packet seq=%d is out-of-order, dropped", tracker_id, received_sequence);
		return; /* out-of-order */
	}

	if (seq_result == 3 && atomic_get(&test_all_state_valid)) {
		/* Same as the normal data path above: drop the sticky
		 * confirmation and re-arm the grace, leaving any pending
		 * command publication alone. */
		atomic_and(&test_all_confirmed_mask, (atomic_val_t)~BIT(tracker_id));
		atomic_set(
			&test_all_ready_after_ms[tracker_id],
			(atomic_val_t)((uint32_t)k_uptime_get() + TEST_ALL_JOIN_CONFIG_GRACE_MS)
		);
	}

	/* Parse sub-packets and reconstruct standard 16-byte packets */
	int pos = 3;                     /* skip header: type, id, sub_count */
	int end = payload->length - 1; /* exclude sequence byte */

	for (int i = 0; i < sub_count && pos < end; i++) {
		uint8_t sub_type = payload->data[pos++];
		int sub_len = composite_body_length(sub_type);

		if (sub_len < 0 || pos + sub_len > end) {
			break;
		}

		/* Reconstruct a standard 16-byte packet */
		uint8_t pkt[16] = {0};
		pkt[0] = sub_type;
		pkt[1] = tracker_id;
		memcpy(&pkt[2], &payload->data[pos], MIN(sub_len, 14));

		/* For status packets (type 3), fill packet loss stats */
		if (sub_type == 3) {
			struct packet_stats *stats = &tracker_stats[tracker_id];
			pkt[4] = stats->status_received;
			pkt[5] = stats->status_lost;
			pkt[6] = 0;
			pkt[7] = 0;
			stats->status_received = 0;
			stats->status_lost = 0;
		}

		hid_write_packet_n(pkt, payload->rssi);
		pos += sub_len;
	}
}

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		atomic_set(&home_beacon_tx_result, 1);
		break;

	case ESB_EVENT_TX_FAILED:
		atomic_set(&home_beacon_tx_result, -EIO);
		// TX failed - log at debug level
		LOG_DBG("TX FAILED (attempts=%u)", event->tx_attempts);
		break;
	case ESB_EVENT_RX_RECEIVED: {
		int err = 0;
		while (!err) {
			err = esb_read_rx_payload(&rx_payload);
			if (err == -ENODATA) {
				break;
			} else if (err) {
				LOG_ERR("Error while reading rx packet: %d", err);
				break;
			}
#if defined(CONFIG_TDMA_DIAGNOSTICS)
			uint32_t current_rx_ticks = k_uptime_ticks();
#else
			const uint32_t current_rx_ticks = 0;
#endif
			if (rx_payload.length == 0) {
				continue;
			}
			uint32_t physical_home = UINT32_MAX;
			(void)esb_get_rf_channel(&physical_home);
			if (auto_channel_receive(rx_payload.data, rx_payload.length, rx_payload.pipe,
						 physical_home <= 100 ? (uint8_t)physical_home : UINT8_MAX)) {
				continue;
			}
			/* Discovery has a checksum byte, not a packet type. */
			bool discovery = rx_payload.pipe == 0 && rx_payload.length == 8;
			/* Private events never enter pose sequence or ACK accounting. */
			if (!discovery && rx_payload.data[0] == TRACKER_EVENT_ESB_TYPE) {
				if (rx_payload.length == TRACKER_EVENT_ESB_LEN
				    && rx_payload.data[1] < stored_trackers
				    && rx_payload.data[1] < MAX_TRACKERS) {
					tracker_events_receive(rx_payload.data, rx_payload.length, k_uptime_get_32());
				}
				continue;
			}
			if (!discovery && rx_payload.data[0] == ESB_COMPOSITE_TYPE) {
				esb_receive_composite(&rx_payload, current_rx_ticks);
				continue;
			}
			switch (rx_payload.length) {
			case 1: // ACK packet
				LOG_DBG("RX ACK len=%u pipe=%u data=%02X", rx_payload.length, rx_payload.pipe, rx_payload.data[0]);
				break;
			case 8: { // Pairing packet (unified handler)
				uint8_t step = rx_payload.data[1];
				LOG_DBG("RX pairing pkt step=%u pipe=%u", step, rx_payload.pipe);

				if (step == 0) {
					// Step 0: Pairing request - handle known devices directly in ISR
					uint64_t raw_addr = 0;
					memcpy(&raw_addr, rx_payload.data, sizeof(raw_addr));
					uint64_t found_addr = (raw_addr >> 16) & 0xFFFFFFFFFFFF;
					uint8_t checksum = crc8_ccitt(0x07, &rx_payload.data[2], 6);
					if (checksum == 0) {
						checksum = 8;
					}

					if (checksum != rx_payload.data[0] || found_addr == 0) {
						LOG_WRN("Invalid pairing checksum or address");
						break;
					}

					// ISR-safe lockless lookup of known devices
					int known_id = esb_find_tracker(found_addr);

					if (!esb_pairing) {
						if (known_id >= 0) {
							LOG_WRN(
								"Received pairing request from known tracker %d, but pairing mode inactive",
								known_id
							);
						} else {
							LOG_INF("Pairing request from unknown %012llX, pairing mode inactive", found_addr);
						}
						break;
					}

					// Check if new devices are blocked (target count reached, waiting for exit delay)
					if (pairing_new_devices_blocked && known_id < 0) {
						LOG_INF("Pairing request from unknown %012llX rejected (target count reached)", found_addr);
						break;
					}

					if (known_id >= 0) {
						// Known device: ack_handler will respond on step 1.
						LOG_INF("Known tracker %d re-pairing (ack_handler responds on step 1)", known_id);
					} else {
						// Unknown device + pairing mode: queue for thread processing
						struct pairing_event evt = {0};
						memcpy(evt.packet, rx_payload.data, sizeof(evt.packet));

						int q_err = k_msgq_put(&esb_pairing_msgq, &evt, K_NO_WAIT);
						if (q_err) {
							// Drop one and retry
							struct pairing_event discarded;
							(void)k_msgq_get(&esb_pairing_msgq, &discarded, K_NO_WAIT);
							q_err = k_msgq_put(&esb_pairing_msgq, &evt, K_NO_WAIT);
						}

						if (q_err) {
							LOG_WRN("Pairing queue full, dropping request from %012llX", found_addr);
						} else {
							LOG_INF("New device %012llX pairing request queued", found_addr);
						}
					}
				} else if (step == 1) {
					LOG_DBG("RX Pairing Sent ACK (step 1)");
				} else if (step == 2) {
					LOG_DBG("RX Pairing Confirm (step 2)");
				} else {
					LOG_WRN("Unexpected pairing packet type %u", step);
				}
				break;
			} break;
			case ESB_PING_LEN: {
				/*
				 * OTA STATUS/FW_INFO share PING length (13). Must not run PING
				 * clock-bias / TDMA bookkeeping — STATUS bytes 3-6 are seq/bytes,
				 * not expected_rx_ticks, and ~500 Hz polls would stretch EVENT IRQ
				 * into ESB RX overflow during tracker OTA.
				 */
				if (rx_payload.data[0] == ESB_OTA_STATUS_TYPE || rx_payload.data[0] == ESB_OTA_FW_INFO_TYPE) {
					esb_ota_relay_process_tracker_packet(rx_payload.data, rx_payload.length);
					break;
				}

				LOG_DBG(
					"Received PING type=%u id=%u ctr=%u",
					rx_payload.data[0],
					rx_payload.data[1],
					rx_payload.data[2]
				);

				// Check for PING control packet and respond with PONG
				if (rx_payload.data[0] == ESB_PING_TYPE) {
					esb_receive_ping(&rx_payload, current_rx_ticks);
				}
				/* Non-PING length-13 OTA types handled at case entry. */
			} break;
			case 17: // 16 bytes data + 1 byte sequence number
			{
				if (rx_payload.data[0] > 223) {
					break;
				}
				uint8_t tracker_id = rx_payload.data[1];

#if defined(CONFIG_TDMA_DIAGNOSTICS)
				// TDMA Slot Check for Data (Type 17)
				tdma_check_slot(tracker_id, current_rx_ticks, rx_payload.rssi);
#endif

				if (tracker_id >= stored_trackers || tracker_id >= MAX_TRACKERS) {
					continue;
				}

				uint8_t received_sequence = rx_payload.data[16];
				int seq_result = check_packet_sequence(tracker_id, received_sequence);
				if (rx_payload.pipe == 1 + (tracker_id % 7)) {
					tdma_shadow_observe(tracker_id, TDMA_SHADOW_EVIDENCE_DATA);
				}
				// Decide whether to forward the packet based on the sequence check result
				// seq_result: 0=normal, 1=potential loss, 2=out of order, 3=reboot, 4=duplicate
				if (seq_result == 4) {
					LOG_DBG("TRK %d: Duplicate packet seq=%d, dropped", tracker_id, received_sequence);
					// Drop duplicate packet
					break;
				}
				if (seq_result == 2) {
					LOG_DBG("TRK %d: Out-of-order packet seq=%d, dropped", tracker_id, received_sequence);
					// Drop out-of-order packet to avoid incorrect pose calculation
					break;
				}

				if (seq_result == 3 && atomic_get(&test_all_state_valid)) {
					/* Quick reboot with ambiguous PING counter: drop the
					 * sticky confirmation and re-arm the grace. The pending
					 * command itself is left alone — ACK PONGs are PING-
					 * driven; the PING restart/timeout path parks the flag
					 * before the next ACK. */
					atomic_and(&test_all_confirmed_mask, (atomic_val_t)~BIT(tracker_id));
					atomic_set(
						&test_all_ready_after_ms[tracker_id],
						(atomic_val_t)((uint32_t)k_uptime_get() + TEST_ALL_JOIN_CONFIG_GRACE_MS)
					);
				}

				// Forward packet for other cases (normal, potential loss, reboot)
				// For status packets (type 3), fill in packet loss statistics before forwarding
				if (rx_payload.data[0] == 3) {
					struct packet_stats *stats = &tracker_stats[tracker_id];
					rx_payload.data[4] = stats->status_received;
					rx_payload.data[5] = stats->status_lost;
					rx_payload.data[6] = 0; // windows_hit (not implemented)
					rx_payload.data[7] = 0; // windows_missed (not implemented)
					stats->status_received = 0;
					stats->status_lost = 0;
				}
				hid_write_packet_n(rx_payload.data,
								   rx_payload.rssi); // write to hid endpoint
			} break;
			default: {
				/* OTA packets from tracker (status, firmware info) */
				uint8_t pkt_type = rx_payload.data[0];
				if (pkt_type == ESB_OTA_STATUS_TYPE || pkt_type == ESB_OTA_FW_INFO_TYPE) {
					esb_ota_relay_process_tracker_packet(rx_payload.data, rx_payload.length);
					break;
				}

				/* Raw data collection packets (types 0x10-0x13): variable length. */
				if (pkt_type == ESB_RAW_IMU_TYPE || pkt_type == ESB_RAW_IMU_QUAT_TYPE || pkt_type == ESB_RAW_MAG_TYPE
					|| pkt_type == ESB_RAW_META_TYPE || pkt_type == ESB_RAW_CAL_TYPE) {
					uint8_t tracker_id = rx_payload.data[1];
					if (tracker_id >= stored_trackers || tracker_id >= MAX_TRACKERS) {
						break;
					}
					bool is_target = data_collect_is_target(tracker_id)
						|| data_collect_batch_is_target(tracker_id);
					if (is_target) {
						if (pkt_type == ESB_RAW_IMU_TYPE || pkt_type == ESB_RAW_IMU_QUAT_TYPE) {
							uint16_t seq = sys_get_be16(&rx_payload.data[2]);
							if (last_raw_valid[tracker_id] && last_raw_seq[tracker_id] == seq) {
								break;
							}
							last_raw_seq[tracker_id] = seq;
							last_raw_valid[tracker_id] = true;
						}
						data_collect_write(rx_payload.data, rx_payload.length, rx_payload.rssi);
					} else if (!data_collect_is_active() && !data_collect_batch_is_active()) {
						uint8_t pending = tracker_remote_command[tracker_id];
						if (pending != ESB_PONG_FLAG_DATA_COLLECT_OFF &&
						    pending != ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF) {
							esb_send_remote_command(tracker_id, ESB_PONG_FLAG_DATA_COLLECT_OFF);
						}
					}
					break;
				}
				LOG_ERR("Wrong packet length: %d", rx_payload.length);
			} break;
			}
		}
	} break;
	}
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;
	int fetch_attempts = 0;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
		if (err && ++fetch_attempts > 10000) {
			LOG_WRN("Unable to fetch Clock request result: %d", err);
			return err;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}
/* ESB pipe addresses (base addresses big-endian). */
static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

static bool esb_initialized = false;

int esb_radio_quiesce(void)
{
	/* stop_rx rejects PTX. disable uses SDK suspend for either mode, removes
	 * IRQ/PPI/timer activity and clears its state even after partial init.
	 * The SDK DISABLED wait is unbounded on a hardware fault. */
	esb_disable();
	if (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) {
		LOG_ERR("ESB RADIO disable failed; retaining radio owner");
		esb_radio_fault = true;
		return -EIO;
	}
	if (esb_radio_fault) {
		return -EIO;
	}
	if (esb_initialized) {
		esb_auto_loss_fence();
	}
	esb_initialized = false;
	return 0;
}

static int esb_initialize_internal(bool tx, bool quiet)
{
	if (esb_radio_fault) {
		return -EIO;
	}
	if (esb_initialized) {
		LOG_WRN("ESB already initialized");
		return -EALREADY;
	}
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	if (tx) {
		config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.event_handler = event_handler;
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = RADIO_RETRANSMIT_DELAY;
		config.tx_mode = ESB_TXMODE_MANUAL;
		config.selective_auto_ack = true;
		config.use_fast_ramp_up = true;
	} else {
		config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
		config.ack_handler = esb_ack_handler_cb;
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = RADIO_RETRANSMIT_DELAY;
		config.selective_auto_ack = true;
		config.use_fast_ramp_up = true;
	}

	if (!quiet) {
		LOG_INF("Initializing ESB, %sX mode", tx ? "T" : "R");
	}
	err = esb_init(&config);

	if (!err) {
		// Use saved channel if available, otherwise use default (stored is encoded)
		uint8_t channel_to_use = esb_rf_channel_decode(receiver_rf_channel);
		if (channel_to_use == ESB_RF_CHANNEL_DEFAULT) {
			channel_to_use = RADIO_RF_CHANNEL;
		}
		err = esb_set_rf_channel(channel_to_use);
		if (!err && !quiet) {
			LOG_INF("Set RF channel to %u", channel_to_use);
		}
	}

	if (!err) {
		err = esb_set_base_address_0(base_addr_0);
	}

	if (!err) {
		err = esb_set_base_address_1(base_addr_1);
	}

	if (!err) {
		err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	}

	if (err) {
		LOG_ERR("ESB initialization failed: %d", err);
		set_status(SYS_STATUS_CONNECTION_ERROR, true);
		int stop_err = esb_radio_quiesce();
		if (stop_err) {
			return stop_err;
		}
		return err;
	}

	esb_initialized = true;
	return 0;
}

int esb_initialize(bool tx)
{
	return esb_initialize_internal(tx, false);
}

int esb_initialize_quiet(bool tx)
{
	return esb_initialize_internal(tx, true);
}

int esb_home_beacon_send(const uint8_t frame[16])
{
	/* Owner-held, after operation_stop. Static RAM remains DMA-safe even
	 * when teardown reports failure; no stack-backed radio payload. */
	static struct esb_payload beacon_payload;
	int err = frame ? esb_initialize_quiet(true) : -EINVAL;
	if (!err) {
		beacon_payload.length = 16;
		beacon_payload.pipe = 0;
		beacon_payload.noack = true;
		memcpy(beacon_payload.data, frame, 16);
		atomic_set(&home_beacon_tx_result, 0);
		err = esb_write_payload(&beacon_payload);
	}
	if (!err) {
		err = esb_start_tx();
	}
	if (!err) {
		uint32_t started = k_cycle_get_32();
		while (!atomic_get(&home_beacon_tx_result)
		       && k_cyc_to_us_floor32(k_cycle_get_32() - started) < 2000U) {
			k_busy_wait(10);
		}
		int result = atomic_get(&home_beacon_tx_result);
		err = result > 0 ? 0 : (result < 0 ? result : -ETIMEDOUT);
	}
	/* NoACK success is only local TX completion, never delivery evidence.
	 * Always stop PTX, including init/write/start/timeout failures. */
	int stop_err = esb_radio_quiesce();
	if (err) {
		LOG_WRN("Home beacon TX failed: %d", err);
	}
	return stop_err ? stop_err : err;
}

int esb_radio_operation_stop(void)
{
	return esb_radio_quiesce();
}

void esb_deinitialize(void)
{
	LOG_INF("ESB deinitialize requested");
	if (esb_initialized) {
		esb_initialized = false;
		LOG_INF("Deinitializing ESB");
		k_msleep(10); // wait for pending transmissions
		if (esb_initialized) {
			LOG_INF("ESB denitialize cancelled");
			return;
		}
		esb_disable();
	}
	esb_initialized = false;
}

// Unified mode: pipe 0 uses discovery address (pairing), pipes 1-7 use paired address (data)
void esb_set_addr_unified(void)
{
	// Pipe 0: discovery base address for pairing
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));

	// Pipes 1-7: paired base address for data/PING
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR;
	uint8_t buf[6] = {0};
	memcpy(buf, addr, 6);
	uint8_t addr_buffer[16] = {0};
	for (int i = 0; i < 4; i++) {
		addr_buffer[i] = buf[i];
		addr_buffer[i + 4] = buf[i] + buf[4];
	}
	for (int i = 0; i < 8; i++) {
		addr_buffer[i + 8] = buf[5] + i;
	}
	for (int i = 0; i < 16; i++) {
		if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55
			|| addr_buffer[i] == 0xAA) { // Avoid invalid addresses (see nrf datasheet)
			addr_buffer[i] += 8;
		}
	}
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));

	// Prefix: pipe 0 = discovery, pipes 1-7 = paired
	addr_prefix[0] = discovery_addr_prefix[0];
	for (int i = 1; i < 8; i++) {
		addr_prefix[i] = addr_buffer[8 + i];
	}
}

/* Caller holds tracker_store_lock and irq_lock. A slot is a new session. */
static void tracker_slot_reset(uint8_t id)
{
	last_packet_sequence[id] = 0;
	sequence_initialized[id] = false;
	last_ping_counter[id] = 0;
	ping_counter_initialized[id] = false;
	last_ping_time[id] = 0;
	last_pong_queued_counter[id] = 0;
	pipe_mismatch_count[id] = 0;
	crc_error_count[id] = 0;
	last_raw_seq[id] = 0;
	last_raw_valid[id] = false;
	tracker_remote_command[id] = ESB_PONG_FLAG_NORMAL;
	wom_token[id] = 0;
	wom_minutes[id] = 0;
	wom_error[id] = 0;
	wom_complete[id] = false;
	wom_advertised[id] = false;
	wom_first_ping_counter[id] = 0;
	pending_cmd_arg[id] = 0;
	tracker_test_on_tps[id] = 0;
	for (unsigned int axis = 0; axis < 3; axis++) {
		pending_sens_data[id][axis] = 0;
	}
	pending_sens_auto_axis[id] = 0;
	pending_sens_auto_revolutions[id] = 0;
	metadata_active_mask[id] = 0;
	metadata_active_chunk[id] = 0;
	metadata_active_token[id] = 0;
	metadata_pending_mask[id] = 0;
	metadata_pending_chunk[id] = 0;
	metadata_pending_token[id] = 0;
	atomic_and(&test_all_confirmed_mask, (atomic_val_t)~BIT(id));
	atomic_set(&test_all_ready_after_ms[id], 0);
	tdma_config_packed[id] = 0;
	tdma_prev_recv[id] = 0;
	tdma_prev_gaps[id] = 0;
	tdma_prev_restarts[id] = 0;
	memset(&tracker_stats[id], 0, sizeof(tracker_stats[id]));
	decision_received[id] = 0;
	decision_gaps[id] = 0;
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	memset(&g_tdma_stats[id], 0, sizeof(g_tdma_stats[id]));
	g_last_ping_rx_time_diff_ticks[id] = 0;
	g_last_ping_rx_time_diff_valid[id] = false;
	g_ping_isr_rx_ticks[id] = 0;
	g_ping_isr_rx_ticks_valid[id] = false;
	g_last_isr_rx_ticks[id] = 0;
	g_last_isr_rx_valid[id] = false;
	g_bias_ppb_valid[id] = 0;
	g_bias_ppb[id] = 0;
	g_bias_ref_offset[id] = 0;
	g_bias_ref_ticks[id] = 0;
	g_last_ping_isr_rx_ticks_raw[id] = 0;
	g_ping_dirty_streak[id] = 0;
	g_ping_reject_count[id] = 0;
	g_extrap_skip_count[id] = 0;
#endif
	channel_target_mask &= ~BIT(id);
	tdma_shadow_forget_tracker(id);
	esb_auto_loss_invalidate();
	hid_reset_rssi_smooth(id);
	tracker_events_pairing_invalidate(BIT(id));
}

int esb_add_pair(uint64_t addr, bool checksum)
{
	if (!tracker_roster_address_valid(addr)) {
		return -EINVAL;
	}

	bool new_entry = false;
	int assigned_id = -1;

	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	if (!tracker_store_ready) {
		k_mutex_unlock(&tracker_store_lock);
		return -EIO;
	}
	for (int i = 0; i < stored_trackers; i++) {
		if (stored_tracker_addr[i] == addr) {
			assigned_id = i;
			break;
		}
	}

	if (assigned_id < 0) {
		if (stored_trackers >= MAX_TRACKERS) {
			k_mutex_unlock(&tracker_store_lock);
			LOG_WRN("Tracker storage full, cannot add %012llX", addr);
			return -ENOSPC;
		}
		assigned_id = stored_trackers;
		uint64_t addresses[MAX_TRACKERS];
		memcpy(addresses, stored_tracker_addr, sizeof(addresses));
		addresses[assigned_id] = addr;
		int err = tracker_roster_save(assigned_id + 1, addresses);
		if (err) {
			k_mutex_unlock(&tracker_store_lock);
			LOG_ERR("Pairing not saved: %d", err);
			return err;
		}
		unsigned int key = irq_lock();
		tracker_slot_reset(assigned_id);
		stored_tracker_addr[assigned_id] = addr;
		__asm__ volatile("" ::: "memory"); // compiler barrier
		stored_trackers = assigned_id + 1;
		irq_unlock(key);
		tracker_events_pairing_cleanup();
		new_entry = true;
	}

	k_mutex_unlock(&tracker_store_lock);

	if (new_entry) {
		LOG_INF("Added device on id %d with address %012llX", assigned_id, addr);
	} else {
		LOG_INF("Device already stored with id %d", assigned_id);
	}

	if (checksum) {
		uint8_t buf[6] = {0};
		memcpy(buf, &addr, 6);
		uint8_t checksum_byte = crc8_ccitt(0x07, buf, 6);
		if (checksum_byte == 0) {
			checksum_byte = 8;
		}
		// Use device address as unique identifier (although it is not actually guaranteed, see datasheet
		uint64_t *receiver_addr = (uint64_t *)NRF_FICR->DEVICEADDR;
		uint64_t pair_addr = (*receiver_addr & 0xFFFFFFFFFFFF) << 16;
		pair_addr |= checksum_byte;              // Add checksum to the address
		pair_addr |= (uint64_t)assigned_id << 8; // Add tracker id to the address
		LOG_INF("Pair the device with %016llX", pair_addr);
	}

	return assigned_id;
}

int esb_pop_pair(void)
{
	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	if (!tracker_store_ready) {
		k_mutex_unlock(&tracker_store_lock);
		return -EIO;
	}
	if (stored_trackers == 0) {
		k_mutex_unlock(&tracker_store_lock);
		return 0;
	}
	uint8_t removed_id = stored_trackers - 1;
	uint64_t removed_addr = stored_tracker_addr[removed_id];
	uint64_t addresses[MAX_TRACKERS];
	memcpy(addresses, stored_tracker_addr, sizeof(addresses));
	addresses[removed_id] = 0;
	int err = tracker_roster_save(removed_id, addresses);
	if (err) {
		k_mutex_unlock(&tracker_store_lock);
		LOG_ERR("Pairing removal not saved: %d", err);
		return err;
	}
	bool raw_target = data_collect_is_target(removed_id);
	esb_ota_forget_tracker(removed_id);
	data_collect_forget_tracker(removed_id);
	unsigned int key = irq_lock();
	if (raw_target) {
		raw_arq_reset();
	}
	stored_trackers = removed_id;
	stored_tracker_addr[removed_id] = 0;
	tracker_slot_reset(removed_id);
	irq_unlock(key);
	tdma_recalculate(true);
	tracker_events_pairing_cleanup();
	k_mutex_unlock(&tracker_store_lock);
	LOG_INF("Removed device on id %d with address %012llX", removed_id, removed_addr);
	return 0;
}

static bool esb_parse_pair(const uint8_t packet[8])
{
	uint64_t raw_addr = 0;
	memcpy(&raw_addr, packet, sizeof(raw_addr));
	uint64_t found_addr = (raw_addr >> 16) & 0xFFFFFFFFFFFF;
	uint8_t checksum = crc8_ccitt(0x07, &packet[2], 6);
	if (checksum == 0) {
		checksum = 8;
	}

	uint16_t send_tracker_id = 0;
	uint8_t tracker_count_snapshot = 0;

	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	tracker_count_snapshot = stored_trackers;
	send_tracker_id = tracker_count_snapshot; // default to next available ID
	for (uint8_t i = 0; i < tracker_count_snapshot; i++) {
		if (found_addr != 0 && stored_tracker_addr[i] == found_addr) {
			send_tracker_id = i;
			break;
		}
	}
	k_mutex_unlock(&tracker_store_lock);

	bool checksum_valid = (checksum == packet[0]);
	bool has_capacity = tracker_count_snapshot < MAX_TRACKERS;
	bool is_new_device = checksum_valid && found_addr != 0 && send_tracker_id == tracker_count_snapshot && has_capacity;
	bool ack_valid = false;

	if (is_new_device) {
		int assigned_id = esb_add_pair(found_addr, false);
		if (assigned_id >= 0) {
			send_tracker_id = (uint16_t)assigned_id;
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
		} else if (assigned_id == -ENOSPC) {
			LOG_WRN("Maximum tracker slots reached, cannot pair %012llX", found_addr);
			return false;
		} else {
			LOG_ERR("Failed to store tracker address %012llX: %d", found_addr, assigned_id);
			return false;
		}
	}

	ack_valid = checksum_valid && send_tracker_id < MAX_TRACKERS;

	return ack_valid;
}

void esb_start_pairing(void)
{
	LOG_INF("Pairing mode enabled (unified)");
	esb_pairing = true;
	esb_auto_loss_invalidate();
	pairing_start_time = k_uptime_get();
	pairing_target_count = 0; // No limit
	pairing_initial_count = stored_trackers;
	k_msgq_purge(&esb_pairing_msgq);
	set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
}

void esb_start_pairing_with_count(uint8_t target_count)
{
	LOG_INF("Pairing mode enabled (unified), target count: %u", target_count);
	esb_pairing = true;
	esb_auto_loss_invalidate();
	pairing_start_time = k_uptime_get();
	pairing_target_count = target_count;
	pairing_initial_count = stored_trackers;
	k_msgq_purge(&esb_pairing_msgq);
	set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
}

// Process new device pairing requests from the queue (called from esb_thread, non-blocking)
static void process_pairing_queue(void)
{
	struct pairing_event evt;
	while (k_msgq_get(&esb_pairing_msgq, &evt, K_NO_WAIT) == 0) {
		if (evt.packet[1] != 0) {
			continue; // Only process step 0 (pairing request)
		}

		bool ack_ready = esb_parse_pair(evt.packet);
		if (!ack_ready) {
			LOG_DBG("Pairing request invalid, not queueing response");
			continue;
		}

		// Device is now registered — ack_handler will respond on the
		// next pairing step 1 via esb_find_tracker() lookup.
		LOG_INF("New device registered, ack_handler will respond on next step 1");
		set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
	}
}

void esb_finish_pair(void)
{
	esb_pairing = false;
	pairing_start_time = 0;
	pairing_target_count = 0;
	pairing_initial_count = 0;
	pairing_target_reached_time = 0;
	pairing_new_devices_blocked = false;
	k_msgq_purge(&esb_pairing_msgq);
	set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_CONNECTION);
	LOG_INF("Pairing mode disabled");
}

int esb_clear(void)
{
	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	uint64_t addresses[MAX_TRACKERS] = {0};
	int err = tracker_roster_save(0, addresses);
	if (err) {
		k_mutex_unlock(&tracker_store_lock);
		LOG_ERR("Pairings not cleared: %d", err);
		return err;
	}
	esb_clearing = true;
	esb_pairing = false;
	k_msgq_purge(&esb_pairing_msgq);
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		esb_ota_forget_tracker(i);
		data_collect_forget_tracker(i);
	}
	unsigned int key = irq_lock();
	atomic_inc(&test_all_generation);
	test_all_invalidate();
	atomic_set(&test_all_enabled, 0);
	test_all_requested_tps = 0;
	test_all_effective_tps = 0;
	raw_arq_reset();
	stored_trackers = 0;
	memset(stored_tracker_addr, 0, sizeof(stored_tracker_addr));
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		tracker_slot_reset(i);
	}
	atomic_set(&tdma_shadow_desired_changed_at_ms, 0);
	tracker_store_ready = true;
	irq_unlock(key);
	tdma_recalculate(true);
	tracker_events_pairing_cleanup();
	esb_clearing = false;
	k_mutex_unlock(&tracker_store_lock);
	LOG_INF("Pairings cleared and saved");
	return 0;
}

uint32_t esb_send_remote_wom_command(uint8_t target_id, uint16_t minutes, uint16_t *token_out)
{
	if (token_out == NULL ||
	    (minutes > ESB_WOM_TIMEOUT_MAX_MINUTES &&
	     minutes != ESB_WOM_TIMEOUT_CLEAR && minutes != ESB_WOM_ENTER) ||
	    (target_id != 0xff && target_id >= MAX_TRACKERS)) {
		return 0;
	}
	*token_out = 0;
	uint64_t scan_start_time = (uint64_t)k_uptime_get();
	if (target_id == 0xff) {
		k_msleep(REMOTE_COMMAND_ACTIVE_SCAN_MS);
	}
	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	unsigned int key = irq_lock();
	uint32_t mask = 0;
	for (uint8_t id = 0; id < stored_trackers && id < MAX_TRACKERS; id++) {
		if (stored_tracker_addr[id] != 0 &&
		    (target_id == id || (target_id == 0xff &&
		     tracker_stats[id].last_packet_time >= scan_start_time))) {
			mask |= BIT(id);
		}
	}
	if (mask != 0) {
		if (++wom_next_token == 0) {
			wom_next_token = 1;
		}
		for (uint8_t id = 0; id < MAX_TRACKERS; id++) {
			if (!(mask & BIT(id))) {
				continue;
			}
			wom_token[id] = wom_next_token;
			wom_minutes[id] = minutes;
			wom_error[id] = 0;
			wom_complete[id] = false;
			wom_advertised[id] = false;
			wom_first_ping_counter[id] = 0;
			tracker_remote_command[id] = ESB_PONG_FLAG_WOM_CONTROL;
		}
		*token_out = wom_next_token;
	}
	irq_unlock(key);
	k_mutex_unlock(&tracker_store_lock);
	return mask;
}

bool esb_get_remote_wom_result(uint8_t id, uint16_t token, uint16_t *minutes, uint16_t *error)
{
	if (id >= MAX_TRACKERS || token == 0 || minutes == NULL || error == NULL) {
		return false;
	}
	unsigned int key = irq_lock();
	bool matches = wom_token[id] == token;
	/* Another control command displaced the pending request: never treat its
	 * eventual generic confirmation as proof of a completed WOM command. */
	if (matches && !wom_complete[id] && tracker_remote_command[id] != ESB_PONG_FLAG_WOM_CONTROL) {
		wom_minutes[id] = 0;
		wom_error[id] = ECANCELED;
		wom_complete[id] = true;
	}
	bool complete = matches && wom_complete[id];
	if (complete) {
		*minutes = wom_minutes[id];
		*error = wom_error[id];
	}
	irq_unlock(key);
	return complete;
}

void esb_cancel_remote_wom_command(uint32_t mask, uint16_t token)
{
	unsigned int key = irq_lock();
	for (uint8_t id = 0; id < MAX_TRACKERS; id++) {
		if ((mask & BIT(id)) && token != 0 && wom_token[id] == token) {
			if (tracker_remote_command[id] == ESB_PONG_FLAG_WOM_CONTROL) {
				tracker_remote_command[id] = ESB_PONG_FLAG_NORMAL;
			}
			wom_token[id] = 0;
			wom_minutes[id] = 0;
			wom_error[id] = 0;
			wom_complete[id] = false;
			wom_advertised[id] = false;
			wom_first_ping_counter[id] = 0;
		}
	}
	irq_unlock(key);
}

void esb_send_remote_command_sens(uint8_t tracker_id, float x, float y, float z)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}
	int16_t sensitivity[3] = {(int16_t)(x * 100.0f), (int16_t)(y * 100.0f),
				 (int16_t)(z * 100.0f)};
	unsigned int key = irq_lock();
	pending_sens_data[tracker_id][0] = sensitivity[0];
	pending_sens_data[tracker_id][1] = sensitivity[1];
	pending_sens_data[tracker_id][2] = sensitivity[2];
	tracker_remote_command[tracker_id] = ESB_PONG_FLAG_SENS_SET;
	irq_unlock(key);
	LOG_INF("Queued SENS_SET for tracker %u: %.2f, %.2f, %.2f", tracker_id, (double)x, (double)y, (double)z);
}

bool esb_send_remote_command_sens_auto(uint8_t tracker_id, uint8_t axis, uint16_t revolutions)
{
	if (tracker_id >= MAX_TRACKERS) {
		return false;
	}

	int64_t now = k_uptime_get();
	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	unsigned int key = irq_lock();
	bool active = tracker_id < stored_trackers && stored_tracker_addr[tracker_id] != 0
			   && tracker_stats[tracker_id].last_packet_time > 0
			   && now - tracker_stats[tracker_id].last_packet_time <= PING_TIMEOUT_MS;
	if (!active) {
		irq_unlock(key);
		k_mutex_unlock(&tracker_store_lock);
		LOG_WRN("SENS_AUTO not queued for inactive tracker %u", tracker_id);
		return false;
	}
	pending_sens_auto_axis[tracker_id] = axis;
	pending_sens_auto_revolutions[tracker_id] = revolutions;
	tracker_remote_command[tracker_id] = ESB_PONG_FLAG_SENS_AUTO;
	irq_unlock(key);
	k_mutex_unlock(&tracker_store_lock);

	if (revolutions == 0) {
		LOG_INF("Queued SENS_AUTO for tracker %u: axis=%u, revolutions=default", tracker_id, axis);
	} else {
		LOG_INF("Queued SENS_AUTO for tracker %u: axis=%u, revolutions=%u", tracker_id, axis, revolutions);
	}
	return true;
}

uint32_t esb_send_remote_command_sens_auto_all(uint8_t axis, uint16_t revolutions)
{
	uint32_t mask = 0;
	uint8_t count = 0;
	int64_t scan_start_time = k_uptime_get();

	k_msleep(REMOTE_COMMAND_ACTIVE_SCAN_MS);

	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	unsigned int key = irq_lock();
	for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; i++) {
		if (stored_tracker_addr[i] != 0 && tracker_stats[i].last_packet_time >= scan_start_time) {
			pending_sens_auto_axis[i] = axis;
			pending_sens_auto_revolutions[i] = revolutions;
			tracker_remote_command[i] = ESB_PONG_FLAG_SENS_AUTO;
			mask |= (1u << i);
			count++;
		}
	}
	irq_unlock(key);
	k_mutex_unlock(&tracker_store_lock);

	if (revolutions == 0) {
		LOG_INF("Queued SENS_AUTO for %u active trackers: axis=%u, revolutions=default", count, axis);
	} else {
		LOG_INF("Queued SENS_AUTO for %u active trackers: axis=%u, revolutions=%u", count, axis, revolutions);
	}

	return mask;
}

/* Trackers that sent any packet since scan_start_time. */
static uint32_t esb_scan_active_mask(int64_t scan_start_time)
{
	uint32_t mask = 0;
	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; i++) {
		if (stored_tracker_addr[i] != 0 && tracker_stats[i].last_packet_time >= scan_start_time) {
			mask |= (1u << i);
		}
	}
	k_mutex_unlock(&tracker_store_lock);
	return mask;
}

static void esb_publish_flag_mask(uint8_t command_flag, uint32_t mask)
{
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		if (mask & BIT(i)) {
			tracker_remote_command[i] = command_flag;
		}
	}
}

/* Test-only publication, called under the test irq_lock: publish only onto
 * idle slots (no pending command is stomped) whose recovery/join grace has
 * elapsed; skipped trackers converge via reconciliation after their grace. */
static void esb_publish_test_flag_mask(uint8_t command_flag, uint32_t mask, uint32_t now_u32)
{
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		if (!(mask & BIT(i)) || tracker_remote_command[i] != ESB_PONG_FLAG_NORMAL) {
			continue;
		}
		uint32_t ready_after = (uint32_t)atomic_get(&test_all_ready_after_ms[i]);
		if ((int32_t)(now_u32 - ready_after) < 0) {
			continue;
		}
		tracker_remote_command[i] = command_flag;
	}
}

uint32_t esb_send_remote_command_test_on(uint8_t tracker_id, uint16_t tps)
{
	if (tracker_id >= MAX_TRACKERS) {
		return 0;
	}
	/* Same critical section as the all-broadcast final check: generation
	 * bump, supersede, and TPS/flag publication cannot interleave. */
	unsigned int key = irq_lock();
	atomic_inc(&test_all_generation);
	/* A targeted command replaces any sticky-all policy and cancels any
	 * in-flight all-target broadcast during its scan. */
	test_all_invalidate();
	/* Publish the TPS before the flag: the ACK handler (radio ISR) echoes it
	 * as soon as it observes the command flag. */
	tracker_test_on_tps[tracker_id] = tps;
	__asm__ volatile("" ::: "memory");
	tracker_remote_command[tracker_id] = ESB_PONG_FLAG_TEST_MODE_ON;
	irq_unlock(key);
	LOG_INF("Queued TEST_MODE_ON (target %u TPS) for tracker %u", tps, tracker_id);
	return BIT(tracker_id);
}

uint32_t esb_send_remote_command_test_off(uint8_t tracker_id)
{
	if (tracker_id >= MAX_TRACKERS) {
		return 0;
	}
	unsigned int key = irq_lock();
	atomic_inc(&test_all_generation);
	test_all_invalidate();
	tracker_test_on_tps[tracker_id] = 0;
	__asm__ volatile("" ::: "memory");
	tracker_remote_command[tracker_id] = ESB_PONG_FLAG_TEST_MODE_OFF;
	irq_unlock(key);
	LOG_INF("Queued TEST_MODE_OFF for tracker %u", tracker_id);
	return BIT(tracker_id);
}

uint32_t esb_send_remote_command_test_on_all(uint16_t tps)
{
	k_mutex_lock(&test_all_mutex, K_FOREVER);
	/* Guard first, then own a generation token: anything that bumps the
	 * counter after this point supersedes us and also releases the guard. */
	atomic_set(&test_all_broadcasting, 1);
	atomic_val_t gen = atomic_inc(&test_all_generation) + 1;
	int64_t scan_start_time = k_uptime_get();
	k_msleep(REMOTE_COMMAND_ACTIVE_SCAN_MS);
	uint32_t mask = esb_scan_active_mask(scan_start_time);

	uint16_t requested = 0;
	/* Final generation check through sticky state, TPS, and flag publication
	 * is one critical section shared with targeted commands and reconcile:
	 * once the check passes under the lock, nothing can supersede us. */
	unsigned int key = irq_lock();
	if (atomic_get(&test_all_generation) != gen) {
		irq_unlock(key);
		/* The mutex guarantees no other all-helper is inside: safe to
		 * release the guard here so reconciliation is not wedged. */
		atomic_set(&test_all_broadcasting, 0);
		LOG_WRN("Sticky TEST_MODE_ON aborted: superseded during active scan");
		k_mutex_unlock(&test_all_mutex);
		return 0;
	}
	atomic_set(&test_all_state_valid, 1);
	atomic_set(&test_all_enabled, 1);
	esb_auto_loss_invalidate();
	atomic_set(&test_all_confirmed_mask, 0);
	test_all_requested_tps = tps;
	requested = test_all_requested_value();
	test_all_effective_tps = MIN(requested, test_all_capacity_tps(tdma_dynamic_active_count));
	uint16_t wire_tps = test_all_wire_tps();
	uint32_t now_u32 = (uint32_t)k_uptime_get();
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		tracker_test_on_tps[i] = wire_tps;
		/* Keep an armed recovery/join grace so reconciliation honors the
		 * remaining window; only expired timers reset. */
		uint32_t ready_after = (uint32_t)atomic_get(&test_all_ready_after_ms[i]);
		if ((int32_t)(now_u32 - ready_after) >= 0) {
			atomic_set(&test_all_ready_after_ms[i], 0);
		}
	}
	__asm__ volatile("" ::: "memory");
	esb_publish_test_flag_mask(ESB_PONG_FLAG_TEST_MODE_ON, mask, now_u32);
	atomic_set(&test_all_broadcasting, 0);
	irq_unlock(key);
	LOG_INF(
		"Sticky TEST_MODE_ON enabled: requested=%u effective=%u active=%u targeted=0x%04x",
		requested,
		test_all_effective_tps,
		tdma_dynamic_active_count,
		(unsigned int)mask
	);
	k_mutex_unlock(&test_all_mutex);
	return mask;
}

uint32_t esb_send_remote_command_test_off_all(void)
{
	k_mutex_lock(&test_all_mutex, K_FOREVER);
	atomic_set(&test_all_broadcasting, 1);
	atomic_val_t gen = atomic_inc(&test_all_generation) + 1;
	int64_t scan_start_time = k_uptime_get();
	k_msleep(REMOTE_COMMAND_ACTIVE_SCAN_MS);
	uint32_t mask = esb_scan_active_mask(scan_start_time);

	unsigned int key = irq_lock();
	if (atomic_get(&test_all_generation) != gen) {
		irq_unlock(key);
		atomic_set(&test_all_broadcasting, 0);
		LOG_WRN("Sticky TEST_MODE_OFF aborted: superseded during active scan");
		k_mutex_unlock(&test_all_mutex);
		return 0;
	}
	atomic_set(&test_all_state_valid, 1);
	atomic_set(&test_all_enabled, 0);
	atomic_set(&test_all_confirmed_mask, 0);
	test_all_effective_tps = 0;
	uint32_t now_u32 = (uint32_t)k_uptime_get();
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		tracker_test_on_tps[i] = 0;
		uint32_t ready_after = (uint32_t)atomic_get(&test_all_ready_after_ms[i]);
		if ((int32_t)(now_u32 - ready_after) >= 0) {
			atomic_set(&test_all_ready_after_ms[i], 0);
		}
	}
	__asm__ volatile("" ::: "memory");
	esb_publish_test_flag_mask(ESB_PONG_FLAG_TEST_MODE_OFF, mask, now_u32);
	atomic_set(&test_all_broadcasting, 0);
	irq_unlock(key);
	LOG_INF("Sticky TEST_MODE_OFF enabled for current and future active trackers");
	k_mutex_unlock(&test_all_mutex);
	return mask;
}

static const char *esb_pong_flag_name(uint8_t flag)
{
	switch (flag) {
	case ESB_PONG_FLAG_NORMAL:
		return "NORMAL";
	case ESB_PONG_FLAG_WOM_CONTROL:
		return "WOM_CONTROL";
	case ESB_PONG_FLAG_SHUTDOWN:
		return "SHUTDOWN";
	case ESB_PONG_FLAG_CALIBRATE:
		return "CALIBRATE";
	case ESB_PONG_FLAG_CALIBRATE_ACC:
		return "CALIBRATE_ACC";
	case ESB_PONG_FLAG_MEOW:
		return "MEOW";
	case ESB_PONG_FLAG_SCAN:
		return "SCAN";
	case ESB_PONG_FLAG_MAG_CLEAR:
		return "MAG_CLEAR";
	case ESB_PONG_FLAG_MAG_CAL:
		return "MAG_CAL";
	case ESB_PONG_FLAG_MAG_ON:
		return "MAG_ON";
	case ESB_PONG_FLAG_MAG_OFF:
		return "MAG_OFF";
	case ESB_PONG_FLAG_MAG_AUTO_ON:
		return "MAG_AUTO_ON";
	case ESB_PONG_FLAG_MAG_AUTO_OFF:
		return "MAG_AUTO_OFF";
	case ESB_PONG_FLAG_REBOOT:
		return "REBOOT";
	case ESB_PONG_FLAG_CLEAR:
		return "CLEAR";
	case ESB_PONG_FLAG_DFU:
		return "DFU";
	case ESB_PONG_FLAG_DFU_OTA:
		return "DFU_OTA";
	case ESB_PONG_FLAG_SET_CHANNEL:
		return "SET_CHANNEL";
	case ESB_PONG_FLAG_CLEAR_CHANNEL:
		return "CLEAR_CHANNEL";
	case ESB_PONG_FLAG_SENS_SET:
		return "SENS_SET";
	case ESB_PONG_FLAG_SENS_RESET:
		return "SENS_RESET";
	case ESB_PONG_FLAG_SENS_AUTO:
		return "SENS_AUTO";
	case ESB_PONG_FLAG_RESET_ZRO:
		return "RESET_ZRO";
	case ESB_PONG_FLAG_RESET_ACC:
		return "RESET_ACC";
	case ESB_PONG_FLAG_RESET_BAT:
		return "RESET_BAT";
	case ESB_PONG_FLAG_PING:
		return "PING";
	case ESB_PONG_FLAG_RESET_TCAL:
		return "RESET_TCAL";
	case ESB_PONG_FLAG_TCAL_AUTO_ON:
		return "TCAL_AUTO_ON";
	case ESB_PONG_FLAG_TCAL_AUTO_OFF:
		return "TCAL_AUTO_OFF";
	case ESB_PONG_FLAG_TCAL_HEATED_START:
		return "TCAL_HEATED_START";
	case ESB_PONG_FLAG_FUSION_RESET:
		return "FUSION_RESET";
	case ESB_PONG_FLAG_TCAL_BOOT_ON:
		return "TCAL_BOOT_ON";
	case ESB_PONG_FLAG_TCAL_BOOT_OFF:
		return "TCAL_BOOT_OFF";
	case ESB_PONG_FLAG_TCAL_ON:
		return "TCAL_ON";
	case ESB_PONG_FLAG_TCAL_OFF:
		return "TCAL_OFF";
	case ESB_PONG_FLAG_TDMA_ON:
		return "TDMA_ON";
	case ESB_PONG_FLAG_TDMA_OFF:
		return "TDMA_OFF";
	case ESB_PONG_FLAG_TEST_MODE_ON:
		return "TEST_MODE_ON";
	case ESB_PONG_FLAG_TEST_MODE_OFF:
		return "TEST_MODE_OFF";
	case ESB_PONG_FLAG_DATA_COLLECT_ON:
		return "DATA_COLLECT_ON";
	case ESB_PONG_FLAG_DATA_COLLECT_OFF:
		return "DATA_COLLECT_OFF";
	case ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON:
		return "DATA_COLLECT_BATCH_ON";
	case ESB_PONG_FLAG_DATA_COLLECT_BATCH_OFF:
		return "DATA_COLLECT_BATCH_OFF";
	case ESB_PONG_FLAG_METADATA_REQUEST:
		return "METADATA_REQUEST";
	case ESB_PONG_FLAG_OTA_QUERY_INFO:
		return "OTA_QUERY_INFO";
	case ESB_PONG_FLAG_OTA_ABORT:
		return "OTA_ABORT";
	case ESB_PONG_FLAG_OTA_SUPPRESS:
		return "OTA_SUPPRESS";
	case ESB_PONG_FLAG_OTA_UNSUPPRESS:
		return "OTA_UNSUPPRESS";
	default:
		return "UNKNOWN";
	}
}

bool esb_request_metadata(uint8_t tracker_id, uint8_t mask, uint8_t chunk)
{
	if (tracker_id >= MAX_TRACKERS || mask == 0 || (mask & ~ESB_METADATA_MASK_VALID) != 0) {
		return false;
	}
	int64_t now = k_uptime_get();
	if (tracker_id >= stored_trackers || stored_tracker_addr[tracker_id] == 0 ||
	    tracker_stats[tracker_id].last_packet_time == 0 ||
	    now - tracker_stats[tracker_id].last_packet_time > PING_TIMEOUT_MS) {
		return false;
	}
	unsigned int key = irq_lock();
	metadata_next_token++;
	if (metadata_next_token == 0) {
		metadata_next_token = 1;
	}
	metadata_pending_token[tracker_id] = metadata_next_token;
	metadata_pending_mask[tracker_id] = mask;
	metadata_pending_chunk[tracker_id] = chunk;
	uint16_t token = metadata_next_token;
	irq_unlock(key);
	LOG_INF("Queued metadata request tracker=%u mask=0x%02X chunk=%u token=%u", tracker_id, mask, chunk,
		token);
	return true;
}
void esb_clear_remote_ota_abort(uint8_t tracker_id)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}
	unsigned int key = irq_lock();
	if (tracker_remote_command[tracker_id] == ESB_PONG_FLAG_OTA_ABORT) {
		tracker_remote_command[tracker_id] = ESB_PONG_FLAG_NORMAL;
	}
	irq_unlock(key);
}

// Send remote command to specified tracker
void esb_send_remote_command(uint8_t tracker_id, uint8_t command_flag)
{
	esb_send_remote_command_arg(tracker_id, command_flag, 0);
}

void esb_send_remote_command_arg(uint8_t tracker_id, uint8_t command_flag, uint8_t arg)
{
	if (command_flag == ESB_PONG_FLAG_SET_CHANNEL || command_flag == ESB_PONG_FLAG_CLEAR_CHANNEL ||
	    command_flag == ESB_PONG_FLAG_WOM_CONTROL ||
	    command_flag == ESB_PING_FLAG_WOM_RESULT ||
	    command_flag == ESB_PING_FLAG_WOM_ERROR) {
		return;
	}
	if (tracker_id < MAX_TRACKERS) {
		unsigned int key = irq_lock();
		if (command_flag == ESB_PONG_FLAG_DATA_COLLECT_ON ||
		    command_flag == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON) {
			last_raw_valid[tracker_id] = false;
		}
		if (command_flag == ESB_PONG_FLAG_DATA_COLLECT_ON) {
			raw_arq_reset();
		}
		pending_cmd_arg[tracker_id] = arg;
		tracker_remote_command[tracker_id] = command_flag;
		irq_unlock(key);
		LOG_INF("Remote command %s (0x%02X) queued for tracker %d", esb_pong_flag_name(command_flag),
			command_flag, tracker_id);
	} else {
		LOG_ERR("Invalid tracker ID: %d", tracker_id);
	}
}

// Send remote command to all paired trackers
uint32_t esb_send_remote_command_all(uint8_t command_flag)
{
	if (command_flag == ESB_PONG_FLAG_SET_CHANNEL || command_flag == ESB_PONG_FLAG_CLEAR_CHANNEL ||
	    command_flag == ESB_PONG_FLAG_WOM_CONTROL ||
	    command_flag == ESB_PING_FLAG_WOM_RESULT ||
	    command_flag == ESB_PING_FLAG_WOM_ERROR) {
		return 0;
	}
	int64_t scan_start_time = k_uptime_get();
	k_msleep(REMOTE_COMMAND_ACTIVE_SCAN_MS);

	uint32_t mask = esb_scan_active_mask(scan_start_time);
	esb_publish_flag_mask(command_flag, mask);

	char active_tracker_ids[(MAX_TRACKERS * 4) + 1];
	size_t active_tracker_ids_len = 0;
	uint8_t count = 0;
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		if (mask & BIT(i)) {
			if (active_tracker_ids_len < sizeof(active_tracker_ids)) {
				int written = snprintk(
					&active_tracker_ids[active_tracker_ids_len],
					sizeof(active_tracker_ids) - active_tracker_ids_len,
					count == 0 ? "%u" : ",%u",
					i
				);
				if (written > 0) {
					active_tracker_ids_len
						+= MIN((size_t)written, sizeof(active_tracker_ids) - active_tracker_ids_len - 1);
				}
			}
			count++;
		}
	}

	if (count == 0) {
		active_tracker_ids[0] = '\0';
	}

	LOG_INF(
		"Remote command %s (0x%02X) queued for %d active trackers after %d ms scan; active IDs: %s",
		esb_pong_flag_name(command_flag),
		command_flag,
		count,
		REMOTE_COMMAND_ACTIVE_SCAN_MS,
		count > 0 ? active_tracker_ids : "none"
	);
	return mask;
}

#if defined(CONFIG_TDMA_DIAGNOSTICS)
static int32_t receiver_phase_percentile(
	const uint32_t histogram[RX_PHASE_BUCKETS], uint32_t count,
	uint32_t numerator, uint32_t denominator
)
{
	if (count == 0) {
		return 0;
	}
	uint32_t target = (uint32_t)(((uint64_t)count * numerator + denominator - 1U) / denominator);
	uint32_t accumulated = 0;
	for (uint32_t i = 0; i < RX_PHASE_BUCKETS; i++) {
		accumulated += histogram[i];
		if (accumulated >= target) {
			return (int32_t)i + RX_PHASE_MIN_TICKS;
		}
	}
	return RX_PHASE_MIN_TICKS + RX_PHASE_BUCKETS - 1;
}

static int32_t receiver_rssi_percentile(const struct tdma_stats *stats, uint32_t numerator, uint32_t denominator)
{
	if (stats->rssi_count == 0) {
		return 0;
	}
	/* Histogram index is attenuation magnitude. p10 RSSI therefore uses its p90 index. */
	uint32_t target_numerator = denominator - numerator;
	uint32_t target
		= (uint32_t)(((uint64_t)stats->rssi_count * target_numerator + denominator - 1U) / denominator);
	uint32_t accumulated = 0;
	for (uint32_t i = 0; i < RX_RSSI_BUCKETS; i++) {
		accumulated += stats->rssi_hist[i];
		if (accumulated >= target) {
			return -(int32_t)i;
		}
	}
	return -RX_RSSI_MAX_DBM;
}

void esb_print_health_snapshot(void)
{
	uint32_t observed_mask = 0;
	uint32_t recent_mask = 0;
	uint32_t total_tps = 0;
	uint32_t total_received = 0;
	uint32_t total_gaps = 0;
	uint64_t now = k_uptime_get();

	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		const struct packet_stats *stats = &tracker_stats[i];
		if (stats->total_received > 0 || stats->duplicate_packets > 0) {
			observed_mask |= BIT(i);
		}
		if (stats->last_packet_time > 0 && now - stats->last_packet_time <= PING_TIMEOUT_MS) {
			recent_mask |= BIT(i);
		}
		total_tps += stats->current_tps;
		total_received += stats->total_received;
		total_gaps += stats->total_gaps;
	}

	unsigned int membership_key = irq_lock();
	uint32_t desired_mask = (uint32_t)atomic_get(&tdma_shadow_desired_mask);
	uint32_t pending_join_mask = (uint32_t)atomic_get(&tdma_shadow_pending_join_mask)
		| (desired_mask & ~tdma_active_mask);
	irq_unlock(membership_key);
	LOG_INF(
		"HEALTH TDMA source=data_ping_shadow stored=%u active=%u mask=0x%04x desired=0x%04x recent=0x%04x slot=%u epoch=%u TPS=%u HID=%u recv=%u gaps=%u hid_drop_total=%u cap=%u lvl=%u/%u loss_pm=%u trig=%u rec=%u probe=%u step_ms=%lld",
		stored_trackers,
		tdma_dynamic_active_count,
		(unsigned int)tdma_active_mask,
		(unsigned int)desired_mask,
		(unsigned int)recent_mask,
		tdma_dynamic_slot_ticks,
		tdma_config_epoch,
		total_tps,
		hid_get_current_tps(),
		total_received,
		total_gaps,
		hid_get_total_drop_count(),
		tdma_cap_ladder[tdma_cap_level],
		tdma_cap_level,
		(uint8_t)TDMA_CAP_LEVEL_MAX,
		tdma_loss_last_permille,
		tdma_loss_trigger_streak_ticks,
		tdma_loss_recover_streak_ticks,
		tdma_loss_probe_streak_ticks,
		tdma_last_loss_step_ms
	);
	LOG_INF("HEALTH observed=0x%04x", (unsigned int)observed_mask);
	uint32_t now_ms = (uint32_t)now;
	uint32_t pending_leave_mask = (uint32_t)atomic_get(&tdma_shadow_pending_leave_mask);
	uint32_t ping_seen_mask = (uint32_t)atomic_get(&tdma_shadow_ever_ping_mask);
	uint32_t data_seen_mask = (uint32_t)atomic_get(&tdma_shadow_ever_data_mask);
	uint32_t desired_changed_at_ms = (uint32_t)atomic_get(&tdma_shadow_desired_changed_at_ms);
	uint32_t stable_ms = desired_changed_at_ms > 0 ? now_ms - desired_changed_at_ms : 0;
	LOG_INF(
		"HEALTH SHADOW desired=0x%04x join=0x%04x leave=0x%04x ping=0x%04x data=0x%04x stable_ms=%u",
		(unsigned int)desired_mask,
		(unsigned int)pending_join_mask,
		(unsigned int)pending_leave_mask,
		(unsigned int)ping_seen_mask,
		(unsigned int)data_seen_mask,
		stable_ms
	);
	struct esb_rx_diagnostics radio_diag;
	int radio_diag_err = esb_get_rx_diagnostics(&radio_diag);
	if (radio_diag_err == 0) {
		LOG_INF(
			"HEALTH RADIO crc=%u fifo_full=%u invalid_len=%u dup=%u",
			radio_diag.crc_failures,
			radio_diag.rx_fifo_full,
			radio_diag.invalid_payload_length,
			radio_diag.duplicates
		);
	} else {
		LOG_WRN("HEALTH RADIO unavailable err=%d", radio_diag_err);
	}
	uint8_t slot_owner[MAX_TRACKERS];
	memset(slot_owner, 0xFF, sizeof(slot_owner));
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		if (!(tdma_active_mask & BIT(i))) {
			continue;
		}
		uint8_t slot = (tdma_config_packed[i] >> 24) & 0xFF;
		if (slot < tdma_dynamic_active_count) {
			slot_owner[slot] = i;
		}
	}

	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		const struct packet_stats *stats = &tracker_stats[i];
		if (!(observed_mask & BIT(i))) {
			continue;
		}
		uint8_t assigned_slot = (tdma_config_packed[i] >> 24) & 0xFF;
		uint8_t predecessor_slot = assigned_slot == 0
			? tdma_dynamic_active_count - 1U : assigned_slot - 1U;
		uint8_t predecessor_id = predecessor_slot < MAX_TRACKERS
			? slot_owner[predecessor_slot] : 0xFF;
		uint64_t age_ms = stats->last_packet_time > 0 && stats->last_packet_time <= now
			? now - stats->last_packet_time : 0;
		const struct tdma_stats *tdma = &g_tdma_stats[i];
		if (tdma->count > 0) {
			LOG_INF(
				"HEALTH RX trk=%u slot=%u pred=%u n=%u adj_min_p50_p99_max=%d/%d/%d/%d raw_min_p50_p99_max=%d/%d/%d/%d rssi_p10_p50=%d/%d viol=%u",
				i,
				assigned_slot,
				predecessor_id,
				tdma->count,
				tdma->min_offset,
				receiver_phase_percentile(tdma->phase_hist, tdma->count, 50, 100),
				receiver_phase_percentile(tdma->phase_hist, tdma->count, 99, 100),
				tdma->max_offset,
				tdma->raw_min_offset,
				receiver_phase_percentile(tdma->raw_phase_hist, tdma->count, 50, 100),
				receiver_phase_percentile(tdma->raw_phase_hist, tdma->count, 99, 100),
				tdma->raw_max_offset,
				receiver_rssi_percentile(tdma, 10, 100),
				receiver_rssi_percentile(tdma, 50, 100),
				tdma->violations
			);
			int64_t sync_age_ms = g_last_ping_isr_rx_ticks_raw[i] != 0
				? k_ticks_to_ms_floor64((uint32_t)k_uptime_ticks() - g_last_ping_isr_rx_ticks_raw[i]) : -1;
			LOG_INF(
				"HEALTH SYNC trk=%u bias=%d skew_ppb=%d bias_valid=%u skew_valid=%u ping_age_ms=%lld ping_rej=%u extrap_skip=%u",
				i,
				g_last_ping_rx_time_diff_ticks[i],
				g_bias_ppb_valid[i] ? g_bias_ppb[i] : 0,
				g_last_ping_rx_time_diff_valid[i] ? 1 : 0,
				g_bias_ppb_valid[i],
				sync_age_ms,
				g_ping_reject_count[i],
				g_extrap_skip_count[i]
			);
		}
		LOG_INF(
			"HEALTH trk=%u TPS=%u recv=%u gaps=%u restart=%u age_ms=%llu hid_drop_total=%u",
			i,
			stats->current_tps,
			stats->total_received,
			stats->total_gaps,
			stats->restart_events,
			age_ms,
			hid_get_total_tracker_drop_count(i)
		);
	}
}

#endif /* CONFIG_TDMA_DIAGNOSTICS */

void esb_reset_all_stats(void)
{
	unsigned int key = irq_lock();
	esb_auto_loss_invalidate();
	for (int i = 0; i < MAX_TRACKERS; i++) {
		memset(&tracker_stats[i], 0, sizeof(struct packet_stats));
		ping_counter_initialized[i] = false;
		last_ping_counter[i] = 0;
		last_pong_queued_counter[i] = 0;
	}
	irq_unlock(key);
	tdma_sync_stats_reset();
	tdma_loss_reset_windows();
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	tdma_last_loss_step_ms = 0;
	tdma_loss_last_permille = 0;
#endif
	memset(tdma_prev_recv, 0, sizeof(tdma_prev_recv));
	memset(tdma_prev_gaps, 0, sizeof(tdma_prev_gaps));
	memset(tdma_prev_restarts, 0, sizeof(tdma_prev_restarts));
	LOG_INF("All packet statistics have been reset");
}
// Toggle detailed statistics display on/off
bool esb_toggle_stats_detailed(void)
{
	stats_detailed_enabled = !stats_detailed_enabled;
	stats_detailed_end_time = 0; // No auto-disable when toggling manually
	return stats_detailed_enabled;
}

// Enable detailed statistics display for a specified duration (0 = toggle on/off permanently)
void esb_set_stats_detailed(uint32_t duration_seconds)
{
	if (duration_seconds == 0) {
		// Toggle mode
		stats_detailed_enabled = !stats_detailed_enabled;
		stats_detailed_end_time = 0;
	} else {
		// Timed mode
		stats_detailed_enabled = true;
		stats_detailed_end_time = k_uptime_get() + (int64_t)duration_seconds * 1000;
	}
}

// Get current detailed stats status
bool esb_get_stats_detailed_enabled(void)
{
	return stats_detailed_enabled;
}

void esb_set_channel_change_done_cb(esb_channel_change_done_cb_t cb)
{
	channel_change_done_cb = cb;
}

void esb_set_remote_confirm_cb(esb_remote_confirm_cb_t cb)
{
	remote_confirm_cb = cb;
}

static int channel_apply(uint8_t encoded)
{
	uint8_t old = receiver_rf_channel;
	uint8_t decoded = esb_rf_channel_decode(encoded);
	uint8_t old_decoded = esb_rf_channel_decode(old);
	int err = esb_stop_rx();
	if (err) {
		return err;
	}
	err = esb_flush_rx();
	if (!err) {
		esb_rx_discontinuity();
	}
	if (!err) {
		err = esb_set_rf_channel(decoded == ESB_RF_CHANNEL_DEFAULT ? RADIO_RF_CHANNEL : decoded);
	}
	if (!err) {
		err = esb_start_rx();
	}
	if (!err) {
		err = sys_write_checked(RF_CHANNEL, &encoded, sizeof(encoded));
	}
	if (err) {
		/* Keep the previous durable home authoritative on every failure. */
		int restore = esb_stop_rx();
		if (!restore) {
			restore = esb_flush_rx();
		}
		if (!restore) {
			esb_rx_discontinuity();
			restore = esb_set_rf_channel(old_decoded == ESB_RF_CHANNEL_DEFAULT
				? RADIO_RF_CHANNEL : old_decoded);
		}
		if (!restore) {
			restore = esb_start_rx();
		}
		if (restore) {
			/* Physical home is now unknown: retain ownership and exclude
			 * loss evidence until reboot, like any unsafe radio teardown. */
			esb_radio_fault = true;
			set_status(SYS_STATUS_CONNECTION_ERROR, true);
			LOG_ERR("Channel rollback failed: %d", restore);
		}
		return err;
	}
	receiver_rf_channel = encoded;
	uint8_t home = decoded == ESB_RF_CHANNEL_DEFAULT ? RADIO_RF_CHANNEL : decoded;
	uint8_t old_home = old_decoded == ESB_RF_CHANNEL_DEFAULT ? RADIO_RF_CHANNEL : old_decoded;
	if (home != old_home) {
		LOG_INF("Receiver home channel changed from %u to %u", old_home, home);
	}
	return 0;
}

static int channel_change_begin(uint8_t encoded, uint8_t command, bool manual,
			       uint8_t expected_home, uint32_t generation)
{
	int err = esb_radio_operation_begin();
	if (err) {
		return err;
	}
	if (manual) {
		auto_channel_set_enabled(false);
	}
	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	uint32_t mask = 0;
	for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; ++i) {
		if (stored_tracker_addr[i] != 0) {
			mask |= BIT(i);
		}
	}
	unsigned int key = irq_lock();
	uint8_t home = esb_rf_channel_decode(receiver_rf_channel);
	if (home == ESB_RF_CHANNEL_DEFAULT) {
		home = RADIO_RF_CHANNEL;
	}
	if (!manual && (!auto_channel_decision_valid(generation) ||
			home != expected_home || esb_pairing)) {
		irq_unlock(key);
		k_mutex_unlock(&tracker_store_lock);
		esb_radio_operation_end();
		return -EBUSY;
	}
	pending_channel = encoded;
	channel_command = command;
	channel_target_mask = mask;
	channel_change_automatic = !manual;
	channel_change_generation = generation;
	channel_change_old_home = home;
	channel_change_timeout = k_uptime_get() + CHANNEL_CHANGE_ANNOUNCE_MS;
	esb_auto_loss_invalidate();
	atomic_set(&channel_change_pending, 1);
	irq_unlock(key);
	k_mutex_unlock(&tracker_store_lock);
	return 0;
}

/* Called only by the ESB thread. Never shorten the announcement on echoes:
 * an old echo contains no transaction ID and cannot confirm a new move. */
static void channel_change_poll(void)
{
	if (!atomic_get(&channel_change_pending) || k_uptime_get() < channel_change_timeout) {
		return;
	}
	atomic_set(&channel_change_pending, 0);
	int err = channel_apply(pending_channel);
	if (err) {
		LOG_ERR("Receiver channel migration failed: %d", err);
	}
	if (channel_change_automatic) {
		uint8_t home = esb_rf_channel_decode(receiver_rf_channel);
		if (home == ESB_RF_CHANNEL_DEFAULT) {
			home = RADIO_RF_CHANNEL;
		}
		auto_channel_migration_complete(channel_change_generation,
			channel_change_old_home, home,
			err == 0 && home != channel_change_old_home);
	}
	if (channel_change_done_cb) {
		channel_change_done_cb(err == 0);
	}
	/* Keep admission closed until completion delivery, so a previous
	 * callback cannot consume the next request's completion registration. */
	esb_radio_operation_end();
}

int esb_set_all_trackers_channel(uint8_t channel)
{
	if (channel > 100) {
		return -EINVAL;
	}
	return channel_change_begin(esb_rf_channel_encode(channel), ESB_PONG_FLAG_SET_CHANNEL, true, 0, 0);
}

int esb_clear_all_trackers_channel(void)
{
	return channel_change_begin(ESB_RF_CHANNEL_DEFAULT, ESB_PONG_FLAG_CLEAR_CHANNEL, true, 0, 0);
}

int esb_auto_channel_request(uint8_t channel, uint8_t expected_home, uint32_t generation)
{
	if (channel > 88 || (channel & 1)) {
		return -EINVAL;
	}
	return channel_change_begin(esb_rf_channel_encode(channel), ESB_PONG_FLAG_SET_CHANNEL,
				    false, expected_home, generation);
}

int esb_set_receiver_channel(uint8_t channel)
{
	if (channel > 100) {
		return -EINVAL;
	}
	int err = esb_radio_operation_begin();
	if (err) {
		return err;
	}
	auto_channel_set_enabled(false);
	err = channel_apply(esb_rf_channel_encode(channel));
	esb_radio_operation_end();
	return err;
}

int esb_clear_receiver_channel(void)
{
	int err = esb_radio_operation_begin();
	if (err) {
		return err;
	}
	auto_channel_set_enabled(false);
	err = channel_apply(ESB_RF_CHANNEL_DEFAULT);
	esb_radio_operation_end();
	return err;
}

// Get the receiver's RF channel (decoded: 0xFF = default)
uint8_t esb_get_receiver_channel(void)
{
	return esb_rf_channel_decode(receiver_rf_channel);
}

// Set up unified addresses: pipe 0 for discovery (pairing), pipes 1-7 for paired data
void esb_receive(void)
{
	esb_set_addr_unified();
}


/* Thread context only; failed reads never replace the published roster. */
static int tracker_store_restore(void)
{
	uint64_t addresses[MAX_TRACKERS];
	uint8_t count;
	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	int err = tracker_store_ready ? 0 : tracker_roster_load(&count, addresses);
	if (!err && !tracker_store_ready) {
		unsigned int key = irq_lock();
		memcpy(stored_tracker_addr, addresses, sizeof(stored_tracker_addr));
		stored_trackers = count;
		tracker_store_ready = true;
		irq_unlock(key);
	}
	k_mutex_unlock(&tracker_store_lock);
	if (err) {
		LOG_ERR("Tracker roster unavailable (%d); storage preserved", err);
	}
	return err;
}

static void esb_thread(void)
{
	clocks_start();

	int roster_err = tracker_store_restore();
	uint32_t roster_retry_at = k_uptime_get_32() + 1000;
	uint8_t tracker_count = stored_trackers;

	// Load saved RF channel from NVS if exists (stored value is encoded).
	uint8_t saved_channel = 0xFF;
	sys_read(RF_CHANNEL, &saved_channel, sizeof(saved_channel));
	uint8_t decoded = esb_rf_channel_decode(saved_channel);
	if (decoded != ESB_RF_CHANNEL_DEFAULT) {
		receiver_rf_channel = saved_channel;
		LOG_INF("Loaded RF channel %u from NVS", decoded);
	} else {
		receiver_rf_channel = ESB_RF_CHANNEL_DEFAULT;
		LOG_INF("No saved RF channel, using default %u", RADIO_RF_CHANNEL);
	}

	LOG_INF("%d/%d devices stored", tracker_count, MAX_TRACKERS);


	// Cache receiver device address for ISR pairing responses
	uint64_t *dev_addr = (uint64_t *)NRF_FICR->DEVICEADDR;
	memcpy(receiver_device_addr, dev_addr, 6);
	LOG_INF("Receiver address: %012llX", *dev_addr & 0xFFFFFFFFFFFF);

	// Always start in unified mode: pipe 0 for pairing, pipes 1-7 for data
	esb_receive();
	esb_initialize(false);
	esb_start_rx();

	// Auto-enable pairing mode if no stored trackers
	if (tracker_store_ready && tracker_count == 0) {
		esb_pairing = true;
		esb_auto_loss_invalidate();
		pairing_start_time = k_uptime_get();
		pairing_target_count = 0;
		pairing_initial_count = 0;
		LOG_INF("No stored trackers, pairing mode auto-enabled");
		set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
	}
	auto_channel_init(receiver_device_addr);

	while (1) {
		/* Retry transient storage failures on the existing maintenance loop,
		 * without delaying radio/USB initialization or adding a worker. */
		uint32_t now_ms = k_uptime_get_32();
		if (roster_err && roster_err != -EINVAL &&
		    (int32_t)(now_ms - roster_retry_at) >= 0) {
			roster_err = tracker_store_restore();
			roster_retry_at = k_uptime_get_32() + 1000;
			if (!roster_err && stored_trackers == 0) {
				esb_start_pairing();
			}
		}
		tracker_events_process(k_uptime_get_32());
		// Process new device pairing requests (non-blocking)
		if (esb_pairing) {
			process_pairing_queue();

			// Check for pairing timeout
			if (PAIRING_TIMEOUT_SECONDS > 0 && pairing_start_time > 0) {
				int64_t elapsed = k_uptime_get() - pairing_start_time;
				if (elapsed >= (PAIRING_TIMEOUT_SECONDS * 1000)) {
					LOG_INF("Pairing mode timeout (%d seconds), auto-exiting", PAIRING_TIMEOUT_SECONDS);
					esb_finish_pair();
				}
			}

			// Check for target count reached
			if (pairing_target_count > 0) {
				uint8_t new_devices = stored_trackers - pairing_initial_count;
				if (new_devices >= pairing_target_count) {
					// Mark when target was reached (only once)
					if (pairing_target_reached_time == 0) {
						pairing_target_reached_time = k_uptime_get();
						pairing_new_devices_blocked = true; // Block new devices, allow re-pairing only
						LOG_INF(
							"Pairing target count reached (%u new devices), exiting in %d seconds...",
							new_devices,
							PAIRING_EXIT_DELAY_MS / 1000
						);
					}
					// Check if delay has passed
					int64_t elapsed_since_target = k_uptime_get() - pairing_target_reached_time;
					if (elapsed_since_target >= PAIRING_EXIT_DELAY_MS) {
						esb_finish_pair();
					}
				}
			}
		}

		channel_change_poll();

		k_msleep(100);
	}
}
