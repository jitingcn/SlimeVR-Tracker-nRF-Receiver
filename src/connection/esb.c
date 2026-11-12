#if defined(CONFIG_RELAY)
#include "frame_ledger.h"
#endif
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
#if defined(CONFIG_RELAY)
#include "relay_demand.h"
#endif

#include <errno.h>
#if defined(CONFIG_RELAY)
#include <zephyr/random/random.h>
#endif
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/kernel.h>
#if defined(CONFIG_RELAY)
#include <zephyr/drivers/timer/nrf_rtc_timer.h>
#endif
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
#if defined(CONFIG_RELAY)
#include "connection/relay_protocol.h"
#include "connection/relay_coord.h"
#endif

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
#define TDMA_BASE_SLOT_TICKS 16
#define TDMA_SLOT_TICKS_MAX_LEVEL0 64
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

struct tracker_slot {
	/* One aligned word: assigned slot, total slots, slot ticks, epoch. */
	uint32_t tdma_config_packed;
	uint32_t tdma_prev_recv;
	uint32_t tdma_prev_gaps;
	uint32_t tdma_prev_restarts;
	uint8_t last_packet_sequence;
	bool sequence_initialized;
	uint8_t last_ping_counter;
	bool ping_counter_initialized;
	uint64_t last_ping_time;
	uint8_t last_pong_queued_counter;
	uint8_t pipe_mismatch_count;
	uint8_t crc_error_count;
	uint16_t last_raw_seq;
	bool last_raw_valid;
	/* Shared ACK tuples are published under irq_lock with their command flag. */
	uint8_t tracker_remote_command;
	uint8_t pending_cmd_arg;
	uint16_t tracker_test_on_tps;
	int16_t pending_sens_data[3];
	uint8_t pending_sens_auto_axis;
	uint16_t pending_sens_auto_revolutions;
	/* Active and pending metadata requests have independent tokens. */
	uint8_t metadata_active_mask;
	uint8_t metadata_active_chunk;
	uint16_t metadata_active_token;
	uint8_t metadata_pending_mask;
	uint8_t metadata_pending_chunk;
	uint16_t metadata_pending_token;
	uint16_t wom_token;
	uint16_t wom_minutes;
	uint16_t wom_error;
	bool wom_complete;
	bool wom_advertised;
	uint8_t wom_first_ping_counter;
	struct packet_stats stats;
	uint32_t decision_received;
	uint32_t decision_gaps;
};

static struct tracker_slot tracker_slots[MAX_TRACKERS];

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
#if defined(CONFIG_RELAY)
/* Peer relay coordinator (see the relay section below). */
static struct relay_coord relay_coord;
static bool relay_user_enabled = true;
static uint16_t relay_forward_published;
#endif
static void tdma_sync_stats_reset(void);

static void tdma_loss_controller_tick(int64_t now);
static struct esb_payload rx_payload;
static atomic_t home_beacon_tx_result;

struct pairing_event {
	uint8_t packet[8];
};

// Queue pairing packets from the ISR context to the pairing worker thread.
static char __noinit __aligned(4)
	_k_fifo_buf_esb_pairing_msgq[8 * sizeof(struct pairing_event)];
static STRUCT_SECTION_ITERABLE(k_msgq, esb_pairing_msgq) =
	Z_MSGQ_INITIALIZER(esb_pairing_msgq, _k_fifo_buf_esb_pairing_msgq,
			   sizeof(struct pairing_event), 8);

static K_MUTEX_DEFINE(tracker_store_lock);
static bool tracker_store_ready;

/* Tokens survive slot cleanup so late replies cannot confirm a replacement. */
static uint16_t metadata_next_token;
static uint16_t wom_next_token;
/* Sticky state for commands addressed to "all": newly active trackers must
 * converge to the same test on/off state. Confirmed means the tracker has
 * executed and acknowledged the current desired state. */
static atomic_t test_all_state_valid;
static atomic_t test_all_enabled;
static atomic_t test_all_broadcasting;
static atomic_t test_all_confirmed_mask;
static uint16_t test_all_requested_tps;
static uint16_t test_all_effective_tps;
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

#if defined(CONFIG_RELAY)
static uint32_t tdma_admissible_mask(uint32_t desired);

#endif
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
#if defined(CONFIG_RELAY)
	if (tdma_admissible_mask((uint32_t)atomic_get(&tdma_shadow_desired_mask)) == tdma_active_mask
		&& !atomic_get(&tdma_shadow_pending_join_mask)) {
#else
	if ((uint32_t)atomic_get(&tdma_shadow_desired_mask) == tdma_active_mask
	    && !atomic_get(&tdma_shadow_pending_join_mask)) {
#endif
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
#if defined(CONFIG_RELAY)
	/* Members beyond max_trackers stay desired but can never be admitted;
	 * comparing the raw mask would re-arm an already expired deadline. */
	if (tdma_admissible_mask(desired_mask) == tdma_active_mask) {
#else
	if (desired_mask == tdma_active_mask) {
#endif
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

#if !defined(CONFIG_RELAY)
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


#endif
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
#if defined(CONFIG_RELAY)
	uint8_t total_slots = active_count + relay_mask_count(relay_forward_published);
	uint16_t capacity = (uint16_t)(32768U / ((uint32_t)sticks * total_slots));
#else
	uint16_t capacity = (uint16_t)(32768U / ((uint32_t)sticks * active_count));
#endif
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
	uint8_t pending_cmd = tracker_slots[tracker_id].tracker_remote_command;
	if (pending_cmd == ESB_PONG_FLAG_TEST_MODE_ON || pending_cmd == ESB_PONG_FLAG_TEST_MODE_OFF) {
		tracker_slots[tracker_id].tracker_remote_command = ESB_PONG_FLAG_NORMAL;
	}
	atomic_and(&test_all_confirmed_mask, (atomic_val_t)~BIT(tracker_id));
	atomic_set(&test_all_ready_after_ms[tracker_id], (atomic_val_t)((uint32_t)now_ms + TEST_ALL_JOIN_CONFIG_GRACE_MS));
}

static void test_all_update_effective(uint8_t active_count, int64_t now, bool layout_changed)
{
	unsigned int key = irq_lock();
	if (!atomic_get(&test_all_state_valid) || !atomic_get(&test_all_enabled)) {
		irq_unlock(key);
		return;
	}
	uint16_t requested = test_all_requested_value();
	uint16_t capacity = test_all_capacity_tps(active_count);
	uint16_t effective = MIN(requested, capacity);
	if (effective == test_all_effective_tps) {
		irq_unlock(key);
		return;
	}
	test_all_effective_tps = effective;
	uint32_t active_mask = tdma_active_mask;
	atomic_and(&test_all_confirmed_mask, (atomic_val_t)~active_mask);
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		if (!(active_mask & BIT(i))) {
			continue;
		}
		tracker_slots[i].tracker_test_on_tps = test_all_wire_tps();
		atomic_set(
			&test_all_ready_after_ms[i],
			(atomic_val_t)((uint32_t)now + (layout_changed ? TEST_ALL_JOIN_CONFIG_GRACE_MS : 0))
		);
	}
	irq_unlock(key);
	LOG_INF(
		"Sticky TEST_MODE_ON reclamp: requested=%u effective=%u active=%u capacity=%u",
		requested,
		effective,
		active_count,
		capacity
	);
}

static void test_all_note_layout(uint32_t old_mask, uint32_t new_mask, int64_t now)
{
	unsigned int key = irq_lock();
	if (!atomic_get(&test_all_state_valid)) {
		irq_unlock(key);
		return;
	}
	uint32_t joined_mask = new_mask & ~old_mask;
	if (joined_mask == 0) {
		irq_unlock(key);
		return;
	}
	atomic_and(&test_all_confirmed_mask, (atomic_val_t)~joined_mask);
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		if (joined_mask & BIT(i)) {
			/* Refresh the joined tracker's TPS even on a same-count swap:
			 * update_effective skips unchanged effective rates. */
			if (atomic_get(&test_all_enabled)) {
				tracker_slots[i].tracker_test_on_tps = test_all_wire_tps();
			}
			/* First allow a NORMAL PONG to deliver the new TDMA layout;
			 * then converge the newly active tracker to test all state. */
			atomic_set(&test_all_ready_after_ms[i], (atomic_val_t)((uint32_t)now + TEST_ALL_JOIN_CONFIG_GRACE_MS));
		}
	}
	irq_unlock(key);
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
				|| tracker_slots[i].tracker_remote_command != ESB_PONG_FLAG_NORMAL) {
				continue;
			}
			if (enabled) {
				tracker_slots[i].tracker_test_on_tps = test_all_wire_tps();
			}
			__asm__ volatile("" ::: "memory"); /* TPS published before the flag the ISR acts on */
			/* Re-read right before publishing: if an EVENT-path recovery
			 * re-armed the grace during iteration, skip this cycle and let
			 * the next reconcile pass handle the tracker. */
			if ((uint32_t)atomic_get(&test_all_ready_after_ms[i]) != ready_after) {
				continue;
			}
			tracker_slots[i].tracker_remote_command = command;
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


#if defined(CONFIG_RELAY)
/* One layout publication: computed by tdma_recalculate(), applied at once
 * or, for an all-relay-capable fleet, staged until activation_ticks (receiver
 * ticks, the PONG timestamp clock) so every tracker switches together. See
 * "Staged layout activation" in relay_protocol.h. */
static struct relay_runtime_settings relay_settings = {
	CONFIG_RELAY_MAX_TRACKERS,
	CONFIG_RELAY_MAX_REPAIRS,
	CONFIG_RELAY_SLOT_TICKS,
	CONFIG_RELAY_TPS_MIN,
	CONFIG_RELAY_TPS_MAX,
	CONFIG_RELAY_REPAIR_FLOOR,
	0,
};
static struct relay_runtime_settings relay_applied_settings;
static uint32_t relay_settings_generation = 1, relay_applied_generation;
static struct relay_geometry relay_receiver_geometry = {16, 1, 16, 120, 288, 1, 0};
const struct relay_geometry *relay_geometry_active = &relay_receiver_geometry;
struct relay_timeline relay_timeline_active;

static struct relay_demand relay_demand_state;
static uint8_t relay_frame_repairs(uint32_t frame)
{
	if (relay_applied_settings.repair_floor == relay_applied_settings.max_repairs) {
		return relay_demand_slots(frame, relay_applied_settings.max_repairs);
	}
	uint32_t offset = frame - relay_demand_state.frame;
	return offset < 3U ? relay_demand_state.queued[offset] : relay_demand_slots(frame, relay_demand_state.target);
}

static void relay_frame_announce(uint32_t frame, uint32_t start)
{
	if (relay_applied_settings.repair_floor < relay_applied_settings.max_repairs) {
		relay_demand_advance(&relay_demand_state, frame, &relay_applied_settings);
	}
	struct relay_runtime_anchor a;
	relay_runtime_anchor_set(
		&a,
		frame,
		start + RELAY_V3_B_PREAMBLE,
		RELAY_V3_DIRECT_SLOTS,
		relay_frame_repairs(frame),
		relay_frame_repairs(frame + 1U),
		relay_frame_repairs(frame + 2U),
		RELAY_V3_SLOT_TICKS,
		relay_geometry_current()->tps
	);
	relay_runtime_anchor_second_b(
		&a,
		relay_frame_repairs(frame),
		relay_frame_repairs(frame + 1U),
		relay_frame_repairs(frame + 2U),
		relay_geometry_current()->tps,
		relay_applied_settings.second_b
	);
	relay_timeline_announce(&relay_timeline_active, &a);
}

static void relay_settings_load(void)
{
	uint8_t record[10];
	struct relay_runtime_settings value = relay_settings;
	int length = sys_read_checked(RELAY_LAYOUT_SETTINGS, record, sizeof(record));
	if (length == 9 && record[0] == 1U && record[8] == relay_crc8(7, record, 8)) {
		value = (struct relay_runtime_settings){
			record[1],
			record[2],
			record[0] == 3U ? record[3] & 63U : record[3],
			relay_get_be16(record + 4),
			relay_get_be16(record + 6),
			record[2],
			0
		};
	}
	if (length == sizeof(record) && (record[0] == 2U || record[0] == 3U) && record[9] == relay_crc8(7, record, 9)) {
		value = (struct relay_runtime_settings){
			record[1],
			record[2],
			record[0] == 3U ? record[3] & 63U : record[3],
			relay_get_be16(record + 4),
			relay_get_be16(record + 6),
			record[8],
			record[0] == 3U ? record[3] >> 6 : 0U
		};
	}
	/* Migrate only the exact historical default tuple. Other choices survive. */
	if ((length == 9 || length == 10) && record[0] <= 2U && value.max_trackers == 11 && value.max_repairs == 5
		&& value.slot_ticks == 16 && value.tps_min == 100 && value.tps_max == 120 && value.repair_floor == 5) {
		value.max_trackers = 16;
		value.tps_min = 120;
	}
	if (!relay_runtime_validate(&value, IS_ENABLED(CONFIG_RELAY_EXPERIMENTAL_SLOT_15))) {
		value = (struct relay_runtime_settings){16, 5, 16, 120, 120, 5, 0};
		LOG_WRN("Invalid relay settings; using 120-mode defaults");
	}
	relay_settings = value;
}

struct tdma_publication {
	struct relay_runtime_settings settings;
	struct relay_geometry geometry;
	uint32_t settings_generation;
	uint32_t old_mask;
	uint32_t new_mask;
	uint16_t forward_mask;
	uint8_t active_count;
	uint8_t forward_count;
	uint8_t slot_ticks;
	uint8_t epoch;
	uint8_t cap_level;
	bool mask_changed;
	bool forward_changed;
	bool test_all_layout;
	bool ladder_step;
	uint8_t profile;
};
static uint8_t relay_profile;
static uint32_t relay_origin;
/* Frame anchor survives profile-1 layout changes that clear current grants. */
static uint32_t relay_frame_anchor, relay_start_anchor;
/* Epoch of the active layout; tdma_config_epoch already names a staged one. */
static uint8_t relay_epoch;
static uint32_t relay_session;
static struct relay_control relay_grants;
static bool relay_repair_delivery;
static uint32_t relay_learn_lock_max;
#ifdef CONFIG_RELAY_IRQ_DIAGNOSTICS
static uint32_t irq_probe_plan_max, irq_probe_plan_count;
static uint32_t irq_probe_tick_max, irq_probe_tick_count;
#define IRQ_PROBE_BEGIN(name) uint32_t irq_probe_start_##name = DWT->CYCCNT
#define IRQ_PROBE_RESTART(name) irq_probe_start_##name = DWT->CYCCNT
#define IRQ_PROBE_END(name) do { \
    uint32_t elapsed = DWT->CYCCNT - irq_probe_start_##name; \
    irq_probe_##name##_max = MAX(irq_probe_##name##_max, elapsed); \
    irq_probe_##name##_count++; \
} while (0)
#else
#define IRQ_PROBE_BEGIN(name)
#define IRQ_PROBE_RESTART(name)
#define IRQ_PROBE_END(name)
#endif

static uint32_t relay_b2_sent, relay_b2_failed;
static int relay_b2_err;
static uint32_t relay_b_sent, relay_b_failed;
/* B timing, ticks: launch past preamble - TX_OFFSET (count, max), plan and grant work (max). */
static uint32_t relay_b_late, relay_b_late_max, relay_b_plan_max, relay_b_grant_max;
#ifdef CONFIG_RELAY_CYCLE_DIAGNOSTICS
static uint32_t relay_b_arm_cycles_max;
static uint32_t relay_b_encode_cycles_max;
#endif
static uint32_t relay_b_entry_late, relay_b_entry_max;
#ifdef CONFIG_RELAY_CYCLE_DIAGNOSTICS
static uint32_t relay_rx_cycles_max, relay_rx_packets;
static uint64_t relay_rx_cycles_total;
#endif
static int relay_b_err;
static uint32_t relay_layout_generation;
static uint32_t relay_direct_start;
static bool relay_direct_pose;
static struct {
	struct tdma_publication p;
	uint32_t activation_ticks;
	bool valid;
} tdma_staged;

static bool tdma_staged_due_locked(uint32_t now_ticks)
{
	return tdma_staged.valid && (int32_t)(now_ticks - tdma_staged.activation_ticks) >= 0;
}

/* Members the next layout can admit: stored slots, then in v3 only the
 * first max_trackers of those. */
static uint32_t tdma_admissible_mask(uint32_t desired)
{
	uint32_t stored_mask
		= stored_trackers >= MAX_TRACKERS ? UINT32_MAX : (stored_trackers > 0 ? BIT(stored_trackers) - 1U : 0U);
	uint32_t candidates = desired & stored_mask;
	if (!relay_coord.v3_requested) {
		return candidates;
	}
	uint32_t selected = 0;
	for (unsigned n = 0; n < relay_settings.max_trackers && candidates; n++) {
		uint32_t bit = candidates & (0U - candidates);
		selected |= bit;
		candidates &= candidates - 1U;
	}
	return selected;
}

/* How long the stats owner may sleep while a membership change is pending.
 * A staged layout cannot be applied before its activation tick, so an
 * expired cohort deadline must not turn that wait into a busy loop. */
static uint32_t tdma_membership_wait_ms(uint32_t maximum_ms)
{
	unsigned int key = irq_lock();
	if (tdma_staged.valid) {
		int32_t staged_ticks = (int32_t)(tdma_staged.activation_ticks - (uint32_t)k_uptime_ticks());
		maximum_ms = staged_ticks > 0 ? MIN(maximum_ms, k_ticks_to_ms_ceil32((uint32_t)staged_ticks)) : 0;
	} else if (tdma_membership_deadline_armed) {
		int64_t now = k_uptime_get();
		int32_t cohort_remaining = (int32_t)(tdma_membership_deadline_ms - (uint32_t)now);
		int64_t gap_remaining = tdma_last_reconfig_valid ? TDMA_RECONFIG_MIN_MS - (now - tdma_last_reconfig_time) : 0;
		int64_t remaining = MAX((int64_t)cohort_remaining, gap_remaining);
		maximum_ms = remaining > 0 ? MIN(maximum_ms, (uint32_t)remaining) : 0;
	}
	irq_unlock(key);
	return maximum_ms;
}

/* Layout tuple a PONG to `id` carries (ISR or irq_lock): a staged layout
 * past its activation tick is already the truth for the trackers, even before
 * the stats thread applies it; wake that thread to do so. */
static uint32_t tdma_advertised_config(uint8_t id, uint32_t now_ticks)
{
	if (!tdma_staged_due_locked(now_ticks) || id >= MAX_TRACKERS) {
		return id < MAX_TRACKERS ? tracker_slots[id].tdma_config_packed : 0;
	}
	const struct tdma_publication *p = &tdma_staged.p;
	uint8_t assigned = 0xFF;
	if (p->new_mask & BIT(id)) {
		assigned = (uint8_t)relay_member_slot((uint16_t)p->new_mask, id);
	}
	k_sem_give(&tdma_membership_wake);
	return tdma_pack_config(assigned, p->active_count + p->forward_count, p->slot_ticks, p->epoch);
}

/* Every tracker affected by the change runs relay firmware that follows
 * staged activation (relay_protocol.h); otherwise publish immediately. */
static bool tdma_staging_supported_locked(uint32_t old_mask, uint32_t new_mask)
{
	uint32_t affected = old_mask | new_mask;
	if (!old_mask || !relay_user_enabled) {
		return false;
	}
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		if ((affected & BIT(i)) && !relay_coord_is_capable(&relay_coord, i)) {
			return false;
		}
	}
	return true;
}

/* Caller holds irq_lock. */
static void tdma_apply_locked(const struct tdma_publication *p, int64_t now)
{
	uint8_t slot = 0;
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		uint8_t assigned = 0xFF;
		if (p->new_mask & BIT(i)) {
			assigned = slot++;
		}
		tracker_slots[i].tdma_config_packed = tdma_pack_config(
			assigned, p->active_count + p->forward_count, p->slot_ticks, p->epoch);
	}
	relay_forward_published = p->forward_mask;
	relay_coord_set_forward_layout(&relay_coord, p->forward_mask);
	relay_applied_settings = relay_runtime_for_members(p->settings, p->active_count);
	relay_coord.cache_frames = relay_applied_settings.repair_floor < relay_applied_settings.max_repairs ? 5 : 3;
	if (p->profile) {
		uint32_t seed = relay_profile ? relay_timeline_active.frame + 1U : 0U;
		relay_receiver_geometry = p->geometry;
		relay_origin = tdma_staged.activation_ticks;
		relay_frame_anchor = seed;
		relay_start_anchor = relay_origin;
		relay_timeline_clear(&relay_timeline_active);
		relay_demand_reset(&relay_demand_state, seed, relay_applied_settings.repair_floor);
		relay_frame_announce(seed, relay_origin);
	}
	relay_applied_generation = p->settings_generation;
	relay_profile = p->profile;
	relay_layout_generation++;
	relay_epoch = p->epoch;
	if (p->profile) {
		relay_coord_fast_reset(&relay_coord);
		memset(&relay_grants, 0, sizeof(relay_grants));
	}
	tdma_dynamic_active_count = p->active_count;
	tdma_dynamic_slot_ticks = p->slot_ticks;
	tdma_active_mask = p->new_mask;
	if (!atomic_get(&tdma_shadow_pending_join_mask)) {
		tdma_membership_deadline_armed = false;
	}
	if (p->mask_changed || !p->test_all_layout) {
		tdma_published_cap_level = p->cap_level;
	}
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	if (p->ladder_step && !p->test_all_layout) {
		tdma_last_loss_step_ms = now;
	}
#else
	ARG_UNUSED(now);
#endif
	esb_auto_loss_invalidate();
}

/* After irq_unlock: test-all bookkeeping and the log line. */
static void tdma_apply_finish(const struct tdma_publication *p, int64_t now, bool staged)
{
	if (!p->active_count) {
		return;
	}
	test_all_note_layout(p->old_mask, p->new_mask, now);
	test_all_update_effective(p->active_count, now, true);
	uint32_t frame_ticks = (uint32_t)p->slot_ticks * (p->active_count + p->forward_count);
	uint32_t est_tps = 32768 / frame_ticks;
	LOG_INF(
		"TDMA reconfig: mode=ping_guard_v2 reason=%s%s active=%u fwd=0x%04x slot=%u frame=%u opportunity=%u/trk aggregate=%u/s cap=%u lvl=%u/%u epoch=%u members=0x%04x",
		p->mask_changed ? "membership" : (p->forward_changed ? "relay_forward"
			: (p->test_all_layout ? "test_all" : "loss_ladder")),
		staged ? " (staged)" : "",
		p->active_count,
		p->forward_mask,
		p->slot_ticks,
		frame_ticks,
		est_tps,
		32768 / p->slot_ticks,
		tdma_cap_ladder[p->cap_level],
		p->cap_level,
		(uint8_t)TDMA_CAP_LEVEL_MAX,
		p->epoch,
		p->new_mask
	);
}

#endif
/* Caller holds tracker_store_lock. Forced administrative cleanup bypasses the
 * ordinary spacing, but only an actual layout change moves its anchor. */
static void tdma_recalculate(bool forced)
{
	bool diagnostics_reset = false;

	for (;;) {
		unsigned int key = irq_lock();
		int64_t now = k_uptime_get();
#if defined(CONFIG_RELAY)
		/* Transitions are serialized: one staged layout at a time. */
		if (tdma_staged.valid) {
			if ((!forced || relay_profile || tdma_staged.p.profile) && !tdma_staged_due_locked((uint32_t)k_uptime_ticks())) {
				irq_unlock(key);
				return;
			}
			struct tdma_publication due = tdma_staged.p;
			tdma_staged.valid = false;
			tdma_apply_locked(&due, now);
			irq_unlock(key);
			tdma_apply_finish(&due, now, true);
			continue;
		}
		if (!forced
			&& (diagnostics_reset || atomic_get(&tdma_shadow_pending_join_mask)
				|| (tdma_admissible_mask((uint32_t)atomic_get(&tdma_shadow_desired_mask)) & ~tdma_active_mask))) {
#else
		if (!forced && (diagnostics_reset || atomic_get(&tdma_shadow_pending_join_mask)
		    || ((uint32_t)atomic_get(&tdma_shadow_desired_mask) & ~tdma_active_mask))) {
#endif
			(void)tdma_shadow_merge_locked((uint32_t)now);
		}
		uint32_t old_mask = tdma_active_mask;
#if defined(CONFIG_RELAY)
		uint32_t new_mask = tdma_admissible_mask((uint32_t)atomic_get(&tdma_shadow_desired_mask));
#else
		uint32_t stored_mask = stored_trackers >= MAX_TRACKERS
			? UINT32_MAX : (stored_trackers > 0 ? BIT(stored_trackers) - 1U : 0U);
		uint32_t new_mask = (uint32_t)atomic_get(&tdma_shadow_desired_mask) & stored_mask;
#endif
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
#if defined(CONFIG_RELAY)
		/* Relay forwarding slots follow the direct members' slots. They
		 * count toward the per-tracker rate cap like any other slot. */
		uint16_t forward_mask = active_count
			? relay_coord_forward_wanted(&relay_coord, (uint16_t)new_mask) : 0;
		uint8_t profile = relay_coord_v3_ready(&relay_coord, (uint16_t)new_mask) ? RELAY_PROFILE_120HZ : 0;
		if (profile) {
			forward_mask = 0;
		}
		uint8_t forward_count = relay_mask_count(forward_mask);
		bool forward_changed = forward_mask != relay_forward_published;
		uint8_t slot_ticks = profile ? relay_settings.slot_ticks : tdma_slot_ticks_for(active_count + forward_count);
#else
		uint8_t slot_ticks = tdma_slot_ticks_for(active_count);
#endif
		bool test_all_layout = atomic_get(&test_all_enabled) != 0;
		bool ladder_step = tdma_cap_level != tdma_published_cap_level;

		/* An initially empty layout is not a publication. */
		if (!mask_changed && active_count == 0) {
			tdma_dynamic_slot_ticks = TDMA_BASE_SLOT_TICKS;
			tdma_published_cap_level = tdma_cap_level;
			irq_unlock(key);
			return;
		}
#if defined(CONFIG_RELAY)
		if (!mask_changed && !forward_changed && profile == relay_profile && slot_ticks == tdma_dynamic_slot_ticks
			&& relay_settings_generation == relay_applied_generation) {
#else
		if (!mask_changed && slot_ticks == tdma_dynamic_slot_ticks) {
#endif
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
#if defined(CONFIG_RELAY)
		struct relay_geometry geometry = relay_receiver_geometry;
		if (profile) {
			(void)relay_geometry_make_b(
				&geometry,
				active_count,
				relay_runtime_for_members(relay_settings, active_count).max_repairs,
				relay_settings.slot_ticks,
				relay_settings.tps_max,
				relay_runtime_for_members(relay_settings, active_count).second_b
			);
#else
		uint8_t slot = 0;
		for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
			uint8_t assigned = 0xFF;
			if (new_mask & BIT(i)) {
				assigned = slot++;
			}
			tracker_slots[i].tdma_config_packed = tdma_pack_config(assigned, active_count, slot_ticks, epoch);
#endif
		}
#if defined(CONFIG_RELAY)
		struct tdma_publication pub = {
			.settings = relay_settings,
			.settings_generation = relay_settings_generation,
			.geometry = geometry,
			.old_mask = old_mask,
			.new_mask = new_mask,
			.forward_mask = forward_mask,
			.active_count = active_count,
			.forward_count = forward_count,
			.slot_ticks = slot_ticks,
			.epoch = epoch,
			.cap_level = tdma_cap_level,
			.mask_changed = mask_changed,
			.forward_changed = forward_changed,
			.test_all_layout = test_all_layout,
			.ladder_step = ladder_step,
			.profile = profile,
		};
#else
		tdma_dynamic_active_count = active_count;
		tdma_dynamic_slot_ticks = slot_ticks;
		tdma_active_mask = new_mask;
#endif
		tdma_last_reconfig_time = now;
		tdma_last_reconfig_valid = true;
#if defined(CONFIG_RELAY)
		if ((!forced || relay_profile || profile) && active_count && tdma_staging_supported_locked(old_mask, new_mask)) {
			tdma_staged.p = pub;
			tdma_staged.activation_ticks = (uint32_t)k_uptime_ticks()
				+ (uint32_t)k_ms_to_ticks_ceil32(RELAY_LAYOUT_ACTIVATION_MS);
			/* Quiet-zone admission fences the old layout; this activation
			 * tick is the first start of the new membership's timeline. */
			tdma_staged.valid = true;
			irq_unlock(key);
			LOG_INF("TDMA layout epoch %u staged: activates in %u ms", epoch,
				(unsigned)RELAY_LAYOUT_ACTIVATION_MS);
#else
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
#endif
			return;
		}
#if defined(CONFIG_RELAY)
		tdma_staged.activation_ticks = (uint32_t)k_uptime_ticks();
		tdma_apply_locked(&pub, now);
		irq_unlock(key);
		tdma_apply_finish(&pub, now, false);
#else
		test_all_note_layout(old_mask, new_mask, now);
		test_all_update_effective(active_count, now, true);
		uint32_t frame_ticks = (uint32_t)slot_ticks * active_count;
		uint32_t est_tps = 32768 / frame_ticks;
		LOG_INF(
			"TDMA reconfig: mode=ping_guard_v2 reason=%s active=%u slot=%u frame=%u opportunity=%u/trk aggregate=%u/s cap=%u lvl=%u/%u epoch=%u members=0x%04x",
			mask_changed ? "membership" : (test_all_layout ? "test_all" : "loss_ladder"),
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
#endif
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
static atomic_t pairing_new_devices_blocked; // When set, only allow re-pairing of known devices
#define PAIRING_EXIT_DELAY_MS 15000                       // Delay before exiting after target count reached

// Cached receiver device address (set once at init, used by ISR for pairing responses)
static uint8_t receiver_device_addr[6] = {0};


static uint32_t auto_loss_reset_epoch;
/* Decision evidence is distinct from lifetime/status diagnostics. A fence
 * drops only the interval crossing a known boundary, never decoder state. */
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
	tracker_slots[tracker_id].decision_received++;
	tracker_slots[tracker_id].decision_gaps += gaps;
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
	for (uint8_t i = 0; i < MAX_TRACKERS; ++i) {
		tracker_slots[i].sequence_initialized = false;
	}
	esb_auto_loss_invalidate();
	irq_unlock(key);
}

void esb_get_auto_loss_snapshot(struct esb_auto_loss_snapshot *out)
{
	BUILD_ASSERT(MAX_TRACKERS == ESB_AUTO_LOSS_TRACKERS);
	unsigned int key = irq_lock();
	for (uint8_t i = 0; i < MAX_TRACKERS; ++i) {
		out->tracker[i].received = tracker_slots[i].decision_received;
		out->tracker[i].gaps = tracker_slots[i].decision_gaps;
		out->tracker[i].restarts = tracker_slots[i].stats.restart_events;
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
static struct k_spinlock stats_display_lock;
static bool stats_detailed_enabled;
static int64_t stats_detailed_end_time; // 0 = no auto-disable
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
	uint32_t phase_hist[RX_PHASE_BUCKETS];
	uint32_t raw_phase_hist[RX_PHASE_BUCKETS];
	int32_t raw_min_offset;
	int32_t raw_max_offset;
	uint32_t rssi_hist[RX_RSSI_BUCKETS];
	uint32_t rssi_count;
};


struct tracker_diag {
	struct tdma_stats stats;
	int32_t g_last_ping_rx_time_diff_ticks;
	bool g_last_ping_rx_time_diff_valid;
	/* RADIO timestamps avoid EVENT IRQ scheduling delay. */
	uint32_t g_ping_isr_rx_ticks;
	bool g_ping_isr_rx_ticks_valid;
	uint32_t g_last_isr_rx_ticks;
	bool g_last_isr_rx_valid;
	/* 0 = unavailable, 1 = initial estimate, 2 = mature estimate. */
	uint8_t g_bias_ppb_valid;
	int32_t g_bias_ppb;
	int32_t g_bias_ref_offset;
	uint32_t g_bias_ref_ticks;
	uint32_t g_last_ping_isr_rx_ticks_raw;
	uint8_t g_ping_dirty_streak;
	uint32_t g_ping_reject_count;
	uint32_t g_extrap_skip_count;
};

static struct tracker_diag tracker_diags[MAX_TRACKERS];
/* Full slot reset retains its outer IRQ exclusion on the single-core receiver. */
static struct k_spinlock tdma_timestamp_lock;
#if defined(CONFIG_RELAY)
static uint32_t tdma_stats_generation[MAX_TRACKERS];
#endif

#endif

static void tdma_sync_stats_reset(void)
{
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	for (uint8_t i = 0; i < MAX_TRACKERS; ++i) {
#if defined(CONFIG_RELAY)
		tdma_stats_generation[i]++;
#endif
		memset(&tracker_diags[i].stats, 0, sizeof(tracker_diags[i].stats));
		tracker_diags[i].g_last_ping_rx_time_diff_valid = false;
		tracker_diags[i].g_bias_ppb_valid = 0;
		tracker_diags[i].g_last_ping_isr_rx_ticks_raw = 0;
		tracker_diags[i].g_last_ping_rx_time_diff_ticks = 0;
		tracker_diags[i].g_ping_dirty_streak = 0;
		tracker_diags[i].g_ping_reject_count = 0;
		tracker_diags[i].g_extrap_skip_count = 0;
	}
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
		uint32_t prev_r = tracker_slots[i].tdma_prev_recv;
		uint32_t prev_g = tracker_slots[i].tdma_prev_gaps;
		uint32_t restarts = evidence.tracker[i].restarts;
		bool restarted = restarts != tracker_slots[i].tdma_prev_restarts;
		tracker_slots[i].tdma_prev_restarts = restarts;
		tracker_slots[i].tdma_prev_recv = r;
		tracker_slots[i].tdma_prev_gaps = g;
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
/**
 * Check if a packet from a tracker arrived in its assigned TDMA slot.
 * Uses the tracker's packed dynamic TDMA parameters.
 *
 * @param tracker_id   The tracker's ID (0-15)
 * @param rx_ticks     Receiver time in ticks when packet arrived (EVENT_IRQ fallback)
 */
#if defined(CONFIG_RELAY)
/* SPSC diagnostic ring. The consumer leaves its entry queued while doing
 * 64-bit drift arithmetic with IRQs enabled. On overflow the ISR consumes the
 * oldest entry; the worker detects that by sequence and cannot apply it twice. */
struct tdma_diag_sample {
	uint32_t ticks, config, ping_ticks, generation;
	int32_t bias, ppb;
	uint8_t id, rssi;
	bool ppb_valid;
};
#define TDMA_DIAG_QUEUE_SIZE 64U
static struct tdma_diag_sample tdma_diag_queue[TDMA_DIAG_QUEUE_SIZE];
static uint32_t tdma_diag_head, tdma_diag_tail, tdma_diag_overflow;

static void tdma_process_slot(const struct tdma_diag_sample *sample, uint32_t sequence)
#else
static void tdma_check_slot(uint8_t tracker_id, uint32_t rx_ticks, int8_t rssi)
#endif
{
#if defined(CONFIG_RELAY)
	/* Use the receive-time clock and layout snapshot. */
	uint8_t tracker_id = sample->id;
	uint32_t rx_ticks = sample->ticks;
	int8_t rssi = sample->rssi;
	bool extrap_skip = false;
	uint32_t cfg = sample->config;
#else
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}

	/* Prefer RADIO ISR timestamp (accurate) over EVENT_IRQ timestamp
	 * (delayed 10-25+ ticks by kernel scheduling).  The ISR timestamp is
	 * now available for ALL packets including NoACK data, thanks to the
	 * ESB library patch that always calls on_radio_disabled_rx_dpl(). */
	k_spinlock_key_t timestamp_key = k_spin_lock(&tdma_timestamp_lock);
	bool timestamp_valid = tracker_diags[tracker_id].g_last_isr_rx_valid;
	uint32_t timestamp_ticks = tracker_diags[tracker_id].g_last_isr_rx_ticks;
	k_spin_unlock(&tdma_timestamp_lock, timestamp_key);
	if (timestamp_valid) {
		rx_ticks = timestamp_ticks;
	}

	/* Read dynamic TDMA config (atomic 32-bit load) */
	uint32_t cfg = tracker_slots[tracker_id].tdma_config_packed;
#endif
	uint8_t expected_slot = (cfg >> 24) & 0xFF;
	uint8_t total_slots = (cfg >> 16) & 0xFF;
	uint8_t slot_ticks = (cfg >> 8) & 0xFF;

	/* Skip validation if tracker has no slot assignment or config invalid */
#if defined(CONFIG_RELAY)
	if (expected_slot == 0xFF || total_slots == 0 || slot_ticks == 0) return;
#else
	if (expected_slot == 0xFF || total_slots == 0 || slot_ticks == 0) {
		return;
	}
#endif

	uint32_t frame_ticks = (uint32_t)slot_ticks * total_slots;

	/*
	 * Compensate receiver clock vs tracker clock offset.
	 * Base bias: last clean PING measurement, extrapolated forward
	 * using the measured drift rate (ppb) to keep it accurate
	 * between PING sync intervals.
	 */
#if defined(CONFIG_RELAY)
	int32_t clock_bias = sample->bias;
#else
	int32_t clock_bias = tracker_diags[tracker_id].g_last_ping_rx_time_diff_valid ? tracker_diags[tracker_id].g_last_ping_rx_time_diff_ticks : 0;
#endif

	/* Drift extrapolation: bias changes linearly between PINGs.
	 * Use the long-baseline ppb estimate to correct for drift
	 * since the last PING, keeping the effective bias accurate
	 * throughout the sync interval. */
#if defined(CONFIG_RELAY)
	if (clock_bias && sample->ping_ticks != 0 && sample->ppb_valid) {
		uint32_t elapsed = rx_ticks - sample->ping_ticks;
#else
	if (clock_bias && tracker_diags[tracker_id].g_last_ping_isr_rx_ticks_raw != 0 && tracker_diags[tracker_id].g_bias_ppb_valid) {
		uint32_t elapsed = rx_ticks - tracker_diags[tracker_id].g_last_ping_isr_rx_ticks_raw;
#endif
		if (elapsed <= TDMA_SYNC_EXTRAP_MAX_TICKS) {
#if defined(CONFIG_RELAY)
			int32_t drift_comp = (int32_t)((int64_t)sample->ppb * elapsed / 1000000000LL);
#else
			int32_t drift_comp = (int32_t)((int64_t)tracker_diags[tracker_id].g_bias_ppb * elapsed / 1000000000LL);
#endif
			clock_bias += drift_comp;
		} else {
#if defined(CONFIG_RELAY)
			extrap_skip = true;
#else
			tracker_diags[tracker_id].g_extrap_skip_count++;
#endif
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

#if defined(CONFIG_RELAY)
	unsigned key = irq_lock();
	/* Overflow may have consumed this sample while the worker calculated. */
	if (tdma_diag_tail != sequence) { irq_unlock(key); return; }
	tdma_diag_tail++;
	if (sample->generation != tdma_stats_generation[tracker_id]) { irq_unlock(key); return; }
	tracker_diags[tracker_id].g_extrap_skip_count += extrap_skip;
#endif
	struct tdma_stats *stats = &tracker_diags[tracker_id].stats;
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
#if defined(CONFIG_RELAY)
		stats->phase_ema_q8 = ref_offset * 256;
#else
		stats->phase_ema_q8 = ref_offset << 8;
#endif
		stats->phase_initialized = true;
	} else {
		/* EMA alpha=1/8: converges in ~8 packets (~50ms at 170 TPS) */
#if defined(CONFIG_RELAY)
		stats->phase_ema_q8 += (((ref_offset * 256) - stats->phase_ema_q8) + 4) >> 3;
#else
		stats->phase_ema_q8 += (((ref_offset << 8) - stats->phase_ema_q8) + 4) >> 3;
#endif
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
#if defined(CONFIG_RELAY)
	irq_unlock(key);
#endif
}
#if defined(CONFIG_RELAY)

static void tdma_deferred_process(void)
{
	for (;;) {
		unsigned key = irq_lock();
		uint32_t sequence = tdma_diag_tail;
		if (sequence == tdma_diag_head) { irq_unlock(key); return; }
		struct tdma_diag_sample sample = tdma_diag_queue[sequence % TDMA_DIAG_QUEUE_SIZE];
		irq_unlock(key);
		tdma_process_slot(&sample, sequence);
	}
}

static void tdma_check_slot(uint8_t tracker_id, uint32_t rx_ticks, int8_t rssi)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}

	/* Prefer RADIO ISR timestamp (accurate) over EVENT_IRQ timestamp
	 * (delayed 10-25+ ticks by kernel scheduling).  The ISR timestamp is
	 * now available for ALL packets including NoACK data, thanks to the
	 * ESB library patch that always calls on_radio_disabled_rx_dpl(). */
	k_spinlock_key_t timestamp_key = k_spin_lock(&tdma_timestamp_lock);
	bool timestamp_valid = tracker_diags[tracker_id].g_last_isr_rx_valid;
	uint32_t timestamp_ticks = tracker_diags[tracker_id].g_last_isr_rx_ticks;
	k_spin_unlock(&tdma_timestamp_lock, timestamp_key);
	if (timestamp_valid) {
		rx_ticks = timestamp_ticks;
	}

	unsigned key = irq_lock();
	uint32_t cfg = tracker_slots[tracker_id].tdma_config_packed;
	if ((cfg >> 24) == 0xff || !(cfg & 0xff0000) || !(cfg & 0xff00)) {
		irq_unlock(key);
		return;
	}
	if (tdma_diag_head - tdma_diag_tail == TDMA_DIAG_QUEUE_SIZE) {
		struct tdma_diag_sample oldest = tdma_diag_queue[tdma_diag_tail % TDMA_DIAG_QUEUE_SIZE];
		tdma_process_slot(&oldest, tdma_diag_tail);
		tdma_diag_overflow++;
	}
	tdma_diag_queue[tdma_diag_head % TDMA_DIAG_QUEUE_SIZE] = (struct tdma_diag_sample){
		.id = tracker_id, .rssi = rssi, .ticks = rx_ticks, .config = cfg,
		.generation = tdma_stats_generation[tracker_id],
		.bias = tracker_diags[tracker_id].g_last_ping_rx_time_diff_valid ?
			tracker_diags[tracker_id].g_last_ping_rx_time_diff_ticks : 0,
		.ppb = tracker_diags[tracker_id].g_bias_ppb,
		.ppb_valid = tracker_diags[tracker_id].g_bias_ppb_valid,
		.ping_ticks = tracker_diags[tracker_id].g_last_ping_isr_rx_ticks_raw,
	};
	tdma_diag_head++;
	irq_unlock(key);
}

#endif
#endif

static void esb_stats_thread(void);
/* The loss snapshot is inlined into this thread under LTO. TDMA log packaging
 * alone exceeds the former 512-byte stack; leave room for kernel/IRQ frames. */
#if defined(CONFIG_RELAY)
/* Linked policy + caller use 816 bytes before leaf calls/context saves. */
K_THREAD_DEFINE(esb_stats_thread_id, 1536, esb_stats_thread, NULL, NULL, NULL, ESB_STATS_THREAD_PRIORITY, 0, 0);
#else
K_THREAD_DEFINE(esb_stats_thread_id, 1024, esb_stats_thread, NULL, NULL, NULL, ESB_STATS_THREAD_PRIORITY, 0, 0);
#endif

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
	if (tracker_slots[id].tracker_remote_command == ESB_PONG_FLAG_WOM_CONTROL &&
	    token != 0 && token == tracker_slots[id].wom_token && !tracker_slots[id].wom_complete &&
	    tracker_slots[id].wom_advertised && counter != tracker_slots[id].wom_first_ping_counter) {
		bool success = flag == ESB_PING_FLAG_WOM_RESULT &&
			(tracker_slots[id].wom_minutes == ESB_WOM_ENTER ? value == ESB_WOM_ENTER :
			 tracker_slots[id].wom_minutes == ESB_WOM_TIMEOUT_CLEAR ? value == 0 :
			 (value >= 1 && value <= ESB_WOM_TIMEOUT_MAX_MINUTES &&
			  (tracker_slots[id].wom_minutes == 0 || tracker_slots[id].wom_minutes == value)));
		bool failure = flag == ESB_PING_FLAG_WOM_ERROR && value != 0;
		if (success || failure) {
			tracker_slots[id].wom_minutes = success ? value : 0;
			tracker_slots[id].wom_error = failure ? value : 0;
			tracker_slots[id].wom_complete = true;
			tracker_slots[id].tracker_remote_command = ESB_PONG_FLAG_NORMAL;
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


/* Call with interrupts locked: RX and the monitor share this window. */
static void packet_stats_refresh(struct packet_stats *stats, uint64_t now)
{
	if (!stats->last_tps_time) {
		stats->last_tps_time = now;
	} else if (now >= stats->last_tps_time &&
		   now - stats->last_tps_time >= TPS_CALCULATION_INTERVAL_MS) {
		stats->current_tps = stats->packets_in_last_second;
		stats->packets_in_last_second = 0;
		stats->last_tps_time = now;
	}
}

/* TPS and liveness; every accepted pose counts, whichever path delivered it. */
static void packet_stats_count(struct packet_stats *stats)
{
	unsigned key = irq_lock();
	uint64_t now = k_uptime_get();
	packet_stats_refresh(stats, now);
	stats->packets_in_last_second++;
	stats->last_packet_time = now;
	irq_unlock(key);
}

static void packet_stats_monitor(struct packet_stats *stats)
{
	unsigned key = irq_lock();
	/* Sample after taking ownership. A pre-learning timestamp can precede an
	 * RX rollover and unsigned subtraction would turn it into a huge age. */
	if (stats->last_tps_time) packet_stats_refresh(stats, k_uptime_get());
	irq_unlock(key);
}

static int check_packet_sequence(uint8_t tracker_id, uint8_t received_seq)
{
	if (tracker_id >= MAX_TRACKERS) {
		return 2;
	}

	struct packet_stats *stats = &tracker_slots[tracker_id].stats;
	packet_stats_count(stats);

	// First packet, accept directly
	if (!tracker_slots[tracker_id].sequence_initialized) {
		// Update received packet count
		stats->total_received++;
		tracker_slots[tracker_id].last_packet_sequence = received_seq;
		tracker_slots[tracker_id].sequence_initialized = true;
		stats->normal_packets++;
		stats->last_sequence = received_seq;
		stats->first_packet = false;
		stats->status_received++;
		decision_accept(tracker_id, 0);
		LOG_DBG("First packet: tracker=%d, seq=%d", tracker_id, received_seq);
		return 0;
	}

	uint8_t last_seq = tracker_slots[tracker_id].last_packet_sequence;
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
		tracker_slots[tracker_id].last_packet_sequence = received_seq;
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
		tracker_slots[tracker_id].last_packet_sequence = received_seq;
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
		tracker_slots[tracker_id].last_packet_sequence = received_seq;
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
	struct packet_stats *stats = &tracker_slots[tracker_id].stats;

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
		if (tracker_slots[i].stats.total_received > 0 || tracker_slots[i].stats.duplicate_packets > 0) {
			print_tracker_stats(i);
		}
	}
}

#if defined(CONFIG_RELAY)
/* Only the statistics thread calls this. Policy may use a slightly older
 * observation, but must never publish across a membership/session change. */
static void relay_learn_update(uint32_t now)
{
	static struct relay_coord snapshot;
	unsigned key = irq_lock();
	uint32_t session = relay_session;
	uint8_t epoch = relay_epoch;
	uint16_t members = (uint16_t)tdma_active_mask;
	irq_unlock(key);
	/* Recheck live timestamps under a per-cell lock. A report newer than
	 * this pass's timestamp is fresh, including across clock rollover. */
	for (unsigned h = 0; h < RC_MAX; h++) {
		for (unsigned c = 0; c < RC_MAX; c++) {
			key = irq_lock();
			uint32_t start = k_cycle_get_32();
			relay_coord_expire_peer(&relay_coord, h, c, now);
			relay_learn_lock_max = MAX(relay_learn_lock_max, k_cycle_get_32() - start);
			irq_unlock(key);
		}
	}
	for (unsigned i = 0; i < RC_MAX; i++) {
		key = irq_lock();
		uint32_t start = k_cycle_get_32();
		snapshot.t[i] = relay_coord.t[i];
		memcpy(snapshot.peer[i], relay_coord.peer[i], sizeof(snapshot.peer[i]));
		memcpy(snapshot.choice[i], relay_coord.choice[i], sizeof(snapshot.choice[i]));
		relay_learn_lock_max = MAX(relay_learn_lock_max, k_cycle_get_32() - start);
		irq_unlock(key);
	}
	snapshot.learn_members = members;
	relay_coord_learn_tick(&snapshot, now);
	key = irq_lock();
	uint32_t start = k_cycle_get_32();
	if (session == relay_session && epoch == relay_epoch && members == tdma_active_mask) {
		memcpy(relay_coord.choice, snapshot.choice, sizeof(relay_coord.choice));
		relay_coord.learn_members = members;
	}
	relay_learn_lock_max = MAX(relay_learn_lock_max, k_cycle_get_32() - start);
	irq_unlock(key);
}

#endif
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

#if defined(CONFIG_RELAY)
		/* Relay decisions every window, before the layout pass below picks
		 * up any change in forwarding slots. OTA runs on direct links only,
		 * so the coordinator pauses (and releases every role) meanwhile. */
		{
			bool relay_enabled = relay_user_enabled && !esb_ota_relay_is_active();
			unsigned int relay_key = irq_lock();
			IRQ_PROBE_BEGIN(tick);
			if (relay_enabled != relay_coord.enabled) {
				relay_coord_set_enabled(&relay_coord, relay_enabled);
			}
			(void)relay_coord_tick(&relay_coord, (uint16_t)tdma_active_mask, k_uptime_get_32());
			IRQ_PROBE_END(tick);
			irq_unlock(relay_key);
		}

		/* Proof can invalidate both incumbents in this 500 ms window.
		 * Refill choices now, not at the next one-second TPS report. */
		relay_learn_update(k_uptime_get_32());

#endif
		for (int i = 0; i < MAX_TRACKERS; i++) {
			packet_stats_monitor(&tracker_slots[i].stats);
		}

		// Check if detailed stats should be auto-disabled
		bool display_expired = false;
		k_spinlock_key_t display_key = k_spin_lock(&stats_display_lock);
		if (stats_detailed_enabled && stats_detailed_end_time > 0 && now >= stats_detailed_end_time) {
			stats_detailed_enabled = false;
			stats_detailed_end_time = 0;
			display_expired = true;
		}
		k_spin_unlock(&stats_display_lock, display_key);
		if (display_expired) {
			LOG_INF("Detailed stats auto-disabled");
		}

		// Print total TPS every second (always)
		if (now - last_tps_print_time >= TPS_CALCULATION_INTERVAL_MS) {
			uint32_t total_tps = 0;
			for (int i = 0; i < MAX_TRACKERS; i++) {
				total_tps += tracker_slots[i].stats.current_tps;
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
		if (esb_get_stats_detailed_enabled()) {
			static int64_t last_detailed_log_time = 0;
			if (last_detailed_log_time == 0) {
				last_detailed_log_time = now;
			}

			if (now - last_detailed_log_time >= STATS_PRINT_INTERVAL_MS) {
				bool has_data = false;
				for (int i = 0; i < MAX_TRACKERS; i++) {
					if (tracker_slots[i].stats.total_received > 0) {
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
 * Only the target tracker is tracked; resets exclude the RADIO IRQ.
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

static uint16_t raw_arq_expected_seq;
static bool raw_arq_seq_initialized;
static uint16_t raw_arq_gap_queue[RAW_ARQ_MAX_GAPS];
static uint8_t raw_arq_gap_count;
static uint32_t raw_arq_gaps_detected;
static uint32_t raw_arq_retransmits_received;

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

#if defined(CONFIG_RELAY)
/* ---------------------------------------------------------------------------
 * Peer relay (relay_protocol.h, relay_coord.h). The coordinator state is
 * touched from the RADIO ISR (ACK handler), the ESB event ISR and the stats
 * thread; every access holds irq_lock.
 * ------------------------------------------------------------------------- */
/* Test hook: deterministic loss of a tracker's direct uplink (pose frames,
 * bundles and PINGs), as if obstructed, while peers still hear it. */
#ifdef CONFIG_RELAY_FRAME_LEDGER
/* Background reservation/validated-packet counters, never in freeze encoding. */
static uint32_t relay_r_frames[6];
static uint32_t relay_direct_expected[16], relay_direct_missing[16];
static uint32_t relay_pool_reserved[RELAY_V3_REPAIR_MAX][4]; /* repair/report/maintenance/idle */
static uint32_t relay_pool_received[RELAY_V3_REPAIR_MAX];
#endif
static uint8_t relay_block_id = 0xFF;
static uint8_t relay_block_percent;
static uint16_t relay_block_mask;
static uint32_t relay_block_deadline;
static struct {
	uint32_t bundles;
	uint32_t bundle_invalid;
	uint32_t records;
	uint32_t records_accepted;
	uint32_t direct_accepted;
	uint32_t duplicate_pose;
	uint32_t blocked;
	/* Cumulative repair funnel and delivery age, sampled by the relay console. */
	struct {
		uint32_t seen, invalid, phase, slot, record;
		uint32_t received[RC_MAX], duplicate[RC_MAX], obsolete[RC_MAX], age[RC_MAX][RELAY_CACHE_MAX_FRAMES];
	} repair;
} relay_rx_stats;
/* Latest private diagnostic frame per tracker (RELAY_DIAG_*), console only. */
static struct {
	uint8_t data[RELAY_DIAG_LEN];
	uint32_t rx_ms;
	bool valid;
} relay_diag[MAX_TRACKERS];

static void esb_receive_relay_diag(const struct esb_payload *payload)
{
	uint8_t id = payload->data[1];
	if (payload->length != RELAY_DIAG_LEN || id >= stored_trackers || id >= MAX_TRACKERS
	    || payload->pipe != 1 + (id % 7)) {
		return;
	}
	memcpy(relay_diag[id].data, payload->data, RELAY_DIAG_LEN);
	relay_diag[id].rx_ms = k_uptime_get_32();
	relay_diag[id].valid = true;
}

static int relay_coord_init_once(void)
{
#if defined(CONFIG_RELAY_CYCLE_DIAGNOSTICS) || defined(CONFIG_RELAY_IRQ_DIAGNOSTICS)
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
	relay_coord_init(&relay_coord);
	return 0;
}
SYS_INIT(relay_coord_init_once, APPLICATION, 0);

static int composite_body_length(uint8_t type);

/* Conservative fast validation. Unusual/event composites stay on the full
 * event path; only a completely known record list may suppress B early. */
static bool relay_early_receipt_valid(const uint8_t *data, uint8_t length)
{
	if (data[0] != ESB_COMPOSITE_TYPE) return length == 17 && data[0] < 0x10;
	if (length < 5 || !data[2]) return false;
	unsigned pos = 3;
	for (unsigned i = 0; i < data[2]; i++) {
		if (pos >= length - 1U) return false;
		uint8_t type = data[pos++];
		if (type > 5) return false;
		int n = composite_body_length(type);
		if (n < 0 || pos + n > length - 1U) return false;
		pos += n;
	}
	return pos == length - 1U;
}

/* Multi-child injection expires without relying on the console client. */
static bool relay_multi_blocked(uint8_t id)
{
	return id < RC_MAX && (relay_block_mask & BIT(id)) &&
		(int32_t)(relay_block_deadline - k_uptime_get_32()) > 0;
}

static bool relay_block_drop(uint8_t id, uint8_t salt)
{
	bool multi = relay_multi_blocked(id);
	if (!multi && (id != relay_block_id || relay_block_percent == 0)) {
		return false;
	}
	uint32_t h = ((uint32_t)salt * 2654435761U) ^ ((uint32_t)id * 40503U);
	h ^= h >> 15;
	h *= 2246822519U;
	h ^= h >> 13;
	bool drop = multi || h % 100U < relay_block_percent;
	if (drop) {
		relay_rx_stats.blocked++;
	}
	return drop;
}

/* Sequence check plus relay accounting: gaps revealed, path, RSSI. */
static int relay_sequence_check(uint8_t id, uint8_t seq, bool relayed, uint8_t rssi)
{
	if (relay_repair_delivery) {
		packet_stats_count(&tracker_slots[id].stats);
		return RC_SEQ_NORMAL;
	}
	if (relay_profile && !relayed && !relay_direct_pose) {
		return RC_SEQ_OLD;
	}
	if (relay_profile && relay_direct_pose && !relayed) {
		uint32_t frame = relay_frame_near(relay_frame_anchor, relay_start_anchor, relay_direct_start);
		int slot = relay_member_slot((uint16_t)tdma_active_mask, id);
		uint32_t phase = relay_direct_start - relay_frame_start(relay_origin, frame);
		if (slot < 0 || phase / RELAY_V3_SLOT_TICKS != (uint32_t)slot) {
			return RC_SEQ_OLD;
		}

		relay_coord_note_frame(&relay_coord, id, RC_DIRECT, RC_SEQ_NORMAL,
				       relay_coord_skipped(&relay_coord, id, frame), rssi,
				       k_uptime_get_32());
		if (!relay_coord_publish(&relay_coord, id, frame)) {
			return RC_SEQ_DUP;
		}
		packet_stats_count(&tracker_slots[id].stats);
#ifdef CONFIG_RELAY_FRAME_LEDGER
		ledger_note(relay_session, frame, id, LEDGER_ACCEPT, -(int16_t)rssi, 255, 0);
#endif
		relay_rx_stats.direct_accepted++;
		return RC_SEQ_NORMAL;
	}
	uint32_t gaps_before = tracker_slots[id].stats.total_gaps;
	int result = check_packet_sequence(id, seq);
	uint32_t gaps = tracker_slots[id].stats.total_gaps - gaps_before;
	unsigned int key = irq_lock();
	relay_coord_note_frame(&relay_coord, id, relayed ? RC_RELAYED : RC_DIRECT, result,
		(uint8_t)MIN(gaps, 255U), rssi, k_uptime_get_32());
	relay_rx_stats.direct_accepted += !relayed && result != 2 && result != 4;
	relay_rx_stats.duplicate_pose += result == 4;
	if (relayed && result != 2 && result != 4) {
		relay_rx_stats.records_accepted++;
	}
	irq_unlock(key);
	return result;
}

/* The info frame's reserved byte carries relay capability; HID gets the
 * stock zero so an unmodified server sees an unchanged packet. */
static void relay_take_caps(uint8_t id, uint8_t *info16)
{
	uint8_t caps = info16[RELAY_INFO_CAPS_INDEX];
	info16[RELAY_INFO_CAPS_INDEX] = 0;
	unsigned int key = irq_lock();
	relay_coord_note_caps(&relay_coord, id, caps);
	irq_unlock(key);
}

/* ---------------------------------------------------------------------------
 * ACK handler — runs in radio ISR context (~130 µs budget).
 * Builds the ACK payload for the *current* packet so the response is
 * returned in the same transaction rather than the next one.
 *
 * Samples state published by event_handler and threads without blocking or logging.
 * -------------------------------------------------------------------------*/
static void relay_rx_metadata(struct esb_payload *payload)
{
	payload->rx_context = relay_layout_generation;
	if (payload->length <= 1 || payload->data[1] >= MAX_TRACKERS) return;
	uint8_t id = payload->data[1];
	int slot = relay_member_slot((uint16_t)tdma_active_mask, id);
	payload->rx_pose = relay_profile && payload->pipe == 1U + id % 7U && slot >= 0
		&& payload->length <= relay_direct_payload_max(relay_profile, (uint8_t)slot)
		&& relay_frame_is_pose(payload->data, payload->length, id);
}

#else
/* ---------------------------------------------------------------------------
 * ACK handler — runs in radio ISR context (~130 µs budget).
 * Builds the ACK payload for the *current* packet so the response is
 * returned in the same transaction rather than the next one.
 *
 * Samples state published by event_handler and threads without blocking or logging.
 * -------------------------------------------------------------------------*/
#endif
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

		if (atomic_get(&pairing_new_devices_blocked) && known_id < 0) {
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

#if defined(CONFIG_RELAY)
	if (relay_profile && data_length > 1 && pdu_data[1] < MAX_TRACKERS && pipe_id > 0) {
		uint8_t id = pdu_data[1];
		/* Judge the slot by where the frame started: a 60-byte composite
		 * ends close enough to the slot edge that its END can land in the
		 * next slot. */
		uint32_t start_ticks = (uint32_t)k_uptime_ticks() - RELAY_V3_AIR_TICKS(data_length);
		int slot = relay_member_slot((uint16_t)tdma_active_mask, id);
		bool pose = pipe_id == 1U + id % 7U && slot >= 0
			&& data_length <= relay_direct_payload_max(relay_profile, (uint8_t)slot)
			&& relay_frame_is_pose(pdu_data, data_length, id);
#ifdef CONFIG_RELAY_FRAME_LEDGER
		/* Observe RF arrival without publishing a forced-dropped frame. The
		 * phase is an ISR-time estimate of its preamble, not a receipt bit. */
		if (pose && ledger_child_enabled(id)) {
			uint32_t frame = relay_frame_near(relay_frame_anchor, relay_start_anchor, start_ticks);
			uint32_t slot_start = relay_frame_start(relay_origin, frame) + slot * RELAY_V3_SLOT_TICKS;
			ledger_note(relay_session, frame, id, LEDGER_DIRECT_SEEN,
				(int32_t)(start_ticks - slot_start), 255, 0);
		}
#endif
		/* B can preempt the deferred RX event after D10 has reached the
		 * radio. Freeze RF receipt here; publication keeps its own watermark.
		 * A configured drop must still be announced as missing. */
		if (pose && id < stored_trackers
		    && !relay_multi_blocked(id) && (id != relay_block_id || !relay_block_percent)
		    && relay_early_receipt_valid(pdu_data, data_length)) {
			uint32_t frame = relay_frame_near(relay_frame_anchor, relay_start_anchor, start_ticks);
			uint32_t phase = start_ticks - relay_frame_start(relay_origin, frame);
			if (phase / RELAY_V3_SLOT_TICKS == (uint32_t)slot) {
				relay_coord_receive(&relay_coord, id, frame);
			}
		}
	}
#endif
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	/* ---- Capture ISR timestamp for any tracker packet (non-pairing) ---- */
	if (data_length > 1 && pipe_id > 0) {
		uint8_t tid = pdu_data[1];
		if (tid < MAX_TRACKERS) {
			uint32_t timestamp_ticks = k_uptime_ticks();
			k_spinlock_key_t timestamp_key = k_spin_lock(&tdma_timestamp_lock);
			tracker_diags[tid].g_last_isr_rx_ticks = timestamp_ticks;
			tracker_diags[tid].g_last_isr_rx_valid = true;
			k_spin_unlock(&tdma_timestamp_lock, timestamp_key);
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
#if defined(CONFIG_RELAY)
		/* Test hook: an obstructed uplink loses the PING, so no PONG. */
		if (relay_block_drop(tracker_id, counter ^ 0xA5U)) {
			*suppress_ack = true;
			return;
		}
#endif
		bool channel_confirm = (pdu_data[7] & ESB_PING_FLAG_CHANNEL_CONFIRM) != 0;
		uint32_t rx_ticks = k_uptime_ticks();
		uint8_t cmd = ESB_PONG_FLAG_CHANNEL_CONFIRM;
		bool ota_aborting = false;
		if (!channel_confirm) {
			cmd = tracker_slots[tracker_id].tracker_remote_command;
			/* Keep an active metadata request on duplicate PINGs until its echo
			 * arrives; channel probes must leave both pending and active tuples alone. */
			unsigned int metadata_key = irq_lock();
			if (cmd == ESB_PONG_FLAG_NORMAL && tracker_slots[tracker_id].metadata_active_mask != 0) {
				cmd = ESB_PONG_FLAG_METADATA_REQUEST;
			} else if (cmd == ESB_PONG_FLAG_NORMAL && tracker_slots[tracker_id].metadata_pending_mask != 0) {
				tracker_slots[tracker_id].metadata_active_mask = tracker_slots[tracker_id].metadata_pending_mask;
				tracker_slots[tracker_id].metadata_active_chunk = tracker_slots[tracker_id].metadata_pending_chunk;
				tracker_slots[tracker_id].metadata_active_token = tracker_slots[tracker_id].metadata_pending_token;
				tracker_slots[tracker_id].metadata_pending_mask = 0;
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
			if (cmd == ESB_PONG_FLAG_WOM_CONTROL && !tracker_slots[tracker_id].wom_advertised &&
			    (pdu_data[7] & 0x7f) != ESB_PONG_FLAG_NORMAL) {
				cmd = ESB_PONG_FLAG_NORMAL;
			}
		}
#if defined(CONFIG_TDMA_DIAGNOSTICS)
		if (pipe_id == 1U + (tracker_id % 7)) {
			k_spinlock_key_t timestamp_key = k_spin_lock(&tdma_timestamp_lock);
			tracker_diags[tracker_id].g_ping_isr_rx_ticks = rx_ticks;
			tracker_diags[tracker_id].g_ping_isr_rx_ticks_valid = true;
			k_spin_unlock(&tdma_timestamp_lock, timestamp_key);
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
			int16_t s0 = tracker_slots[tracker_id].pending_sens_data[0];
			int16_t s1 = tracker_slots[tracker_id].pending_sens_data[1];
			int16_t s2 = tracker_slots[tracker_id].pending_sens_data[2];
			ack_payload->data[3] = (s0 >> 8) & 0xFF;
			ack_payload->data[4] = (s0) & 0xFF;
			ack_payload->data[5] = (s1 >> 8) & 0xFF;
			ack_payload->data[6] = (s1) & 0xFF;
			ack_payload->data[8] = (s2 >> 8) & 0xFF;
			ack_payload->data[9] = (s2) & 0xFF;
			ack_payload->data[10] = 0;
			ack_payload->data[11] = 0;
		} else if (cmd == ESB_PONG_FLAG_SENS_AUTO) {
			uint8_t axis = tracker_slots[tracker_id].pending_sens_auto_axis;
			uint16_t rev = tracker_slots[tracker_id].pending_sens_auto_revolutions;
			ack_payload->data[3] = axis;
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
				uint16_t minutes = tracker_slots[tracker_id].wom_minutes;
				uint16_t token = tracker_slots[tracker_id].wom_token;
				ack_payload->data[8] = minutes >> 8;
				ack_payload->data[9] = minutes & 0xff;
				ack_payload->data[10] = token >> 8;
				ack_payload->data[11] = token & 0xff;
			} else if (cmd == ESB_PONG_FLAG_TEST_MODE_ON) {
				uint16_t tps = tracker_slots[tracker_id].tracker_test_on_tps;
				ack_payload->data[8] = (tps >> 8) & 0xFF;
				ack_payload->data[9] = tps & 0xFF;
				ack_payload->data[10] = 0;
			} else if (cmd == ESB_PONG_FLAG_METADATA_REQUEST) {
				uint8_t mask = tracker_slots[tracker_id].metadata_active_mask;
				uint8_t chunk = tracker_slots[tracker_id].metadata_active_chunk;
				uint16_t token = tracker_slots[tracker_id].metadata_active_token;
				ack_payload->data[8] = mask;
				ack_payload->data[9] = chunk;
				ack_payload->data[10] = (token >> 8) & 0xFF;
				ack_payload->data[11] = token & 0xFF;
			} else if (cmd == ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON) {
				ack_payload->data[8] = tracker_slots[tracker_id].pending_cmd_arg;
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
#if defined(CONFIG_RELAY)
				/* Preserve the legacy TDMA tuple byte for byte: the active
				 * layout, or a staged one whose activation tick has passed. */
				uint32_t cfg = tdma_advertised_config(tracker_id, rx_ticks);
#else
				/* Preserve the legacy TDMA tuple byte for byte. */
				uint32_t cfg = tracker_slots[tracker_id].tdma_config_packed;
#endif
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
#if defined(CONFIG_RELAY)

		/* Relay-capable trackers get the trailer: head assignment, forwarding
		 * slot, member mask and relayed mask, all for the layout whose epoch
		 * the legacy tuple of a NORMAL PONG carries. Others keep 13 bytes. */
		if (relay_coord_is_capable(&relay_coord, tracker_id)) {
			struct relay_pong_trailer trailer;
			/* Judged at the PONG timestamp, like the tuple. */
			bool due = tdma_staged_due_locked(rx_ticks);
			uint8_t epoch = (uint8_t)(tdma_advertised_config(tracker_id, rx_ticks) & 0xFFU);
			uint16_t members = (uint16_t)(due ? tdma_staged.p.new_mask : tdma_active_mask);
			relay_coord_trailer(&relay_coord, tracker_id, epoch, members, &trailer);
			trailer.pending = false;
			/* Announce a staged layout on NORMAL PONGs, whose bytes 3..6
			 * carry the receiver timestamp the activation is relative to. */
			if (tdma_staged.valid && !due && ack_payload->data[7] == ESB_PONG_FLAG_NORMAL) {
				uint32_t lead = tdma_staged.activation_ticks - rx_ticks;
				trailer.pending = (int32_t)lead > 0;
				trailer.pending_epoch = tdma_staged.p.epoch;
				trailer.pending_total
					= tdma_staged.p.profile
						? relay_geometry_code(tdma_staged.p.active_count, tdma_staged.p.settings.max_repairs)
						: tdma_staged.p.active_count + tdma_staged.p.forward_count;
				trailer.pending_slot_ticks = tdma_staged.p.slot_ticks;
				trailer.pending_member_mask = (uint16_t)tdma_staged.p.new_mask;
				trailer.activation_units = (uint16_t)MIN(
					(lead + RELAY_ACTIVATION_UNIT_TICKS - 1U) / RELAY_ACTIVATION_UNIT_TICKS, UINT16_MAX);
			} else if (relay_profile && !tdma_staged.valid && ack_payload->data[7] == ESB_PONG_FLAG_NORMAL) {
				/* The legacy tuple cannot say profile 1, so a tracker joining a
				 * running profile learns it as a layout activating on the next
				 * actual frame boundary; B then anchors its cap and future R. */
				uint32_t frame = relay_frame_near(relay_frame_anchor, relay_start_anchor, rx_ticks);
				uint32_t lead = relay_frame_start(relay_origin, frame + 1U) - rx_ticks;
				trailer.pending = true;
				trailer.pending_epoch = epoch;
				trailer.pending_total
					= relay_geometry_code(relay_mask_count(members), relay_applied_settings.max_repairs);
				trailer.pending_slot_ticks = RELAY_V3_SLOT_TICKS;
				trailer.pending_member_mask = members;
				trailer.activation_units = (uint16_t)((lead + RELAY_ACTIVATION_UNIT_TICKS - 1U)
					/ RELAY_ACTIVATION_UNIT_TICKS);
			}
			ack_payload->length = (uint8_t)(ESB_PONG_LEN
				+ relay_trailer_encode(&ack_payload->data[ESB_PONG_LEN], &trailer));
			if (trailer.children_mask) {
				relay_coord_note_trailer_sent(&relay_coord, tracker_id, trailer.children_mask,
					k_uptime_get_32());
			}
		}
#endif

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
#if defined(CONFIG_RELAY)
		if (*has_ack_payload &&
		    (ack_payload->length == ESB_PONG_LEN || relay_pong_is_extended(ack_payload->length)) &&
#else
		if (*has_ack_payload && ack_payload->length == ESB_PONG_LEN &&
#endif
		    ack_payload->data[0] == ESB_PONG_TYPE &&
		    ack_payload->data[7] == ESB_PONG_FLAG_WOM_CONTROL &&
		    tracker_slots[tracker_id].tracker_remote_command == ESB_PONG_FLAG_WOM_CONTROL &&
		    tracker_slots[tracker_id].wom_token != 0 &&
		    (((uint16_t)ack_payload->data[8] << 8) | ack_payload->data[9]) == tracker_slots[tracker_id].wom_minutes &&
		    (((uint16_t)ack_payload->data[10] << 8) | ack_payload->data[11]) == tracker_slots[tracker_id].wom_token &&
		    !tracker_slots[tracker_id].wom_advertised) {
			tracker_slots[tracker_id].wom_first_ping_counter = counter;
			tracker_slots[tracker_id].wom_advertised = true;
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
	struct tdma_stats *stats = &tracker_diags[tracker_id].stats;

	/* Use RADIO ISR timestamp to avoid EVENT_IRQ scheduling jitter. */
	k_spinlock_key_t timestamp_key = k_spin_lock(&tdma_timestamp_lock);
	bool timestamp_valid = tracker_diags[tracker_id].g_ping_isr_rx_ticks_valid;
	uint32_t timestamp_ticks = tracker_diags[tracker_id].g_ping_isr_rx_ticks;
	k_spin_unlock(&tdma_timestamp_lock, timestamp_key);
	uint32_t isr_rx_ticks = timestamp_valid ? timestamp_ticks : current_rx_ticks;
	int32_t rx_time_diff_ticks = (int32_t)(isr_rx_ticks - expected_rx_ticks);

	/* The tracker estimates TX start, but the ISR samples after the complete
	 * PING. Remove the 13-byte 2 Mbps on-air time. Retransmissions have an
	 * additional retry interval and are excluded by the clean-PING gate. */
	int32_t prev_bias = tracker_diags[tracker_id].g_last_ping_rx_time_diff_valid
		? tracker_diags[tracker_id].g_last_ping_rx_time_diff_ticks : 0;
	#define PING_TX_START_TO_RX_TICKS 5

	bool clean_ping;
	if (!tracker_diags[tracker_id].g_last_ping_rx_time_diff_valid) {
		clean_ping = true;
	} else if (rx_time_diff_ticks <= tracker_diags[tracker_id].g_last_ping_rx_time_diff_ticks
					     + PING_TX_START_TO_RX_TICKS + 2) {
		tracker_diags[tracker_id].g_ping_dirty_streak = 0;
		clean_ping = true;
	} else {
		tracker_diags[tracker_id].g_ping_reject_count++;
		if (++tracker_diags[tracker_id].g_ping_dirty_streak >= PING_CLEAN_REACQUIRE_STREAK) {
			tracker_diags[tracker_id].g_ping_dirty_streak = 0;
			tracker_diags[tracker_id].g_bias_ppb_valid = 0;
			prev_bias = 0;
			clean_ping = true;
		} else {
			clean_ping = false;
		}
	}
	if (clean_ping) {
		tracker_diags[tracker_id].g_last_ping_rx_time_diff_ticks = rx_time_diff_ticks - PING_TX_START_TO_RX_TICKS;
		tracker_diags[tracker_id].g_last_ping_rx_time_diff_valid = true;
	}

	if (clean_ping && prev_bias != 0) {
		if (!tracker_diags[tracker_id].g_bias_ppb_valid) {
			tracker_diags[tracker_id].g_bias_ref_offset = tracker_diags[tracker_id].g_last_ping_rx_time_diff_ticks;
			tracker_diags[tracker_id].g_bias_ref_ticks = isr_rx_ticks;
			tracker_diags[tracker_id].g_bias_ppb = 0;
			tracker_diags[tracker_id].g_bias_ppb_valid = 1;
		} else {
			uint32_t elapsed = isr_rx_ticks - tracker_diags[tracker_id].g_bias_ref_ticks;
			if (elapsed >= 32768u) {
				int32_t new_bias = tracker_diags[tracker_id].g_last_ping_rx_time_diff_ticks;
				int64_t total_drift = (int64_t)(new_bias - tracker_diags[tracker_id].g_bias_ref_offset);
				int32_t raw_ppb = (int32_t)(total_drift * 1000000000LL / elapsed);
				if (tracker_diags[tracker_id].g_bias_ppb_valid > 1) {
					tracker_diags[tracker_id].g_bias_ppb += (raw_ppb - tracker_diags[tracker_id].g_bias_ppb) / 4;
				} else {
					tracker_diags[tracker_id].g_bias_ppb = raw_ppb;
					tracker_diags[tracker_id].g_bias_ppb_valid = 2;
				}
			}
			if (elapsed > 60u * 32768u) {
				tracker_diags[tracker_id].g_bias_ref_offset = tracker_diags[tracker_id].g_last_ping_rx_time_diff_ticks;
				tracker_diags[tracker_id].g_bias_ref_ticks = isr_rx_ticks;
			}
		}
	}
	if (clean_ping) {
		tracker_diags[tracker_id].g_last_ping_isr_rx_ticks_raw = isr_rx_ticks;
	}

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
		tracker_slots[tracker_id].pipe_mismatch_count++;
		LOG_DBG(
			"PING pipe mismatch (x%u): id=%u, expected "
			"pipe=%u got pipe=%u",
			tracker_slots[tracker_id].pipe_mismatch_count,
			tracker_id,
			1 + (tracker_id % 7),
			payload->pipe
		);
	}

	// check crc for PING
	uint8_t crc_calc = crc8_ccitt(0x07, payload->data, ESB_PING_LEN - 1);
	if (payload->data[ESB_PING_LEN - 1] != crc_calc) {
		// CRC error - only log warning if consecutive errors occur
		tracker_slots[tracker_id].crc_error_count++;

		LOG_DBG(
			"PING CRC mismatch (x%u): id=%u expected %02X got %02X",
			tracker_slots[tracker_id].crc_error_count,
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
	if (!tracker_slots[tracker_id].ping_counter_initialized) {
		tracker_slots[tracker_id].ping_counter_initialized = true;
		tracker_slots[tracker_id].last_ping_counter = counter;
		tracker_slots[tracker_id].last_ping_time = k_uptime_get();
		tracker_slots[tracker_id].last_pong_queued_counter = 0xFF; // Mark as not queued
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
		if (tracker_slots[tracker_id].last_ping_time > 0
			&& (current_time - tracker_slots[tracker_id].last_ping_time) > PING_TIMEOUT_MS) {
			LOG_WRN(
				"PING timeout (%llu ms), resetting tracker %u counter tracking",
				current_time - tracker_slots[tracker_id].last_ping_time,
				tracker_id
			);
			tracker_slots[tracker_id].last_ping_counter = counter;
			tracker_slots[tracker_id].last_ping_time = current_time;
			tracker_slots[tracker_id].last_pong_queued_counter = 0xFF;
			if (atomic_get(&test_all_state_valid)) {
				test_all_invalidate_tracker(tracker_id, (int64_t)current_time);
			}
			// Continue processing this PING
		} else {
			// Calculate difference
			int counter_diff = (int)counter - (int)tracker_slots[tracker_id].last_ping_counter;
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
						tracker_slots[tracker_id].last_ping_counter,
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
						tracker_slots[tracker_id].last_ping_counter,
						counter,
						backward_amount
					);
				}
			} else {
				// counter_diff is between 101-128 - large packet loss
				is_large_gap = true;
			}

			// Update timestamp
			tracker_slots[tracker_id].last_ping_time = current_time;
		}

		// Handle various cases
		if (is_tracker_restart) {
			// Tracker restarted, reset counter tracking
			tracker_slots[tracker_id].last_ping_counter = counter;
			// Reset PONG queue tracking
			tracker_slots[tracker_id].last_pong_queued_counter = 0xFF;
			/* The first data sequence after reboot is unrelated to
			 * the old stream; retain totals but re-anchor admission. */
			tracker_slots[tracker_id].sequence_initialized = false;
			decision_ready_mask &= ~BIT(tracker_id);
			decision_fence_mask |= BIT(tracker_id);
			tracker_slots[tracker_id].stats.restart_events++;
			if (atomic_get(&test_all_state_valid)) {
				test_all_invalidate_tracker(tracker_id, (int64_t)current_time);
			}
			// Continue processing this PING, send PONG
		} else if (is_duplicate) {
			// Same counter as last time - likely a retransmit
			LOG_DBG("Duplicate PING detected: id=%u ctr=%u (retransmit or retry)", tracker_id, counter);

			// Check if we already queued a PONG for this counter
			if (counter == tracker_slots[tracker_id].last_pong_queued_counter) {
				LOG_DBG("PONG already queued for ctr=%u, skipping duplicate queue", counter);
				// Don't queue again, tracker will get PONG on next packet
				return;
			}
		} else if (is_out_of_order) {
			// Out-of-order PING - this is an old packet that arrived late
			// Don't process it to avoid sending stale PONG
			int backward_amount = 256 - ((int)counter - (int)tracker_slots[tracker_id].last_ping_counter + 256) % 256;
			LOG_WRN(
				"Out-of-order PING: id=%u ctr=%u (expected >%u, -%d backward), SKIPPING",
				tracker_id,
				counter,
				tracker_slots[tracker_id].last_ping_counter,
				backward_amount
			);
			// Don't update last_ping_counter, don't queue PONG
			return;
		}
		if (is_large_gap) {
			// Large gap detected - possible packet loss
			int counter_diff = (int)counter - (int)tracker_slots[tracker_id].last_ping_counter;
			if (counter_diff < 0) {
				counter_diff += 256;
			}
			if (counter_diff > 10) {
				LOG_WRN(
					"Large PING counter gap: id=%u last=%u new=%u (gap=%d)",
					tracker_id,
					tracker_slots[tracker_id].last_ping_counter,
					counter,
					counter_diff
				);
			}
		}

		// Update last seen counter (only if not out-of-order and not skipped)
		if (!is_out_of_order) {
			tracker_slots[tracker_id].last_ping_counter = counter;
		}
	} // End of else branch for ping_counter_initialized

	remote_wom_receive(tracker_id, ping_ack_flag, &payload->data[8], counter);
	if (ping_ack_flag == ESB_PONG_FLAG_METADATA_REQUEST) {
		uint16_t echoed_token = ((uint16_t)payload->data[10] << 8) | payload->data[11];
		bool metadata_matches = tracker_slots[tracker_id].metadata_active_mask != 0 &&
			payload->data[8] == tracker_slots[tracker_id].metadata_active_mask &&
			payload->data[9] == tracker_slots[tracker_id].metadata_active_chunk &&
			echoed_token == tracker_slots[tracker_id].metadata_active_token;
		if (metadata_matches) {
			tracker_slots[tracker_id].metadata_active_mask = 0;
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
			|| ping_ack_tps == tracker_slots[tracker_id].tracker_test_on_tps;
		bool batch_payload_matches = ping_ack_flag != ESB_PONG_FLAG_DATA_COLLECT_BATCH_ON
			|| payload->data[8] == tracker_slots[tracker_id].pending_cmd_arg;
		if (tracker_slots[tracker_id].tracker_remote_command == ping_ack_flag
		    && test_payload_matches && batch_payload_matches) {
			tracker_slots[tracker_id].tracker_remote_command = ESB_PONG_FLAG_NORMAL;
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
	tracker_slots[tracker_id].last_pong_queued_counter = counter;
}

#if defined(CONFIG_RELAY)
/* Learning does not affect receipt/deduplication evidence at the freeze.
 * Copy the grant with reports: a queued report must use its receive-time grant. */
struct relay_learning_record {
	struct relay_control grant;
	struct relay_peer_report report;
	uint32_t now;
	uint8_t helper, child, rssi, slot;
	bool is_report;
};
K_MSGQ_DEFINE(relay_learning_queue, sizeof(struct relay_learning_record), 16, 4);
static uint32_t relay_learning_overflow;

static void relay_learning_apply(const struct relay_learning_record *r)
#else
static void esb_receive_composite(const struct esb_payload *payload, uint32_t event_ticks)
#endif
{
#if defined(CONFIG_RELAY)
	unsigned key = irq_lock();
	if (r->grant.session == relay_session && r->grant.epoch == relay_epoch) {
		if (r->is_report) {
			(void)relay_coord_report(&relay_coord, &r->grant, r->slot, &r->report, r->now);
		} else {
			relay_coord_capture(&relay_coord, r->helper, r->child, r->rssi, r->now);
		}
#else
	/* Composite packet (type ESB_COMPOSITE_TYPE): variable length.
	 * Format: [ESB_COMPOSITE_TYPE][tracker_id][sub_count][sub_type0][sub_data0...]...[sequence]
	 * Each sub-packet: 1 byte type + variable data.
	 */
	if (payload->length < 5 || payload->data[0] != ESB_COMPOSITE_TYPE) {
		LOG_ERR("Wrong packet length: %d", payload->length);
		return;
#endif
	}
#if defined(CONFIG_RELAY)
	irq_unlock(key);
}
#endif

#if defined(CONFIG_RELAY)
static void relay_learning_submit(const struct relay_learning_record *r)
{
	if (k_msgq_put(&relay_learning_queue, r, K_NO_WAIT)) {
		/* Preserve ordering and evidence under overload; expose the slow fallback. */
		struct relay_learning_record pending;
		while (!k_msgq_get(&relay_learning_queue, &pending, K_NO_WAIT)) {
			relay_learning_apply(&pending);
		}
		relay_learning_overflow++;
		relay_learning_apply(r);
	}
}

static void relay_defer_capture(struct relay_coord *rc, uint8_t helper, uint8_t child,
	uint8_t rssi, uint32_t now)
{
	ARG_UNUSED(rc);
	struct relay_learning_record r = {.grant = {.session = relay_session, .epoch = relay_epoch},
		.helper = helper, .child = child, .rssi = rssi, .now = now};
	relay_learning_submit(&r);
}

static bool relay_defer_report(struct relay_coord *rc, const struct relay_control *grant,
	unsigned slot, const struct relay_peer_report *report, uint32_t now)
{
	ARG_UNUSED(rc);
	struct relay_learning_record r = {.grant = *grant, .report = *report,
		.slot = slot, .now = now, .is_report = true};
	relay_learning_submit(&r);
	return true;
}

static void relay_learning_thread(void)
{
	struct relay_learning_record r;
	for (;;) {
#if defined(CONFIG_TDMA_DIAGNOSTICS)
		tdma_deferred_process();
#endif
		/* Dequeue and apply atomically with respect to the overflow fallback. */
		unsigned key = irq_lock();
		int err = k_msgq_get(&relay_learning_queue, &r, K_NO_WAIT);
		if (!err) relay_learning_apply(&r);
		irq_unlock(key);
		if (err) k_usleep(200);
	}
}
K_THREAD_DEFINE(relay_learning_thread_id, 1536, relay_learning_thread, NULL, NULL, NULL, 3, 0, 0);

/* Copy accepted output records; timestamp, sequence and freeze evidence stay
 * in the ISR. 128 records cover >40 ms at three records per pose at 1300 Hz. */
struct relay_hid_record {
	uint8_t data[16];
	uint8_t rssi;
};
K_MSGQ_DEFINE(relay_hid_queue, sizeof(struct relay_hid_record), 128, 4);
static uint32_t relay_hid_overflow, relay_hid_dropped_output, relay_hid_enqueued, relay_hid_refused;

static void relay_hid_packet(const uint8_t *data, uint8_t rssi)
{
	struct relay_hid_record record = {.rssi = rssi};
	memcpy(record.data, data, sizeof(record.data));
	if (k_msgq_put(&relay_hid_queue, &record, K_NO_WAIT)) {
		/* USB can stall indefinitely. Drop this newest output in bounded ISR
		 * work; RF acceptance and deadlines do not depend on USB progress. */
		relay_hid_overflow++;
		relay_hid_dropped_output++;
	}
}

/* Worker retains one record until downstream accepts it, preserving order. */
static bool relay_hid_step(struct relay_hid_record *record, bool *pending)
{
	if (!*pending) {
		if (k_msgq_get(&relay_hid_queue, record, K_NO_WAIT)) return false;
		*pending = true;
	}
	if (!hid_try_write_packet_n(record->data, record->rssi)) {
		relay_hid_refused++;
		return false;
	}
	relay_hid_enqueued++;
	*pending = false;
	return true;
}

static void relay_hid_thread(void)
{
	struct relay_hid_record record;
	bool pending = false;
	for (;;) {
		if (!relay_hid_step(&record, &pending)) k_usleep(200);
	}
}
K_THREAD_DEFINE(relay_hid_thread_id, 768, relay_hid_thread, NULL, NULL, NULL, 3, 0, 0);

struct composite_result {
	uint8_t sub_count;
	int event_pos;
	bool business;
};

/* Full validation is deliberately distinct from conservative early receipts. */
static bool composite_validate(const struct esb_payload *payload, struct composite_result *out)
{
	if (payload->length < 5 || payload->data[0] != ESB_COMPOSITE_TYPE) return false;
#endif
	uint8_t tracker_id = payload->data[1];
	uint8_t sub_count = payload->data[2];
#if defined(CONFIG_RELAY)
	bool business = false;
#else

	if (tracker_id >= stored_trackers || tracker_id >= MAX_TRACKERS) {
		return;
	}

#endif
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
#if defined(CONFIG_RELAY)
		if (type != TRACKER_EVENT_ESB_TYPE) business = true;
#endif
		scan += body_len;
	}
	if (!valid || (!unknown_tail && scan != frame_end)) {
#if defined(CONFIG_RELAY)
		return false;
	}
	*out = (struct composite_result){sub_count, event_pos, business};
	return true;
}

static void esb_execute_composite(struct esb_payload *payload, uint32_t event_ticks, bool relayed,
                                  const struct composite_result *validated)
{
	/* Composite packet (type ESB_COMPOSITE_TYPE): variable length.
	 * Format: [ESB_COMPOSITE_TYPE][tracker_id][sub_count][sub_type0][sub_data0...]...[sequence]
	 * Each sub-packet: 1 byte type + variable data.
	 */
	if (payload->length < 5 || payload->data[0] != ESB_COMPOSITE_TYPE) {
		LOG_ERR("Wrong packet length: %d", payload->length);
#endif
		return;
	}
#if defined(CONFIG_RELAY)

	uint8_t tracker_id = payload->data[1];
	uint8_t sub_count = validated->sub_count;

	if (tracker_id >= stored_trackers || tracker_id >= MAX_TRACKERS) {
		return;
	}
	if (!relayed && relay_block_drop(tracker_id, payload->data[payload->length - 1])) {
		return;
	}

	int event_pos = validated->event_pos;
#endif
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
#if defined(CONFIG_RELAY)
	if (!relayed) {
		tdma_check_slot(tracker_id, event_ticks, payload->rssi);
	}
#else
	tdma_check_slot(tracker_id, event_ticks, payload->rssi);
#endif
#else
	ARG_UNUSED(event_ticks);
#endif

	LOG_DBG("Received composite packet from tracker %d with %d sub-packets", tracker_id, sub_count);

	/* Sequence byte is at the very end of the composite packet */
	uint8_t received_sequence = payload->data[payload->length - 1];
#if defined(CONFIG_RELAY)
	int seq_result = relay_sequence_check(tracker_id, received_sequence, relayed, (uint8_t)payload->rssi);
#else
	int seq_result = check_packet_sequence(tracker_id, received_sequence);
#endif
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
			struct packet_stats *stats = &tracker_slots[tracker_id].stats;
			pkt[4] = stats->status_received;
			pkt[5] = stats->status_lost;
			pkt[6] = 0;
			pkt[7] = 0;
			stats->status_received = 0;
			stats->status_lost = 0;
		}
#if defined(CONFIG_RELAY)
		if (sub_type == 0) {
			relay_take_caps(tracker_id, pkt);
		}
#endif

#if defined(CONFIG_RELAY)
		relay_hid_packet(pkt, payload->rssi);
#else
		hid_write_packet_n(pkt, payload->rssi);
#endif
		pos += sub_len;
	}
}

#if defined(CONFIG_RELAY)
static void esb_receive_composite(struct esb_payload *payload, uint32_t event_ticks, bool relayed)
{
	if (payload->length < 5 || payload->data[0] != ESB_COMPOSITE_TYPE) {
		LOG_ERR("Wrong packet length: %d", payload->length);
		return;
	}
	struct composite_result validated;
	if (composite_validate(payload, &validated)) {
		esb_execute_composite(payload, event_ticks, relayed, &validated);
	}
}

/* One 17-byte pose frame, received directly or extracted from a relay bundle.
 * Relayed frames skip the slot-timing diagnostic (they arrive in the head's
 * slot) and are otherwise processed exactly like direct ones, so HID output
 * is byte-identical. */
static void esb_receive_normal(struct esb_payload *payload, uint32_t event_ticks, bool relayed)
{
	if (payload->data[0] > 223) {
		return;
	}
	uint8_t tracker_id = payload->data[1];

#if defined(CONFIG_TDMA_DIAGNOSTICS)
	// TDMA Slot Check for Data (Type 17)
	if (!relayed) {
		tdma_check_slot(tracker_id, event_ticks, payload->rssi);
	}
#else
	ARG_UNUSED(event_ticks);
#endif

	if (tracker_id >= stored_trackers || tracker_id >= MAX_TRACKERS) {
		return;
	}

	uint8_t received_sequence = payload->data[16];
	if (!relayed && relay_block_drop(tracker_id, received_sequence)) {
		return;
	}
	int seq_result = relay_sequence_check(tracker_id, received_sequence, relayed, (uint8_t)payload->rssi);
	if (payload->pipe == 1 + (tracker_id % 7)) {
		tdma_shadow_observe(tracker_id, TDMA_SHADOW_EVIDENCE_DATA);
	}
	// Decide whether to forward the packet based on the sequence check result
	// seq_result: 0=normal, 1=potential loss, 2=out of order, 3=reboot, 4=duplicate
	if (seq_result == 4) {
		LOG_DBG("TRK %d: Duplicate packet seq=%d, dropped", tracker_id, received_sequence);
		// Drop duplicate packet
		return;
	}
	if (seq_result == 2) {
		LOG_DBG("TRK %d: Out-of-order packet seq=%d, dropped", tracker_id, received_sequence);
		// Drop out-of-order packet to avoid incorrect pose calculation
		return;
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
	if (payload->data[0] == 3) {
		struct packet_stats *stats = &tracker_slots[tracker_id].stats;
		payload->data[4] = stats->status_received;
		payload->data[5] = stats->status_lost;
		payload->data[6] = 0; // windows_hit (not implemented)
		payload->data[7] = 0; // windows_missed (not implemented)
		stats->status_received = 0;
		stats->status_lost = 0;
	}
	if (payload->data[0] == 0) {
		relay_take_caps(tracker_id, payload->data);
	}
	relay_hid_packet(payload->data,
					   payload->rssi); // write to hid endpoint
}

static void esb_receive_peer_report(const struct esb_payload *payload, uint32_t event_ticks)
{
	struct relay_peer_report report;
	if (!relay_profile || !relay_peer_report_decode(payload->data, payload->length, &report) ||
	    payload->pipe != 1U + report.helper % 7U) {
		return;
	}
	uint32_t preamble = event_ticks - RELAY_V3_AIR_TICKS(payload->length);
	uint32_t frame = relay_frame_near(relay_frame_anchor, relay_start_anchor, preamble);
	uint32_t phase = relay_frame_phase(relay_origin, frame, preamble);
	if (frame != relay_grants.frame || phase < RELAY_V3_REPAIR_START) {
		return;
	}
	unsigned slot = relay_repair_slot(phase);
	(void)relay_defer_report(&relay_coord, &relay_grants, slot, &report, k_uptime_get_32());
}

static void esb_receive_repair(const struct esb_payload *payload, uint32_t event_ticks)
{
	struct relay_repair repair;
	relay_rx_stats.repair.seen++;
	if (!relay_profile || !relay_repair_decode(payload->data, payload->length, &repair) ||
	    repair.session != relay_session || repair.epoch != relay_grants.epoch ||
	    payload->pipe != 1U + repair.helper % 7U) {
		relay_rx_stats.repair.invalid++;
		return;
	}
	uint32_t preamble = event_ticks - RELAY_V3_AIR_TICKS(payload->length);
	uint32_t frame = relay_frame_near(relay_frame_anchor, relay_start_anchor, preamble);
	uint32_t phase = relay_frame_phase(relay_origin, frame, preamble);
	if (frame != relay_grants.frame || phase < RELAY_V3_REPAIR_START) {
		relay_rx_stats.repair.phase++;
		return;
	}
	unsigned slot = relay_repair_slot(phase);
	if (slot >= relay_grants.repair_count || relay_grants.grants[slot].owner != repair.helper) {
		relay_rx_stats.repair.slot++;
		return;
	}
	relay_rx_stats.records += repair.count;
	struct composite_result validated[RELAY_MAX_CHILDREN];
	for (unsigned i = 0; i < repair.count; i++) {
		struct relay_repair_record *r = &repair.records[i];
		uint8_t child = r->source.frame[1];
		uint16_t age = (uint16_t)(frame - r->capture_frame);
		if (r->source.frame[0] == RELAY_COMPOSITE_TYPE) {
			struct esb_payload part = {.length = r->source.length};
			memcpy(part.data, r->source.frame, part.length);
			if (!composite_validate(&part, &validated[i]) || !validated[i].business) {
				relay_rx_stats.repair.record++;
				return;
			}
		}
		if (child >= stored_trackers || age >= relay_control_cache_frames(&relay_grants)
			|| !(relay_grants.grants[slot].children & BIT(child))
			|| !(relay_control_missing(&relay_grants, age) & BIT(child))) {
			relay_rx_stats.repair.record++;
			return;
		}
	}
#ifdef CONFIG_RELAY_FRAME_LEDGER
	relay_pool_received[slot]++;
#endif

	for (unsigned i = 0; i < repair.count; i++) {
		struct relay_repair_record *r = &repair.records[i];
		uint8_t child = r->source.frame[1];
		uint32_t captured = frame - (uint16_t)(frame - r->capture_frame);
		relay_rx_stats.repair.received[child]++;
		relay_defer_capture(&relay_coord, repair.helper, child, r->source.rssi, k_uptime_get_32());
		uint8_t skipped = relay_coord_skipped(&relay_coord, child, captured);
		if (!relay_coord_publish(&relay_coord, child, captured)) {
			relay_rx_stats.duplicate_pose++;
			if (relay_coord_was_published(&relay_coord, child, captured)) {
				relay_rx_stats.repair.duplicate[child]++;
#ifdef CONFIG_RELAY_FRAME_LEDGER
				ledger_note(relay_session, captured, child, LEDGER_DUPLICATE, -(int16_t)r->source.rssi, repair.helper, frame - captured);
#endif
			} else {
				relay_rx_stats.repair.obsolete[child]++;
#ifdef CONFIG_RELAY_FRAME_LEDGER
				ledger_note(relay_session, captured, child, LEDGER_OBSOLETE, -(int16_t)r->source.rssi, repair.helper, frame - captured);
#endif
			}
			continue;
		}
		struct esb_payload part = {.length = r->source.length,
					   .rssi = MAX(payload->rssi, r->source.rssi)};
		memcpy(part.data, r->source.frame, part.length);
		relay_repair_delivery = true;
		if (part.data[0] == RELAY_COMPOSITE_TYPE) {
			esb_execute_composite(&part, event_ticks, true, &validated[i]);
		} else {
			esb_receive_normal(&part, event_ticks, true);
		}
		relay_repair_delivery = false;
		relay_rx_stats.records_accepted++;
		relay_rx_stats.repair.age[child][frame - captured]++;
#ifdef CONFIG_RELAY_FRAME_LEDGER
		ledger_note(relay_session, captured, child, LEDGER_ACCEPT, -(int16_t)r->source.rssi, repair.helper, frame - captured);
#endif
		/* A fresh repaired pose also renews the child's membership. */
		tdma_shadow_observe(child, TDMA_SHADOW_EVIDENCE_DATA);
		relay_coord_note_frame(&relay_coord, child, RC_RELAYED, RC_SEQ_NORMAL, skipped,
				       payload->rssi, k_uptime_get_32());
	}
}

/* Relay bundle from a head: its own frame counts as direct reception from the
 * head; each record is the child's frame exactly as the child sent it. */
static void esb_receive_bundle(const struct esb_payload *payload, uint32_t event_ticks)
{
	if (relay_profile) {
		return;
	}
	struct relay_bundle_view view;
	relay_rx_stats.bundles++;
	if (!relay_bundle_decode(payload->data, (uint8_t)payload->length, &view)
	    || view.head_id >= stored_trackers || view.head_id >= MAX_TRACKERS
	    || payload->pipe != 1U + view.head_id % 7U) {
		relay_rx_stats.bundle_invalid++;
		return;
	}
	/* Blocking the head blocks its whole uplink, bundles included. */
	uint8_t salt = view.own_len ? view.own[view.own_len - 1]
		: view.records[0].frame[view.records[0].length - 1];
	if (relay_block_drop(view.head_id, salt ^ 0x5AU)) {
		return;
	}
	unsigned int key = irq_lock();
	relay_coord_note_bundle(&relay_coord, view.head_id, view.overflow);
	irq_unlock(key);

	static struct esb_payload part;
	if (view.own_len) {
		memcpy(part.data, view.own, view.own_len);
		part.length = view.own_len;
		part.pipe = payload->pipe;
		part.rssi = payload->rssi;
		if (part.data[0] == ESB_COMPOSITE_TYPE) {
			esb_receive_composite(&part, event_ticks, false);
		} else {
			esb_receive_normal(&part, event_ticks, false);
		}
	}
	for (uint8_t i = 0; i < view.record_count; i++) {
		const struct relay_record_view *record = &view.records[i];
		uint8_t child = record->frame[1];
		if (child >= stored_trackers || child >= MAX_TRACKERS) {
			continue;
		}
		relay_rx_stats.records++;
		memcpy(part.data, record->frame, record->length);
		part.length = record->length;
		part.pipe = 1U + child % 7U; /* the head received it on this pipe */
		/* Report the weaker hop so the server never sees a rosier link. */
		part.rssi = (int8_t)MAX((uint8_t)payload->rssi, record->rssi);
		if (part.data[0] == ESB_COMPOSITE_TYPE) {
			esb_receive_composite(&part, event_ticks, true);
		} else {
			esb_receive_normal(&part, event_ticks, true);
		}
	}
}

#ifdef CONFIG_RELAY_CYCLE_DIAGNOSTICS
static void relay_rx_measure(uint32_t start)
{
	uint32_t cycles = DWT->CYCCNT - start;
	relay_rx_cycles_max = MAX(relay_rx_cycles_max, cycles);
	relay_rx_cycles_total += cycles;
	relay_rx_packets++;
}

#endif
#endif
static void event_handler(struct esb_evt const *event)
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
#if defined(CONFIG_RELAY)
#ifdef CONFIG_RELAY_CYCLE_DIAGNOSTICS
		bool measured = false;
		uint32_t packet_start = 0;
#endif
#endif
		while (!err) {
#if defined(CONFIG_RELAY)
#ifdef CONFIG_RELAY_CYCLE_DIAGNOSTICS
			if (measured) relay_rx_measure(packet_start);
			measured = false;
			packet_start = DWT->CYCCNT;
#endif
#endif
			err = esb_read_rx_payload(&rx_payload);
			if (err == -ENODATA) {
				break;
			} else if (err) {
				LOG_ERR("Error while reading rx packet: %d", err);
				break;
			}
#if defined(CONFIG_RELAY)
#ifdef CONFIG_RELAY_CYCLE_DIAGNOSTICS
			measured = true;
#endif
			uint32_t current_rx_ticks = rx_payload.rx_ticks;
			if (relay_profile && rx_payload.rx_context != relay_layout_generation) continue;
			relay_direct_start = current_rx_ticks - RELAY_V3_AIR_TICKS(rx_payload.length);
			relay_direct_pose = rx_payload.rx_pose;
#else
#if defined(CONFIG_TDMA_DIAGNOSTICS)
			uint32_t current_rx_ticks = k_uptime_ticks();
#else
			const uint32_t current_rx_ticks = 0;
#endif
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
#if defined(CONFIG_RELAY)
			if (!discovery && rx_payload.data[0] == RELAY_DIAG_TYPE) {
				esb_receive_relay_diag(&rx_payload);
				continue;
			}
			if (!discovery && rx_payload.data[0] == RELAY_PEER_REPORT_V3) {
				esb_receive_peer_report(&rx_payload, current_rx_ticks);
				continue;
			}
			if (!discovery && rx_payload.data[0] == RELAY_REPAIR_V3) {
				esb_receive_repair(&rx_payload, current_rx_ticks);
				continue;
			}
			if (!discovery && rx_payload.data[0] == RELAY_BUNDLE_TYPE) {
				esb_receive_bundle(&rx_payload, current_rx_ticks);
				continue;
			}
#endif
			if (!discovery && rx_payload.data[0] == ESB_COMPOSITE_TYPE) {
#if defined(CONFIG_RELAY)
				esb_receive_composite(&rx_payload, current_rx_ticks, false);
#else
				esb_receive_composite(&rx_payload, current_rx_ticks);
#endif
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
					if (atomic_get(&pairing_new_devices_blocked) && known_id < 0) {
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
							LOG_DBG("New device %012llX pairing request queued", found_addr);
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
#if defined(CONFIG_RELAY)
				if (rx_payload.data[0] == ESB_PING_TYPE
				    && !relay_block_drop(rx_payload.data[1], rx_payload.data[2] ^ 0xA5U)) {
#else
				if (rx_payload.data[0] == ESB_PING_TYPE) {
#endif
					esb_receive_ping(&rx_payload, current_rx_ticks);
				}
				/* Non-PING length-13 OTA types handled at case entry. */
			} break;
			case 17: // 16 bytes data + 1 byte sequence number
#if defined(CONFIG_RELAY)
				esb_receive_normal(&rx_payload, current_rx_ticks, false);
				break;
#else
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
					struct packet_stats *stats = &tracker_slots[tracker_id].stats;
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
#endif
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
							if (tracker_slots[tracker_id].last_raw_valid && tracker_slots[tracker_id].last_raw_seq == seq) {
								break;
							}
							tracker_slots[tracker_id].last_raw_seq = seq;
							tracker_slots[tracker_id].last_raw_valid = true;
						}
						data_collect_write(rx_payload.data, rx_payload.length, rx_payload.rssi);
					} else if (!data_collect_is_active() && !data_collect_batch_is_active()) {
						uint8_t pending = tracker_slots[tracker_id].tracker_remote_command;
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
#if defined(CONFIG_RELAY)
		config.rx_metadata_handler = relay_rx_metadata;
#endif
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
	memset(&tracker_slots[id], 0, sizeof(tracker_slots[id]));
#if defined(CONFIG_TDMA_DIAGNOSTICS)
#if defined(CONFIG_RELAY)
	tdma_stats_generation[id]++;
#endif
	memset(&tracker_diags[id], 0, sizeof(tracker_diags[id]));
#endif
	atomic_and(&test_all_confirmed_mask, (atomic_val_t)~BIT(id));
	atomic_set(&test_all_ready_after_ms[id], 0);
	channel_target_mask &= ~BIT(id);
	tdma_shadow_forget_tracker(id);
#if defined(CONFIG_RELAY)
	relay_coord_forget(&relay_coord, id, k_uptime_get_32());
#endif
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

/* Pairing requests are processed by the ESB worker; keep retries visible
 * without flooding the console when a tracker keeps searching. */
static void warn_pairing_storage_full(uint64_t addr, uint8_t count)
{
	static bool warned;
	static int64_t last_warning_ms;
	int64_t now_ms = k_uptime_get();

	if (!warned || now_ms - last_warning_ms >= 5000) {
		warned = true;
		last_warning_ms = now_ms;
		LOG_WRN("PAIRING FAILED: tracker storage full (%u/%u), cannot pair %012llX. "
			"Use 'list' to see paired trackers, 'remove' to remove the last, "
			"or 'clear' to delete ALL.",
			(unsigned int)count, (unsigned int)MAX_TRACKERS, (unsigned long long)addr);
	}
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
	if (checksum_valid && tracker_roster_address_valid(found_addr)
	    && send_tracker_id == tracker_count_snapshot && !has_capacity) {
		warn_pairing_storage_full(found_addr, tracker_count_snapshot);
		return false;
	}
	bool is_new_device = checksum_valid && found_addr != 0 && send_tracker_id == tracker_count_snapshot && has_capacity;
	bool ack_valid = false;

	if (is_new_device) {
		int assigned_id = esb_add_pair(found_addr, false);
		if (assigned_id == -ENOSPC) {
			LOG_WRN("Maximum tracker slots reached, cannot pair %012llX", found_addr);
			return false;
		}
		if (assigned_id < 0) {
			LOG_ERR("Failed to store tracker address %012llX: %d", found_addr, assigned_id);
			return false;
		}
		send_tracker_id = (uint16_t)assigned_id;
		set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
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
	atomic_set(&pairing_new_devices_blocked, 0);
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
		     tracker_slots[id].stats.last_packet_time >= scan_start_time))) {
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
			tracker_slots[id].wom_token = wom_next_token;
			tracker_slots[id].wom_minutes = minutes;
			tracker_slots[id].wom_error = 0;
			tracker_slots[id].wom_complete = false;
			tracker_slots[id].wom_advertised = false;
			tracker_slots[id].wom_first_ping_counter = 0;
			tracker_slots[id].tracker_remote_command = ESB_PONG_FLAG_WOM_CONTROL;
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
	bool matches = tracker_slots[id].wom_token == token;
	/* Another control command displaced the pending request: never treat its
	 * eventual generic confirmation as proof of a completed WOM command. */
	if (matches && !tracker_slots[id].wom_complete && tracker_slots[id].tracker_remote_command != ESB_PONG_FLAG_WOM_CONTROL) {
		tracker_slots[id].wom_minutes = 0;
		tracker_slots[id].wom_error = ECANCELED;
		tracker_slots[id].wom_complete = true;
	}
	bool complete = matches && tracker_slots[id].wom_complete;
	if (complete) {
		*minutes = tracker_slots[id].wom_minutes;
		*error = tracker_slots[id].wom_error;
	}
	irq_unlock(key);
	return complete;
}

void esb_cancel_remote_wom_command(uint32_t mask, uint16_t token)
{
	unsigned int key = irq_lock();
	for (uint8_t id = 0; id < MAX_TRACKERS; id++) {
		if ((mask & BIT(id)) && token != 0 && tracker_slots[id].wom_token == token) {
			if (tracker_slots[id].tracker_remote_command == ESB_PONG_FLAG_WOM_CONTROL) {
				tracker_slots[id].tracker_remote_command = ESB_PONG_FLAG_NORMAL;
			}
			tracker_slots[id].wom_token = 0;
			tracker_slots[id].wom_minutes = 0;
			tracker_slots[id].wom_error = 0;
			tracker_slots[id].wom_complete = false;
			tracker_slots[id].wom_advertised = false;
			tracker_slots[id].wom_first_ping_counter = 0;
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
	tracker_slots[tracker_id].pending_sens_data[0] = sensitivity[0];
	tracker_slots[tracker_id].pending_sens_data[1] = sensitivity[1];
	tracker_slots[tracker_id].pending_sens_data[2] = sensitivity[2];
	tracker_slots[tracker_id].tracker_remote_command = ESB_PONG_FLAG_SENS_SET;
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
			   && tracker_slots[tracker_id].stats.last_packet_time > 0
			   && now - tracker_slots[tracker_id].stats.last_packet_time <= PING_TIMEOUT_MS;
	if (!active) {
		irq_unlock(key);
		k_mutex_unlock(&tracker_store_lock);
		LOG_WRN("SENS_AUTO not queued for inactive tracker %u", tracker_id);
		return false;
	}
	tracker_slots[tracker_id].pending_sens_auto_axis = axis;
	tracker_slots[tracker_id].pending_sens_auto_revolutions = revolutions;
	tracker_slots[tracker_id].tracker_remote_command = ESB_PONG_FLAG_SENS_AUTO;
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
		if (stored_tracker_addr[i] != 0 && tracker_slots[i].stats.last_packet_time >= (uint64_t)scan_start_time) {
			tracker_slots[i].pending_sens_auto_axis = axis;
			tracker_slots[i].pending_sens_auto_revolutions = revolutions;
			tracker_slots[i].tracker_remote_command = ESB_PONG_FLAG_SENS_AUTO;
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
		if (stored_tracker_addr[i] != 0 && tracker_slots[i].stats.last_packet_time >= scan_start_time) {
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
			tracker_slots[i].tracker_remote_command = command_flag;
		}
	}
}

/* Test-only publication, called under the test irq_lock: publish only onto
 * idle slots (no pending command is stomped) whose recovery/join grace has
 * elapsed; skipped trackers converge via reconciliation after their grace. */
static void esb_publish_test_flag_mask(uint8_t command_flag, uint32_t mask, uint32_t now_u32)
{
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		if (!(mask & BIT(i)) || tracker_slots[i].tracker_remote_command != ESB_PONG_FLAG_NORMAL) {
			continue;
		}
		uint32_t ready_after = (uint32_t)atomic_get(&test_all_ready_after_ms[i]);
		if ((int32_t)(now_u32 - ready_after) < 0) {
			continue;
		}
		tracker_slots[i].tracker_remote_command = command_flag;
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
	tracker_slots[tracker_id].tracker_test_on_tps = tps;
	__asm__ volatile("" ::: "memory");
	tracker_slots[tracker_id].tracker_remote_command = ESB_PONG_FLAG_TEST_MODE_ON;
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
	tracker_slots[tracker_id].tracker_test_on_tps = 0;
	__asm__ volatile("" ::: "memory");
	tracker_slots[tracker_id].tracker_remote_command = ESB_PONG_FLAG_TEST_MODE_OFF;
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
	uint8_t active_count = tdma_dynamic_active_count;
	uint16_t effective = MIN(requested, test_all_capacity_tps(active_count));
	test_all_effective_tps = effective;
	uint16_t wire_tps = test_all_wire_tps();
	uint32_t now_u32 = (uint32_t)k_uptime_get();
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		tracker_slots[i].tracker_test_on_tps = wire_tps;
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
		effective,
		active_count,
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
		tracker_slots[i].tracker_test_on_tps = 0;
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
	    tracker_slots[tracker_id].stats.last_packet_time == 0 ||
	    now - tracker_slots[tracker_id].stats.last_packet_time > PING_TIMEOUT_MS) {
		return false;
	}
	unsigned int key = irq_lock();
	metadata_next_token++;
	if (metadata_next_token == 0) {
		metadata_next_token = 1;
	}
	tracker_slots[tracker_id].metadata_pending_token = metadata_next_token;
	tracker_slots[tracker_id].metadata_pending_mask = mask;
	tracker_slots[tracker_id].metadata_pending_chunk = chunk;
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
	if (tracker_slots[tracker_id].tracker_remote_command == ESB_PONG_FLAG_OTA_ABORT) {
		tracker_slots[tracker_id].tracker_remote_command = ESB_PONG_FLAG_NORMAL;
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
			tracker_slots[tracker_id].last_raw_valid = false;
		}
		if (command_flag == ESB_PONG_FLAG_DATA_COLLECT_ON) {
			raw_arq_reset();
		}
		tracker_slots[tracker_id].pending_cmd_arg = arg;
		tracker_slots[tracker_id].tracker_remote_command = command_flag;
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
		const struct packet_stats *stats = &tracker_slots[i].stats;
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
#if defined(CONFIG_RELAY)
	LOG_INF("HEALTH pose direct=%u repair=%u duplicate=%u",
		relay_rx_stats.direct_accepted, relay_rx_stats.records_accepted, relay_rx_stats.duplicate_pose);
#endif
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
		uint8_t slot = (tracker_slots[i].tdma_config_packed >> 24) & 0xFF;
		if (slot < tdma_dynamic_active_count) {
			slot_owner[slot] = i;
		}
	}

	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		const struct packet_stats *stats = &tracker_slots[i].stats;
		if (!(observed_mask & BIT(i))) {
			continue;
		}
		uint8_t assigned_slot = (tracker_slots[i].tdma_config_packed >> 24) & 0xFF;
		uint8_t predecessor_slot = assigned_slot == 0
			? tdma_dynamic_active_count - 1U : assigned_slot - 1U;
		uint8_t predecessor_id = predecessor_slot < MAX_TRACKERS
			? slot_owner[predecessor_slot] : 0xFF;
		uint64_t age_ms = stats->last_packet_time > 0 && stats->last_packet_time <= now
			? now - stats->last_packet_time : 0;
		const struct tdma_stats *tdma = &tracker_diags[i].stats;
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
			int64_t sync_age_ms = tracker_diags[i].g_last_ping_isr_rx_ticks_raw != 0
				? k_ticks_to_ms_floor64((uint32_t)k_uptime_ticks() - tracker_diags[i].g_last_ping_isr_rx_ticks_raw) : -1;
			LOG_INF(
				"HEALTH SYNC trk=%u bias=%d skew_ppb=%d bias_valid=%u skew_valid=%u ping_age_ms=%lld ping_rej=%u extrap_skip=%u",
				i,
				tracker_diags[i].g_last_ping_rx_time_diff_ticks,
				tracker_diags[i].g_bias_ppb_valid ? tracker_diags[i].g_bias_ppb : 0,
				tracker_diags[i].g_last_ping_rx_time_diff_valid ? 1 : 0,
				tracker_diags[i].g_bias_ppb_valid,
				sync_age_ms,
				tracker_diags[i].g_ping_reject_count,
				tracker_diags[i].g_extrap_skip_count
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
		memset(&tracker_slots[i].stats, 0, sizeof(struct packet_stats));
		tracker_slots[i].ping_counter_initialized = false;
		tracker_slots[i].last_ping_counter = 0;
		tracker_slots[i].last_pong_queued_counter = 0;
	}
	irq_unlock(key);
	tdma_sync_stats_reset();
	tdma_loss_reset_windows();
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	tdma_last_loss_step_ms = 0;
	tdma_loss_last_permille = 0;
#endif
	for (uint8_t i = 0; i < MAX_TRACKERS; ++i) {
		tracker_slots[i].tdma_prev_recv = 0;
		tracker_slots[i].tdma_prev_gaps = 0;
		tracker_slots[i].tdma_prev_restarts = 0;
	}
	LOG_INF("All packet statistics have been reset");
}
// Toggle detailed statistics display on/off
bool esb_toggle_stats_detailed(void)
{
	k_spinlock_key_t key = k_spin_lock(&stats_display_lock);
	stats_detailed_enabled = !stats_detailed_enabled;
	stats_detailed_end_time = 0;
	bool enabled = stats_detailed_enabled;
	k_spin_unlock(&stats_display_lock, key);
	return enabled;
}

// Enable detailed statistics display for a specified duration (0 = toggle on/off permanently)
void esb_set_stats_detailed(uint32_t duration_seconds)
{
	int64_t end_time = duration_seconds
		? k_uptime_get() + (int64_t)duration_seconds * 1000 : 0;
	k_spinlock_key_t key = k_spin_lock(&stats_display_lock);
	if (duration_seconds == 0) {
		// Toggle mode
		stats_detailed_enabled = !stats_detailed_enabled;
		stats_detailed_end_time = 0;
	} else {
		// Timed mode
		stats_detailed_enabled = true;
		stats_detailed_end_time = end_time;
	}
	k_spin_unlock(&stats_display_lock, key);
}

// Get current detailed stats status
bool esb_get_stats_detailed_enabled(void)
{
	k_spinlock_key_t key = k_spin_lock(&stats_display_lock);
	bool enabled = stats_detailed_enabled;
	k_spin_unlock(&stats_display_lock, key);
	return enabled;
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
#if defined(CONFIG_RELAY)
	relay_session = sys_rand32_get();
#endif

#if defined(CONFIG_RELAY)
	relay_settings_load();
#endif
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
	esb_set_addr_unified();
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
						atomic_set(&pairing_new_devices_blocked, 1); // Allow re-pairing only
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
#if defined(CONFIG_RELAY)

/* ---------------------------------------------------------------------------
 * Relay console / OTA interface.
 * ------------------------------------------------------------------------- */
void esb_relay_set_enabled(bool enabled)
{
	unsigned int key = irq_lock();
	relay_user_enabled = enabled;
	relay_coord_set_enabled(&relay_coord, enabled && !esb_ota_relay_is_active());
	irq_unlock(key);
}

void esb_relay_set_v3_enabled(bool enabled)
{
	unsigned int key = irq_lock();
	relay_coord.v3_requested = enabled;
	irq_unlock(key);
}

void esb_relay_pin(uint8_t id, bool pinned)
{
	unsigned int key = irq_lock();
	relay_coord_pin(&relay_coord, id, pinned);
	irq_unlock(key);
}

void esb_relay_block(uint8_t id, uint8_t percent)
{
	unsigned int key = irq_lock();
	relay_block_id = percent ? id : 0xFF;
	relay_block_percent = MIN(percent, 100U);
	relay_block_mask = 0;
	irq_unlock(key);
}

bool esb_relay_block_mask(uint16_t mask, uint8_t seconds)
{
	unsigned count = relay_mask_count(mask);
	if (count > 3 || seconds > 60 || (mask && !seconds)) {
		return false;
	}
	unsigned key = irq_lock();
	relay_block_id = 0xff;
	relay_block_percent = 0;
	relay_block_deadline = k_uptime_get_32() + seconds * 1000U;
	relay_block_mask = mask;
	irq_unlock(key);
	return true;
}

bool esb_relay_blocks_ota(uint8_t id)
{
	unsigned int key = irq_lock();
	bool relayed = relay_coord_is_relayed(&relay_coord, id)
		|| !relay_coord_direct_ready(&relay_coord, id, k_uptime_get_32());
	irq_unlock(key);
	return relayed;
}

void esb_relay_print_pool(void)
{
#ifdef CONFIG_RELAY_FRAME_LEDGER
	for (unsigned ch = 0; ch < 16; ch++) {
		printk(
			"  direct_frames child=%u expected=%u missing=%u\n",
			ch,
			relay_direct_expected[ch],
			relay_direct_missing[ch]
		);
	}
	for (unsigned r = 0; r < 6; r++) {
		printk("  R_frames R=%u frames=%u\n", r, relay_r_frames[r]);
	}
	for (unsigned slot = 0; slot < MAX(2U, RELAY_V3_REPAIR_SLOTS); slot++) {
		printk(
			"  pool slot=%u repair=%u report=%u maintenance=%u idle=%u received=%u\n",
			slot,
			relay_pool_reserved[slot][0],
			relay_pool_reserved[slot][1],
			relay_pool_reserved[slot][2],
			relay_pool_reserved[slot][3],
			relay_pool_received[slot]
		);
	}
#endif
}

void esb_relay_layout_print(void)
{
	unsigned key = irq_lock();
	struct relay_runtime_settings desired = relay_settings, applied = relay_applied_settings;
	uint32_t requested = relay_settings_generation, current = relay_applied_generation;
	uint8_t direct = RELAY_V3_DIRECT_SLOTS, profile = relay_profile;
	struct relay_control announced = relay_grants;
	irq_unlock(key);
	printk(
		"relay layout: max_trackers=%u max_repairs=%u slot_ticks=%u tps_min=%u tps_max=%u "
		"generation=%u applied=%u repair_floor=%u second_b=%u\n",
		desired.max_trackers,
		desired.max_repairs,
		desired.slot_ticks,
		desired.tps_min,
		desired.tps_max,
		requested,
		current,
		desired.repair_floor,
		desired.second_b
	);
	printk(
		"relay layout active: profile=%u direct=%u max_repairs=%u slot_ticks=%u tps_min=%u "
		"tps_max=%u repair_floor=%u second_b=%u\n",
		profile,
		direct,
		applied.max_repairs,
		applied.slot_ticks,
		applied.tps_min,
		applied.tps_max,
		applied.repair_floor,
		applied.second_b
	);
	printk(
		"relay B plan: session=%u frame=%u direct=%u R=%u next_R=%u next2_R=%u slot_ticks=%u tps_max=%u\n",
		announced.session,
		announced.frame,
		announced.direct_count,
		announced.repair_count,
		relay_control_next_repairs(&announced),
		relay_control_next2_repairs(&announced),
		announced.slot_ticks,
		announced.tps_max
	);
}

int esb_relay_layout_set(const char *name, uint16_t value)
{
	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	struct relay_runtime_settings candidate = relay_settings;
	if (!strcmp(name, "max_trackers") && value <= 16U) {
		candidate.max_trackers = value;
	} else if (!strcmp(name, "max_repairs") && value <= 5U) {
		candidate.max_repairs = value;
		/* Equality selects static mode and follows every ceiling edit. */
		if (relay_settings.repair_floor == relay_settings.max_repairs || candidate.repair_floor > value) {
			candidate.repair_floor = value;
		}
	} else if (!strcmp(name, "second_b") && value <= 2U) {
		candidate.second_b = value;
	} else if (!strcmp(name, "repair_floor") && value <= 5U) {
		candidate.repair_floor = value;
	} else if (!strcmp(name, "slot_ticks") && value <= 255U) {
		candidate.slot_ticks = value;
	} else if (!strcmp(name, "tps_min")) {
		candidate.tps_min = value;
	} else if (!strcmp(name, "tps_max")) {
		candidate.tps_max = value;
	} else {
		k_mutex_unlock(&tracker_store_lock);
		return -EINVAL;
	}
	if (!relay_runtime_validate(&candidate, IS_ENABLED(CONFIG_RELAY_EXPERIMENTAL_SLOT_15))) {
		k_mutex_unlock(&tracker_store_lock);
		return -EINVAL;
	}
	if (candidate.max_trackers == relay_settings.max_trackers && candidate.max_repairs == relay_settings.max_repairs
		&& candidate.slot_ticks == relay_settings.slot_ticks && candidate.tps_min == relay_settings.tps_min
		&& candidate.tps_max == relay_settings.tps_max && candidate.repair_floor == relay_settings.repair_floor
		&& candidate.second_b == relay_settings.second_b) {
		k_mutex_unlock(&tracker_store_lock);
		esb_relay_layout_print();
		return 0;
	}
	uint8_t record[10]
		= {3, candidate.max_trackers, candidate.max_repairs, candidate.slot_ticks | (candidate.second_b << 6)};
	relay_put_be16(record + 4, candidate.tps_min);
	relay_put_be16(record + 6, candidate.tps_max);
	record[8] = candidate.repair_floor;
	record[9] = relay_crc8(7, record, 9);
	int result = sys_write_checked(RELAY_LAYOUT_SETTINGS, record, sizeof(record));
	if (!result) {
		unsigned key = irq_lock();
		relay_settings = candidate;
		relay_settings_generation++;
		irq_unlock(key);
		tdma_recalculate(true);
	}
	k_mutex_unlock(&tracker_store_lock);
	if (!result) {
		k_sem_give(&tdma_membership_wake);
		esb_relay_layout_print();
	}
	return result;
}

void esb_relay_print_status(void)
{
#ifdef CONFIG_RELAY_IRQ_DIAGNOSTICS
    unsigned probe_key = irq_lock();
    uint32_t pm = irq_probe_plan_max, pc = irq_probe_plan_count;
    uint32_t tm = irq_probe_tick_max, tc = irq_probe_tick_count;
    irq_unlock(probe_key);
    printk("irq_probe plan count=%u max=%u tick count=%u max=%u cycles\n", pc, pm, tc, tm);
#endif
	static struct relay_coord snapshot;
	/* Diagnostic rows need not share one instant. Bound interrupt masking
	 * while copying the large coordinator, so querying it cannot delay B. */
	for (size_t offset = 0; offset < sizeof(snapshot); offset += 128U) {
		unsigned key = irq_lock();
		memcpy((uint8_t *)&snapshot + offset, (const uint8_t *)&relay_coord + offset,
			MIN(128U, sizeof(snapshot) - offset));
		irq_unlock(key);
	}
	unsigned int key = irq_lock();
	struct relay_control control = relay_grants;
	uint16_t members = (uint16_t)tdma_active_mask;
	uint8_t block_id = relay_block_id, block_pct = relay_block_percent;
	uint16_t block_mask = relay_block_mask;
	int32_t block_left = (int32_t)(relay_block_deadline - k_uptime_get_32());
#ifdef CONFIG_RELAY_CYCLE_DIAGNOSTICS
	uint64_t rx_cycles = relay_rx_cycles_total;
	uint32_t rx_packets = relay_rx_packets, rx_max = relay_rx_cycles_max;
#endif
	irq_unlock(key);
	printk("relay: %s pinned=0x%04x members=0x%04x forward=0x%04x relayed=0x%04x assignments=%u rotations=%u releases=%u\n",
		snapshot.enabled ? "on" : (relay_user_enabled ? "paused(ota)" : "off"),
		snapshot.pinned, members, snapshot.forward_mask, snapshot.relayed_mask,
		snapshot.assignments, snapshot.rotations, snapshot.releases);
	printk("relay: profile=%u B_sent=%u B_failed=%u last_err=%d\n", relay_profile, relay_b_sent,
		relay_b_failed, relay_b_err);
	printk("relay: B2_sent=%u B2_failed=%u last_err=%d\n", relay_b2_sent, relay_b2_failed, relay_b2_err);
	printk("relay: B late=%u late_max=%u plan_max=%u grant_max=%u ticks\n", relay_b_late,
		relay_b_late_max, relay_b_plan_max, relay_b_grant_max);
	printk("relay: learning irq_max=%u ticks\n", relay_learn_lock_max);
	printk("relay: HID enqueued=%u refused_attempts=%u dropped_output=%u overflow=%u\n",
		relay_hid_enqueued, relay_hid_refused, relay_hid_dropped_output, relay_hid_overflow);
#ifdef CONFIG_RELAY_CYCLE_DIAGNOSTICS
	printk("relay: B entry_late=%u entry_max=%u ticks encode_max=%u cycles\n",
		relay_b_entry_late, relay_b_entry_max, relay_b_encode_cycles_max);
	printk("relay: B freeze_arm_max=%u cycles; RX max=%u avg=%u cycles packets=%u HID_overflow=%u learning_overflow=%u HID_dropped_output=%u HID_enqueued=%u HID_refused=%u\n",
		relay_b_arm_cycles_max, rx_max,
		rx_packets ? (uint32_t)(rx_cycles / rx_packets) : 0,
		rx_packets, relay_hid_overflow, relay_learning_overflow, relay_hid_dropped_output, relay_hid_enqueued, relay_hid_refused);

#endif
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	printk("relay: diagnostics_overflow=%u\n", tdma_diag_overflow);
#endif
	printk("relay: v3 requested=%s eligible=%u\n", snapshot.v3_requested ? "on" : "off",
		relay_coord_v3_ready(&snapshot, members));
	printk("relay: repair seen=%u invalid=%u phase=%u slot=%u record=%u\n",
	relay_rx_stats.repair.seen, relay_rx_stats.repair.invalid, relay_rx_stats.repair.phase, relay_rx_stats.repair.slot, relay_rx_stats.repair.record);
	if (relay_profile) {
		for (uint8_t child = 0; child < RC_MAX; child++) {
			if (!(members & BIT(child))) {
				continue;
			}
			printk("  repair child=%u received=%u duplicate=%u obsolete=%u age0=%u age1=%u age2=%u\n",
	child, relay_rx_stats.repair.received[child], relay_rx_stats.repair.duplicate[child], relay_rx_stats.repair.obsolete[child],
	relay_rx_stats.repair.age[child][0], relay_rx_stats.repair.age[child][1], relay_rx_stats.repair.age[child][2]);
			printk("  v3 child=%u", child);
			for (unsigned k = 0; k < 2; k++) {
				bool valid = (k ? control.backup_valid : control.primary_valid) & BIT(child);
				uint8_t h = (control.helpers[child] >> (4U * k)) & 15U;
				const struct rc_peer *p = &snapshot.peer[h][child];
				uint16_t ratio = relay_coord_peer_ratio(p);
				printk(" %s=%d ratio=%u rssi=-%u expected=%u state=%s", k ? "backup" : "primary",
					valid ? h : -1, valid ? ratio : 0, valid ? p->rssi_x8 / 8U : 0,
					valid ? p->expected : 0, !valid ? "none(no coverage)" :
					(p->expected >= 120 && ratio >= RC_HEAD_MIN_PERMILLE ? "proven" : "provisional"));
			}
			printk("\n");
			if (!(control.primary_valid & BIT(child)) || !(control.backup_valid & BIT(child))) {
				uint16_t uplink = 0, topology = 0, negative = 0, capacity = 0, eligible = 0;
				for (unsigned h = 0; h < RC_MAX; h++) {
					if (!(members & BIT(h))) continue;
					if (snapshot.t[h].direct_permille < RC_HEAD_MIN_PERMILLE) uplink |= BIT(h);
					if (!relay_can_overhear(members, h, child)) topology |= BIT(h);
					const struct rc_peer *p = &snapshot.peer[h][child];
					if (p->expected >= 30U && !p->heard) negative |= BIT(h);
					unsigned load = 0;
					for (unsigned c = 0; c < RC_MAX; c++)
						for (unsigned k = 0; k < 2; k++)
							load += snapshot.choice[c][k].valid && snapshot.choice[c][k].helper == h;
					if (load >= RELAY_MAX_CHILDREN) capacity |= BIT(h);
				}
				eligible = members & ~(uplink | topology | negative);
				printk("  coverage child=%u eligible=0x%04x reject_uplink=0x%04x topology=0x%04x negative=0x%04x full=0x%04x\n",
					child, eligible, uplink, topology, negative, capacity);
			}
		}
	}
	esb_relay_print_pool();

	printk("relay: bundles=%u invalid=%u records=%u accepted=%u blocked=%u block=%d@%u%%\n",
		relay_rx_stats.bundles, relay_rx_stats.bundle_invalid, relay_rx_stats.records,
		relay_rx_stats.records_accepted, relay_rx_stats.blocked,
		block_id == 0xFF ? -1 : block_id, block_pct);
	printk("relay: block_mask=0x%04x remaining_ms=%u\n",
		block_left > 0 ? block_mask : 0, block_mask && block_left > 0 ? (unsigned)block_left : 0);

	for (uint8_t i = 0; i < RC_MAX && i < stored_trackers; i++) {
		const struct rc_tracker *t = &snapshot.t[i];
		if (!t->seen && !t->capable) {
			continue;
		}
		const char *role = t->role == RC_ROLE_HEAD ? "head" : (t->role == RC_ROLE_CHILD ? "child" : "-");
		printk("  trk %2u cap=%u role=%-5s direct=%3u.%u%% rssi=-%u", i, t->capable, role,
			t->direct_permille / 10, t->direct_permille % 10, t->rssi);
		if (t->role == RC_ROLE_CHILD) {
			printk(" head=%u delivered=%u", t->head, t->delivered);
		} else if (t->role == RC_ROLE_HEAD) {
			printk(" children=0x%04x fwd_slot=%u", t->children,
				relay_coord_forward_slot(&snapshot, i, members));
		}
		printk("\n");
	}
	uint32_t now = k_uptime_get_32();
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		uint8_t d[RELAY_DIAG_LEN];
		unsigned int diag_key = irq_lock();
		bool valid = relay_diag[i].valid;
		uint32_t age = now - relay_diag[i].rx_ms;
		memcpy(d, relay_diag[i].data, sizeof(d));
		irq_unlock(diag_key);
		if (!valid) {
			continue;
		}
		static const char *const links[] = {"healthy", "degraded", "lost", "?"};
		uint16_t c[RELAY_DIAG_COUNTERS];
		for (unsigned k = 0; k < RELAY_DIAG_COUNTERS; k++) {
			c[k] = (uint16_t)((d[RELAY_DIAG_HEADER_LEN + 2U * k] << 8) | d[RELAY_DIAG_HEADER_LEN + 2U * k + 1U]);
		}
		printk("  diag %2u age=%ums link=%s flags=%s%s%s%s%s epoch=%u/%u slot=%u/%u S=%u children=0x%04x members=0x%04x\n",
			i, age, links[(d[2] >> RELAY_DIAG_LINK_SHIFT) & 3U],
			d[2] & RELAY_DIAG_F_ASSIGNED ? "A" : "-", d[2] & RELAY_DIAG_F_LEASE ? "L" : "-",
			d[2] & RELAY_DIAG_F_EPOCH ? "E" : "-", d[2] & RELAY_DIAG_F_HEAD ? "H" : "-",
			d[2] & RELAY_DIAG_F_LAYOUT ? "T" : "-", d[3], d[4], d[5], d[6], d[7],
			(d[8] << 8) | d[9], (d[10] << 8) | d[11]);
		printk("       windows=%u no_window=%u busy=%u rx_any=%u crc_fail=%u len_fail=%u rssi(fail/cap)=-%u/-%u captures=%u foreign=%u forwarded=%u expired=%u presence=%u/%u lease_exp=%u\n",
			c[RELAY_DIAG_WINDOWS], c[RELAY_DIAG_NO_WINDOW], c[RELAY_DIAG_BUSY],
			c[RELAY_DIAG_RX_ANY], c[RELAY_DIAG_CRC_FAIL], c[RELAY_DIAG_LEN_FAIL],
			c[RELAY_DIAG_RSSI] >> 8, c[RELAY_DIAG_RSSI] & 0xFFU, c[RELAY_DIAG_CAPTURES],
			c[RELAY_DIAG_FOREIGN], c[RELAY_DIAG_FORWARDED], c[RELAY_DIAG_EXPIRED],
			c[RELAY_DIAG_PRESENCE_RX], c[RELAY_DIAG_PRESENCE], c[RELAY_DIAG_LEASE_EXPIRED]);
		printk("       start_late_max=%u ticks lengths(fail/cap)=%u/%u\n",
			c[RELAY_DIAG_START_LATE], c[RELAY_DIAG_LENGTHS] >> 8, c[RELAY_DIAG_LENGTHS] & 0xFFU);
	}
}

/* This is the entire mask-dependent and radio-arm path. Keep its boundary
 * visible to the linked-firmware instruction benchmark. IRQs are held by caller. */
struct relay_broadcast_work {
	struct relay_control *control;
	const struct rc_grant_plan *plan;
	uint8_t *wire;
	struct relay_control_crc_plan crc;
	uint32_t start_event;
	volatile int result;
	uint32_t freeze_rtc;
	uint32_t generation;
};

/* Keep the five-age work out of the static encoder's register lifetime. */
__attribute__((noipa, optimize("O3"))) static __ramfunc void
relay_freeze_demand(struct relay_coord *rc, const struct relay_broadcast_work *w)
{
	struct relay_control *c = w->control;

	for (unsigned age = 3; age < 5; age++) {
		const struct rc_receipt *r = &rc->receipt[w->plan->receipt_index[age]];
		relay_control_missing(c, age) = c->expected & ~(r->frame == c->frame - age ? r->received : 0);
	}
	uint16_t debt = 0;
	for (unsigned age = 0; age < 5; age++) {
		debt |= relay_control_missing(c, age) & w->plan->demand_eligible[age];
	}
	relay_demand_update(&relay_demand_state, c->frame, relay_demand_count(debt), &relay_applied_settings);
	relay_control_set_next2_repairs(c, relay_demand_state.queued[2]);
	relay_coord_finish_frozen(c, w->plan, w->wire);
	relay_control_finish_encode(w->wire, c, w->crc, false);
}

/* Attribute provenance: Round 11 linked model and DWT arm evidence retain the freeze boundary. Whole-thread O3 has only group-level evidence; keep pending isolated A/B. */
__attribute__((noipa, optimize("O3")))
static __ramfunc int relay_freeze_arm(struct relay_coord *rc, const struct relay_broadcast_work *w)
{
#ifdef CONFIG_RELAY_CYCLE_DIAGNOSTICS
	uint32_t cycles = DWT->CYCCNT;
#endif
	struct relay_control *c = w->control;
	if (!relay_profile || relay_epoch != c->epoch || relay_session != c->session) return -ESTALE;
	if (!atomic_cas(&radio_operation_busy, 0, 1)) return -EBUSY;
	int err = esb_prx_broadcast_begin();
	if (err) goto failed;
	for (unsigned age = 0; age < RELAY_V3_CACHE_FRAMES; age++) {
		const struct rc_receipt *r = &rc->receipt[w->plan->receipt_index[age]];
		relay_control_missing(c, age) = c->expected & ~(r->frame == c->frame - age ? r->received : 0);
	}
	/* Arm the immutable radio schedule before encoding the frozen masks.
	 * TIMER delays TXEN by 21 us after the launch edge, then RADIO ramps
	 * for 40 us before START consumes the DMA payload. */
	err = esb_prx_broadcast_arm_event(k_ticks_to_us_floor32(RELAY_V3_TX_OFFSET) - 40U, w->start_event);
#ifdef CONFIG_RELAY_CYCLE_DIAGNOSTICS
	uint32_t armed = DWT->CYCCNT;
#endif
	if (err) goto failed;
#ifdef CONFIG_RELAY_CYCLE_DIAGNOSTICS
	relay_b_arm_cycles_max = MAX(relay_b_arm_cycles_max, armed - cycles);
#endif
	if (c->on_demand) {
		relay_freeze_demand(rc, w);
	} else {
		relay_coord_finish_static_frozen(c, w->plan, w->wire);
		relay_control_finish_static_encode(w->wire, c, w->crc, false);
	}
	__DMB();
#ifdef CONFIG_RELAY_CYCLE_DIAGNOSTICS
	relay_b_encode_cycles_max = MAX(relay_b_encode_cycles_max, DWT->CYCCNT - armed);
#endif
	return 0;
failed:
	atomic_set(&radio_operation_busy, 0);
	return err;
}

/* Construct the second immutable copy before its launch freeze. */
__attribute__((noipa, optimize("O3"))) static __ramfunc void
relay_repeat_encode(uint8_t *out, const uint8_t *first, uint32_t preamble)
{
	static const uint8_t delta[4][256] = {
		{0,   248, 247, 15,  233, 17,  30,  230, 213, 45,  34,  218, 60,  196, 203, 51,  173, 85,  90,  162, 68,  188,
		 179, 75,  120, 128, 143, 119, 145, 105, 102, 158, 93,  165, 170, 82,  180, 76,  67,  187, 136, 112, 127, 135,
		 97,  153, 150, 110, 240, 8,   7,   255, 25,  225, 238, 22,  37,  221, 210, 42,  204, 52,  59,  195, 186, 66,
		 77,  181, 83,  171, 164, 92,  111, 151, 152, 96,  134, 126, 113, 137, 23,  239, 224, 24,  254, 6,   9,   241,
		 194, 58,  53,  205, 43,  211, 220, 36,  231, 31,  16,  232, 14,  246, 249, 1,   50,  202, 197, 61,  219, 35,
		 44,  212, 74,  178, 189, 69,  163, 91,  84,  172, 159, 103, 104, 144, 118, 142, 129, 121, 115, 139, 132, 124,
		 154, 98,  109, 149, 166, 94,  81,  169, 79,  183, 184, 64,  222, 38,  41,  209, 55,  207, 192, 56,  11,  243,
		 252, 4,   226, 26,  21,  237, 46,  214, 217, 33,  199, 63,  48,  200, 251, 3,   12,  244, 18,  234, 229, 29,
		 131, 123, 116, 140, 106, 146, 157, 101, 86,  174, 161, 89,  191, 71,  72,  176, 201, 49,  62,  198, 32,  216,
		 215, 47,  28,  228, 235, 19,  245, 13,  2,   250, 100, 156, 147, 107, 141, 117, 122, 130, 177, 73,  70,  190,
		 88,  160, 175, 87,  148, 108, 99,  155, 125, 133, 138, 114, 65,  185, 182, 78,  168, 80,  95,  167, 57,  193,
		 206, 54,  208, 40,  39,  223, 236, 20,  27,  227, 5,   253, 242, 10},
		{0,   73,  146, 219, 35,  106, 177, 248, 70,  15,  212, 157, 101, 44,  247, 190, 140, 197, 30,  87,  175, 230,
		 61,  116, 202, 131, 88,  17,  233, 160, 123, 50,  31,  86,  141, 196, 60,  117, 174, 231, 89,  16,  203, 130,
		 122, 51,  232, 161, 147, 218, 1,   72,  176, 249, 34,  107, 213, 156, 71,  14,  246, 191, 100, 45,  62,  119,
		 172, 229, 29,  84,  143, 198, 120, 49,  234, 163, 91,  18,  201, 128, 178, 251, 32,  105, 145, 216, 3,   74,
		 244, 189, 102, 47,  215, 158, 69,  12,  33,  104, 179, 250, 2,   75,  144, 217, 103, 46,  245, 188, 68,  13,
		 214, 159, 173, 228, 63,  118, 142, 199, 28,  85,  235, 162, 121, 48,  200, 129, 90,  19,  124, 53,  238, 167,
		 95,  22,  205, 132, 58,  115, 168, 225, 25,  80,  139, 194, 240, 185, 98,  43,  211, 154, 65,  8,   182, 255,
		 36,  109, 149, 220, 7,   78,  99,  42,  241, 184, 64,  9,   210, 155, 37,  108, 183, 254, 6,   79,  148, 221,
		 239, 166, 125, 52,  204, 133, 94,  23,  169, 224, 59,  114, 138, 195, 24,  81,  66,  11,  208, 153, 97,  40,
		 243, 186, 4,   77,  150, 223, 39,  110, 181, 252, 206, 135, 92,  21,  237, 164, 127, 54,  136, 193, 26,  83,
		 171, 226, 57,  112, 93,  20,  207, 134, 126, 55,  236, 165, 27,  82,  137, 192, 56,  113, 170, 227, 209, 152,
		 67,  10,  242, 187, 96,  41,  151, 222, 5,   76,  180, 253, 38,  111},
		{0,   193, 133, 68,  13,  204, 136, 73,  26,  219, 159, 94,  23,  214, 146, 83,  52,  245, 177, 112, 57,  248,
		 188, 125, 46,  239, 171, 106, 35,  226, 166, 103, 104, 169, 237, 44,  101, 164, 224, 33,  114, 179, 247, 54,
		 127, 190, 250, 59,  92,  157, 217, 24,  81,  144, 212, 21,  70,  135, 195, 2,   75,  138, 206, 15,  208, 17,
		 85,  148, 221, 28,  88,  153, 202, 11,  79,  142, 199, 6,   66,  131, 228, 37,  97,  160, 233, 40,  108, 173,
		 254, 63,  123, 186, 243, 50,  118, 183, 184, 121, 61,  252, 181, 116, 48,  241, 162, 99,  39,  230, 175, 110,
		 42,  235, 140, 77,  9,   200, 129, 64,  4,   197, 150, 87,  19,  210, 155, 90,  30,  223, 167, 102, 34,  227,
		 170, 107, 47,  238, 189, 124, 56,  249, 176, 113, 53,  244, 147, 82,  22,  215, 158, 95,  27,  218, 137, 72,
		 12,  205, 132, 69,  1,   192, 207, 14,  74,  139, 194, 3,   71,  134, 213, 20,  80,  145, 216, 25,  93,  156,
		 251, 58,  126, 191, 246, 55,  115, 178, 225, 32,  100, 165, 236, 45,  105, 168, 119, 182, 242, 51,  122, 187,
		 255, 62,  109, 172, 232, 41,  96,  161, 229, 36,  67,  130, 198, 7,   78,  143, 203, 10,  89,  152, 220, 29,
		 84,  149, 209, 16,  31,  222, 154, 91,  18,  211, 151, 86,  5,   196, 128, 65,  8,   201, 141, 76,  43,  234,
		 174, 111, 38,  231, 163, 98,  49,  240, 180, 117, 60,  253, 185, 120},
		{0,   152, 55,  175, 110, 246, 89,  193, 220, 68,  235, 115, 178, 42,  133, 29,  191, 39,  136, 16,  209, 73,
		 230, 126, 99,  251, 84,  204, 13,  149, 58,  162, 121, 225, 78,  214, 23,  143, 32,  184, 165, 61,  146, 10,
		 203, 83,  252, 100, 198, 94,  241, 105, 168, 48,  159, 7,   26,  130, 45,  181, 116, 236, 67,  219, 242, 106,
		 197, 93,  156, 4,   171, 51,  46,  182, 25,  129, 64,  216, 119, 239, 77,  213, 122, 226, 35,  187, 20,  140,
		 145, 9,   166, 62,  255, 103, 200, 80,  139, 19,  188, 36,  229, 125, 210, 74,  87,  207, 96,  248, 57,  161,
		 14,  150, 52,  172, 3,   155, 90,  194, 109, 245, 232, 112, 223, 71,  134, 30,  177, 41,  227, 123, 212, 76,
		 141, 21,  186, 34,  63,  167, 8,   144, 81,  201, 102, 254, 92,  196, 107, 243, 50,  170, 5,   157, 128, 24,
		 183, 47,  238, 118, 217, 65,  154, 2,   173, 53,  244, 108, 195, 91,  70,  222, 113, 233, 40,  176, 31,  135,
		 37,  189, 18,  138, 75,  211, 124, 228, 249, 97,  206, 86,  151, 15,  160, 56,  17,  137, 38,  190, 127, 231,
		 72,  208, 205, 85,  250, 98,  163, 59,  148, 12,  174, 54,  153, 1,   192, 88,  247, 111, 114, 234, 69,  221,
		 28,  132, 43,  179, 104, 240, 95,  199, 6,   158, 49,  169, 180, 44,  131, 27,  218, 66,  237, 117, 215, 79,
		 224, 120, 185, 33,  142, 22,  11,  147, 60,  164, 101, 253, 82,  202}
	};
	uint8_t v = first[71] ^ 69;
	for (unsigned i = 0; i < 4; i++) {
		v ^= delta[i][first[12 + i] ^ (uint8_t)(preamble >> (24U - 8U * i))];
	}
	memcpy(out, first, RELAY_CONTROL_LEN);
	out[0] = RELAY_CONTROL_REPEAT;
	relay_put_be32(out + 12, preamble);
	out[71] = v;
}

__attribute__((noipa, optimize("O3"))) static __ramfunc int relay_repeat_arm(uint32_t event, uint32_t generation)
{
	if (generation != relay_layout_generation) {
		return -ESTALE;
	}
	if (!atomic_cas(&radio_operation_busy, 0, 1)) {
		return -EBUSY;
	}
	int err = esb_prx_broadcast_begin();
	if (!err) {
		err = esb_prx_broadcast_arm_event(k_ticks_to_us_floor32(RELAY_V3_TX_OFFSET) - 40U, event);
	}
	if (err) {
		atomic_set(&radio_operation_busy, 0);
	}
	return err;
}

K_SEM_DEFINE(relay_freeze_done, 0, 1);

static void relay_freeze_callback(int32_t channel, uint64_t expires, void *context)
{
	ARG_UNUSED(channel);
	ARG_UNUSED(expires);
	struct relay_broadcast_work *w = context;
	unsigned polls = 65536;
	while ((int32_t)((NRF_RTC1->COUNTER - w->freeze_rtc) << 8) < 0) {
		if (!--polls) {
			w->result = -ETIMEDOUT;
			k_sem_give(&relay_freeze_done);
			return;
		}
	}
	unsigned key = irq_lock();
	uint32_t lag = (NRF_RTC1->COUNTER - w->freeze_rtc) & 0x00ffffffU;
	int err = relay_freeze_arm(&relay_coord, w);
	if (!err) relay_b_grant_max = MAX(relay_b_grant_max,
		(NRF_RTC1->COUNTER - w->freeze_rtc) & 0x00ffffffU);
	relay_b_entry_late += lag > 0;
	relay_b_entry_max = MAX(relay_b_entry_max, lag);
	relay_b_late += err == -ETIME;
	relay_b_late_max = MAX(relay_b_late_max, lag);
	if (!err) {
		relay_coord_read_grants(w->control, w->wire);
		relay_grants = *w->control;
		if (w->control->on_demand) {
			relay_frame_announce(w->control->frame, w->control->preamble_ticks - RELAY_V3_B_PREAMBLE);
		}
		relay_coord_commit_grants(&relay_coord, w->control, w->plan);
	}
	w->result = err;
	irq_unlock(key);
	k_sem_give(&relay_freeze_done);
}

/* Copy each mutable row/history atomically, then plan with IRQs enabled.
 * Freeze rechecks current receipts; publication rejects a changed network. */
static void relay_repeat_callback(int32_t channel, uint64_t expires, void *context)
{
	ARG_UNUSED(channel);
	ARG_UNUSED(expires);
	struct relay_broadcast_work *w = context;
	unsigned polls = 65536;
	while ((int32_t)((NRF_RTC1->COUNTER - w->freeze_rtc) << 8) < 0) {
		if (!--polls) {
			w->result = -ETIMEDOUT;
			k_sem_give(&relay_freeze_done);
			return;
		}
	}
	unsigned key = irq_lock();
	w->result = relay_repeat_arm(w->start_event, w->generation);
	irq_unlock(key);
	k_sem_give(&relay_freeze_done);
}

static bool relay_broadcast_context_current(const struct relay_control *control, uint32_t origin);
static void relay_broadcast_plan(struct relay_control *control, struct rc_grant_plan *plan, struct esb_payload *payload)
{
	static struct relay_coord snapshot;
	for (unsigned i = 0; i < RC_MAX; i++) {
		unsigned key = irq_lock();
		IRQ_PROBE_BEGIN(plan);
		snapshot.t[i] = relay_coord.t[i];
		memcpy(snapshot.peer[i], relay_coord.peer[i], sizeof(snapshot.peer[i]));
		memcpy(snapshot.choice[i], relay_coord.choice[i], sizeof(snapshot.choice[i]));
		snapshot.maintenance_ms[i] = relay_coord.maintenance_ms[i];
		snapshot.report_ms[i] = relay_coord.report_ms[i];
		IRQ_PROBE_END(plan);
		irq_unlock(key);
	}
	for (unsigned i = 0; i < RELAY_CACHE_MAX_FRAMES; i++) {
		unsigned key = irq_lock();
		IRQ_PROBE_BEGIN(plan);
		snapshot.watch[i] = relay_coord.watch[i];
		snapshot.receipt[i] = relay_coord.receipt[i];
		IRQ_PROBE_END(plan);
		irq_unlock(key);
	}
	unsigned key = irq_lock();
	IRQ_PROBE_BEGIN(plan);
	snapshot.learn_members = relay_coord.learn_members;
	snapshot.maintenance_next = relay_coord.maintenance_next;
	snapshot.report_next = relay_coord.report_next;
	snapshot.survey_next = relay_coord.survey_next;
	IRQ_PROBE_END(plan);
	irq_unlock(key);
	snapshot.cache_frames = relay_coord.cache_frames;
	relay_coord_plan_watch(&snapshot, control);
	relay_coord_plan_grants(&snapshot, control, plan, k_uptime_get_32());
	if (relay_applied_settings.repair_floor < 2U && control->frame % 8U == 0 && control->repair_count >= 2U
		&& plan->maintenance >= 0) {
		plan->urgent = true;
	}
	relay_control_encode(payload->data, control);
}

static bool relay_broadcast_context_current(const struct relay_control *control, uint32_t origin)
{
	return esb_initialized && relay_profile && !esb_pairing
		&& relay_coord_v3_ready(&relay_coord, (uint16_t)tdma_active_mask) && control->session == relay_session
		&& control->epoch == relay_epoch && control->members == (uint16_t)tdma_active_mask && origin == relay_origin;
}

/* B is prepared outside the radio ISR. ESB owns the bounded PRX/TX/PRX turn. */
/* Attribute provenance: Round 11 linked model and DWT arm evidence retain the freeze boundary. Whole-thread O3 has only group-level evidence; keep pending isolated A/B. */
__attribute__((optimize("O3")))
static void relay_broadcast_thread(void)
{
	int rtc_channel = z_nrf_rtc_timer_chan_alloc();
	if (rtc_channel < 0) {
		relay_b_err = rtc_channel;
		LOG_ERR("No RTC launch channel for relay broadcast: %d", relay_b_err);
		return;
	}
	int freeze_channel = z_nrf_rtc_timer_chan_alloc();
	if (freeze_channel < 0) {
		z_nrf_rtc_timer_chan_free(rtc_channel);
		relay_b_err = freeze_channel;
		LOG_ERR("No RTC freeze channel for relay broadcast: %d", relay_b_err);
		return;
	}
	/* This channel only drives PPI. Keep its event latched until the next
	 * exact_set so the SDK can reject a missed deadline even if the RTC ISR
	 * ran before freeze. The channel is owned for this thread's lifetime. */
	(void)z_nrf_rtc_timer_compare_int_lock(rtc_channel);
	uint32_t launch_event = z_nrf_rtc_timer_compare_evt_address_get(rtc_channel);
	uint32_t sent = UINT32_MAX;
	uint8_t epoch = 0;
	for (;;) {
		unsigned key = irq_lock();
		IRQ_PROBE_BEGIN(plan);
		bool active = esb_initialized && relay_profile && !esb_pairing
			&& relay_coord_v3_ready(&relay_coord, (uint16_t)tdma_active_mask);
		uint32_t origin = relay_origin;
		uint32_t now = (uint32_t)k_uptime_ticks();
		uint32_t frame = relay_timeline_active.frame, start;
		unsigned advances = 0;
		while (active && relay_timeline_start(&relay_timeline_active, frame + 1U, &start) && (int32_t)(now - start) >= 0
			   && advances++ < 8U) {
			frame++;
			relay_frame_announce(frame, start);
		}
		uint32_t preamble = relay_frame_start(origin, frame) + RELAY_V3_B_PREAMBLE;
		relay_frame_anchor = frame;
		relay_start_anchor = preamble - RELAY_V3_B_PREAMBLE;
		if (!active || (sent == frame && epoch == relay_epoch)) {
			IRQ_PROBE_END(plan);
			irq_unlock(key);
			k_usleep(200);
			continue;
		}
		/* Plan the watch map early in the frame; only the masks wait for
		 * the freeze. With 11 members planning takes up to ~0.4 ms. */
		int32_t lead = preamble - now;
		uint32_t plan_lead = relay_plan_lead((uint16_t)tdma_active_mask);
#if defined(CONFIG_RELAY_FORCE_FULL_FLEET_PLAN)
		plan_lead = relay_plan_lead(UINT16_MAX);
#endif
		if (lead < 8 || lead > (int32_t)plan_lead) {
			IRQ_PROBE_END(plan);
			irq_unlock(key);
			k_usleep(lead > (int32_t)plan_lead ? k_ticks_to_us_floor32(lead - plan_lead) : 100);
			continue;
		}
		struct relay_control control = {
			.direct_count = RELAY_V3_DIRECT_SLOTS,
			.slot_ticks = RELAY_V3_SLOT_TICKS,
			.tps_max = relay_geometry_current()->tps,
			.epoch = relay_epoch,
			.repair_count = relay_frame_repairs(frame),
			.second_b = relay_applied_settings.second_b,
			.on_demand = relay_applied_settings.repair_floor < relay_applied_settings.max_repairs,
			.session = relay_session,
			.frame = frame,
			.preamble_ticks = preamble,
			.members = (uint16_t)tdma_active_mask,
			.expected = (uint16_t)tdma_active_mask,
			.profile = RELAY_PROFILE_120HZ,
		};
		relay_control_set_next_repairs(&control, relay_frame_repairs(frame + 1U));
		relay_control_set_next2_repairs(&control, relay_frame_repairs(frame + 2U));
		uint32_t t_plan = (uint32_t)k_uptime_ticks();
		/* Sole broadcast-thread ownership; the callback finishes before reuse.
		 * Keep the five-age snapshot off the unchanged 1536-byte stack. */
		static struct rc_grant_plan plan;
		IRQ_PROBE_END(plan);
		irq_unlock(key);
		struct esb_payload payload = {.pipe = 1, .noack = true, .length = RELAY_CONTROL_LEN};
		relay_broadcast_plan(&control, &plan, &payload);
		struct relay_control_crc_plan crc_plan = relay_control_plan_crc(payload.data);
		key = irq_lock();
		IRQ_PROBE_RESTART(plan);
		if (!relay_broadcast_context_current(&control, origin)) {
			IRQ_PROBE_END(plan);
			irq_unlock(key);
			continue;
		}

		/* Helper selection runs on live state between broadcasts. Publish only
		 * this membership input; never overwrite concurrently learned rows. */
		relay_coord.learn_members = control.members;
		uint8_t *wire = esb_prx_broadcast_prepare(&payload);
		struct relay_broadcast_work work = {
			.control = &control,
			.plan = &plan,
			.wire = wire,
			.crc = crc_plan,
			.start_event = launch_event,
			.result = -ETIMEDOUT,
			.generation = relay_layout_generation,
		};
		relay_b_plan_max = MAX(relay_b_plan_max, (uint32_t)k_uptime_ticks() - t_plan);
		uint32_t launch_tick = preamble - RELAY_V3_TX_OFFSET;
		uint64_t plan_ticks = k_uptime_ticks();
		uint64_t launch_rtc = z_nrf_rtc_timer_get_ticks(K_TIMEOUT_ABS_TICKS(
			plan_ticks + (int32_t)(launch_tick - (uint32_t)plan_ticks)));
		int schedule_err = z_nrf_rtc_timer_exact_set(rtc_channel, launch_rtc, NULL, NULL);
		work.freeze_rtc = (uint32_t)(launch_rtc - 1U);
		if (!schedule_err && wire) {
			schedule_err = z_nrf_rtc_timer_exact_set(freeze_channel, launch_rtc - 3U,
				relay_freeze_callback, &work);
		}
		sent = frame;
		epoch = control.epoch;
		IRQ_PROBE_END(plan);
		irq_unlock(key);
		if (schedule_err || !wire) {
			relay_b_failed++;
			relay_b_err = schedule_err ? schedule_err : -EBUSY;
			continue;
		}
		/* Wake the RTC ISR ahead of the unchanged freeze edge so callback
		 * dispatch does not consume the arm interval. Keep the work alive until that callback finishes. */
		k_sem_take(&relay_freeze_done, K_FOREVER);
		int err = work.result;
		if (!err) {
			err = esb_prx_broadcast_wait(k_ticks_to_us_ceil32(RELAY_V3_B_PREP_LEAD_TICKS));
			atomic_set(&radio_operation_busy, 0);
		}
		/* Re-encode only the timestamp/type/CRC outside the frozen path. The
		 * grants and current/next/next2 plan are the first copy's immutable bytes. */
		if (!err && control.second_b) {
			uint32_t repeat = preamble
							+ relay_second_b_offset(
								  control.direct_count,
								  control.repair_count,
								  control.slot_ticks,
								  control.second_b
							);
			relay_repeat_encode(payload.data, work.wire, repeat);
			key = irq_lock();
			bool current = relay_broadcast_context_current(&control, origin);
			wire = current ? esb_prx_broadcast_prepare(&payload) : NULL;
			uint64_t ticks = k_uptime_ticks();
			uint64_t repeat_rtc = z_nrf_rtc_timer_get_ticks(
				K_TIMEOUT_ABS_TICKS(ticks + (int32_t)(repeat - RELAY_V3_TX_OFFSET - (uint32_t)ticks))
			);
			work.freeze_rtc = (uint32_t)(repeat_rtc - 1U);
			work.result = -ETIMEDOUT;
			int schedule = !wire ? -ESTALE : z_nrf_rtc_timer_exact_set(rtc_channel, repeat_rtc, NULL, NULL);
			if (!schedule) {
				schedule = z_nrf_rtc_timer_exact_set(freeze_channel, repeat_rtc - 3U, relay_repeat_callback, &work);
			}
			irq_unlock(key);
			if (!schedule) {
				k_sem_take(&relay_freeze_done, K_FOREVER);
				schedule = work.result;
				if (!schedule) {
					schedule = esb_prx_broadcast_wait(k_ticks_to_us_ceil32(RELAY_V3_B_PREP_LEAD_TICKS));
					atomic_set(&radio_operation_busy, 0);
				}
			}
			relay_b2_sent += schedule == 0;
			relay_b2_failed += schedule != 0;
			relay_b2_err = schedule;
			relay_b_err = schedule;
		}
#ifdef CONFIG_RELAY_FRAME_LEDGER
		if (!err) {
			relay_r_frames[control.repair_count]++;
			for (unsigned ch = 0; ch < 16; ch++) {
				if (control.expected & BIT(ch)) {
					relay_direct_expected[ch]++;
					relay_direct_missing[ch] += !!(control.missing[0] & BIT(ch));
				}
			}
			for (unsigned slot = 0; slot < control.repair_count; slot++) {
				const struct relay_grant *g = &control.grants[slot];
				unsigned kind = g->owner == RELAY_OWNER_MAINTENANCE ? 2 :
					g->owner >= RELAY_MAX_TRACKERS ? 3 : g->children ? 0 : 1;
				relay_pool_reserved[slot][kind]++;
			}
		}
#endif

#ifdef CONFIG_RELAY_FRAME_LEDGER
		ledger_note(
			relay_session,
			control.frame,
			3,
			40,
			control.repair_count,
			relay_control_next_repairs(&control),
			relay_control_next2_repairs(&control)
		);
		ledger_note(relay_session, control.frame, 3, 42, relay_demand_state.quiet, relay_demand_state.target, 0);
		for (unsigned child = 0; child < RELAY_MAX_TRACKERS; child++) {
			if (!ledger_child_enabled(child)) continue;
			uint16_t helpers = 0;
			for (unsigned slot = 0; slot < control.repair_count; slot++) {
				const struct relay_grant *g = &control.grants[slot];
				if (g->owner < RELAY_MAX_TRACKERS && (g->children & BIT(child)))
					helpers |= BIT(g->owner);
			}
			for (unsigned age = 0; age < relay_control_cache_frames(&control); age++) {
				if (!(relay_control_missing(&control, age) & BIT(child))) {
					continue;
				}
				uint32_t captured = control.frame - age;
				ledger_note(relay_session, captured, child, LEDGER_B, err, 255, age);
				/* Record the actual frozen grants, including non-USB helpers.
				 * Planning evidence stays separate from receiver acceptance. */
				ledger_note(relay_session, captured, child, LEDGER_RESCUE_PLAN,
					plan.rescue_child, plan.rescue_helper, age);
				ledger_note(relay_session, captured, child, LEDGER_GRANT_HELPERS,
					helpers, plan.rescue_age, age);
				ledger_note(
					relay_session,
					captured,
					child,
					LEDGER_SURVEY_HISTORY,
					relay_plan_watch_const(&plan, age)->survey,
					relay_plan_watch_const(&plan, age)->helpers[child],
					age
				);
			}
		}
#endif
        relay_b_sent += err == 0;
		relay_b_failed += err != 0;
		relay_b_err = err ? err : relay_b_err;
	}
}
K_THREAD_DEFINE(relay_broadcast_thread_id, 1536, relay_broadcast_thread,
	NULL, NULL, NULL, 1, 0, 0);
#endif
