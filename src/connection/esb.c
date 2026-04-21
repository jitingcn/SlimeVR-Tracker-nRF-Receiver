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

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#include "globals.h"
#include "hid.h"
#include "system/system.h"
#include "data_collect.h"

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

#define RADIO_RETRANSMIT_DELAY CONFIG_RADIO_RETRANSMIT_DELAY
#define RADIO_RF_CHANNEL CONFIG_RADIO_RF_CHANNEL
#define PAIRING_TIMEOUT_SECONDS CONFIG_PAIRING_TIMEOUT

// TDMA parameters — now dynamically computed based on active tracker count.
// The receiver periodically scans for active trackers and packs per-tracker
// TDMA config (slot index, total slots, slot ticks, epoch) into a volatile
// uint32_t that the RADIO ISR reads atomically when constructing PONG packets.
#define TDMA_ENABLED 1
#define TDMA_MIN_SLOT_TICKS    14  /* minimum slot width (~427μs at 32768Hz) */
#define TDMA_MAX_TPS_TARGET   200 /* max TPS per tracker */
#define TDMA_TOLERANCE_TICKS    3  /* slot violation tolerance band */
#define TDMA_RECONFIG_MIN_MS 5000  /* minimum interval between TDMA reconfigurations */

// Dynamic TDMA config packed into uint32_t for atomic ISR access (ARM Cortex-M).
// Layout: [31:24]=assigned_slot, [23:16]=total_slots, [15:8]=slot_ticks, [7:0]=epoch
static volatile uint32_t tdma_config_packed[MAX_TRACKERS];
static uint8_t tdma_config_epoch;           // wrapping counter, incremented on each recalc
static uint8_t tdma_dynamic_active_count;   // current number of active trackers
static uint8_t tdma_dynamic_slot_ticks;     // current slot width in ticks
static uint32_t tdma_active_mask;           // bitmask of active trackers (for change detection)
static int64_t tdma_last_reconfig_time;     // timestamp of last reconfiguration (0 = never)

static inline uint32_t tdma_pack_config(uint8_t slot, uint8_t total, uint8_t slot_ticks, uint8_t epoch)
{
	return ((uint32_t)slot << 24) | ((uint32_t)total << 16)
	     | ((uint32_t)slot_ticks << 8) | (uint32_t)epoch;
}

/* Forward declaration — defined after last_ping_time / PING_TIMEOUT_MS */
static void tdma_recalculate(void);

static struct esb_payload rx_payload;

struct pairing_event {
	uint8_t packet[8];
};

// Queue pairing packets from the ISR context to the pairing worker thread.
K_MSGQ_DEFINE(esb_pairing_msgq, sizeof(struct pairing_event), 8, 4);

static K_MUTEX_DEFINE(tracker_store_lock);

static uint8_t last_packet_sequence[MAX_TRACKERS];    // Track the last packet sequence for each tracker
static uint8_t last_ping_counter[MAX_TRACKERS] = {0}; // Track the last PING counter for each tracker
static bool ping_counter_initialized[MAX_TRACKERS]
	= {false};                                      // Flag indicating if a PING has been received from this tracker
static uint64_t last_ping_time[MAX_TRACKERS] = {0}; // Track the last time a PING was received from each tracker
static uint8_t last_pong_queued_counter[MAX_TRACKERS] = {0}; // Track the last PONG counter enqueued for each tracker
static uint8_t packet_count[MAX_TRACKERS] = {0};             // Packet count received from each tracker
// Shared ACK state: written by threads/event_handler, read by ack_handler (radio ISR).
// On single-core Cortex-M, volatile ensures visibility between ISR priorities.
static volatile uint8_t tracker_remote_command[MAX_TRACKERS]; // Command flag for next PONG
static volatile uint32_t tracker_channel_value;               // Channel value for SET_CHANNEL command
static volatile int16_t pending_sens_data[MAX_TRACKERS][3];   // SENS_SET sensitivity data
static uint8_t receiver_rf_channel = 0xFF; // Current RF channel of the receiver, 0xFF indicates using default value

#define PING_TIMEOUT_MS 5000 // PING timeout threshold: 5 seconds
#define REMOTE_COMMAND_ACTIVE_SCAN_MS 1000 // Time window to detect trackers actively sending data

/**
 * Recalculate dynamic TDMA parameters based on active trackers.
 * Called periodically from esb_stats_thread (non-ISR context).
 */
static void tdma_recalculate(void)
{
	uint64_t now = k_uptime_get();
	uint8_t active_ids[MAX_TRACKERS];
	uint8_t active_count = 0;
	uint32_t new_mask = 0;

	/* Scan for active trackers (received PING within timeout) */
	for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; i++) {
		if (last_ping_time[i] > 0 &&
		    (now - last_ping_time[i]) <= PING_TIMEOUT_MS) {
			active_ids[active_count++] = i;
			new_mask |= (1U << i);
		}
	}

	/* Skip update if active set hasn't changed */
	if (new_mask == tdma_active_mask && active_count == tdma_dynamic_active_count) {
		return;
	}

	/*
	 * Rate-limit reconfiguration when trackers *disappear* to avoid
	 * flapping from brief PING gaps.  New trackers should get slots
	 * immediately so they stop transmitting in ALOHA mode ASAP.
	 */
	bool new_trackers_appeared = (new_mask & ~tdma_active_mask) != 0;
	if (!new_trackers_appeared && tdma_last_reconfig_time > 0 &&
	    (now - tdma_last_reconfig_time) < TDMA_RECONFIG_MIN_MS) {
		return;
	}

	/* active_ids is already sorted (ascending) since we iterate 0..N */

	uint8_t slot_ticks;
	if (active_count <= 1) {
		slot_ticks = TDMA_MIN_SLOT_TICKS;
		if (active_count == 1) {
			/* slot_ticks = 32768/200 = 163, but cap to keep frame short */
			slot_ticks = 163;
		}
	} else {
		/* slot_ticks = ceil(32768 / (200 * active_count)) */
		uint32_t numerator = 32768;
		uint32_t denominator = TDMA_MAX_TPS_TARGET * active_count;
		slot_ticks = (uint8_t)((numerator + denominator - 1) / denominator);
		if (slot_ticks < TDMA_MIN_SLOT_TICKS) {
			slot_ticks = TDMA_MIN_SLOT_TICKS;
		}
	}

	tdma_config_epoch++;
	uint8_t epoch = tdma_config_epoch;

	/* Pack config for each active tracker */
	for (uint8_t s = 0; s < active_count; s++) {
		uint8_t tid = active_ids[s];
		tdma_config_packed[tid] = tdma_pack_config(s, active_count, slot_ticks, epoch);
	}

	/* Mark inactive trackers as unassigned */
	for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
		if (!(new_mask & (1U << i))) {
			tdma_config_packed[i] = tdma_pack_config(0xFF, active_count, slot_ticks, epoch);
		}
	}

	uint8_t old_count = tdma_dynamic_active_count;
	uint8_t old_slot = tdma_dynamic_slot_ticks;
	tdma_dynamic_active_count = active_count;
	tdma_dynamic_slot_ticks = slot_ticks;
	tdma_active_mask = new_mask;
	tdma_last_reconfig_time = now;

	if (active_count != old_count || slot_ticks != old_slot) {
		uint32_t frame_ticks = (uint32_t)slot_ticks * active_count;
		uint32_t est_tps = frame_ticks > 0 ? 32768 / frame_ticks : 0;
		LOG_INF("TDMA reconfig: %u active, slot=%u ticks, frame=%u ticks, ~%u TPS/trk (epoch=%u)",
			active_count, slot_ticks, frame_ticks, est_tps, epoch);
	}
}

// Channel change confirmation tracking
static atomic_t channel_change_pending = ATOMIC_INIT(0); // Indicates if a channel change is pending
static uint8_t pending_channel = 0;                      // The channel value to switch to
static atomic_t channel_ack_mask
	= ATOMIC_INIT(0);                      // Bitmask to track which trackers have acknowledged the channel change
static int64_t channel_change_timeout = 0; // Timestamp for channel change timeout
#define CHANNEL_CHANGE_TIMEOUT_MS 30000    // Timeout duration for waiting for all trackers to acknowledge

// Pairing state (declared here for ISR access in event_handler)
static bool esb_pairing = false;
static bool esb_paired = false;
static bool esb_clearing = false;

// Pairing timeout and target count
static int64_t pairing_start_time = 0;
static uint8_t pairing_target_count = 0; // 0 = no limit, >0 = exit after N new devices
static uint8_t pairing_initial_count = 0; // Number of devices when pairing started
static int64_t pairing_target_reached_time = 0; // Time when target count was reached (for delayed exit)
static volatile bool pairing_new_devices_blocked = false; // When true, only allow re-pairing of known devices
#define PAIRING_EXIT_DELAY_MS 15000 // Delay before exiting after target count reached

// Cached receiver device address (set once at init, used by ISR for pairing responses)
static uint8_t receiver_device_addr[6] = {0};

// NVS async write infrastructure
struct nvs_write_request {
	uint16_t id;
	uint8_t len;
	uint8_t data[8]; // large enough for uint64_t
};

K_MSGQ_DEFINE(nvs_write_msgq, sizeof(struct nvs_write_request), 20, 4);

static void nvs_writer_thread(void);

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
#define STATS_PRINT_INTERVAL_MS 1000     // Print detailed statistics every 1 second (when enabled)
#define TPS_CALCULATION_INTERVAL_MS 1000 // Calculate TPS every second
#define TPS_MONITOR_INTERVAL_MS 500

// Statistics display control
static volatile bool stats_detailed_enabled = false;        // Whether detailed stats are enabled
static volatile int64_t stats_detailed_end_time = 0;        // Timestamp when detailed stats should auto-disable (0 = no auto-disable)
static int64_t last_tps_print_time = 0;                     // Last time total TPS was printed

// TDMA Statistics
struct tdma_stats {
	int64_t sum_offset;      // Sum of center offsets in ticks
	uint64_t sum_sq_offset;  // Sum of squared offsets for variance
	int32_t min_offset;      // Minimum offset seen
	int32_t max_offset;      // Maximum offset seen
	uint32_t count;          // Packet count
	uint32_t violations;     // Slot violation count
	int64_t last_log_time;   // Last log output time
};

static struct tdma_stats g_tdma_stats[MAX_TRACKERS] = {0};

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

#if TDMA_ENABLED
/**
 * Check if a packet from a tracker arrived in its assigned TDMA slot.
 * Uses dynamic parameters from tdma_config_packed[].
 *
 * @param tracker_id   The tracker's ID (0-15)
 * @param rx_ticks     Receiver time in ticks when packet arrived
 */
static void tdma_check_slot(uint8_t tracker_id, uint32_t rx_ticks)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}

	/* Read dynamic TDMA config (atomic 32-bit load) */
	uint32_t cfg = tdma_config_packed[tracker_id];
	uint8_t expected_slot = (cfg >> 24) & 0xFF;
	uint8_t total_slots   = (cfg >> 16) & 0xFF;
	uint8_t slot_ticks    = (cfg >> 8) & 0xFF;

	/* Skip validation if tracker has no slot assignment or config invalid */
	if (expected_slot == 0xFF || total_slots == 0 || slot_ticks == 0) {
		return;
	}

	uint32_t frame_ticks = (uint32_t)slot_ticks * total_slots;

	/*
	 * Compensate receiver clock vs tracker clock offset.
	 */
	int32_t clock_bias = g_last_ping_rx_time_diff_valid[tracker_id]
		? g_last_ping_rx_time_diff_ticks[tracker_id]
		: 0;
	uint32_t adjusted_rx_ticks = (uint32_t)((int32_t)rx_ticks - clock_bias);

	/* Calculate current position in the TDMA frame (tracker-relative) */
	uint32_t frame_phase = adjusted_rx_ticks % frame_ticks;

	/*
	 * Offset relative to the CENTER of this tracker's assigned slot.
	 * Normalize to [-frame_ticks/2, frame_ticks/2] for wrap-around.
	 */
	int32_t slot_center = (int32_t)(expected_slot * slot_ticks + slot_ticks / 2);
	int32_t ref_offset = (int32_t)frame_phase - slot_center;

	if (ref_offset > (int32_t)(frame_ticks / 2)) {
		ref_offset -= frame_ticks;
	} else if (ref_offset < -(int32_t)(frame_ticks / 2)) {
		ref_offset += frame_ticks;
	}

	/* Update statistics */
	struct tdma_stats *stats = &g_tdma_stats[tracker_id];
	stats->count++;
	stats->sum_offset += ref_offset;
	stats->sum_sq_offset += ((int64_t)ref_offset * ref_offset);

	if (stats->count == 1) {
		stats->min_offset = ref_offset;
		stats->max_offset = ref_offset;
	} else {
		if (ref_offset < stats->min_offset) {
			stats->min_offset = ref_offset;
		}
		if (ref_offset > stats->max_offset) {
			stats->max_offset = ref_offset;
		}
	}

	int32_t half_slot = (int32_t)(slot_ticks / 2);
	int32_t abs_offset = ref_offset < 0 ? -ref_offset : ref_offset;
	if (abs_offset > half_slot + TDMA_TOLERANCE_TICKS) {
		stats->violations++;

		if ((stats->violations & 0x0F) == 1) {
			LOG_DBG(
				"TDMA viol trk=%u exp_slot=%u/%u phase=%u off=%d bias=%d",
				tracker_id,
				expected_slot,
				total_slots,
				frame_phase,
				ref_offset,
				clock_bias
			);
		}
	}
}
#else
/* When TDMA is disabled, provide a no-op stub */
static inline void tdma_check_slot(uint8_t tracker_id, uint32_t rx_ticks)
{
	ARG_UNUSED(tracker_id);
	ARG_UNUSED(rx_ticks);
}
#endif

static void esb_stats_thread(void);
K_THREAD_DEFINE(esb_stats_thread_id, 512, esb_stats_thread, NULL, NULL, NULL, 8, 0, 0);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 1024, esb_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(nvs_writer_thread_id, 1024, nvs_writer_thread, NULL, NULL, NULL, 9, 0, 0);

static bool esb_parse_pair(const uint8_t packet[8]);
static void process_pairing_queue(void);

// Find tracker by address without locks.
// On single-core ARM Cortex-M, compiler barrier ensures correct ordering.
// stored_tracker_addr[] is append-only from ISR perspective — thread writes
// the address first, then increments count with a compiler barrier in between.
static inline int esb_find_tracker(uint64_t addr)
{
	if (addr == 0) {
		return -1;
	}
	uint8_t count = stored_trackers; // single atomic byte read on ARM
	__asm__ volatile("" ::: "memory"); // compiler barrier: ensure count is read before array
	for (int i = 0; i < count && i < MAX_TRACKERS; i++) {
		if (stored_tracker_addr[i] == addr) {
			return i;
		}
	}
	return -1;
}

// Queue an NVS write for async processing (thread-safe, non-blocking)
static inline void nvs_write_async(uint16_t id, const void *data, size_t len)
{
	struct nvs_write_request req = {.id = id, .len = (uint8_t)MIN(len, sizeof(req.data))};
	memcpy(req.data, data, req.len);
	if (k_msgq_put(&nvs_write_msgq, &req, K_NO_WAIT) != 0) {
		LOG_WRN("NVS write queue full, dropping write for id %u", id);
	}
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
	if (packet_count[tracker_id] == 0) {
		// Update received packet count
		stats->total_received++;
		last_packet_sequence[tracker_id] = received_seq;
		packet_count[tracker_id] = 1;
		stats->normal_packets++;
		stats->last_sequence = received_seq;
		stats->first_packet = false;
		stats->status_received++;
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
		packet_count[tracker_id]++;
		stats->normal_packets++;
		stats->last_sequence = received_seq;
		stats->status_received++;
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
		last_packet_sequence[tracker_id] = received_seq;
		packet_count[tracker_id]++;
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
		last_packet_sequence[tracker_id] = received_seq;
		packet_count[tracker_id]++;
		stats->last_sequence = received_seq;
		// Log single gap only at DEBUG level to reduce output (summary statistics will show total gaps)
		LOG_DBG(
			"Gap detected: tracker=%d, seq=%d, gap=%d (forward=%d)",
			tracker_id,
			received_seq,
			gaps,
			diff_forward
		);
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

	while (1) {
		k_msleep(TPS_MONITOR_INTERVAL_MS);

		uint64_t now = (uint64_t)k_uptime_get();

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

			/* Recalculate dynamic TDMA config every TPS print cycle (~1s) */
			tdma_recalculate();
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
#define RAW_ARQ_MARKER   0xAA
/* Maximum sequence distance before a gap is considered stale and unrecoverable.
 * Must be <= tracker's ring buffer size (RAW_RING_SIZE = 512). */
#define RAW_ARQ_STALE_DISTANCE 400

static volatile uint16_t raw_arq_expected_seq;
static volatile bool     raw_arq_seq_initialized;
static volatile uint16_t raw_arq_gap_queue[RAW_ARQ_MAX_GAPS];
static volatile uint8_t  raw_arq_gap_count;
/* Counters for diagnostics (read by event_handler, not ISR-critical) */
static volatile uint32_t raw_arq_gaps_detected;
static volatile uint32_t raw_arq_retransmits_received;

/* Called from esb_ack_handler_cb (ISR context) when raw data packet arrives.
 * Updates gap queue and builds ACK payload with retransmit requests. */
static void raw_arq_process_isr(uint16_t received_seq,
				struct esb_payload *ack_payload,
				bool *has_ack_payload)
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
		if (to_add > remaining) to_add = remaining;
		for (uint16_t i = 0; i < to_add; i++) {
			raw_arq_gap_queue[raw_arq_gap_count++] =
				(uint16_t)(expected + i);
		}
		raw_arq_gaps_detected += (uint32_t)diff;
		raw_arq_expected_seq = received_seq + 1;
	} else if (diff < 0 && diff > -100) {
		/* Retransmitted (out-of-order) packet: remove from gap queue */
		for (uint8_t i = 0; i < raw_arq_gap_count; i++) {
			if (raw_arq_gap_queue[i] == received_seq) {
				/* Shift remaining entries */
				for (uint8_t j = i; j + 1 < raw_arq_gap_count; j++) {
					raw_arq_gap_queue[j] = raw_arq_gap_queue[j + 1];
				}
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
			sys_put_be16(raw_arq_gap_queue[i],
				     &ack_payload->data[2 + i * 2]);
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
static void esb_ack_handler_cb(const uint8_t *pdu_data, uint8_t data_length,
				uint32_t pipe_id, struct esb_payload *ack_payload,
				bool *has_ack_payload, bool *suppress_ack)
{
	*has_ack_payload = false;
	*suppress_ack = false;

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

		uint32_t rx_ticks = k_uptime_ticks();
		/* Save accurate RADIO ISR timestamp for clock_bias computation in event_handler */
		g_ping_isr_rx_ticks[tracker_id] = rx_ticks;
		g_ping_isr_rx_ticks_valid[tracker_id] = true;

		uint8_t counter = pdu_data[2];
		uint8_t cmd = tracker_remote_command[tracker_id];

		/* In data collection mode, force SHUTDOWN for non-target trackers */
		if (data_collect_is_active() &&
		    !data_collect_is_target(tracker_id)) {
			cmd = ESB_PONG_FLAG_SHUTDOWN;
		}

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
		} else {
			/* Normal: embed receiver timestamp for time sync */
			ack_payload->data[3] = (rx_ticks >> 24) & 0xFF;
			ack_payload->data[4] = (rx_ticks >> 16) & 0xFF;
			ack_payload->data[5] = (rx_ticks >> 8) & 0xFF;
			ack_payload->data[6] = (rx_ticks) & 0xFF;

			if (cmd == ESB_PONG_FLAG_SET_CHANNEL) {
				uint32_t ch = tracker_channel_value;
				ack_payload->data[8] = (ch >> 24) & 0xFF;
				ack_payload->data[9] = (ch >> 16) & 0xFF;
				ack_payload->data[10] = (ch >> 8) & 0xFF;
				ack_payload->data[11] = (ch) & 0xFF;
			} else if (cmd == ESB_PONG_FLAG_NORMAL) {
				/* Piggyback dynamic TDMA config in bytes 8-11.
				 * Atomic 32-bit read on ARM Cortex-M — ISR safe. */
				uint32_t cfg = tdma_config_packed[tracker_id];
				ack_payload->data[8]  = (cfg >> 24) & 0xFF; /* assigned_slot */
				ack_payload->data[9]  = (cfg >> 16) & 0xFF; /* total_slots */
				ack_payload->data[10] = (cfg >> 8) & 0xFF;  /* slot_ticks */
				ack_payload->data[11] = (cfg) & 0xFF;       /* config_epoch */
			} else {
				memset(&ack_payload->data[8], 0, 4);
			}
		}

		ack_payload->data[ESB_PONG_LEN - 1] =
			crc8_ccitt(0x07, ack_payload->data, ESB_PONG_LEN - 1);
		*has_ack_payload = true;
		return;
	}

	/* Other packet types: no ACK payload, handled by event_handler */

	/* ---- Raw data ARQ (type 0x10, data collection active) ---- */
	if (data_length >= 4 && pdu_data[0] == ESB_RAW_IMU_TYPE) {
		uint8_t tracker_id = pdu_data[1];
		if (data_collect_is_active() &&
		    data_collect_is_target(tracker_id)) {
			uint16_t seq = sys_get_be16(&pdu_data[2]);
			raw_arq_process_isr(seq, ack_payload, has_ack_payload);
			if (*has_ack_payload) {
				ack_payload->pipe = pipe_id;
			}
		}
	}
}

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		// TX success - no action needed
		break;

	case ESB_EVENT_TX_FAILED:
		// TX failed - log at debug level
		LOG_DBG("TX FAILED (attempts=%u)", event->tx_attempts);
		break;
	case ESB_EVENT_RX_RECEIVED: {
		int err = 0;
		uint32_t current_rx_ticks = k_uptime_ticks();
		while (!err) {
			err = esb_read_rx_payload(&rx_payload);
			if (err == -ENODATA) {
				break;
			} else if (err) {
				LOG_ERR("Error while reading rx packet: %d", err);
				break;
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
				LOG_DBG(
					"Received PING type=%u id=%u ctr=%u",
					rx_payload.data[0],
					rx_payload.data[1],
					rx_payload.data[2]
				);

				// TDMA Slot Check for PING
				uint8_t tracker_id = rx_payload.data[1];

				// Parse new PING fields (added in protocol update)
				uint32_t expected_rx_ticks = ((uint32_t)rx_payload.data[3] << 24) | ((uint32_t)rx_payload.data[4] << 16)
										   | ((uint32_t)rx_payload.data[5] << 8) | ((uint32_t)rx_payload.data[6]);

				if (tracker_id < MAX_TRACKERS) {
					// Update stats
					struct tdma_stats *stats = &g_tdma_stats[tracker_id];

					/* Use RADIO ISR timestamp for clock_bias: avoids EVENT_IRQ scheduling
					 * jitter (10-25 ticks) that would inflate bias and cause false violations. */
					uint32_t isr_rx_ticks = g_ping_isr_rx_ticks_valid[tracker_id]
						? g_ping_isr_rx_ticks[tracker_id] : current_rx_ticks;
					int32_t rx_time_diff_ticks = (int32_t)(isr_rx_ticks - expected_rx_ticks);

					/*
					 * Rate-limited clock_bias update.
					 *
					 * PING retransmissions inflate rx_time_diff by 16-60+ ticks per
					 * retry (ESB retransmit cycle ≈ 500-600 µs ≈ 16-20 ticks).
					 * This inflated bias causes ALL data packets in the following
					 * period to appear early (large negative Mean offset).
					 *
					 * Since retransmissions only INCREASE the measurement:
					 * - Decreases: adopt immediately (clean PING measurement)
					 * - Increases: cap at 2 ticks per PING period
					 *
					 * Real clock drift is <0.1 tick/second, so 2 ticks/PING easily
					 * tracks legitimate drift while rejecting retransmission spikes
					 * (a 400-tick spike only moves bias by 2 instead of 50).
					 */
					#define CLOCK_BIAS_MAX_UP_PER_PING 2
					if (!g_last_ping_rx_time_diff_valid[tracker_id]) {
						g_last_ping_rx_time_diff_ticks[tracker_id] = rx_time_diff_ticks;
					} else {
						int32_t prev = g_last_ping_rx_time_diff_ticks[tracker_id];
						int32_t diff = rx_time_diff_ticks - prev;
						if (diff <= 0) {
							/* Decrease: adopt immediately (clean measurement) */
							g_last_ping_rx_time_diff_ticks[tracker_id] = rx_time_diff_ticks;
						} else if (diff <= CLOCK_BIAS_MAX_UP_PER_PING) {
							/* Small increase within rate limit */
							g_last_ping_rx_time_diff_ticks[tracker_id] = rx_time_diff_ticks;
						} else {
							/* Large increase: clamp to rate limit */
							g_last_ping_rx_time_diff_ticks[tracker_id] = prev + CLOCK_BIAS_MAX_UP_PER_PING;
						}
					}
					g_last_ping_rx_time_diff_valid[tracker_id] = true;

					uint64_t rx_time_diff_us
						= k_ticks_to_us_floor64((rx_time_diff_ticks < 0 ? -rx_time_diff_ticks : rx_time_diff_ticks));
#if !TDMA_ENABLED
					ARG_UNUSED(rx_time_diff_us);
#endif
					// Log periodically
					int64_t now_ms = k_uptime_get();
					if (stats->count > 0) {
						int64_t mean = stats->sum_offset / stats->count;
						uint64_t variance = (stats->sum_sq_offset / stats->count) - (mean * mean);
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

#if TDMA_ENABLED
						if (stats->violations > 12 && esb_get_stats_detailed_enabled()) {
							LOG_WRN(
								"TDMA Stats ID=%u Count=%u Viol=%u Mean=%lld ticks StdDev=%u ticks Range=[%d, %d] "
								"RxTimeDiff=%s%llu us",
								tracker_id,
								stats->count,
								stats->violations,
								mean,
								std_dev,
								stats->min_offset,
								stats->max_offset,
								rx_time_diff_ticks >= 0 ? "+" : "-",
								rx_time_diff_us
							);
						} else {
							LOG_DBG(
								"TDMA Stats ID=%u Count=%u Viol=%u Mean=%lld ticks StdDev=%u ticks Range=[%d, %d] "
								"RxTimeDiff=%s%llu us",
								tracker_id,
								stats->count,
								stats->violations,
								mean,
								std_dev,
								stats->min_offset,
								stats->max_offset,
								rx_time_diff_ticks >= 0 ? "+" : "-",
								rx_time_diff_us
							);
						}
#else
						ARG_UNUSED(rx_time_diff_ticks);
#endif
					}

					// Reset stats
					memset(stats, 0, sizeof(struct tdma_stats));
					stats->last_log_time = now_ms;
				}

				// Check for PING control packet and respond with PONG
				if (rx_payload.data[0] == ESB_PING_TYPE) {
					uint8_t tracker_id = rx_payload.data[1];
					uint8_t counter = rx_payload.data[2];

					uint8_t ping_ack_flag = rx_payload.data[7];

					if (rx_payload.pipe != 1 + (tracker_id % 7)) {
						static uint8_t pipe_mismatch_count[MAX_TRACKERS] = {0};
						pipe_mismatch_count[tracker_id]++;

						if (pipe_mismatch_count[tracker_id] % 10 == 1) {
							LOG_WRN(
								"PING pipe mismatch (x%u): id=%u, expected "
								"pipe=%u got pipe=%u",
								pipe_mismatch_count[tracker_id],
								tracker_id,
								1 + (tracker_id % 7),
								rx_payload.pipe
							);
						} else {
							LOG_DBG(
								"PING pipe mismatch: id=%u, expected pipe=%u "
								"got pipe=%u",
								tracker_id,
								1 + (tracker_id % 7),
								rx_payload.pipe
							);
						}
					}

					// check crc for PING
					uint8_t crc_calc = crc8_ccitt(0x07, rx_payload.data, ESB_PING_LEN - 1);
					if (rx_payload.data[ESB_PING_LEN - 1] != crc_calc) {
						// CRC error - only log warning if consecutive errors occur
						static uint8_t crc_error_count[MAX_TRACKERS] = {0};
						crc_error_count[tracker_id]++;

						if (crc_error_count[tracker_id] % 5 == 1) {
							// Log every 5th error
							LOG_WRN(
								"PING CRC mismatch (x%u): id=%u expected %02X "
								"got %02X",
								crc_error_count[tracker_id],
								tracker_id,
								crc_calc,
								rx_payload.data[ESB_PING_LEN - 1]
							);
						} else {
							LOG_DBG(
								"PING CRC mismatch: id=%u expected %02X got "
								"%02X",
								tracker_id,
								crc_calc,
								rx_payload.data[ESB_PING_LEN - 1]
							);
						}
						break;
					}

					uint32_t tracker_estimated_server_ticks
						= ((uint32_t)rx_payload.data[3] << 24) | ((uint32_t)rx_payload.data[4] << 16)
						| ((uint32_t)rx_payload.data[5] << 8) | ((uint32_t)rx_payload.data[6]);
					// Calculate signed ticks difference
					int32_t ticks_diff = (int32_t)current_rx_ticks - (int32_t)tracker_estimated_server_ticks;
					// Get absolute value for conversion to microseconds
					uint32_t ticks_diff_abs = (uint32_t)(ticks_diff < 0 ? -ticks_diff : ticks_diff);
					LOG_DBG(
						"Tracker %u PING ctr=%u ticks_offset=%s%d us",
						tracker_id,
						counter,
						ticks_diff >= 0 ? "+" : "-",          // Add sign to the log
						k_ticks_to_us_floor32(ticks_diff_abs) // Convert absolute tick difference to microseconds
					);

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
							LOG_INF(
								"PING timeout (%llu ms), resetting tracker %u counter tracking",
								current_time - last_ping_time[tracker_id],
								tracker_id
							);
							last_ping_counter[tracker_id] = counter;
							last_ping_time[tracker_id] = current_time;
							last_pong_queued_counter[tracker_id] = 0xFF;
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
									LOG_INF(
										"Tracker restart detected: id=%u old_ctr=%u new_ctr=%u (backward=%d)",
										tracker_id,
										last_ping_counter[tracker_id],
										counter,
										backward_amount
									);
								} else {
									// Large backward step but counter not small - long packet loss + wrap-around
									is_large_gap = true;
									LOG_WRN(
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
							// Continue processing this PING, send PONG
						} else if (is_duplicate) {
							// Same counter as last time - likely a retransmit
							LOG_DBG("Duplicate PING detected: id=%u ctr=%u (retransmit or retry)", tracker_id, counter);

							// Check if we already queued a PONG for this counter
							if (counter == last_pong_queued_counter[tracker_id]) {
								LOG_DBG("PONG already queued for ctr=%u, skipping duplicate queue", counter);
								// Don't queue again, tracker will get PONG on next packet
								break;
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
							break;
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

					if (ping_ack_flag != ESB_PONG_FLAG_NORMAL) {
						if (tracker_remote_command[tracker_id] == ping_ack_flag) {
							tracker_remote_command[tracker_id] = ESB_PONG_FLAG_NORMAL;
							LOG_DBG(
								"Tracker %u confirmed command 0x%02X, clearing "
								"flag",
								tracker_id,
								ping_ack_flag
							);

							if ((ping_ack_flag == ESB_PONG_FLAG_SET_CHANNEL
								 || ping_ack_flag == ESB_PONG_FLAG_CLEAR_CHANNEL)
								&& atomic_get(&channel_change_pending)) {
								// Use the return value (old mask) of atomic_or to compute
								// the new mask locally, avoiding a separate atomic_get that
								// could race with other trackers confirming concurrently.
								atomic_val_t old_mask = atomic_or(&channel_ack_mask, (1 << tracker_id));
								atomic_val_t new_mask = old_mask | (1 << tracker_id);
								LOG_INF(
									"Tracker %u confirmed channel change "
									"(%u/%u confirmed)",
									tracker_id,
									__builtin_popcount(new_mask),
									stored_trackers
								);
							}
						}
					}

					/* PONG is now built by ack_handler in radio ISR context.
					 * event_handler only needs to track sequence state. */
					last_pong_queued_counter[tracker_id] = counter;
				}
			} break;
			case 17: // 16 bytes data + 1 byte sequence number
			{
				uint8_t tracker_id = rx_payload.data[1];

				// TDMA Slot Check for Data (Type 17)
				tdma_check_slot(tracker_id, current_rx_ticks);

				if (tracker_id >= stored_trackers) { // not a stored tracker
					continue;
				}

				if (rx_payload.data[0] > 223) { // reserved for receiver only
					break;
				}

				uint8_t received_sequence = rx_payload.data[16];
				int seq_result = check_packet_sequence(tracker_id, received_sequence);
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
			case 16: // legacy format without sequence number, no TDMA check, just for backward compatibility
			{
				uint8_t imu_id = rx_payload.data[1];

				if (imu_id >= stored_trackers) { // not a stored tracker
					continue;
				}

				if (rx_payload.data[0] > 223) { // reserved for receiver only
					break;
				}
				hid_write_packet_n(rx_payload.data,
								   rx_payload.rssi); // write to hid endpoint
			} break;
			default:
			{
				/* Raw data collection packets (types 0x10-0x12): variable length.
				 * Forward raw payload to CDC for PC-side data collection.
				 * Duplicate raw IMU packets (same tracker+sequence) are
				 * dropped since trackers send each sample twice for
				 * redundancy in noack mode. */
				uint8_t pkt_type = rx_payload.data[0];
				if (pkt_type == ESB_RAW_IMU_TYPE ||
				    pkt_type == ESB_RAW_MAG_TYPE ||
				    pkt_type == ESB_RAW_META_TYPE) {
					uint8_t tracker_id = rx_payload.data[1];
					if (tracker_id < stored_trackers &&
					    data_collect_is_target(tracker_id)) {
						/* Dedup raw IMU by tracker_id + sequence */
						if (pkt_type == ESB_RAW_IMU_TYPE) {
							static uint8_t last_raw_tracker = 0xFF;
							static uint16_t last_raw_seq = 0xFFFF;
							uint16_t seq = sys_get_be16(&rx_payload.data[2]);
							if (tracker_id == last_raw_tracker && seq == last_raw_seq) {
								break; /* duplicate */
							}
							last_raw_tracker = tracker_id;
							last_raw_seq = seq;
						}
						data_collect_write(rx_payload.data,
								   rx_payload.length,
								   rx_payload.rssi);
					}
					break;
				}
				/* Composite packet (type 0x05): variable length.
				 * Format: [0x05][tracker_id][sub_count][sub_type0][sub_data0...]...[sequence]
				 * Each sub-packet: 1 byte type + variable data.
				 */
				if (rx_payload.length < 5 || rx_payload.data[0] != ESB_COMPOSITE_TYPE) {
					LOG_ERR("Wrong packet length: %d", rx_payload.length);
					break;
				}

				uint8_t tracker_id = rx_payload.data[1];
				uint8_t sub_count = rx_payload.data[2];

				// TDMA Slot Check for Composite Packet
				tdma_check_slot(tracker_id, current_rx_ticks);

				if (tracker_id >= stored_trackers) {
					continue;
				}

				LOG_DBG("Received composite packet from tracker %d with %d sub-packets", tracker_id, sub_count);

				/* Sequence byte is at the very end of the composite packet */
				uint8_t received_sequence = rx_payload.data[rx_payload.length - 1];
				int seq_result = check_packet_sequence(tracker_id, received_sequence);
				if (seq_result == 2) {
					LOG_WRN(
						"TRK %d: Composite packet seq=%d is out-of-order, dropped",
						tracker_id,
						received_sequence
					);
					break; /* out-of-order */
				}

				/* Parse sub-packets and reconstruct standard 16-byte packets */
				int pos = 3; /* skip header: type, id, sub_count */
				int end = rx_payload.length - 1; /* exclude sequence byte */

				for (int i = 0; i < sub_count && pos < end; i++) {
					uint8_t sub_type = rx_payload.data[pos++];
					int sub_len;

					/* Determine sub-packet data length */
					switch (sub_type) {
					case 0: sub_len = 13; break; /* info */
					case 1: sub_len = 14; break; /* quat+accel */
					case 2: sub_len = 13; break; /* compact quat */
					case 3: sub_len = 2; break;  /* status */
					case 4: sub_len = 14; break; /* quat+mag */
					default:
						LOG_ERR("Unknown composite sub-type: %d", sub_type);
						sub_len = -1;
						break;
					}

					if (sub_len < 0 || pos + sub_len > end) {
						break;
					}

					/* Reconstruct a standard 16-byte packet */
					uint8_t pkt[16] = {0};
					pkt[0] = sub_type;
					pkt[1] = tracker_id;
					memcpy(&pkt[2], &rx_payload.data[pos], MIN(sub_len, 14));

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

					hid_write_packet_n(pkt, rx_payload.rssi);
					pos += sub_len;
				}
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

// this was randomly generated
// TODO: I have no idea?
static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

static bool esb_initialized = false;

int esb_initialize(bool tx)
{
	if (esb_initialized) {
		LOG_WRN("ESB already initialized");
	}
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	if (tx) {
		config.protocol = ESB_PROTOCOL_ESB_DPL;
		// config.mode = ESB_MODE_PTX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = RADIO_RETRANSMIT_DELAY;
		// config.retransmit_count = 0;
		config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;  // config by CONFIG_ESB_MAX_PAYLOAD_LENGTH
		config.selective_auto_ack = true;
		config.use_fast_ramp_up = true;
	} else {
		config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
		config.ack_handler = esb_ack_handler_cb;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = RADIO_RETRANSMIT_DELAY;
		// config.retransmit_count = 3;
		// config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;  // config by CONFIG_ESB_MAX_PAYLOAD_LENGTH
		config.selective_auto_ack = true;
		config.use_fast_ramp_up = true;
	}

	LOG_INF("Initializing ESB, %sX mode", tx ? "T" : "R");
	err = esb_init(&config);

	if (!err) {
		// Use saved channel if available, otherwise use default
		uint8_t channel_to_use = (receiver_rf_channel <= 100) ? receiver_rf_channel : RADIO_RF_CHANNEL;
		err = esb_set_rf_channel(channel_to_use);
		LOG_INF("Set RF channel to %u", channel_to_use);
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
		return err;
	}

	esb_initialized = true;
	return 0;
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

inline void esb_set_addr_discovery(void)
{
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

inline void esb_set_addr_paired(void)
{
	// Generate addresses from device address
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is
													   // not actually guaranteed, see datasheet)
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
	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
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

int esb_add_pair(uint64_t addr, bool checksum)
{
	if (addr == 0) {
		return -EINVAL;
	}

	bool new_entry = false;
	int assigned_id = -1;

	k_mutex_lock(&tracker_store_lock, K_FOREVER);
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
		// Write addr first, then barrier, then increment count
		// This ensures ISR lockless reads see consistent data
		stored_tracker_addr[assigned_id] = addr;
		__asm__ volatile("" ::: "memory"); // compiler barrier
		stored_trackers = assigned_id + 1;
		new_entry = true;
	}

	k_mutex_unlock(&tracker_store_lock);

	if (new_entry) {
		LOG_INF("Added device on id %d with address %012llX", assigned_id, addr);
		// Async NVS writes (non-blocking)
		nvs_write_async(STORED_ADDR_0 + assigned_id, &stored_tracker_addr[assigned_id], sizeof(stored_tracker_addr[0]));
		uint8_t count = stored_trackers;
		nvs_write_async(STORED_TRACKERS, &count, sizeof(count));
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

void esb_pop_pair(void)
{
	uint64_t removed_addr = 0;
	int removed_id = -1;

	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	if (stored_trackers > 0) {
		removed_id = stored_trackers - 1;
		removed_addr = stored_tracker_addr[removed_id];
		// Zero entry first, then barrier, then decrement count
		// This ensures ISR lockless reads never match a removed entry
		stored_tracker_addr[removed_id] = 0;
		__asm__ volatile("" ::: "memory"); // compiler barrier
		stored_trackers = (uint8_t)removed_id;
	}
	k_mutex_unlock(&tracker_store_lock);

	if (removed_id >= 0) {
		uint8_t count = stored_trackers;
		nvs_write_async(STORED_TRACKERS, &count, sizeof(count));
		uint64_t zero_addr = 0;
		nvs_write_async(STORED_ADDR_0 + removed_id, &zero_addr, sizeof(zero_addr));
		LOG_INF("Removed device on id %d with address %012llX", removed_id, removed_addr);
	} else {
		LOG_WRN("No devices to remove");
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
	bool is_new_device = checksum_valid && found_addr != 0 && send_tracker_id == tracker_count_snapshot && has_capacity;
	bool ack_valid = false;

	if (is_new_device) {
		int assigned_id = esb_add_pair(found_addr, false);
		if (assigned_id >= 0) {
			send_tracker_id = (uint16_t)assigned_id;
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
		} else if (assigned_id == -ENOSPC) {
			LOG_WRN("Maximum tracker slots reached, cannot pair %012llX", found_addr);
		} else {
			LOG_ERR("Failed to store tracker address %012llX: %d", found_addr, assigned_id);
		}
	}

	ack_valid = checksum_valid && send_tracker_id < MAX_TRACKERS;

	return ack_valid;
}

void esb_start_pairing(void)
{
	LOG_INF("Pairing mode enabled (unified)");
	esb_pairing = true;
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

void esb_reset_pair(void)
{
	// In unified mode, just enable pairing (no ESB reinit needed)
	esb_start_pairing();
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

void esb_clear(void)
{
	esb_clearing = true;

	// Disable pairing mode during clear
	esb_pairing = false;
	k_msgq_purge(&esb_pairing_msgq);

	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	uint8_t previous_count = stored_trackers;
	// Set count to 0 first — ISR immediately stops reading the array
	stored_trackers = 0;
	__asm__ volatile("" ::: "memory"); // compiler barrier
	memset(stored_tracker_addr, 0, sizeof(stored_tracker_addr));
	k_mutex_unlock(&tracker_store_lock);

	// Async NVS writes
	uint8_t zero_count = 0;
	nvs_write_async(STORED_TRACKERS, &zero_count, sizeof(zero_count));
	for (uint8_t i = 0; i < previous_count && i < MAX_TRACKERS; i++) {
		uint64_t zero_addr = 0;
		nvs_write_async(STORED_ADDR_0 + i, &zero_addr, sizeof(zero_addr));
	}
	LOG_INF("NVS Reset");

	// Reset packet sequence state for all trackers
	for (int i = 0; i < MAX_TRACKERS; i++) {
		last_packet_sequence[i] = 0;
		packet_count[i] = 0;
		memset(&tracker_stats[i], 0, sizeof(struct packet_stats));
	}
	LOG_INF("Packet sequence state and statistics reset for all trackers");

	hid_reset_all_rssi_smooth();
	esb_clearing = false;
}

// Reset packet sequence state for a specific tracker
void esb_reset_tracker_sequence(uint8_t tracker_id)
{
	if (tracker_id < MAX_TRACKERS) {
		last_packet_sequence[tracker_id] = 0;
		packet_count[tracker_id] = 0;
		// Reset PING counter tracking
		last_ping_counter[tracker_id] = 0;
		ping_counter_initialized[tracker_id] = false;
		last_pong_queued_counter[tracker_id] = 0;
		// Reset statistics
		memset(&tracker_stats[tracker_id], 0, sizeof(struct packet_stats));
		// Reset RSSI smoothing state
		hid_reset_rssi_smooth(tracker_id);
		LOG_INF("Packet sequence state and statistics reset for tracker %d", tracker_id);
	}
}

void esb_send_remote_command_sens(uint8_t tracker_id, float x, float y, float z)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}
	pending_sens_data[tracker_id][0] = (int16_t)(x * 100.0f);
	pending_sens_data[tracker_id][1] = (int16_t)(y * 100.0f);
	pending_sens_data[tracker_id][2] = (int16_t)(z * 100.0f);
	tracker_remote_command[tracker_id] = ESB_PONG_FLAG_SENS_SET;
	LOG_INF("Queued SENS_SET for tracker %u: %.2f, %.2f, %.2f", tracker_id, (double)x, (double)y, (double)z);
}

void esb_send_remote_command_channel(uint8_t tracker_id, uint8_t channel)
{
	if (tracker_id < MAX_TRACKERS) {
		tracker_remote_command[tracker_id] = ESB_PONG_FLAG_SET_CHANNEL;
		tracker_channel_value = channel;
		LOG_INF("Queued SET_CHANNEL %u for tracker %u", channel, tracker_id);
	}
}

// Send remote command to specified tracker
void esb_send_remote_command(uint8_t tracker_id, uint8_t command_flag)
{
	if (tracker_id < MAX_TRACKERS) {
		tracker_remote_command[tracker_id] = command_flag;

		/* Reset ARQ state when data collection starts */
		if (command_flag == ESB_PONG_FLAG_DATA_COLLECT_ON) {
			raw_arq_reset();
		}

		const char *cmd_name = "UNKNOWN";
		switch (command_flag) {
		case ESB_PONG_FLAG_NORMAL:
			cmd_name = "NORMAL";
			break;
		case ESB_PONG_FLAG_SHUTDOWN:
			cmd_name = "SHUTDOWN";
			break;
		case ESB_PONG_FLAG_CALIBRATE:
			cmd_name = "CALIBRATE";
			break;
		case ESB_PONG_FLAG_SIX_SIDE_CAL:
			cmd_name = "SIX_SIDE_CAL";
			break;
		case ESB_PONG_FLAG_MEOW:
			cmd_name = "MEOW";
			break;
		case ESB_PONG_FLAG_SCAN:
			cmd_name = "SCAN";
			break;
		case ESB_PONG_FLAG_MAG_CLEAR:
			cmd_name = "MAG_CLEAR";
			break;
		case ESB_PONG_FLAG_MAG_CAL:
			cmd_name = "MAG_CAL";
			break;
		case ESB_PONG_FLAG_MAG_ON:
			cmd_name = "MAG_ON";
			break;
		case ESB_PONG_FLAG_MAG_OFF:
			cmd_name = "MAG_OFF";
			break;
		case ESB_PONG_FLAG_REBOOT:
			cmd_name = "REBOOT";
			break;
		case ESB_PONG_FLAG_CLEAR:
			cmd_name = "CLEAR";
			break;
		case ESB_PONG_FLAG_DFU:
			cmd_name = "DFU";
			break;
		case ESB_PONG_FLAG_DFU_OTA:
			cmd_name = "DFU_OTA";
			break;
		case ESB_PONG_FLAG_SET_CHANNEL:
			cmd_name = "SET_CHANNEL";
			break;
		case ESB_PONG_FLAG_SENS_SET:
			cmd_name = "SENS_SET";
			break;
		case ESB_PONG_FLAG_SENS_RESET:
			cmd_name = "SENS_RESET";
			break;
		case ESB_PONG_FLAG_RESET_ZRO:
			cmd_name = "RESET_ZRO";
			break;
		case ESB_PONG_FLAG_RESET_ACC:
			cmd_name = "RESET_ACC";
			break;
		case ESB_PONG_FLAG_RESET_BAT:
			cmd_name = "RESET_BAT";
			break;
		case ESB_PONG_FLAG_PING:
			cmd_name = "PING";
			break;
		case ESB_PONG_FLAG_RESET_TCAL:
			cmd_name = "RESET_TCAL";
			break;
		case ESB_PONG_FLAG_TCAL_ON:
			cmd_name = "TCAL_ON";
			break;
		case ESB_PONG_FLAG_TCAL_OFF:
			cmd_name = "TCAL_OFF";
			break;
		case ESB_PONG_FLAG_TCAL_AUTO_ON:
			cmd_name = "TCAL_AUTO_ON";
			break;
		case ESB_PONG_FLAG_TCAL_AUTO_OFF:
			cmd_name = "TCAL_AUTO_OFF";
			break;
		case ESB_PONG_FLAG_FUSION_RESET:
			cmd_name = "FUSION_RESET";
			break;
		case ESB_PONG_FLAG_TCAL_BOOT_ON:
			cmd_name = "TCAL_BOOT_ON";
			break;
		case ESB_PONG_FLAG_TCAL_BOOT_OFF:
			cmd_name = "TCAL_BOOT_OFF";
			break;
		case ESB_PONG_FLAG_TDMA_ON:
			cmd_name = "TDMA_ON";
			break;
		case ESB_PONG_FLAG_TDMA_OFF:
			cmd_name = "TDMA_OFF";
			break;
		case ESB_PONG_FLAG_TEST_MODE_ON:
			cmd_name = "TEST_MODE_ON";
			break;
		case ESB_PONG_FLAG_TEST_MODE_OFF:
			cmd_name = "TEST_MODE_OFF";
			break;
		case ESB_PONG_FLAG_DATA_COLLECT_ON:
			cmd_name = "DATA_COLLECT_ON";
			break;
		case ESB_PONG_FLAG_DATA_COLLECT_OFF:
			cmd_name = "DATA_COLLECT_OFF";
			break;
		}
		LOG_INF("Remote command %s (0x%02X) queued for tracker %d", cmd_name, command_flag, tracker_id);
	} else {
		LOG_ERR("Invalid tracker ID: %d", tracker_id);
	}
}

// Send remote command to all paired trackers
void esb_send_remote_command_all(uint8_t command_flag)
{
	uint8_t count = 0;
	int64_t scan_start_time = k_uptime_get();
	char active_tracker_ids[(MAX_TRACKERS * 4) + 1];
	size_t active_tracker_ids_len = 0;
	const char *cmd_name = "UNKNOWN";
	switch (command_flag) {
	case ESB_PONG_FLAG_NORMAL:
		cmd_name = "NORMAL";
		break;
	case ESB_PONG_FLAG_SHUTDOWN:
		cmd_name = "SHUTDOWN";
		break;
	case ESB_PONG_FLAG_CALIBRATE:
		cmd_name = "CALIBRATE";
		break;
	case ESB_PONG_FLAG_SIX_SIDE_CAL:
		cmd_name = "SIX_SIDE_CAL";
		break;
	case ESB_PONG_FLAG_MEOW:
		cmd_name = "MEOW";
		break;
	case ESB_PONG_FLAG_SCAN:
		cmd_name = "SCAN";
		break;
	case ESB_PONG_FLAG_MAG_CLEAR:
		cmd_name = "MAG_CLEAR";
		break;
	case ESB_PONG_FLAG_MAG_CAL:
		cmd_name = "MAG_CAL";
		break;
	case ESB_PONG_FLAG_MAG_ON:
		cmd_name = "MAG_ON";
		break;
	case ESB_PONG_FLAG_MAG_OFF:
		cmd_name = "MAG_OFF";
		break;
	case ESB_PONG_FLAG_REBOOT:
		cmd_name = "REBOOT";
		break;
	case ESB_PONG_FLAG_CLEAR:
		cmd_name = "CLEAR";
		break;
	case ESB_PONG_FLAG_DFU:
		cmd_name = "DFU";
		break;
	case ESB_PONG_FLAG_DFU_OTA:
		cmd_name = "DFU_OTA";
		break;
	case ESB_PONG_FLAG_SET_CHANNEL:
		cmd_name = "SET_CHANNEL";
		break;
	case ESB_PONG_FLAG_SENS_SET:
		cmd_name = "SENS_SET";
		break;
	case ESB_PONG_FLAG_SENS_RESET:
		cmd_name = "SENS_RESET";
		break;
	case ESB_PONG_FLAG_RESET_ZRO:
		cmd_name = "RESET_ZRO";
		break;
	case ESB_PONG_FLAG_RESET_ACC:
		cmd_name = "RESET_ACC";
		break;
	case ESB_PONG_FLAG_RESET_BAT:
		cmd_name = "RESET_BAT";
		break;
	case ESB_PONG_FLAG_PING:
		cmd_name = "PING";
		break;
	case ESB_PONG_FLAG_RESET_TCAL:
		cmd_name = "RESET_TCAL";
		break;
	case ESB_PONG_FLAG_TCAL_ON:
		cmd_name = "TCAL_ON";
		break;
	case ESB_PONG_FLAG_TCAL_OFF:
		cmd_name = "TCAL_OFF";
		break;
	case ESB_PONG_FLAG_TCAL_AUTO_ON:
		cmd_name = "TCAL_AUTO_ON";
		break;
	case ESB_PONG_FLAG_TCAL_AUTO_OFF:
		cmd_name = "TCAL_AUTO_OFF";
		break;
	case ESB_PONG_FLAG_FUSION_RESET:
		cmd_name = "FUSION_RESET";
		break;
	case ESB_PONG_FLAG_TCAL_BOOT_ON:
		cmd_name = "TCAL_BOOT_ON";
		break;
	case ESB_PONG_FLAG_TCAL_BOOT_OFF:
		cmd_name = "TCAL_BOOT_OFF";
		break;
	case ESB_PONG_FLAG_TDMA_ON:
		cmd_name = "TDMA_ON";
		break;
	case ESB_PONG_FLAG_TDMA_OFF:
		cmd_name = "TDMA_OFF";
		break;
	case ESB_PONG_FLAG_TEST_MODE_ON:
		cmd_name = "TEST_MODE_ON";
		break;
	case ESB_PONG_FLAG_TEST_MODE_OFF:
		cmd_name = "TEST_MODE_OFF";
		break;
	}

	k_msleep(REMOTE_COMMAND_ACTIVE_SCAN_MS);

	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; i++) {
		if (stored_tracker_addr[i] != 0 && tracker_stats[i].last_packet_time >= scan_start_time) {
			tracker_remote_command[i] = command_flag;
			if (active_tracker_ids_len < sizeof(active_tracker_ids)) {
				int written = snprintk(
					&active_tracker_ids[active_tracker_ids_len],
					sizeof(active_tracker_ids) - active_tracker_ids_len,
					count == 0 ? "%u" : ",%u",
					i
				);
				if (written > 0) {
					active_tracker_ids_len += MIN((size_t)written, sizeof(active_tracker_ids) - active_tracker_ids_len - 1);
				}
			}
			count++;
		}
	}
	k_mutex_unlock(&tracker_store_lock);

	if (count == 0) {
		active_tracker_ids[0] = '\0';
	}

	LOG_INF(
		"Remote command %s (0x%02X) queued for %d active trackers after %d ms scan; active IDs: %s",
		cmd_name,
		command_flag,
		count,
		REMOTE_COMMAND_ACTIVE_SCAN_MS,
		count > 0 ? active_tracker_ids : "none"
	);
}

// Manually print statistics for all active trackers
void esb_print_all_stats(void)
{
	LOG_INF("=== Packet Statistics Summary ===");
	print_tracker_stats_batch();
	LOG_INF("================================");
}

void esb_reset_all_stats(void)
{
	for (int i = 0; i < MAX_TRACKERS; i++) {
		memset(&tracker_stats[i], 0, sizeof(struct packet_stats));
		ping_counter_initialized[i] = false;
		last_ping_counter[i] = 0;
		last_pong_queued_counter[i] = 0;
	}
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

// Get remaining time for detailed stats (0 if disabled or no auto-disable)
uint32_t esb_get_stats_detailed_remaining(void)
{
	if (!stats_detailed_enabled || stats_detailed_end_time == 0) {
		return 0;
	}
	int64_t remaining = stats_detailed_end_time - k_uptime_get();
	if (remaining <= 0) {
		return 0;
	}
	return (uint32_t)(remaining / 1000);
}

// Sets the RF channel for all trackers
void esb_set_all_trackers_channel(uint8_t channel)
{
	if (channel < 1 || channel > 100) {
		LOG_ERR("Invalid channel value: %u (must be 1-100)", channel);
		return;
	}

	if (atomic_get(&channel_change_pending)) {
		LOG_WRN("Channel change already in progress, please wait");
		return;
	}

	tracker_channel_value = channel;
	pending_channel = channel;
	channel_change_timeout = k_uptime_get() + CHANNEL_CHANGE_TIMEOUT_MS;
	// Clear mask before setting the pending flag to avoid losing confirmations
	// that arrive between the flag set and the mask clear.
	atomic_set(&channel_ack_mask, 0);
	atomic_set(&channel_change_pending, 1);

	esb_send_remote_command_all(ESB_PONG_FLAG_SET_CHANNEL);
	LOG_INF("RF channel %u command sent to all trackers, waiting for confirmation...", channel);
}

// Clear RF channel settings for all trackers (restore to default)
void esb_clear_all_trackers_channel(void)
{
	if (atomic_get(&channel_change_pending)) {
		LOG_WRN("Channel change already in progress, please wait");
		return;
	}

	pending_channel = 0xFF; // Special value to indicate clearing
	channel_change_timeout = k_uptime_get() + CHANNEL_CHANGE_TIMEOUT_MS;
	// Clear mask before setting the pending flag to avoid losing confirmations
	// that arrive between the flag set and the mask clear.
	atomic_set(&channel_ack_mask, 0);
	atomic_set(&channel_change_pending, 1);

	esb_send_remote_command_all(ESB_PONG_FLAG_CLEAR_CHANNEL);
	LOG_INF("CLEAR_CHANNEL command sent to all trackers, waiting for confirmation...");
}

// Set the receiver's RF channel (local, does not affect trackers)
void esb_set_receiver_channel(uint8_t channel)
{
	if (channel < 1 || channel > 100) {
		LOG_ERR("Invalid channel value: %u (must be 1-100)", channel);
		return;
	}

	LOG_INF("Setting receiver RF channel to %u (local only)", channel);
	receiver_rf_channel = channel;

	// Save to NVS
	sys_write(RF_CHANNEL, NULL, &receiver_rf_channel, sizeof(receiver_rf_channel));

	// Reinitialize ESB with new channel
	esb_deinitialize();
	esb_initialize(false);
	esb_start_rx();

	LOG_INF("Receiver channel switched to %u", receiver_rf_channel);
}

// Clear the receiver's RF channel setting (local, does not affect trackers)
void esb_clear_receiver_channel(void)
{
	LOG_INF("Clearing receiver RF channel (local only)");
	receiver_rf_channel = 0xFF;

	// Clear from NVS
	sys_write(RF_CHANNEL, NULL, &receiver_rf_channel, sizeof(receiver_rf_channel));

	// Reinitialize ESB with default channel
	esb_deinitialize();
	esb_initialize(false);
	esb_start_rx();

	LOG_INF("Receiver channel cleared, using default");
}

// Get the receiver's RF channel
uint8_t esb_get_receiver_channel(void)
{
	return receiver_rf_channel;
}

// Set up unified addresses: pipe 0 for discovery (pairing), pipes 1-7 for paired data
void esb_receive(void)
{
	esb_set_addr_unified();
	esb_paired = true;
}

// NVS writer thread: processes async NVS write requests
static void nvs_writer_thread(void)
{
	struct nvs_write_request req;
	while (1) {
		k_msgq_get(&nvs_write_msgq, &req, K_FOREVER);
		k_msleep(5); // Simulate async write delay
		sys_write(req.id, NULL, req.data, req.len);
	}
}

static void esb_thread(void)
{
	clocks_start();

	sys_read(STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	uint8_t tracker_count = stored_trackers;
	for (uint8_t i = 0; i < tracker_count && i < MAX_TRACKERS; i++) {
		sys_read(STORED_ADDR_0 + i, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
	}
	k_mutex_unlock(&tracker_store_lock);

	// Load saved RF channel from NVS if exists
	uint8_t saved_channel = 0xFF;
	sys_read(RF_CHANNEL, &saved_channel, sizeof(saved_channel));
	// 0xFF and 0 both indicate "use default"
	if (saved_channel != 0xFF && saved_channel != 0 && saved_channel <= 100) {
		// Valid saved channel found, use it
		receiver_rf_channel = saved_channel;
		LOG_INF("Loaded RF channel %u from NVS", saved_channel);
	} else {
		LOG_INF("No saved RF channel, using default %u", RADIO_RF_CHANNEL);
		// If channel was 0 (uninitialized), write 0xFF to NVS
		if (saved_channel == 0) {
			saved_channel = 0xFF;
			sys_write(RF_CHANNEL, NULL, &saved_channel, sizeof(saved_channel));
		}
	}

	if (tracker_count) {
		esb_paired = true;
	}
	LOG_INF("%d/%d devices stored", tracker_count, MAX_TRACKERS);

	/* Pre-fill TDMA config based on stored tracker count so trackers
	 * receive a valid config on the very first PONG (before the periodic
	 * recalculation detects them as active). */
	if (tracker_count > 0) {
		uint8_t init_slot;
		if (tracker_count == 1) {
			init_slot = 163;
		} else {
			uint32_t denom = TDMA_MAX_TPS_TARGET * tracker_count;
			init_slot = (uint8_t)((32768 + denom - 1) / denom);
			if (init_slot < TDMA_MIN_SLOT_TICKS) init_slot = TDMA_MIN_SLOT_TICKS;
		}
		tdma_config_epoch++;
		for (uint8_t i = 0; i < tracker_count && i < MAX_TRACKERS; i++) {
			tdma_config_packed[i] = tdma_pack_config(
				i, tracker_count, init_slot, tdma_config_epoch);
		}
		tdma_dynamic_active_count = tracker_count;
		tdma_dynamic_slot_ticks = init_slot;
		tdma_active_mask = (1U << tracker_count) - 1;
		uint32_t init_frame = (uint32_t)init_slot * tracker_count;
		LOG_INF("TDMA initial: %u stored, slot=%u, frame=%u, ~%u TPS (epoch=%u)",
			tracker_count, init_slot, init_frame,
			init_frame > 0 ? 32768 / init_frame : 0, tdma_config_epoch);
	}

	// Cache receiver device address for ISR pairing responses
	uint64_t *dev_addr = (uint64_t *)NRF_FICR->DEVICEADDR;
	memcpy(receiver_device_addr, dev_addr, 6);
	LOG_INF("Receiver address: %012llX", *dev_addr & 0xFFFFFFFFFFFF);

	// Always start in unified mode: pipe 0 for pairing, pipes 1-7 for data
	esb_receive();
	esb_initialize(false);
	esb_start_rx();

	// Auto-enable pairing mode if no stored trackers
	if (tracker_count == 0) {
		esb_pairing = true;
		pairing_start_time = k_uptime_get();
		pairing_target_count = 0;
		pairing_initial_count = 0;
		LOG_INF("No stored trackers, pairing mode auto-enabled");
		set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
	}

	while (1) {
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
						LOG_INF("Pairing target count reached (%u new devices), exiting in %d seconds...",
							new_devices, PAIRING_EXIT_DELAY_MS / 1000);
					}
					// Check if delay has passed
					int64_t elapsed_since_target = k_uptime_get() - pairing_target_reached_time;
					if (elapsed_since_target >= PAIRING_EXIT_DELAY_MS) {
						esb_finish_pair();
					}
				}
			}
		}

		// Check if channel change is complete
		if (atomic_get(&channel_change_pending)) {
			uint32_t expected_mask = (stored_trackers < 32) ? ((1U << stored_trackers) - 1U) : 0xFFFFFFFFU;
			uint32_t current_mask = (uint32_t)atomic_get(&channel_ack_mask);
			int64_t now = k_uptime_get();

			if (current_mask == expected_mask) {
				// All trackers confirmed, switch receiver channel
				if (pending_channel == 0xFF) {
					// Clear channel setting
					LOG_INF("All trackers confirmed channel clear, restoring receiver to default");
					receiver_rf_channel = 0xFF;

					// Clear from NVS
					sys_write(RF_CHANNEL, NULL, &receiver_rf_channel, sizeof(receiver_rf_channel));

					// Reinitialize ESB with default channel (unified addresses)
					esb_deinitialize();
					esb_set_addr_unified();
					esb_initialize(false);
					esb_start_rx();

					LOG_INF("Receiver channel cleared, using default %u", RADIO_RF_CHANNEL);
				} else {
					// Set new channel
					LOG_INF("All trackers confirmed channel change to %u, switching receiver", pending_channel);
					receiver_rf_channel = pending_channel;

					// Save to NVS
					sys_write(RF_CHANNEL, NULL, &receiver_rf_channel, sizeof(receiver_rf_channel));

					// Reinitialize ESB with new channel (unified addresses)
					esb_deinitialize();
					esb_set_addr_unified();
					esb_initialize(false);
					esb_start_rx();

					LOG_INF("Receiver channel switched to %u successfully", receiver_rf_channel);
				}

				atomic_set(&channel_change_pending, 0);
			} else if (now >= channel_change_timeout) {
				// Timeout, cancel channel change
				LOG_WRN(
					"Channel change timeout, %u/%u trackers confirmed",
					__builtin_popcount(current_mask),
					stored_trackers
				);
				atomic_set(&channel_change_pending, 0);
				tracker_channel_value = 0; // Reset pending channel value
			}
		}

		k_msleep(100);
	}
}
