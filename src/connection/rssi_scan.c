/*
 * SlimeVR Code is placed under the MIT license
 * Copyright (c) 2026 SlimeVR Contributors
 */

#include "connection/rssi_scan.h"
#include "connection/rssi_scan_fem.h"

#include "connection/esb.h"

#include <esb.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>

#include <hal/nrf_radio.h>

#define RSSI_SCAN_FIRST_CHANNEL 0
#define RSSI_SCAN_LAST_CHANNEL 88

#define RSSI_SCAN_SAMPLES_PER_CH 50
#define RSSI_SCAN_SWEEPS 200

// Optional spacing. 0 = as fast as possible.
#define RSSI_SCAN_INTER_SAMPLE_US 0
#define RSSI_SCAN_INTER_SWEEP_MS 1

// RADIO->RSSISAMPLE is 7-bit, 0..127, representing -dBm.
// Higher sample => weaker energy (cleaner channel).
#define RSSI_SAMPLE_MAX 127

#define RSSI_SCAN_READY_TIMEOUT_US 200
#define RSSI_SCAN_RSSIEND_TIMEOUT_US 200
#define RSSI_SCAN_DISABLED_TIMEOUT_US 200

static bool wait_for_event_and_clear(volatile uint32_t *evt_reg, uint32_t timeout_us)
{
	uint32_t start = k_cycle_get_32();
	for (;;) {
		/* Events were cleared before triggering this operation. A fresh
		 * completion remains valid if the thread was preempted meanwhile. */
		if (*evt_reg) {
			*evt_reg = 0;
			return true;
		}
		if (k_cyc_to_us_floor32(k_cycle_get_32() - start) >= timeout_us) break;
		k_busy_wait(1);
	}
	/* Completion can arrive while preempted between the event read and the
	 * elapsed-time read. Observe it once more before declaring a timeout. */
	if (*evt_reg) {
		*evt_reg = 0;
		return true;
	}
	return false;
}

/**
 * Measure a channel by repeatedly triggering RSSI measurement and return the
 * minimum observed sample.
 *
 * Note:
 * - RSSISAMPLE represents -dBm.
 * - Therefore MIN(sample) corresponds to the strongest energy observed (lowest case).
 */
static int rssi_scan_channel_measure_min_n(uint8_t channel, uint32_t samples, uint8_t *result)
{
	/* Persistent DMA target: a failed disable must never expose expired stack. */
	static uint8_t energy_packet[256] __aligned(4);
	if (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) return -EBUSY;
	uint32_t saved_shorts = NRF_RADIO->SHORTS;
	/* Like Nordic radio_test's non-coded continuous RX: a matching packet
	 * END must restart RX rather than leave later RSSI tasks in RXIDLE. */
	NRF_RADIO->SHORTS = RADIO_SHORTS_END_START_Msk;
	int err = rssi_scan_fem_prepare();
	if (err) {
		(void)rssi_scan_fem_release();
		NRF_RADIO->SHORTS = saved_shorts;
		return err;
	}
	NRF_RADIO->PACKETPTR = (uintptr_t)energy_packet;
	// The RADIO->FREQUENCY register expects an offset in MHz from 2400 MHz.
	NRF_RADIO->FREQUENCY = channel;

	// Clear events we depend on.
	NRF_RADIO->EVENTS_READY = 0;
	NRF_RADIO->EVENTS_RSSIEND = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;

	NRF_RADIO->TASKS_RXEN = 1;
	if (!wait_for_event_and_clear(&NRF_RADIO->EVENTS_READY, RSSI_SCAN_READY_TIMEOUT_US)) {
		err = -ETIMEDOUT;
		goto disable;
	}
	NRF_RADIO->TASKS_START = 1;

	uint8_t min_sample = RSSI_SAMPLE_MAX;
	for (uint32_t i = 0; i < samples; i++) {
		NRF_RADIO->EVENTS_RSSIEND = 0;
		NRF_RADIO->TASKS_RSSISTART = 1;
		if (!wait_for_event_and_clear(&NRF_RADIO->EVENTS_RSSIEND, RSSI_SCAN_RSSIEND_TIMEOUT_US)) {
			err = -ETIMEDOUT;
			break;
		}

		uint8_t sample = (uint8_t)(NRF_RADIO->RSSISAMPLE & 0x7F);
		if (sample < min_sample) {
			min_sample = sample;
		}

		if (RSSI_SCAN_INTER_SAMPLE_US) {
			k_busy_wait(RSSI_SCAN_INTER_SAMPLE_US);
		}
	}

disable:
	NRF_RADIO->TASKS_DISABLE = 1;
	if (!wait_for_event_and_clear(&NRF_RADIO->EVENTS_DISABLED, RSSI_SCAN_DISABLED_TIMEOUT_US)) {
		err = -ETIMEDOUT;
	}
	int fem_err = rssi_scan_fem_release();
	if (fem_err) err = fem_err;
	if (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) return -EIO;
	NRF_RADIO->SHORTS = saved_shorts;
	if (!err) *result = min_sample;
	return err;
}

int rssi_scan_measure_bounded(uint8_t channel, struct rssi_scan_observation *out)
{
	static uint8_t packet[256] __aligned(4);
	if (channel > 100 || !out) return -EINVAL;
	if (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) return -EBUSY;
	uint32_t shorts = NRF_RADIO->SHORTS;
	uint32_t addresses = NRF_RADIO->RXADDRESSES;
	struct rssi_scan_observation result = {.peak = RSSI_SAMPLE_MAX};
	int err = rssi_scan_fem_prepare();
	if (err) {
		(void)rssi_scan_fem_release();
		return err;
	}
	NRF_RADIO->SHORTS = RADIO_SHORTS_END_START_Msk;
	NRF_RADIO->PACKETPTR = (uintptr_t)packet;
	NRF_RADIO->FREQUENCY = channel;
	/* Only unified paired pipes: ADDRESS activity is ambiguity, not proof of
	 * a valid packet. Public beacon traffic remains ordinary measured energy. */
	NRF_RADIO->RXADDRESSES = 0xfe;
	NRF_RADIO->EVENTS_ADDRESS = 0;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_READY = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_RXEN = 1;
	if (!wait_for_event_and_clear(&NRF_RADIO->EVENTS_READY, RSSI_SCAN_READY_TIMEOUT_US)) {
		err = -ETIMEDOUT;
		goto disable;
	}
	NRF_RADIO->TASKS_START = 1;
	/* Drain a packet already in flight before RX recognized its address.
	 * 2Mbps max configured payload plus conservative framing allowance.
	 * These guard intervals do not contribute observation coverage. */
	k_busy_wait(((NRF_RADIO->PCNF1 & 0xff) + 10) * 4);
	uint32_t start = k_cycle_get_32();
	uint32_t slot_cycles = k_us_to_cyc_ceil32(RSSI_SCAN_SLOT_US);
	for (unsigned slot = 0; slot < RSSI_SCAN_SLICE_US / RSSI_SCAN_SLOT_US; slot++) {
		uint32_t end = (slot + 1) * slot_cycles;
		/* Never compress missed temporal slots into a late sample burst. */
		if (k_cycle_get_32() - start >= end) {
			err = -ETIMEDOUT;
			break;
		}
		NRF_RADIO->EVENTS_RSSIEND = 0;
		NRF_RADIO->TASKS_RSSISTART = 1;
		if (!wait_for_event_and_clear(&NRF_RADIO->EVENTS_RSSIEND, RSSI_SCAN_SLOT_US) ||
		    k_cycle_get_32() - start >= end) {
			err = -ETIMEDOUT;
			break;
		}
		uint8_t value = NRF_RADIO->RSSISAMPLE & 0x7f;
		if (value < result.peak) result.peak = value;
		if (value > result.quiet_floor) result.quiet_floor = value;
		result.samples++;
		/* Native cycle bins work with 32768Hz RTC; skipped bins are invalid. */
		while (k_cycle_get_32() - start < end) k_busy_wait(1);
		if (k_cycle_get_32() - start > end) {
			err = -ETIMEDOUT;
			break;
		}
	}
	/* Reading phases span one tick: report the conservative lower bound,
	 * equal per accepted visit, not fictitious exact wall-clock dwell. */
	if (!err) result.coverage_us = k_cyc_to_us_floor32(
		slot_cycles * (RSSI_SCAN_SLICE_US / RSSI_SCAN_SLOT_US) - 1);
	/* Address completion can trail a sample at the final boundary. Any matched
	 * address, even incomplete/CRC-failed, makes the whole visit ambiguous.
	 * Drop numerator AND denominator; never infer packet identity from RSSI
	 * or race CRCSTATUS against continuous END_START. */
	k_busy_wait(25);
	if (!err && NRF_RADIO->EVENTS_ADDRESS) err = -EAGAIN;
disable:
	NRF_RADIO->TASKS_DISABLE = 1;
	if (!wait_for_event_and_clear(&NRF_RADIO->EVENTS_DISABLED, RSSI_SCAN_DISABLED_TIMEOUT_US))
		err = -ETIMEDOUT;
	int fem_err = rssi_scan_fem_release();
	if (fem_err) err = fem_err;
	if (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) return -EIO;
	NRF_RADIO->SHORTS = shorts;
	NRF_RADIO->RXADDRESSES = addresses;
	if (!err) *out = result;
	return err;
}

static uint8_t get_current_effective_channel(void)
{
	uint8_t ch = esb_get_receiver_channel();
	if (ch == 0xFF) {
		ch = CONFIG_RADIO_RF_CHANNEL;
	}
	return ch;
}

static bool better_channel(
	uint8_t ch_a,
	uint8_t lowest_a,
	uint8_t ch_b,
	uint8_t lowest_b,
	bool ch_b_valid,
	uint8_t current_ch
)
{
	if (!ch_b_valid) {
		return true;
	}
	// Primary: maximize lowest (higher = cleaner).
	if (lowest_a > lowest_b) {
		return true;
	}
	if (lowest_a < lowest_b) {
		return false;
	}

	// Tie-breaker: stay close to current channel to reduce disruption.
	int dist_a = (ch_a > current_ch) ? (ch_a - current_ch) : (current_ch - ch_a);
	int dist_b = (ch_b > current_ch) ? (ch_b - current_ch) : (current_ch - ch_b);
	if (dist_a < dist_b) {
		return true;
	}
	if (dist_a > dist_b) {
		return false;
	}

	// Final tie-breaker: smaller channel number.
	return ch_a < ch_b;
}

/* A failed visit invalidates the entire channel, not just that visit's
 * samples. Keep failure counts even when later visits complete normally. */
static struct {
	int16_t score;
	uint16_t failures;
} scan_results[RSSI_SCAN_LAST_CHANNEL + 1];

void rssi_scan_run_and_print(void)
{
	int admission = esb_radio_operation_begin();
	if (admission) {
		printk(admission == -EBUSY ? "RSSI scan busy: radio in use\n" :
		       "RSSI scan unavailable: radio startup or teardown failed\n");
		return;
	}

	printk(
		"RSSI scan (ESB channels %u..%u): samples/ch/sweep=%u, sweeps=%d (total=%u samples/ch)\n",
		RSSI_SCAN_FIRST_CHANNEL,
		RSSI_SCAN_LAST_CHANNEL,
		(uint32_t)RSSI_SCAN_SAMPLES_PER_CH,
		RSSI_SCAN_SWEEPS,
		(uint32_t)RSSI_SCAN_SAMPLES_PER_CH * (uint32_t)RSSI_SCAN_SWEEPS
	);

	/* The shared owner excludes migration, OTA, raw sessions and auto scan. */
	if (esb_radio_operation_stop()) {
		esb_radio_operation_end();
		printk("RSSI scan could not stop radio\n");
		return;
	}
	esb_rx_discontinuity();
	NRF_RADIO->SHORTS = 0;


	for (uint8_t ch = RSSI_SCAN_FIRST_CHANNEL; ch <= RSSI_SCAN_LAST_CHANNEL; ch++) {
		scan_results[ch].score = RSSI_SAMPLE_MAX;
		scan_results[ch].failures = 0;
	}

	for (int sweep = 0; sweep < RSSI_SCAN_SWEEPS; sweep++) {
		// printk("RSSI scan sweep %d/%d...\n", sweep + 1, RSSI_SCAN_SWEEPS);
		for (uint8_t ch = RSSI_SCAN_FIRST_CHANNEL; ch <= RSSI_SCAN_LAST_CHANNEL; ch++) {
			if (!esb_channel_is_allowed(ch)) {
				continue; // only preferred channels
			}
			uint8_t lowest;
			int err = rssi_scan_channel_measure_min_n(ch, RSSI_SCAN_SAMPLES_PER_CH, &lowest);
			if (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled || !rssi_scan_fem_is_safe()) {
				printk("RSSI RADIO/frontend cleanup failed; retaining radio owner\n");
				return;
			}
			if (err) {
				scan_results[ch].failures++;
				scan_results[ch].score = -1;
			} else if (lowest < scan_results[ch].score) {
				scan_results[ch].score = lowest;
			}
		}

		// Yield to other threads/USB log processing.
		if (RSSI_SCAN_INTER_SWEEP_MS) {
			k_msleep(RSSI_SCAN_INTER_SWEEP_MS);
		} else {
			k_yield();
		}
	}

	esb_set_addr_unified();
	int restore_err = esb_radio_quiesce();
	if (!restore_err) restore_err = esb_initialize_quiet(false);
	if (!restore_err) esb_rx_discontinuity();
	if (!restore_err) restore_err = esb_start_rx();
	if (!restore_err) esb_radio_operation_end();
	if (restore_err) {
		printk("RSSI scan home radio restore failed: %d\n", restore_err);
		return;
	}

	uint8_t current_ch = get_current_effective_channel();

	/* Only complete, failure-free channels can be recommended. */
	enum { NUM_CH = RSSI_SCAN_LAST_CHANNEL - RSSI_SCAN_FIRST_CHANNEL + 1 };
	uint8_t cand_ch[NUM_CH];
	int cand_count = 0;
	for (uint8_t ch = RSSI_SCAN_FIRST_CHANNEL; ch <= RSSI_SCAN_LAST_CHANNEL; ch++) {
		if (esb_channel_is_allowed(ch) && scan_results[ch].score >= 0) {
			cand_ch[cand_count++] = ch;
		}
	}

	uint8_t recommended = 0;
	bool has_recommended = false;
	for (int i = 0; i < cand_count; i++) {
		uint8_t ch = cand_ch[i];
		if (better_channel(
				ch,
				scan_results[ch].score,
				recommended,
				has_recommended ? scan_results[recommended].score : 0,
				has_recommended,
				current_ch
			)) {
			recommended = ch;
			has_recommended = true;
		}
	}

	printk("Current channel (effective): %u\n", current_ch);
	if (has_recommended) {
		printk("Recommended: channel %u (-%d dBm)\n\n",
		       recommended, scan_results[recommended].score);
	} else {
		printk("Recommended: none (no valid complete channel measurements)\n\n");
	}
	for (uint8_t ch = RSSI_SCAN_FIRST_CHANNEL; ch <= RSSI_SCAN_LAST_CHANNEL; ch++) {
		if (scan_results[ch].failures)
			printk("Channel %u invalid: %u/%u visits failed; excluded\n",
			       ch, scan_results[ch].failures, RSSI_SCAN_SWEEPS);
	}

	// Sort measured channels only, using simple insertion sort.
	uint8_t sorted_ch[NUM_CH];
	for (int i = 0; i < cand_count; i++) {
		sorted_ch[i] = cand_ch[i];
	}
	for (int i = 1; i < cand_count; i++) {
		uint8_t key = sorted_ch[i];
		int j = i - 1;
		while (j >= 0
			   && better_channel(
				   key,
				   scan_results[key].score,
				   sorted_ch[j],
				   scan_results[sorted_ch[j]].score,
				   true,
				   current_ch
			   )) {
			sorted_ch[j + 1] = sorted_ch[j];
			j--;
		}
		sorted_ch[j + 1] = key;
	}

	// Print all channels in compact table format
	// Format: "CH(WST)" where WST is lowest (min across all samples).
	printk("All channels sorted by RSSI (lower is better):\n");
	enum { PER_LINE = 8 };
	for (int i = 0; i < cand_count; i++) {
		uint8_t ch = sorted_ch[i];
		printk("%3u (-%2d dBm)", ch, scan_results[ch].score);
		if ((i + 1) % PER_LINE == 0 || i == cand_count - 1) {
			printk("\n");
			k_msleep(20);
		} else {
			printk("  ");
		}
	}

}
