#include "rssi_scan_fem.h"

#if defined(CONFIG_MPSL_FEM)
#include <errno.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <nrfx_timer.h>
#include <hal/nrf_radio.h>
#include <mpsl_fem_protocol_api.h>

/* All receiver boards currently enabling FEM are nRF52. A future DPPI FEM
 * board needs an owned, published DISABLED channel and radio-domain timer;
 * a peripheral address is NOT a valid MPSL generic event on those SoCs. */
#if !defined(CONFIG_SOC_SERIES_NRF52)
#error "RSSI FEM requires nRF52 PPI; DPPI FEM needs owned DISABLED publication and radio-domain TIMER"
#endif

/* Follow the public ESB Kconfig timer selection, without private SDK headers.
 * The shared radio owner guarantees SDK timer deinit before this handle is used.
 * A distinct nrfx handle alone does not enforce peripheral ownership. */
#if defined(CONFIG_ESB_SYS_TIMER_INSTANCE_LEADING_ZERO)
#define SCAN_TIMER_ID NRFX_CONCAT_2(0, CONFIG_ESB_SYS_TIMER_INSTANCE)
#else
#define SCAN_TIMER_ID CONFIG_ESB_SYS_TIMER_INSTANCE
#endif
static nrfx_timer_t scan_timer = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(SCAN_TIMER_ID));
/* MPSL keeps the event pointers until the matching configuration clear, which
 * can happen long after prepare() returns. Automatic storage would hand it a
 * reused stack frame, so the registered configuration must have static
 * lifetime. */
static mpsl_fem_event_t scan_activate_event;
static mpsl_fem_event_t scan_deactivate_event;
static bool timer_owned;
static bool fem_owned;
static bool cleanup_fault;

/* Conservative preparation horizon, exceeding ESB's 124-us non-fast RX
 * horizon. MPSL chooses assertion using the same installed DT frontend
 * configuration as normal reception. CC2 belongs to MPSL; CC3 independently
 * proves the full settling horizon before raw RXEN. */
#define FEM_SETTLED_US 140U
#define FEM_WAIT_TIMEOUT_US 1000U
#if DT_NODE_HAS_PROP(DT_PHANDLE(DT_NODELABEL(radio), fem), crx_settle_time_us)
BUILD_ASSERT(DT_PROP(DT_PHANDLE(DT_NODELABEL(radio), fem), crx_settle_time_us) < FEM_SETTLED_US,
	     "FEM RX settling exceeds scan preparation horizon");
#endif
bool rssi_scan_fem_is_safe(void)
{
	return !timer_owned && !fem_owned && !cleanup_fault;
}


int rssi_scan_fem_release(void)
{
	int err = 0;
	if (timer_owned) {
		nrfx_timer_disable(&scan_timer);
		nrf_timer_shorts_set(scan_timer.p_reg, 0);
		nrfx_timer_clear(&scan_timer);
	}
	if (fem_owned) {
		/* Configuration can be partially installed even when set failed. Stop
		 * its source first, then remove routing and force the physical output off. */
		if (mpsl_fem_lna_configuration_clear()) err = -EIO;
		mpsl_fem_deactivate_now(MPSL_FEM_ALL);
		if (mpsl_fem_disable()) err = -EIO;
		fem_owned = false;
	}
	if (timer_owned) {
		nrfx_timer_uninit(&scan_timer);
		timer_owned = false;
	}
	/* A subsequent no-op release must not hide an earlier failed cleanup. */
	cleanup_fault |= err != 0;
	return cleanup_fault ? -EIO : 0;
}

int rssi_scan_fem_prepare(void)
{
	if (cleanup_fault) return -EIO;
	if (timer_owned || fem_owned || NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled)
		return -EBUSY;
	nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(1000000);
	int err = nrfx_timer_init(&scan_timer, &config, NULL);
	if (err) return err;
	timer_owned = true;
	nrfx_timer_clear(&scan_timer);
	nrf_timer_shorts_set(scan_timer.p_reg, 0);
	nrf_timer_event_clear(scan_timer.p_reg, NRF_TIMER_EVENT_COMPARE2);
	nrf_timer_event_clear(scan_timer.p_reg, NRF_TIMER_EVENT_COMPARE3);
	const mpsl_fem_event_t activate = {
		.type = MPSL_FEM_EVENT_TYPE_TIMER,
		.event.timer = {
			.p_timer_instance = scan_timer.p_reg,
			.counter_period.end = FEM_SETTLED_US,
			.compare_channel_mask = BIT(2),
		},
	};
	const mpsl_fem_event_t deactivate = {
		.type = MPSL_FEM_EVENT_TYPE_GENERIC,
		.event.generic.event = nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_DISABLED),
	};
	scan_activate_event = activate;
	scan_deactivate_event = deactivate;
	mpsl_fem_enable();
	fem_owned = true;
	err = mpsl_fem_lna_configuration_set(&scan_activate_event, &scan_deactivate_event);
	if (err) goto fail;
	nrf_timer_cc_set(scan_timer.p_reg, NRF_TIMER_CC_CHANNEL3, FEM_SETTLED_US);
	nrf_timer_shorts_set(scan_timer.p_reg,
		NRF_TIMER_SHORT_COMPARE3_STOP_MASK | NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK);
	uint32_t start = k_cycle_get_32();
	nrfx_timer_enable(&scan_timer);
	for (;;) {
		if (nrf_timer_event_check(scan_timer.p_reg, NRF_TIMER_EVENT_COMPARE3)) return 0;
		if (k_cyc_to_us_floor32(k_cycle_get_32() - start) >= FEM_WAIT_TIMEOUT_US) {
			/* Hardware can complete while the thread is preempted at expiry. */
			if (nrf_timer_event_check(scan_timer.p_reg, NRF_TIMER_EVENT_COMPARE3)) return 0;
			err = -ETIMEDOUT;
			break;
		}
		k_busy_wait(1);
	}
fail:
	if (rssi_scan_fem_release()) return -EIO;
	return err;
}
#endif
