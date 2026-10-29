#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define MAX_TRACKERS 16
#define BIT(n) (1U << (n))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static char output[8192];
static size_t output_length;

static void capture(const char *format, ...)
{
	va_list arguments;
	va_start(arguments, format);
	int written = vsnprintf(output + output_length, sizeof(output) - output_length, format, arguments);
	va_end(arguments);
	assert(written >= 0 && (size_t)written < sizeof(output) - output_length);
	output_length += (size_t)written;
}

#define printk(...) capture(__VA_ARGS__)
#define LOG_INF(...) do { capture(__VA_ARGS__); capture("\n"); } while (0)
#define LOG_WRN(...) LOG_INF(__VA_ARGS__)

/* PRODUCTION HEALTH EXPORT */

#if defined(CONFIG_TDMA_DIAGNOSTICS)
/* PRODUCTION TYPES */

static struct packet_stats tracker_stats[MAX_TRACKERS];
static uint8_t stored_trackers = 4;
static uint8_t tdma_dynamic_active_count = 2;
static uint8_t tdma_dynamic_slot_ticks = 48;
static uint8_t tdma_config_epoch = 7;
static uint32_t tdma_active_mask = 5;
static uint32_t tdma_config_packed[MAX_TRACKERS];
static uint8_t tdma_cap_level = 1;
static uint8_t tdma_loss_trigger_streak_ticks = 2;
static uint8_t tdma_loss_recover_streak_ticks = 3;
static uint8_t tdma_loss_probe_streak_ticks = 4;

typedef uint32_t atomic_t;
static atomic_t tdma_shadow_desired_mask = 13;
static atomic_t tdma_shadow_pending_join_mask = 2;
static atomic_t tdma_shadow_pending_leave_mask = 4;
static atomic_t tdma_shadow_ever_ping_mask = 5;
static atomic_t tdma_shadow_ever_data_mask = 9;
static atomic_t tdma_shadow_desired_changed_at_ms = 8000;
static unsigned int irq_depth;

static uint32_t atomic_get(const atomic_t *value) { return *value; }
static unsigned int irq_lock(void) { return irq_depth++; }
static void irq_unlock(unsigned int key) { assert(irq_depth == key + 1); irq_depth = key; }
static int64_t k_uptime_get(void) { return 10000; }
static int64_t k_uptime_ticks(void) { return 327680; }
static int64_t k_ticks_to_ms_floor64(uint64_t ticks) { return (int64_t)(ticks * 1000 / 32768); }
static uint32_t hid_get_current_tps(void) { return 8; }
static uint32_t hid_get_total_drop_count(void) { return 11; }
static uint32_t hid_get_total_tracker_drop_count(uint8_t id) { return 20U + id; }

struct esb_rx_diagnostics {
	uint32_t crc_failures;
	uint32_t rx_fifo_full;
	uint32_t invalid_payload_length;
	uint32_t duplicates;
};
static int radio_error;
static int esb_get_rx_diagnostics(struct esb_rx_diagnostics *diagnostics)
{
	*diagnostics = (struct esb_rx_diagnostics){1, 2, 3, 4};
	return radio_error;
}
#endif

/* PRODUCTION HEALTH */

static void health_command(void)
{
/* PRODUCTION HEALTH COMMAND */
}

#if defined(CONFIG_TDMA_DIAGNOSTICS)
static const char expected_prefix[] =
	"HEALTH TDMA source=data_ping_shadow stored=4 active=2 mask=0x0005 desired=0x000d recent=0x0001 slot=48 epoch=7 TPS=9 HID=8 recv=12 gaps=3 hid_drop_total=11 cap=180 lvl=1/4 loss_pm=217 trig=2 rec=3 probe=4 step_ms=7654\n"
	"HEALTH observed=0x0003\n"
	"HEALTH SHADOW desired=0x000d join=0x000a leave=0x0004 ping=0x0005 data=0x0009 stable_ms=2000\n";
static const char expected_rx[] =
	"HEALTH RX trk=0 slot=1 pred=2 n=4 adj_min_p50_p99_max=-2/0/3/3 raw_min_p50_p99_max=-4/-4/6/6 rssi_p10_p50=-70/-70 viol=2\n";
static const char expected_tail[] =
	"HEALTH trk=0 TPS=7 recv=12 gaps=3 restart=2 age_ms=1000 hid_drop_total=20\n"
	"HEALTH trk=1 TPS=2 recv=0 gaps=0 restart=0 age_ms=6000 hid_drop_total=21\n";

static void expect_snapshot(const char *radio, const char *sync)
{
	char expected[8192];
	int written = snprintf(expected, sizeof(expected), "%s%s%s%s%s",
		expected_prefix, radio, expected_rx, sync, expected_tail);
	assert(written > 0 && (size_t)written < sizeof(expected));
	if (strcmp(output, expected) != 0) {
		fprintf(stderr, "Expected:\n%sActual:\n%s", expected, output);
		assert(false);
	}
	assert(irq_depth == 0);
}

static void test_percentiles(void)
{
	struct tdma_stats stats = {0};
	assert(receiver_phase_percentile(stats.phase_hist, 0, 50, 100) == 0);
	assert(receiver_rssi_percentile(&stats, 10, 100) == 0);
	stats.phase_hist[0] = 1;
	stats.phase_hist[64] = 1;
	stats.phase_hist[127] = 1;
	assert(receiver_phase_percentile(stats.phase_hist, 3, 50, 100) == 0);
	assert(receiver_phase_percentile(stats.phase_hist, 3, 99, 100) == 63);
	assert(receiver_phase_percentile(stats.phase_hist, 3, 1, 100) == -64);
	stats.rssi_count = 3;
	stats.rssi_hist[40] = 1;
	stats.rssi_hist[70] = 1;
	stats.rssi_hist[100] = 1;
	assert(receiver_rssi_percentile(&stats, 10, 100) == -100);
	assert(receiver_rssi_percentile(&stats, 50, 100) == -70);
	assert(receiver_rssi_percentile(&stats, 90, 100) == -40);
	memset(stats.phase_hist, 0, sizeof(stats.phase_hist));
	memset(stats.rssi_hist, 0, sizeof(stats.rssi_hist));
	assert(receiver_phase_percentile(stats.phase_hist, 1, 99, 100) == 63);
	assert(receiver_rssi_percentile(&stats, 10, 100) == -127);
}

static void test_health(void)
{
	tracker_stats[0] = (struct packet_stats){
		.total_received = 12, .total_gaps = 3, .restart_events = 2,
		.last_packet_time = 9000, .current_tps = 7,
	};
	tracker_stats[1] = (struct packet_stats){
		.duplicate_packets = 1, .last_packet_time = 4000, .current_tps = 2,
	};
	tdma_config_packed[0] = 1U << 24;
	tdma_config_packed[2] = 0;
	tdma_loss_last_permille = 217;
	tdma_last_loss_step_ms = 7654;
	g_tdma_stats[0] = (struct tdma_stats){
		.count = 4, .min_offset = -2, .max_offset = 3,
		.raw_min_offset = -4, .raw_max_offset = 6, .violations = 2,
		.rssi_count = 4,
	};
	g_tdma_stats[0].phase_hist[62] = 1;
	g_tdma_stats[0].phase_hist[64] = 2;
	g_tdma_stats[0].phase_hist[67] = 1;
	g_tdma_stats[0].raw_phase_hist[60] = 2;
	g_tdma_stats[0].raw_phase_hist[70] = 2;
	g_tdma_stats[0].rssi_hist[40] = 1;
	g_tdma_stats[0].rssi_hist[70] = 3;
	g_last_ping_rx_time_diff_ticks[0] = 5;
	g_last_ping_rx_time_diff_valid[0] = true;
	g_bias_ppb_valid[0] = 1;
	g_bias_ppb[0] = -120;
	g_last_ping_isr_rx_ticks_raw[0] = 294912;
	g_ping_reject_count[0] = 6;
	g_extrap_skip_count[0] = 7;
	struct packet_stats packets_before[MAX_TRACKERS];
	struct tdma_stats phases_before[MAX_TRACKERS];
	memcpy(packets_before, tracker_stats, sizeof(packets_before));
	memcpy(phases_before, g_tdma_stats, sizeof(phases_before));
	health_command();
	expect_snapshot("HEALTH RADIO crc=1 fifo_full=2 invalid_len=3 dup=4\n",
		"HEALTH SYNC trk=0 bias=5 skew_ppb=-120 bias_valid=1 skew_valid=1 ping_age_ms=1000 ping_rej=6 extrap_skip=7\n");
	assert(memcmp(packets_before, tracker_stats, sizeof(packets_before)) == 0);
	assert(memcmp(phases_before, g_tdma_stats, sizeof(phases_before)) == 0);

	output_length = 0;
	output[0] = '\0';
	radio_error = -5;
	g_last_ping_isr_rx_ticks_raw[0] = 0;
	g_last_ping_rx_time_diff_valid[0] = false;
	g_bias_ppb_valid[0] = 0;
	health_command();
	expect_snapshot("HEALTH RADIO unavailable err=-5\n",
		"HEALTH SYNC trk=0 bias=5 skew_ppb=0 bias_valid=0 skew_valid=0 ping_age_ms=-1 ping_rej=6 extrap_skip=7\n");
}
#endif

int main(void)
{
#if defined(CONFIG_TDMA_DIAGNOSTICS)
	test_percentiles();
	test_health();
	puts("tdma_health diagnostics-on passed");
#else
	health_command();
	assert(strcmp(output, "health requires CONFIG_TDMA_DIAGNOSTICS=y\n") == 0);
	puts("tdma_health diagnostics-off passed");
#endif
	return 0;
}
