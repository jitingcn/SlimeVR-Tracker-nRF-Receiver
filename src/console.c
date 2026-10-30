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
#include "globals.h"
#include "system/system.h"
#include "build_defines.h"
#include "parse_args.h"

#define USB DT_NODELABEL(usbd)
#if DT_NODE_HAS_STATUS(USB, okay)

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/util.h>
#include "connection/esb.h"
#include "connection/auto_channel.h"
#include "console_send.h"
#include "data_collect.h"
#include "esb_ota.h"
#include "rcv_cmd.h"
#include "rcv_hid_cmd.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

#define CONSOLE_LINE_QUEUE_DEPTH 4
#define CONSOLE_ECHO_BUFFER_SIZE 512
#define CONSOLE_INPUT_DRAIN_MAX 4096
#define CONSOLE_LINE_MAX_LEN CONFIG_CONSOLE_INPUT_MAX_LINE_LEN

BUILD_ASSERT(CONSOLE_LINE_MAX_LEN >= 2, "Console line buffer must hold an empty line");

struct console_line_message {
	uint32_t epoch;
	char line[CONSOLE_LINE_MAX_LEN];
};

enum console_escape_state {
	CONSOLE_ESCAPE_NONE,
	CONSOLE_ESCAPE_START,
	CONSOLE_ESCAPE_CSI,
	CONSOLE_ESCAPE_SS3,
};

struct console_input_state {
	struct k_spinlock lock;
	bool initialized;
	bool active;
	bool overflow;
	bool last_was_cr;
	enum console_escape_state escape_state;
	bool escape_has_value;
	bool escape_ignore_value;
	uint16_t escape_value;
	uint16_t cursor;
	uint16_t tail;
	uint32_t epoch;
	char line[CONSOLE_LINE_MAX_LEN];
	uint8_t echo[CONSOLE_ECHO_BUFFER_SIZE];
	uint16_t echo_head;
	uint16_t echo_tail;
};

static const struct device *const console_uart_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static struct console_input_state console_input;
static struct k_thread console_thread_id;
static K_THREAD_STACK_DEFINE(console_thread_stack, 2048);
static bool console_thread_started;
K_MSGQ_DEFINE(console_line_msgq, sizeof(struct console_line_message),
	      CONSOLE_LINE_QUEUE_DEPTH, 4);

static void console_thread(void);
static void console_uart_irq(const struct device *dev, void *user_data);

#define DFU_EXISTS (CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER || CONFIG_BOOTLOADER_MCUBOOT)

static const char *meows[] = {
	"Mew", "Meww", "Meow", "Meow meow", "Mrrrp", "Mrrf", "Mreow", "Mrrrow", "Mrrr", "Purr",
	"mew", "meww", "meow", "meow meow", "mrrrp", "mrrf", "mreow", "mrrrow", "mrrr", "purr",
};

static const char *meow_punctuations[] = {".", "?", "!", "-", "~", ""};

static const char *meow_suffixes[]
	= {" :3", " :3c", " ;3", " ;3c", " x3", " x3c", " X3", " X3c", " >:3", " >:3c", " >;3", " >;3c", ""};

static void print_meow(void)
{
	int64_t ticks = k_uptime_ticks();

	ticks %= ARRAY_SIZE(meows) * ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes); // silly number generator
	uint8_t meow = ticks / (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	ticks %= (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	uint8_t punctuation = ticks / ARRAY_SIZE(meow_suffixes);
	uint8_t suffix = ticks % ARRAY_SIZE(meow_suffixes);

	printk("%s%s%s\n", meows[meow], meow_punctuations[punctuation], meow_suffixes[suffix]);
}

static void print_channelauto_exclusions(uint64_t excluded)
{
	bool any = false;
	for (unsigned ch = 0; ch <= 88; ch += 2) {
		if ((excluded & (UINT64_C(1) << (ch / 2))) != 0) {
			printk(" %u", ch);
			any = true;
		}
	}
	if (!any) {
		printk(" none observed");
	}
	printk("\n");
}

static void print_channelauto_energy(const char *name,
				    const struct auto_channel_energy_status *energy)
{
	printk("  %s: ", name);
	if (energy->peak < 0 || energy->quiet_floor < 0) {
		printk("invalid");
	} else {
		printk("peak=-%d dBm quiet-floor=-%d dBm",
		       energy->peak, energy->quiet_floor);
	}
	printk(" dwell-lower-bound=%u us samples=%u clean-visits=%u attempts=%u\n",
	       (unsigned)energy->coverage_us, (unsigned)energy->samples,
	       (unsigned)energy->visits, (unsigned)energy->attempts);
}

static void print_channelauto_status(void)
{
	/* Only the console thread formats this bounded copy; keep it off its stack. */
	static struct auto_channel_snapshot snapshot;
	auto_channel_get_snapshot(&snapshot);
	const char *state = !snapshot.initialized ? "uninitialized/Kconfig-disabled" :
		snapshot.radio_fault ? "radio fault" :
		snapshot.entropy_fault ? "entropy fault" :
		snapshot.interference_latched ? "interference-stopped" :
		snapshot.enabled ? "active" : "off/pinned";
	printk("Automatic channel avoidance: %s (snapshot at %lld ms)\n",
	       state, (long long)snapshot.now_ms);
	printk("  Receiver %02X%02X%02X%02X%02X%02X HOME=%u generation=%u phase=%s\n",
	       snapshot.identity[5], snapshot.identity[4], snapshot.identity[3],
	       snapshot.identity[2], snapshot.identity[1], snapshot.identity[0],
	       snapshot.home, (unsigned)snapshot.generation,
	       snapshot.phase ? snapshot.phase : "unavailable");
	printk("  Recovery required=0x%08X ready=0x%08X recovered=0x%08X missing=0x%08X layout-epoch=%u cap-pending=%s\n",
	       (unsigned)snapshot.required_mask, (unsigned)snapshot.ready_mask,
	       (unsigned)snapshot.recovered_mask,
	       (unsigned)(snapshot.required_mask & ~snapshot.recovered_mask),
	       snapshot.layout_epoch, snapshot.cap_pending ? "yes" : "no");
	if (!snapshot.initialized || !snapshot.enabled || snapshot.interference_latched ||
	    snapshot.radio_fault || snapshot.entropy_fault || snapshot.pairing) {
		printk("  Broadcast blocked:%s%s%s%s%s%s\n",
		       !snapshot.initialized ? " uninitialized/Kconfig-disabled" : "",
		       !snapshot.enabled ? " auto-off/manual-pin" : "",
		       snapshot.interference_latched ? " interference-stop" : "",
		       snapshot.radio_fault ? " radio-fault" : "",
		       snapshot.entropy_fault ? " entropy-fault" : "",
		       snapshot.pairing ? " pairing" : "");
	} else {
		printk("  HOME-only broadcasts eligible; radio busy may defer them.\n");
	}
	printk("  Beacon TX attempts=%u local-completions=%u; delivery unknown (no ACK).\n",
	       (unsigned)snapshot.tx_attempts, (unsigned)snapshot.tx_completed);
	unsigned fresh = 0;
	for (unsigned i = 0; i < snapshot.peer_count; i++) {
		const struct auto_channel_peer_status *peer = &snapshot.peers[i];
		int64_t age = snapshot.now_ms - peer->last_seen_ms;
		fresh += peer->fresh;
		printk("  Peer %02X%02X%02X%02X%02X%02X HOME=%u age=%lld ms observations=%u seq=%u %s\n",
		       peer->identity[5], peer->identity[4], peer->identity[3],
		       peer->identity[2], peer->identity[1], peer->identity[0],
		       peer->home, (long long)age, peer->observations, peer->sequence,
		       peer->fresh ? "fresh" : "expired");
	}
	if (!fresh) {
		printk("  No fresh peers observed on this HOME; other channels unknown, not vacant.\n");
	} else {
		printk("  Other channels remain unknown; HOME beacons do not discover them.\n");
	}
	printk("  Current peer exclusions (even channels):");
	print_channelauto_exclusions(snapshot.excluded);
	if (snapshot.table_full) {
		printk("  Fresh peer table full: automatic destinations vetoed.\n");
	}
	if (snapshot.decision.valid) {
		const struct auto_channel_decision_status *decision = &snapshot.decision;
		printk("  Last decision: %s age=%lld ms HOME=%u target=%u generation=%u\n",
		       decision->reason ? decision->reason : "unavailable",
		       (long long)(snapshot.now_ms - decision->time_ms),
		       decision->home, decision->target, (unsigned)decision->generation);
		print_channelauto_energy("HOME evidence", &decision->reference);
		print_channelauto_energy("candidate evidence", &decision->candidate);
		printk("  Decision-time peer exclusions:");
		print_channelauto_exclusions(decision->excluded);
	} else {
		printk("  No completed energy decision in this HOME/generation.\n");
	}
	printk("  Moves use local loss and energy evidence, not identity-based turn-taking.\n");
}

static void print_help(void)
{
	printk(
		"\n=== Available Commands ===\n\n"
		"Device Information:\n"
		"  info                       Get device information\n"
		"  uptime                     Get device uptime\n"
		"  list                       Get paired devices\n"
		"\n"
	);

	printk(
		"Device Management:\n"
		"  reboot                     Soft reset the device\n"
		"  add <address>              Manually add a device\n"
		"  remove                     Remove last device\n"
		"  pair [count]               Enter pairing mode\n"
		"    pair                     Pair indefinitely (timeout after %d seconds)\n"
		"    pair 4                   Exit after pairing 4 new devices\n"
		"  exit                       Exit pairing mode\n"
		"  clear                      Clear stored devices\n"
		"\n",
		CONFIG_PAIRING_TIMEOUT
	);

	printk(
		"Statistics:\n"
		"  health                     Print one non-resetting receiver health snapshot\n"
		"  stats                      Toggle detailed packet statistics\n"
		"  stats <seconds>            Show detailed stats for N seconds\n"
		"  resetstats                 Reset packet statistics\n"
		"\n"
	);

	printk(
		"RF Channel (Local Receiver):\n"
		"  channel <0-100>            Set receiver RF channel only\n"
		"    Example: channel 25       Set receiver to channel 25\n"
		"  clearchannel               Clear receiver RF channel (use default)\n"
		"  channelauto <on|off|status> Runtime automatic channel avoidance (off after every reboot)\n"
		"    Manual channel changes disable avoidance; use channelauto on to enable explicitly\n"
		"\n"
	);

	printk(
		"RSSI / Channel Scan:\n"
		"  rssi_scan                  Scan RSSI across preferred channels and print a recommendation\n"
		"\n"
	);

	printk(
		"Remote Commands:\n"
		"  send <id|all> <command>    Send remote command to tracker(s)\n"
		"    Commands: shutdown, calibrate [acc], meow, scan,\n"
		"              mag <on|off|clear|cal|auto on|auto off>, reboot, clear, dfu [ota],\n"
		"              channel <0-100>, clearchannel,\n"
		"              sens <x,y,z|reset|auto <x|y|z> [rev]>,\n"
		"              reset <zro|acc|bat|mag|tcal|fusion>, ping\n"
		"    calibrate: zero-bias; calibrate acc: accelerometer (18 orientations)\n"
		"    6-side: historical alias for calibrate acc\n"
	);

	printk(
		"    Examples:\n"
		"      send 0 shutdown          Shutdown tracker 0\n"
		"      send all calibrate       Zero-bias calibration on all active trackers\n"
		"      send 0 calibrate acc     Accelerometer calibration (18 orientations)\n"
		"      send 1 meow              Make tracker 1 meow\n"
		"      send 2 reboot            Reboot tracker 2\n"
		"      send 0 sens 1.0,1.0,1.0  Set sensitivity for tracker 0\n"
		"      send 0 sens auto z       Auto-calibrate Z sensitivity on tracker 0\n"
		"      send all sens reset      Reset sensitivity for all\n"
		"      send 1 reset zro         Reset ZRO calibration on tracker 1\n"
		"      send all ping            Ping all active trackers\n"
		"      send all wom enter      Request WOM now (accepted does not mean asleep)\n"
		"      send 0 wom timeout      Query normal WOM timeout in minutes\n"
		"      send all wom timeout 10 Set normal timeout (1-35791 minutes)\n"
		"      send all wom timeout 0  Clear saved override; restore firmware default\n"
	);

	printk(
		"      send 3 clear             Clear pairing on tracker 3\n"
		"      send all dfu             Enter UF2 DFU mode on all active trackers\n"
		"      send all dfu ota         Enter OTA DFU mode on all active trackers\n"
		"      send all channel 25      Set all active trackers to channel 25\n"
		"      send all clearchannel    Clear channel for all active trackers\n"
		"\n"
	);

#if DFU_EXISTS
	printk(
		"Bootloader:\n"
		"  dfu [ota]                  Enter the configured DFU bootloader\n"
		"\n"
	);
#endif

	printk(
		"Other:\n"
		"  collect <id>               Start raw sensor data collection from tracker\n"
		"  collectall <rate_hz>       Start batch raw data collection from all paired trackers\n"
		"  collectmeta <id> <mask> <chunk>  Request metadata sections (mask 0x01-0x3f, chunk 0-255)\n"
		"  collect off                Stop data collection\n"
		"  collectstop                Stop batch data collection\n"
		"  collect                    Show data collection status\n"
		"  ota                        Show OTA update status\n"
		"  ota info <id>              Query firmware info from tracker\n"
		"  ota abort                  Abort active OTA session\n"
		"  meow                       Meow!\n"
		"  help                       Show this help message\n"
		"\n"
	);

	printk(
		"Button Functions:\n"
		"  Short press (1x):          Status check\n"
		"  Quick press (2x):          Exit pairing mode\n"
		"  Quick press (3x):          Enter pairing mode\n"
		"  Long press (5s):           Clear all pairings\n"
	);

#if DFU_EXISTS
	printk("  Long press (10s):          Enter DFU mode\n");
#endif
	printk("\n");
}

static inline void strtolower(char *str)
{
	for (int i = 0; str[i] != '\0'; i++) {
		str[i] = (char)tolower((unsigned char)str[i]);
	}
}

static bool parse_u8_arg(const char *str, uint8_t *value)
{
	char *endptr = NULL;
	unsigned long parsed = strtoul(str, &endptr, 10);

	if (endptr == str || *endptr != '\0' || parsed > UINT8_MAX) {
		return false;
	}

	*value = (uint8_t)parsed;
	return true;
}
static bool console_echo_has_data_locked(void)
{
	return console_input.echo_head != console_input.echo_tail;
}

static void console_echo_put_locked(uint8_t byte)
{
	uint16_t next = (uint16_t)((console_input.echo_head + 1U) % CONSOLE_ECHO_BUFFER_SIZE);

	if (next == console_input.echo_tail) {
		return;
	}

	console_input.echo[console_input.echo_head] = byte;
	console_input.echo_head = next;
}

static void console_echo_text_locked(const char *text)
{
	while (*text != '\0') {
		console_echo_put_locked((uint8_t)*text++);
	}
}

static void console_echo_cursor_locked(uint8_t direction, uint16_t count)
{
	if (count == 0U) {
		return;
	}

	console_echo_put_locked(0x1b);
	console_echo_put_locked('[');
	if (count >= 100U) {
		console_echo_put_locked((uint8_t)('0' + count / 100U));
		count %= 100U;
		console_echo_put_locked((uint8_t)('0' + count / 10U));
		console_echo_put_locked((uint8_t)('0' + count % 10U));
	} else if (count >= 10U) {
		console_echo_put_locked((uint8_t)('0' + count / 10U));
		console_echo_put_locked((uint8_t)('0' + count % 10U));
	} else {
		console_echo_put_locked((uint8_t)('0' + count));
	}
	console_echo_put_locked(direction);
}

static void console_reset_line_locked(void)
{
	console_input.overflow = false;
	console_input.last_was_cr = false;
	console_input.escape_state = CONSOLE_ESCAPE_NONE;
	console_input.escape_has_value = false;
	console_input.escape_ignore_value = false;
	console_input.escape_value = 0;
	console_input.cursor = 0;
	console_input.tail = 0;
}

static void console_drop_queued_lines_locked(void)
{
	struct console_line_message dropped;

	while (k_msgq_get(&console_line_msgq, &dropped, K_NO_WAIT) == 0) {
	}
}

static void console_finish_line_locked(void)
{
	struct console_line_message message = {0};
	uint16_t length = (uint16_t)(console_input.cursor + console_input.tail);

	if (!console_input.overflow) {
		message.epoch = console_input.epoch;
		memcpy(message.line, console_input.line, length);
		message.line[length] = '\0';
		if (k_msgq_put(&console_line_msgq, &message, K_NO_WAIT) != 0) {
			console_echo_put_locked('\a');
		}
	}

	console_echo_text_locked("\r\n");
	console_reset_line_locked();
}

static void console_insert_char_locked(uint8_t byte)
{
	uint16_t length = (uint16_t)(console_input.cursor + console_input.tail);

	if (length >= CONSOLE_LINE_MAX_LEN - 1U) {
		if (!console_input.overflow) {
			console_input.overflow = true;
			console_echo_put_locked('\a');
		}
		return;
	}

	for (uint16_t i = length; i > console_input.cursor; i--) {
		console_input.line[i] = console_input.line[i - 1U];
	}
	console_input.line[console_input.cursor++] = (char)byte;

	console_echo_put_locked(byte);
	if (console_input.tail != 0U) {
		console_echo_text_locked("\x1b[s");
		for (uint16_t i = console_input.cursor;
		     i < console_input.cursor + console_input.tail; i++) {
			console_echo_put_locked((uint8_t)console_input.line[i]);
		}
		console_echo_text_locked("\x1b[u");
	}
}

static void console_backspace_locked(void)
{
	uint16_t length;

	if (console_input.cursor == 0U) {
		return;
	}

	length = (uint16_t)(console_input.cursor + console_input.tail);
	console_input.cursor--;
	for (uint16_t i = console_input.cursor; i + 1U < length; i++) {
		console_input.line[i] = console_input.line[i + 1U];
	}

	console_echo_put_locked('\b');
	if (console_input.tail == 0U) {
		console_echo_text_locked(" \b");
	} else {
		console_echo_text_locked("\x1b[s");
		for (uint16_t i = console_input.cursor;
		     i < console_input.cursor + console_input.tail; i++) {
			console_echo_put_locked((uint8_t)console_input.line[i]);
		}
		console_echo_text_locked(" \x1b[u");
	}
}

static void console_delete_locked(void)
{
	uint16_t length;

	if (console_input.tail == 0U) {
		return;
	}

	length = (uint16_t)(console_input.cursor + console_input.tail);
	for (uint16_t i = console_input.cursor; i + 1U < length; i++) {
		console_input.line[i] = console_input.line[i + 1U];
	}
	console_input.tail--;

	console_echo_cursor_locked('C', 1);
	console_echo_put_locked('\b');
	if (console_input.tail == 0U) {
		console_echo_text_locked(" \b");
	} else {
		console_echo_text_locked("\x1b[s");
		for (uint16_t i = console_input.cursor;
		     i < console_input.cursor + console_input.tail; i++) {
			console_echo_put_locked((uint8_t)console_input.line[i]);
		}
		console_echo_text_locked(" \x1b[u");
	}
}

static void console_move_home_locked(void)
{
	console_echo_cursor_locked('D', console_input.cursor);
	console_input.tail = (uint16_t)(console_input.tail + console_input.cursor);
	console_input.cursor = 0;
}

static void console_move_end_locked(void)
{
	console_echo_cursor_locked('C', console_input.tail);
	console_input.cursor = (uint16_t)(console_input.cursor + console_input.tail);
	console_input.tail = 0;
}

static void console_apply_escape_locked(uint8_t final)
{
	uint16_t count = console_input.escape_has_value && console_input.escape_value != 0U
		? console_input.escape_value : 1U;

	switch (final) {
	case 'D':
		count = MIN(count, console_input.cursor);
		console_input.cursor -= count;
		console_input.tail = (uint16_t)(console_input.tail + count);
		console_echo_cursor_locked('D', count);
		break;
	case 'C':
		count = MIN(count, console_input.tail);
		console_input.cursor = (uint16_t)(console_input.cursor + count);
		console_input.tail -= count;
		console_echo_cursor_locked('C', count);
		break;
	case 'H':
		console_move_home_locked();
		break;
	case 'F':
		console_move_end_locked();
		break;
	case '~':
		if (console_input.escape_value == 3U) {
			console_delete_locked();
		} else if (console_input.escape_value == 1U ||
			   console_input.escape_value == 7U) {
			console_move_home_locked();
		} else if (console_input.escape_value == 4U ||
			   console_input.escape_value == 8U) {
			console_move_end_locked();
		}
		break;
	default:
		break;
	}
}

static bool console_handle_escape_locked(uint8_t byte)
{
	switch (console_input.escape_state) {
	case CONSOLE_ESCAPE_START:
		if (byte == '[') {
			console_input.escape_state = CONSOLE_ESCAPE_CSI;
			console_input.escape_has_value = false;
			console_input.escape_ignore_value = false;
			console_input.escape_value = 0;
		} else if (byte == 'O') {
			console_input.escape_state = CONSOLE_ESCAPE_SS3;
		} else {
			console_input.escape_state = CONSOLE_ESCAPE_NONE;
		}
		return true;
	case CONSOLE_ESCAPE_CSI:
		if (byte >= 0x30 && byte <= 0x3f) {
			if (byte >= '0' && byte <= '9') {
				if (!console_input.escape_ignore_value) {
					console_input.escape_has_value = true;
					if (console_input.escape_value < 999U) {
						console_input.escape_value =
							(uint16_t)MIN(999U,
								      console_input.escape_value * 10U +
								      (uint16_t)(byte - '0'));
					}
				}
			} else if (byte == ';') {
				console_input.escape_ignore_value = true;
			}
			return true;
		}
		if (byte >= 0x20 && byte <= 0x2f) {
			return true;
		}
		if (byte >= 0x40 && byte <= 0x7e) {
			console_apply_escape_locked(byte);
		}
		console_input.escape_state = CONSOLE_ESCAPE_NONE;
		return true;
	case CONSOLE_ESCAPE_SS3:
		if (byte >= 0x20 && byte <= 0x2f) {
			return true;
		}
		if (byte >= 0x40 && byte <= 0x7e) {
			console_apply_escape_locked(byte);
		}
		console_input.escape_state = CONSOLE_ESCAPE_NONE;
		return true;
	case CONSOLE_ESCAPE_NONE:
	default:
		return false;
	}
}

static void console_input_byte_locked(uint8_t byte)
{
	if (byte == '\n' && console_input.last_was_cr) {
		console_input.last_was_cr = false;
		return;
	}

	if (byte == '\r' || byte == '\n') {
		console_finish_line_locked();
		console_input.last_was_cr = byte == '\r';
		return;
	}

	console_input.last_was_cr = false;
	if (console_input.overflow) {
		return;
	}

	if (console_input.escape_state != CONSOLE_ESCAPE_NONE) {
		(void)console_handle_escape_locked(byte);
		return;
	}

	if (byte == 0x1b) {
		console_input.escape_state = CONSOLE_ESCAPE_START;
		console_input.escape_has_value = false;
		console_input.escape_ignore_value = false;
		console_input.escape_value = 0;
		return;
	}

	switch (byte) {
	case 0x08:
	case 0x7f:
		console_backspace_locked();
		break;
	case '\t':
		break;
	default:
		if (isprint((unsigned char)byte) != 0) {
			console_insert_char_locked(byte);
		}
		break;
	}
}

static void console_echo_flush(const struct device *dev)
{
	while (true) {
		int ready = uart_irq_tx_ready(dev);
		if (ready <= 0) {
			return;
		}

		k_spinlock_key_t key = k_spin_lock(&console_input.lock);
		if (!console_echo_has_data_locked()) {
			k_spin_unlock(&console_input.lock, key);
			uart_irq_tx_disable(dev);
			return;
		}

		uint16_t head = console_input.echo_head;
		uint16_t tail = console_input.echo_tail;
		uint16_t contiguous = head > tail
			? (uint16_t)(head - tail)
			: (uint16_t)(CONSOLE_ECHO_BUFFER_SIZE - tail);
		int count = (int)contiguous;
		int sent = uart_fifo_fill(dev, &console_input.echo[tail], count);
		if (sent > 0) {
			console_input.echo_tail =
				(uint16_t)((tail + (uint16_t)sent) % CONSOLE_ECHO_BUFFER_SIZE);
		}
		bool empty = !console_echo_has_data_locked();
		k_spin_unlock(&console_input.lock, key);

		if (sent <= 0) {
			return;
		}
		if (empty) {
			uart_irq_tx_disable(dev);
			return;
		}
	}
}

static void console_uart_irq(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (dev != console_uart_dev) {
		return;
	}

	while (uart_irq_update(dev) > 0 && uart_irq_is_pending(dev) > 0) {
		if (uart_irq_rx_ready(dev) > 0) {
			do {
				uint8_t byte;
				k_spinlock_key_t key = k_spin_lock(&console_input.lock);
				int received = uart_fifo_read(dev, &byte, 1);

				if (received > 0 && console_input.initialized && console_input.active) {
					console_input_byte_locked(byte);
				}
				k_spin_unlock(&console_input.lock, key);

				if (received <= 0) {
					break;
				}
			} while (uart_irq_rx_ready(dev) > 0);
		}

		if (uart_irq_tx_ready(dev) > 0) {
			console_echo_flush(dev);
		}
	}

	k_spinlock_key_t key = k_spin_lock(&console_input.lock);
	bool echo_pending = console_echo_has_data_locked();
	k_spin_unlock(&console_input.lock, key);
	if (echo_pending) {
		uart_irq_tx_enable(dev);
	} else {
		uart_irq_tx_disable(dev);
	}
}

static void console_drain_uart_locked(void)
{
	unsigned char byte;

	for (size_t i = 0; i < CONSOLE_INPUT_DRAIN_MAX; i++) {
		if (uart_poll_in(console_uart_dev, &byte) != 0) {
			break;
		}
	}
}

static int console_input_install(void)
{
	k_spinlock_key_t key = k_spin_lock(&console_input.lock);
	bool initialized = console_input.initialized;
	k_spin_unlock(&console_input.lock, key);
	if (initialized) {
		return 0;
	}

	if (!device_is_ready(console_uart_dev)) {
		LOG_ERR("Console UART is not ready");
		return -ENODEV;
	}

	uart_irq_rx_disable(console_uart_dev);
	uart_irq_tx_disable(console_uart_dev);
	key = k_spin_lock(&console_input.lock);
	console_drain_uart_locked();
	k_spin_unlock(&console_input.lock, key);

	int ret = uart_irq_callback_user_data_set(console_uart_dev, console_uart_irq, NULL);
	if (ret != 0) {
		LOG_ERR("Failed to install console UART input callback: %d", ret);
		return ret;
	}

	key = k_spin_lock(&console_input.lock);
	console_input.initialized = true;
	k_spin_unlock(&console_input.lock, key);
	uart_irq_rx_enable(console_uart_dev);
	return 0;
}

static bool console_line_is_current(uint32_t epoch)
{
	k_spinlock_key_t key = k_spin_lock(&console_input.lock);
	/* Completed lines remain eligible across ordinary DTR close/reopen. */
	bool current = console_input.epoch == epoch;
	k_spin_unlock(&console_input.lock, key);
	return current;
}

static void console_print_banner(void)
{
	printk("*** " CONFIG_SLIMEVR_USB_DEVICE_MANUFACTURER " " CONFIG_SLIMEVR_USB_DEVICE_PRODUCT " ***\n");
	printk(FW_STRING);
	printk("Repo: %s | Branch: %s\n", FW_GIT_REPO_URL, FW_GIT_BRANCH);
}

void console_serial_start(void)
{
	bool opened = false;
	bool create_thread = false;

	if (console_input_install() != 0) {
		return;
	}

#if defined(CONFIG_DATA_COLLECT) && !defined(CONFIG_DATA_COLLECT_HID)
	static bool data_collect_initialized;
	if (!data_collect_initialized) {
		data_collect_init();
		data_collect_initialized = true;
	}
#endif

	k_spinlock_key_t key = k_spin_lock(&console_input.lock);
	if (console_input.active) {
		k_spin_unlock(&console_input.lock, key);
		return;
	}

	uart_irq_rx_disable(console_uart_dev);
	uart_irq_tx_disable(console_uart_dev);
	console_drain_uart_locked();

	console_input.active = true;
	console_reset_line_locked();
	console_input.echo_head = 0;
	console_input.echo_tail = 0;
	opened = true;
	if (!console_thread_started) {
		console_thread_started = true;
		create_thread = true;
	}
	uart_irq_rx_enable(console_uart_dev);
	k_spin_unlock(&console_input.lock, key);

	if (create_thread) {
		k_thread_create(&console_thread_id, console_thread_stack,
				K_THREAD_STACK_SIZEOF(console_thread_stack),
				(k_thread_entry_t)console_thread, NULL, NULL, NULL,
				CONSOLE_THREAD_PRIORITY, 0, K_NO_WAIT);
	}
	if (opened) {
		console_print_banner();
	}
}

static void console_close_input_locked(void)
{
	console_input.active = false;
	console_reset_line_locked();
	console_input.echo_head = 0;
	console_input.echo_tail = 0;
	if (console_input.initialized) {
		uart_irq_tx_disable(console_uart_dev);
		uart_irq_rx_disable(console_uart_dev);
		console_drain_uart_locked();
	}
}

void console_serial_close(void)
{
	k_spinlock_key_t key = k_spin_lock(&console_input.lock);
	console_close_input_locked();
	k_spin_unlock(&console_input.lock, key);
}

void console_serial_stop(void)
{
	k_spinlock_key_t key = k_spin_lock(&console_input.lock);
	console_close_input_locked();
	console_input.epoch++;
	console_drop_queued_lines_locked();
	k_spin_unlock(&console_input.lock, key);
}

static void console_thread(void)
{

	const char command_info[] = "info";
	const char command_uptime[] = "uptime";
	const char command_list[] = "list";
	const char command_reboot[] = "reboot";
	const char command_add[] = "add";
	const char command_remove[] = "remove";
	const char command_pair[] = "pair";
	const char command_exit[] = "exit";
	const char command_clear[] = "clear";
	const char command_stats[] = "stats";
	const char command_health[] = "health";
	const char command_resetstats[] = "resetstats";
	const char command_channel[] = "channel";
	const char command_clearchannel[] = "clearchannel";
	const char command_channelauto[] = "channelauto";
	const char command_rssi_scan[] = "rssi_scan";
	const char command_send[] = "send";
	const char command_help[] = "help";

#if DFU_EXISTS
	const char command_dfu[] = "dfu";
#endif

	const char command_meow[] = "meow";
	const char command_collectall[] = "collectall";
	const char command_collectstop[] = "collectstop";
	const char command_collect[] = "collect";
	const char command_collectmeta[] = "collectmeta";
	const char command_ota[] = "ota";

	while (1) {
		struct console_line_message message;
		k_msgq_get(&console_line_msgq, &message, K_FOREVER);
		if (!console_line_is_current(message.epoch)) {
			continue;
		}
		char *line = message.line;
		char *argv[8] = {NULL};
		size_t argc = parse_args(line, argv, ARRAY_SIZE(argv));
		if (argc == 0) {
			continue;
		}
		for (size_t i = 0; i < argc; i++) {
			strtolower(argv[i]);
		}

		char *arg = argc > 1 ? argv[1] : NULL;
		char *arg2 = argc > 2 ? argv[2] : NULL;
		char *arg3 = argc > 3 ? argv[3] : NULL;
		char *arg4 = argc > 4 ? argv[4] : NULL;
		char *arg5 = argc > 5 ? argv[5] : NULL;

		if (strcmp(argv[0], command_help) == 0) {
			print_help();
		} else if (strcmp(argv[0], command_info) == 0) {
			rcv_cmd_info();
		} else if (strcmp(argv[0], command_uptime) == 0) {
			rcv_cmd_uptime();
		} else if (strcmp(argv[0], command_add) == 0) {
			if (argc != 2) {
				printk("Invalid number of arguments\n");
				continue;
			}
			uint64_t addr = parse_u64(arg, 16);
			char buf[13];
			snprintk(buf, 13, "%012llx", addr);
			if (addr != 0 && strcmp(buf, arg) == 0) {
				int8_t slot = -1;
				uint8_t st = rcv_cmd_add(addr, &slot);
				if (st == RCV_HID_ST_OK) {
					printk("Tracker stored in slot %d\n", slot);
				} else if (st == RCV_HID_ST_ENOSPC) {
					printk("Tracker list is full\n");
				} else {
					printk("Invalid tracker address\n");
				}
			} else {
				printk("Invalid address\n");
			}
		} else if (strcmp(argv[0], command_remove) == 0) {
			rcv_cmd_remove();
		} else if (strcmp(argv[0], command_list) == 0) {
			rcv_cmd_list();
		} else if (strcmp(argv[0], command_reboot) == 0) {
			rcv_cmd_reboot();
		} else if (strcmp(argv[0], command_pair) == 0) {
			if (!arg) {
				rcv_cmd_pair(0);
				printk("Pairing mode enabled (auto-exit after %d seconds)\n", CONFIG_PAIRING_TIMEOUT);
			} else {
				long count;
				if (!parse_long_bounded(arg, 0, 255, &count)) {
					printk("Invalid count. Usage: pair [count]\n");
					printk("  pair       - Pair indefinitely (timeout after %d seconds)\n", CONFIG_PAIRING_TIMEOUT);
					printk("  pair 4     - Exit after pairing 4 new devices\n");
				} else if (count == 0) {
					rcv_cmd_pair(0);
					printk("Pairing mode enabled (auto-exit after %d seconds)\n", CONFIG_PAIRING_TIMEOUT);
				} else {
					rcv_cmd_pair((uint8_t)count);
					printk(
						"Pairing mode enabled (auto-exit after %u new devices or %d seconds)\n",
						(uint8_t)count,
						CONFIG_PAIRING_TIMEOUT
					);
				}
			}
		} else if (strcmp(argv[0], command_exit) == 0) {
			rcv_cmd_exit_pair();
		} else if (strcmp(argv[0], command_clear) == 0) {
			rcv_cmd_clear();
		} else if (strcmp(argv[0], command_health) == 0) {
#if defined(CONFIG_TDMA_DIAGNOSTICS)
			esb_print_health_snapshot();
#else
			printk("health requires CONFIG_TDMA_DIAGNOSTICS=y\n");
#endif
		} else if (strcmp(argv[0], command_stats) == 0) {
			if (!arg) {
				rcv_cmd_stats(0);
				if (esb_get_stats_detailed_enabled()) {
					printk("Detailed stats enabled (toggle again to disable)\n");
				} else {
					printk("Detailed stats disabled\n");
				}
			} else {
				long duration;
				if (!parse_long_bounded(arg, 0, 86400, &duration)) {
					printk("Invalid duration. Usage: stats [seconds]\n");
					printk("  stats       - Toggle detailed stats on/off\n");
					printk("  stats 30    - Show detailed stats for 30 seconds\n");
				} else if (duration == 0) {
					rcv_cmd_stats(0);
					if (esb_get_stats_detailed_enabled()) {
						printk("Detailed stats enabled (toggle again to disable)\n");
					} else {
						printk("Detailed stats disabled\n");
					}
				} else {
					rcv_cmd_stats((uint32_t)duration);
					printk("Detailed stats enabled for %ld seconds\n", duration);
				}
			}
		} else if (strcmp(argv[0], command_resetstats) == 0) {
			rcv_cmd_resetstats();
		} else if (strcmp(argv[0], command_channelauto) == 0) {
			if (arg2 || (arg && strcmp(arg, "on") != 0 &&
				     strcmp(arg, "off") != 0 && strcmp(arg, "status") != 0)) {
				printk("Usage: channelauto <on|off|status>\n");
			} else {
				if (arg && strcmp(arg, "status") != 0) {
					auto_channel_set_enabled(strcmp(arg, "on") == 0);
				}
				print_channelauto_status();
			}
		} else if (strcmp(argv[0], command_rssi_scan) == 0) {
			/* Use the same radio-operation busy gate as HID. */
			uint8_t st = rcv_cmd_rssi_scan();
			if (st == RCV_HID_ST_EBUSY) {
				printk("RSSI scan already in progress\n");
			} else if (st == RCV_HID_ST_STARTED) {
				printk("RSSI scan started\n");
			}
		} else if (strcmp(argv[0], command_channel) == 0) {
			if (!arg) {
				printk("Usage: channel <0-100>\n");
				printk("Example: channel 25 - Set receiver RF channel to 25 (local only)\n");
			} else {
				long channel;
				if (!parse_long_bounded(arg, 0, 100, &channel)) {
					printk("Invalid channel. Must be a number between 0 and 100.\n");
				} else if (rcv_cmd_channel_set((uint8_t)channel) == RCV_HID_ST_OK) {
					printk("Receiver RF channel set to %d (local only)\n", (int)channel);
				} else {
					printk("Receiver RF channel change failed or radio is busy\n");
				}
			}
		} else if (strcmp(argv[0], command_clearchannel) == 0) {
			uint8_t st = rcv_cmd_channel_clear();
			if (st == RCV_HID_ST_OK) {
				printk("Receiver RF channel cleared (local only)\n");
			} else {
				printk("Receiver RF channel clear failed (status %u)\n", st);
			}
		} else if (strcmp(argv[0], command_send) == 0) {
			console_handle_send(arg, arg2, arg3, arg4, arg5);
		}
#if DFU_EXISTS
		else if (strcmp(argv[0], command_dfu) == 0) {
			bool ota = false;
			if (arg) {
				if (strcmp(arg, "ota") == 0) {
					ota = true;
				} else {
					printk("Unknown dfu argument: %s (use 'ota' or omit it)\n", arg);
					continue;
				}
			}
			if (rcv_cmd_dfu(ota) == RCV_HID_ST_ENOTSUP) {
				printk("DFU not available on this build\n");
			}
		}
#endif
		else if (strcmp(argv[0], command_collectmeta) == 0) {
			uint8_t id, mask, chunk;
			if (argc != 4 || !parse_u8_arg(arg, &id) || !parse_u8_arg(arg2, &mask) ||
			    !parse_u8_arg(arg3, &chunk) || mask == 0 || (mask & ~ESB_METADATA_MASK_VALID) != 0) {
				printk("Usage: collectmeta <tracker_id> <mask 1-63> <chunk 0-255>\n");
			} else {
				uint8_t st = rcv_cmd_collect_meta(id, mask, chunk);
				printk("collectmeta tracker=%u mask=0x%02x chunk=%u status=%u\n", id, mask, chunk, st);
			}
		} else if (strcmp(argv[0], command_collectall) == 0) {
#ifdef CONFIG_DATA_COLLECT
			uint8_t rate;
			if (!arg || !parse_u8_arg(arg, &rate)) {
				printk("Invalid rate. Must be 0-255 Hz.\n");
			} else if (rcv_cmd_collect_batch_start(rate) == RCV_HID_ST_OK) {
				printk("Batch data collection started at %u Hz\n", rate);
			}
#else
			printk("Data collection not available (build with CONFIG_DATA_COLLECT=y)\n");
#endif
		} else if (strcmp(argv[0], command_collectstop) == 0) {
#ifdef CONFIG_DATA_COLLECT
			rcv_cmd_collect_batch_stop();
			printk("Batch data collection stopped\n");
#else
			printk("Data collection not available (build with CONFIG_DATA_COLLECT=y)\n");
#endif
		} else if (strcmp(argv[0], command_meow) == 0) {
			print_meow();
		} else if (strcmp(argv[0], command_collect) == 0) {
#ifdef CONFIG_DATA_COLLECT
			if (arg && strcmp(arg, "off") == 0) {
				bool stopped_any = false;
				if (data_collect_batch_is_active()) {
					rcv_cmd_collect_batch_stop();
					printk("Batch data collection stopped\n");
					stopped_any = true;
				}
				if (data_collect_is_active()) {
					uint8_t tid = data_collect_get_target_id();
					rcv_cmd_collect_stop();
					printk("Data collection stopped, sent OFF to tracker %u\n", tid);
					stopped_any = true;
				}
				if (!stopped_any) {
					printk("Data collection is not active\n");
				}
			} else if (arg) {
				uint8_t id;
				if (parse_u8_arg(arg, &id) && id < 255) {
					if (rcv_cmd_collect_start(id) == RCV_HID_ST_OK) {
						printk("Data collection started for tracker %u\n", (unsigned)id);
						printk("Test mode enabled on tracker (prevents sleep)\n");
						printk("Non-target trackers will receive SHUTDOWN\n");
						printk("Use 'collect off' to stop\n");
					}
				} else {
					printk("Invalid tracker ID: %s\n", arg);
				}
			} else {
				if (data_collect_is_active()) {
					printk("Data collection ACTIVE for tracker %u\n", data_collect_get_target_id());
				} else if (data_collect_batch_is_active()) {
					uint32_t mask = 0;
					for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
						if (data_collect_batch_is_target(i)) {
							mask |= BIT(i);
						}
					}
					printk("Batch data collection ACTIVE, tracker mask 0x%08x (use 'collectstop')\n",
					       (unsigned int)mask);
				} else {
					printk("Data collection inactive\n");
					printk("Usage: collect <tracker_id> | collect off | collectall <rate_hz>\n");
				}
			}
#else
			printk("Data collection not available (build with CONFIG_DATA_COLLECT=y)\n");
#endif
		} else if (strcmp(argv[0], command_ota) == 0) {
			if (!arg) {
				/* "ota" with no args → show status */
				esb_ota_relay_console_cmd(0, "status");
			} else if (strcmp(arg, "abort") == 0 || strcmp(arg, "cancel") == 0) {
				if (arg2) {
					uint8_t id;
					if (!parse_u8_arg(arg2, &id)) {
						printk("Invalid tracker ID: %s\n", arg2);
						continue;
					}
					esb_ota_relay_console_cmd(id, "abort");
				} else {
					/* Abort all OTA targets */
					esb_ota_relay_console_cmd(0xFF, "abort");
				}
			} else if (strcmp(arg, "status") == 0) {
				esb_ota_relay_console_cmd(0, "status");
			} else if (strcmp(arg, "info") == 0) {
				if (arg2) {
					uint8_t id;
					if (!parse_u8_arg(arg2, &id)) {
						printk("Invalid tracker ID: %s\n", arg2);
						continue;
					}
					esb_ota_relay_console_cmd(id, "info");
				} else {
					printk("Usage: ota info <tracker_id>\n");
				}
			} else {
				printk("OTA commands: ota, ota info <id>, ota abort, ota status\n");
			}
		} else {
			printk("Unknown command\n");
		}
	}
}

#endif
