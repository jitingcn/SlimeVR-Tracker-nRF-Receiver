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

#define USB DT_NODELABEL(usbd)
#if DT_NODE_HAS_STATUS(USB, okay)

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/console/console.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log_ctrl.h>
#include "connection/esb.h"
#include "connection/rssi_scan.h"
#include "data_collect.h"
#include "esb_ota.h"

#include <ctype.h>
#include <stdlib.h>
#include <errno.h>

#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

uint32_t *dbl_reset_mem = ((uint32_t *)DFU_DBL_RESET_MEM);

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

static void console_thread(void);
K_THREAD_DEFINE(console_thread_id, 1024, console_thread, NULL, NULL, NULL, 6, 0, 0);

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2
#define NRF5_BOOTLOADER CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define ADAFRUIT_DFU_MAGIC_UF2_RESET 0x57
#define ADAFRUIT_DFU_MAGIC_OTA_RESET 0xA8

#if NRF5_BOOTLOADER
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
#endif

static const char *meows[] = {
	"Mew", "Meww", "Meow", "Meow meow", "Mrrrp", "Mrrf", "Mreow", "Mrrrow", "Mrrr", "Purr",
	"mew", "meww", "meow", "meow meow", "mrrrp", "mrrf", "mreow", "mrrrow", "mrrr", "purr",
};

static const char *meow_punctuations[] = {".", "?", "!", "-", "~", ""};

static const char *meow_suffixes[]
	= {" :3", " :3c", " ;3", " ;3c", " x3", " x3c", " X3", " X3c", " >:3", " >:3c", " >;3", " >;3c", ""};

static void skip_dfu(void)
{
#if DFU_EXISTS                            // Using Adafruit bootloader
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
	ram_range_retain(dbl_reset_mem, sizeof(dbl_reset_mem), true);
#endif
}

static void request_local_dfu(bool ota)
{
#if ADAFRUIT_BOOTLOADER
	NRF_POWER->GPREGRET = ota ? ADAFRUIT_DFU_MAGIC_OTA_RESET : ADAFRUIT_DFU_MAGIC_UF2_RESET;
	k_msleep(100);
	sys_reboot(SYS_REBOOT_COLD);
#elif NRF5_BOOTLOADER
	ARG_UNUSED(ota);
	gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
	k_msleep(100);
	sys_reboot(SYS_REBOOT_COLD);
#else
	ARG_UNUSED(ota);
#endif
}

static void print_info(void)
{
	printk(CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT "\n");
	printk(FW_STRING);
	printk("Repo: %s | Branch: %s | Author: %s\n", FW_GIT_REPO_URL, FW_GIT_BRANCH, FW_GIT_AUTHOR);

	printk("\nBoard: " CONFIG_BOARD "\n");
	printk("SOC: " CONFIG_SOC "\n");
	printk("Target: " CONFIG_BOARD_TARGET "\n");

	printk("\nDevice address: %012llX\n", *(uint64_t *)NRF_FICR->DEVICEADDR & 0xFFFFFFFFFFFF);

	// Display RF channel info
	uint8_t current_channel = esb_get_receiver_channel();
	if (current_channel != 0xFF && current_channel <= 100) {
		printk("RF Channel: %u (custom)\n", current_channel);
	} else {
		printk("RF Channel: %u (default)\n", CONFIG_RADIO_RF_CHANNEL);
	}
}

static void print_uptime(void)
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

	printk("Uptime: %u.%02u:%02u:%02u.%03u,%03u\n", days, hours, minutes, seconds, milliseconds, microseconds);
}

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
		"  stats                      Toggle detailed packet statistics\n"
		"  stats <seconds>            Show detailed stats for N seconds\n"
		"  resetstats                 Reset packet statistics\n"
		"\n"
	);

	printk(
		"RF Channel (Local Receiver):\n"
		"  channel <1-100>            Set receiver RF channel only\n"
		"    Example: channel 25       Set receiver to channel 25\n"
		"  clearchannel               Clear receiver RF channel (use default)\n"
		"\n"
	);

	printk(
		"RSSI / Channel Scan:\n"
		"  rssi_scan                  Scan RSSI across channels 1-100 and print a recommended channel\n"
		"\n"
	);

	printk(
		"Remote Commands:\n"
		"  send <id|all> <command>    Send remote command to tracker(s)\n"
		"    Commands: shutdown, calibrate, 6-side, meow, scan,\n"
		"              mag <on|off|clear|cal>, reboot, clear, dfu [ota],\n"
		"              channel <1-100>, clearchannel,\n"
		"              sens <x,y,z|reset>, reset <zro|acc|bat|mag|tcal|fusion>, ping\n"
	);

	printk(
		"    Examples:\n"
		"      send 0 shutdown          Shutdown tracker 0\n"
		"      send all calibrate       Calibrate all active trackers\n"
		"      send 1 meow              Make tracker 1 meow\n"
		"      send 2 reboot            Reboot tracker 2\n"
		"      send 0 sens 1.0,1.0,1.0  Set sensitivity for tracker 0\n"
		"      send all sens reset      Reset sensitivity for all\n"
		"      send 1 reset zro         Reset ZRO calibration on tracker 1\n"
		"      send all ping            Ping all active trackers\n"
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
		"  dfu [ota]                  Enter DFU bootloader (default UF2, optional OTA)\n"
		"\n"
	);
#endif

	printk(
		"Other:\n"
		"  collect <id>               Start raw sensor data collection from tracker\n"
		"  collect off                Stop data collection\n"
		"  collect                    Show data collection status\n"
		"  ota                        Show ESB OTA update status\n"
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

static void print_list(void)
{
	printk("Stored devices:\n");
	for (uint8_t i = 0; i < stored_trackers; i++) {
		printk("%012llX\n", stored_tracker_addr[i]);
	}
}

static void console_thread(void)
{
#if DFU_EXISTS
	if (button_read()) // button held on usb connect, enter DFU
	{
#if ADAFRUIT_BOOTLOADER
		request_local_dfu(false);
#endif
#if NRF5_BOOTLOADER
		gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
		k_msleep(100); // Wait for GPIO to be configured
		sys_reboot(SYS_REBOOT_COLD);
#endif
	}
#endif

	/* Data collection: HID mode uses SYS_INIT, CDC mode needs manual init */
#if defined(CONFIG_DATA_COLLECT) && !defined(CONFIG_DATA_COLLECT_HID)
	data_collect_init();
#endif

	console_getline_init();

	// Wait for any pending log data to be processed
	while (log_data_pending()) {
		k_usleep(1);
	}

	// Wait for USB CDC to be ready by checking DTR (Data Terminal Ready) signal
	// This ensures the terminal is actually connected and ready to receive data
	const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	if (device_is_ready(uart_dev)) {
		uint32_t dtr = 0;
		// Wait up to 5 seconds for DTR to be asserted (terminal connected)
		for (int i = 0; i < 50; i++) {
			if (uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr) == 0 && dtr) {
				break;
			}
			k_msleep(100);
		}
		// Give a bit more time for the terminal to be fully ready
		k_msleep(100);
	}

	printk("*** " CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT " ***\n");
	printk(FW_STRING);
	printk("Repo: %s | Branch: %s | Author: %s\n", FW_GIT_REPO_URL, FW_GIT_BRANCH, FW_GIT_AUTHOR);

	// Print help on startup
	print_help();

	uint8_t command_info[] = "info";
	uint8_t command_uptime[] = "uptime";
	uint8_t command_list[] = "list";
	uint8_t command_reboot[] = "reboot";
	uint8_t command_add[] = "add";
	uint8_t command_remove[] = "remove";
	uint8_t command_pair[] = "pair";
	uint8_t command_exit[] = "exit";
	uint8_t command_clear[] = "clear";
	uint8_t command_stats[] = "stats";
	uint8_t command_resetstats[] = "resetstats";
	uint8_t command_channel[] = "channel";
	uint8_t command_clearchannel[] = "clearchannel";
	uint8_t command_rssi_scan[] = "rssi_scan";
	uint8_t command_send[] = "send";
	uint8_t command_help[] = "help";

#if DFU_EXISTS
	uint8_t command_dfu[] = "dfu";
#endif

	uint8_t command_meow[] = "meow";
	uint8_t command_collect[] = "collect";
	uint8_t command_ota[] = "ota";

	while (1) {
		uint8_t *line = console_getline();
		uint8_t *arg = NULL;
		uint8_t *arg2 = NULL;
		uint8_t *arg3 = NULL;
		uint8_t *arg4 = NULL;

		// Parse command and arguments
		uint8_t *p = line;
		while (*p) {
			*p = tolower(*p);
			p++;
		}

		// Split by spaces
		p = line;
		while (*p && *p != ' ') {
			p++;
		}
		if (*p == ' ') {
			*p = 0;
			p++;
			while (*p == ' ') {
				p++; // Skip multiple spaces
			}
			if (*p) {
				arg = p;
				// Find second argument
				while (*p && *p != ' ') {
					p++;
				}
				if (*p == ' ') {
					*p = 0;
					p++;
					while (*p == ' ') {
						p++;
					}
					if (*p) {
						arg2 = p;
						// Find third argument
						while (*p && *p != ' ') {
							p++;
						}
						if (*p == ' ') {
							*p = 0;
							p++;
							while (*p == ' ') {
								p++;
							}
							if (*p) {
								arg3 = p;
								// Find fourth argument
								while (*p && *p != ' ') {
									p++;
								}
								if (*p == ' ') {
									*p = 0;
									p++;
									while (*p == ' ') {
										p++;
									}
									if (*p) {
										arg4 = p;
									}
								}
							}
						}
					}
				}
			}
		}

		if (memcmp(line, command_help, sizeof(command_help)) == 0) {
			print_help();
		} else if (memcmp(line, command_info, sizeof(command_info)) == 0) {
			print_info();
		} else if (memcmp(line, command_uptime, sizeof(command_uptime)) == 0) {
			print_uptime();
		} else if (memcmp(line, command_add, sizeof(command_add)) == 0) {
			uint64_t addr = strtoull(arg, NULL, 16);
			uint8_t buf[13];
			snprintk(buf, 13, "%012llx", addr);
			if (addr != 0 && memcmp(buf, arg, 13) == 0) {
				int slot = esb_add_pair(addr, true);
				if (slot >= 0) {
					printk("Tracker stored in slot %d\n", slot);
				} else if (slot == -ENOSPC) {
					printk("Tracker list is full\n");
				} else if (slot == -EINVAL) {
					printk("Invalid tracker address\n");
				} else {
					printk("Failed to add tracker: %d\n", slot);
				}
			} else {
				printk("Invalid address\n");
			}
		} else if (memcmp(line, command_remove, sizeof(command_remove)) == 0) {
			esb_pop_pair();
		} else if (memcmp(line, command_list, sizeof(command_list)) == 0) {
			print_list();
		} else if (memcmp(line, command_reboot, sizeof(command_reboot)) == 0) {
			skip_dfu();
			sys_reboot(SYS_REBOOT_COLD);
		} else if (memcmp(line, command_pair, sizeof(command_pair)) == 0) {
			if (!arg) {
				// No argument: traditional pairing mode with timeout
				esb_start_pairing();
				printk("Pairing mode enabled (auto-exit after %d seconds)\n", CONFIG_PAIRING_TIMEOUT);
			} else {
				// Parse target count
				char *endptr;
				long count = strtol(arg, &endptr, 10);
				if (*endptr != '\0' || count < 0 || count > 255) {
					printk("Invalid count. Usage: pair [count]\n");
					printk("  pair       - Pair indefinitely (timeout after %d seconds)\n", CONFIG_PAIRING_TIMEOUT);
					printk("  pair 4     - Exit after pairing 4 new devices\n");
				} else if (count == 0) {
					// pair 0 = same as no argument
					esb_start_pairing();
					printk("Pairing mode enabled (auto-exit after %d seconds)\n", CONFIG_PAIRING_TIMEOUT);
				} else {
					esb_start_pairing_with_count((uint8_t)count);
					printk("Pairing mode enabled (auto-exit after %u new devices or %d seconds)\n", (uint8_t)count, CONFIG_PAIRING_TIMEOUT);
				}
			}
		} else if (memcmp(line, command_exit, sizeof(command_exit)) == 0) {
			esb_finish_pair();
		} else if (memcmp(line, command_clear, sizeof(command_clear)) == 0) {
			esb_clear();
		} else if (memcmp(line, command_stats, sizeof(command_stats)) == 0) {
			if (!arg) {
				// No argument: toggle detailed stats
				bool enabled = esb_toggle_stats_detailed();
				if (enabled) {
					printk("Detailed stats enabled (toggle again to disable)\n");
				} else {
					printk("Detailed stats disabled\n");
				}
			} else {
				// Parse duration
				char *endptr;
				long duration = strtol(arg, &endptr, 10);
				if (*endptr != '\0' || duration < 0 || duration > 86400) {
					printk("Invalid duration. Usage: stats [seconds]\n");
					printk("  stats       - Toggle detailed stats on/off\n");
					printk("  stats 30    - Show detailed stats for 30 seconds\n");
				} else if (duration == 0) {
					// stats 0 = toggle off
					esb_set_stats_detailed(0);
					if (esb_get_stats_detailed_enabled()) {
						printk("Detailed stats disabled\n");
					} else {
						printk("Detailed stats enabled (toggle again to disable)\n");
					}
				} else {
					esb_set_stats_detailed((uint32_t)duration);
					printk("Detailed stats enabled for %ld seconds\n", duration);
				}
			}
		} else if (memcmp(line, command_resetstats, sizeof(command_resetstats)) == 0) {
			esb_reset_all_stats();
		} else if (memcmp(line, command_rssi_scan, sizeof(command_rssi_scan)) == 0) {
			rssi_scan_run_and_print();
		} else if (memcmp(line, command_channel, sizeof(command_channel)) == 0) {
			if (!arg) {
				printk("Usage: channel <1-100>\n");
				printk("Example: channel 25 - Set receiver RF channel to 25 (local only)\n");
			} else {
				char *endptr;
				long channel = strtol(arg, &endptr, 10);

				if (*endptr != '\0' || channel < 1 || channel > 100) {
					printk("Invalid channel. Must be a number between 1 and 100.\n");
				} else {
					esb_set_receiver_channel((uint8_t)channel);
					printk("Receiver RF channel set to %d (local only)\n", (int)channel);
				}
			}
		} else if (memcmp(line, command_clearchannel, sizeof(command_clearchannel)) == 0) {
			esb_clear_receiver_channel();
			printk("Receiver RF channel cleared (local only)\n");
		} else if (memcmp(line, command_send, sizeof(command_send)) == 0) {
			if (!arg || !arg2) {
				printk("Usage: send <id|all> <command>\n");
				printk("Examples:\n");
				printk("  send 0 shutdown      - Shutdown tracker 0\n");
				printk("  send all shutdown    - Shutdown all active trackers\n");
				printk("  send 1 calibrate     - Calibrate tracker 1\n");
				printk("  send all meow        - Make all active trackers meow\n");
				printk("  send 2 reboot        - Reboot tracker 2\n");
				printk("  send 3 clear         - Clear pairing on tracker 3\n");
				printk("  send all dfu         - Enter DFU mode on all active trackers\n");
				printk(
					"Available commands: shutdown, calibrate, 6-side, meow, scan, mag, reboot, clear, dfu, sens, "
					"reset, ping, tcal, tdma, test\n"
				);
			} else {
				// Parse target (id or "all")
				bool target_all = (strcmp(arg, "all") == 0);
				uint8_t tracker_id = 0;

				if (!target_all) {
					// Parse tracker ID
					char *endptr;
					long id = strtol(arg, &endptr, 10);

					if (*endptr != '\0' || id < 0 || id > 255) {
						printk("Invalid tracker ID. Use a number (0-255) or 'all'\n");
						continue;
					}
					tracker_id = (uint8_t)id;
				}

				// Process command
				uint8_t cmd_flag = 0xFF;
				const char *cmd_name = NULL;

				if (strcmp(arg2, "shutdown") == 0) {
					cmd_flag = ESB_PONG_FLAG_SHUTDOWN;
					cmd_name = "Shutdown";
				} else if (strcmp(arg2, "calibrate") == 0) {
					cmd_flag = ESB_PONG_FLAG_CALIBRATE;
					cmd_name = "Calibrate";
				} else if (strcmp(arg2, "6-side") == 0) {
					cmd_flag = ESB_PONG_FLAG_SIX_SIDE_CAL;
					cmd_name = "6-side calibration";
				} else if (strcmp(arg2, "meow") == 0) {
					cmd_flag = ESB_PONG_FLAG_MEOW;
					cmd_name = "Meow";
				} else if (strcmp(arg2, "scan") == 0) {
					cmd_flag = ESB_PONG_FLAG_SCAN;
					cmd_name = "Sensor scan";
				} else if (strcmp(arg2, "mag") == 0) {
					// mag command - supports "on", "off", "clear", "cal"
					if (!arg3) {
						printk("Usage: send <id|all> mag <on|off|clear|cal>\n");
						printk("  on    - Enable magnetometer\n");
						printk("  off   - Disable magnetometer\n");
						printk("  clear - Clear magnetometer calibration\n");
						printk("  cal   - Start magnetometer calibration\n");
						continue;
					}

					uint8_t mag_cmd = 0xFF;
					const char *mag_name = NULL;

					if (strcmp(arg3, "on") == 0) {
						mag_cmd = ESB_PONG_FLAG_MAG_ON;
						mag_name = "Magnetometer enable";
					} else if (strcmp(arg3, "off") == 0) {
						mag_cmd = ESB_PONG_FLAG_MAG_OFF;
						mag_name = "Magnetometer disable";
					} else if (strcmp(arg3, "clear") == 0) {
						mag_cmd = ESB_PONG_FLAG_MAG_CLEAR;
						mag_name = "Magnetometer calibration clear";
					} else if (strcmp(arg3, "cal") == 0 || strcmp(arg3, "calibrate") == 0) {
						mag_cmd = ESB_PONG_FLAG_MAG_CAL;
						mag_name = "Magnetometer calibration";
					} else {
						printk("Unknown mag subcommand: %s (use 'on', 'off', 'clear' or 'cal')\n", arg3);
						continue;
					}

					if (target_all) {
						esb_send_remote_command_all(mag_cmd);
						printk("%s request sent to all active trackers\n", mag_name);
					} else {
						esb_send_remote_command(tracker_id, mag_cmd);
						printk("%s request sent to tracker %d\n", mag_name, tracker_id);
					}
					continue;
				} else if (strcmp(arg2, "reboot") == 0) {
					cmd_flag = ESB_PONG_FLAG_REBOOT;
					cmd_name = "Reboot";
				} else if (strcmp(arg2, "clear") == 0) {
					cmd_flag = ESB_PONG_FLAG_CLEAR;
					cmd_name = "Clear pairing";
				} else if (strcmp(arg2, "dfu") == 0) {
					if (arg3 && strcmp(arg3, "ota") == 0) {
						cmd_flag = ESB_PONG_FLAG_DFU_OTA;
						cmd_name = "OTA DFU mode";
					} else if (!arg3) {
						cmd_flag = ESB_PONG_FLAG_DFU;
						cmd_name = "UF2 DFU mode";
					} else {
						printk("Unknown dfu subcommand: %s (use 'ota' or omit it)\n", arg3);
						continue;
					}
				} else if (strcmp(arg2, "fusion") == 0) {
					cmd_flag = ESB_PONG_FLAG_FUSION_RESET;
					cmd_name = "Fusion reset";
				} else if (strcmp(arg2, "channel") == 0) {
					// Special handling for channel command - needs arg3
					if (!arg3) {
						printk("Usage: send all channel <1-100>\n");
						printk("Example: send all channel 25 - Set all active trackers to channel 25\n");
						continue;
					}

					char *endptr;
					long channel = strtol(arg3, &endptr, 10);

					if (*endptr != '\0' || channel < 1 || channel > 100) {
						printk("Invalid channel. Must be a number between 1 and 100.\n");
						continue;
					}

					if (!target_all) {
						printk("Channel command only supports 'all' target\n");
						continue;
					}

					esb_set_all_trackers_channel((uint8_t)channel);
					printk("Setting RF channel to %d for all active trackers and receiver\n", (int)channel);
					continue;
				} else if (strcmp(arg2, "clearchannel") == 0) {
					if (!target_all) {
						printk("Clearchannel command only supports 'all' target\n");
						continue;
					}

					esb_clear_all_trackers_channel();
					printk("Clearing RF channel for all active trackers and receiver\n");
					continue;
				} else if (strcmp(arg2, "sens") == 0) {
					// sens command - needs arg3 for values or "reset"
					if (!arg3) {
						printk("Usage: send <id|all> sens <x>,<y>,<z> or send <id|all> sens reset\n");
						printk("Example: send 0 sens 1.0,1.0,1.0\n");
						printk("Example: send all sens reset\n");
						continue;
					}

					if (strcmp(arg3, "reset") == 0) {
						// sens reset command
						if (target_all) {
							esb_send_remote_command_all(ESB_PONG_FLAG_SENS_RESET);
							printk("Sens reset request sent to all active trackers\n");
						} else {
							esb_send_remote_command(tracker_id, ESB_PONG_FLAG_SENS_RESET);
							printk("Sens reset request sent to tracker %d\n", tracker_id);
						}
					} else {
						// Parse comma-separated floats
						char *token;
						char *endptr;
						int token_count = 0;
						float values[3];

						token = strtok(arg3, ",");
						while (token != NULL && token_count < 3) {
							values[token_count] = strtof(token, &endptr);
							if (token == endptr || *endptr != '\0') {
								printk("Invalid float value: %s\n", token);
								break;
							}
							token_count++;
							token = strtok(NULL, ",");
						}

						if (token_count == 3) {
							if (target_all) {
								for (uint8_t i = 0; i < MAX_TRACKERS; i++) {
									esb_send_remote_command_sens(i, values[0], values[1], values[2]);
								}
								printk(
									"Sens set (%.2f,%.2f,%.2f) request sent to all active trackers\n",
									(double)values[0],
									(double)values[1],
									(double)values[2]
								);
							} else {
								esb_send_remote_command_sens(tracker_id, values[0], values[1], values[2]);
								printk(
									"Sens set (%.2f,%.2f,%.2f) request sent to tracker %d\n",
									(double)values[0],
									(double)values[1],
									(double)values[2],
									tracker_id
								);
							}
						} else {
							printk("Error: Invalid format. Use: sens <x>,<y>,<z> or sens reset\n");
							printk("Example: sens 10.5,-2.1,15.0\n");
						}
					}
					continue;
				} else if (strcmp(arg2, "reset") == 0) {
					// reset command - needs arg3 for subcommand
					if (!arg3) {
						printk("Usage: send <id|all> reset <zro|acc|bat|mag|tcal|fusion>\n");
						printk("Example: send 0 reset zro\n");
						printk("Example: send all reset acc\n");
						continue;
					}

					uint8_t reset_cmd = 0xFF;
					const char *reset_name = NULL;

					if (strcmp(arg3, "zro") == 0) {
						reset_cmd = ESB_PONG_FLAG_RESET_ZRO;
						reset_name = "ZRO reset";
					} else if (strcmp(arg3, "acc") == 0) {
						reset_cmd = ESB_PONG_FLAG_RESET_ACC;
						reset_name = "Accelerometer reset";
					} else if (strcmp(arg3, "bat") == 0) {
						reset_cmd = ESB_PONG_FLAG_RESET_BAT;
						reset_name = "Battery reset";
					} else if (strcmp(arg3, "mag") == 0) {
						reset_cmd = ESB_PONG_FLAG_MAG_CLEAR;
						reset_name = "Magnetometer calibration reset";
					} else if (strcmp(arg3, "tcal") == 0) {
						reset_cmd = ESB_PONG_FLAG_RESET_TCAL;
						reset_name = "Temperature calibration reset";
					} else if (strcmp(arg3, "fusion") == 0) {
						reset_cmd = ESB_PONG_FLAG_FUSION_RESET;
						reset_name = "Fusion reset";
					} else {
						printk("Unknown reset command: %s\n", arg3);
						printk("Available: zro, acc, bat, mag, tcal, fusion\n");
						continue;
					}

					if (target_all) {
						esb_send_remote_command_all(reset_cmd);
						printk("%s request sent to all active trackers\n", reset_name);
					} else {
						esb_send_remote_command(tracker_id, reset_cmd);
						printk("%s request sent to tracker %d\n", reset_name, tracker_id);
					}
					continue;
				} else if (strcmp(arg2, "ping") == 0) {
					// ping command
					if (target_all) {
						esb_send_remote_command_all(ESB_PONG_FLAG_PING);
						printk("Ping request sent to all active trackers\n");
					} else {
						esb_send_remote_command(tracker_id, ESB_PONG_FLAG_PING);
						printk("Ping request sent to tracker %d\n", tracker_id);
					}
					continue;
				} else if (strcmp(arg2, "tcal") == 0) {
					// tcal command - supports "on/off", "auto on/off", "boot on/off" and "clear"
					if (!arg3) {
						printk("Usage: send <id|all> tcal <on|off|auto on|auto off|boot on|boot off|clear>\n");
						printk("Example: send 0 tcal on       - Enable temperature calibration on tracker 0\n");
						printk("Example: send all tcal off    - Disable temperature calibration on all active trackers\n");
						printk("Example: send 0 tcal auto on  - Enable auto-calibration on tracker 0\n");
						printk("Example: send all tcal auto off - Disable auto-calibration on all active trackers\n");
						printk("Example: send 0 tcal boot on - Enable boot calibration on tracker 0\n");
						printk("Example: send 0 tcal clear - Clear temperature calibration on tracker 0\n");
						continue;
					}

					if (strcmp(arg3, "on") == 0) {
						// Enable T-Cal
						if (target_all) {
							esb_send_remote_command_all(ESB_PONG_FLAG_TCAL_ON);
							printk("T-Cal enable request sent to all active trackers\n");
						} else {
							esb_send_remote_command(tracker_id, ESB_PONG_FLAG_TCAL_ON);
							printk("T-Cal enable request sent to tracker %d\n", tracker_id);
						}
					} else if (strcmp(arg3, "off") == 0) {
						// Disable T-Cal
						if (target_all) {
							esb_send_remote_command_all(ESB_PONG_FLAG_TCAL_OFF);
							printk("T-Cal disable request sent to all active trackers\n");
						} else {
							esb_send_remote_command(tracker_id, ESB_PONG_FLAG_TCAL_OFF);
							printk("T-Cal disable request sent to tracker %d\n", tracker_id);
						}
					} else if (strcmp(arg3, "auto") == 0) {
						if (!arg4) {
							printk("Usage: send <id|all> tcal auto <on|off>\n");
							continue;
						}

						uint8_t tcal_cmd = 0xFF;
						const char *tcal_name = NULL;

						if (strcmp(arg4, "on") == 0) {
							tcal_cmd = ESB_PONG_FLAG_TCAL_AUTO_ON;
							tcal_name = "T-Cal auto-calibration enable";
						} else if (strcmp(arg4, "off") == 0) {
							tcal_cmd = ESB_PONG_FLAG_TCAL_AUTO_OFF;
							tcal_name = "T-Cal auto-calibration disable";
						} else {
							printk("Invalid tcal auto argument: %s (use 'on' or 'off')\n", arg4);
							continue;
						}

						if (target_all) {
							esb_send_remote_command_all(tcal_cmd);
							printk("%s request sent to all active trackers\n", tcal_name);
						} else {
							esb_send_remote_command(tracker_id, tcal_cmd);
							printk("%s request sent to tracker %d\n", tcal_name, tracker_id);
						}
					} else if (strcmp(arg3, "clear") == 0) {
						// clear command - clear tcal data
						if (target_all) {
							esb_send_remote_command_all(ESB_PONG_FLAG_RESET_TCAL);
							printk("T-Cal clear request sent to all active trackers\n");
						} else {
							esb_send_remote_command(tracker_id, ESB_PONG_FLAG_RESET_TCAL);
							printk("T-Cal clear request sent to tracker %d\n", tracker_id);
						}
					} else if (strcmp(arg3, "boot") == 0) {
						if (!arg4) {
							printk("Usage: send <id|all> tcal boot <on|off>\n");
							continue;
						}

						uint8_t tcal_cmd = 0xFF;
						const char *tcal_name = NULL;

						if (strcmp(arg4, "on") == 0) {
							tcal_cmd = ESB_PONG_FLAG_TCAL_BOOT_ON;
							tcal_name = "T-Cal boot-calibration enable";
						} else if (strcmp(arg4, "off") == 0) {
							tcal_cmd = ESB_PONG_FLAG_TCAL_BOOT_OFF;
							tcal_name = "T-Cal boot-calibration disable";
						} else {
							printk("Invalid tcal boot argument: %s (use 'on' or 'off')\n", arg4);
							continue;
						}

						if (target_all) {
							esb_send_remote_command_all(tcal_cmd);
							printk("%s request sent to all active trackers\n", tcal_name);
						} else {
							esb_send_remote_command(tracker_id, tcal_cmd);
							printk("%s request sent to tracker %d\n", tcal_name, tracker_id);
						}
					} else {
							printk("Unknown tcal subcommand: %s (use 'on', 'off', 'auto', 'boot' or 'clear')\n", arg3);
						}
						continue;
					} else if (strcmp(arg2, "tdma") == 0) {
						// tdma command - supports "on/off"
						if (!arg3) {
							printk("Usage: send <id|all> tdma <on|off>\n");
							printk("Example: send 0 tdma on       - Enable TDMA scheduling on tracker 0\n");
							printk("Example: send all tdma off    - Disable TDMA scheduling on all active trackers\n");
							continue;
						}

						if (strcmp(arg3, "on") == 0) {
							// Enable TDMA
							if (target_all) {
								esb_send_remote_command_all(ESB_PONG_FLAG_TDMA_ON);
								printk("TDMA enable request sent to all active trackers\n");
							} else {
								esb_send_remote_command(tracker_id, ESB_PONG_FLAG_TDMA_ON);
								printk("TDMA enable request sent to tracker %d\n", tracker_id);
							}
						} else if (strcmp(arg3, "off") == 0) {
							// Disable TDMA
							if (target_all) {
								esb_send_remote_command_all(ESB_PONG_FLAG_TDMA_OFF);
								printk("TDMA disable request sent to all active trackers\n");
							} else {
								esb_send_remote_command(tracker_id, ESB_PONG_FLAG_TDMA_OFF);
								printk("TDMA disable request sent to tracker %d\n", tracker_id);
							}
						} else {
							printk("Unknown tdma subcommand: %s (use 'on' or 'off')\n", arg3);
						}
						continue;
					} else if (strcmp(arg2, "test") == 0) {
						// test mode command - supports "on/off"
						if (!arg3) {
							printk("Usage: send <id|all> test <on|off>\n");
							printk("Example: send 0 test on       - Enable test mode on tracker 0\n");
							printk("Example: send all test on     - Enable test mode on all active trackers\n");
							continue;
						}

						if (strcmp(arg3, "on") == 0) {
							if (target_all) {
								esb_send_remote_command_all(ESB_PONG_FLAG_TEST_MODE_ON);
								printk("Test mode enable request sent to all active trackers\n");
							} else {
								esb_send_remote_command(tracker_id, ESB_PONG_FLAG_TEST_MODE_ON);
								printk("Test mode enable request sent to tracker %d\n", tracker_id);
							}
						} else if (strcmp(arg3, "off") == 0) {
							if (target_all) {
								esb_send_remote_command_all(ESB_PONG_FLAG_TEST_MODE_OFF);
								printk("Test mode disable request sent to all active trackers\n");
							} else {
								esb_send_remote_command(tracker_id, ESB_PONG_FLAG_TEST_MODE_OFF);
								printk("Test mode disable request sent to tracker %d\n", tracker_id);
							}
						} else {
							printk("Unknown test subcommand: %s (use 'on' or 'off')\n", arg3);
						}
						continue;
					}

					if (cmd_flag != 0xFF) {
					if (target_all) {
						esb_send_remote_command_all(cmd_flag);
						printk("%s request sent to all active trackers\n", cmd_name);
					} else {
						esb_send_remote_command(tracker_id, cmd_flag);
						printk("%s request sent to tracker %d\n", cmd_name, tracker_id);
					}
				} else {
					printk("Unknown command: %s\n", arg2);
					printk(
						"Available commands: shutdown, calibrate, 6-side, meow, scan, mag, reboot, clear, dfu [ota], fusion, sens, "
						"reset, ping, tcal, tdma, test\n"
					);
				}
			}
		}
#if DFU_EXISTS
		else if (memcmp(line, command_dfu, sizeof(command_dfu)) == 0) {
			bool ota = false;
			if (arg) {
				if (strcmp((char *)arg, "ota") == 0) {
					ota = true;
				} else {
					printk("Unknown dfu argument: %s (use 'ota' or omit it)\n", arg);
					continue;
				}
			}
			request_local_dfu(ota);
		}
#endif
		else if (memcmp(line, command_meow, sizeof(command_meow)) == 0) {
			print_meow();
		}
		else if (memcmp(line, command_collect, sizeof(command_collect) - 1) == 0) {
#ifdef CONFIG_DATA_COLLECT
			if (arg && strcmp((char *)arg, "off") == 0) {
				if (data_collect_is_active()) {
					uint8_t tid = data_collect_get_target_id();
					data_collect_stop();
					esb_send_remote_command(tid, ESB_PONG_FLAG_DATA_COLLECT_OFF);
					printk("Data collection stopped, sent OFF to tracker %u\n", tid);
				} else {
					printk("Data collection is not active\n");
				}
			} else if (arg) {
				char *endptr = NULL;
				unsigned long id = strtoul((char *)arg, &endptr, 10);
				if (endptr != (char *)arg && id < 255) {
					data_collect_start((uint8_t)id);
					esb_send_remote_command((uint8_t)id, ESB_PONG_FLAG_DATA_COLLECT_ON);
					printk("Data collection started for tracker %u\n", (unsigned)id);
					printk("Test mode enabled on tracker (prevents sleep)\n");
					printk("Non-target trackers will receive SHUTDOWN\n");
					printk("Use 'collect off' to stop\n");
				} else {
					printk("Invalid tracker ID: %s\n", arg);
				}
			} else {
				if (data_collect_is_active()) {
					printk("Data collection ACTIVE for tracker %u\n",
						data_collect_get_target_id());
				} else {
					printk("Data collection inactive\n");
					printk("Usage: collect <tracker_id> | collect off\n");
				}
			}
#else
			printk("Data collection not available (build with CONFIG_DATA_COLLECT=y)\n");
#endif
		}
		else if (memcmp(line, command_ota, sizeof(command_ota) - 1) == 0) {
			if (!arg) {
				/* "ota" with no args → show status */
				esb_ota_relay_console_cmd(0, "status");
			} else if (strcmp((char *)arg, "abort") == 0 ||
				   strcmp((char *)arg, "cancel") == 0) {
				if (arg2) {
					int id = atoi((char *)arg2);
					esb_ota_relay_console_cmd((uint8_t)id, "abort");
				} else {
					/* Abort all OTA targets */
					esb_ota_relay_console_cmd(0xFF, "abort");
				}
			} else if (strcmp((char *)arg, "status") == 0) {
				esb_ota_relay_console_cmd(0, "status");
			} else if (strcmp((char *)arg, "info") == 0) {
				if (arg2) {
					int id = atoi((char *)arg2);
					esb_ota_relay_console_cmd((uint8_t)id, "info");
				} else {
					printk("Usage: ota info <tracker_id>\n");
				}
			} else {
				printk("OTA commands: ota, ota info <id>, ota abort, ota status\n");
			}
		}
		else {
			printk("Unknown command\n");
		}
	}
}

#endif
