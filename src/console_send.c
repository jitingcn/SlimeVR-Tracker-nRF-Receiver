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
#include "console_send.h"
#include "parse_args.h"

#include "rcv_cmd.h"

#include <stdlib.h>
#include <string.h>
#include <zephyr/sys/printk.h>

#define SENS_AUTO_MAX_REVOLUTIONS 100

static void console_send_flag(bool target_all, uint8_t tracker_id, uint8_t flag, const char *name)
{
	uint8_t tid = target_all ? RCV_HID_TARGET_ALL : tracker_id;
	uint8_t st = rcv_cmd_remote_flag(tid, flag);
	if (st == RCV_HID_ST_QUEUED) {
		if (target_all) {
			printk("%s request sent to all active trackers\n", name);
		} else {
			printk("%s request sent to tracker %d\n", name, tracker_id);
		}
	} else {
		printk("%s failed: status %u\n", name, st);
	}
}

void console_handle_send(char *arg, char *arg2, char *arg3, char *arg4, char *arg5)
{
	if (!arg || !arg2) {
		printk("Usage: send <id|all> <command>\n");
		printk("Examples:\n");
		printk("  send 0 shutdown      - Shutdown tracker 0\n");
		printk("  send all shutdown    - Shutdown all active trackers\n");
		printk("  send 1 calibrate     - Zero-bias calibration on tracker 1\n");
		printk("  send 1 calibrate acc - Accelerometer calibration (18 orientations)\n");
		printk("  send all meow        - Make all active trackers meow\n");
		printk("  send 2 reboot        - Reboot tracker 2\n");
		printk("  send 3 clear         - Clear pairing on tracker 3\n");
		printk("  send all dfu         - Enter DFU mode on all active trackers\n");
		printk(
			"Available commands: shutdown, calibrate [acc], meow, scan, mag, reboot, clear, dfu, sens, "
			"reset, ping, tcal, tdma, test, wom enter, wom timeout [minutes]\n"
		);
		return;
	}

	bool target_all = (strcmp(arg, "all") == 0);
	uint8_t tracker_id = 0;

	if (!target_all) {
		long id;
		if (!parse_long_bounded(arg, 0, 255, &id)) {
			printk("Invalid tracker ID. Use a number (0-255) or 'all'\n");
			return;
		}
		tracker_id = (uint8_t)id;
	}

	if (strcmp(arg2, "wom") == 0) {
		bool enter = arg3 && strcmp(arg3, "enter") == 0;
		if (!arg3 || (!enter && strcmp(arg3, "timeout") != 0) ||
		    arg5 || (enter && arg4)) {
			printk("Usage: send <id|all> wom enter | wom timeout [minutes: 0-35791; 0 restores firmware default]\n");
			return;
		}
		uint32_t minutes = enter ? ESB_WOM_ENTER : 0;
		if (arg4) {
			long value;
			if (*arg4 < '0' || *arg4 > '9' ||
			    !parse_long_bounded(arg4, 0, 35791, &value)) {
				printk("Minutes must be an integer from 0 to 35791 (0 restores firmware default)\n");
				return;
			}
			minutes = (uint32_t)value;
			if (minutes == 0) {
				minutes = ESB_WOM_TIMEOUT_CLEAR;
			}
		}
		uint8_t st = rcv_cmd_remote_wom_command(
			target_all ? RCV_HID_TARGET_ALL : tracker_id, (uint16_t)minutes);
		if (st == RCV_HID_ST_STARTED) {
			printk("WOM command queued; waiting for tracker results (entry success means accepted, not asleep)\n");
		} else {
			printk("WOM command request failed: status %u\n", st);
		}
		return;
	}

	uint8_t cmd_flag = 0xFF;
	const char *cmd_name = NULL;

	if (strcmp(arg2, "shutdown") == 0) {
		cmd_flag = ESB_PONG_FLAG_SHUTDOWN;
		cmd_name = "Shutdown";
	} else if (strcmp(arg2, "calibrate") == 0) {
		if (arg4 || arg5 || (arg3 && strcmp(arg3, "acc") != 0)) {
			printk("Usage: send <id|all> calibrate [acc]\n");
			return;
		}
		cmd_flag = arg3 ? ESB_PONG_FLAG_CALIBRATE_ACC : ESB_PONG_FLAG_CALIBRATE;
		cmd_name = arg3 ? "Accelerometer calibration (18 orientations)" : "Zero-bias calibration";
	} else if (strcmp(arg2, "6-side") == 0) {
		if (arg3 || arg4 || arg5) {
			printk("Usage: send <id|all> calibrate acc\n");
			return;
		}
		printk("'6-side' is the historical name; use 'calibrate acc' (18 orientations).\n");
		cmd_flag = ESB_PONG_FLAG_CALIBRATE_ACC;
		cmd_name = "Accelerometer calibration (18 orientations)";
	} else if (strcmp(arg2, "meow") == 0) {
		cmd_flag = ESB_PONG_FLAG_MEOW;
		cmd_name = "Meow";
	} else if (strcmp(arg2, "scan") == 0) {
		cmd_flag = ESB_PONG_FLAG_SCAN;
		cmd_name = "Sensor scan";
	} else if (strcmp(arg2, "mag") == 0) {
		if (!arg3) {
			printk("Usage: send <id|all> mag <on|off|clear|cal|auto on|auto off>\n");
			printk("  on       - Enable magnetometer\n");
			printk("  off      - Disable magnetometer\n");
			printk("  clear    - Clear magnetometer calibration\n");
			printk("  cal      - Start magnetometer calibration\n");
			printk("  auto on  - Enable online magnetometer calibration\n");
			printk("  auto off - Disable online magnetometer calibration\n");
			return;
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
		} else if (strcmp(arg3, "auto") == 0 || strcmp(arg3, "online") == 0) {
			if (!arg4) {
				printk("Usage: send <id|all> mag %s <on|off>\n", arg3);
				return;
			}
			if (strcmp(arg4, "on") == 0) {
				mag_cmd = ESB_PONG_FLAG_MAG_AUTO_ON;
				mag_name = "Online magnetometer calibration enable";
			} else if (strcmp(arg4, "off") == 0) {
				mag_cmd = ESB_PONG_FLAG_MAG_AUTO_OFF;
				mag_name = "Online magnetometer calibration disable";
			} else {
				printk("Invalid mag %s argument: %s (use 'on' or 'off')\n", arg3, arg4);
				return;
			}
		} else {
			printk("Unknown mag subcommand: %s (use 'on', 'off', 'clear', 'cal' or 'auto')\n", arg3);
			return;
		}

		console_send_flag(target_all, tracker_id, mag_cmd, mag_name);
		return;
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
			return;
		}
	} else if (strcmp(arg2, "fusion") == 0) {
		cmd_flag = ESB_PONG_FLAG_FUSION_RESET;
		cmd_name = "Fusion reset";
	} else if (strcmp(arg2, "channel") == 0) {
		if (!arg3) {
			printk("Usage: send all channel <0-100>\n");
			printk("Example: send all channel 25 - Set all active trackers to channel 25\n");
			return;
		}

		long channel;
		if (!parse_long_bounded(arg3, 0, 100, &channel)) {
			printk("Invalid channel. Must be a number between 0 and 100.\n");
			return;
		}

		if (!target_all) {
			printk("Channel command only supports 'all' target\n");
			return;
		}

		uint8_t st = rcv_cmd_tracker_channel_all((uint8_t)channel);
		if (st == RCV_HID_ST_STARTED || st == RCV_HID_ST_OK) {
			printk("Setting RF channel to %d for all active trackers and receiver\n", (int)channel);
		}
		return;
	} else if (strcmp(arg2, "clearchannel") == 0) {
		if (!target_all) {
			printk("Clearchannel command only supports 'all' target\n");
			return;
		}

		uint8_t st = rcv_cmd_tracker_channel_clear_all();
		if (st == RCV_HID_ST_STARTED || st == RCV_HID_ST_OK) {
			printk("Clearing RF channel for all active trackers and receiver\n");
		}
		return;
	} else if (strcmp(arg2, "sens") == 0) {
		if (!arg3) {
			printk("Usage: send <id|all> sens <x>,<y>,<z>\n");
			printk("Usage: send <id|all> sens reset\n");
			printk("Usage: send <id|all> sens auto <x|y|z> [revolutions]\n");
			printk("Example: send 0 sens 1.0,1.0,1.0\n");
			printk("Example: send all sens reset\n");
			printk("Example: send 0 sens auto z 5\n");
			return;
		}

		if (strcmp(arg3, "reset") == 0) {
			console_send_flag(target_all, tracker_id, ESB_PONG_FLAG_SENS_RESET, "Sens reset");
		} else if (strcmp(arg3, "auto") == 0) {
			if (!arg4 || arg4[0] == '\0') {
				printk("Usage: send <id|all> sens auto <x|y|z> [revolutions]\n");
				return;
			}

			if (arg4[1] != '\0') {
				printk("Usage: send <id|all> sens auto <x|y|z> [revolutions]\n");
				return;
			}

			uint8_t axis;
			if (arg4[0] == 'x') {
				axis = 0;
			} else if (arg4[0] == 'y') {
				axis = 1;
			} else if (arg4[0] == 'z') {
				axis = 2;
			} else {
				printk("Invalid axis '%s'. Use x, y, or z.\n", arg4);
				return;
			}

			uint16_t revolutions = 0;
			if (arg5 && *arg5) {
				long value;
				if (!parse_long_bounded(arg5, 1, SENS_AUTO_MAX_REVOLUTIONS, &value)) {
					printk("Invalid revolutions '%s'. Use 1 to %u.\n", arg5, SENS_AUTO_MAX_REVOLUTIONS);
					return;
				}
				revolutions = (uint16_t)value;
			}

			uint8_t tid = target_all ? RCV_HID_TARGET_ALL : tracker_id;
			uint8_t st = rcv_cmd_remote_sens_auto(tid, axis, revolutions);
			if (st == RCV_HID_ST_QUEUED) {
				if (target_all) {
					if (revolutions == 0) {
						printk(
							"Sens auto request sent to all active trackers on %c axis using tracker "
							"default rev\n",
							'x' + axis
						);
					} else {
						printk(
							"Sens auto request sent to all active trackers on %c axis for %u rev\n",
							'x' + axis,
							revolutions
						);
					}
				} else if (revolutions == 0) {
					printk(
						"Sens auto request sent to tracker %d on %c axis using tracker default rev\n",
						tracker_id,
						'x' + axis
					);
				} else {
					printk(
						"Sens auto request sent to tracker %d on %c axis for %u rev\n",
						tracker_id,
						'x' + axis,
						revolutions
					);
				}
			} else if (!target_all) {
				printk("Sens auto request not queued: tracker %d is not active or invalid\n", tracker_id);
			}
		} else {
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
				uint8_t tid = target_all ? RCV_HID_TARGET_ALL : tracker_id;
				if (rcv_cmd_remote_sens_set(tid, values[0], values[1], values[2]) == RCV_HID_ST_QUEUED) {
					if (target_all) {
						printk(
							"Sens set (%.2f,%.2f,%.2f) request sent to all active trackers\n",
							(double)values[0],
							(double)values[1],
							(double)values[2]
						);
					} else {
						printk(
							"Sens set (%.2f,%.2f,%.2f) request sent to tracker %d\n",
							(double)values[0],
							(double)values[1],
							(double)values[2],
							tracker_id
						);
					}
				}
			} else {
				printk("Error: Invalid format. Use: sens <x>,<y>,<z> or sens reset\n");
				printk("Example: sens 10.5,-2.1,15.0\n");
			}
		}
		return;
	} else if (strcmp(arg2, "reset") == 0) {
		if (!arg3) {
			printk("Usage: send <id|all> reset <zro|acc|bat|mag|tcal|fusion>\n");
			printk("Example: send 0 reset zro\n");
			printk("Example: send all reset acc\n");
			return;
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
			return;
		}

		console_send_flag(target_all, tracker_id, reset_cmd, reset_name);
		return;
	} else if (strcmp(arg2, "ping") == 0) {
		console_send_flag(target_all, tracker_id, ESB_PONG_FLAG_PING, "Ping");
		return;
	} else if (strcmp(arg2, "tcal") == 0) {
		if (!arg3) {
			printk("Usage: send <id|all> tcal <on|off|auto on|auto off|boot on|boot off|clear|heat start>\n");
			printk("Example: send 0 tcal on       - Enable temperature calibration on tracker 0\n");
			printk(
				"Example: send all tcal off    - Disable temperature calibration on all active trackers\n"
			);
			printk("Example: send 0 tcal auto on  - Enable auto-calibration on tracker 0\n");
			printk("Example: send all tcal auto off - Disable auto-calibration on all active trackers\n");
			printk("Example: send 0 tcal boot on - Enable boot calibration on tracker 0\n");
			printk("Example: send 0 tcal clear - Clear temperature calibration on tracker 0\n");
			printk("Example: send 0 tcal heat start - Request heated calibration at the tracker default target\n");
			printk("Heated-start ACK confirms delivery only, not heating acceptance.\n");
			return;
		}

		if (strcmp(arg3, "heat") == 0) {
			if (!arg4 || strcmp(arg4, "start") != 0 || arg5) {
				printk("Usage: send <id|all> tcal heat start (no target argument)\n");
				return;
			}
			console_send_flag(target_all, tracker_id, ESB_PONG_FLAG_TCAL_HEATED_START, "Heated T-Cal start");
		} else if (strcmp(arg3, "on") == 0) {
			console_send_flag(target_all, tracker_id, ESB_PONG_FLAG_TCAL_ON, "T-Cal enable");
		} else if (strcmp(arg3, "off") == 0) {
			console_send_flag(target_all, tracker_id, ESB_PONG_FLAG_TCAL_OFF, "T-Cal disable");
		} else if (strcmp(arg3, "auto") == 0) {
			if (!arg4) {
				printk("Usage: send <id|all> tcal auto <on|off>\n");
				return;
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
				return;
			}

			console_send_flag(target_all, tracker_id, tcal_cmd, tcal_name);
		} else if (strcmp(arg3, "clear") == 0) {
			console_send_flag(target_all, tracker_id, ESB_PONG_FLAG_RESET_TCAL, "T-Cal clear");
		} else if (strcmp(arg3, "boot") == 0) {
			if (!arg4) {
				printk("Usage: send <id|all> tcal boot <on|off>\n");
				return;
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
				return;
			}

			console_send_flag(target_all, tracker_id, tcal_cmd, tcal_name);
		} else {
			printk("Unknown tcal subcommand: %s (use 'on', 'off', 'auto', 'boot', 'clear' or 'heat start')\n", arg3);
		}
		return;
	} else if (strcmp(arg2, "tdma") == 0) {
		if (!arg3) {
			printk("Usage: send <id|all> tdma <on|off>\n");
			printk("Example: send 0 tdma on       - Enable TDMA scheduling on tracker 0\n");
			printk("Example: send all tdma off    - Disable TDMA scheduling on all active trackers\n");
			return;
		}

		if (strcmp(arg3, "on") == 0) {
			console_send_flag(target_all, tracker_id, ESB_PONG_FLAG_TDMA_ON, "TDMA enable");
		} else if (strcmp(arg3, "off") == 0) {
			console_send_flag(target_all, tracker_id, ESB_PONG_FLAG_TDMA_OFF, "TDMA disable");
		} else {
			printk("Unknown tdma subcommand: %s (use 'on' or 'off')\n", arg3);
		}
		return;
	} else if (strcmp(arg2, "test") == 0) {
		if (!arg3) {
			printk("Usage: send <id|all> test <on|off> [tps]\n");
			printk("Example: send 0 test on       - Enable test mode on tracker 0 (default rate)\n");
			printk("Example: send all test on 200 - Enable test mode, target 200 TPS\n");
			printk("  TPS is clamped to the tracker's TDMA slot capacity; 0 = built-in default.\n");
			return;
		}

		if (strcmp(arg3, "on") == 0) {
			/* One test-specific command carries the optional TPS (0 =
			 * built-in default) and the ON flag, so the rate is published
			 * before the tracker sees the flag. */
			uint16_t tps = 0;
			if (arg4 && *arg4) {
				long v;
				if (!parse_long_bounded(arg4, 0, 1000, &v)) {
					printk("Invalid TPS. Must be 0-1000 (0 = default).\n");
					return;
				}
				tps = (uint16_t)v;
			}
			uint8_t st = rcv_cmd_remote_test_on(target_all ? RCV_HID_TARGET_ALL : tracker_id, tps);
			if (st == RCV_HID_ST_OK || st == RCV_HID_ST_STARTED) {
				if (target_all) {
					printk(
						"Test mode enable sent to all active trackers (target %u TPS%s)\n",
						tps,
						tps == 0 ? ", built-in default" : ""
					);
				} else {
					printk(
						"Test mode enable sent to tracker %d (target %u TPS%s)\n",
						tracker_id,
						tps,
						tps == 0 ? ", built-in default" : ""
					);
				}
			} else if (st == RCV_HID_ST_ENOENT) {
				printk("Test mode enable failed: no active trackers\n");
			} else {
				printk("Test mode enable failed: status %u\n", st);
			}
		} else if (strcmp(arg3, "off") == 0) {
			uint8_t st = rcv_cmd_remote_test_off(target_all ? RCV_HID_TARGET_ALL : tracker_id);
			if (st == RCV_HID_ST_OK || st == RCV_HID_ST_STARTED) {
				if (target_all) {
					printk("Test mode disable sent to all active trackers\n");
				} else {
					printk("Test mode disable sent to tracker %d\n", tracker_id);
				}
			} else if (st == RCV_HID_ST_ENOENT) {
				printk("Test mode disable failed: no active trackers\n");
			} else {
				printk("Test mode disable failed: status %u\n", st);
			}
		} else {
			printk("Unknown test subcommand: %s (use 'on [tps]' or 'off')\n", arg3);
		}
		return;
	}

	if (cmd_flag != 0xFF) {
		console_send_flag(target_all, tracker_id, cmd_flag, cmd_name);
	} else {
		printk("Unknown command: %s\n", arg2);
		printk(
			"Available commands: shutdown, calibrate [acc], meow, scan, mag, reboot, clear, dfu [ota], "
			"fusion, sens, "
			"reset, ping, tcal, tdma, test\n"
		);
	}
}
