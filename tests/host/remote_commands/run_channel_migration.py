#!/usr/bin/env python3
"""Execute receiver channel transactions and ACK payload branches from production."""
import importlib.util
import os
from pathlib import Path
import shlex
import re
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
spec = importlib.util.spec_from_file_location("extract", HERE.parent / "tdma_loss/run.py")
extract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(extract)
source = (ROOT / "src/connection/esb.c").read_text()
header = (ROOT / "src/connection/esb.h").read_text()
functions = [
    "decision_accept", "esb_auto_loss_invalidate", "esb_rx_discontinuity", "check_packet_sequence",
    "esb_channel_is_busy", "radio_operation_begin", "esb_radio_operation_begin",
    "esb_radio_collection_begin", "esb_radio_operation_end",
    "channel_command_for_tracker", "esb_send_remote_command_arg", "esb_send_remote_command",
    "channel_apply", "channel_change_begin", "channel_change_poll",
    "esb_set_all_trackers_channel", "esb_clear_all_trackers_channel",
    "esb_auto_channel_request", "esb_set_receiver_channel", "esb_clear_receiver_channel",
    "esb_get_receiver_channel",
    "remote_wom_receive", "esb_send_remote_wom_command",
    "esb_get_remote_wom_result", "esb_cancel_remote_wom_command",
]
start = source.index("/* Migration owns an ACK overlay")
end = source.index("bool esb_channel_is_busy", start)
production = source[start:end]
production += extract.block(source, source.index('struct packet_stats {')) + ';\n'
production += 'static struct packet_stats tracker_stats[MAX_TRACKERS];\n'
production += '\n'.join(re.findall(
    r'^static uint32_t decision_\w+[^;]*;', source, re.MULTILINE)) + '\n'
production += extract.function(header, "esb_rf_channel_encode")
production += extract.function(header, "esb_rf_channel_decode")
production += "\n".join(extract.function(source, name) for name in functions)
# Execute exactly the production migration selection and channel wire encoding,
# with only the unrelated TDMA/metadata ACK fields omitted.
start = source.index("uint8_t migration_cmd = channel_command_for_tracker")
overlay = source[start:source.index(";", start) + 1] + "\n"
overlay += extract.block(source, source.index("if (migration_cmd != ESB_PONG_FLAG_NORMAL)", start))
start = source.index("if (cmd == ESB_PONG_FLAG_SET_CHANNEL)")
encoding = extract.block(source, start)
production += "\nstatic uint8_t pong(uint8_t tracker_id, struct payload *ack_payload) {\n"
production += "uint8_t cmd = tracker_remote_command[tracker_id];\n" + overlay + encoding
production += "return cmd;\n}\n"
fixture = (HERE / "test_channel_migration.c").read_text()
prelude, cases = fixture.split("/* PRODUCTION BODIES */")
prelude = prelude.replace("/* DIAGNOSTIC DECLARATIONS */",
                          extract.diagnostic_declarations(source))
# Exercise the complete production ACK callback, including pairing, channel
# confirmation priority, command formats, OTA override and CRC generation.
defines = []
for line in header.splitlines():
    match = re.match(r"#define (ESB_(?:(?:PING|PONG|RAW|OTA|CHANNEL_CONFIRM|WOM_TIMEOUT)_\w+|WOM_ENTER))\s+", line)
    if match and not re.search(r"#define\s+" + match[1] + r"\b", prelude):
        defines.append(line)
defines.extend(re.findall(r'^#define (?:PING_TIMEOUT_MS|PING_CLEAN_REACQUIRE_STREAK)\b[^\n]*',
                          source, re.MULTILINE))
prelude = "\n".join(defines) + "\n" + prelude
production += "\n" + extract.function(source, "esb_ack_handler_cb")
production += "\n" + extract.diagnostic_function(source, "account_ping_clock")
production += "\n" + extract.function(source, "esb_receive_ping")
production += """
static void echo(uint8_t tracker_id, uint8_t flag) {
    struct esb_payload packet = {.length = ESB_PING_LEN, .pipe = 1 + tracker_id % 7};
    packet.data[0] = ESB_PING_TYPE;
    packet.data[1] = tracker_id;
    packet.data[2] = ++echo_counter[tracker_id];
    packet.data[7] = flag;
    packet.data[ESB_PING_LEN - 1] = crc8_ccitt(0x07, packet.data, ESB_PING_LEN - 1);
    esb_receive_ping(&packet, (uint32_t)now);
}
"""
control = (ROOT / "src/rcv_cmd.c").read_text()
production += "\nuint8_t rcv_cmd_collect_stop(void);\nuint8_t rcv_cmd_collect_batch_stop(void);\n"
production += "\n".join(extract.function(control, name) for name in (
    "rcv_cmd_collect_start", "rcv_cmd_collect_stop",
    "rcv_cmd_collect_batch_start", "rcv_cmd_collect_batch_stop"))
with tempfile.TemporaryDirectory(prefix="receiver-channel-migration-") as directory:
    temporary = Path(directory)
    generated = temporary / "test.c"
    generated.write_text(prelude + production + cases)
    binary = temporary / "test"
    for diagnostics in (False, True):
        flags = ["-DCONFIG_TDMA_DIAGNOSTICS=1"] if diagnostics else []
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
            "-Wno-unused-function", "-Wno-unused-variable", "-g", "-O1",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
            *flags, str(generated), "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
