#!/usr/bin/env python3
"""Compare receiver ACK and FIFO dispatch with diagnostics undefined and enabled.

Both variants assert protocol behavior and compare their wire/output traces.
SDK and work scheduling are modeled. --source accepts a baseline esb.c.
"""

import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent / 'harness'))
from c_extract import extract_block

CASES = ('wrong-type', 'bad-crc', 'wrong-pipe', 'out-of-range', 'interleaved-pipe',
         'ota-status', 'ota-info', 'valid', 'valid-dirty', 'reacquire',
         'event-fallback', 'valid-after-invalid', 'duplicate', 'out-of-order', 'drain',
         'detailed-warning', 'late-metadata', 'pairing-gate', 'status-admission')
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source', type=Path, default=ROOT / 'src/connection/esb.c')
parser.add_argument('--case', action='append', choices=CASES)
args = parser.parse_args()
source = args.source.read_text()
header = (ROOT / 'src/connection/esb.h').read_text()
auto_source = (ROOT / 'src/connection/auto_channel.c').read_text()
hid_source = (ROOT / 'src/hid.c').read_text()
hid_cmd = (ROOT / 'src/rcv_hid_cmd.h').read_text()


def function(name):
    return extract_block(source, rf'^(?:static\s+)?void\s+{name}\s*\([^;{{}}]*\)\s*\{{')


constants = '\n'.join(re.findall(
    r'^#define ESB_(?:PING|PONG|CHANNEL|COMPOSITE|OTA|RAW)_\w+\s+[^\n]+', header, re.M))
shape_constants = '\n'.join(re.findall(
    r'^#define (?:RX_PHASE_BUCKETS|RX_RSSI_BUCKETS)\b[^\n]*',
    source, re.M))
shape = '\n'.join((
    shape_constants,
    extract_block(source, r'^struct packet_stats\s*\{', semicolon=True),
    extract_block(source, r'^struct tracker_slot\s*\{', semicolon=True),
    'static struct tracker_slot tracker_slots[MAX_TRACKERS];',
    '#if defined(CONFIG_TDMA_DIAGNOSTICS)',
    extract_block(source, r'^struct tdma_stats\s*\{', semicolon=True),
    extract_block(source, r'^struct tracker_diag\s*\{', semicolon=True),
    'static struct tracker_diag tracker_diags[MAX_TRACKERS];',
    '\n'.join(re.findall(r'^static struct k_spinlock tdma_timestamp_lock;', source, re.M)),
    '#endif',
))
ack = function('esb_ack_handler_cb').split('\t/* Other packet types:', 1)[0] + '\n}\n'
event = function('event_handler')
clock = function('account_ping_clock') if re.search(r'^static void account_ping_clock\s*\(', source, re.M) else ''
clock = '#if defined(CONFIG_TDMA_DIAGNOSTICS)\n' + clock + '\n#endif\n'
lengths = extract_block(source, r'^static int composite_body_length\s*\([^;{}]*\)\s*\{')
packet_helpers = '\n'.join(function(name) for name in (
    'esb_receive_ping', 'esb_forward_tracker_packet', 'esb_receive_composite'
) if re.search(rf'^static void {name}\s*\(', source, re.M))
hid_bodies = '\n'.join((
    *re.findall(r'^#define RCV_HID_TYPE_CMD(?:_ACK)?\s+[^\n]+', hid_cmd, re.M),
    hid_source[hid_source.index('#define MAX_REPORTS'):hid_source.index('static bool hid_fifo_is_empty')],
    *re.findall(r'^#define RSSI_EMA_ALPHA\b[^\n]*', hid_source, re.M),
    extract_block(hid_source, r'^struct rssi_ema_state\s*\{', True),
    *re.findall(r'^static [^\n]+\b(?:rssi_states|dropped_reports|max_dropped_reports|'
                r'total_dropped_reports|tracker_drops|total_tracker_drops)\b[^;\n]*;', hid_source, re.M),
    *(extract_block(hid_source, rf'^[\w \t]+\b{name}\s*\([^;{{}}]*\)\s*\{{') for name in (
        'rssi_smooth_update', 'hid_type_byte1_is_tracker_id', 'hid_type_has_rssi_slot',
        'hid_enqueue_packet', 'hid_write_packet_n')),
))
fixture = (HERE / 'fixture.c').read_text()
auto_channel = extract_block(auto_source, r'^bool auto_channel_is_beacon\s*\([^;{}]*\)\s*\{')
auto_enabled_source = auto_source[:auto_source.rindex('\nbool auto_channel_receive')]
auto_channel += '\n' + extract_block(
    auto_enabled_source, r'^bool auto_channel_receive\s*\([^;{}]*\)\s*\{')
fixture = fixture.replace('/* ACTUAL AUTO CHANNEL */', auto_channel)
fixture = fixture.replace('/* PROTOCOL CONSTANTS */', constants)
fixture = fixture.replace('/* PRODUCTION STATE */', shape)
fixture = fixture.replace('/* ACTUAL HID FIFO */', hid_bodies)
fixture = fixture.replace('/* ACTUAL PRODUCTION FUNCTIONS */',
                          '\n\n'.join((lengths, clock, ack, packet_helpers, event)))

with tempfile.TemporaryDirectory(prefix='receiver-ping-clock-') as directory:
    directory = Path(directory)
    generated = directory / 'fixture.c'
    generated.write_text(fixture)
    compiler = shlex.split(os.environ.get('CC', 'cc'))
    failures = []
    protocol_outputs = {}
    for diagnostics in (False, True):
        variant = 'on' if diagnostics else 'off'
        executable = directory / ('fixture-' + variant)
        config = ['-DCONFIG_TDMA_DIAGNOSTICS=1'] if diagnostics else ['-UCONFIG_TDMA_DIAGNOSTICS']
        subprocess.run(compiler + [
            '-std=c11', '-g', '-O1', '-Wall', '-Wextra',
            '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
            '-fno-pie', '-no-pie', *config, '-I', str(ROOT / 'src'),
            str(generated), '-o', str(executable),
        ], check=True)
        for case in args.case or CASES:
            result = subprocess.run([str(executable), case], timeout=20,
                                    capture_output=True, text=True)
            print(f'diagnostics={variant} {case}')
            print(result.stdout, end='')
            print(result.stderr, end='', file=sys.stderr)
            if result.returncode:
                failures.append(f'{variant}/{case}')
            protocol = [line for line in result.stdout.splitlines()
                        if line.startswith('PROTOCOL ')]
            if diagnostics:
                if protocol != protocol_outputs[case]:
                    failures.append(f'protocol-mismatch/{case}')
            else:
                protocol_outputs[case] = protocol
    if failures:
        raise SystemExit('Failed cases: ' + ', '.join(failures))
