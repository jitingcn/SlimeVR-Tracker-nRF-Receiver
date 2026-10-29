#!/usr/bin/env python3
"""Exercise the actual receiver ACK and complete FIFO event dispatch.

SDK, scheduler and output leaves are modeled. --source accepts a baseline esb.c.
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
         'event-fallback', 'valid-after-invalid', 'duplicate', 'out-of-order', 'drain')
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source', type=Path, default=ROOT / 'src/connection/esb.c')
parser.add_argument('--case', action='append', choices=CASES)
args = parser.parse_args()
source = args.source.read_text()
header = (ROOT / 'src/connection/esb.h').read_text()
auto_source = (ROOT / 'src/connection/auto_channel.c').read_text()


def function(name):
    return extract_block(source, rf'^(?:static\s+)?void\s+{name}\s*\([^;{{}}]*\)\s*\{{')


constants = '\n'.join(re.findall(
    r'^#define ESB_(?:PING|PONG|CHANNEL|COMPOSITE|OTA|RAW)_\w+\s+[^\n]+', header, re.M))
ack = function('esb_ack_handler_cb').split('\t/* Other packet types:', 1)[0] + '\n}\n'
event = function('event_handler')
clock = function('account_ping_clock') if re.search(r'^static void account_ping_clock\s*\(', source, re.M) else ''
lengths = extract_block(source, r'^static int composite_body_length\s*\([^;{}]*\)\s*\{')
packet_helpers = '\n'.join(function(name) for name in ('esb_receive_ping', 'esb_receive_composite')
                           if re.search(rf'^static void {name}\s*\(', source, re.M))
fixture = (HERE / 'fixture.c').read_text()
auto_channel = extract_block(auto_source, r'^bool auto_channel_is_beacon\s*\([^;{}]*\)\s*\{')
auto_enabled_source = auto_source[:auto_source.rindex('\nbool auto_channel_receive')]
auto_channel += '\n' + extract_block(
    auto_enabled_source, r'^bool auto_channel_receive\s*\([^;{}]*\)\s*\{')
fixture = fixture.replace('/* ACTUAL AUTO CHANNEL */', auto_channel)
fixture = fixture.replace('/* PROTOCOL CONSTANTS */', constants)
fixture = fixture.replace('/* ACTUAL PRODUCTION FUNCTIONS */',
                          '\n\n'.join((lengths, clock, ack, packet_helpers, event)))

with tempfile.TemporaryDirectory(prefix='receiver-ping-clock-') as directory:
    directory = Path(directory)
    generated = directory / 'fixture.c'
    generated.write_text(fixture)
    executable = directory / 'fixture'
    compiler = shlex.split(os.environ.get('CC', 'cc'))
    subprocess.run(compiler + [
        '-std=c11', '-g', '-O1', '-Wall', '-Wextra',
        '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
        '-fno-pie', '-no-pie', '-I', str(ROOT / 'src'), str(generated), '-o', str(executable),
    ], check=True)
    failures = []
    for case in args.case or CASES:
        result = subprocess.run([str(executable), case], timeout=20)
        if result.returncode:
            failures.append(case)
    if failures:
        raise SystemExit('Failed cases: ' + ', '.join(failures))
