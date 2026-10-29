#!/usr/bin/env python3
"""Exercise actual RADIO ACK and 13-byte EVENT dispatch with modeled hardware leaves.

Other packet-length cases, composite handling, RF timing and RTOS scheduling
are outside this host fixture. --source accepts an unmodified baseline esb.c.
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
         'event-fallback', 'valid-after-invalid', 'duplicate', 'out-of-order')
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source', type=Path, default=ROOT / 'src/connection/esb.c')
parser.add_argument('--case', action='append', choices=CASES)
args = parser.parse_args()
source = args.source.read_text()
header = (ROOT / 'src/connection/esb.h').read_text()


def function(name):
    return extract_block(source, rf'^(?:static\s+)?void\s+{name}\s*\([^;{{}}]*\)\s*\{{')


constants = '\n'.join(re.findall(
    r'^#define ESB_(?:PING|PONG|CHANNEL|COMPOSITE|OTA)_\w+\s+[^\n]+', header, re.M))
ack = function('esb_ack_handler_cb').split('\t/* Other packet types:', 1)[0] + '\n}\n'
event = function('event_handler')
event = event.split('\t\t\tswitch (rx_payload.length) {', 1)[0]
ping_case = extract_block(source, r'^[ \t]*case ESB_PING_LEN:\s*\{')
event += ('\t\t\tswitch (rx_payload.length) {\n' + ping_case + '\n'
          '\t\t\tbreak;\n\t\t\tdefault: break;\n\t\t\t}\n'
          '\t\thandle_composite_packet:;\n\t\t}\n\t\tbreak;\n\t}\n\t}\n}\n')
helper = function('account_ping_clock') if re.search(r'^static void account_ping_clock\s*\(', source, re.M) else ''
fixture = (HERE / 'fixture.c').read_text()
fixture = fixture.replace('/* PROTOCOL CONSTANTS */', constants)
fixture = fixture.replace('/* ACTUAL PRODUCTION FUNCTIONS */', '\n\n'.join((helper, ack, event)))

with tempfile.TemporaryDirectory(prefix='receiver-ping-clock-') as directory:
    directory = Path(directory)
    generated = directory / 'fixture.c'
    generated.write_text(fixture)
    executable = directory / 'fixture'
    compiler = shlex.split(os.environ.get('CC', 'cc'))
    subprocess.run(compiler + [
        '-std=c11', '-g', '-O1', '-Wall', '-Wextra',
        '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
        '-fno-pie', '-no-pie', str(generated), '-o', str(executable),
    ], check=True)
    failures = []
    for case in args.case or CASES:
        result = subprocess.run([str(executable), case], timeout=20)
        if result.returncode:
            failures.append(case)
    if failures:
        raise SystemExit('Failed cases: ' + ', '.join(failures))
