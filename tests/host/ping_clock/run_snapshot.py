#!/usr/bin/env python3
"""Exercise receiver diagnostic timestamp snapshots under modeled RADIO preemption."""

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

source = (ROOT / 'src/connection/esb.c').read_text()
header = (ROOT / 'src/connection/esb.h').read_text()
auto_source = (ROOT / 'src/connection/auto_channel.c').read_text()
constants = '\n'.join(re.findall(
    r'^#define (?:RX_PHASE_MIN_TICKS|RX_PHASE_BUCKETS|RX_RSSI_MAX_DBM|RX_RSSI_BUCKETS|TDMA_SYNC_EXTRAP_MAX_TICKS|PING_CLEAN_REACQUIRE_STREAK)\b[^\n]*',
    source, re.M))
protocol = '\n'.join(re.findall(
    r'^#define ESB_(?:PING|PONG|CHANNEL|COMPOSITE|OTA|RAW)_\w+\s+[^\n]+', header, re.M))
lock_declarations = re.findall(r'^static struct k_spinlock tdma_timestamp_lock;', source, re.M)
if len(lock_declarations) != 1:
    raise ValueError(f'Expected one timestamp lock declaration, got {len(lock_declarations)}')
shape = '\n'.join((
    constants,
    extract_block(source, r'^struct packet_stats\s*\{', semicolon=True),
    extract_block(source, r'^struct tracker_slot\s*\{', semicolon=True),
    'static struct tracker_slot tracker_slots[MAX_TRACKERS];',
    '#if defined(CONFIG_TDMA_DIAGNOSTICS)',
    extract_block(source, r'^struct tdma_stats\s*\{', semicolon=True),
    extract_block(source, r'^struct tracker_diag\s*\{', semicolon=True),
    'static struct tracker_diag tracker_diags[MAX_TRACKERS];',
    lock_declarations[0],
    '#endif',
))


def instrument(name, fields):
    body = extract_block(source, rf'^static void {name}\s*\([^;{{}}]*\)\s*\{{')
    for field, checked in fields:
        expression = f'tracker_diags[tracker_id].{field}'
        body, count = re.subn(re.escape(expression) + r'\b',
                              f'{checked}({expression})', body)
        if count != 1:
            raise ValueError(f'{name}: expected one read of {field}, got {count}')
    return body


clock = instrument('account_ping_clock', (
    ('g_ping_isr_rx_ticks_valid', 'checked_ping_valid'),
    ('g_ping_isr_rx_ticks', 'checked_ping_ticks'),
))
slot = instrument('tdma_check_slot', (
    ('g_last_isr_rx_valid', 'checked_slot_valid'),
    ('g_last_isr_rx_ticks', 'checked_slot_ticks'),
))
ack = extract_block(source, r'^static void esb_ack_handler_cb\s*\([^;{}]*\)\s*\{')
ack = ack.split('\t/* Other packet types:', 1)[0] + '\n}\n'
for index, field, target, point in (
        ('tid', 'g_last_isr_rx_ticks', 'SLOT_TIMESTAMP', 'TICKS_WRITE'),
        ('tid', 'g_last_isr_rx_valid', 'SLOT_TIMESTAMP', 'VALID_WRITE'),
        ('tracker_id', 'g_ping_isr_rx_ticks', 'PING_TIMESTAMP', 'TICKS_WRITE'),
        ('tracker_id', 'g_ping_isr_rx_ticks_valid', 'PING_TIMESTAMP', 'VALID_WRITE')):
    expression = f'tracker_diags[{index}].{field}'
    pattern = r'(^[ \t]*' + re.escape(expression) + r'\s*=[^;\n]*;)'
    ack, count = re.subn(pattern, rf'\1\n\tattempt_preemption({target}, {point});', ack, flags=re.M)
    if count != 1:
        raise ValueError(f'esb_ack_handler_cb: expected one store of {field}, got {count}')
ack = ack.replace('k_uptime_ticks()', 'snapshot_uptime_ticks()')

prelude = (HERE / 'fixture.c').read_text().split('/* ACTUAL AUTO CHANNEL */', 1)[0]
stub = extract_block(prelude, r'^static void tdma_check_slot\s*\([^;{}]*\)\s*\{')
prelude = prelude.replace(stub, '')
prelude = prelude.replace('/* PROTOCOL CONSTANTS */', protocol)
prelude = prelude.replace('/* PRODUCTION STATE */', shape)
auto_channel = extract_block(auto_source, r'^bool auto_channel_is_beacon\s*\([^;{}]*\)\s*\{')
fixture = prelude + '\n' + auto_channel + '\n' + (HERE / 'snapshot_fixture.c').read_text()
fixture = fixture.replace('/* PRODUCTION FUNCTIONS */',
                          '#if defined(CONFIG_TDMA_DIAGNOSTICS)\n' + clock + '\n' + slot + '\n#endif\n' + ack)
with tempfile.TemporaryDirectory(prefix='receiver-timestamp-snapshot-') as directory:
    directory = Path(directory)
    generated = directory / 'fixture.c'
    generated.write_text(fixture)
    compiler = shlex.split(os.environ.get('CC', 'cc'))
    for diagnostics in (False, True):
        executable = directory / ('snapshot-on' if diagnostics else 'snapshot-off')
        flags = ['-DTEST_TIMESTAMP_SNAPSHOT=1']
        flags += ['-DCONFIG_TDMA_DIAGNOSTICS=1'] if diagnostics else ['-UCONFIG_TDMA_DIAGNOSTICS']
        subprocess.run(compiler + [
            '-std=c11', '-g', '-O1', '-Wall', '-Wextra',
            '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
            '-fno-pie', '-no-pie', '-I', str(ROOT / 'src'), *flags, str(generated), '-o', str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True, timeout=20)
