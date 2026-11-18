#!/usr/bin/env python3
"""Run receiver boot/scan admission against clock manager and radio leaves."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2] / 'src'
sys.path.insert(0, str(HERE.parent / 'harness'))
from c_extract import extract_block


def function(source, name):
    return extract_block(source, rf'^(?:static )?(?:__attribute__\(\(noinline\)\) )?'
                         rf'(?:void|int|bool|uint8_t) {name}\([^;{{}}]*\)\s*\{{')


esb = (SRC / 'connection/esb.c').read_text()
scan = (SRC / 'connection/rssi_scan.c').read_text()
commands = (SRC / 'rcv_cmd.c').read_text()
admission = extract_block(esb, r'^enum esb_radio_startup \{', semicolon=True)
admission += '\n' + '\n'.join(re.findall(
    r'^static (?:atomic_t|bool) (?:radio_startup_state|radio_operation_busy|esb_radio_fault|esb_initialized)\b[^;]*;',
    esb, re.M))
admission += '\n' + '\n'.join(function(esb, name) for name in (
    'esb_channel_is_busy', 'radio_operation_begin', 'esb_radio_operation_begin',
    'esb_radio_collection_begin', 'esb_radio_self_ota_begin', 'esb_radio_operation_end',
    'esb_radio_quiesce', 'esb_radio_operation_stop'))
boot = esb[esb.index('#define HFXO_START_TIMEOUT_MS'):esb.index('/* ESB pipe addresses')]
boot += '\n' + function(esb, 'esb_thread')
scan_defines = '\n'.join(re.findall(
    r'^#define (?:RSSI_SCAN_(?:FIRST_CHANNEL|LAST_CHANNEL|SAMPLES_PER_CH|SWEEPS|INTER_SWEEP_MS)|RSSI_SAMPLE_MAX)\b[^\n]*',
    scan, re.M))
scan_body = scan[scan.index('static uint8_t get_current_effective_channel'):]
hid = (SRC / 'rcv_hid_cmd.h').read_text()
statuses = '\n'.join(re.findall(r'^#define RCV_HID_ST_(?:EBUSY|STARTED)\b[^\n]*', hid, re.M))
control = '\n'.join(function(commands, name) for name in (
    'rssi_scan_work_handler', 'rcv_cmd_rssi_scan'))
fixture = (HERE / 'radio_startup_fixture.c').read_text()
fixture = fixture.replace('/* ADMISSION */', admission)
fixture = fixture.replace('/* BOOT */', boot)
fixture = fixture.replace('/* SCAN */', '\n'.join((scan_defines, scan_body, statuses, control)))
with tempfile.TemporaryDirectory(prefix='receiver-radio-startup-') as directory:
    temporary = Path(directory)
    source = temporary / 'startup.c'
    source.write_text(fixture)
    binary = temporary / 'startup'
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
        '-std=c11', '-Wall', '-Wextra', '-Werror', '-g',
        '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
        str(source), '-o', str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True, timeout=10)
