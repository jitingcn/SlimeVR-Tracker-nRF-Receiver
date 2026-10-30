#!/usr/bin/env python3
"""Exercise production OTA ownership with pthread leaves and controlled failures.

Modeled semaphore and join failures return immediately. They do not measure
Zephyr wait durations, scheduling latency, or actual join behavior.
"""
import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

CASES = [
    'begin_timeout', 'abort_timeout', 'verify_timeout', 'complete',
    'activation_error', 'abort', 'data_payload', 'crc_settings_first',
    'crc_settings_later', 'crc_settings_ok', 'verify_enqueue_error',
    'activate_enqueue_error', 'admission_complete', 'admission_error',
    'activation_eligibility', 'slot_boundary', 'slot_reuse_crc',
    'slot_tail_short', 'slot_tail_padded', 'slot_exact_page', 'slot_blocked',
    'slot_queue_failure', 'slot_erase_failure', 'slot_write_failure',
    'slot_abort_partial', 'slot_join_timeout', 'slot_immediate',
    'slot_verify_fifo',
]

parser = argparse.ArgumentParser()
parser.add_argument('--source-root', type=Path, default=Path(__file__).resolve().parents[3])
parser.add_argument('--case', action='append', choices=CASES)
args = parser.parse_args()
source = (args.source_root / 'src/receiver_ota.c').read_text()
# Keep production declarations and every lifecycle function, replacing only the
# unrelated board-info leaf and hardware-only RAM copier.
source = source[:source.index('/* ── RAM-resident flash copier')]
source = re.sub(r'^#include[^\n]*\n', '', source, flags=re.M)
start = source.index('static void rcv_ota_send_fw_info(void)\n{')
end = source.index('/* ── STATUS', start)
source = source[:start] + 'static void rcv_ota_send_fw_info(void) {}\n\n' + source[end:]
source = source.replace(
    'static void rcv_ota_send_status(void)\n{',
    'static void rcv_ota_send_status(void)\n{\n\tif (status_entry_hook) status_entry_hook();',
)
fixtures = Path(__file__).with_name('fixture.c').read_text()
prelude, tests = fixtures.split('/* PRODUCTION_SOURCE */')
scenario_dir = Path(__file__).parent
cases_source = '\n'.join(
    (scenario_dir / filename).read_text()
    for filename in ('data_payload_cases.c', 'slot_cases.c')
)
tests = tests.replace('int main(', cases_source + '\nint main(', 1)
tests = tests.replace('/* CRC_SETTINGS_CASES */', (scenario_dir / 'crc_settings_cases.c').read_text())
cases = args.case or CASES
protocol = (Path(__file__).resolve().parents[3] / 'src/esb_ota.h').read_text()
constants = '\n'.join(re.findall(r'^#define (?:OTA_|HID_OTA_)[^\n]+', protocol, re.M))
with tempfile.TemporaryDirectory(prefix='receiver-ota-writer-') as directory:
    root = Path(directory)
    (root / 'probe.c').write_text(prelude + constants + '\n' + source + tests)
    for mcuboot, uf2 in ((0, 0), (1, 0), (0, 1)):
        executable = root / f'probe-{mcuboot}-{uf2}'
        subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
            '-std=gnu11', '-O1', '-g', '-pthread', '-no-pie',
            '-Wl,--defsym,_flash_used=4096', f'-DTEST_MCUBOOT={mcuboot}',
            f'-DTEST_UF2={uf2}',
            str(root / 'probe.c'), '-o', str(executable),
        ], check=True)
        for case in cases:
            if case.startswith('crc_settings_') and not uf2:
                continue
            if mcuboot and case == 'slot_erase_failure':
                continue
            if not mcuboot and case in ('complete', 'activation_error', 'admission_complete', 'admission_error'):
                continue
            subprocess.run([str(executable), case], check=True, timeout=20)
