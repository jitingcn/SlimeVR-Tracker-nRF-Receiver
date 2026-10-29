#!/usr/bin/env python3
"""Exercise production OTA ownership with pthread leaves and controlled join errors.
The injected timeout is a lifecycle probe, not proof of Zephyr join semantics.
"""
import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--source-root', type=Path, default=Path(__file__).resolve().parents[3])
parser.add_argument('--case', action='append', choices=['begin_timeout', 'abort_timeout', 'verify_timeout', 'complete', 'activation_error', 'abort', 'data_payload'])
args = parser.parse_args()
source = (args.source_root / 'src/receiver_ota.c').read_text()
# Keep production declarations and every lifecycle function, replacing only the
# unrelated board-info leaf and hardware-only RAM copier.
source = source[:source.index('/* ── RAM-resident flash copier')]
source = re.sub(r'^#include[^\n]*\n', '', source, flags=re.M)
start = source.index('static void rcv_ota_send_fw_info(void)\n{')
end = source.index('/* ── STATUS', start)
source = source[:start] + 'static void rcv_ota_send_fw_info(void) {}\n\n' + source[end:]
fixtures = Path(__file__).with_name('fixture.c').read_text()
prelude, tests = fixtures.split('/* PRODUCTION_SOURCE */')
tests = tests.replace('int main(', Path(__file__).with_name('data_payload_cases.c').read_text() + '\nint main(', 1)
cases = args.case or ['begin_timeout', 'abort_timeout', 'verify_timeout', 'complete', 'activation_error', 'abort', 'data_payload']
protocol = (Path(__file__).resolve().parents[3] / 'src/esb_ota.h').read_text()
constants = '\n'.join(re.findall(r'^#define (?:OTA_|HID_OTA_)[^\n]+', protocol, re.M))
with tempfile.TemporaryDirectory(prefix='receiver-ota-writer-') as directory:
    root = Path(directory)
    (root / 'probe.c').write_text(prelude + constants + '\n' + source + tests)
    for mcuboot in (0, 1):
        executable = root / f'probe-{mcuboot}'
        subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
            '-std=gnu11', '-O1', '-g', '-pthread', '-no-pie',
            '-Wl,--defsym,_flash_used=4096', f'-DTEST_MCUBOOT={mcuboot}',
            str(root / 'probe.c'), '-o', str(executable),
        ], check=True)
        for case in cases:
            if not mcuboot and case in ('complete', 'activation_error'):
                continue
            subprocess.run([str(executable), case], check=True, timeout=20)
