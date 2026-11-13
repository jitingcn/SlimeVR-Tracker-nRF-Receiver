#!/usr/bin/env python3
"""Run verbatim receiver recovery paths with register/RTOS leaves modeled."""
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
parser = argparse.ArgumentParser()
parser.add_argument('--source-root', type=Path, default=ROOT)
parser.add_argument('--autoconf', type=Path,
                    help='Use a real generated Zephyr autoconf.h instead of synthetic configs')
parser.add_argument('--config-context', choices=('UF2', 'LEGACY', 'MCUBOOT'),
                    default='UF2', help='Expected behavior of --autoconf target')
args = parser.parse_args()
sys.path.insert(0, str(HERE.parent / 'harness'))
from c_extract import extract_block


def function(source, name):
    return extract_block(source, r'^[\w \t*]+\b' + re.escape(name) +
                         r'\s*\([^;{}]*\)\s*\{') + '\n'


system = (args.source_root / 'src/system/system.c').read_text()
main = (args.source_root / 'src/main.c').read_text()
constants = '\n'.join(re.findall(r'^#define (?:ADAFRUIT_\w+|DFU_EXISTS) .+$', system, re.M))
production = constants + '\nstatic uint32_t boot_reset_reason;\n'
production += '#if ADAFRUIT_RECOVERY_ENABLED\n'
production += re.search(r'^static bool bootloader_supports_recovery;', system, re.M).group(0)
production += '\n#endif\n'
header = (args.source_root / 'src/system/system.h').read_text()
production += re.search(r'^struct sys_fatal_record \{.*?^\};', header, re.M | re.S).group(0) + '\n'
production += '#define __noinit\n' + re.search(
    r'^static __noinit struct sys_fatal_record fatal_record;\n#define FATAL_RECORD_MAGIC .*$', system, re.M).group(0) + '\n'
production += '\n'.join(function(system, name) for name in (
    'sys_reset_reason_init', 'sys_mark_boot_success', 'sys_skip_dfu_marker',
    'fatal_check', 'sys_last_fatal', 'k_sys_fatal_error_handler', 'sys_button_init'))
production += function(main, 'boot_confirm_after_healthy_uptime')
production += '#define main receiver_main\n' + function(main, 'main') + '#undef main\n'
with tempfile.TemporaryDirectory(prefix='receiver-recovery-') as directory:
    tmp = Path(directory)
    (tmp / 'production.inc').write_text(production)
    contexts = (args.config_context,) if args.autoconf else ('UF2', 'LEGACY', 'MCUBOOT')
    for context in contexts:
        binary = tmp / context
        command = shlex.split(os.environ.get('CC', 'cc')) + [
            '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
            '-fsanitize=undefined', '-fno-sanitize-recover=all',
            '-DTEST_' + context, '-I', str(tmp), str(HERE / 'test_recovery.c'),
            '-o', str(binary)]
        if args.autoconf:
            command += ['-DTEST_GENERATED_CONFIG', '-include', str(args.autoconf.resolve())]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
