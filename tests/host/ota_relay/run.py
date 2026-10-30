#!/usr/bin/env python3
import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--source-root', type=Path, default=Path(__file__).resolve().parents[3])
parser.add_argument('--expect-unsafe', action='store_true',
                    help='replay the mixed-command ACK against an unsynchronized source')
args = parser.parse_args()
source_root = args.source_root.resolve()
source_path = source_root / 'src/esb_ota.c'
fixture = Path(__file__).with_name('fixture.c')

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'harness'))
from c_extract import extract_block

radio_source = (source_root / 'src/connection/esb.c').read_text()
radio_functions = '\n\n'.join(
    extract_block(radio_source, r'^[\w \t*]+\b' + re.escape(name) +
                  r'\s*\([^;{}]*\)\s*\{')
    for name in ('radio_operation_begin', 'esb_radio_operation_begin',
                 'esb_radio_operation_end')
)

# The second build inserts scheduling points after scalar command stores.
# The first build includes the original source directly without instrumentation.
assignment = re.compile(
    r'(?m)^(?P<indent>[ \t]*)'
    r'(?P<lhs>[A-Za-z_]\w*(?:(?:->|\.)[A-Za-z_]\w*|\[[^\]\n]+\])*'
    r'(?:->|\.)pending_cmd_(?:type|len|set_ticks|send_count))'
    r'[ \t]*(?P<op>(?:=(?!=)|\+=|-=)[^;\n]*|\+\+|--);'
)

def add_store_point(match):
    lhs = match['lhs']
    return (f"{match['indent']}do {{ {lhs} {match['op']}; "
            f"test_after_store(&({lhs})); }} while (0);")

with tempfile.TemporaryDirectory(prefix='receiver-ota-relay-') as directory:
    root = Path(directory)
    radio_path = root / 'radio_admission.c'
    radio_path.write_text(radio_functions)
    for name in ('esb.h', 'zephyr/kernel.h', 'zephyr/sys/byteorder.h',
                 'zephyr/sys/crc.h', 'zephyr/logging/log.h'):
        stub = root / name
        stub.parent.mkdir(parents=True, exist_ok=True)
        stub.write_text('/* Host declarations are in fixture.c. */\n')
    if args.expect_unsafe:
        builds = [('expected-unsafe', source_path, 0)]
    else:
        instrumented, count = assignment.subn(add_store_point, source_path.read_text())
        if count < 4:
            raise RuntimeError(f'Only {count} command metadata stores were instrumented')
        instrumented_path = root / 'esb_ota_instrumented.c'
        instrumented_path.write_text(instrumented)
        builds = [('production', source_path, 0),
                  ('store-instrumented', instrumented_path, 1)]
    for name, source, enabled in builds:
        executable = root / name
        subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
            '-std=c11', '-O0', '-g', '-Wall', '-Wextra',
            '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
            '-I', str(root), '-I', str(source_root / 'src'),
            f'-DRELAY_SOURCE="{source}"', f'-DINSTRUMENTED={enabled}',
            f'-DRADIO_SOURCE="{radio_path}"',
            f'-DEXPECT_UNSAFE={int(args.expect_unsafe)}',
            str(fixture), '-o', str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True, timeout=60)
