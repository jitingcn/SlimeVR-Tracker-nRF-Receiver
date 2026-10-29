#!/usr/bin/env python3
"""Exercise source-extracted receiver HEALTH output with diagnostics on and off.

Time, IRQ locking, HID counters, SDK radio diagnostics and logging are modeled.
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

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source', type=Path, default=ROOT / 'src/connection/esb.c')
args = parser.parse_args()
source = args.source.read_text()
console = (ROOT / 'src/console.c').read_text()
header = (ROOT / 'src/connection/esb.h').read_text()


def unique(pattern, text=source):
    matches = list(re.finditer(pattern, text, re.MULTILINE))
    if len(matches) != 1:
        raise ValueError(f'Expected one source match for {pattern}, got {len(matches)}')
    return matches[0].group(0)


health = unique(
    r'^#if defined\(CONFIG_TDMA_DIAGNOSTICS\)\n'
    r'static int32_t receiver_phase_percentile\([\s\S]*?'
    r'^#endif /\* CONFIG_TDMA_DIAGNOSTICS \*/')
export = unique(
    r'^#if defined\(CONFIG_TDMA_DIAGNOSTICS\)\n'
    r'void esb_print_health_snapshot\(void\);\n#endif', header)
command = unique(
    r'^#if defined\(CONFIG_TDMA_DIAGNOSTICS\)\n'
    r'\s*esb_print_health_snapshot\(\);\n#else\n'
    r'\s*printk\("health requires CONFIG_TDMA_DIAGNOSTICS=y\\n"\);\n#endif', console)
constants = '\n'.join(unique(rf'^#define {name}\s+[^\n]+') for name in (
    'RX_PHASE_MIN_TICKS', 'RX_PHASE_BUCKETS', 'RX_RSSI_MAX_DBM', 'RX_RSSI_BUCKETS',
    'TDMA_CAP_LEVEL_MAX', 'PING_TIMEOUT_MS'))
ladder = unique(r'^static const uint16_t tdma_cap_ladder\[\] = [^\n]+;')
packet_type = extract_block(source, r'^struct packet_stats\s*\{', semicolon=True)
phase_type = extract_block(source, r'^struct tdma_stats\s*\{', semicolon=True)
diag_names = (
    'g_tdma_stats', 'g_last_ping_rx_time_diff_ticks', 'g_last_ping_rx_time_diff_valid',
    'g_bias_ppb_valid', 'g_bias_ppb', 'g_last_ping_isr_rx_ticks_raw',
    'g_ping_reject_count', 'g_extrap_skip_count',
    'tdma_loss_last_permille', 'tdma_last_loss_step_ms',
)
diag_declarations = '\n'.join(unique(
    rf'^static[ \t]+(?:volatile[ \t]+)?(?:const[ \t]+)?'
    rf'(?:[A-Za-z_]\w*[ \t]+)+{re.escape(name)}\b[^;]*;') for name in diag_names)
fixture = (HERE / 'fixture.c').read_text()
fixture = fixture.replace('/* PRODUCTION TYPES */', '\n'.join((
    constants, ladder, packet_type,
    '#if defined(CONFIG_TDMA_DIAGNOSTICS)', phase_type, diag_declarations, '#endif',
)))
fixture = fixture.replace('/* PRODUCTION HEALTH EXPORT */', export)
fixture = fixture.replace('/* PRODUCTION HEALTH */', health)
fixture = fixture.replace('/* PRODUCTION HEALTH COMMAND */', command)

with tempfile.TemporaryDirectory(prefix='receiver-tdma-health-') as directory:
    directory = Path(directory)
    generated = directory / 'fixture.c'
    generated.write_text(fixture)
    compiler = shlex.split(os.environ.get('CC', 'cc'))
    for enabled in (False, True):
        executable = directory / ('health-on' if enabled else 'health-off')
        subprocess.run(compiler + [
            '-std=c11', '-g', '-O1', '-Wall', '-Wextra', '-Werror',
            '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
            '-fno-pie', '-no-pie',
        ] + (['-DCONFIG_TDMA_DIAGNOSTICS=1'] if enabled else []) + [
            str(generated), '-o', str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True, timeout=20)
