#!/usr/bin/env python3
"""Exercise receiver admission batching and compact layout publication.

NVS and hardware leaves are modeled. The real PING validation, membership,
deadline and publication bodies are exercised without RF or HID timing.
"""
import argparse
import importlib.util
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

parser = argparse.ArgumentParser()
parser.add_argument('--source', type=Path, default=ROOT / 'src/connection/esb.c')
parser.add_argument('--case', choices=['owner-store-lock-delay'])
args = parser.parse_args()
source = args.source.read_text()
header = (ROOT / 'src/connection/esb.h').read_text()
spec = importlib.util.spec_from_file_location('tdma_loss_extract', HERE.parent / 'tdma_loss/run.py')
extract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(extract)


def declaration(name):
    pattern = (rf'^static[ \t]+(?:volatile[ \t]+)?(?:const[ \t]+)?'
               rf'(?:[A-Za-z_]\w*[ \t]+)+{re.escape(name)}\b[^;]*;')
    matches = list(re.finditer(pattern, source, re.MULTILINE))
    if len(matches) != 1:
        raise ValueError(f'Expected one production declaration for {name}, found {len(matches)}')
    return matches[0].group()


def function(name):
    return extract_block(
        source,
        rf'^(?:static\s+)?(?:inline\s+)?(?:bool|int|void|uint8_t|uint32_t)\s+'
        rf'{re.escape(name)}\([^;{{]*\)\s*\{{',
    )


constants = '\n'.join(re.findall(
    r'^#define (?:PING_TIMEOUT_MS|PING_CLEAN_REACQUIRE_STREAK|TDMA_SHADOW_\w+|TDMA_MEMBERSHIP_COHORT_MS|'
    r'TDMA_(?:BASE_SLOT_TICKS|SLOT_TICKS_MAX_LEVEL0|SLOT_ROTATION|'
    r'RECONFIG_MIN_MS|LOSS_RECONFIG_MIN_MS|CAP_LEVEL_MAX))\b[^\n]*', source, re.MULTILINE))
constants += '\n' + '\n'.join(re.findall(
    r'^#define ESB_(?:PING|PONG)_\w+\b[^\n]*', header, re.MULTILINE))
variables = '''tdma_cap_ladder tdma_config_packed tdma_config_epoch
    tdma_dynamic_active_count tdma_dynamic_slot_ticks tdma_active_mask
    tdma_last_reconfig_time tdma_last_reconfig_valid tdma_cap_level tdma_published_cap_level
    tdma_loss_trigger_streak_ticks tdma_loss_recover_streak_ticks
    tdma_loss_probe_streak_ticks tdma_loss_pool_received tdma_loss_pool_gaps
    tdma_loss_pool_ticks test_all_enabled tracker_store_ready
    tdma_shadow_last_seen_ms tdma_shadow_ever_ping_mask tdma_shadow_ever_data_mask
    tdma_shadow_live_evidence_mask tdma_shadow_desired_mask
    tdma_shadow_pending_join_mask tdma_shadow_pending_leave_mask tdma_shadow_desired_changed_at_ms
    tdma_membership_deadline_ms tdma_membership_deadline_armed'''.split()
declarations = [constants, extract_block(source, r'^enum tdma_shadow_evidence\s*\{', True)]
declarations.extend(declaration(name) for name in variables)
declarations.append(extract.diagnostic_loss_declarations(source))
functions = [function(name) for name in (
    'tdma_loss_reset_windows', 'tdma_pack_config', 'tdma_shadow_observe',
    'tdma_shadow_forget_tracker', 'tdma_shadow_merge_locked', 'tdma_shadow_update',
    'tdma_membership_wait_ms', 'tdma_slot_ticks_for', 'tdma_recalculate',
    'tracker_store_restore',
)]
functions.append('static uint32_t sys_get_be32(const uint8_t *p) {\n'
                 'return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | '
                 '((uint32_t)p[2] << 8) | p[3];\n}\n')
functions.append(extract.diagnostic_function(source, 'account_ping_clock'))
functions.append(function('esb_receive_ping'))
functions.append('static bool host_validated_ping(void) {\n'
                 'if (rx_payload.length != ESB_PING_LEN || rx_payload.data[0] != ESB_PING_TYPE) return false;\n'
                 'uint8_t id = rx_payload.data[1];\n'
                 'uint32_t errors = id < MAX_TRACKERS ? crc_error_count[id] : 0;\n'
                 'esb_receive_ping(&rx_payload, 0);\n'
                 'return id < MAX_TRACKERS && crc_error_count[id] == errors;\n}')
owner = function('esb_stats_thread')
early_wake = extract_block(owner, r'^[ \t]*if \(\(int64_t\)now < next_monitor_time\)\s*\{')
functions.append('static void host_early_owner_wake(uint64_t now, int64_t next_monitor_time) {\n'
                 'do {\n' + early_wake + '\n} while (0);\n}')
functions.append('static void host_forget_id(uint8_t id) {\n'
                 'unsigned int key = irq_lock();\n'
                 'tdma_shadow_forget_tracker(id); irq_unlock(key);\n}')
fixture = (HERE / 'fixture.c').read_text()
fixture = fixture.replace('/* DIAGNOSTIC DECLARATIONS */', extract.diagnostic_declarations(source))
fixture = fixture.replace('/* PRODUCTION DECLARATIONS */', '\n'.join(declarations))
fixture = fixture.replace('/* PRODUCTION FUNCTIONS */', '\n\n'.join(functions))
with tempfile.TemporaryDirectory(prefix='receiver-tdma-membership-') as directory:
    directory = Path(directory)
    generated = directory / 'fixture.c'
    generated.write_text(fixture)
    executable = directory / 'fixture'
    compiler = shlex.split(os.environ.get('CC', 'cc'))
    for diagnostics in (False, True):
        flags = ['-DCONFIG_TDMA_DIAGNOSTICS=1'] if diagnostics else []
        subprocess.run(compiler + [
            '-std=c11', '-g', '-O1', '-Wall', '-Wextra',
            '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
            '-fno-pie', '-no-pie', *flags, str(generated), '-o', str(executable),
        ], check=True)
        subprocess.run([str(executable)] + ([args.case] if args.case else []), check=True, timeout=20)
