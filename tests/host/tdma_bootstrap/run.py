#!/usr/bin/env python3
"""Exercise the actual receiver's first-PING coalescing and compact layout.

NVS and hardware leaves are modeled. The extracted real PING validation, shadow,
one-shot deadline and layout bodies are exercised; no RF or HID latency is measured.
"""
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
    r'^#define (?:PING_TIMEOUT_MS|TDMA_SHADOW_\w+|TDMA_INITIAL_COHORT_MS|'
    r'TDMA_(?:BASE_SLOT_TICKS|SLOT_TICKS_MAX_LEVEL0|SLOT_ROTATION|'
    r'RECONFIG_MIN_MS|LOSS_RECONFIG_MIN_MS|CAP_LEVEL_MAX))\b[^\n]*', source, re.MULTILINE))
constants += '\n' + '\n'.join(re.findall(
    r'^#define ESB_PING_(?:LEN|TYPE|COMMAND_MASK)\b[^\n]*', header, re.MULTILINE))
variables = '''tdma_cap_ladder tdma_config_packed tdma_config_epoch
    tdma_dynamic_active_count tdma_dynamic_slot_ticks tdma_active_mask
    tdma_last_reconfig_time tdma_cap_level tdma_published_cap_level
    tdma_loss_trigger_streak_ticks tdma_loss_recover_streak_ticks
    tdma_loss_probe_streak_ticks tdma_loss_pool_received tdma_loss_pool_gaps
    tdma_loss_pool_ticks test_all_enabled tracker_store_ready
    tdma_shadow_last_seen_ms tdma_shadow_ever_ping_mask tdma_shadow_ever_data_mask
    tdma_shadow_live_evidence_mask tdma_shadow_join_since_ms tdma_shadow_desired_mask
    tdma_shadow_pending_join_mask tdma_shadow_pending_leave_mask tdma_shadow_desired_changed_at_ms
    tdma_initial_allowed_mask tdma_initial_ping_mask tdma_initial_ping_at_ms
    tdma_initial_deadline_ms tdma_initial_started tdma_initial_done'''.split()
declarations = [constants, extract_block(source, r'^enum tdma_shadow_evidence\s*\{', True)]
declarations.extend(declaration(name) for name in variables)
declarations.append('static void tdma_recalculate(void);')
functions = [function(name) for name in (
    'tdma_loss_reset_windows', 'tdma_pack_config', 'tdma_shadow_observe',
    'tdma_shadow_forget_tracker', 'tdma_shadow_update', 'tdma_initial_observe',
    'tdma_initial_wait_ms', 'tdma_initial_due', 'tdma_initial_process',
    'tdma_slot_ticks_for', 'tdma_recalculate', 'tracker_store_restore',
)]
# Extract the real event-path ID/CRC checks and its shadow/coalescing calls.
marker = source.index('// Check for PING control packet and respond with PONG')
begin = source.index('\t\t\t\t\tuint8_t tracker_id = rx_payload.data[1];', marker)
end = source.index('\n\t\t\t\t\tuint32_t tracker_estimated_server_ticks', begin)
functions.append('static bool host_validated_ping(void) {\n'
                 'if (rx_payload.length != ESB_PING_LEN || rx_payload.data[0] != ESB_PING_TYPE) return false;\n'
                 'bool accepted = false; do {\n' + source[begin:end]
                 + '\naccepted = true;\n} while (0); return accepted;\n}')
slot_reset = function('tracker_slot_reset')
begin = slot_reset.index('\t/* A removed/reused ID')
end = slot_reset.index('\n\ttdma_shadow_forget_tracker(id);', begin)
functions.append('static void host_forget_initial_id(uint8_t id) {\n'
                 'unsigned int key = irq_lock();\n' + slot_reset[begin:end]
                 + '\ntdma_shadow_forget_tracker(id); irq_unlock(key);\n}')
fixture = (HERE / 'fixture.c').read_text()
fixture = fixture.replace('/* PRODUCTION DECLARATIONS */', '\n'.join(declarations))
fixture = fixture.replace('/* PRODUCTION FUNCTIONS */', '\n\n'.join(functions))
with tempfile.TemporaryDirectory(prefix='receiver-tdma-initial-') as directory:
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
    subprocess.run([str(executable)], check=True, timeout=20)
