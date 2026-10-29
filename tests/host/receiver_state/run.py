#!/usr/bin/env python3
"""Run actual receiver state functions with controlled storage/IRQ leaves.

Store and peripheral leaves are modeled; tuple tests inject a possible interrupt
after each production shared-field store, without changing the store or order.
--source-root selects function bodies while declarations use the live contract.
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

parser = argparse.ArgumentParser()
parser.add_argument('--source-root', type=Path, default=ROOT)
parser.add_argument('--case', action='append', choices=['slots', 'shadow', 'tuples'])
args = parser.parse_args()
cases = args.case or ['slots', 'shadow', 'tuples']
source = (args.source_root / 'src/connection/esb.c').read_text()
live = (ROOT / 'src/connection/esb.c').read_text()

# Explicit inventory is independent of the reset implementation: removing a
# production reset store must leave a seeded field nonzero and fail the test.
slot_arrays = '''last_packet_sequence sequence_initialized last_ping_counter
ping_counter_initialized last_ping_time last_pong_queued_counter last_raw_seq
last_raw_valid tracker_remote_command pending_cmd_arg tracker_test_on_tps
pending_sens_data pending_sens_auto_axis pending_sens_auto_revolutions
metadata_active_mask metadata_active_chunk metadata_active_token
metadata_pending_mask metadata_pending_chunk metadata_pending_token
wom_token wom_minutes wom_error wom_complete
wom_advertised wom_first_ping_counter
pipe_mismatch_count crc_error_count
 test_all_ready_after_ms tdma_config_packed tdma_prev_recv tdma_prev_gaps
tdma_prev_restarts tracker_stats decision_received decision_gaps
tdma_shadow_last_seen_ms'''.split()
diagnostic_arrays = '''g_tdma_stats g_last_ping_rx_time_diff_ticks
g_last_ping_rx_time_diff_valid g_ping_isr_rx_ticks g_ping_isr_rx_ticks_valid
g_last_isr_rx_ticks g_last_isr_rx_valid g_bias_ppb_valid g_bias_ppb
g_bias_ref_offset g_bias_ref_ticks g_last_ping_isr_rx_ticks_raw
g_ping_dirty_streak g_ping_reject_count g_extrap_skip_count'''.split()
slot_masks = '''test_all_confirmed_mask channel_target_mask tdma_shadow_ever_ping_mask
tdma_shadow_ever_data_mask tdma_shadow_live_evidence_mask tdma_shadow_desired_mask
tdma_shadow_pending_join_mask tdma_shadow_pending_leave_mask'''.split()
scalars = '''tdma_shadow_desired_changed_at_ms tracker_store_ready esb_clearing
esb_pairing tdma_active_mask tdma_membership_deadline_ms tdma_membership_deadline_armed
test_all_state_valid test_all_enabled test_all_requested_tps test_all_effective_tps
test_all_generation'''.split()


def declaration(name):
    pattern = rf'^static[ \t]+(?:volatile[ \t]+)?(?:struct[ \t]+\w+|\w+)[ \t]+{name}\b[^;]*;'
    found = list(re.finditer(pattern, live, re.M))
    if len(found) != 1:
        raise ValueError(f'Expected one production declaration: {name}')
    return found[0].group()


def function(name):
    return extract_block(source, rf'^(?:static\s+)?(?:void|bool|int|uint32_t)\s+{name}\([^;{{]*\)\s*\{{')


constants = '\n'.join(re.findall(
    r'^#define (?:PING_TIMEOUT_MS|REMOTE_COMMAND_ACTIVE_SCAN_MS|TDMA_SHADOW_\w+|TDMA_MEMBERSHIP_COHORT_MS|RX_PHASE_BUCKETS|RX_RSSI_BUCKETS)\b[^\n]*', live, re.M))
protocol = (ROOT / 'src/connection/esb.h').read_text()
constants += '\n' + '\n'.join(re.findall(
    r'^#define ESB_(?:PONG_FLAG_\w+|PING_FLAG_WOM_(?:RESULT|ERROR))\b[^\n]*',
    protocol, re.M))
declarations = constants + '\n' + '\n'.join(
    extract_block(live, rf'^{kind} {name}\s*\{{', semicolon=True)
    for kind, name in [('struct', 'packet_stats'), ('enum', 'tdma_shadow_evidence')])
declarations += '\n' + '\n'.join(declaration(name) for name in slot_arrays + slot_masks + scalars)
declarations += '\n#if defined(CONFIG_TDMA_DIAGNOSTICS)\n'
declarations += extract_block(live, r'^struct tdma_stats\s*\{', semicolon=True)
declarations += '\n' + '\n'.join(declaration(name) for name in diagnostic_arrays)
declarations += '\n#endif\n'

names = ['tdma_shadow_observe', 'tdma_shadow_forget_tracker', 'tdma_shadow_merge_locked', 'tdma_shadow_update',
         'esb_send_remote_command_sens', 'esb_send_remote_command_sens_auto',
         'esb_send_remote_command_sens_auto_all', 'esb_send_remote_command_arg']
if 'slots' in cases:
    names += ['tracker_slot_reset', 'esb_add_pair', 'esb_pop_pair', 'esb_clear']
functions = []
for name in names:
    body = function(name)
    if name.startswith('esb_send_remote_command_'):
        body = re.sub(
            r'(^[ \t]*(?:pending_sens_data|pending_sens_auto_axis|pending_sens_auto_revolutions|pending_cmd_arg|tracker_remote_command)\[[^\n;]*=[^\n;]*;)',
            r'\1 preempt();', body, flags=re.M)
    functions.append(body)

seed = ['static void seed_slot(unsigned id) {']
check = ['static void check_slot(unsigned id) {',
         'uint8_t zero[sizeof(struct packet_stats) + 32] = {0};']
for name in slot_arrays + diagnostic_arrays:
    if name in diagnostic_arrays:
        seed.append('#if defined(CONFIG_TDMA_DIAGNOSTICS)')
        check.append('#if defined(CONFIG_TDMA_DIAGNOSTICS)')
    seed.append(f'memset((void *)&{name}[id], 1, sizeof({name}[id]));')
    check.append(f'assert(memcmp((const void *)&{name}[id], zero, sizeof({name}[id])) == 0);'
                 if name != 'g_tdma_stats' else
                 'for (size_t i = 0; i < sizeof(g_tdma_stats[id]); ++i) '
                 'assert(((const uint8_t *)&g_tdma_stats[id])[i] == 0);')
    if name in diagnostic_arrays:
        seed.append('#endif')
        check.append('#endif')
for name in slot_masks:
    seed.append(f'{name} |= BIT(id);')
    check.append(f'assert(!({name} & BIT(id)));')
seed += ['rssi[id] = ota[id] = collection[id] = 1;', '}']
check += ['assert(rssi[id] == 0);', '}']
fixture = (HERE / 'fixture.c').read_text()
fixture = fixture.replace('/* PRODUCTION_DECLARATIONS */', declarations)
fixture = fixture.replace('/* PRODUCTION_FUNCTIONS */', '\n'.join(functions))
fixture = fixture.replace('/* GENERATED_SLOT_CHECKS */', '\n'.join(seed + check))
with tempfile.TemporaryDirectory(prefix='receiver-state-') as directory:
    temporary = Path(directory)
    log = temporary / 'zephyr/logging/log.h'
    log.parent.mkdir(parents=True)
    log.write_text('/* Logging shim; fixture supplies the only used macros. */\n')
    probe = temporary / 'probe.c'
    probe.write_text(fixture)
    executable = temporary / 'probe'
    flags = [] if 'slots' in cases else ['-DSTATE_ONLY_LEGACY']
    for diagnostics in (False, True):
        diagnostic_flags = ['-DCONFIG_TDMA_DIAGNOSTICS=1'] if diagnostics else []
        subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
            '-std=gnu11', '-g', '-O1', '-fsanitize=address,undefined',
            '-fno-omit-frame-pointer', '-fno-pie', '-no-pie',
            '-I', str(temporary), '-I', str(ROOT / 'src'), *flags, *diagnostic_flags,
            str(probe), str(ROOT / 'src/connection/tracker_roster.c'),
            '-o', str(executable),
        ], check=True)
        for case in cases:
            subprocess.run([str(executable), case], check=True, timeout=20)
