#!/usr/bin/env python3
"""Exercise production sequence, membership and recovery bodies."""

import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


CASES = (
    'fast-recovery',
    'slow-probe',
    'sparse-recovery',
    'idle-breaks-evidence',
    'loss-blocks-probe',
    'threshold-boundaries',
    'counter-reset',
    'counter-wrap',
    'restart-breaks-evidence',
    'ping-restart-sequence',
    'sequence-wrap',
    'pending-publication',
    'excluded-modes',
    'auto-snapshot',
    'sequence-lifetime',
    'blackout-reanchor',
    'ingress-membership',
    'blackout-loss-evidence',
    'decision-common-boundary',
    'decision-short-boundary',
    'decision-layout-boundary',
    'decision-pending-membership',
    'decision-genuine-loss',
)


def block(text, start):
    # Keep offsets intact while ignoring braces in comments and literals.
    non_code = re.compile(
        r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
        re.DOTALL,
    )
    code = non_code.sub(lambda match: re.sub(r'[^\n]', ' ', match.group()), text)
    begin = code.index('{', start)
    depth = 0
    for token in re.finditer(r'[{}]', code[begin:]):
        depth += 1 if token.group() == '{' else -1
        if depth == 0:
            return text[start:begin + token.end()] + '\n'
    raise ValueError('Unclosed production block')


def function(source, name, optional=False):
    pattern = r'^[\w \t*]+\b' + re.escape(name) + r'\s*\([^;{}]*\)\s*\{'
    match = re.search(pattern, source, re.MULTILINE)
    if match is not None:
        return block(source, match.start())
    if optional:
        return ''
    raise ValueError(f'Missing production function: {name}')

def declaration(source, name):
    pattern = (r'^static[ \t]+(?:volatile[ \t]+)?(?:const[ \t]+)?'
               r'(?:[A-Za-z_]\w*[ \t]+)+' + re.escape(name) + r'\b[^;]*;')
    matches = list(re.finditer(pattern, source, re.MULTILINE))
    if len(matches) != 1:
        raise ValueError(f'Expected one production declaration for {name}, found {len(matches)}')
    return matches[0].group()


def membership_declarations(source):
    start = source.index('#define PING_TIMEOUT_MS')
    end = source.index('static void tdma_shadow_observe', start)
    return '\n'.join((
        declaration(source, 'tdma_last_reconfig_time'),
        declaration(source, 'tdma_last_reconfig_valid'),
        source[start:end],
    ))


def membership_bodies(source):
    return '\n'.join(function(source, name) for name in (
        'tdma_shadow_observe', 'tdma_shadow_forget_tracker',
        'tdma_shadow_merge_locked', 'tdma_shadow_update',
        'tdma_membership_wait_ms',
    ))



def production_bodies(source, header=''):
    # Extract each selected source tree's constants and loss-controller state.
    constants_start = source.index('#define TDMA_RECONFIG_MIN_MS')
    constants_end = source.index('#define TDMA_SYNC_EXTRAP_MAX_TICKS')
    constants = source[constants_start:constants_end]
    declarations = '\n'.join(re.findall(
        r'^static (?:uint8_t|uint32_t) '
        r'tdma_(?:cap_level|published_cap_level|loss_\w+|prev_\w+)[^;]*;',
        source,
        re.MULTILINE,
    ))
    declarations += '\n' + '\n'.join(re.findall(
        r'^static (?:uint32_t|bool) (?:decision_\w+|tdma_decision_ready)[^;]*;',
        source, re.MULTILINE,
    ))
    structure_start = re.search(r'\bstruct\s+packet_stats\s*\{', source)
    if structure_start is None:
        raise ValueError('Missing production packet_stats structure')
    structure = block(source, structure_start.start()) + ';\n'

    ping_constants = '\n'.join(re.findall(
        r'^#define ESB_(?:PING|PONG)_\w+\b[^\n]*', header, re.MULTILINE))
    ping_constants += '\n' + re.search(
        r'^#define PING_CLEAN_REACQUIRE_STREAK\b[^\n]*', source, re.MULTILINE).group()
    snapshot = ''
    if header and 'struct esb_auto_loss_snapshot' in header:
        snapshot = '#define ESB_AUTO_LOSS_TRACKERS 16\n'
        snapshot += block(header, header.index('struct esb_auto_loss_snapshot')) + ';\n'
        snapshot += function(header, 'esb_rf_channel_decode')
        snapshot += function(source, 'esb_auto_loss_fence', optional=True)
        snapshot += function(source, 'decision_accept', optional=True)
        snapshot += function(source, 'esb_auto_loss_invalidate')
        snapshot += function(source, 'esb_get_auto_loss_snapshot')
        snapshot += function(source, 'esb_rx_discontinuity')
        snapshot += function(source, 'esb_auto_loss_rebase', optional=True)
        snapshot += function(source, 'esb_reset_all_stats')

    start = source.index('uint8_t received_sequence = rx_payload.data[16];')
    end = source.index('if (seq_result == 3', start)
    admission = ('\nstatic bool normal_admit(uint8_t tracker_id, struct esb_payload rx_payload) {\n'
                 'do {\n' + source[start:end] + 'return true;\n} while (0);\nreturn false;\n}\n')
    admission += '''
static bool composite_admit(uint8_t tracker_id, struct esb_payload payload) {
    unsigned before = fixture_pose_count;
    payload.data[1] = tracker_id;
    esb_receive_composite(&payload, 0);
    return fixture_pose_count != before;
}
static void restart_ping(uint8_t tracker_id) {
    struct esb_payload payload = {.length = ESB_PING_LEN, .pipe = 0};
    payload.data[0] = ESB_PING_TYPE;
    payload.data[1] = tracker_id;
    payload.data[ESB_PING_LEN - 1] = crc8_ccitt(0x07, payload.data, ESB_PING_LEN - 1);
    ping_counter_initialized[tracker_id] = true;
    last_ping_counter[tracker_id] = 100;
    last_ping_time[tracker_id] = (uint64_t)now;
    last_pong_queued_counter[tracker_id] = 0xff;
    esb_receive_ping(&payload, 0);
}
'''
    return '\n'.join((
        constants,
        ping_constants,
        membership_declarations(source),
        declarations,
        structure,
        'static struct packet_stats tracker_stats[MAX_TRACKERS];',
        # The pre-recovery-fix baseline has no shared reset helper.
        function(source, 'tdma_loss_reset_windows', optional=True),
        snapshot,
        function(source, 'tdma_loss_controller_update'),
        function(source, 'tdma_loss_controller_tick'),
        function(source, 'check_packet_sequence'),
        function(source, 'composite_body_length'),
        function(source, 'tdma_pack_config'),
        function(source, 'tdma_slot_ticks_for'),
        membership_bodies(source),
        function(source, 'account_ping_clock'),
        function(source, 'esb_receive_ping'),
        function(source, 'esb_receive_composite'),
        function(source, 'tdma_recalculate'),
        admission,
    ))


def parse_args():
    default_root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--source-root',
        type=Path,
        default=Path(os.environ.get('SOURCE_ROOT', default_root)),
    )
    parser.add_argument('--case', action='append')
    return parser.parse_args()


def main():
    args = parse_args()
    source = (args.source_root / 'src/connection/esb.c').read_text()
    header = (args.source_root / 'src/connection/esb.h').read_text()
    fixture = Path(__file__).with_name('fixture.c').read_text()
    prelude, scenarios = fixture.split('/* PRODUCTION BODIES */')

    with tempfile.TemporaryDirectory(prefix='receiver-tdma-loss-') as directory:
        path = Path(directory)
        generated = path / 'test.c'
        generated.write_text(prelude + production_bodies(source, header) + scenarios)
        binary = path / 'test'
        compiler = shlex.split(os.environ.get('CC', 'cc'))
        flags = [
            '-std=c11',
            '-Wall',
            '-Wextra',
            '-Werror',
            '-Wno-unused-function',
            '-Wno-unused-variable',
            '-Wno-unused-parameter',
        ]
        subprocess.run(compiler + flags + ['-I', str(args.source_root / 'src'), str(generated), '-o', str(binary)], check=True)
        for case in args.case or CASES:
            subprocess.run([str(binary), case], check=True)
            print(f'PASS {case}', flush=True)


if __name__ == '__main__':
    main()
