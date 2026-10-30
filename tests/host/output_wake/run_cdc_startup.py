#!/usr/bin/env python3
"""Exercise USB startup, HID collection commands, CDC output, and console lifecycle.

Use --source-root to run the same fixture against another receiver source tree.
"""
import argparse
import importlib.util
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument('--source-root', type=Path, default=HERE.parents[2])
parser.add_argument('--case', choices=(
    'raw-only', 'single-console', 'batch-console', 'raw-reopen', 'early-vbus',
    'device-not-ready', 'callback-failure', 'disabled', 'dedicated-hid',
))
args = parser.parse_args()
src = args.source_root.resolve() / 'src'

spec = importlib.util.spec_from_file_location('c_extract', HERE.parent / 'harness/c_extract.py')
extract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(extract)


def function(source, name):
    return extract.extract_block(
        source, r'^[\w \t*]+\b' + re.escape(name) + r'\s*\([^;{}]*\)\s*\{') + '\n'


usb = (src / 'usb.c').read_text()
console = (src / 'console.c').read_text()
collect = (src / 'data_collect.c').read_text()
command = (src / 'rcv_cmd.c').read_text()
hid = (src / 'hid.c').read_text()
esb = (src / 'connection/esb.c').read_text()
esb_header = (src / 'connection/esb.h').read_text()

flags = '\n'.join(re.findall(r'^#define ESB_(?:PONG_FLAG|RAW_)\w+[^\n]*', esb_header, re.M))
protocol = '#define SLIMENRF_ESB\n' + flags + '\n#include "rcv_cmd.h"\n'
console_declarations = console[console.index('#define CONSOLE_LINE_QUEUE_DEPTH'):console.index('#define DFU_EXISTS')]
console_functions = console[console.index('static bool console_echo_has_data_locked(void)'):
                            console.index('static void console_thread(void)\n{')]
raw_start = esb.index('/* Raw data collection packets (types 0x10-0x13)')
raw_end = esb.index('LOG_ERR("Wrong packet length:', raw_start)
raw_dispatch = esb[raw_start:raw_end]

production = '#if defined(CONFIG_DATA_COLLECT) && !defined(CONFIG_DATA_COLLECT_HID)\n'
production += collect[collect.index('LOG_MODULE_REGISTER'):]
production += '\n' + '\n'.join(function(command, name) for name in (
    'rcv_cmd_collect_start', 'rcv_cmd_collect_stop', 'rcv_cmd_collect_batch_start',
    'rcv_cmd_collect_batch_stop', 'fill_ack', 'rcv_cmd_process_hid'))
production += '\n' + hid[hid.index('static void handle_output_report('):hid.index('static void int_in_ready_cb(')]
production += '\n' + function(hid, 'set_report_cb')
production += '\n' + console_declarations + '\n' + console_functions
production += '''\nstatic void receive_raw(uint8_t tracker, uint16_t sequence)
{
    struct raw_payload rx_payload = {
        .data = {ESB_RAW_IMU_TYPE, tracker, sequence >> 8, sequence, 0x55, 0x66},
        .length = 6,
        .rssi = 0xd7,
    };
    uint8_t pkt_type = rx_payload.data[0];
    do {\n''' + raw_dispatch + '''\n    } while (0);
}
#endif
'''
production += '\n'.join(function(usb, name) for name in (
    'receiver_usb_set_configured', 'usbd_add_string_descriptors', 'usbd_setup',
    'usb_vbus_present', 'usb_enable_device', 'usb_close_serial', 'status_cb',
    'usb_init_thread'))

cases = {
    'cdc': ('raw-only', 'single-console', 'batch-console', 'raw-reopen',
            'early-vbus', 'device-not-ready', 'callback-failure'),
    'disabled': ('disabled',),
    'hid': ('dedicated-hid',),
}
with tempfile.TemporaryDirectory(prefix='receiver-cdc-startup-') as directory:
    temp = Path(directory)
    (temp / 'protocol.inc').write_text(protocol)
    (temp / 'production.inc').write_text(production)
    for variant, variant_cases in cases.items():
        selected = [case for case in variant_cases if args.case is None or args.case == case]
        if not selected:
            continue
        defines = {
            'cdc': ['-DCONFIG_DATA_COLLECT'],
            'disabled': [],
            'hid': ['-DCONFIG_DATA_COLLECT', '-DCONFIG_DATA_COLLECT_HID'],
        }[variant]
        binary = temp / variant
        subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
            '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
            '-Wno-unused-function', '-Wno-unused-variable', '-Wno-unused-but-set-variable',
            '-g', '-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
            '-fno-pie', '-no-pie', *defines, '-I', str(temp), '-I', str(src),
            str(HERE / 'cdc_startup.c'), '-o', str(binary),
        ], check=True)
        for case in selected:
            subprocess.run([str(binary), case], check=True)
