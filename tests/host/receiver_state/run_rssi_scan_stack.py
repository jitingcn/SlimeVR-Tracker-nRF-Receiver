#!/usr/bin/env python3
"""Replay the linked RSSI work queue, scanner, restoration and printk packaging.

    uv run --with unicorn --with pyelftools run_rssi_scan_stack.py PATH/zephyr.elf

FEM receivers also read chip identity; pass --soc nrf52833 for nRF52833 boards
(default: nrf52840). The part is not recoverable from the ELF.

Two opt-in FEM cases exist. --case fem_timer_stall suppresses FEM settling TIMER
compares while RTC time continues; it audits the real pre-CC2 timeout/abort
path. --case fem_retained_events checks that the event configuration handed to
mpsl_fem_lna_configuration_set() outlives prepare(): if the descriptors are
inside the worker stack frame they are treated as reused after return and, with
a deterministic expiring-frame pattern, cleanup must still observe the
registered bytes. Neither case is an explanation or reproduction of a field
reboot by itself.

Uses an unstripped nRF52 receiver ELF with DWARF, including FEM builds. The real
rcv_cmd_rssi_scan queues work; the real work_queue_main dispatches and retires
it. The linked scanner, polling, FEM/MPSL ownership/cleanup, SDK ESB
initialization, address setup, start_rx and printk/cbvprintf_package are real.

Hardware leaves: immediate RADIO task/event/state completion, synthetic RSSI,
one-shot TIMER compares, EGU/PPI routing, GPIO register storage, HF-clock
completion, RTC reads and sleeps. Kernel leaves: thread creation/wakeup/naming,
scheduler switching, spinlock ownership diagnostics and TLS. Real deferred-log
allocation/publication run against an empty, DWARF-initialized linked ring; an
independent logger consumer drains committed records at modeled sleeps.
CPU interrupts and physical FEM pins are absent. This is not a radio/USB,
RF-timing, log-pressure, watchdog or hardware proof.

With deferred logging, printk packages on the worker and formats on the logger:
real packaging stays on the measured worker stack. Captured printk arguments
are additionally rendered by the linked cbvprintf on a separate scratch stack;
Python never implements printf. Logger execution is not charged to the worker.

Stack depth includes the actual work_queue_main wrapper and is compared against
the actual rssi_wq_stack object and the size passed by real rssi_wq_init to
k_thread_create. Cross-checks the architecture reservation against a linked
static thread, then separately requires a basic Cortex-M exception frame. Depth
is a LOWER BOUND: thread startup/TLS placement, interrupts, preemption and modeled
leaves are excluded. An over-budget execution is a failure, not hardware proof
that every execution with headroom is safe.
"""

import argparse
from collections import Counter
import json
from pathlib import Path
import re
import struct
import sys

from elftools.elf.elffile import ELFFile
from unicorn import (
    Uc, UcError, UC_ARCH_ARM, UC_HOOK_CODE, UC_HOOK_MEM_WRITE,
    UC_MODE_MCLASS, UC_MODE_THUMB,
)
from unicorn.arm_const import (
    UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1,
    UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP,
)

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent / 'tdma_loss'))
from run_stack import load_image, SCRATCH_BASE, SCRATCH_SIZE  # noqa: E402

STOP = SCRATCH_BASE + 0x100
OUTPUT = SCRATCH_BASE + 0x180
TLS = SCRATCH_BASE + 0x200
ARGS = SCRATCH_BASE + 0x300
INIT_TOP = SCRATCH_BASE + SCRATCH_SIZE
INSTRUCTION_LIMIT = 150_000_000
EXCEPTION_RESERVE = 32
HOME = 18
SWEEPS = 200
SAMPLES = 50
# Independent contract, not extracted from scan implementation under test.
CHANNELS = tuple(ch for ch in range(0, 89, 2) if ch not in (4, 22, 26, 80))
CASES = ('success_repeat', 'one_invalid', 'all_invalid', 'restore_failure', 'disable_failure')
OPTIONAL_CASES = ('fem_timer_stall', 'fem_retained_events')
# FICR INFO.PART plus the nrf52_erratas.h family/revision bytes used by the
# proprietary FEM library. Explicit, because an ELF does not carry the part.
SOCS = {
    'nrf52840': {'name': 'nRF52840 revision 2', 'part': 0x52840, 'variant': 8, 'revision': 2},
    'nrf52833': {'name': 'nRF52833 revision 1', 'part': 0x52833, 'variant': 0x0D, 'revision': 1},
}
RADIO = 0x40001000
PPI = 0x4001F000
GPIOTE = 0x40006000
GPIO_PORTS = (0x50000000, 0x50000300)
TIMERS = (0x40008000, 0x40009000, 0x4000A000, 0x4001A000, 0x4001B000)
EGUS = tuple(0x40014000 + 0x1000 * i for i in range(6))
READY, DISABLED, RSSIEND = 0x100, 0x110, 0x11C
SHORTS, PACKETPTR, FREQUENCY, RSSISAMPLE, STATE = 0x200, 0x504, 0x508, 0x548, 0x550
# Nonvolatile home receive configuration, including DMA target and addresses.
HOME_REGISTERS = (SHORTS, PACKETPTR, FREQUENCY, 0x50c, 0x510, 0x514, 0x518,
                  0x51c, 0x520, 0x524, 0x528, 0x52c, 0x530, 0x534, 0x538,
                  0x53c, 0x544, 0x650)


def layouts(path, names):
    """Read this ELF's member offsets/sizes, including named typedef structs."""
    result = {}
    with path.open('rb') as stream:
        for unit in ELFFile(stream).get_dwarf_info().iter_CUs():
            for die in unit.iter_DIEs():
                attr = die.attributes.get('DW_AT_name')
                if attr is None or attr.value.decode(errors='replace') not in names:
                    continue
                name = attr.value.decode()
                target = die
                while 'DW_AT_byte_size' not in target.attributes and 'DW_AT_type' in target.attributes:
                    target = target.get_DIE_from_attribute('DW_AT_type')
                size = target.attributes.get('DW_AT_byte_size')
                if size is None:
                    continue
                members = {}
                for child in target.iter_children():
                    key = child.attributes.get('DW_AT_name')
                    offset = child.attributes.get('DW_AT_data_member_location')
                    if key and offset and isinstance(offset.value, int):
                        members[key.value.decode()] = offset.value
                if members:
                    result[name] = (members, size.value)
            if names <= result.keys():
                return result
    raise ValueError(f'Missing DWARF layouts: {sorted(names - result.keys())}')


def status_code(name):
    text = (ROOT / 'src/rcv_hid_cmd.h').read_text()
    match = re.search(rf'^#define\s+{name}\s+(0x[0-9a-fA-F]+|\d+)\s*$', text, re.M)
    if not match:
        raise ValueError(f'Missing HID status {name}')
    return int(match.group(1), 0)


def score(channel):
    return 60 + (channel % 7) * 5


def replay(path, image, abi, case, soc):
    symbols, functions, sections, offsets = image

    def symbol(name):
        matches = {value for key, value in symbols.items()
                   if value[1] and (key == name or key.startswith(name + '.'))}
        if len(matches) != 1:
            raise ValueError(f'Ambiguous/missing ELF symbol {name}: {matches}')
        return matches.pop()

    machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    for start in (0, 0x10000000, 0x20000000, 0x40000000, 0x50000000, 0xE0000000, SCRATCH_BASE):
        machine.mem_map(start, 0x100000)
    for address, data in sections:
        machine.mem_write(address, data)

    def word(address):
        return struct.unpack('<I', machine.mem_read(address, 4))[0]

    def put_word(address, value):
        machine.mem_write(address, struct.pack('<I', value & 0xffffffff))

    def put(name, value, size=4):
        machine.mem_write(symbol(name)[0], value.to_bytes(size, 'little'))

    stack = symbol('rssi_wq_stack')
    top = stack[0] + stack[1]
    queue = symbol('rssi_wq')[0]
    work = symbol('rssi_scan_work')[0]
    qm, _ = abi['k_work_q']
    wm, _ = abi['k_work']
    sl, _ = abi['_slist']
    # The MPSL FEM library may be linked even when the application compiles the
    # non-FEM stubs; the scanner's own FEM translation unit is the real signal.
    scan_fem_enabled = any(key.startswith('rssi_scan_fem_prepare') and value[1]
                           for key, value in symbols.items())
    fem_event_size = abi['mpsl_fem_event_t'][1] if 'mpsl_fem_event_t' in abi else 0
    # MPSL keeps the event pointers it was given until the matching clear.
    # Track whether those objects legally outlive rssi_scan_fem_prepare().
    retained = {'prepare_return_address': 0, 'prepare_returned': False,
                'activate': 0, 'deactivate': 0,
                'activate_bytes': b'', 'deactivate_bytes': b'',
                'outside_worker_stack': None, 'reused_frame_poisoned': False,
                'clear_checked': False, 'contents_unchanged_at_clear': False}
    lm, log_size = abi['mpsc_pbuf_buffer']
    log_buffer = symbol('log_buffer')[0]
    log_storage, log_bytes = symbol('buf32')
    machine.mem_write(log_buffer, bytes(log_size))
    for field, value in (
            ('notify_drop', symbol('z_log_notify_drop')[0]),
            ('get_wlen', symbol('log_msg_generic_get_wlen')[0]),
            ('buf', log_storage), ('size', log_bytes // 4),
            ('flags', 1 if (log_bytes // 4) & (log_bytes // 4 - 1) == 0 else 0)):
        put_word(log_buffer + lm[field], value)
    put_word(symbol('curr_log_buffer')[0], log_buffer)
    put_word(symbol('timestamp_func')[0], symbol('default_get_timestamp')[0])
    static_stack = symbol('_k_thread_stack_esb_stats_thread_id')
    static_data = symbol('_k_thread_data_esb_stats_thread_id')[0]
    reserved = static_stack[1] - word(static_data + offsets['init_stack_size'])
    if not 0 < reserved < static_stack[1]:
        raise ValueError('Invalid architecture reservation from linked static thread')
    budget = stack[1] - reserved
    dispatcher = None
    phase = 'init'
    lowest = top
    deepest = []
    recent = []
    instructions = 0
    virtual_us = 0
    reached = Counter()
    modeled = Counter()
    print_calls = []
    rendered = bytearray()
    visits = Counter()
    sample_attempts = 0
    samples = 0
    visit_sample = 0
    scans = []
    completed = False
    injecting = False
    restores = 0
    release_states = []
    failures = []
    home_radio = {}

    # Stable physical identity for real esb_set_addr_unified and nRF variant tests.
    machine.mem_write(0x100000A4, bytes.fromhex('123456789abc0000'))
    put_word(0x10000100, soc['part'])
    # Nordic nrf52_erratas.h identifies the CHIP family in FICR[0x130] and the
    # revision in FICR[0x134]; seed a supported revision, not a zero identity.
    put_word(0x10000130, soc['variant'])
    put_word(0x10000134, soc['revision'])
    put('receiver_rf_channel', HOME, 1)
    put_word(TLS, queue + qm['thread'])

    def drain_logs():
        # Model the separately scheduled logger consumer, never worker-side
        # printf/allocation/publication. No record is still under construction
        # at the sleep boundary. The output oracle uses the real formatter.
        committed = word(log_buffer + lm['wr_idx'])
        check(committed == word(log_buffer + lm['tmp_wr_idx']),
              'Logger consumer encountered an uncommitted allocation')
        put_word(log_buffer + lm['rd_idx'], committed)
        put_word(log_buffer + lm['tmp_rd_idx'], committed)
        put_word(log_buffer + lm['flags'], word(log_buffer + lm['flags']) & ~8)
        put('buffered_cnt', 0)

    def ret(value=0):
        machine.reg_write(UC_ARM_REG_R0, value & 0xffffffff)
        machine.reg_write(UC_ARM_REG_R1, 0)
        machine.reg_write(UC_ARM_REG_PC, machine.reg_read(UC_ARM_REG_LR))

    def check(condition, message):
        if not condition:
            raise RuntimeError(message)

    def gpio_output(config, high):
        port = GPIO_PORTS[(config >> 13) & 1]
        pin = 1 << ((config >> 8) & 31)
        previous = word(port + 0x504)
        put_word(port + 0x504, previous | pin if high else previous & ~pin)

    def event(address, depth=0):
        check(depth < 16, 'Hardware-model PPI event recursion')
        put_word(address, 1)
        enabled = word(PPI + 0x500)
        for channel in range(20):
            if enabled & (1 << channel) and word(PPI + 0x510 + 8 * channel) == address:
                task(word(PPI + 0x514 + 8 * channel), depth + 1)
                fork = word(PPI + 0x910 + 4 * channel)
                if fork:
                    task(fork, depth + 1)

    def task(address, depth=0):
        nonlocal virtual_us, samples, sample_attempts, visit_sample
        if address == RADIO + 4:  # RXEN
            channel = word(RADIO + FREQUENCY)
            raw = word(RADIO + SHORTS) == 0x20  # scanner's END_START only
            if phase == 'worker' and raw:
                check(word(symbol('radio_operation_busy')[0]) == 1, 'Raw RX without radio owner')
                check(word(RADIO + STATE) == 0, 'Raw RX before disable')
                check(channel in CHANNELS, f'Scanned forbidden channel {channel}')
                dma = word(RADIO + PACKETPTR)
                check(not stack[0] <= dma < top, 'Raw RX DMA points into the worker stack')
                visits[channel] += 1
                visit_sample = 0
            put_word(RADIO + STATE, 2)  # RXIDLE
            event(RADIO + READY, depth)
            if word(RADIO + SHORTS) & 1:  # READY_START
                task(RADIO + 8, depth + 1)
        elif address == RADIO + 8:  # START
            put_word(RADIO + STATE, 3)
        elif address == RADIO + 0x14:  # RSSISTART
            channel = word(RADIO + FREQUENCY)
            check(word(RADIO + STATE) == 3, 'RSSI task issued outside active RX')
            sample_attempts += 1
            visit_sample += 1
            bad = injecting and (case in ('all_invalid', 'restore_failure') or
                                 (case == 'one_invalid' and channel == HOME and visits[channel] == 1))
            if not bad:
                value = score(channel) + (visit_sample - 1) % 5 + (visits[channel] - 1) % 3
                put_word(RADIO + RSSISAMPLE, value)
                event(RADIO + RSSIEND, depth)
                samples += 1
        elif address == RADIO + 0x10:  # DISABLE
            if injecting and case == 'disable_failure' and visits:
                return
            put_word(RADIO + STATE, 0)
            event(RADIO + DISABLED, depth)
        elif GPIOTE <= address < GPIOTE + 0x80:
            offset = address - GPIOTE
            bank, offset = divmod(offset, 0x30)
            if offset < 32:
                config = word(GPIOTE + 0x510 + offset)
                if config & 3 == 3:
                    polarity = (config >> 16) & 3
                    if bank == 1 or (bank == 0 and polarity == 1):
                        gpio_output(config, True)
                    elif bank == 2 or (bank == 0 and polarity == 2):
                        gpio_output(config, False)
                    elif bank == 0 and polarity == 3:
                        port = GPIO_PORTS[(config >> 13) & 1]
                        high = not (word(port + 0x504) & (1 << ((config >> 8) & 31)))
                        gpio_output(config, high)
        elif address & ~0xfff in EGUS and address & 0xfff < 0x40:
            event((address & ~0xfff) + 0x100 + (address & 0xfff), depth)
        elif address in TIMERS:  # START, sufficient for bounded FEM settling.
            if phase == 'worker' and injecting and restores == 0 and case == 'fem_timer_stall':
                return
            periods = [(word(address + 0x540 + 4 * i), i) for i in range(6)]
            elapsed = 0
            for period, channel in sorted(periods):
                if period:
                    virtual_us += period - elapsed
                    elapsed = period
                    event(address + 0x140 + 4 * channel, depth)
                    if word(address + SHORTS) & (1 << (8 + channel)):
                        break
        elif PPI <= address < PPI + 0x30:  # Channel-group EN/DIS tasks.
            group, disable = divmod(address - PPI, 8)
            mask = word(PPI + 0x800 + 4 * group)
            enabled = word(PPI + 0x500)
            put_word(PPI + 0x500, enabled & ~mask if disable else enabled | mask)
        # Other TIMER stop/clear tasks need no action.

    def mmio(cpu, access, address, size, value, unused):
        if size != 4:
            return
        if address == 0xE000ED0C and value & 4:  # AIRCR.SYSRESETREQ
            raise RuntimeError('Firmware requested a system reset through AIRCR')
        if address == PPI + 0x504:
            put_word(PPI + 0x500, word(PPI + 0x500) | value)
        elif address == PPI + 0x508:
            put_word(PPI + 0x500, word(PPI + 0x500) & ~value)
        elif GPIOTE + 0x510 <= address < GPIOTE + 0x530 and value & 3 == 3:
            gpio_output(value, bool(value & (1 << 20)))
        elif any(address == port + offset for port in GPIO_PORTS
                 for offset in (0x508, 0x50c, 0x518, 0x51c)):
            for port in GPIO_PORTS:
                for offset, target, clear in ((0x508, 0x504, False), (0x50c, 0x504, True),
                                              (0x518, 0x514, False), (0x51c, 0x514, True)):
                    if address == port + offset:
                        current = word(port + target)
                        put_word(port + target, current & ~value if clear else current | value)
        elif value == 1 and ((RADIO <= address < RADIO + 0x20) or
                             (address & ~0xfff in EGUS and address & 0xfff < 0x40) or
                             (GPIOTE <= address < GPIOTE + 0x80) or
                             address in TIMERS):
            task(address)

    def prepare_returned(cpu):
        retained['prepare_returned'] = True
        low, high = stack[0], stack[0] + stack[1]
        inside = any(low <= address < high for address in
                     (retained['activate'], retained['deactivate']))
        retained['outside_worker_stack'] = not inside
        if inside:
            # The prepare frame's lifetime ended at this return. A later call
            # or exception frame may legitimately reuse it before MPSL clears
            # the configuration that still points into it.
            pattern = bytes((0x5a + i) & 0xff for i in range(fem_event_size))
            machine.mem_write(retained['activate'], pattern)
            machine.mem_write(retained['deactivate'], pattern)
            retained['reused_frame_poisoned'] = True

    def hook(cpu, pc, size, unused):
        nonlocal lowest, deepest, instructions, virtual_us, dispatcher
        nonlocal completed, restores
        instructions += 1
        if (case == 'fem_retained_events' and phase == 'worker'
                and retained['prepare_return_address']
                and not retained['prepare_returned']
                and pc == retained['prepare_return_address']):
            prepare_returned(cpu)
        if phase == 'worker':
            sp = cpu.reg_read(UC_ARM_REG_SP)
            if sp < lowest:
                lowest, deepest = sp, recent[-12:]
        if pc == OUTPUT:
            rendered.append(cpu.reg_read(UC_ARM_REG_R0) & 0xff)
            ret(cpu.reg_read(UC_ARM_REG_R0))
            return
        name = functions.get(pc)
        if not name:
            return
        base = name.split('.')[0]
        recent.append(name)
        del recent[:-24]
        if phase == 'worker':
            reached[base] += 1
        if re.search(r'(^|_)(assert\w*|fatal\w*|(?:hard|bus|usage)?fault\w*|except\w*)(_|$)',
                     base.lower()):
            raise RuntimeError(f'Unexpected firmware failure: {name}')
        if base == 'printk' and phase == 'worker':
            # Keep real printk/package execution. Save the ABI arguments before
            # its prologue for independent linked formatter output afterwards.
            args = struct.pack('<III', *(cpu.reg_read(reg) for reg in
                                       (UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3)))
            args += bytes(cpu.mem_read(cpu.reg_read(UC_ARM_REG_SP), 64))
            print_calls.append((cpu.reg_read(UC_ARM_REG_R0), args))
            return
        if base == 'z_log_msg_finalize' and phase == 'worker':
            check(cpu.reg_read(UC_ARM_REG_R0) != 0, 'Logger fixture exhausted its linked buffer')
        if case == 'fem_retained_events' and phase == 'worker':
            if base == 'rssi_scan_fem_prepare':
                retained['prepare_return_address'] = cpu.reg_read(UC_ARM_REG_LR) & ~1
            elif base == 'mpsl_fem_lna_configuration_set' and not retained['activate']:
                retained['activate'] = cpu.reg_read(UC_ARM_REG_R0)
                retained['deactivate'] = cpu.reg_read(UC_ARM_REG_R1)
                retained['activate_bytes'] = bytes(
                    machine.mem_read(retained['activate'], fem_event_size))
                retained['deactivate_bytes'] = bytes(
                    machine.mem_read(retained['deactivate'], fem_event_size))
            elif (base == 'mpsl_fem_lna_configuration_clear' and retained['activate']
                  and not retained['clear_checked']):
                retained['clear_checked'] = True
                unchanged = (bytes(machine.mem_read(retained['activate'], fem_event_size))
                             == retained['activate_bytes'] and
                             bytes(machine.mem_read(retained['deactivate'], fem_event_size))
                             == retained['deactivate_bytes'])
                retained['contents_unchanged_at_clear'] = unchanged
                check(unchanged, 'MPSL retained FEM event descriptors changed before clear')
        if base == 'esb_initialize_internal' and phase == 'worker':
            restores += 1
        if base == 'esb_radio_operation_end' and phase == 'worker':
            release_states.append(word(RADIO + STATE))
        if base == 'nrfx_timer_init' and injecting and case == 'restore_failure' and restores:
            # Failure at the driver leaf, NOT replacing restoration or SDK init.
            ret(-5)
        elif base == 'k_thread_create':
            check(cpu.reg_read(UC_ARM_REG_R0) == queue + qm['thread'], 'Wrong workqueue thread')
            check(cpu.reg_read(UC_ARM_REG_R1) == stack[0], 'Workqueue uses a different stack')
            check(cpu.reg_read(UC_ARM_REG_R2) == budget, 'Linked init disagrees with stack budget')
            dispatcher = cpu.reg_read(UC_ARM_REG_R3)
            check(dispatcher & ~1 in functions and
                  functions[dispatcher & ~1].split('.')[0] == 'work_queue_main',
                  'rssi_wq_init did not create the real work_queue_main dispatcher')
            ret(queue + qm['thread'])
        elif base in ('z_pend_curr', 'z_sched_wait') and phase == 'worker':
            pending = queue + qm['pending']
            check(word(pending + sl['head']) == 0, 'Dispatcher slept with pending work')
            check(word(work + wm['flags']) == 0, 'Dispatcher did not retire the work item')
            check(word(symbol('rssi_scan_busy')[0]) == 0, 'Worker left rssi_scan_busy set')
            completed = True
            cpu.emu_stop()
        elif base in ('z_spin_lock_valid', 'z_spin_unlock_valid'):
            ret(1)
        elif base == '__aeabi_read_tp':
            ret(TLS)
        elif base.startswith('k_busy_wait'):
            virtual_us += cpu.reg_read(UC_ARM_REG_R0)
            ret()
        elif base == 'sys_clock_cycle_get_32':
            # One RTC tick per read: deterministic timeouts, no real-time claim.
            virtual_us += 31
            ret(virtual_us * 32768 // 1_000_000)
        elif base.startswith('k_msleep') or base in ('k_sleep', 'z_impl_k_sleep'):
            virtual_us += 1000
            drain_logs()
            ret()
        elif base.startswith('k_uptime_get'):
            ret(virtual_us // 1000)
        elif base == 'onoff_request':
            cm, _ = abi['onoff_client']
            nm, _ = abi['sys_notify']
            notify = cpu.reg_read(UC_ARM_REG_R1) + cm['notify']
            put_word(notify + nm['flags'], 0)
            put_word(notify + nm['result'], 0)
            ret()
        elif base in ('z_spin_lock_set_owner', 'k_wakeup',
                      'z_impl_k_wakeup', 'k_thread_name_set', 'z_impl_k_thread_name_set',
                      'reschedule', 'z_reschedule', 'z_reschedule_unlocked', 'z_swap_irqlock',
                      'z_sched_wake', 'z_sched_wake_all', 'z_impl_k_yield', 'k_yield',
                      'sys_clock_set_timeout'):
            ret()
        else:
            return
        modeled[base] += 1

    machine.hook_add(UC_HOOK_CODE, hook)
    machine.hook_add(UC_HOOK_MEM_WRITE, mmio, begin=0x40000000, end=0x400fffff)
    machine.hook_add(UC_HOOK_MEM_WRITE, mmio, begin=0x50000000, end=0x50000fff)
    machine.hook_add(UC_HOOK_MEM_WRITE, mmio, begin=0xE000ED0C, end=0xE000ED0F)

    def run(address, args=(), measured=False):
        nonlocal phase
        previous = phase
        phase = 'worker' if measured else 'control'
        machine.reg_write(UC_ARM_REG_SP, top if measured else INIT_TOP)
        machine.reg_write(UC_ARM_REG_LR, STOP | 1)
        for reg, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3),
                              tuple(args) + (0,) * (4 - len(args))):
            machine.reg_write(reg, value)
        machine.emu_start(address | 1, STOP, count=INSTRUCTION_LIMIT)
        if not measured:
            check(machine.reg_read(UC_ARM_REG_PC) == STOP, 'Control call hit instruction limit')
        result = machine.reg_read(UC_ARM_REG_R0)
        phase = previous
        return result

    def call(name, args=()):
        return run(symbol(name)[0], args)

    def initializer_target(address):
        im, _ = abi['init_entry']
        device = word(address + im['dev'])
        if device:
            dm, _ = abi['device']
            om, _ = abi['device_ops']
            function = word(device + dm['ops'] + om['init'])
        else:
            function = word(address + im['init_fn'])
        return function, device

    def run_initializer(entry):
        function, device = initializer_target(symbol(entry)[0])
        check(function & ~1 in functions, f'Invalid linked initializer {entry}')
        check(run(function, (device,)) == 0, f'Real linked initializer failed: {entry}')

    def initialize_frontend():
        # Reproduce only the real boot prerequisites of the FEM-only build.
        # GPIO device setup establishes its callback/control state (allocation
        # masks are already preserved from ELF .data). Library init installs its
        # API, then the DT wrapper allocates actual PA/LNA GPIO/PPI resources.
        im, init_size = abi['init_entry']
        gpio_entries = []
        for name, (address, size) in symbols.items():
            if not name.startswith('__init_') or size != init_size:
                continue
            function, _ = initializer_target(address)
            if functions.get(function & ~1, '').split('.')[0] == 'gpio_nrfx_init':
                gpio_entries.append((address, name))
        check(gpio_entries, 'FEM build lacks linked Nordic GPIO initializers')
        for _, name in sorted(gpio_entries):
            run_initializer(name)
        run_initializer('__init_mpsl_fem_api_init')
        run_initializer('__init_mpsl_fem_init')

    def rendered_output(calls):
        nonlocal phase
        rendered.clear()
        phase = 'format'
        for fmt, args in calls:
            machine.mem_write(ARGS, args)
            run(symbol('cbvprintf')[0], (OUTPUT | 1, 0, fmt, ARGS))
        phase = 'control'
        return rendered.decode('utf-8')

    def request(expected):
        check(call('rcv_cmd_rssi_scan') == expected, f'Wrong command status; expected {expected}')

    def dispatch(expected_mode):
        nonlocal completed, injecting, restores, phase
        completed = False
        restores = 0
        visits.clear()
        first_samples, first_attempts = samples, sample_attempts
        first_prepares = reached['rssi_scan_fem_prepare']
        first_calls = len(print_calls)
        first_releases = len(release_states)
        injecting = expected_mode not in ('success', 'busy')
        check(dispatcher is not None, 'Missing linked workqueue dispatcher')
        phase = 'control'
        run(dispatcher, (queue, 0, 0), measured=True)
        check(completed, 'Work queue did not finish the handler and return to idle')
        text = rendered_output(print_calls[first_calls:])
        owner = word(symbol('radio_operation_busy')[0])
        count = samples - first_samples
        attempts = sample_attempts - first_attempts
        scans.append({'mode': expected_mode, 'visits': sum(visits.values()),
                      'samples': count, 'sample_attempts': attempts,
                      'radio_owner_retained': bool(owner), 'radio_state': word(RADIO + STATE),
                      'home_reinitializations': restores, 'output': text})
        scans[-1]['frontend_preparations'] = reached['rssi_scan_fem_prepare'] - first_prepares
        if expected_mode == 'busy':
            check(not visits and count == 0 and restores == 0, 'Busy worker disturbed radio')
            check(owner == 1 and 'RSSI scan busy: radio in use' in text,
                  'Busy worker lost external owner or did not report rejection')
        elif expected_mode == 'disable_failure':
            check(owner == 1 and restores == 0 and len(release_states) == first_releases,
                  'Unsafe disable released owner or attempted home restore')
            check(sum(visits.values()) == 1 and word(RADIO + STATE) == 3,
                  'Disable fault did not stop scan at first unsafe visit')
            check('cleanup failed; retaining radio owner' in text, 'Missing unsafe cleanup diagnostic')
        else:
            stalled = expected_mode == 'fem_timer_stall'
            expected_visits = {} if stalled else {ch: SWEEPS for ch in CHANNELS}
            check(dict(visits) == expected_visits, f'Incomplete scan visit coverage: {dict(visits)}')
            if stalled:
                check(scans[-1]['frontend_preparations'] == len(CHANNELS) * SWEEPS,
                      'FEM timer stall did not exercise every real preparation/timeout')
            expected_samples = (0 if expected_mode in ('all_invalid', 'restore_failure', 'fem_timer_stall') else
                                len(CHANNELS) * SWEEPS * SAMPLES -
                                (SAMPLES if expected_mode == 'one_invalid' else 0))
            expected_attempts = (0 if stalled else len(CHANNELS) * SWEEPS if expected_samples == 0 else
                                 expected_samples + (1 if expected_mode == 'one_invalid' else 0))
            check(count == expected_samples and attempts == expected_attempts,
                  f'Wrong sample coverage: samples={count}/{expected_samples}, '
                  f'attempts={attempts}/{expected_attempts}')
            check(restores == 1, 'Real home ESB reinitialization was not executed exactly once')
            if expected_mode == 'restore_failure':
                check(owner == 1 and len(release_states) == first_releases,
                      'Failed home restore released radio owner')
                check(word(RADIO + STATE) == 0 and 'home radio restore failed' in text,
                      'Restore failure did not retain disabled radio/report failure')
                check('Recommended:' not in text, 'Failed restore printed a recommendation')
            else:
                check(owner == 0 and word(RADIO + STATE) == 3,
                      'Scan did not restore RX and release owner')
                check(release_states[first_releases:] == [3], 'Owner released before physical RX')
                check(word(RADIO + FREQUENCY) == HOME, 'Scan did not restore home channel')
                mismatches = {hex(offset): (expected, word(RADIO + offset))
                              for offset, expected in home_radio.items()
                              if word(RADIO + offset) != expected}
                check(not mismatches, f'Home RADIO configuration was not restored: {mismatches}')
                if scan_fem_enabled:
                    check(call('rssi_scan_fem_is_safe') == 1, 'Scan leaked FEM/timer ownership')
                check('Current channel (effective): 18' in text, 'Wrong effective home output')
                invalid = {HOME} if expected_mode == 'one_invalid' else (
                    set(CHANNELS) if expected_mode in ('all_invalid', 'fem_timer_stall') else set())
                expected_order = sorted((ch for ch in CHANNELS if ch not in invalid),
                                        key=lambda ch: (-score(ch), abs(ch - HOME), ch))
                table = text.split('All channels sorted by RSSI (lower is better):\n')
                check(len(table) == 2, 'Missing/duplicate sorted RSSI table')
                actual = [(int(ch), int(value)) for ch, value in
                          re.findall(r'(\d+)\s+\(-\s*(\d+) dBm\)', table[1])]
                check(actual == [(ch, score(ch)) for ch in expected_order],
                      f'Incorrect sorted measurement table: {actual}')
                if expected_order:
                    best = expected_order[0]
                    check(f'Recommended: channel {best} (-{score(best)} dBm)' in text,
                          'Wrong recommended channel/minimum sample')
                else:
                    check('Recommended: none (no valid complete channel measurements)' in text,
                          'All-invalid scan recommended a channel')
                invalid_rows = [(int(ch), int(bad), int(total)) for ch, bad, total in
                                re.findall(r'Channel (\d+) invalid: (\d+)/(\d+) visits failed; excluded', text)]
                check(invalid_rows == [(ch, 1 if expected_mode == 'one_invalid' else SWEEPS, SWEEPS)
                                       for ch in sorted(invalid)], 'Incorrect invalid visit accounting')

    try:
        # Boot normally initializes the SDK allocator before application init.
        # Preserve its real channel/group allocation/free implementation.
        check(call('gppi_init') == 0, 'Real GPPI allocator initialization failed')
        if scan_fem_enabled:
            initialize_frontend()
        check(call('rssi_wq_init') == 0 and dispatcher is not None, 'Real workqueue init failed')
        call('esb_set_addr_unified')
        # The quiet wrapper can be LTO-inlined; its internal function has the
        # same (tx, quiet) ABI and remains the actual full SDK restoration path.
        check(call('esb_initialize_internal', (0, 1)) == 0, 'Initial real ESB initialization failed')
        check(call('esb_start_rx') == 0 and word(RADIO + STATE) == 3, 'Initial physical RX failed')
        home_radio = {offset: word(RADIO + offset) for offset in HOME_REGISTERS}
        started, busy = status_code('RCV_HID_ST_STARTED'), status_code('RCV_HID_ST_EBUSY')
        request(started)
        request(busy)  # real atomic command gate, before work starts
        dispatch('success' if case in ('success_repeat', 'fem_retained_events') else case)
        if case == 'success_repeat':
            check(call('esb_radio_operation_begin') == 0, 'Cannot acquire external radio owner')
            request(busy)
            call('esb_radio_operation_end')
            request(started)
            # Another operation wins ownership after submission, before dispatch.
            check(call('esb_radio_operation_begin') == 0, 'Cannot inject queued-owner race')
            dispatch('busy')
            call('esb_radio_operation_end')
            request(started)
            dispatch('success')  # prove busy flag and work item are reusable
        elif case in ('restore_failure', 'disable_failure'):
            request(busy)  # physical safety failure must not permit a new scan
        else:
            # Acceptance independently proves invalid scans release both gates;
            # do not execute another full scan just to test the command gate.
            request(started)
        check(reached['printk'] > 0, 'Real printk was not executed')
        check(reached['z_log_msg_alloc'] > 0 and reached['z_log_msg_finalize'] > 0,
              'Real worker-side log allocation/publication was not executed')
        if case != 'disable_failure':
            check(any(name.startswith('cbvprintf_package') and count for name, count in reached.items()),
                  'Real worker-side printf packaging was not executed')
        if scan_fem_enabled:
            check(reached['rssi_scan_fem_prepare'] > 0 and reached['rssi_scan_fem_release'] > 0,
                  'FEM build did not execute real preparation and cleanup')
            check(reached['mpsl_fem_lna_configuration_set'] > 0 and
                  reached['mpsl_fem_lna_configuration_clear'] > 0,
                  'Real proprietary FEM configuration/cleanup was not executed')
    except (UcError, RuntimeError, ValueError) as exc:
        failures.append(f'{phase}: {exc}; pc={machine.reg_read(UC_ARM_REG_PC):#x}; '
                        f'recent={recent[-8:]}')
    used = top - lowest
    if case == 'fem_retained_events':
        if not (retained['prepare_returned'] and retained['clear_checked']):
            failures.append('Retained-event case did not execute prepare/return/clear')
        if retained['outside_worker_stack'] is False:
            failures.append('FEM event descriptors passed to MPSL live in the worker stack frame')
    if used > budget:
        failures.append(f'Measured worker stack {used} exceeds linked budget {budget}')
    if used + EXCEPTION_RESERVE > budget:
        failures.append(f'Measured worker stack {used} plus basic exception frame '
                        f'{EXCEPTION_RESERVE} exceeds linked budget {budget}')
    return {'case': case, 'entry': 'work_queue_main -> rssi_scan_work_handler',
            'modeled_soc': soc['name'],
            'linked_stack_object_bytes': stack[1], 'architecture_reserved_bytes': reserved,
            'linked_stack_budget': budget, 'measured_worker_stack_lower_bound': used,
            'basic_exception_frame_reserve': EXCEPTION_RESERVE,
            'headroom_after_basic_exception_frame': budget - used - EXCEPTION_RESERVE,
            'deepest_recent_functions': deepest, 'instructions': instructions,
            'real_worker_printk_calls': reached['printk'],
            'real_worker_log_allocations': reached['z_log_msg_alloc'],
            'real_worker_log_publications': reached['z_log_msg_finalize'],
            'real_worker_cbvprintf_package_calls': reached['cbvprintf_package'],
            'modeled_leaves': dict(modeled), 'scans': scans, 'failures': failures,
            'retained_fem_events': {
                'activate_address': hex(retained['activate']),
                'deactivate_address': hex(retained['deactivate']),
                'object_bytes': fem_event_size,
                'outside_worker_stack': retained['outside_worker_stack'],
                'expired_frame_reuse_modeled': retained['reused_frame_poisoned'],
                'contents_unchanged_at_clear': retained['contents_unchanged_at_clear'],
            }}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=Path, help='Unstripped linked nRF52 receiver ELF with DWARF')
    parser.add_argument('--case', choices=CASES + OPTIONAL_CASES, action='append', dest='cases',
                        help='Run selected cases (repeatable); default: all standard '
                             'cases supported by the linked build')
    parser.add_argument('--soc', choices=sorted(SOCS), default='nrf52840',
                        help='Modeled receiver SoC for chip-identity reads (default: nrf52840)')
    args = parser.parse_args()
    image = load_image(args.elf)
    fem_build = any(key.startswith('rssi_scan_fem_prepare') and value[1]
                    for key, value in image[0].items())
    if args.cases and not fem_build and set(args.cases) & set(OPTIONAL_CASES):
        parser.error(f'{" / ".join(sorted(set(args.cases) & set(OPTIONAL_CASES)))} '
                     'requires a linked FEM build')
    # The restore fault is injected at the real FEM driver leaf; non-FEM builds
    # have that SDK initialization inlined and provide no honest injection point.
    if args.cases and not fem_build and 'restore_failure' in args.cases:
        parser.error('restore_failure requires a linked FEM build')
    names = {'k_work_q', 'k_work', '_slist', 'init_entry', 'mpsc_pbuf_buffer',
             'onoff_client', 'sys_notify', 'device', 'device_ops'}
    if fem_build:
        names.add('mpsl_fem_event_t')
    abi = layouts(args.elf, names)
    default_cases = CASES if fem_build else tuple(c for c in CASES if c != 'restore_failure')
    results = [replay(args.elf, image, abi, case, SOCS[args.soc])
               for case in args.cases or default_cases]
    print(json.dumps({'elf': str(args.elf), 'hardware_stack_safety_proof': False,
                      'cases': results}, indent=2))
    return int(any(result['failures'] for result in results))


if __name__ == '__main__':
    raise SystemExit(main())
