#!/usr/bin/env python3
"""Replay the linked ARM stats thread, including LTO TDMA and cbprintf code.

Run with an unstripped receiver zephyr.elf containing DWARF:
    uv run --with unicorn --with pyelftools run_stack.py PATH/zephyr.elf

Only uptime, semaphore/sleep scheduling, uncontended mutex ownership, scheduler
lock/unlock, and log allocation/finalization are replaced. The semaphore leaf
models finite tick timeouts, an already-pending signal, and a signal delivered while waiting.
Tracker cases inject the post-validation queued PING evidence, first-evidence
deadline and semaphore signal; the linked
stats owner performs the real merge and layout publication.
One additional case supplies sixteen distinct validated PING events 250 ms
apart, with no unmodeled refreshes, and checks the linked one-second loss and
TPS-log cadence through all 20 seconds.
The measured maximum is a LOWER BOUND: thread entry, exception/context frames,
interrupts, and the replaced leaves are excluded. Passing is not hardware stack
safety proof; exceeding the linked usable stack budget is a definite failure.
No source matching, guessed guard size, or added safety margin is used.
"""

import argparse
import errno
import json
from pathlib import Path
import struct

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UcError, UC_ARCH_ARM, UC_HOOK_CODE, UC_HOOK_MEM_WRITE, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import (
    UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1,
    UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP,
)


DURATION_MS = 20_000
COHORT_DELAY_MS = 101
DELAYED_PING_MS = 399
RECURRING_START_MS = 6_000
RECURRING_INTERVAL_MS = 250
INSTRUCTION_LIMIT = 2_000_000
SCRATCH_BASE = 0x21000000
SCRATCH_SIZE = 0x100000


def static_thread_offsets(elf):
    """Use this ELF's ABI, not the host ABI or a guessed Zephyr struct layout."""
    required = {'init_stack', 'init_stack_size', 'init_entry'}
    for unit in elf.get_dwarf_info().iter_CUs():
        for die in unit.iter_DIEs():
            name = die.attributes.get('DW_AT_name')
            if (die.tag != 'DW_TAG_structure_type' or name is None
                    or name.value != b'_static_thread_data'):
                continue
            offsets = {}
            for member in die.iter_children():
                name = member.attributes.get('DW_AT_name')
                location = member.attributes.get('DW_AT_data_member_location')
                if name and location and isinstance(location.value, int):
                    offsets[name.value.decode()] = location.value
            if required <= offsets.keys():
                return offsets
    raise ValueError('ELF needs DWARF for _static_thread_data member offsets')


def log_package_layout(elf):
    """Extract the linked message/package fields and descriptor bitfield."""
    required = {
        'log_msg': {'data'},
        'cbprintf_package_hdr_ext': {'fmt'},
        'log_msg_desc': {'package_len'},
    }
    found = {}
    for unit in elf.get_dwarf_info().iter_CUs():
        for die in unit.iter_DIEs():
            name = die.attributes.get('DW_AT_name')
            if die.tag != 'DW_TAG_structure_type' or name is None:
                continue
            name = name.value.decode()
            if name not in required or name in found:
                continue
            members = {}
            for member in die.iter_children():
                member_name = member.attributes.get('DW_AT_name')
                if member.tag == 'DW_TAG_member' and member_name:
                    members[member_name.value.decode()] = member
            if required[name] <= members.keys() and 'DW_AT_byte_size' in die.attributes:
                found[name] = (die, members)
        if required.keys() <= found.keys():
            break
    if not required.keys() <= found.keys():
        raise ValueError('ELF needs DWARF for the log message and cbprintf package layouts')

    msg, msg_members = found['log_msg']
    package, package_members = found['cbprintf_package_hdr_ext']
    desc, desc_members = found['log_msg_desc']
    data = msg_members['data'].attributes.get('DW_AT_data_member_location')
    fmt_member = package_members['fmt']
    fmt = fmt_member.attributes.get('DW_AT_data_member_location')
    fmt_type = fmt_member.get_DIE_from_attribute('DW_AT_type')
    if (data is None or not isinstance(data.value, int)
            or fmt is None or not isinstance(fmt.value, int)
            or fmt_type.tag != 'DW_TAG_pointer_type'
            or fmt_type.attributes.get('DW_AT_byte_size') is None
            or fmt_type.attributes['DW_AT_byte_size'].value != 4
            or data.value > msg.attributes['DW_AT_byte_size'].value
            or fmt.value + 4 > package.attributes['DW_AT_byte_size'].value
            or desc.attributes['DW_AT_byte_size'].value != 4):
        raise ValueError('Unsupported linked log message/package ABI')

    field = desc_members['package_len'].attributes
    bit_size = field.get('DW_AT_bit_size')
    if bit_size is None:
        raise ValueError('ELF needs DWARF for the log package length bitfield')
    if 'DW_AT_data_bit_offset' in field:
        shift = field['DW_AT_data_bit_offset'].value
    elif all(key in field for key in ('DW_AT_bit_offset', 'DW_AT_byte_size',
                                       'DW_AT_data_member_location')):
        shift = (field['DW_AT_data_member_location'].value * 8
                 + field['DW_AT_byte_size'].value * 8
                 - field['DW_AT_bit_offset'].value - bit_size.value)
    else:
        raise ValueError('Unsupported DWARF log package length bitfield')
    if not 0 <= shift < 32 or not 0 < bit_size.value <= 32 - shift:
        raise ValueError('Invalid DWARF log package length bitfield')
    return {
        'data': data.value,
        'fmt': fmt.value,
        'package_len_shift': shift,
        'package_len_mask': (1 << bit_size.value) - 1,
    }


def load_image(path):
    with path.open('rb') as stream:
        elf = ELFFile(stream)
        if elf.elfclass != 32 or not elf.little_endian or elf['e_machine'] != 'EM_ARM':
            raise ValueError('Expected a little-endian 32-bit ARM receiver ELF')
        symtab = elf.get_section_by_name('.symtab')
        if symtab is None:
            raise ValueError('ELF must retain its symbol table')
        symbols = {symbol.name: (int(symbol['st_value']), int(symbol['st_size']))
                   for symbol in symtab.iter_symbols()}
        functions = {int(symbol['st_value']) & ~1: symbol.name
                     for symbol in symtab.iter_symbols()
                     if symbol['st_info']['type'] == 'STT_FUNC'
                     and symbol['st_shndx'] != 'SHN_UNDEF'}
        sections = [(int(section['sh_addr']), section.data())
                    for section in elf.iter_sections()
                    if section['sh_flags'] & 2 and section['sh_type'] != 'SHT_NOBITS'
                    and section['sh_size']]
        return symbols, functions, sections, static_thread_offsets(elf)


def replay(image, trackers, signal_mode=None):
    symbols, functions, sections, offsets, log_layout = image
    machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    # The supported nRF52 receiver image address spaces. Peripheral memory is
    # inert: no RF/USB device emulation or timing claims are made by this test.
    for start in (0, 0x20000000, 0x40000000, 0xE0000000, SCRATCH_BASE):
        machine.mem_map(start, 0x100000)
    for address, data in sections:
        machine.mem_write(address, data)

    def word(address):
        return struct.unpack('<I', machine.mem_read(address, 4))[0]

    def put(name, value, size=4):
        machine.mem_write(symbols[name][0], value.to_bytes(size, 'little'))

    stack, storage = symbols['_k_thread_stack_esb_stats_thread_id']
    thread_data = symbols['_k_thread_data_esb_stats_thread_id'][0]
    budget = word(thread_data + offsets['init_stack_size'])
    entry = word(thread_data + offsets['init_entry'])
    if word(thread_data + offsets['init_stack']) != stack:
        raise ValueError('Static thread initializer does not reference its stack')
    if entry != symbols['esb_stats_thread'][0] or not 0 < budget <= storage:
        raise ValueError('Inconsistent linked stats thread initializer')
    # Zephyr places the usable downward-growing stack at the top of its storage;
    # init_stack_size already excludes the architecture-reserved guard bytes.
    top = stack + storage
    machine.reg_write(UC_ARM_REG_SP, top)
    machine.reg_write(UC_ARM_REG_LR, 0x80001)
    put('stored_trackers', trackers, 1)
    ticks_per_second = symbols['CONFIG_SYS_CLOCK_TICKS_PER_SEC'][0]
    if ticks_per_second >= 1 << 31:
        raise ValueError('Linked ELF has an invalid system tick frequency')
    timeout_64 = bool(symbols.get('CONFIG_TIMEOUT_64BIT', (0, 0))[0])
    if not ticks_per_second:
        raise ValueError('Linked ELF has no usable system tick frequency')
    if signal_mode not in (None, 'pending', 'delayed', 'recurring') or (signal_mode and not trackers):
        raise ValueError('Invalid membership-signal scenario')
    if signal_mode == 'recurring' and trackers != 16:
        raise ValueError('Recurring admissions require 16 stored tracker IDs')
    duration_ticks = (DURATION_MS * ticks_per_second + 999) // 1000
    signal_ticks = (DELAYED_PING_MS * ticks_per_second + 999) // 1000
    now_ticks = 0
    semaphore_count = 0
    immediate_wakes = 0
    delayed_wakes = 0
    semaphore_timeouts = 0
    semaphore_no_waits = 0
    timeout_ticks_seen = set()
    ping_injected = False
    ping_time_ms = None
    ping_events = []
    recurring_index = 0
    loss_tick_times = []
    tps_log_times = []
    first_layout = None
    lowest = top
    instructions = 0
    packages = 0
    tdma_packages = 0
    finalized = 0
    allocated = SCRATCH_BASE
    allocations = {}
    mutex_depths = {}
    mutex_locks = 0
    mutex_unlocks = 0
    trace = []
    deepest = []
    completed = False

    def uptime_ms():
        return now_ticks * 1000 // ticks_per_second

    def ret(value=0):
        value &= 0xFFFFFFFFFFFFFFFF
        machine.reg_write(UC_ARM_REG_R0, value & 0xFFFFFFFF)
        machine.reg_write(UC_ARM_REG_R1, value >> 32)
        machine.reg_write(UC_ARM_REG_PC, machine.reg_read(UC_ARM_REG_LR))

    def inject_valid_ping(mask):
        nonlocal semaphore_count, ping_injected, ping_time_ms
        assert trackers > 0 and mask and not mask & ~((1 << trackers) - 1)
        now_ms = uptime_ms()
        desired = word(symbols['tdma_shadow_desired_mask'][0])
        queued = word(symbols['tdma_shadow_pending_join_mask'][0])
        if mask & (desired | queued):
            raise RuntimeError('A modeled admission PING is already desired or queued')
        put('tdma_shadow_pending_join_mask', queued | mask)
        if not machine.mem_read(symbols['tdma_membership_deadline_armed'][0], 1)[0]:
            put('tdma_membership_deadline_ms', now_ms + COHORT_DELAY_MS)
            put('tdma_membership_deadline_armed', 1, 1)
        put('tdma_shadow_live_evidence_mask', word(symbols['tdma_shadow_live_evidence_mask'][0]) | mask)
        put('tdma_shadow_ever_ping_mask', word(symbols['tdma_shadow_ever_ping_mask'][0]) | mask)
        stamps = symbols['tdma_shadow_last_seen_ms'][0]
        for tracker_id in range(trackers):
            if mask & (1 << tracker_id):
                machine.mem_write(stamps + 4 * tracker_id, struct.pack('<I', now_ms))
        semaphore_count = 1
        ping_injected = True
        if ping_time_ms is None:
            ping_time_ms = now_ms
        ping_events.append((now_ms, mask))

    if trackers and signal_mode in (None, 'pending'):
        # The validated PING arrived before the stats owner enters its wait.
        inject_valid_ping((1 << trackers) - 1)

    def hook(cpu, pc, size, user_data):
        nonlocal now_ticks, lowest, instructions, packages, tdma_packages, finalized
        nonlocal allocated, deepest, completed, semaphore_count, immediate_wakes
        nonlocal delayed_wakes, semaphore_timeouts, semaphore_no_waits, first_layout
        nonlocal recurring_index
        nonlocal mutex_locks, mutex_unlocks
        instructions += 1
        sp = cpu.reg_read(UC_ARM_REG_SP)
        if sp < lowest:
            lowest = sp
            deepest = trace[-12:]
        name = functions.get(pc)
        if name is None:
            return
        trace.append(name)
        if len(trace) > 24:
            del trace[:-12]
        if 'assert' in name or 'fatal' in name:
            raise RuntimeError(f'Unexpected firmware failure: {name}')
        if name.startswith('k_uptime_get'):
            ret(uptime_ms())
        elif name.startswith(('k_sem_take', 'z_impl_k_sem_take')) and (
            cpu.reg_read(UC_ARM_REG_R0) == symbols['tdma_membership_wake'][0]
        ):
            if mutex_depths:
                raise RuntimeError('Stats thread waited while holding a modeled mutex')
            if signal_mode and first_layout is None and machine.mem_read(
                symbols['tdma_config_epoch'][0], 1
            )[0]:
                first_layout = {
                    'time_ms': uptime_ms(),
                    'mask': word(symbols['tdma_active_mask'][0]),
                    'count': machine.mem_read(symbols['tdma_dynamic_active_count'][0], 1)[0],
                    'epoch': machine.mem_read(symbols['tdma_config_epoch'][0], 1)[0],
                    'configs': list(struct.unpack(
                        '<16I', machine.mem_read(symbols['tdma_config_packed'][0], 64))),
                }
            # ARM AAPCS aligns a 64-bit k_timeout_t after the semaphore pointer
            # to r2/r3. A 32-bit timeout occupies r1. Frequency/config come
            # from absolute CONFIG symbols in this exact linked ELF.
            timeout_ticks = (cpu.reg_read(UC_ARM_REG_R2)
                             | cpu.reg_read(UC_ARM_REG_R3) << 32) if timeout_64 else cpu.reg_read(UC_ARM_REG_R1)
            timeout_ticks_seen.add(timeout_ticks)
            timeout_limit = (1 << (64 if timeout_64 else 32)) - 1
            if timeout_ticks & (1 << (63 if timeout_64 else 31)):
                raise RuntimeError(f'Unexpected non-finite/absolute semaphore timeout: {timeout_ticks:#x}')
            if now_ticks >= duration_ticks:
                completed = True
                cpu.emu_stop()
                return
            if semaphore_count:
                semaphore_count -= 1
                immediate_wakes += 1
                ret(0)
                return
            if signal_mode == 'delayed' and not ping_injected and now_ticks < signal_ticks <= now_ticks + timeout_ticks:
                now_ticks = signal_ticks
                inject_valid_ping((1 << trackers) - 1)
                semaphore_count -= 1
                delayed_wakes += 1
                ret(0)
                return
            if signal_mode == 'recurring' and recurring_index < trackers:
                event_ms = RECURRING_START_MS + recurring_index * RECURRING_INTERVAL_MS
                event_ticks = (event_ms * ticks_per_second + 999) // 1000
                if now_ticks < event_ticks <= now_ticks + timeout_ticks:
                    now_ticks = event_ticks
                    inject_valid_ping(1 << recurring_index)
                    recurring_index += 1
                    semaphore_count -= 1
                    delayed_wakes += 1
                    ret(0)
                    return
            if timeout_ticks == 0:
                semaphore_no_waits += 1
                ret(-errno.EBUSY)
                return
            if timeout_ticks == timeout_limit:
                raise RuntimeError('Unexpected infinite semaphore wait')
            now_ticks += timeout_ticks
            semaphore_timeouts += 1
            if ping_injected and signal_mode != 'recurring':
                cpu.mem_write(symbols['tdma_shadow_last_seen_ms'][0], struct.pack(
                    '<16I', *([uptime_ms() + 1] * trackers + [0] * (16 - trackers))))
            ret(-errno.EAGAIN)
        elif name.startswith('k_msleep'):
            delay = cpu.reg_read(UC_ARM_REG_R0)
            if not 0 < delay <= DURATION_MS:
                raise RuntimeError(f'Unexpected stats sleep: {delay}')
            if now_ticks >= duration_ticks:
                completed = True
                cpu.emu_stop()
                return
            now_ticks += (delay * ticks_per_second + 999) // 1000
            if ping_injected and signal_mode != 'recurring':
                cpu.mem_write(symbols['tdma_shadow_last_seen_ms'][0], struct.pack(
                    '<16I', *([uptime_ms() + 1] * trackers + [0] * (16 - trackers))))
            ret()
        elif name in ('k_sched_lock', 'k_sched_unlock'):
            ret()
        elif name.startswith('z_impl_k_mutex_lock'):
            # The linked k_mutex_lock wrapper branches here. Model the
            # uncontended, recursive Zephyr mutex for this single owner.
            mutex = cpu.reg_read(UC_ARM_REG_R0)
            if not mutex:
                raise RuntimeError('Attempt to lock a null mutex')
            mutex_depths[mutex] = mutex_depths.get(mutex, 0) + 1
            mutex_locks += 1
            ret()
        elif name.startswith('z_impl_k_mutex_unlock'):
            # Execute the real wrapper; intercept its kernel implementation.
            mutex = cpu.reg_read(UC_ARM_REG_R0)
            depth = mutex_depths.get(mutex, 0)
            if not depth:
                raise RuntimeError(f'Unlock of an unheld mutex: {mutex:#x}')
            if depth == 1:
                del mutex_depths[mutex]
            else:
                mutex_depths[mutex] = depth - 1
            mutex_unlocks += 1
            ret()
        elif name.startswith('z_log_msg_finalize'):
            # AAPCS32 passes msg, source, and the 32-bit descriptor in r0-r2.
            # Read the actual package before stubbing out the finalizer. Simple
            # integer-only logs are built directly and bypass cbprintf_convert.
            msg = cpu.reg_read(UC_ARM_REG_R0)
            if signal_mode == 'recurring' and msg:
                length = allocations.get(msg)
                if length is None:
                    raise RuntimeError('Finalized log was not allocated by the modeled allocator')
                desc = cpu.reg_read(UC_ARM_REG_R2)
                package_len = ((desc >> log_layout['package_len_shift'])
                               & log_layout['package_len_mask'])
                if package_len:
                    fmt_end = log_layout['fmt'] + 4
                    if package_len < fmt_end or log_layout['data'] + package_len > length:
                        raise RuntimeError('Finalized log has an invalid package length')
                    format_address = word(msg + log_layout['data'] + log_layout['fmt'])
                    tps_format = b'Total TPS: %u\0'
                    if bytes(cpu.mem_read(format_address, len(tps_format))) == tps_format:
                        tps_log_times.append(uptime_ms())
            finalized += 1
            ret()
        elif name == 'z_log_msg_alloc':
            # Zephyr allocator's argument is a count of 32-bit words. Keep each
            # allocation separate while executing the real package copy code.
            length = cpu.reg_read(UC_ARM_REG_R0) * 4
            if not length or allocated + length > SCRATCH_BASE + SCRATCH_SIZE:
                raise RuntimeError('Unexpected/excessive log allocation')
            result = allocated
            allocated += (length + 7) & ~7
            allocations[result] = length
            ret(result)
        elif name.startswith('cbprintf_package_convert'):
            # Use the linked package header's DWARF format-pointer location.
            packages += 1
            package = cpu.reg_read(UC_ARM_REG_R0)
            format_address = word(package + log_layout['fmt'])
            prefix = bytes(cpu.mem_read(format_address, len(b'TDMA reconfig:')))
            if prefix == b'TDMA reconfig:':
                tdma_packages += 1

    def record_loss_tick(cpu, access, address, size, value, user_data):
        # This first baseline word is written once by each real loss update.
        if address <= loss_baseline < address + size:
            loss_tick_times.append(uptime_ms())

    if signal_mode == 'recurring':
        loss_baseline = symbols['tdma_prev_recv'][0]
        machine.hook_add(UC_HOOK_MEM_WRITE, record_loss_tick,
                         begin=loss_baseline, end=loss_baseline + 3)
    machine.hook_add(UC_HOOK_CODE, hook)
    error = None
    try:
        machine.emu_start(entry, 0x80000, count=INSTRUCTION_LIMIT)
    except (UcError, RuntimeError) as exc:
        error = str(exc)
    used = top - lowest
    failures = []
    if error:
        failures.append(error)
    if not completed:
        failures.append('Did not execute the full 20-second startup scenario')
    if uptime_ms() < DURATION_MS:
        failures.append(f'Insufficient elapsed time: {uptime_ms()} ms')
    if not semaphore_timeouts:
        failures.append('No finite stats semaphore timeout was exercised')
    if not mutex_locks or mutex_locks != mutex_unlocks or mutex_depths:
        failures.append(f'Unbalanced modeled mutex calls: {mutex_locks}/{mutex_unlocks}, {mutex_depths}')
    if finalized < DURATION_MS // 1000:
        failures.append(f'Too few finalized stats logs: {finalized}')
    last_tps_print_ms = struct.unpack(
        '<q', machine.mem_read(symbols['last_tps_print_time'][0], 8))[0]
    if not DURATION_MS - 1000 <= last_tps_print_ms <= uptime_ms():
        failures.append(f'Stats cadence did not reach the end of the replay: {last_tps_print_ms} ms')
    if trackers and (not tdma_packages or not finalized):
        failures.append('Tracker scenario did not package/finalize a TDMA reconfiguration log')
    if used > budget:
        failures.append(f'Measured stack {used} exceeds linked budget {budget}')
    active_mask = word(symbols['tdma_active_mask'][0])
    active_count = machine.mem_read(symbols['tdma_dynamic_active_count'][0], 1)[0]
    if active_mask != (1 << trackers) - 1 or active_count != trackers:
        failures.append(f'Wrong final TDMA roster: mask={active_mask:#x}, count={active_count}')
    cap_level = machine.mem_read(symbols['tdma_cap_level'][0], 1)[0]
    published_cap_level = machine.mem_read(symbols['tdma_published_cap_level'][0], 1)[0]
    if cap_level or published_cap_level:
        failures.append(f'Unexpected loss cap in clean evidence model: {cap_level}/{published_cap_level}')
    if signal_mode:
        if not ping_injected or first_layout is None:
            failures.append('Queued PING did not produce a linked first layout')
        else:
            if first_layout['time_ms'] != ping_time_ms + COHORT_DELAY_MS:
                failures.append(f'Wrong first publication time: {first_layout["time_ms"]} ms')
            first_count = 1 if signal_mode == 'recurring' else trackers
            if first_layout['mask'] != (1 << first_count) - 1 or first_layout['count'] != first_count:
                failures.append('First layout did not contain exactly the modeled membership cohort')
            for tracker_id in range(first_count):
                cfg = first_layout['configs'][tracker_id]
                if ((cfg >> 24) != tracker_id or ((cfg >> 16) & 0xff) != first_count
                    or ((cfg >> 8) & 0xff) < 16 or (cfg & 0xff) != first_layout['epoch']):
                    failures.append(f'Invalid first layout tuple for tracker {tracker_id}: {cfg:#x}')
            cohort_ticks = (COHORT_DELAY_MS * ticks_per_second + 999) // 1000
            if cohort_ticks not in timeout_ticks_seen:
                failures.append('The linked owner did not request the finite membership-deadline timeout')
        if signal_mode == 'pending' and (immediate_wakes != 1 or delayed_wakes):
            failures.append('Already-pending semaphore signal was not consumed immediately exactly once')
        if signal_mode == 'delayed' and (delayed_wakes != 1 or immediate_wakes):
            failures.append('Semaphore signal during a wait did not wake the owner exactly once')
        if signal_mode == 'recurring':
            if recurring_index != trackers or delayed_wakes != trackers or immediate_wakes:
                failures.append('Not every distinct modeled admission woke the linked owner')
            expected_times = list(range(1000, DURATION_MS + 1, 1000))
            if loss_tick_times != expected_times:
                failures.append(f'Expected 20 exact one-second loss ticks: {loss_tick_times}')
            if tps_log_times != expected_times:
                failures.append(f'Expected 20 exact one-second TPS logs: {tps_log_times}')
            if loss_tick_times != tps_log_times:
                failures.append(f'Loss/log cadence diverged: {loss_tick_times} vs {tps_log_times}')
    return {
        'trackers': trackers,
        'elapsed_ms': uptime_ms(),
        'last_tps_print_ms': last_tps_print_ms,
        'system_ticks_per_second': ticks_per_second,
        'timeout_bits': 64 if timeout_64 else 32,
        'membership_signal_mode': signal_mode,
        'first_ping_ms': ping_time_ms,
        'modeled_ping_events': ping_events,
        'loss_tick_times_ms': loss_tick_times,
        'tps_log_times_ms': tps_log_times,
        'first_layout': first_layout,
        'semaphore_immediate_wakes': immediate_wakes,
        'semaphore_delayed_wakes': delayed_wakes,
        'semaphore_timeouts': semaphore_timeouts,
        'semaphore_no_waits': semaphore_no_waits,
        'semaphore_timeout_ticks': sorted(timeout_ticks_seen),
        'mutex_locks': mutex_locks,
        'mutex_unlocks': mutex_unlocks,
        'tdma_cap_level': cap_level,
        'tdma_published_cap_level': published_cap_level,
        'tdma_active_mask': active_mask,
        'tdma_dynamic_active_count': active_count,
        'linked_stack_budget': budget,
        'stack_storage_bytes': storage,
        'measured_thread_stack': used,
        'headroom_excluding_entry_interrupts_and_stubbed_leaves': budget - used,
        'cbprintf_conversion_calls': packages,
        'tdma_reconfig_conversion_calls': tdma_packages,
        'finalized_logs': finalized,
        'instructions': instructions,
        'deepest_recent_functions': deepest,
        'failures': failures,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=Path, help='Unstripped linked receiver ELF with DWARF')
    args = parser.parse_args()
    image = load_image(args.elf)
    with args.elf.open('rb') as stream:
        image = (*image, log_package_layout(ELFFile(stream)))
    # Independent machines preserve linked startup state for every roster size.
    results = [replay(image, trackers) for trackers in (0, 1, 16)]
    results.extend(replay(image, trackers, mode) for trackers, mode in (
        (1, 'pending'), (16, 'pending'), (1, 'delayed'), (16, 'delayed')))
    results.append(replay(image, 16, 'recurring'))
    print(json.dumps({'elf': str(args.elf), 'cases': results}, indent=2))
    return int(any(result['failures'] for result in results))


if __name__ == '__main__':
    raise SystemExit(main())
