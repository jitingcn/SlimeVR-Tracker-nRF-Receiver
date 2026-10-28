#!/usr/bin/env python3
"""Replay the linked ARM stats thread, including LTO TDMA and cbprintf code.

Run with an unstripped receiver zephyr.elf containing DWARF:
    uv run --with unicorn --with pyelftools run_stack.py PATH/zephyr.elf

Only uptime, semaphore/sleep scheduling, scheduler lock/unlock, and log
allocation/finalization are replaced. The semaphore leaf models finite tick
timeouts, an already-pending signal, and a signal delivered while waiting.
Ordinary cases supply tracker evidence without simulating RF. First-cohort
cases also inject the post-validation PING state and semaphore signal; the
linked stats owner performs the real promotion and layout publication.
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
from unicorn import Uc, UcError, UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import (
    UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1,
    UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP,
)


DURATION_MS = 20_000
COHORT_DELAY_MS = 101
DELAYED_PING_MS = 399
INSTRUCTION_LIMIT = 1_000_000
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
    symbols, functions, sections, offsets = image
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
    if signal_mode not in (None, 'pending', 'delayed') or (signal_mode and not trackers):
        raise ValueError('Invalid initial-cohort scenario')
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
    first_layout = None
    lowest = top
    instructions = 0
    packages = 0
    tdma_packages = 0
    finalized = 0
    allocated = SCRATCH_BASE
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

    def inject_valid_ping():
        nonlocal semaphore_count, ping_injected, ping_time_ms
        assert not ping_injected and trackers > 0
        now_ms = uptime_ms()
        mask = (1 << trackers) - 1
        put('tdma_initial_allowed_mask', mask)
        put('tdma_initial_ping_mask', mask)
        put('tdma_initial_started', 1, 1)
        put('tdma_initial_done', 0, 1)
        put('tdma_initial_deadline_ms', now_ms + COHORT_DELAY_MS)
        put('tdma_shadow_live_evidence_mask', mask)
        put('tdma_shadow_ever_ping_mask', mask)
        stamps = struct.pack('<16I', *([now_ms] * trackers + [0] * (16 - trackers)))
        machine.mem_write(symbols['tdma_initial_ping_at_ms'][0], stamps)
        machine.mem_write(symbols['tdma_shadow_last_seen_ms'][0], stamps)
        semaphore_count = 1
        ping_injected = True
        ping_time_ms = now_ms

    if signal_mode == 'pending':
        # The validated PING arrived before the stats owner enters its wait.
        inject_valid_ping()

    def hook(cpu, pc, size, user_data):
        nonlocal now_ticks, lowest, instructions, packages, tdma_packages, finalized
        nonlocal allocated, deepest, completed, semaphore_count, immediate_wakes
        nonlocal delayed_wakes, semaphore_timeouts, semaphore_no_waits, first_layout
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
            cpu.reg_read(UC_ARM_REG_R0) == symbols['tdma_initial_wake'][0]
        ):
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
                inject_valid_ping()
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
            if trackers:
                put('tdma_shadow_live_evidence_mask', (1 << trackers) - 1)
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
            if trackers:
                put('tdma_shadow_live_evidence_mask', (1 << trackers) - 1)
                cpu.mem_write(symbols['tdma_shadow_last_seen_ms'][0], struct.pack(
                    '<16I', *([uptime_ms() + 1] * trackers + [0] * (16 - trackers))))
            ret()
        elif name in ('k_sched_lock', 'k_sched_unlock'):
            ret()
        elif name.startswith('z_log_msg_finalize'):
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
            ret(result)
        elif name.startswith('cbprintf_package_convert'):
            # ARM cbprintf package header occupies one word, followed by the
            # format pointer. Inspect executed packages, never source strings.
            packages += 1
            package = cpu.reg_read(UC_ARM_REG_R0)
            format_address = word(package + 4)
            prefix = bytes(cpu.mem_read(format_address, len(b'TDMA reconfig:')))
            if prefix == b'TDMA reconfig:':
                tdma_packages += 1

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
            failures.append('Initial PING did not produce a linked first layout')
        else:
            if first_layout['time_ms'] != ping_time_ms + COHORT_DELAY_MS:
                failures.append(f'Wrong initial publication time: {first_layout["time_ms"]} ms')
            if first_layout['mask'] != (1 << trackers) - 1 or first_layout['count'] != trackers:
                failures.append('First layout did not contain exactly the modeled initial cohort')
            for tracker_id in range(trackers):
                cfg = first_layout['configs'][tracker_id]
                if ((cfg >> 24) != tracker_id or ((cfg >> 16) & 0xff) != trackers
                    or ((cfg >> 8) & 0xff) < 16 or (cfg & 0xff) != first_layout['epoch']):
                    failures.append(f'Invalid initial layout tuple for tracker {tracker_id}: {cfg:#x}')
            cohort_ticks = (COHORT_DELAY_MS * ticks_per_second + 999) // 1000
            if cohort_ticks not in timeout_ticks_seen:
                failures.append('The linked owner did not request the finite cohort-deadline timeout')
        if signal_mode == 'pending' and (immediate_wakes != 1 or delayed_wakes):
            failures.append('Already-pending semaphore signal was not consumed immediately exactly once')
        if signal_mode == 'delayed' and (delayed_wakes != 1 or immediate_wakes):
            failures.append('Semaphore signal during a wait did not wake the owner exactly once')
    return {
        'trackers': trackers,
        'elapsed_ms': uptime_ms(),
        'last_tps_print_ms': last_tps_print_ms,
        'system_ticks_per_second': ticks_per_second,
        'timeout_bits': 64 if timeout_64 else 32,
        'initial_signal_mode': signal_mode,
        'first_ping_ms': ping_time_ms,
        'first_layout': first_layout,
        'semaphore_immediate_wakes': immediate_wakes,
        'semaphore_delayed_wakes': delayed_wakes,
        'semaphore_timeouts': semaphore_timeouts,
        'semaphore_no_waits': semaphore_no_waits,
        'semaphore_timeout_ticks': sorted(timeout_ticks_seen),
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
    # Independent machines preserve linked startup state for every roster size.
    results = [replay(image, trackers) for trackers in (0, 1, 16)]
    results.extend(replay(image, trackers, mode) for trackers, mode in (
        (1, 'pending'), (16, 'pending'), (1, 'delayed'), (16, 'delayed')))
    print(json.dumps({'elf': str(args.elf), 'cases': results}, indent=2))
    return int(any(result['failures'] for result in results))


if __name__ == '__main__':
    raise SystemExit(main())
