#!/usr/bin/env python3
"""Replay linked ESB startup through roster restore, NVS and startup logging.

    uv run --with unicorn --with pyelftools run_startup_stack.py PATH/zephyr.elf

Imports the existing stats replay's ELF/DWARF loader. Executes real NVS mount
on a separate init stack, then real z_thread_entry -> esb_thread -> roster/NVS,
including the nRF flash READ driver and libc. Stops at radio initialization,
after roster publication, RF-channel read and startup logs, before RF hardware.
Fixtures are real NVS data/ATE bytes, not mocked nvs_read return values.

Only clock startup, OS mutexes, uptime, TLS, log allocation/finalization, flash
page geometry and flash WRITE/ERASE device leaves are modeled. No NVS/roster
function is replaced. Measured depth is a LOWER BOUND; replaced leaf frames,
preemption/context switching, full boot and RF operation are not covered.
Separately require 32 bytes for the Cortex-M basic exception frame (R0-R3,
R12, LR, PC, xPSR). This is a minimum, not an FPU/alignment/scheduler margin or
hardware stack-safety proof. Guard bytes are excluded by the linked budget.
"""

import argparse
from collections import Counter
import json
from pathlib import Path
import struct

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UcError, UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import (
    UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1,
    UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP,
)

from run_stack import load_image, SCRATCH_BASE, SCRATCH_SIZE

INSTRUCTION_LIMIT = 2_000_000
EXCEPTION_RESERVE = 32
PAGE_SIZE = 4096
SECTORS = 4
ROSTER_ID = 260
STOP = SCRATCH_BASE + 0x100
INIT_TOP = SCRATCH_BASE + SCRATCH_SIZE
TLS = SCRATCH_BASE + 0x200
READBACK = SCRATCH_BASE + 0x300


def structure_members(path, name):
    with path.open('rb') as stream:
        for unit in ELFFile(stream).get_dwarf_info().iter_CUs():
            for die in unit.iter_DIEs():
                attr = die.attributes.get('DW_AT_name')
                if die.tag != 'DW_TAG_structure_type' or not attr or attr.value != name.encode():
                    continue
                members = {}
                for member in die.iter_children():
                    attr = member.attributes.get('DW_AT_name')
                    offset = member.attributes.get('DW_AT_data_member_location')
                    if attr and offset and isinstance(offset.value, int):
                        members[attr.value.decode()] = offset.value
                if members:
                    return members
    raise ValueError(f'Missing DWARF layout for {name}')


def crc8(data):
    value = 0xff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (0x07 if value & 0x80 else 0)) & 0xff
    return value


def fixture(records):
    """One open sector, packed data upward and CRC-valid ATEs downward."""
    flash = bytearray(b'\xff' * (PAGE_SIZE * SECTORS))
    data_at = 0
    ate_at = PAGE_SIZE - 16  # final eight bytes reserved for the close ATE
    # A GC-done marker is normal on a freshly initialized NVS sector.
    for key, data in [(0xffff, b''), *records]:
        flash[data_at:data_at + len(data)] = data
        ate = struct.pack('<HHHB', key, data_at, len(data), 0xff)
        flash[ate_at:ate_at + 8] = ate + bytes([crc8(ate)])
        data_at += (len(data) + 3) & ~3
        ate_at -= 8
    return bytes(flash)


def replay(path, image, scenario, count):
    symbols, functions, sections, offsets = image
    geometry = structure_members(path, 'flash_pages_info')
    machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    for start in (0, 0x10000000, 0x20000000, 0x40000000, 0xe0000000, SCRATCH_BASE):
        machine.mem_map(start, 0x100000)
    for address, data in sections:
        machine.mem_write(address, data)
    # nRF52 FICR CODEPAGESIZE/CODESIZE; the linked read driver still validates
    # offsets and executes memcpy rather than being intercepted by the model.
    machine.mem_write(0x10000010, struct.pack('<II', PAGE_SIZE, 256))

    def symbol(name):
        matches = {value for key, value in symbols.items()
                   if key == name or key.startswith(name + '.')}
        if len(matches) != 1:
            raise ValueError(f'Ambiguous/missing ELF symbol {name}: {matches}')
        return matches.pop()

    def word(address):
        return struct.unpack('<I', machine.mem_read(address, 4))[0]

    def putword(address, value):
        machine.mem_write(address, struct.pack('<I', value))

    addresses = [0x123456780001 + i for i in range(count)]
    roster = bytes([1, count]) + b''.join(addr.to_bytes(6, 'little') for addr in addresses)
    roster += bytes(6 * (16 - count))
    records = [(ROSTER_ID, roster)] if scenario == 'valid' else (
        [(1, bytes([count]))] + [(3 + i, struct.pack('<Q', addr))
                               for i, addr in enumerate(addresses)]
        if scenario == 'legacy' else [])
    # Persist an encoded channel too, exercising the second startup NVS read.
    records.append((259, b'\x00'))
    flash_bytes = fixture(records)
    stack, storage = symbol('_k_thread_stack_esb_thread_id')
    thread = symbol('_k_thread_data_esb_thread_id')[0]
    budget = word(thread + offsets['init_stack_size'])
    entry = word(thread + offsets['init_entry'])
    if word(thread + offsets['init_stack']) != stack or entry != symbol('esb_thread')[0]:
        raise ValueError('Linked ESB thread initializer does not match its stack/entry')
    if not 0 < budget <= storage:
        raise ValueError('Invalid linked stack budget')
    top = stack + storage
    lowest = top
    phase = 'mount'
    calls = Counter()
    modeled = Counter()
    recent = []
    deepest = []
    instructions = 0
    allocated = SCRATCH_BASE + 0x1000
    flash_offset = None
    writes = 0
    reached_radio = False

    def ret(value=0):
        machine.reg_write(UC_ARM_REG_R0, value & 0xffffffff)
        machine.reg_write(UC_ARM_REG_R1, 0)
        machine.reg_write(UC_ARM_REG_PC, machine.reg_read(UC_ARM_REG_LR))

    def hook(cpu, pc, size, unused):
        nonlocal lowest, deepest, instructions, allocated, flash_offset, writes, reached_radio
        instructions += 1
        if phase == 'startup':
            sp = cpu.reg_read(UC_ARM_REG_SP)
            if sp < lowest:
                lowest, deepest = sp, recent[-16:]
        name = functions.get(pc)
        if not name:
            return
        base = name.split('.')[0]
        calls[f'{phase}:{base}'] += 1
        recent.append(name)
        del recent[:-24]
        if any(text in base for text in ('assert', 'fatal', 'fault')):
            raise RuntimeError(f'Unexpected firmware failure: {name}')
        if base == 'esb_initialize_internal' and phase == 'startup':
            reached_radio = True
            cpu.emu_stop()
            return
        if base == 'flash_get_page_info':
            offset, info = cpu.reg_read(UC_ARM_REG_R1), cpu.reg_read(UC_ARM_REG_R2)
            if flash_offset is None:
                flash_offset = offset
                cpu.mem_write(offset, flash_bytes)
            if not flash_offset <= offset < flash_offset + len(flash_bytes):
                raise RuntimeError(f'Unexpected flash geometry offset {offset:#x}')
            for field, value in [('start_offset', offset // PAGE_SIZE * PAGE_SIZE),
                                 ('size', PAGE_SIZE), ('index', offset // PAGE_SIZE)]:
                putword(info + geometry[field], value)
            ret()
        elif base in ('flash_nrf_write', 'flash_nrf_erase'):
            offset = cpu.reg_read(UC_ARM_REG_R1)
            length = cpu.reg_read(UC_ARM_REG_R3 if base == 'flash_nrf_write' else UC_ARM_REG_R2)
            if flash_offset is None or not flash_offset <= offset <= offset + length <= flash_offset + len(flash_bytes):
                raise RuntimeError('Flash operation outside the NVS partition')
            if base == 'flash_nrf_write':
                data = bytes(cpu.mem_read(cpu.reg_read(UC_ARM_REG_R2), length))
                old = bytes(cpu.mem_read(offset, length))
                if any((a & b) != b for a, b in zip(old, data)):
                    raise RuntimeError('NVS attempted a zero-to-one flash write')
                cpu.mem_write(offset, data)
                if phase == 'startup':
                    writes += 1
            else:
                cpu.mem_write(offset, b'\xff' * length)
            ret()
        elif base in ('clocks_start', 'k_mutex_init', 'k_mutex_lock', 'k_mutex_unlock',
                      'z_impl_k_mutex_lock', 'z_impl_k_mutex_unlock',
                      'k_sched_lock', 'k_sched_unlock'):
            ret()
        elif base.startswith('k_uptime_get'):
            ret(100)
        elif base == '__aeabi_read_tp':
            ret(TLS)
        elif base == 'z_log_msg_alloc':
            length = cpu.reg_read(UC_ARM_REG_R0) * 4
            if not length or allocated + length > INIT_TOP - 0x10000:
                raise RuntimeError('Unexpected log allocation size')
            result = allocated
            allocated += (length + 7) & ~7
            ret(result)
        elif base == 'z_log_msg_finalize':
            ret()
        else:
            return
        modeled[f'{phase}:{base}'] += 1

    machine.hook_add(UC_HOOK_CODE, hook)

    def run(function, args, sp):
        machine.reg_write(UC_ARM_REG_SP, sp)
        machine.reg_write(UC_ARM_REG_LR, STOP | 1)
        for reg, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), args):
            machine.reg_write(reg, value)
        machine.emu_start(function, STOP, count=INSTRUCTION_LIMIT)

    failures = []
    try:
        # APPLICATION SYS_INIT normally mounts NVS before threads run. Execute
        # that initializer rather than inventing fs/cache state or bypassing it.
        run(symbol('sys_nvs_init')[0], (0, 0, 0, 0), INIT_TOP)
        if machine.reg_read(UC_ARM_REG_PC) != STOP or machine.reg_read(UC_ARM_REG_R0) != 0:
            raise RuntimeError('Real NVS mount did not return success')
        phase = 'startup'
        run(symbol('z_thread_entry')[0], (entry, 0, 0, 0), top)
        if not reached_radio:
            raise RuntimeError('ESB startup did not reach radio initialization')
        actual_count = machine.mem_read(symbol('stored_trackers')[0], 1)[0]
        actual_addresses = list(struct.unpack('<16Q', machine.mem_read(symbol('stored_tracker_addr')[0], 128)))
        if actual_count != count or actual_addresses != addresses + [0] * (16 - count):
            raise RuntimeError(f'Wrong published roster: {actual_count}, {actual_addresses}')
        if machine.mem_read(symbol('tracker_store_ready')[0], 1)[0] != 1:
            raise RuntimeError('Roster was not marked ready')
        # LTO may inline nvs_read_hist into sys_read_checked. Require the read
        # boundary, real ATE walk and flash access, not a separate call frame.
        for required in ('tracker_store_restore', 'sys_read_checked', 'nvs_prev_ate', 'flash_nrf_read'):
            if not calls[f'startup:{required}']:
                raise RuntimeError(f'Required real linked path not executed: {required}')
        if scenario != 'valid' and not writes:
            raise RuntimeError('Migration did not write a transactional roster')
        # Read the committed result back through linked NVS, independently of
        # the published RAM roster. Exclude this separate caller from depth.
        phase = 'readback'
        run(symbol('sys_read_checked')[0], (ROSTER_ID, READBACK, len(roster), 0), INIT_TOP)
        if (machine.reg_read(UC_ARM_REG_PC) != STOP
                or machine.reg_read(UC_ARM_REG_R0) != len(roster)
                or bytes(machine.mem_read(READBACK, len(roster))) != roster):
            raise RuntimeError('NVS roster readback differs from the expected committed record')
    except (UcError, RuntimeError, ValueError) as exc:
        failures.append(str(exc))
    used = top - lowest
    if used > budget:
        failures.append(f'Measured stack {used} exceeds linked budget {budget}')
    if used + EXCEPTION_RESERVE > budget:
        failures.append(f'Measured stack {used} plus basic exception frame {EXCEPTION_RESERVE} exceeds linked budget {budget}')
    return {
        'scenario': scenario, 'trackers': count, 'reached_radio_initialization': reached_radio,
        'linked_stack_budget': budget, 'stack_storage_bytes': storage,
        'measured_thread_stack_lower_bound': used,
        'basic_exception_frame_reserve': EXCEPTION_RESERVE,
        'headroom_after_basic_exception_frame': budget - used - EXCEPTION_RESERVE,
        'startup_flash_writes': writes, 'executed_functions': dict(calls),
        'modeled_leaves': dict(modeled), 'instructions': instructions,
        'deepest_recent_functions': deepest, 'failures': failures,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=Path, help='Unstripped nRF52 receiver ELF with DWARF')
    args = parser.parse_args()
    image = load_image(args.elf)
    results = [replay(args.elf, image, scenario, count) for scenario, count in
               [('valid', 1), ('valid', 16), ('absent', 0), ('legacy', 16)]]
    print(json.dumps({'elf': str(args.elf), 'cases': results}, indent=2))
    return int(any(result['failures'] for result in results))


if __name__ == '__main__':
    raise SystemExit(main())
