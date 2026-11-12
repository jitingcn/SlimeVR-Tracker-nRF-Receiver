#!/usr/bin/env python3
"""Model the linked nRF52840 freeze-to-arm path; compare its DMA bytes to the old policy.

Run after west build, with unicorn==2.1.2 and pyelftools. This is an instruction
and peripheral-latency estimate, not a substitute for on-device DWT measurements.
"""
import argparse
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile

import unicorn
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, UC_HOOK_MEM_WRITE, UC_HOOK_MEM_READ
from unicorn.arm_const import UC_CPU_ARM_CORTEX_M4, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3
from elftools.elf.elffile import ELFFile

if unicorn.__version__ != '2.1.2':
    sys.exit('Use unicorn==2.1.2: 2.1.4 misexecutes Thumb IT blocks in this benchmark')
here = Path(__file__).resolve().parent
root = here.parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--elf', type=Path, default=root.parent / 'build/relay-receiver/receiver/zephyr/zephyr.elf')
parser.add_argument('--seeds', type=int, default=10000)
parser.add_argument('--refill', type=int, default=3, help='extra cycles per taken branch (1-3 on Cortex-M4)')
parser.add_argument('--repairs', type=int, choices=range(6), default=5)
parser.add_argument('--max-trackers', type=int, choices=range(1,17), default=16)
parser.add_argument('--slot-ticks', type=int, choices=(15,16), default=16)
parser.add_argument('--tps-max', type=int, default=120)
parser.add_argument('--second-b',type=int,choices=(0,1,2),default=0)
parser.add_argument('--on-demand', action='store_true')
args = parser.parse_args()
if not 1 <= args.tps_max <= 32768:
    parser.error('--tps-max must be 1..32768')
fixture_flags = ['-DRELAY_BENCH_SECOND_B='+str(args.second_b),'-DRELAY_BENCH_DEMAND='+str(int(args.on_demand)),'-DRELAY_HOST_REPAIR_SLOTS='+str(args.repairs),
                 '-DRELAY_HOST_DIRECT_SLOTS='+str(args.max_trackers),
                 '-DRELAY_BENCH_MAX_MEMBERS='+str(args.max_trackers),
                 '-DRELAY_HOST_SLOT_TICKS='+str(args.slot_ticks),
                 '-DRELAY_HOST_TPS_MAX='+str(args.tps_max)]
objdump = os.environ.get('ARM_OBJDUMP', str(root.parent / 'tools/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump'))
u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
for address, size in [(0x21000000,0x20000), (0, 0x100000), (0x10000000, 0x10000), (0x20000000, 0x40000),
                      (0x40000000, 0x200000), (0x50000000, 0x100000), (0xe0000000, 0x100000)]:
    u.mem_map(address, size)


def load(path):
    with path.open('rb') as stream:
        elf = ELFFile(stream)
        for segment in elf.iter_segments():
            if segment['p_type'] == 'PT_LOAD' and segment['p_filesz']:
                u.mem_write(segment['p_vaddr'], segment.data())
        return {s.name: (s['st_value'], s['st_size'], s['st_info']['type'])
                for s in elf.get_section_by_name('.symtab').iter_symbols()}


symbols = load(args.elf)
sym = {name: value[0] for name, value in symbols.items()}
dis = subprocess.check_output([objdump, '-d', str(args.elf)], text=True)
instructions = {}
for line in dis.splitlines():
    match = re.match(r'\s*([0-9a-f]+):\s+(?:[0-9a-f]{4}\s+)+\s*(.*)', line)
    if not match:
        continue
    address, text = int(match[1], 16), match[2]
    op = text.split()[0]
    cost = 2 if op.startswith(('ldr', 'str')) else 1
    if op.startswith(('push', 'pop', 'ldm', 'stm')):
        registers = text[text.index('{') + 1:text.index('}')].split(',')
        cost = 1 + len(registers)
    if op.startswith(('dsb', 'dmb', 'isb')):
        cost = 3
    if op.startswith(('udiv', 'sdiv')):
        cost = 12
    instructions[address] = cost
functions = {}
for name, (address, size, kind) in symbols.items():
    if kind == 'STT_FUNC':
        for address in range(address & ~1, (address & ~1) + size, 2):
            functions[address] = name
stop = 0xff000
u.mem_write(stop, b'\x00\xbe')
cycles = 0
last = None
phases = {}
measuring = False
disable_ready = None
dwt_reads = []
arm_boundary = None
entry_prologue = 0


def write32(address, value):
    u.mem_write(address, struct.pack('<I', value))


def read32(address):
    return struct.unpack('<I', u.mem_read(address, 4))[0]


def hook_code(uc, address, size, _):
    global cycles, last, arm_boundary, entry_prologue
    if address == stop:
        uc.emu_stop()
        return
    if not measuring:
        return
    if address not in instructions:
        raise RuntimeError(f'Unmodeled instruction at {address:#x}')
    if (last and functions.get(last[0], '').startswith('esb_prx_broadcast_arm_event')
            and functions.get(address, '') == 'relay_freeze_arm'):
        arm_boundary = cycles
    cost = instructions[address]
    if last and last[0] + last[1] != address:
        cost += args.refill
    cycles += cost
    if last is None:
        # The old first DWT read followed the entry register save. The fixed
        # 24-cycle caller allowance already covers this prologue. Keep that
        # measurement boundary instead of counting the save twice.
        entry_prologue = cost
    last = (address, size)
    name = functions.get(address, 'unknown')
    phases[name] = phases.get(name, 0) + cost


disable_stuck = False

def hook_mmio(uc, access, address, size, value, _):
    global disable_ready
    if access == unicorn.UC_MEM_READ and address == 0xe0001004:
        write32(address, cycles)
        dwt_reads.append(cycles)
    if access == unicorn.UC_MEM_WRITE and address in (0x4001f504, 0x4001f508):
        enabled = read32(0x4001f500)
        write32(0x4001f500, enabled | value if address == 0x4001f504 else enabled & ~value)
    if access == unicorn.UC_MEM_WRITE and address == 0x40001010 and value:
        # Allow 6 us for radio disable; grant work overlaps this modeled delay.
        disable_ready = cycles + 384 if measuring else 0
    if access == unicorn.UC_MEM_READ and address == 0x40001110 and disable_ready is not None:
        if not disable_stuck and cycles >= disable_ready:
            write32(address, 1)
            disable_ready = None


# Keep hooks installed over every device function from initialization onward.
# Pure fixture code lives at 0x80000 and cannot execute in the measured path.
# Excluding only that disjoint fixture region avoids Python dispatch on the
# expensive unmeasured planner without translating device code uninstrumented.
for name,(address,size,kind) in symbols.items():
    if kind=='STT_FUNC':
        assert (address & ~1)<0x80000 or 0x20000000 <= (address & ~1)<0x20040000, (name,hex(address))
u.hook_add(UC_HOOK_CODE, hook_code, begin=0, end=0x7ffff)
u.hook_add(UC_HOOK_CODE, hook_code, begin=0x20000000, end=0x2003ffff)
u.hook_add(UC_HOOK_MEM_WRITE | UC_HOOK_MEM_READ, hook_mmio, begin=0x40000000, end=0x401fffff)
u.hook_add(UC_HOOK_MEM_READ, hook_mmio, begin=0xe0001004, end=0xe0001007)


def call(name, *values, measure=False):
    global measuring, cycles, last, phases, dwt_reads, arm_boundary
    arm_boundary = None
    measuring = measure
    cycles, last, phases, dwt_reads = 0, None, {}, []
    address = sym.get(name)
    if address is None:
        address = next(value for key, value in sym.items() if key.startswith(name + '.constprop'))
    u.reg_write(UC_ARM_REG_SP, 0x2003f000)
    u.reg_write(UC_ARM_REG_LR, stop | 1)
    for register, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), values):
        u.reg_write(register, value)
    u.emu_start(address | 1, stop, count=100000 if measure else 2000000)
    if u.reg_read(UC_ARM_REG_PC) != stop:
        raise RuntimeError(f'{name}{values} failed to return at {u.reg_read(UC_ARM_REG_PC):#x}')
    return u.reg_read(UC_ARM_REG_R0)


# Real linked MPSL FEM configuration, initialized as the PRX receive path does.
write32(0x10000130, 8)
call('mpsl_fem_api_init')
call('gppi_init')
call('mpsl_fem_init.lto_priv.0')
u.mem_write(sym['esb_cfg.lto_priv.0'], b'\x01\x01' + bytes(26))  # DPL, PRX
u.mem_write(sym['esb_cfg.lto_priv.0'] + 23, b'\x01')  # fast ramp
u.mem_write(sym['esb_cfg.lto_priv.0'] + 14, b'\x1e')  # 0 dBm enumeration
u.mem_write(sym['esb_state.lto_priv.0'], b'\x06')  # PRX
u.mem_write(sym['esb_timer.lto_priv.0'] + 12, b'\x01')  # nrfx initialized
write32(sym['disable_event.lto_priv.0'] + 4, 0x40001110)
write32(0x4000a510, 4)  # TIMER2, 1 MHz
# ESB init allocates six distinct PPI channels; allocation is outside the timed path.
for channel, name in enumerate(('egu_ramp_up', 'disabled_egu', 'egu_timer_start',
                                'radio_address_timer_stop', 'timer_compare0_radio_disable',
                                'timer_compare1_radio_txen'), 8):
    u.mem_write(sym[name + '.lto_priv.0'], bytes([channel]))
call('esb_fem_for_rx_set')
u.mem_write(sym['relay_profile.lto_priv.0'], b'\x01')
# Keep the synthetic coordinator fixture outside all linked device RAM.
# Ledger images extend beyond the historical fixed 0x20028000 placement.
assert sym['_image_ram_end'] < 0x2003c000, 'Device overlaps model scratch'
fixture_ram = 0x21000000
initial_ram = bytes(u.mem_read(0x20000000,sym['_image_ram_end']-0x20000000))
initial_peripherals = bytes(u.mem_read(0x40000000, 0x20000))
with tempfile.TemporaryDirectory(prefix='relay-arm-model-') as temporary:
    temporary = Path(temporary)
    fixture = temporary / 'fixture.elf'
    environment = dict(os.environ, BENCH_SECOND_B=str(args.second_b), BENCH_DEMAND=str(int(args.on_demand)), BENCH_EXPORT=str(fixture), BENCH_TEXT_ORIGIN='0x80000', BENCH_RAM_ORIGIN=hex(fixture_ram), BENCH_REPAIR_SLOTS=str(args.repairs),
                       BENCH_MAX_MEMBERS=str(args.max_trackers), BENCH_SLOT_TICKS=str(args.slot_ticks),
                       BENCH_TPS_MAX=str(args.tps_max))
    subprocess.run([sys.executable, str(here / 'bench_cycles.py')], env=environment, check=True)
    with fixture.open('rb') as stream:
        fixture_elf = ELFFile(stream)
        for segment in fixture_elf.iter_segments():
            address, size = segment['p_vaddr'], segment['p_memsz']
            if segment['p_type'] == 'PT_LOAD' and size and address >= 0x20000000:
                assert fixture_ram <= address and address + size <= fixture_ram+0x1c000, 'Fixture overlaps model scratch'
    print(f'Device RAM ends at {sym["_image_ram_end"]:#x}; fixture RAM starts at {fixture_ram:#x}')
    bench = {name: value[0] for name, value in load(fixture).items()}
    sym['setup'] = bench['setup']
    golden_c = temporary / 'golden.c'
    golden_c.write_text('''#include <stdio.h>
#include "bench_cycles.c"
#define RC_BIT(id) (1U << (id))
#include "grants_reference.inc"
int main(int argc, char **argv) {
    unsigned n = (unsigned)strtoul(argv[1], NULL, 10);
    for (unsigned seed = 0; seed < n; ++seed) {
        setup(seed);
        for (unsigned age = 0; age < 3; ++age) c.missing[age] = c.expected;
        if (RELAY_BENCH_DEMAND) timed(); else reference_build_grants(&rc, &c, 5000, &plan);
        if (!RELAY_BENCH_DEMAND) relay_control_encode(payload, &c);
        if (fwrite(payload, sizeof(payload), 1, stdout) != 1) return 1;
    }
    return 0;
}
'''.replace('#include <stdio.h>', '#include <stdio.h>\n#include <stdlib.h>'))
    golden_exe = temporary / 'golden'
    subprocess.run([os.environ.get('CC', 'cc'), '-O2', *fixture_flags, '-Wno-unknown-attributes', '-I', str(here), '-I', str(root / 'src/connection'), str(golden_c), str(root / 'src/connection/relay_coord.c'), '-o', str(golden_exe)], check=True)
    golden = subprocess.check_output([str(golden_exe), str(args.seeds)])
    maximum, worst_seed, worst_phases = 0, 0, {}
    repeat_arm_max=repeat_encode_max=repeat_post_end_max=0
    arm_max, encode_max = 0, 0
    member_maxima = {}
    payload, work, event = 0x2003c000, 0x2003c100, 0x40011144
    for seed in range(args.seeds):
        u.mem_write(0x20000000, initial_ram)
        u.mem_write(0x40000000, initial_peripherals)
        disable_ready = None
        call('setup', seed)
        if args.on_demand:
            u.mem_write(sym['relay_demand_state.lto_priv.0'], bytes(u.mem_read(bench['demand_state'], 16)))
            u.mem_write(sym['relay_applied_settings.lto_priv.0'], bytes(u.mem_read(bench['demand_settings'], 10)))

        u.mem_write(payload, bytes([72, 1, 0, 1, 0]) + bytes(u.mem_read(bench['payload'], 72)))
        wire = call('esb_prx_broadcast_prepare', payload)
        assert wire
        u.mem_write(work, struct.pack('<III4sI', bench['c'], bench['plan'], wire, bytes(u.mem_read(bench['crc'], 4)), event))
        result = call('relay_freeze_arm', bench['rc'], work, measure=True)
        assert result == 0, (seed, result)
        if dwt_reads:
            assert len(dwt_reads) in (3, 4), dwt_reads
            arm_max = max(arm_max, dwt_reads[1] - dwt_reads[0])
            encode_max = max(encode_max, dwt_reads[-1] - dwt_reads[1])
        else:
            # Production has no MMIO probes. Count the full entry/prologue
            # through arm_event return, then through helper return.
            assert arm_boundary is not None
            arm_max = max(arm_max, arm_boundary - entry_prologue)
            encode_max = max(encode_max, cycles - arm_boundary)
        minimum = min(11, args.max_trackers)
        present = minimum + seed % (args.max_trackers - minimum + 1)  # bench_cycles.c setup: includes 16 present.
        case_arm = dwt_reads[1]-dwt_reads[0] if dwt_reads else arm_boundary-entry_prologue
        case_encode = dwt_reads[-1]-dwt_reads[1] if dwt_reads else cycles-arm_boundary
        previous = member_maxima.setdefault(present, [0, 0, 0])
        previous[0] = max(previous[0], case_arm + 24)
        previous[1] = max(previous[1], case_encode + 24)
        previous[2] += 1
        assert read32(sym['radio_operation_busy.lto_priv.0']) == 1
        actual = bytes(u.mem_read(wire, 72))
        check_crc=7
        for byte in actual[:71]:
            check_crc ^= byte
            for _ in range(8):check_crc=((check_crc<<1) ^ (7 if check_crc&128 else 0))&255
        assert actual[71]==check_crc, (seed,'independent wire CRC',actual[71],check_crc)
        expected = golden[seed * 72:(seed + 1) * 72]
        assert actual == expected, (seed, [(i, a, b) for i, (a, b) in enumerate(zip(actual, expected)) if a != b])
        assert read32(0x4000a544) == 21  # unchanged TX offset, fast ramp = 40 us
        assert read32(0x4000a548) == 59  # MPSL PA enable 2 us before TX ramp ends
        # TIMER START must be driven by the scheduled RTC event, not software.
        assert read32(0x4000a000) == 0
        enabled = read32(0x4001f500)
        endpoints = [(read32(0x4001f510 + channel * 8), read32(0x4001f514 + channel * 8))
                     for channel in range(20) if enabled & (1 << channel)]
        assert (event, 0x4000a000) in endpoints
        assert (0x4000a144, 0x40001000) in endpoints, (hex(enabled), endpoints)
        if cycles > maximum:
            maximum, worst_seed, worst_phases = cycles, seed, dict(phases)
        if args.second_b:
            # The copy is built before its own freeze. Measure that work and
            # the actual repeat arm entry separately against the same ceilings.
            source = 0x2003c200
            u.mem_write(source, actual)
            at = int.from_bytes(actual[12:16],'big') + ((args.repairs+1)*args.slot_ticks if args.second_b==2 else 2*args.slot_ticks)
            # Execute the real copy-1 DISABLED/PRX restoration,
            # then encoding/preparation; caller/wait-return cost is reserved below.
            # No assumption that this work happens
            # before the first packet END is allowed in the duplicate deadline.
            call('on_radio_disabled_broadcast.lto_priv.0', measure=True)
            post_end_cycles = cycles + 24
            write32(sym['radio_operation_busy.lto_priv.0'], 0)
            call('relay_repeat_encode', payload+5, source, at & 0xffffffff, measure=True)
            post_end_cycles += cycles + 24
            repeat_encode_max=max(repeat_encode_max,cycles+24)
            repeat=bytes(u.mem_read(payload+5,72))
            assert repeat[0]==0xec and repeat[1:12]==actual[1:12] and repeat[16:71]==actual[16:71]
            check=7
            for byte in repeat[:71]:
                check ^= byte
                for _ in range(8):check=((check<<1) ^ (7 if check&128 else 0))&255
            assert repeat[71]==check
            # Model completion of copy 1's bounded PRX restoration, then
            # prepare copy 2 through the same linked SDK entry points.
            disable_ready=None
            u.mem_write(payload,bytes([72,1,0,1,0])+repeat)
            assert call('esb_prx_broadcast_prepare',payload,measure=True)
            post_end_cycles += cycles + 24
            # Keep a separate 128us allowance for IRQ/thread dispatch and RTC
            # exact_set callers. This is pipeline slack, not an arm/encode limit.
            post_end_cycles += 8192
            callback_ticks = ((args.repairs+1)*args.slot_ticks if args.second_b==2 else 2*args.slot_ticks)-2-3
            slack_cycles = (callback_ticks-11)*64000000//32768
            repeat_post_end_max=max(repeat_post_end_max,post_end_cycles)
            assert post_end_cycles < slack_cycles, (seed,'B2 callback already due',post_end_cycles,slack_cycles)
            generation=read32(sym['relay_layout_generation.lto_priv.0'])
            assert call('relay_repeat_arm',event,generation+1) & 0x80000000
            assert read32(sym['radio_operation_busy.lto_priv.0'])==0
            assert call('relay_repeat_arm',event,generation,measure=True)==0
            repeat_arm_max=max(repeat_arm_max,cycles+24)
    if args.second_b:
        print('second B arm/encode:',repeat_arm_max,repeat_encode_max)
        print('B1 END to prepared B2, including 128us dispatch/caller allowance:',repeat_post_end_max,'cycles; callback slack',slack_cycles,'cycles')
        assert repeat_arm_max<=1130 and repeat_encode_max<=805
    # Failures must not steal another owner's radio, or leak our ownership.
    for failure in ('stale', 'busy', 'not_prx', 'late', 'reinit', 'disabled_stuck', 'disabled_late'):
        u.mem_write(0x20000000, initial_ram)
        u.mem_write(0x40000000, initial_peripherals)
        disable_ready = None
        call('setup', 1)
        u.mem_write(payload, bytes([72, 1, 0, 1, 0]) + bytes(u.mem_read(bench['payload'], 72)))
        wire = call('esb_prx_broadcast_prepare', payload)
        u.mem_write(work, struct.pack('<III4sI', bench['c'], bench['plan'], wire,
                                     bytes(u.mem_read(bench['crc'], 4)), event))
        busy = sym['radio_operation_busy.lto_priv.0']
        disable_stuck = failure.startswith('disabled_')
        if failure == 'reinit':
            u.mem_write(sym['broadcast_length.lto_priv.0'], b'\0')
        elif failure == 'disabled_stuck':
            pass
        elif failure == 'disabled_late':
            write32(event, 1)
        elif failure == 'stale':
            u.mem_write(sym['relay_epoch.lto_priv.0'], b'\x01')
        elif failure == 'busy':
            write32(busy, 1)
        elif failure == 'not_prx':
            u.mem_write(sym['esb_state.lto_priv.0'], b'\x00')
        else:
            write32(event, 1)
        result = call('relay_freeze_arm', bench['rc'], work)
        assert result & 0x80000000, (failure, result)
        assert read32(busy) == (failure == 'busy'), failure
        if disable_stuck:
            assert read32(sym['esb_state.lto_priv.0']) & 255 == 0, failure
        disable_stuck = False
    print(f'Runtime model R={args.repairs}, max members={args.max_trackers}, width={args.slot_ticks}, cap={args.tps_max}')
    print('stale, busy, non-PRX, reinit, missed deadline and stuck DISABLED ownership checks passed')
    print(f'{args.seeds} linked-firmware DMA payloads match the original policy')
    print(f'freeze-arm helper: max {maximum} modeled cycles, seed {worst_seed}, branch refill +{args.refill}')
    print('member counts: arm, encode, cases:', dict(sorted(member_maxima.items())))
    print('phase cycles:', dict(sorted(worst_phases.items(), key=lambda pair: -pair[1])))

    # The launch edge and the payload-readiness edge are separate deadlines.
    # Keep the original 80% arm budget, and finish encoding within the existing
    # 21 us TIMER delay, before TXEN and its additional 40 us ramp to START.
    print(f'arm: {arm_max + 24} modeled cycles including caller allowance')
    print(f'encode after arm: {encode_max + 24} modeled cycles including allowance')
    assert arm_max + 24 <= 1562, '80% arm cycle budget exceeded'
    assert encode_max + 24 <= 21 * 64, 'payload not ready before TXEN'

    assert arm_max + 24 <= 1130, 'modeled arm baseline exceeded'
    assert encode_max + 24 <= 805, 'modeled encode baseline exceeded'
