#!/usr/bin/env python3
"""Optional Thumb instruction model. Requires unicorn and pyelftools.

Estimates core cycles, not flash stalls, peripheral latency or a WCET proof.
Builds the real policy and encoder with the firmware compiler's -Os default;
function-specific optimization attributes remain active.
"""
from pathlib import Path
import os
import tempfile
from unicorn import *
import unicorn
if unicorn.__version__ != "2.1.2":
 raise SystemExit("Use unicorn==2.1.2: 2.1.4 misexecutes Thumb IT blocks in this benchmark")
from unicorn.arm_const import *
from elftools.elf.elffile import ELFFile
import subprocess,re
here = Path(__file__).resolve().parent
root = here.parents[2]
cc = os.environ.get('ARM_CC', str(root.parent / 'tools/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc'))
objdump = cc.removesuffix('gcc') + 'objdump'
tmp = tempfile.TemporaryDirectory()
path = str(Path(tmp.name) / 'bench.elf')
ld = Path(tmp.name) / 'bench.ld'
ld.write_text("MEMORY { bench_flash(rx): ORIGIN = " + os.environ.get("BENCH_TEXT_ORIGIN", "0x10000") + ", LENGTH = 512K\n"
              "bench_ram(rwx): ORIGIN = " + os.environ.get("BENCH_RAM_ORIGIN", "0x20000000") + ", LENGTH = 128K }\n"
              "SECTIONS { .text : { *(.text*) *(.rodata*) } > bench_flash\n"
              ".data : { *(.data*) } > bench_ram\n"
              ".bss : { *(.bss*) *(COMMON) } > bench_ram }\n")
subprocess.run([cc, '-Os', '-g', '-mcpu=cortex-m4', '-mthumb', '-ffunction-sections',
 '-fdata-sections', '-fno-builtin', '-DRELAY_BENCH_FREESTANDING', '-DRELAY_BENCH_SECOND_B='+os.environ.get('BENCH_SECOND_B','0'), '-DRELAY_BENCH_DEMAND='+os.environ.get('BENCH_DEMAND','0'), '-DRELAY_HOST_REPAIR_SLOTS=' + os.environ.get('BENCH_REPAIR_SLOTS','5'), '-DRELAY_HOST_DIRECT_SLOTS=' + os.environ.get('BENCH_MAX_MEMBERS','16'), '-DRELAY_BENCH_MAX_MEMBERS=' + os.environ.get('BENCH_MAX_MEMBERS','16'), '-DRELAY_HOST_SLOT_TICKS=' + os.environ.get('BENCH_SLOT_TICKS','16'), '-DRELAY_HOST_TPS_MAX=' + os.environ.get('BENCH_TPS_MAX','120'), '-I', str(root / 'src/connection'), str(here / 'bench_cycles.c'),
 str(root / 'src/connection/relay_coord.c'),
 '-Wl,--gc-sections,-e,setup,-u,timed', '-T', str(ld), '-nostdlib', '-lgcc', '-o', path], check=True)
if os.environ.get('BENCH_EXPORT'):
 import shutil
 shutil.copyfile(path, os.environ['BENCH_EXPORT'])
 exit()
f=open(path,'rb');elf=ELFFile(f)
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
u.mem_map(0,0x100000);u.mem_map(0x20000000,0x40000)
for seg in elf.iter_segments():
 if seg['p_type']=='PT_LOAD' and seg['p_filesz']:
  address = seg['p_vaddr']
  if not (address < 0x100000 or 0x20000000 <= address < 0x20040000):
   u.mem_map(address & ~4095, (seg['p_memsz'] + (address & 4095) + 4095) & ~4095)
  u.mem_write(address,seg.data())
sym={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
stop=0x80000;u.mem_write(stop,b'\x00\xbe')
dis=subprocess.check_output([objdump,'-d',path],text=True)
if os.environ.get('BENCH_DUMP'): Path(os.environ['BENCH_DUMP']).write_text(dis)
ins={int(m[1],16):m[2] for l in dis.splitlines() if (m:=re.match(r'\s*([0-9a-f]+):\s+(?:[0-9a-f]{4}\s+)+\s*(.*)',l))}
count=cycles=0
last=None
phase_cycles={}
functions = [(s['st_value'] & ~1, (s['st_value'] & ~1)+s['st_size'], s.name) for s in elf.get_section_by_name('.symtab').iter_symbols() if s['st_info']['type'] == 'STT_FUNC']
def hook(u,a,size,data):
 global count,cycles,last
 if a==stop:u.emu_stop();return
 count+=1
 text=ins.get(a,'');op=text.split()[0] if text else ''
 cost=1
 if op.startswith(('ldr','str')):cost=2
 if op.startswith(('push','pop','ldm','stm')):cost=1+max(1,text.count(',')+1)
 if last and last[0]+last[1]!=a: cost+=2
 for lo,hi,name in functions:
  if lo<=a<hi:phase_cycles[name]=phase_cycles.get(name,0)+cost;break
 cycles+=cost;last=(a,size)
h=u.hook_add(UC_HOOK_CODE,hook)
def call(name,arg=0):
 u.reg_write(UC_ARM_REG_SP,0x2003f000);u.reg_write(UC_ARM_REG_LR,stop|1);u.reg_write(UC_ARM_REG_R0,arg)
 u.emu_start(sym[name]|1,stop+2,count=1000000)
maximum=(0,0,0)
# One cycle per ALU instruction, two for loads/stores, two extra per taken
# branch, and register-count costs for multiple transfers. No stall model.
for seed in range(int(os.environ.get("BENCH_SEEDS", "10000"))):
 call('setup',seed);count=cycles=0;last=None;phase_cycles.clear();call('timed')
 if cycles>maximum[0]:maximum=(cycles,count,seed); worst_phases=dict(phase_cycles)
print('estimated cycles, instructions, seed:',maximum)

print(worst_phases)
