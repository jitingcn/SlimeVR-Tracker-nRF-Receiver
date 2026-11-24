#!/usr/bin/env python3
"""Verify the receiver self-OTA RAM engine blob in a linked Zephyr ELF.

Proves that once copied to RAM, every instruction and literal inside
`.ota_ram_engine` is self-contained and position-independent:
  1. Entry symbol (`ota_ram_engine_entry`) sits at offset 0 of `.ota_ram_engine`.
  2. Every branch / call target stays strictly inside `[blob_start, blob_end)`.
  3. No indirect branch/call (`blx rX`, `bx rX` other than `bx lr`).
  4. Every PC-relative load/address reads strictly inside `[blob_start, blob_end)`.
  5. No literal pool word points into flash (`[0x00000000, 0x00100000)`) or
     static Zephyr SRAM (`[0x20000000, 0x20080000)`).
  6. Section is 4-byte aligned, <= 4096 bytes, and lives in flash (not `.ramfunc`).
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

SECTION_NAME = ".ota_ram_engine"
ENTRY_SYMBOL = "ota_ram_engine_entry"
MAX_BLOB_SIZE = 4096

# Allowed literal pool constant ranges (peripherals, SCS/DWT/MPU/SCB, and known
# algorithmic constants). Any literal in flash or SRAM fails verification.
FLASH_RANGE = (0x00000000, 0x00100000)
SRAM_RANGE = (0x20000000, 0x20080000)
PERIPH_RANGE = (0x40000000, 0x50000000)
CORE_RANGE = (0xE0000000, 0xE0100000)
KNOWN_CONSTANTS = {
    0xEDB88320,  # CRC32 polynomial
    0x6E524635,  # WDT RR reload key
    0x05FA0004,  # SCB->AIRCR VECTKEY | SYSRESETREQ
    0x05FA0000,  # SCB->AIRCR VECTKEY
    0x96F3B83D,  # MCUboot image magic
    0x39387000,  # 15s @ 64MHz (IDLE_TIMEOUT)
    0x39386FFF,  # 15s - 1 @ 64MHz
    0x0030D400,  # 50ms @ 64MHz
    0x0030D3FF,  # 50ms - 1 @ 64MHz
    0x00138800,  # 20ms @ 64MHz
    0x001387FF,  # 20ms - 1 @ 64MHz
    0x0009C400,  # 10ms @ 64MHz
    0x0009C3FF,  # 10ms - 1 @ 64MHz
    0x0004E200,  # 5ms @ 64MHz
    0x0004E1FF,  # 5ms - 1 @ 64MHz
}


def run(cmd: list[str]) -> str:
    res = subprocess.run(cmd, check=True, capture_output=True, text=True)
    return res.stdout


def parse_section_bounds(readelf_out: str) -> tuple[int, int] | None:
    # Example: [18] .ota_ram_engine PROGBITS 00023c00 023c00 0005e4 00 AX 0 0 4
    pat = re.compile(
        r"\[\s*\d+\]\s+" + re.escape(SECTION_NAME) + r"\s+\S+\s+([0-9a-fA-F]+)\s+[0-9a-fA-F]+\s+([0-9a-fA-F]+)"
    )
    for line in readelf_out.splitlines():
        m = pat.search(line)
        if m:
            addr = int(m.group(1), 16)
            size = int(m.group(2), 16)
            return addr, addr + size
    return None


def parse_symbols(nm_out: str) -> dict[str, int]:
    syms: dict[str, int] = {}
    for line in nm_out.splitlines():
        parts = line.split()
        if len(parts) >= 3:
            try:
                syms[parts[2]] = int(parts[0], 16)
            except ValueError:
                pass
    return syms


def check_elf(elf: Path, tool_prefix: str, verbose: bool = False) -> bool:
    readelf_out = run([f"{tool_prefix}readelf", "-S", "-W", str(elf)])
    bounds = parse_section_bounds(readelf_out)
    if bounds is None:
        print(f"FAIL: section {SECTION_NAME} not found in {elf}", file=sys.stderr)
        return False

    blob_start, blob_end = bounds
    blob_size = blob_end - blob_start

    errors: list[str] = []
    if blob_size == 0 or (blob_size & 3) != 0 or (blob_start & 3) != 0:
        errors.append(f"invalid blob alignment/size: start=0x{blob_start:08x} size={blob_size}")
    if blob_size > MAX_BLOB_SIZE:
        errors.append(f"blob size {blob_size} exceeds {MAX_BLOB_SIZE} byte arena budget")
    if SRAM_RANGE[0] <= blob_start < SRAM_RANGE[1]:
        errors.append(f"blob is linked in SRAM (0x{blob_start:08x}), expected flash")

    nm_out = run([f"{tool_prefix}nm", "-n", str(elf)])
    syms = parse_symbols(nm_out)

    if ENTRY_SYMBOL not in syms:
        errors.append(f"entry symbol {ENTRY_SYMBOL} not found")
    elif (syms[ENTRY_SYMBOL] & ~1) != blob_start:
        errors.append(
            f"entry symbol {ENTRY_SYMBOL} at 0x{syms[ENTRY_SYMBOL]:08x}, expected blob start 0x{blob_start:08x}"
        )

    for sym_name, expected in (
        ("__ota_ram_engine_start", blob_start),
        ("__ota_ram_engine_end", blob_end),
    ):
        if syms.get(sym_name) != expected:
            errors.append(
                f"linker symbol {sym_name}=0x{syms.get(sym_name, 0):08x}, expected 0x{expected:08x}"
            )

    # Disassemble the entire blob section.
    disasm = run([f"{tool_prefix}objdump", "-d", "-j", SECTION_NAME, str(elf)])

    # objdump separates address, hex bytes, mnemonic, and operands with tabs:
    # "   2469c:\tf7ff f89e \tbl\t237dc <crc16_step>"
    insn_re = re.compile(r"^\s*([0-9a-fA-F]+):\t([0-9a-fA-F ]+?)\t(\S+)(?:\t(.*))?$")
    branch_mnemonics = re.compile(
        r"^(?:b|bl|blx|bx|cbz|cbnz|b(?:eq|ne|cs|hs|cc|lo|mi|pl|vs|vc|hi|ls|ge|lt|gt|le|al))(?:\.[nw])?$"
    )
    pc_load_re = re.compile(r"\[pc,\s*#[^\]]+\]\s*(?:;|@)\s*\(?([0-9a-fA-F]+)")
    branch_target_re = re.compile(r"^([0-9a-fA-F]+)\s+<([^>]+)>")

    insns = 0
    branches = 0
    pc_loads = 0
    literals: list[tuple[int, int]] = []
    pc_load_addrs: set[int] = set()

    for line in disasm.splitlines():
        m = insn_re.match(line)
        if not m:
            continue
        addr = int(m.group(1), 16)
        mnem = m.group(3)
        ops = (m.group(4) or "").strip()

        if mnem == ".word":
            val_str = ops.split()[0]
            val = int(val_str, 16)
            literals.append((addr, val))
            continue
        if mnem in (".short", ".byte"):
            continue

        insns += 1

        # Reject table branches referencing outside PC-inline tables (none expected with -fno-jump-tables)
        if mnem in ("tbb", "tbh"):
            errors.append(f"0x{addr:08x}: unexpected table branch `{mnem} {ops}`")

        # Check all branches / calls
        if branch_mnemonics.match(mnem):
            branches += 1
            if mnem.startswith("bx"):
                if ops != "lr":
                    errors.append(f"0x{addr:08x}: disallowed indirect branch `{mnem} {ops}`")
                continue
            if mnem.startswith("blx"):
                errors.append(f"0x{addr:08x}: disallowed indirect call `{mnem} {ops}`")
                continue
            tm = branch_target_re.search(ops)
            if not tm:
                # cbz/cbnz format: "r3, 23c24 <foo+0x10>"
                parts = ops.split(",", 1)
                if len(parts) == 2:
                    tm = branch_target_re.search(parts[1].strip())
            if not tm:
                errors.append(f"0x{addr:08x}: unparsed branch `{mnem} {ops}`")
                continue
            target = int(tm.group(1), 16)
            target_sym = tm.group(2)
            if not (blob_start <= target < blob_end):
                errors.append(
                    f"0x{addr:08x}: branch `{mnem} {ops}` escapes blob -> 0x{target:08x} <{target_sym}>"
                )

        # Check PC-relative loads / adr
        pm = pc_load_re.search(ops)
        if pm:
            pc_loads += 1
            target = int(pm.group(1), 16)
            pc_load_addrs.add(target)
            if not (blob_start <= target < blob_end):
                errors.append(
                    f"0x{addr:08x}: PC-relative load `{mnem} {ops}` escapes blob -> 0x{target:08x}"
                )
        elif mnem.startswith("adr"):
            errors.append(f"0x{addr:08x}: unexpected adr `{mnem} {ops}`")

    # Inspect every literal pool word for flash or static SRAM pointers.
    for lit_addr, val in literals:
        in_flash = FLASH_RANGE[0] <= val < FLASH_RANGE[1]
        in_sram = SRAM_RANGE[0] <= val < SRAM_RANGE[1]
        in_periph = PERIPH_RANGE[0] <= val < PERIPH_RANGE[1]
        in_core = CORE_RANGE[0] <= val < CORE_RANGE[1]
        if in_sram:
            errors.append(
                f"0x{lit_addr:08x}: literal 0x{val:08x} points into static SRAM"
            )
        elif in_flash and val not in KNOWN_CONSTANTS:
            errors.append(
                f"0x{lit_addr:08x}: literal 0x{val:08x} falls in flash address range"
            )
        elif not (in_periph or in_core or val in KNOWN_CONSTANTS):
            # Warn or flag unknown large constants so nothing slips past review
            if verbose:
                print(f"  note: literal at 0x{lit_addr:08x} = 0x{val:08x} (scalar constant)")

    # Verify zero symbols from receiver_ota_ram.c landed in .ramfunc
    ramfunc_bounds = None
    ramfunc_pat = re.compile(r"\[\s*\d+\]\s+\.ramfunc\s+\S+\s+([0-9a-fA-F]+)\s+[0-9a-fA-F]+\s+([0-9a-fA-F]+)")
    for line in readelf_out.splitlines():
        m = ramfunc_pat.search(line)
        if m:
            rs = int(m.group(1), 16)
            ramfunc_bounds = (rs, rs + int(m.group(2), 16))
            break

    if ramfunc_bounds:
        rs, re_end = ramfunc_bounds
        for name, addr in syms.items():
            if rs <= addr < re_end and ("ota_ram" in name or "rcv_ota" in name):
                errors.append(f"OTA symbol `{name}` still in .ramfunc at 0x{addr:08x}")

    if errors:
        print(f"FAIL ({elf}):")
        for err in errors:
            print(f"  - {err}")
        return False

    print(
        f"PASS ({elf.name}): {SECTION_NAME} @ 0x{blob_start:08x}..0x{blob_end:08x} "
        f"({blob_size} B), entry={ENTRY_SYMBOL}+0x0, "
        f"{insns} insns, {branches} internal branches, {pc_loads} PC-rel loads, "
        f"{len(literals)} literal words (0 flash/SRAM refs)"
    )
    if verbose:
        for lit_addr, val in literals:
            kind = (
                "periph" if PERIPH_RANGE[0] <= val < PERIPH_RANGE[1]
                else "core" if CORE_RANGE[0] <= val < CORE_RANGE[1]
                else "const"
            )
            print(f"  literal 0x{lit_addr:08x}: 0x{val:08x} ({kind})")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path, help="Path to linked zephyr.elf")
    parser.add_argument(
        "--tool-prefix",
        default="arm-zephyr-eabi-",
        help="GNU binutils prefix (default: arm-zephyr-eabi-)",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="Print literal table")
    args = parser.parse_args()
    return 0 if check_elf(args.elf, args.tool_prefix, args.verbose) else 1


if __name__ == "__main__":
    sys.exit(main())
