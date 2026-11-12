#!/usr/bin/env python
# /// script
# dependencies = [
#   "hidapi",
# ]
# ///
"""
ESB OTA Firmware Update Tool for SlimeNRF

Sends firmware to a tracker via the receiver's USB HID interface.

Data flow: PC → HID OUT → Receiver → ESB ACK → Tracker

Usage:
    python esb_ota.py <firmware.uf2|firmware.signed.bin> --tracker <id> [--board <board_target>]
    python esb_ota.py --info --tracker <id>
    python esb_ota.py --abort

Requirements:
    pip install hidapi
"""

import argparse
import struct
import sys
import time
import zlib
from dataclasses import dataclass
from pathlib import Path

try:
    import hid
except ImportError:
    print("Error: hidapi not installed. Run: pip install hidapi")
    sys.exit(1)

# ── HID Device ──────────────────────────────────────────────────────

VID = 0x1209
PID = 0x7690
REPORT_SIZE = 64  # HID IN/OUT report size (bytes)

# ── HID OTA Report Types ────────────────────────────────────────────

HID_OTA_QUERY_INFO = 0xF0
HID_OTA_FW_INFO    = 0xF1
HID_OTA_BEGIN      = 0xF2
HID_OTA_DATA       = 0xF3
HID_OTA_STATUS     = 0xF4
HID_OTA_VERIFY     = 0xF5
HID_OTA_ACTIVATE   = 0xF6
HID_OTA_ABORT      = 0xF7

# ── OTA Status Codes ────────────────────────────────────────────────

OTA_STATUS_IDLE           = 0x00
OTA_STATUS_READY          = 0x01
OTA_STATUS_RECEIVING      = 0x02
OTA_STATUS_VERIFY_OK      = 0x03
OTA_STATUS_VERIFY_FAIL    = 0x04
OTA_STATUS_ACTIVATING     = 0x05
OTA_STATUS_COMPLETE       = 0x06
OTA_STATUS_ERROR          = 0x10
OTA_STATUS_BOARD_MISMATCH = 0x11
OTA_STATUS_FLASH_ERROR    = 0x12
OTA_STATUS_SIZE_ERROR     = 0x13
OTA_STATUS_SEQ_ERROR      = 0x14
OTA_STATUS_TIMEOUT        = 0x15

STATUS_NAMES = {
    0x00: "IDLE",
    0x01: "READY",
    0x02: "RECEIVING",
    0x03: "VERIFY_OK",
    0x04: "VERIFY_FAIL",
    0x05: "ACTIVATING",
    0x06: "COMPLETE",
    0x10: "ERROR",
    0x11: "BOARD_MISMATCH",
    0x12: "FLASH_ERROR",
    0x13: "SIZE_ERROR",
    0x14: "SEQ_ERROR",
    0x15: "TIMEOUT",
}

TERMINAL_STATUSES = {
    OTA_STATUS_COMPLETE,
    OTA_STATUS_ERROR,
    OTA_STATUS_VERIFY_FAIL,
    OTA_STATUS_TIMEOUT,
    OTA_STATUS_BOARD_MISMATCH,
    OTA_STATUS_SIZE_ERROR,
    OTA_STATUS_FLASH_ERROR,
    OTA_STATUS_SEQ_ERROR,
}

# ── OTA Protocol Constants ──────────────────────────────────────────

OTA_PROTOCOL_VERSION = 1
OTA_DATA_MAX_PAYLOAD = 60  # Max firmware bytes per data packet
OTA_BOARD_TARGET_MAX = 48
RING_BUFFER_SIZE     = 128  # Must match firmware OTA_TX_RING_SIZE
RECEIVER_OTA_ID      = 0xFE  # tracker_id for receiver self-OTA


# ── UF2 Parser ──────────────────────────────────────────────────────

UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END    = 0x0AB16F30
UF2_BLOCK_SIZE   = 512
UF2_DATA_SIZE    = 256  # Payload per block (default)
UF2_FAMILY_NRF52840 = 0xADA52840


@dataclass
class FirmwareImage:
    """Contiguous firmware binary extracted from UF2."""
    data: bytes
    base_address: int
    board_target: str  # From UF2 metadata or user override
    crc32: int
    mcuboot_image: bool = False


def parse_uf2(path: Path, flash_offset: int = 0x1000) -> FirmwareImage:
    """Parse a UF2 file and extract the firmware binary.

    UF2 blocks: 512 bytes each, 256-byte payload, sorted by address.
    We extract blocks matching the application region (at/after flash_offset).
    """
    raw = path.read_bytes()
    if len(raw) % UF2_BLOCK_SIZE != 0:
        raise ValueError(f"UF2 file size ({len(raw)}) not a multiple of 512")

    num_blocks = len(raw) // UF2_BLOCK_SIZE
    blocks: list[tuple[int, bytes]] = []

    for i in range(num_blocks):
        block = raw[i * UF2_BLOCK_SIZE : (i + 1) * UF2_BLOCK_SIZE]

        magic0, magic1 = struct.unpack_from("<II", block, 0)
        magic_end = struct.unpack_from("<I", block, 508)[0]

        if magic0 != UF2_MAGIC_START0 or magic1 != UF2_MAGIC_START1:
            raise ValueError(f"Block {i}: bad start magic")
        if magic_end != UF2_MAGIC_END:
            raise ValueError(f"Block {i}: bad end magic")

        flags, addr, size, block_no, num_blks, family = struct.unpack_from(
            "<IIIIII", block, 8
        )

        if size > UF2_DATA_SIZE:
            size = UF2_DATA_SIZE

        # Skip non-application blocks (e.g., softdevice at 0x0, bootloader)
        if addr < flash_offset:
            continue

        payload = block[32 : 32 + size]
        blocks.append((addr, payload))

    if not blocks:
        raise ValueError("No application blocks found in UF2")

    # Sort by address and create contiguous image
    blocks.sort(key=lambda b: b[0])
    base = blocks[0][0]
    end = blocks[-1][0] + len(blocks[-1][1])
    image_size = end - base

    # Allocate and fill (0xFF for gaps - erased flash)
    image = bytearray(b"\xFF" * image_size)
    for addr, payload in blocks:
        offset = addr - base
        image[offset : offset + len(payload)] = payload

    image_bytes = bytes(image)
    crc = zlib.crc32(image_bytes) & 0xFFFFFFFF

    print(f"UF2: {num_blocks} blocks, {len(blocks)} app blocks")
    print(f"     Base: 0x{base:08X}, Size: {image_size} bytes ({image_size/1024:.1f} KB)")
    print(f"     CRC32: 0x{crc:08X}")

    return FirmwareImage(
        data=image_bytes,
        base_address=base,
        board_target="",
        crc32=crc,
    )


# ── Intel HEX Parser ───────────────────────────────────────────────

def parse_hex(path: Path, flash_offset: int = 0x1000) -> FirmwareImage:
    """Parse an Intel HEX file and extract the firmware binary.

    Supports record types 00 (data), 01 (EOF), 02 (extended segment),
    04 (extended linear address). Extracts contiguous app image
    starting from flash_offset.
    """
    records: list[tuple[int, bytes]] = []
    base_addr = 0  # Extended address (upper 16 bits)

    with open(path, 'r') as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line or line[0] != ':':
                continue
            # Parse Intel HEX record
            raw = bytes.fromhex(line[1:])
            if len(raw) < 5:
                raise ValueError(f"Line {line_no}: record too short")
            byte_count = raw[0]
            address = (raw[1] << 8) | raw[2]
            rec_type = raw[3]
            data = raw[4:4 + byte_count]
            checksum = raw[4 + byte_count]

            # Verify checksum
            calc_sum = sum(raw[:4 + byte_count]) & 0xFF
            calc_check = (~calc_sum + 1) & 0xFF
            if checksum != calc_check:
                raise ValueError(f"Line {line_no}: checksum mismatch "
                                 f"(got 0x{checksum:02X}, expected 0x{calc_check:02X})")

            if rec_type == 0x00:  # Data record
                full_addr = base_addr + address
                if full_addr >= flash_offset:
                    records.append((full_addr, data))
            elif rec_type == 0x01:  # EOF
                break
            elif rec_type == 0x02:  # Extended segment address
                base_addr = ((data[0] << 8) | data[1]) << 4
            elif rec_type == 0x04:  # Extended linear address
                base_addr = ((data[0] << 8) | data[1]) << 16
            # Ignore other types (03, 05)

    if not records:
        raise ValueError("No application data found in HEX file")

    # Sort by address and create contiguous image
    records.sort(key=lambda r: r[0])
    base = records[0][0]
    end = records[-1][0] + len(records[-1][1])
    image_size = end - base

    # Allocate and fill (0xFF for gaps)
    image = bytearray(b"\xFF" * image_size)
    for addr, data in records:
        offset = addr - base
        image[offset : offset + len(data)] = data

    image_bytes = bytes(image)
    crc = zlib.crc32(image_bytes) & 0xFFFFFFFF

    print(f"HEX: {len(records)} data records")
    print(f"     Base: 0x{base:08X}, Size: {image_size} bytes ({image_size/1024:.1f} KB)")
    print(f"     CRC32: 0x{crc:08X}")

    return FirmwareImage(
        data=image_bytes,
        base_address=base,
        board_target="",
        crc32=crc,
    )


def parse_firmware(path: Path, flash_offset: int = 0x1000) -> FirmwareImage:
    """Auto-detect UF2, HEX, or an MCUboot update binary."""
    suffix = path.suffix.lower()
    if suffix == '.uf2':
        return parse_uf2(path, flash_offset)
    elif suffix in ('.hex', '.ihex'):
        return parse_hex(path, flash_offset)
    elif suffix == '.bin':
        data = path.read_bytes()
        if len(data) < 32 or struct.unpack_from("<I", data, 0)[0] != 0x96F3B83D:
            raise ValueError("BIN is not an MCUboot update image")
        return FirmwareImage(
            data=data,
            base_address=0,
            board_target="",
            crc32=zlib.crc32(data) & 0xFFFFFFFF,
            mcuboot_image=True,
        )
    else:
        # Try to detect by content
        with open(path, 'rb') as f:
            magic = f.read(4)
        if magic == b'UF2\n' or (len(magic) >= 4 and
                struct.unpack_from("<I", magic, 0)[0] == UF2_MAGIC_START0):
            return parse_uf2(path, flash_offset)
        # Assume Intel HEX if it starts with ':'
        with open(path, 'r') as f:
            first_line = f.readline().strip()
        if first_line.startswith(':'):
            return parse_hex(path, flash_offset)
        raise ValueError(f"Unknown firmware format: {path.suffix}")


def looks_like_sd_plus_app(firmware: FirmwareImage, device_base: int) -> bool:
    """Check if firmware contains SoftDevice + Application.

    Detects by looking for a valid ARM Cortex-M vector table at the
    device's expected app start address within the firmware image.
    """
    if firmware.base_address >= device_base:
        return False
    offset = device_base - firmware.base_address
    if offset + 8 > len(firmware.data):
        return False

    # Read initial SP and reset vector at device's app base
    initial_sp = struct.unpack_from("<I", firmware.data, offset)[0]
    reset_vector = struct.unpack_from("<I", firmware.data, offset + 4)[0]

    # SP should point to RAM (0x20000000 - 0x20040000 for nRF52)
    sp_valid = 0x20000000 <= initial_sp <= 0x20040000
    # Reset vector should be in flash at/after device base (odd = Thumb)
    rv_valid = device_base <= reset_vector < 0x100000 and (reset_vector & 1) == 1

    return sp_valid and rv_valid


# ── HID Communication ──────────────────────────────────────────────

def enumerate_receivers() -> list[dict]:
    """Enumerate all connected SlimeNRF receivers."""
    devices = hid.enumerate(VID, PID)
    # Deduplicate by path (some OSes list multiple interfaces)
    seen = set()
    unique = []
    for d in devices:
        path = d["path"]
        if path not in seen:
            seen.add(path)
            unique.append(d)
    return unique


def select_receiver(devices: list[dict], index: int | None = None) -> bytes:
    """Select a receiver device path.

    If index is specified, use it directly.
    If only one device, use it.
    If multiple, prompt the user.
    Returns the device path.
    """
    if not devices:
        print("Error: No SlimeNRF receivers found.")
        print(f"       (looking for VID={VID:#06x} PID={PID:#06x})")
        sys.exit(1)

    if index is not None:
        if index < 0 or index >= len(devices):
            print(f"Error: Receiver index {index} out of range (0-{len(devices)-1})")
            sys.exit(1)
        return devices[index]["path"]

    if len(devices) == 1:
        return devices[0]["path"]

    # Multiple devices — list and prompt
    print(f"\nMultiple receivers found ({len(devices)}):")
    for i, d in enumerate(devices):
        serial = d.get("serial_number", "") or "n/a"
        product = d.get("product_string", "") or "unknown"
        trackers = discover_trackers(d["path"])
        online = [t for t in sorted(trackers) if trackers[t]["online"]]
        tracker_str = ", ".join(str(t) for t in online) if online else "none online"
        print(f"  [{i}] {product}  (serial: {serial}, trackers: {tracker_str})")
    print()
    try:
        choice = input(f"Select receiver [0-{len(devices)-1}]: ")
        idx = int(choice.strip())
        if idx < 0 or idx >= len(devices):
            print("Invalid choice.")
            sys.exit(1)
        return devices[idx]["path"]
    except (ValueError, EOFError, KeyboardInterrupt):
        print("\nAborted.")
        sys.exit(1)


def discover_trackers(device_path: bytes, duration_s: float = 1.5) -> dict[int, dict]:
    """Open a receiver briefly and discover connected tracker IDs and addresses.

    Reads HID reports for `duration_s` seconds and extracts tracker info.

    Returns dict of tracker_id → {"addr": hex_string, "online": bool}.
    "online" is True if the tracker is actively sending data packets,
    False if only seen via device_addr registration (padding).
    """
    try:
        dev = hid.device()
        dev.open_path(device_path)
        dev.set_nonblocking(True)
    except Exception:
        return {}

    trackers: dict[int, dict] = {}
    deadline = time.monotonic() + duration_s

    while time.monotonic() < deadline:
        data = dev.read(REPORT_SIZE, timeout_ms=100)
        if not data or len(data) < 16:
            continue
        # Split 64-byte frame into 4 × 16-byte sub-reports
        for offset in range(0, min(len(data), 64), 16):
            sub = data[offset:offset + 16]
            if len(sub) < 8:
                continue
            pkt_type = sub[0]
            tid = sub[1]
            if pkt_type == 255 and tid < 64:
                # Device address registration (padding) — all registered trackers
                addr_bytes = sub[2:8]
                addr_str = "".join(f"{b:02X}" for b in reversed(addr_bytes))
                if tid not in trackers:
                    trackers[tid] = {"addr": addr_str, "online": False}
                else:
                    trackers[tid]["addr"] = addr_str
            elif pkt_type != 0 and pkt_type < 0xF0 and tid < 64:
                # Active data packet — tracker is online
                if tid not in trackers:
                    trackers[tid] = {"addr": "", "online": True}
                else:
                    trackers[tid]["online"] = True

    dev.close()
    return trackers


class OTAClient:
    """Communicates with the receiver via USB HID for OTA firmware updates."""

    def __init__(self, device_path: bytes | None = None):
        self.dev = hid.device()
        if device_path:
            self.dev.open_path(device_path)
        else:
            self.dev.open(VID, PID)
        self.dev.set_nonblocking(True)
        info = self.dev.get_product_string()
        print(f"Connected: {info}")

    def close(self):
        self.dev.close()

    def _write(self, data: bytes):
        """Send a 64-byte HID OUT report."""
        # hidapi on some platforms needs a prepended report ID byte (0x00)
        padded = data.ljust(REPORT_SIZE, b"\x00")
        self.dev.write(b"\x00" + padded)

    def _read(self, timeout_ms: int = 100) -> bytes | None:
        """Read a HID IN report. Returns None on timeout."""
        data = self.dev.read(REPORT_SIZE, timeout_ms)
        if data and len(data) > 0:
            return bytes(data)
        return None

    def _read_ota_reports(self, timeout_ms: int = 500) -> list[bytes]:
        """Read all pending HID reports, filtering for OTA types.

        The receiver packs 4 × 16-byte sub-reports into each 64-byte HID frame.
        We need to check all 4 sub-reports for OTA data.
        """
        reports = []
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
            data = self._read(timeout_ms=remaining_ms)
            if data is None:
                break  # No more data within deadline
            # Split 64-byte frame into 4 × 16-byte sub-reports
            for offset in range(0, min(len(data), 64), 16):
                sub = data[offset:offset + 16]
                if len(sub) >= 1 and 0xF0 <= sub[0] <= 0xF7:
                    reports.append(bytes(sub))
        return reports

    # ── Commands ─────────────────────────────────────────────────

    def query_info(self, tracker_id: int):
        """Send firmware info query for a tracker."""
        pkt = bytearray(REPORT_SIZE)
        pkt[0] = HID_OTA_QUERY_INFO
        pkt[1] = tracker_id
        self._write(bytes(pkt))

    def send_begin(self, tracker_id: int, image_size: int, image_crc32: int,
                   total_packets: int, board_target: str, flash_base: int = 0):
        """Send OTA BEGIN packet."""
        pkt = bytearray(REPORT_SIZE)
        pkt[0] = HID_OTA_BEGIN
        pkt[1] = tracker_id
        struct.pack_into("<I", pkt, 2, image_size)
        struct.pack_into("<I", pkt, 6, image_crc32)
        struct.pack_into(">H", pkt, 10, total_packets)
        pkt[12] = OTA_PROTOCOL_VERSION
        target_bytes = board_target.encode("utf-8")[:OTA_BOARD_TARGET_MAX - 1]
        pkt[13 : 13 + len(target_bytes)] = target_bytes
        # Bytes 61-62: flash_base_address >> 12 (page-aligned, big-endian)
        if flash_base > 0:
            struct.pack_into(">H", pkt, 61, flash_base >> 12)
        self._write(bytes(pkt))

    def send_data(self, tracker_id: int, seq: int, data: bytes):
        """Send one OTA DATA packet (up to 44 bytes of firmware)."""
        pkt = bytearray(REPORT_SIZE)
        pkt[0] = HID_OTA_DATA
        pkt[1] = tracker_id
        struct.pack_into(">H", pkt, 2, seq)
        payload_len = min(len(data), OTA_DATA_MAX_PAYLOAD)
        pkt[4 : 4 + payload_len] = data[:payload_len]
        self._write(bytes(pkt))

    def send_verify(self, tracker_id: int):
        """Request CRC32 verification."""
        pkt = bytearray(REPORT_SIZE)
        pkt[0] = HID_OTA_VERIFY
        pkt[1] = tracker_id
        self._write(bytes(pkt))

    def send_activate(self, tracker_id: int):
        """Activate new firmware (writes bootloader settings, reboots)."""
        pkt = bytearray(REPORT_SIZE)
        pkt[0] = HID_OTA_ACTIVATE
        pkt[1] = tracker_id
        self._write(bytes(pkt))

    def send_abort(self, tracker_id: int = 0xFF):
        """Abort OTA session."""
        pkt = bytearray(REPORT_SIZE)
        pkt[0] = HID_OTA_ABORT
        pkt[1] = tracker_id
        self._write(bytes(pkt))

    # ── Status Parsing ───────────────────────────────────────────

    @staticmethod
    def parse_status(report: bytes) -> dict:
        """Parse an OTA_STATUS HID IN report."""
        if len(report) < 10 or report[0] != HID_OTA_STATUS:
            return {}
        return {
            "tracker_id": report[1],
            "status": report[2],
            "status_name": STATUS_NAMES.get(report[2], f"0x{report[2]:02X}"),
            "next_seq": struct.unpack_from(">H", report, 3)[0],
            "bytes_written": struct.unpack_from("<I", report, 5)[0],
            "ring_count": report[9],
        }

    @staticmethod
    def parse_fw_info(chunks: list[bytes]) -> dict:
        """Parse FW_INFO HID IN reports (6 chunks → 66-byte info)."""
        info_data = bytearray(66)
        for chunk in chunks:
            if len(chunk) < 3 or chunk[0] != HID_OTA_FW_INFO:
                continue
            idx = chunk[2]
            if idx > 5:
                continue
            offset = 2 + idx * 13  # Skip type+id in original ESB packet
            copy_len = min(13, 66 - offset)
            if copy_len > 0:
                info_data[offset : offset + copy_len] = chunk[3 : 3 + copy_len]

        # Parse the 48-byte firmware info
        # Byte 2: major, 3: minor, 4: patch
        # Byte 5-8: build_datetime packed (32-bit BE)
        #   bits 31-25: year-2020, 24-21: month, 20-16: day,
        #   15-11: hour, 10-5: minute, 4-0: second/2
        # Byte 9-12: firmware_size LE
        # Byte 13: bootloader_type
        # Byte 14: ota_protocol_version
        # Byte 15-44: board_target
        major = info_data[2]
        minor = info_data[3]
        patch = info_data[4]
        build_dt_raw = struct.unpack_from(">I", info_data, 5)[0]
        year = ((build_dt_raw >> 25) & 0x7F) + 2020
        month = (build_dt_raw >> 21) & 0x0F
        day = (build_dt_raw >> 16) & 0x1F
        hour = (build_dt_raw >> 11) & 0x1F
        minute = (build_dt_raw >> 5) & 0x3F
        second = (build_dt_raw & 0x1F) * 2
        fw_size = struct.unpack_from("<I", info_data, 9)[0]
        bl_type = info_data[13]
        proto_ver = info_data[14]
        board_end = info_data.find(0, 15, 63)
        if board_end < 0:
            board_end = 63
        board_target = info_data[15:board_end].decode("utf-8", errors="replace")

        # Flash base address (bytes 63-64, page-aligned: value << 12)
        flash_base_raw = struct.unpack_from(">H", info_data, 63)[0]
        flash_base = flash_base_raw << 12

        bl_types = {
            0: "none",
            1: "adafruit_uf2",
            2: "nrf5_opendfu",
            3: "mcuboot",
        }

        return {
            "version": f"{major}.{minor}.{patch}",
            "build_date": f"{year}-{month:02d}-{day:02d} {hour:02d}:{minute:02d}:{second:02d}",
            "firmware_size": fw_size,
            "bootloader": bl_types.get(bl_type, f"unknown({bl_type})"),
            "protocol_version": proto_ver,
            "board_target": board_target,
            "flash_base": flash_base,
        }

    def wait_for_status(self, tracker_id: int, expected_statuses: set[int],
                        timeout_s: float = 30.0) -> dict | None:
        """Wait for a specific status from the tracker."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            reports = self._read_ota_reports(timeout_ms=200)
            for r in reports:
                if r[0] == HID_OTA_STATUS and r[1] == tracker_id:
                    status = self.parse_status(r)
                    if status.get("status") in expected_statuses:
                        return status
        return None

    def wait_for_multi_status(self, tracker_ids: list[int],
                              expected_statuses: set[int],
                              timeout_s: float = 30.0,
                              resend_cb: "callable | None" = None,
                              resend_interval_s: float = 3.0) -> dict[int, dict]:
        """Wait for specific status from multiple trackers.
        Returns dict of tracker_id → status for those that responded.

        If resend_cb is provided, it is called with each pending tracker_id
        every resend_interval_s seconds to retry the command.
        """
        results: dict[int, dict] = {}
        pending = set(tracker_ids)
        deadline = time.monotonic() + timeout_s
        next_resend = time.monotonic() + resend_interval_s if resend_cb else deadline + 1
        resend_count = 0
        while pending and time.monotonic() < deadline:
            # Periodic resend for unresponsive trackers
            if resend_cb and time.monotonic() >= next_resend:
                resend_count += 1
                for tid in sorted(pending):
                    resend_cb(tid)
                    time.sleep(0.05)
                print(f"    (retry #{resend_count} for tracker{'s' if len(pending) > 1 else ''}"
                      f" {', '.join(str(t) for t in sorted(pending))})")
                next_resend = time.monotonic() + resend_interval_s

            reports = self._read_ota_reports(timeout_ms=200)
            for r in reports:
                if r[0] == HID_OTA_STATUS and r[1] in pending:
                    status = self.parse_status(r)
                    if status.get("status") in expected_statuses:
                        results[r[1]] = status
                        pending.discard(r[1])
        return results


# ── OTA Update Procedure ────────────────────────────────────────────

def do_query_info(client: OTAClient, tracker_id: int, quiet: bool = False):
    """Query and display firmware info from a tracker."""
    if not quiet:
        print(f"\nQuerying firmware info for tracker {tracker_id}...")
    client.query_info(tracker_id)

    # Collect FW_INFO chunks (6 expected). The tracker answers with one
    # unacknowledged-by-host ESB frame; if it is lost (e.g. during a TDMA
    # layout change) nothing retries it, so re-query while nothing arrived.
    chunks = {}
    start = time.monotonic()
    deadline = start + 6.0
    next_resend = start + 1.5
    while time.monotonic() < deadline and len(chunks) < 6:
        if not chunks and time.monotonic() >= next_resend:
            client.query_info(tracker_id)
            next_resend = time.monotonic() + 1.5
        reports = client._read_ota_reports(timeout_ms=200)
        for r in reports:
            if r[0] == HID_OTA_FW_INFO and r[1] == tracker_id and r[2] not in chunks:
                chunks[r[2]] = r
                if not quiet:
                    print(f"  [chunk {r[2]}] data={r.hex()}")
    chunks = [chunks[k] for k in sorted(chunks)]

    if not chunks:
        if not quiet:
            print("Error: No firmware info response received.")
            print("       Make sure the tracker is connected and responding.")
        return None

    if len(chunks) < 5:
        if not quiet:
            print(f"  Warning: Only received {len(chunks)}/6 FW_INFO chunks")

    info = OTAClient.parse_fw_info(chunks)
    if not quiet:
        print(f"\n  Firmware Version: {info['version']}")
        print(f"  Build Date:      {info['build_date']}")
        print(f"  Firmware Size:   {info['firmware_size']} bytes")
        print(f"  Bootloader:      {info['bootloader']}")
        print(f"  OTA Protocol:    v{info['protocol_version']}")
        print(f"  Board Target:    {info['board_target']}")
        print(f"  Flash Base:      0x{info['flash_base']:08X}")
    return info


def do_update(client: OTAClient, tracker_ids: list[int], firmware: FirmwareImage,
              board_target: str):
    """Perform OTA update for one or more trackers with the same firmware.

    All trackers in tracker_ids must use the same firmware image.
    Data is streamed once; the receiver fans it out via per-tracker cursors.
    """
    image_size = len(firmware.data)
    total_packets = (image_size + OTA_DATA_MAX_PAYLOAD - 1) // OTA_DATA_MAX_PAYLOAD
    n_targets = len(tracker_ids)

    print(f"\n{'='*60}")
    print(f"  ESB OTA Firmware Update (Parallel)")
    print(f"  Trackers:     {', '.join(str(t) for t in tracker_ids)} ({n_targets} target{'s' if n_targets > 1 else ''})")
    print(f"  Board Target: {board_target}")
    print(f"  Image Size:   {image_size} bytes ({image_size/1024:.1f} KB)")
    print(f"  Flash Base:   0x{firmware.base_address:08X}")
    print(f"  Packets:      {total_packets}")
    print(f"  CRC32:        0x{firmware.crc32:08X}")
    print(f"{'='*60}")

    # ── Step 1: Send BEGIN to each tracker ───────────────────────
    print(f"\n[1/4] Sending BEGIN to {n_targets} tracker(s)...")
    for tid in tracker_ids:
        client.send_begin(tid, image_size, firmware.crc32,
                          total_packets, board_target, firmware.base_address)
        time.sleep(0.05)  # Small delay between BEGINs

    # Wait for all trackers to be READY (with PC-side retry every 3s, 15s timeout)
    def _resend_begin(tid):
        client.send_begin(tid, image_size, firmware.crc32,
                          total_packets, board_target, firmware.base_address)

    ready_statuses = client.wait_for_multi_status(
        tracker_ids,
        {OTA_STATUS_READY, OTA_STATUS_RECEIVING,
         OTA_STATUS_BOARD_MISMATCH, OTA_STATUS_SIZE_ERROR, OTA_STATUS_ERROR},
        timeout_s=15.0,
        resend_cb=_resend_begin,
        resend_interval_s=3.0,
    )

    # Check results
    active_ids = []
    for tid in tracker_ids:
        st = ready_statuses.get(tid)
        if st is None:
            print(f"  Tracker {tid}: No response (skipping)")
        elif st["status"] in TERMINAL_STATUSES or st["status"] not in (
            OTA_STATUS_READY, OTA_STATUS_RECEIVING
        ):
            print(f"  Tracker {tid}: Rejected ({st['status_name']})")
        else:
            print(f"  Tracker {tid}: Ready")
            active_ids.append(tid)

    if not active_ids:
        print("Error: No trackers ready for update.")
        client.send_abort(0xFF)  # Unsuppress all trackers
        return False

    # ── Step 2: Stream DATA packets ─────────────────────────────
    print(f"\n[2/4] Streaming firmware data to {len(active_ids)} tracker(s)...")
    next_seq_to_send = 0
    last_progress = -1
    retransmit_count = 0
    start_time = time.monotonic()

    # Track per-tracker status for retransmission
    tracker_next_seq: dict[int, int] = {tid: 0 for tid in active_ids}

    BURST_SIZE = 48  # max packets per burst before checking status
    # Max packets in-flight (PC sent but tracker hasn't confirmed).
    # Must be < receiver ring buffer size (128) to prevent ring overflow.
    MAX_IN_FLIGHT = RING_BUFFER_SIZE - 16  # 111
    warmup = True  # start slow until we get first STATUS from tracker
    max_attempts = 5  # max gap-fill re-stream attempts
    attempt = 0
    overall_timeout = time.monotonic() + 120.0  # 2 minute total timeout

    while attempt < max_attempts and time.monotonic() < overall_timeout:
        # ── Streaming phase: send all remaining packets ──
        while next_seq_to_send < total_packets and time.monotonic() < overall_timeout:
            # Compute in-flight count (packets sent but not yet confirmed)
            min_consumed = min(tracker_next_seq.get(t, 0) for t in active_ids)
            in_flight = next_seq_to_send - min_consumed

            if in_flight >= MAX_IN_FLIGHT:
                # Ring buffer at capacity — wait for tracker to consume
                reports = client._read_ota_reports(timeout_ms=50)
                for r in reports:
                    if r[0] == HID_OTA_STATUS and r[1] in active_ids:
                        st = OTAClient.parse_status(r)
                        tid = r[1]
                        if st["status"] in TERMINAL_STATUSES:
                            print(f"\n  Tracker {tid}: Error: {st['status_name']}")
                            active_ids.remove(tid)
                            if not active_ids:
                                print("  All trackers failed.")
                                return False
                            continue
                        tracker_next_seq[tid] = st["next_seq"]
                        if warmup:
                            warmup = False
                # Update progress while waiting
                min_consumed = min(tracker_next_seq.get(t, 0) for t in active_ids)
                progress = (min_consumed * 100) // total_packets
                if progress != last_progress:
                    elapsed = time.monotonic() - start_time
                    bytes_consumed = min(min_consumed * OTA_DATA_MAX_PAYLOAD, image_size)
                    speed = bytes_consumed / elapsed if elapsed > 0 else 0
                    bar = "█" * (progress // 5) + "░" * (20 - progress // 5)
                    targets_str = f" [{len(active_ids)} targets]" if n_targets > 1 else ""
                    print(f"\r  [{bar}] {progress:3d}% "
                          f"({bytes_consumed/1024:.0f}/{image_size/1024:.0f} KB) "
                          f"{speed/1024:.1f} KB/s{targets_str}", end="", flush=True)
                    last_progress = progress
                continue

            # Dynamic burst size: send up to available headroom
            can_send = MAX_IN_FLIGHT - in_flight
            if warmup:
                burst = min(8, can_send)
            else:
                burst = min(BURST_SIZE, can_send)

            sent_count = 0
            for _ in range(burst):
                if next_seq_to_send >= total_packets:
                    break
                offset = next_seq_to_send * OTA_DATA_MAX_PAYLOAD
                chunk = firmware.data[offset : offset + OTA_DATA_MAX_PAYLOAD]
                if not chunk:
                    break
                client.send_data(active_ids[0], next_seq_to_send, chunk)
                next_seq_to_send += 1
                sent_count += 1

            read_timeout = 50 if warmup else 5
            reports = client._read_ota_reports(timeout_ms=read_timeout)
            for r in reports:
                if r[0] == HID_OTA_STATUS and r[1] in active_ids:
                    st = OTAClient.parse_status(r)
                    tid = r[1]
                    if st["status"] in TERMINAL_STATUSES:
                        print(f"\n  Tracker {tid}: Error: {st['status_name']}")
                        active_ids.remove(tid)
                        if not active_ids:
                            print("  All trackers failed.")
                            return False
                        continue
                    tracker_next_seq[tid] = st["next_seq"]
                    if warmup:
                        warmup = False

            # Progress display
            min_consumed = min(tracker_next_seq.get(t, 0) for t in active_ids)
            progress = (min_consumed * 100) // total_packets
            if progress != last_progress:
                elapsed = time.monotonic() - start_time
                bytes_consumed = min(min_consumed * OTA_DATA_MAX_PAYLOAD, image_size)
                speed = bytes_consumed / elapsed if elapsed > 0 else 0
                bar = "█" * (progress // 5) + "░" * (20 - progress // 5)
                targets_str = f" [{len(active_ids)} targets]" if n_targets > 1 else ""
                print(f"\r  [{bar}] {progress:3d}% "
                      f"({bytes_consumed/1024:.0f}/{image_size/1024:.0f} KB) "
                      f"{speed/1024:.1f} KB/s{targets_str}", end="", flush=True)
                last_progress = progress

        # ── Gap-fill: wait for tracker to finish consuming ──────────
        # With seq-indexed ring buffer, the receiver automatically retransmits
        # lost ACKs. Gap-fill only needed if receiver dropped HID packets.
        stall_count = 0
        prev_min_seq = -1
        gap_timeout = time.monotonic() + 15.0

        while time.monotonic() < gap_timeout:
            min_consumed = min(tracker_next_seq.get(t, 0) for t in active_ids)
            if min_consumed >= total_packets:
                break

            reports = client._read_ota_reports(timeout_ms=200)
            for r in reports:
                if r[0] == HID_OTA_STATUS and r[1] in active_ids:
                    st = OTAClient.parse_status(r)
                    tid = r[1]
                    if st["status"] in TERMINAL_STATUSES:
                        active_ids.remove(tid)
                        continue
                    tracker_next_seq[tid] = st["next_seq"]

            min_consumed = min(tracker_next_seq.get(t, 0) for t in active_ids)
            if min_consumed >= total_packets or not active_ids:
                break

            in_flight = next_seq_to_send - min_consumed
            if min_consumed == prev_min_seq:
                stall_count += 1
                if stall_count >= 5 and in_flight == 0:
                    # Stalled with empty ring — receiver lost some packets
                    remaining = total_packets - min_consumed
                    print(f"\n  Gap-fill #{attempt+1}: resending from seq {min_consumed} "
                          f"({remaining} remaining)")
                    retransmit_count += remaining
                    next_seq_to_send = min_consumed
                    warmup = False
                    attempt += 1
                    break
            else:
                stall_count = 0
            prev_min_seq = min_consumed

            # Update progress
            progress = (min_consumed * 100) // total_packets
            if progress != last_progress:
                elapsed_now = time.monotonic() - start_time
                bytes_consumed = min(min_consumed * OTA_DATA_MAX_PAYLOAD, image_size)
                speed = bytes_consumed / elapsed_now if elapsed_now > 0 else 0
                bar = "█" * (progress // 5) + "░" * (20 - progress // 5)
                print(f"\r  [{bar}] {progress:3d}% "
                      f"({bytes_consumed/1024:.0f}/{image_size/1024:.0f} KB) "
                      f"{speed/1024:.1f} KB/s", end="", flush=True)
                last_progress = progress
        else:
            # gap_timeout expired without resolving or stalling — break outer loop
            break

        # Check if all consumed
        min_consumed = min(tracker_next_seq.get(t, 0) for t in active_ids)
        if min_consumed >= total_packets:
            break

    elapsed = time.monotonic() - start_time
    effective_speed = image_size * len(active_ids) / elapsed / 1024
    print(f"\n  Transfer complete: {image_size/1024:.1f} KB in {elapsed:.1f}s "
          f"({image_size/elapsed/1024:.1f} KB/s per tracker, "
          f"{effective_speed:.1f} KB/s total)")
    if retransmit_count > 0:
        print(f"  Retransmissions: {retransmit_count} packets")

    # ── Step 3: Verify CRC32 for each tracker ───────────────────
    print(f"\n[3/4] Requesting CRC32 verification...")
    time.sleep(0.5)
    for tid in active_ids:
        client.send_verify(tid)
        time.sleep(0.05)

    verify_results = client.wait_for_multi_status(
        active_ids,
        {OTA_STATUS_VERIFY_OK, OTA_STATUS_VERIFY_FAIL, OTA_STATUS_ERROR},
        timeout_s=30.0,
        resend_cb=lambda tid: client.send_verify(tid),
        resend_interval_s=3.0,
    )

    verified_ids = []
    for tid in active_ids:
        st = verify_results.get(tid)
        if st is None:
            print(f"  Tracker {tid}: No verification response")
        elif st["status"] != OTA_STATUS_VERIFY_OK:
            print(f"  Tracker {tid}: Verification FAILED ({st['status_name']})")
        else:
            print(f"  Tracker {tid}: CRC32 verified OK")
            verified_ids.append(tid)

    if not verified_ids:
        print("  Error: No trackers passed verification.")
        client.send_abort(0xFF)  # Unsuppress all trackers
        return False

    # ── Step 4: Activate verified trackers ──────────────────────
    print(f"\n[4/4] Activating new firmware on {len(verified_ids)} tracker(s)...")
    for tid in verified_ids:
        client.send_activate(tid)
        time.sleep(0.05)

    activate_results = client.wait_for_multi_status(
        verified_ids,
        {OTA_STATUS_COMPLETE, OTA_STATUS_ERROR, OTA_STATUS_FLASH_ERROR},
        timeout_s=15.0,
        resend_cb=lambda tid: client.send_activate(tid),
        resend_interval_s=3.0,
    )

    success_count = 0
    for tid in verified_ids:
        st = activate_results.get(tid)
        if st is None:
            print(f"  Tracker {tid}: No response (may have rebooted - OK)")
            success_count += 1
        elif st["status"] == OTA_STATUS_COMPLETE:
            print(f"  Tracker {tid}: Firmware activated, rebooting!")
            success_count += 1
        else:
            print(f"  Tracker {tid}: Activation failed ({st['status_name']})")

    # Always clean up session on receiver side
    client.send_abort(0xFF)

    return success_count > 0


# ── Receiver Self-OTA ───────────────────────────────────────────────

def do_receiver_query_info(client: OTAClient) -> dict | None:
    """Query and display firmware info from the receiver itself."""
    print(f"\nQuerying receiver firmware info...")
    client.query_info(RECEIVER_OTA_ID)

    chunks = []
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and len(chunks) < 6:
        reports = client._read_ota_reports(timeout_ms=500)
        for r in reports:
            if r[0] == HID_OTA_FW_INFO and r[1] == RECEIVER_OTA_ID:
                chunks.append(r)
                print(f"  [chunk {r[2]}] data={r.hex()}")

    if not chunks:
        print("Error: No firmware info response from receiver.")
        return None

    if len(chunks) < 5:
        print(f"  Warning: Only received {len(chunks)}/6 FW_INFO chunks")

    info = OTAClient.parse_fw_info(chunks)
    print(f"\n  Firmware Version: {info['version']}")
    print(f"  Build Date:      {info['build_date']}")
    print(f"  Firmware Size:   {info['firmware_size']} bytes")
    print(f"  Bootloader:      {info['bootloader']}")
    print(f"  OTA Protocol:    v{info['protocol_version']}")
    print(f"  Board Target:    {info['board_target']}")
    print(f"  Flash Base:      0x{info['flash_base']:08X}")
    if info.get("bootloader") == "nrf5_opendfu":
        print(f"\n  ⚠ nRF5 OpenDFU bootloader: self-OTA not available (ACL flash protection).")
        print(f"    Update this receiver via SWD or DFU instead.")
    return info


def do_receiver_update(client: OTAClient, firmware: FirmwareImage,
                       board_target: str):
    """Perform OTA update on the receiver itself.

    Simpler than tracker OTA: no ESB relay, no ring buffer, direct HID transfer.
    """
    image_size = len(firmware.data)
    total_packets = (image_size + OTA_DATA_MAX_PAYLOAD - 1) // OTA_DATA_MAX_PAYLOAD

    print(f"\n{'='*60}")
    print(f"  Receiver Self-OTA Firmware Update")
    print(f"  Board Target: {board_target}")
    print(f"  Image Size:   {image_size} bytes ({image_size/1024:.1f} KB)")
    print(f"  Flash Base:   0x{firmware.base_address:08X}")
    print(f"  Packets:      {total_packets}")
    print(f"  CRC32:        0x{firmware.crc32:08X}")
    print(f"{'='*60}")

    # ── Step 1: Send BEGIN ───────────────────────────────────────
    print(f"\n[1/4] Sending BEGIN to receiver...")
    client.send_begin(RECEIVER_OTA_ID, image_size, firmware.crc32,
                      total_packets, board_target, firmware.base_address)

    # Wait for READY status
    status = client.wait_for_status(
        RECEIVER_OTA_ID,
        {OTA_STATUS_READY, OTA_STATUS_RECEIVING,
         OTA_STATUS_BOARD_MISMATCH, OTA_STATUS_SIZE_ERROR, OTA_STATUS_ERROR},
        timeout_s=10.0,
    )

    if status is None:
        print("  Error: No response from receiver")
        return False

    if status["status"] not in (OTA_STATUS_READY, OTA_STATUS_RECEIVING):
        print(f"  Error: Receiver rejected OTA ({status['status_name']})")
        return False

    print(f"  Receiver ready")

    # ── Step 2: Stream DATA packets ─────────────────────────────
    print(f"\n[2/4] Streaming firmware data to receiver...")
    start_time = time.monotonic()
    last_progress = -1
    receiver_next_seq = 0

    for seq in range(total_packets):
        offset = seq * OTA_DATA_MAX_PAYLOAD
        chunk = firmware.data[offset : offset + OTA_DATA_MAX_PAYLOAD]
        client.send_data(RECEIVER_OTA_ID, seq, chunk)

        # Read status periodically
        if (seq + 1) % 50 == 0 or seq == total_packets - 1:
            reports = client._read_ota_reports(timeout_ms=5)
            for r in reports:
                if r[0] == HID_OTA_STATUS and r[1] == RECEIVER_OTA_ID:
                    st = OTAClient.parse_status(r)
                    if st["status"] in TERMINAL_STATUSES:
                        print(f"\n  Error: {st['status_name']}")
                        return False
                    receiver_next_seq = st["next_seq"]

        # Progress display
        progress = ((seq + 1) * 100) // total_packets
        if progress != last_progress:
            elapsed = time.monotonic() - start_time
            bytes_sent = min((seq + 1) * OTA_DATA_MAX_PAYLOAD, image_size)
            speed = bytes_sent / elapsed if elapsed > 0 else 0
            bar = "█" * (progress // 5) + "░" * (20 - progress // 5)
            print(f"\r  [{bar}] {progress:3d}% "
                  f"({bytes_sent/1024:.0f}/{image_size/1024:.0f} KB) "
                  f"{speed/1024:.1f} KB/s", end="", flush=True)
            last_progress = progress

    elapsed = time.monotonic() - start_time
    print(f"\n  Transfer complete: {image_size/1024:.1f} KB in {elapsed:.1f}s "
          f"({image_size/elapsed/1024:.1f} KB/s)")

    # ── Step 3: Verify CRC32 ────────────────────────────────────
    print(f"\n[3/4] Requesting CRC32 verification...")
    time.sleep(0.5)
    client.send_verify(RECEIVER_OTA_ID)

    status = client.wait_for_status(
        RECEIVER_OTA_ID,
        {OTA_STATUS_VERIFY_OK, OTA_STATUS_VERIFY_FAIL, OTA_STATUS_ERROR},
        timeout_s=30.0,
    )

    if status is None:
        print("  Error: No verification response")
        client.send_abort(RECEIVER_OTA_ID)
        return False

    if status["status"] != OTA_STATUS_VERIFY_OK:
        print(f"  Verification FAILED ({status['status_name']})")
        client.send_abort(RECEIVER_OTA_ID)
        return False

    print(f"  CRC32 verified OK")

    # ── Step 4: Activate ────────────────────────────────────────
    print(f"\n[4/4] Activating new firmware...")
    client.send_activate(RECEIVER_OTA_ID)

    # Receiver will flash-copy and reset — USB will disconnect
    time.sleep(2.0)
    print(f"  Receiver is updating and rebooting...")
    print(f"\n{'='*60}")
    print("  Receiver OTA update completed!")
    print("  The receiver should reboot with the new firmware.")
    print(f"{'='*60}")
    return True


# ── CLI ─────────────────────────────────────────────────────────────

def parse_tracker_ids(value: str) -> list[int]:
    """Parse comma-separated tracker IDs: '0,1,2,3' → [0, 1, 2, 3]"""
    ids = []
    for part in value.split(","):
        part = part.strip()
        if "-" in part:
            # Support ranges: '0-3' → [0, 1, 2, 3]
            start, end = part.split("-", 1)
            ids.extend(range(int(start), int(end) + 1))
        else:
            ids.append(int(part))
    return sorted(set(ids))


def main():
    parser = argparse.ArgumentParser(
        description="ESB OTA Firmware Update Tool for SlimeNRF",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  %(prog)s firmware.uf2 --tracker 0
  %(prog)s firmware.uf2 --tracker 0,1,2,3      # Parallel update
  %(prog)s firmware.uf2 --tracker 0-7           # Range of trackers
  %(prog)s --info --tracker 0
  %(prog)s --abort
  %(prog)s --receiver-info                       # Query receiver firmware info
  %(prog)s firmware.uf2 --receiver-update        # OTA update the receiver itself
  %(prog)s firmware.hex --receiver-update        # Intel HEX format also supported
        """,
    )
    parser.add_argument("firmware", nargs="?", type=Path,
                        help="UF2 firmware file to flash")
    parser.add_argument("-t", "--tracker", type=str, default="0",
                        help="Target tracker ID(s): single (0), list (0,1,2), or range (0-3)")
    parser.add_argument("-b", "--board", type=str, default="",
                        help="Board target string override (e.g. promicro_uf2/nrf52840)")
    parser.add_argument("--info", action="store_true",
                        help="Query firmware info from tracker(s)")
    parser.add_argument("--abort", action="store_true",
                        help="Abort any active OTA session")
    parser.add_argument("--flash-offset", type=lambda x: int(x, 0),
                        default=0x1000,
                        help="Flash load offset (default: 0x1000)")
    parser.add_argument("-r", "--receiver", type=int, default=None,
                        help="Receiver index when multiple are connected (use --list to see)")
    parser.add_argument("--list", action="store_true",
                        help="List connected receivers and their trackers")
    parser.add_argument("--scan", action="store_true",
                        help="Scan all receivers and query firmware info from each tracker")
    parser.add_argument("--no-verify", action="store_true",
                        help="Skip CRC32 verification (not recommended)")
    parser.add_argument("--no-activate", action="store_true",
                        help="Don't activate after upload (for testing)")
    parser.add_argument("--receiver-info", action="store_true",
                        help="Query firmware info from the receiver itself")
    parser.add_argument("--receiver-update", action="store_true",
                        help="OTA update the receiver itself (not trackers)")

    args = parser.parse_args()
    tracker_ids = parse_tracker_ids(args.tracker)

    if not args.firmware and not args.info and not args.abort and not args.list and not args.scan and not args.receiver_info and not args.receiver_update:
        parser.print_help()
        sys.exit(1)

    # Enumerate receivers
    devices = enumerate_receivers()

    if args.list:
        if not devices:
            print("No SlimeNRF receivers found.")
        else:
            print(f"Found {len(devices)} receiver(s):")
            for i, d in enumerate(devices):
                serial = d.get("serial_number", "") or "n/a"
                product = d.get("product_string", "") or "unknown"
                trackers = discover_trackers(d["path"])
                print(f"  [{i}] {product}  (serial: {serial})")
                if trackers:
                    for tid in sorted(trackers):
                        info = trackers[tid]
                        addr = info["addr"]
                        online = info["online"]
                        parts = [f"Tracker {tid}"]
                        if addr:
                            parts.append(f"({addr})")
                        parts.append("online" if online else "offline")
                        print(f"       {' '.join(parts)}")
                else:
                    print(f"       No trackers detected")
        return

    if args.scan:
        if not devices:
            print("No SlimeNRF receivers found.")
            return
        print(f"Scanning {len(devices)} receiver(s)...\n")
        for i, d in enumerate(devices):
            serial = d.get("serial_number", "") or "n/a"
            product = d.get("product_string", "") or "unknown"
            trackers = discover_trackers(d["path"])
            print(f"Receiver [{i}] {product}  (serial: {serial})")
            if not trackers:
                print(f"  No trackers detected\n")
                continue

            online_ids = [tid for tid in sorted(trackers) if trackers[tid]["online"]]
            offline_ids = [tid for tid in sorted(trackers) if not trackers[tid]["online"]]

            if not online_ids:
                for tid in sorted(trackers):
                    addr = trackers[tid]["addr"]
                    print(f"  Tracker {tid} ({addr})  offline")
                print()
                continue

            # Open receiver to query firmware info for online trackers
            try:
                client = OTAClient(d["path"])
            except Exception as e:
                print(f"  Error opening receiver: {e}\n")
                continue
            for tid in online_ids:
                addr = trackers[tid]["addr"]
                info = do_query_info(client, tid, quiet=True)
                if info:
                    print(f"  Tracker {tid} ({addr})  v{info['version']}  "
                          f"{info['build_date']}  {info['board_target']}  "
                          f"flash:0x{info['flash_base']:05X}  "
                          f"{info['firmware_size']/1024:.0f}KB")
                else:
                    print(f"  Tracker {tid} ({addr})  no response")
            for tid in offline_ids:
                addr = trackers[tid]["addr"]
                print(f"  Tracker {tid} ({addr})  offline")
            client.close()
            print()
        return

    device_path = select_receiver(devices, args.receiver)

    # Open HID device
    try:
        client = OTAClient(device_path)
    except Exception as e:
        print(f"Error: Cannot open HID device (VID={VID:#06x} PID={PID:#06x}): {e}")
        print("       Make sure the receiver is connected via USB.")
        sys.exit(1)

    try:
        if args.abort:
            print("Sending OTA abort...")
            client.send_abort(0xFF)  # Abort all
            print("Abort sent.")
            return

        if args.info:
            for tid in tracker_ids:
                do_query_info(client, tid)
            return

        if args.receiver_info:
            do_receiver_query_info(client)
            return

        if args.receiver_update:
            if not args.firmware or not args.firmware.exists():
                print(f"Error: Firmware file not found: {args.firmware}")
                sys.exit(1)

            firmware = parse_firmware(args.firmware, args.flash_offset)

            # Query receiver info to get board target
            if args.board:
                board_target = args.board
            else:
                info = do_receiver_query_info(client)
                if not info or not info.get("board_target"):
                    print("Error: Cannot determine receiver board target.")
                    print("       Use --board to specify manually.")
                    sys.exit(1)
                board_target = info["board_target"]

                # Block OTA for nRF5 OpenDFU bootloader (ACL write-protects app region)
                if info.get("bootloader") == "nrf5_opendfu":
                    print("\n  ✗ Receiver has nRF5 OpenDFU bootloader — self-OTA is not supported.")
                    print("    The bootloader sets ACL flash write-protection on the app region,")
                    print("    preventing in-place firmware copy. Update via SWD or DFU instead.")
                    sys.exit(1)

                if (info.get("bootloader") == "mcuboot") != firmware.mcuboot_image:
                    print("\n  ERROR: MCUboot targets require an update BIN; other targets require UF2/HEX.")
                    sys.exit(1)

                # Validate flash base
                rcv_base = info.get("flash_base", 0)
                if rcv_base == 0 and looks_like_sd_plus_app(firmware, 0x27000):
                    # Old firmware doesn't report flash_base, but firmware is SD+App
                    rcv_base = 0x27000
                if rcv_base != 0 and firmware.base_address != rcv_base:
                    if firmware.base_address > rcv_base:
                        print(f"\n  ERROR: Firmware base 0x{firmware.base_address:08X} > receiver base 0x{rcv_base:08X}")
                        print(f"    Target firmware requires SoftDevice that is not present.")
                        print(f"    Cannot flash this firmware via OTA.")
                        sys.exit(1)
                    # firmware.base_address < rcv_base: check if SD+App or pure app
                    if looks_like_sd_plus_app(firmware, rcv_base):
                        print(f"\n  Detected SD+App image, extracting app at 0x{rcv_base:08X}...")
                        firmware = parse_firmware(args.firmware, flash_offset=rcv_base)
                    else:
                        print(f"\n  Cross-base update: firmware at 0x{firmware.base_address:08X}, receiver at 0x{rcv_base:08X}")
                        print(f"  (Transitioning from SoftDevice to no-SoftDevice firmware)")

            try:
                resp = input("\n  Proceed with receiver OTA? [y/N] ")
            except (EOFError, KeyboardInterrupt):
                print("\nAborted.")
                return

            if resp.lower() not in ("y", "yes"):
                print("Aborted.")
                return

            success = do_receiver_update(client, firmware, board_target)
            if not success:
                sys.exit(1)
            return

        # ── Firmware update ──────────────────────────────────────
        if not args.firmware or not args.firmware.exists():
            print(f"Error: Firmware file not found: {args.firmware}")
            sys.exit(1)

        # Parse firmware (UF2 or HEX)
        firmware = parse_firmware(args.firmware, args.flash_offset)

        # Query info from all targets and group by board_target
        board_groups: dict[str, list[int]] = {}
        for tid in tracker_ids:
            if args.board:
                # User override — all trackers use same board target
                board_groups.setdefault(args.board, []).append(tid)
            else:
                info = do_query_info(client, tid)
                if info and info.get("board_target"):
                    bt = info["board_target"]

                    # Block OTA for nRF5 OpenDFU bootloader trackers
                    if info.get("bootloader") == "nrf5_opendfu":
                        print(f"\n  ⚠ Tracker {tid} has nRF5 OpenDFU bootloader — OTA not supported (ACL flash protection).")
                        print(f"    Update via SWD or DFU instead. Skipping.")
                        continue

                    if (info.get("bootloader") == "mcuboot") != firmware.mcuboot_image:
                        print(f"\n  ERROR: Tracker {tid} bootloader and firmware format do not match.")
                        print("    MCUboot requires an update BIN; UF2/OpenDFU requires UF2/HEX.")
                        continue

                    # Validate flash base address
                    tracker_base = info.get("flash_base", 0)
                    if tracker_base == 0 and looks_like_sd_plus_app(firmware, 0x27000):
                        # Old firmware doesn't report flash_base, but firmware is SD+App
                        tracker_base = 0x27000
                    if tracker_base != 0 and firmware.base_address != tracker_base:
                        if firmware.base_address > tracker_base:
                            # Firmware expects SoftDevice not present — block
                            print(f"\n  ERROR: Firmware base 0x{firmware.base_address:08X} > tracker base 0x{tracker_base:08X}")
                            print(f"    Target firmware requires SoftDevice that is not present.")
                            print(f"    Skipping tracker {tid}.")
                            continue
                        # firmware.base_address < tracker_base: check if SD+App or pure app
                        if looks_like_sd_plus_app(firmware, tracker_base):
                            print(f"\n  Detected SD+App image, extracting app at 0x{tracker_base:08X}...")
                            try:
                                firmware = parse_firmware(args.firmware, flash_offset=tracker_base)
                            except ValueError as e:
                                print(f"\n  ERROR: Cannot extract app from SD+App image: {e}")
                                print(f"    Skipping tracker {tid}.")
                                continue
                        else:
                            print(f"\n  Cross-base update: firmware at 0x{firmware.base_address:08X}, tracker at 0x{tracker_base:08X}")
                            print(f"  (Transitioning from SoftDevice to no-SoftDevice firmware)")

                    board_groups.setdefault(bt, []).append(tid)
                else:
                    print(f"\n  Warning: Could not get info for tracker {tid}, skipping.")

        if not board_groups:
            print("\nError: No valid targets found.")
            sys.exit(1)

        # Show update plan
        print(f"\n{'='*60}")
        print(f"  Update Plan:")
        for bt, ids in board_groups.items():
            if "nrf52833" in bt:
                mp = 1
            elif "nrf52840" in bt:
                mp = 2
            else:
                mp = 2
            batches = (len(ids) + mp - 1) // mp
            print(f"    Board '{bt}': trackers {ids} (max {mp} parallel, {batches} batch{'es' if batches > 1 else ''})")
        if len(board_groups) > 1:
            print(f"    ({len(board_groups)} board groups, will update sequentially)")
        print(f"{'='*60}")

        try:
            resp = input("\n  Proceed? [y/N] ")
        except (EOFError, KeyboardInterrupt):
            print("\nAborted.")
            return

        if resp.lower() not in ("y", "yes"):
            print("Aborted.")
            return

        # Update each board group (different boards sequentially,
        # same board trackers in parallel)
        all_success = True
        for board_target, group_ids in board_groups.items():
            # Determine max parallel based on chip type
            if "nrf52833" in board_target:
                max_parallel = 1
            elif "nrf52840" in board_target:
                max_parallel = 2
            else:
                max_parallel = 2  # default conservative limit
            for i in range(0, len(group_ids), max_parallel):
                batch = group_ids[i:i+max_parallel]
                success = do_update(client, batch, firmware, board_target)
                if not success:
                    all_success = False
                if i + max_parallel < len(group_ids):
                    time.sleep(2.0)  # Brief pause between batches

        if all_success:
            print(f"\n{'='*60}")
            print("  OTA update completed successfully!")
            print(f"{'='*60}")
        else:
            print(f"\n{'='*60}")
            print("  OTA update completed with errors.")
            print("  Failed trackers should reboot to UF2 bootloader for recovery.")
            print(f"{'='*60}")
            sys.exit(1)

    except KeyboardInterrupt:
        print("\n\nInterrupted! Sending abort...")
        client.send_abort(0xFF)
        print("Abort sent.")
        sys.exit(1)
    finally:
        client.close()


if __name__ == "__main__":
    main()
