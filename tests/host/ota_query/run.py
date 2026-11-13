#!/usr/bin/env python3
"""Exercise the production OTA query through a deterministic HID boundary."""

import importlib.util
import io
from contextlib import redirect_stdout
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[3]
TRACKER_ID = 3
BOARD = "promicro_uf2/nrf52840"


def firmware_reports(tracker_id):
    # The receiver maps ESB bytes 2..65 into 13-byte HID chunks. The sixth
    # chunk is padding, but is still part of the six-report response.
    packet = bytearray(66)
    packet[2:5] = bytes((1, 2, 3))
    packed_date = (6 << 25) | (11 << 21) | (13 << 16) | (5 << 11) | (42 << 5) | 15
    struct.pack_into(">I", packet, 5, packed_date)
    struct.pack_into("<I", packet, 9, 393216)
    packet[13:15] = bytes((1, 1))
    board = BOARD.encode("utf-8")
    packet[15:15 + len(board)] = board
    struct.pack_into(">H", packet, 63, 0x27)

    return [
        bytes((0xF1, tracker_id, index))
        + bytes(packet[2 + 13 * index:2 + 13 * (index + 1)]).ljust(13, b"\0")
        for index in range(6)
    ]


class VirtualClock:
    def __init__(self):
        self.now = 0.0
        self.calls = 0

    def monotonic(self):
        self.calls += 1
        if self.now > 30 or self.calls > 100000:
            raise AssertionError("OTA query exceeded the virtual time bound")
        return self.now

    def sleep(self, seconds):
        if seconds < 0:
            raise AssertionError("negative sleep")
        self.now += seconds


class HidDevice:
    def __init__(self, clock, scenario):
        self.clock = clock
        self.scenario = scenario
        self.opened = False
        self.nonblocking = False
        self.query_count = 0
        self.replied = False
        self.events = []

    def open_path(self, path):
        self.opened = True

    def open(self, vendor_id, product_id):
        self.opened = True

    def set_nonblocking(self, enabled):
        self.nonblocking = enabled

    def get_product_string(self):
        return "virtual receiver"

    def close(self):
        self.opened = False

    def queue_frame(self, delay, reports):
        if not 1 <= len(reports) <= 4 or any(len(report) != 16 for report in reports):
            raise AssertionError("invalid HID fixture frame")
        frame = b"".join(reports).ljust(64, b"\0")
        self.events.append((self.clock.now + delay, frame))
        self.events.sort(key=lambda event: event[0])

    def queue_complete_reply(self, tracker_id):
        reports = firmware_reports(tracker_id)
        self.queue_frame(0.01, reports[:4])
        self.queue_frame(0.02, reports[4:])

    def write(self, data):
        if not self.opened or len(data) != 65 or data[:3] != bytes((0, 0xF0, TRACKER_ID)):
            raise AssertionError("invalid OTA query HID report")
        self.query_count += 1
        if self.replied or self.scenario == "timeout":
            return len(data)
        if self.scenario == "lost_first" and self.query_count == 1:
            return len(data)

        self.replied = True
        if self.scenario == "duplicates":
            reports = firmware_reports(TRACKER_ID)
            self.queue_frame(0.01, [reports[0]] * 4)
            self.queue_frame(0.02, [reports[0]] * 2)
            # Delay unique chunks past the old collector's first 500 ms read.
            self.queue_frame(0.80, reports[1:5])
            self.queue_frame(0.81, reports[5:])
        elif self.scenario == "other_tracker":
            self.queue_complete_reply(TRACKER_ID + 1)
        else:
            self.queue_complete_reply(TRACKER_ID)
        return len(data)

    def read(self, size, timeout_ms):
        if not self.opened or size != 64 or timeout_ms < 1:
            raise AssertionError("invalid HID read")
        deadline = self.clock.now + timeout_ms / 1000
        if self.events and self.events[0][0] <= deadline:
            when, frame = self.events.pop(0)
            self.clock.now = max(self.clock.now, when)
            return list(frame)
        self.clock.now = deadline
        return []


def load_ota(device):
    fake_hid = types.ModuleType("hid")
    fake_hid.device = lambda: device
    name = "_slimenrf_ota_query_regression"
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts/esb_ota.py")
    module = importlib.util.module_from_spec(spec)
    # Dataclass decoration resolves the module by name during import. Restore
    # both entries afterward, including any pre-existing hid installation.
    with patch.dict(sys.modules, {"hid": fake_hid, name: module}), patch.object(
        sys, "dont_write_bytecode", True
    ):
        spec.loader.exec_module(module)
    return module


class OtaQueryTests(unittest.TestCase):
    def query(self, scenario):
        clock = VirtualClock()
        device = HidDevice(clock, scenario)
        ota = load_ota(device)
        # Replace the imported module's clock, not the process-wide time module.
        with patch.object(ota, "time", clock), redirect_stdout(io.StringIO()):
            client = ota.OTAClient(b"virtual-receiver")
            try:
                result = ota.do_query_info(client, TRACKER_ID, quiet=True)
            finally:
                client.close()
        return result, device

    def assert_complete_info(self, info):
        self.assertIsNotNone(info)
        self.assertEqual(info["version"], "1.2.3")
        self.assertEqual(info["build_date"], "2026-11-13 05:42:30")
        self.assertEqual(info["firmware_size"], 393216)
        self.assertEqual(info["bootloader"], "adafruit_uf2")
        self.assertEqual(info["protocol_version"], 1)
        self.assertEqual(info["board_target"], BOARD)
        self.assertEqual(info["flash_base"], 0x27000)

    def test_complete_reply(self):
        info, device = self.query("healthy")
        self.assertGreaterEqual(device.query_count, 1)
        self.assert_complete_info(info)

    def test_lost_first_reply_is_requeried(self):
        info, device = self.query("lost_first")
        self.assertGreaterEqual(device.query_count, 2)
        self.assert_complete_info(info)

    def test_duplicate_chunks_do_not_complete_the_reply(self):
        info, _ = self.query("duplicates")
        self.assert_complete_info(info)

    def test_other_tracker_does_not_satisfy_query(self):
        info, _ = self.query("other_tracker")
        self.assertIsNone(info)

    def test_no_reply_times_out(self):
        info, _ = self.query("timeout")
        self.assertIsNone(info)


if __name__ == "__main__":
    unittest.main()
