#!/usr/bin/env python3
import contextlib
import io
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
with patch.dict(sys.modules, {'hid': types.ModuleType('hid')}):
    import esb_ota as ota


class Clock:
    now = 0.0

    def sleep(self, seconds):
        self.now += seconds

    def monotonic(self):
        return self.now


class HIDDevice:
    def __init__(self, clock, on_write):
        self.clock = clock
        self.on_write = on_write
        self.reads = []
        self.writes = []

    def receive(self, seconds, report):
        self.reads.append((self.clock.now + seconds, report.ljust(64, b'\x00')))
        self.reads.sort()

    def status(self, tracker, status, *, detail=0, seq=0, delay=0):
        report = bytearray(16)
        report[:3] = bytes([0xf4, tracker, status])
        report[3:5] = seq.to_bytes(2, 'big')
        report[11] = detail
        self.receive(delay, report)

    def fw_info(self, tracker, *, delay, count=6):
        info = bytearray(66)
        info[2:5] = b'\x02\x03\x04'
        info[13:15] = b'\x01\x01'
        info[15:20] = b'board'
        for idx in range(count):
            offset = 2 + 13 * idx
            self.receive(delay + idx * 0.002,
                         bytes([0xf1, tracker, idx]) + info[offset:offset + 13])

    def read(self, size, timeout_ms):
        until = self.clock.now + timeout_ms / 1000
        if self.reads and self.reads[0][0] <= until:
            at, report = self.reads.pop(0)
            self.clock.now = max(at, self.clock.now)
            return report[:size]
        self.clock.now = until
        return []

    def write(self, data):
        assert len(data) == 65 and data[0] == 0
        self.writes.append(tuple(data[1:3]))
        self.on_write(self, *self.writes[-1])


class OTAHostTests(unittest.TestCase):
    image = ota.FirmwareImage(b'firmware', 0x1000, 'board', 0x12345678)

    def setUp(self):
        self.clock = Clock()
        self.output = io.StringIO()
        for context in (contextlib.redirect_stdout(self.output), patch.object(ota, 'time', self.clock)):
            context.__enter__()
            self.addCleanup(context.__exit__, None, None, None)

    def connect(self, on_write):
        client = ota.OTAClient.__new__(ota.OTAClient)
        client.dev = HIDDevice(self.clock, on_write)
        return client

    def test_rejection_is_terminal_without_abort_or_retry(self):
        for detail in (0, 1):
            with self.subTest(detail=detail):
                client = self.connect(lambda dev, _, tid: dev.status(tid, 0x10, detail=detail))
                self.assertFalse(ota.do_update(client, [4], self.image, 'board'))
                self.assertEqual(client.dev.writes, [(0xf2, 4)])
        self.assertIn('a recent healthy direct link is required', self.output.getvalue())
        self.assertLess(self.clock.now, 1)

    def test_busy_retries_and_cleans_up_only_confirmed_target(self):
        def react(dev, cmd, tid):
            if cmd == 0xf2 and tid == 0:
                if dev.writes.count((cmd, tid)) == 1:
                    dev.status(tid, 0x00, detail=2)
                else:
                    dev.status(tid, 0x01)
            elif cmd == 0xf2:
                dev.status(tid, 0x10, detail=1)
            elif cmd == 0xf3:
                dev.status(tid, 0x02, seq=1)

        client = self.connect(react)
        self.assertFalse(ota.do_update(client, [0, 1], self.image, 'board'))
        self.assertEqual(client.dev.writes.count((0xf2, 0)), 2)
        self.assertEqual(client.dev.writes.count((0xf2, 1)), 1)
        self.assertIn('BEGIN busy (retrying)', self.output.getvalue())
        self.assertIn((0xf3, 0), client.dev.writes)
        self.assertEqual([w for w in client.dev.writes if w[0] == 0xf7], [(0xf7, 0)])

    def test_late_begin_stops_shared_data_without_aborting_unknown_target(self):
        for targets in ([0, 1], [1]):
            with self.subTest(targets=targets):
                def react(dev, cmd, tid):
                    if cmd == 0xf2 and tid == 0:
                        dev.status(tid, 0x01)
                    elif cmd == 0xf2 and dev.writes.count((cmd, tid)) == 1:
                        dev.status(tid, 0x01, delay=16)

                client = self.connect(react)
                with self.assertRaisesRegex(TimeoutError, r'tracker\(s\) 1'):
                    ota.do_update(client, targets, self.image, 'board')
                self.assertFalse(any(w[0] == 0xf3 for w in client.dev.writes))
                self.assertTrue(client.dev.reads)  # READY can still arrive after the deadline.
                self.assertEqual([w for w in client.dev.writes if w[0] == 0xf7],
                                 [(0xf7, 0)] if 0 in targets else [])

    def test_tracker_error_after_ready_does_not_abort_or_stream(self):
        def react(dev, cmd, tid):
            self.assertEqual(cmd, 0xf2)
            dev.status(tid, 0x01)
            dev.status(tid, 0x11, delay=0.01)

        client = self.connect(react)
        self.assertFalse(ota.do_update(client, [4], self.image, 'board'))
        self.assertEqual(client.dev.writes, [(0xf2, 4)])

    def test_cleanup_abort_failure_still_aborts_remaining_and_preserves_error(self):
        def react(dev, cmd, tid):
            if cmd == 0xf2 and tid in (0, 1):
                dev.status(tid, 0x01)
            elif cmd == 0xf7 and tid == 0:
                raise OSError("HID write failed")

        client = self.connect(react)
        with self.assertRaisesRegex(TimeoutError, r'tracker\(s\) 2'):
            ota.do_update(client, [0, 1, 2], self.image, 'board')
        self.assertEqual([w for w in client.dev.writes if w[0] == 0xf7],
                         [(0xf7, 0), (0xf7, 1)])
        self.assertIn('Failed to send ABORT to tracker 0', self.output.getvalue())

    def test_query_fits_two_slow_stages_and_echo(self):
        def react(dev, cmd, tid):
            self.assertEqual((cmd, tid), (0xf0, 7))
            if len(dev.writes) == 1:
                dev.fw_info(tid, delay=8 + 6.5 + 8 + 2.5)

        client = self.connect(react)
        self.assertEqual(ota.do_query_info(client, 7, quiet=True)['board_target'], 'board')
        self.assertGreaterEqual(self.clock.now, 25)
        self.assertLess(self.clock.now, 26)
        self.assertGreater(len(client.dev.writes), 4)

    def test_query_returns_first_legacy_result_without_long_wait(self):
        for chunk_count in (5, 6):
            with self.subTest(chunk_count=chunk_count):
                start = self.clock.now
                client = self.connect(lambda dev, _, tid: dev.fw_info(tid, delay=0.02, count=chunk_count))
                self.assertEqual(ota.do_query_info(client, 4, quiet=True)['version'], '2.3.4')
                self.assertEqual(len(client.dev.writes), 1)
                self.assertLess(self.clock.now - start, 1)

    def test_query_offline_has_a_single_deadline(self):
        client = self.connect(lambda *_: None)
        self.assertIsNone(ota.do_query_info(client, 4, quiet=True))
        self.assertGreaterEqual(self.clock.now, 26)
        self.assertLess(self.clock.now, 26.3)


if __name__ == '__main__':
    unittest.main()
