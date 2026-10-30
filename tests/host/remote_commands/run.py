#!/usr/bin/env python3
"""Exercise remote calibration parser boundaries without USB or radio hardware."""
import importlib.util
import contextlib
import io
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import types
import unittest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.modules.setdefault("hid", types.ModuleType("hid"))
spec = importlib.util.spec_from_file_location("hid_cmd", ROOT / "scripts/hid_cmd.py")
sender = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sender)


class HeatedSend(unittest.TestCase):
    def test_start_target_and_alias(self):
        for target, tid in (("0", 0), ("7", 7), ("all", 255)):
            for tokens in (["tcal", "heat", "start"], ["tcal-heat-start"]):
                with self.subTest(target=target, tokens=tokens):
                    self.assertEqual(sender.build_send(target, tokens), (0x27, bytes([tid])))

    def test_reject_unsupported_heat_requests(self):
        for tokens in (
            ["tcal", "heat"], ["tcal", "heat", "stop"],
            ["tcal", "heat", "status"], ["tcal", "heat", "44"],
            ["tcal", "heat", "start", "44"],
            ["tcal", "heat", "start", "nan"],
            ["tcal-heat-start", "44"], ["tcal-heat-start", "stop"],
        ):
            for target in ("0", "all"):
                with self.subTest(target=target, tokens=tokens):
                    with self.assertRaises(ValueError):
                        sender.build_send(target, tokens)


class CalibrationSend(unittest.TestCase):
    def test_calibration_operation_and_target(self):
        for target, tid in (("0", 0), ("7", 7), ("all", 255)):
            for tokens, opcode in (
                (["calibrate"], 0x02),
                (["calibrate", "acc"], 0x03),
                (["6-side"], 0x03),
            ):
                with self.subTest(target=target, tokens=tokens):
                    self.assertEqual(sender.build_send(target, tokens), (opcode, bytes([tid])))

    def test_reject_invalid_calibration_arguments(self):
        for target in ("0", "all"):
            for tokens in (
                ["calibrate", "gyro"], ["calibrate", "acc", "extra"],
                ["calibrate", "acc", "extra", "more"], ["6-side", "extra"],
            ):
                with self.subTest(target=target, tokens=tokens):
                    with self.assertRaises(ValueError):
                        sender.build_send(target, tokens)


class WomSend(unittest.TestCase):
    def test_enter(self):
        for target, tid in (("0", 0), ("7", 7), ("all", 255)):
            self.assertEqual(sender.build_send(target, ["wom", "enter"]),
                             (0x28, bytes([tid, 0xfe, 0xff])))
        for rest in (["enter", "0"], ["enter", "extra"], ["enter", "1", "extra"]):
            with self.assertRaises(ValueError):
                sender.build_send("all", ["wom"] + rest)

    def test_enter_result_rendering(self):
        result = bytes([1, 3, 0xfe, 0xff, 0, 0, 9, 0, 0, 0, 0, 0])
        with contextlib.redirect_stdout(io.StringIO()) as captured:
            sender.print_wom_result(result)
        self.assertIn("WOM entry accepted", captured.getvalue())
        self.assertNotIn("minutes", captured.getvalue())
        self.assertNotIn("asleep", captured.getvalue())
        error = bytearray(result)
        error[4] = 16
        with contextlib.redirect_stdout(io.StringIO()) as captured:
            sender.print_wom_result(bytes(error))
        self.assertIn("error 16", captured.getvalue())
        self.assertNotIn("accepted", captured.getvalue())

    def test_query_set_and_clear(self):
        for target, tid in (("0", 0), ("all", 255)):
            for rest, value in (([], 0), (["0"], 0xffff), (["1"], 1), (["35791"], 35791)):
                self.assertEqual(sender.build_send(target, ["wom", "timeout"] + rest),
                                 (0x28, bytes([tid]) + value.to_bytes(2, "little")))

    def test_invalid_minutes(self):
        for rest in ([], ["off"], ["timeout", "1", "extra"]):
            with self.assertRaises(ValueError):
                sender.build_send("all", ["wom"] + rest)
        for value in ("", "-1", "+1", "1.5", "1m", "35792", "65534", "65535", "９", "99999999999999999999"):
            with self.assertRaises(ValueError):
                sender.build_send("0", ["wom", "timeout", value])

    def test_decode_and_wait_all_results(self):
        result = bytes([1, 3, 10, 0, 0, 0, 9, 0, 0, 0, 0, 0])
        error = bytes([1, 4, 0, 0, 110, 0, 9, 0, 0, 0, 0, 0])
        summary = bytes([2, 9, 0, 8, 0, 0, 0, 16, 0, 0, 0, 0])
        self.assertEqual(sender.decode_wom_result(result),
                         {"tracker_id": 3, "minutes": 10, "error": 0, "token": 9})
        self.assertEqual(sender.decode_wom_result(error)["error"], 110)
        self.assertEqual(sender.decode_wom_result(summary),
                         {"token": 9, "success_mask": 8, "failure_mask": 16})
        client = object.__new__(sender.HidCmdClient)
        client.acks = {}
        client._write = lambda packet: None
        replies = iter(((7, bytes(12)), (7, result), (7, error), (4, summary)))
        client._wait_ack = lambda seq, opcode, timeout: next(replies)
        self.assertEqual(client.command(0x28, b"\xff\0\0", wait_completion=True),
                         (4, summary))

    def test_clear_result_rendering(self):
        result = bytes([1, 3, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0])
        error = bytes([1, 3, 0, 0, 5, 0, 9, 0, 0, 0, 0, 0])
        with contextlib.redirect_stdout(io.StringIO()) as captured:
            sender.print_wom_result(result)
        self.assertIn("restored firmware default", captured.getvalue())
        self.assertNotIn("0 minutes", captured.getvalue())
        with contextlib.redirect_stdout(io.StringIO()) as captured:
            sender.print_wom_result(error)
        self.assertIn("error 5", captured.getvalue())
        self.assertNotIn("restored", captured.getvalue())



if __name__ == "__main__":
    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    if not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful():
        raise SystemExit(1)
    with tempfile.TemporaryDirectory(prefix="receiver-remote-commands-") as directory:
        temporary = Path(directory)
        (temporary / "zephyr/sys").mkdir(parents=True)
        (temporary / "esb.h").write_text("struct esb_evt;\n")
        (temporary / "zephyr/sys/printk.h").write_text("#define printk(...) ((void)0)\n")
        extract_spec = importlib.util.spec_from_file_location("extract", HERE.parent / "tdma_loss/run.py")
        extract = importlib.util.module_from_spec(extract_spec)
        extract_spec.loader.exec_module(extract)
        (temporary / "console_u8.inc").write_text(
            extract.function((ROOT / "src/console.c").read_text(), "parse_u8_arg"))
        binary = temporary / "console-send"
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
            "-g", "-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            "-fno-pie", "-no-pie", "-I", str(temporary), "-I", str(ROOT / "src"),
            str(ROOT / "src/console_send.c"), str(ROOT / "src/parse_args.c"),
            str(HERE / "test_console_send.c"),
            "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
        source = (ROOT / "src/rcv_cmd.c").read_text()
        production = "\n".join(extract.function(source, name) for name in (
            "fill_ack", "remote_hid_abort", "remote_hid_begin", "remote_wom_result",
            "remote_wom_poll", "remote_wom_begin", "rcv_cmd_remote_wom_command",
            "on_remote_confirm"))
        start = source.index("if (opcode == ESB_PONG_FLAG_WOM_CONTROL)")
        end = source.index("} else if (opcode == ESB_PONG_FLAG_SENS_SET)", start)
        production += ("\nstatic uint8_t hid_wom(const uint8_t *buf, size_t args_len) {\n"
                       "uint8_t seq=buf[1], opcode=buf[2], target=buf[4], status;\n"
                       "const uint8_t *args = &buf[4];\n" + source[start:end] +
                       "} else { assert(false); } return status; }\n")
        prelude, cases = (HERE / "test_wom_completion.c").read_text().split("/* PRODUCTION BODIES */")
        generated = temporary / "wom.c"
        generated.write_text(prelude + production + cases)
        binary = temporary / "wom"
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
            "-g", "-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            "-fno-pie", "-no-pie", "-I", str(temporary), "-I", str(ROOT / "src"),
            str(generated), "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
