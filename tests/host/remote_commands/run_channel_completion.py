#!/usr/bin/env python3
"""Force completion to race HID registration using production locking/callback code."""
from pathlib import Path as _RelayPath
import sys as _relay_sys
_relay_sys.path.insert(0, str(_RelayPath(__file__).resolve().parents[1] / 'harness'))
from relay_config import select_relay

import importlib.util
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
spec = importlib.util.spec_from_file_location("extract", HERE.parent / "tdma_loss/run.py")
extract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(extract)
source = select_relay((ROOT / "src/rcv_cmd.c").read_text())
production = "\n".join(extract.function(source, name) for name in (
    "fill_ack", "arm_ch_hid_completion", "on_tracker_channel_done"))
start = source.index("case RCV_HID_OP_TRACKER_CH_ALL:")
end = source.index("default:", start)
production += "\nstatic uint8_t hid_start(uint8_t seq, uint8_t opcode) {\n"
production += "uint8_t status = 0, data[1] = {44}; const uint8_t *args = data; size_t args_len = 1;\n"
production += "switch (opcode) {\n" + source[start:end] + "default: abort(); } return status; }\n"
fixture = (HERE / "test_channel_completion.c").read_text()
prelude, cases = fixture.split("/* PRODUCTION BODIES */")
with tempfile.TemporaryDirectory(prefix="receiver-channel-completion-") as directory:
    temporary = Path(directory)
    generated = temporary / "test.c"
    generated.write_text(prelude + production + cases)
    binary = temporary / "test"
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-pthread",
        "-g", "-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
        str(generated), "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True, timeout=10)
