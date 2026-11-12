#!/usr/bin/env python3
"""Compile unchanged production console input/lifecycle bodies against host leaves."""

from pathlib import Path as _RelayPath
import sys as _relay_sys
_relay_sys.path.insert(0, str(_RelayPath(__file__).resolve().parents[1] / 'harness'))
from relay_config import select_relay

import os
from pathlib import Path
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
SRC = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2])) / "src"
source = select_relay((SRC / "console.c").read_text())
declarations = source[source.index("#define CONSOLE_LINE_QUEUE_DEPTH"):source.index("#define DFU_EXISTS")]
functions = source[source.index("static bool console_echo_has_data_locked(void)"):source.index("static void console_thread(void)\n{")]

with tempfile.TemporaryDirectory(prefix="receiver-console-lifecycle-") as directory:
    temporary = Path(directory)
    (temporary / "production.inc").write_text(declarations + "\n" + functions)
    binary = temporary / "console-lifecycle"
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
        "-g", "-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-fno-pie", "-no-pie", "-I", str(temporary), "-I", str(SRC),
        str(HERE / "test_console.c"), "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)
