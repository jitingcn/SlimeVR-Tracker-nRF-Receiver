#!/usr/bin/env python3
"""Replay production LED state/worker code with the pinned WS2812 SPI driver.

The scheduler, Zephyr synchronization objects and hardware are modeled; the
receiver and driver function bodies are compiled from the current sources.
SOURCE_ROOT selects a receiver checkout and ZEPHYR_BASE selects its SDK.
"""
import importlib.util
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("SOURCE_ROOT", HERE.parents[2]))
ZEPHYR = Path(os.environ.get(
    "ZEPHYR_BASE", (HERE.parents[3] / "sdk-nrf").resolve().parent / "zephyr"
))

spec = importlib.util.spec_from_file_location("c_extract", HERE.parent / "harness/c_extract.py")
extract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(extract)


def function(source, name):
    return extract.extract_block(
        source, rf"^[ \t]*[^\n;{{}}]*\b{re.escape(name)}\s*\([^;{{}}]*\)\s*\{{"
    )


def structure(source, name, prefix=""):
    return extract.extract_block(
        source, rf"^{re.escape(prefix)}struct\s+{re.escape(name)}\s*\{{", semicolon=True
    )


led = (ROOT / "src/system/led.c").read_text()
driver = (ZEPHYR / "drivers/led_strip/ws2812_spi.c").read_text()
api = (ZEPHYR / "include/zephyr/drivers/led_strip.h").read_text()
board = (ROOT / "boards/crazt/sk_cheesecake_recv33/sk_cheesecake_recv33_nrf52833_uf2.dts").read_text()
node = re.search(r"led_strip:\s*ws2812@0\s*\{(.*?)\};", board, re.S).group(1)


def prop(name):
    return int(re.search(rf"\b{re.escape(name)}\s*=\s*<\s*(\S+)\s*>", node).group(1), 0)


# The RECV33 node uses the binding's default eight bits per symbol and GRB.
# Use its current physical frame encodings and reset interval in the driver.
assert prop("chain-length") == 1
app = led[led.index("#if LED_EXISTS || LED_STRIP_EXISTS\nstatic enum sys_led_pattern"):]
driver_source = [
    "#if LED_STRIP_EXISTS",
    f'#include "{ZEPHYR / "include/zephyr/dt-bindings/led/led.h"}"',
    structure(api, "led_rgb"),
    "typedef int (*led_api_update_rgb)(const struct device *, struct led_rgb *, size_t);",
    "typedef int (*led_api_update_channels)(const struct device *, uint8_t *, size_t);",
    "typedef size_t (*led_api_length)(const struct device *);",
    "#define __subsystem",
    structure(api, "led_strip_driver_api", "__subsystem "),
    structure(driver, "ws2812_spi_cfg"),
    structure(driver, "ws2812_spi_data"),
]
for name in ("dev_cfg", "ws2812_spi_ser", "ws2812_reset_delay", "ws2812_spi_update_rgb",
             "ws2812_strip_update_rgb", "ws2812_strip_length"):
    driver_source.append(function(driver, name))
driver_source.append(f"""
static uint8_t pixel_buffer[24];
static const uint8_t color_mapping[] = {{LED_COLOR_ID_GREEN, LED_COLOR_ID_RED, LED_COLOR_ID_BLUE}};
static struct ws2812_spi_data wsdata;
static const struct ws2812_spi_cfg wsconfig = {{
    .px_buf = pixel_buffer, .one_frame = {prop('spi-one-frame')},
    .zero_frame = {prop('spi-zero-frame')}, .bits_per_symbol = 8,
    .num_colors = 3, .color_mapping = color_mapping, .length = 1,
    .reset_delay = {prop('reset-delay')}
}};
static const struct led_strip_driver_api wsapi = {{
    .update_rgb = ws2812_strip_update_rgb, .length = ws2812_strip_length
}};
static const struct device device = {{.name = "modeled-ws2812", .config = &wsconfig,
                                      .data = &wsdata, .api = &wsapi}};
static const struct device *const strip = &device;
""")
driver_source += [function(api, "led_strip_update_rgb"), function(led, "led_strip_set_rgb"), "#endif"]

with tempfile.TemporaryDirectory(prefix="receiver-led-lifecycle-") as directory:
    tmp = Path(directory)
    (tmp / "driver.inc").write_text("\n\n".join(driver_source) + "\n")
    (tmp / "production.inc").write_text(app)
    for variant in range(4):  # no LED, GPIO, single PWM, WS2812 SPI
        binary = tmp / f"led-{variant}"
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
            "-std=gnu11", "-Wall", "-Wextra", "-Wno-unused-function",
            "-Wno-unused-variable", "-Wno-unused-parameter", "-g", "-O1",
            "-fsanitize=undefined", "-fno-omit-frame-pointer", f"-DVARIANT={variant}",
            "-I", str(tmp), "-I", str(ROOT / "src"),
            str(HERE / "test_led.c"), "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True, timeout=10)
