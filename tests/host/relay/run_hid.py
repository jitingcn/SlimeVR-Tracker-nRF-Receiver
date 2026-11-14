#!/usr/bin/env python3
"""HID parity: the receiver's actual RX path turns a relayed frame into the
same HID packets as the same frame received directly, so an unmodified
SlimeVR server sees nothing new. Runs the production functions extracted from
esb.c (sequence check, normal/composite/bundle processing, capability strip)
against a recording hid_write_packet_n()."""
from pathlib import Path as _RelayPath
import sys as _relay_sys
_relay_sys.path.insert(0, str(_RelayPath(__file__).resolve().parents[1] / 'harness'))
from relay_config import select_relay

import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "harness"))
from c_extract import extract_block  # noqa: E402

SRC = HERE.parents[2] / "src"
esb = select_relay((SRC / "connection/esb.c").read_text())
esb_h = (SRC / "connection/esb.h").read_text()


def function(name):
    return extract_block(esb, r"^static [\w \t*]+\b" + re.escape(name) + r"\s*\([^;{}]*\)\s*\{") + "\n"


def block(pattern, semicolon=True):
    return extract_block(esb, pattern, semicolon) + "\n"


constants = "\n".join(re.findall(r"^#define ESB_(?:PING|PONG|COMPOSITE)\w*[^\n]*", esb_h, re.MULTILINE))
relay_state = "\n".join(re.findall(r"^static uint(?:8|16|32)_t relay_block_(?:id|percent|mask|deadline)[^\n]*;", esb, re.MULTILINE))
stats = re.search(r"^static struct \{\n\tuint32_t bundles;.*?\} relay_rx_stats;", esb, re.MULTILINE | re.DOTALL)
if not stats:
    raise SystemExit("relay_rx_stats not found")
relay_state += "\n" + stats.group(0) + "\n"

PRELUDE = r'''
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define MAX_TRACKERS 16
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define BIT(n) (1U<<(n))
#define ARG_UNUSED(a) ((void)(a))
#define LOG_DBG(...) ((void)0)
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_ERR(...) ((void)0)
typedef long atomic_t;
typedef long atomic_val_t;
static inline long atomic_get(const atomic_t *a) { return *a; }
static inline void atomic_set(atomic_t *a, long v) { *a = v; }
static inline void atomic_and(atomic_t *a, long v) { *a &= v; }
static inline unsigned irq_lock(void) { return 0; }
static inline void irq_unlock(unsigned k) { (void)k; }
static uint32_t host_now = 1000;
static inline int64_t k_uptime_get(void) { return host_now; }
static inline uint32_t k_uptime_get_32(void) { return host_now; }
struct esb_payload { uint8_t length; uint8_t pipe; int8_t rssi; uint8_t noack; uint8_t pid; uint8_t data[72]; };
#include "tracker_event_protocol.h"
#include "relay_protocol.h"
#include "../frame_ledger.h"
#include "relay_coord.h"
static uint32_t tdma_active_mask;
static uint8_t relay_profile;
static bool relay_repair_delivery;
static uint32_t relay_session;
static struct relay_control relay_grants;
static uint32_t relay_frame_anchor, relay_start_anchor;
static uint32_t relay_origin, relay_receipt_frame[3], relay_direct_start;
static uint16_t relay_receipts[3];
static bool relay_direct_pose;
static uint8_t stored_trackers = 8;
static atomic_t test_all_state_valid;
static atomic_t test_all_confirmed_mask;
static atomic_t test_all_ready_after_ms[MAX_TRACKERS];
#define TEST_ALL_JOIN_CONFIG_GRACE_MS 1500
#define TPS_CALCULATION_INTERVAL_MS 1000
enum tdma_shadow_evidence { TDMA_SHADOW_EVIDENCE_PING, TDMA_SHADOW_EVIDENCE_DATA };
static uint32_t shadow_seen;
static void tdma_shadow_observe(uint8_t id, enum tdma_shadow_evidence e) { (void)e; shadow_seen |= 1U << id; }
static uint32_t decision_ready_mask, decision_fence_mask;
static void decision_accept(uint8_t id, uint8_t gaps) { (void)id; (void)gaps; }
static bool tracker_events_receive(const uint8_t *p, size_t n, uint32_t now) { (void)p; (void)n; (void)now; return true; }
struct hid_record { uint8_t pkt[16]; uint8_t rssi; };
static struct hid_record hid_out[32];
static int hid_count;
static void hid_write_packet_n(const uint8_t *data, uint8_t rssi)
{
	memcpy(hid_out[hid_count].pkt, data, 16);
	hid_out[hid_count].rssi = rssi;
	hid_count++;
}
#define relay_hid_packet hid_write_packet_n
#define relay_defer_capture relay_coord_capture
#define relay_defer_report relay_coord_report
static struct relay_coord relay_coord;
'''


def main():
    parts = [PRELUDE, constants,
             block(r"^struct packet_stats \{"), block(r"^struct tracker_slot \{"),
             "static struct tracker_slot tracker_slots[MAX_TRACKERS];\n",
             relay_state,
             function("packet_stats_refresh"), function("packet_stats_count"), function("check_packet_sequence"),
             function("relay_multi_blocked"), function("relay_block_drop"), function("relay_sequence_check"), function("relay_take_caps"),
             function("composite_body_length"),
             block(r"^struct composite_result\s*\{"), function("composite_validate"), function("esb_execute_composite"), function("esb_receive_composite"), function("esb_receive_normal"), function("esb_receive_bundle"), function("esb_receive_repair"), function("esb_receive_peer_report"),
             (HERE / "hid_cases.c").read_text()]
    with tempfile.TemporaryDirectory() as tmp:
        source = Path(tmp) / "hid.c"
        source.write_text("\n".join(parts))
        exe = Path(tmp) / "hid"
        cc = shlex.split(os.environ.get("CC", "cc"))
        cmd = cc + ["-std=gnu11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function", "-Wno-unused-variable",
                    "-Wno-unused-parameter", "-g", "-O1", "-fsanitize=address,undefined",
                    "-I", str(SRC / "connection"), str(source), str(SRC / "connection/relay_coord.c"), "-o", str(exe)]
        subprocess.run(cmd, check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
