"""Peer-relay support for receiver fixtures that extract esb.c functions.

The receive paths, ACK handler and TDMA recalculation consult the relay
coordinator and test hooks. Fixtures paste relay_section(esb) after their
tracker_slots/check_packet_sequence definitions (or relay_declarations(esb)
when they extract no function that needs the helpers) and compile with
-I <src/connection>. The pure coordinator is pulled in as a translation-unit
include so fixtures need no extra link inputs. It runs unmodified; with no
capable trackers it never assigns a head or forwarding slot, so direct-only
fixtures see stock behaviour.
"""
import re

from c_extract import extract_block
from relay_config import select_relay


def _function(esb, name):
    return extract_block(esb, r"^static [\w \t*]+\b" + re.escape(name) + r"\s*\([^;{}]*\)\s*\{")


def relay_declarations(esb):
    """Coordinator, published layout, test-hook state and RX statistics."""
    esb = select_relay(esb)
    state = re.findall(r"^static (?:struct relay_coord relay_coord|bool relay_user_enabled|"
                       r"uint16_t relay_forward_published|uint(?:8|16|32)_t relay_block_(?:id|percent|mask|deadline))\b[^\n]*;",
                       esb, re.MULTILINE)
    if len(state) != 7:
        raise ValueError(f"relay state declarations changed: {state}")
    stats = re.search(r"^static struct \{\n\tuint32_t bundles;.*?\} relay_rx_stats;", esb,
                      re.MULTILINE | re.DOTALL)
    if not stats:
        raise ValueError("relay_rx_stats not found")
    diag = re.search(r"^static struct \{\n\tuint8_t data\[RELAY_DIAG_LEN\];.*?\} relay_diag\[MAX_TRACKERS\];", esb,
                     re.MULTILINE | re.DOTALL)
    if not diag:
        raise ValueError("relay_diag not found")
    publication = re.search(r"^struct tdma_publication \{.*?^\} tdma_staged;", esb, re.MULTILINE | re.DOTALL)
    if not publication:
        raise ValueError("tdma_publication/tdma_staged not found")
    runtime_start = esb.index("static struct relay_runtime_settings relay_settings =")
    runtime_end = esb.index("struct tdma_publication {", runtime_start)
    runtime = esb[runtime_start:runtime_end]
    # Settings persistence is exercised separately; staged/ACK fixtures use
    # the actual runtime geometry and announcement helpers without NVS mocks.
    load_start = runtime.index("static void relay_settings_load(")
    runtime = runtime[:load_start]
    defaults = "\n".join(("#define CONFIG_RELAY_MAX_TRACKERS 16", "#define CONFIG_RELAY_MAX_REPAIRS 1", "#define CONFIG_RELAY_REPAIR_FLOOR 0",
                          "#define CONFIG_RELAY_SLOT_TICKS 16", "#define CONFIG_RELAY_TPS_MIN 100",
                          "#define CONFIG_RELAY_TPS_MAX 120"))
    return "\n".join(['#include "relay_coord.c"', '#include "relay_demand.h"', defaults, runtime, *state, stats.group(0), diag.group(0),
                      publication.group(0),
                      "#if defined(CONFIG_TDMA_DIAGNOSTICS)\nstatic uint32_t tdma_stats_generation[MAX_TRACKERS];\n#endif",
                      "#define relay_hid_packet hid_write_packet_n\n#define relay_defer_capture relay_coord_capture\n#define relay_defer_report relay_coord_report",
                      "static struct { uint32_t CYCCNT; } fixture_dwt;",
                      "#define DWT (&fixture_dwt)",
                      "static inline void relay_rx_measure(uint32_t start) { (void)start; }",
                      _function(esb, "tdma_staged_due_locked"),
                      "__attribute__((constructor)) static void relay_fixture_init(void)"
                      " { relay_coord_init(&relay_coord); }", ""])


STAGING_RECALC = ("tdma_staging_supported_locked", "tdma_apply_locked", "tdma_apply_finish")


def staging_helpers(esb, recalc=False):
    """Staged layout publication (needs tracker_slots, tdma_pack_config and
    tdma_membership_wake; with recalc, everything tdma_recalculate uses)."""
    if recalc:
        return "\n\n".join(_function(esb, name) for name in ("tdma_advertised_config",) + STAGING_RECALC) + "\n"
    # ACK-only fixtures have no stats thread: count wakeups instead.
    return "\n".join((_function(esb, "tdma_pack_config"),
                      "static unsigned tdma_membership_wake_gives;",
                      "#define k_sem_give(sem) ((void)(sem), tdma_membership_wake_gives++)",
                      "#define tdma_membership_wake tdma_membership_wake_gives",
                      _function(esb, "tdma_advertised_config"),
                      "#undef k_sem_give", "#undef tdma_membership_wake", ""))


def relay_helpers(esb, stats=True):
    """The production relay RX helpers (need tracker_slots and check_packet_sequence).
    stats=False when the fixture already extracted packet_stats_count."""
    # Fixtures extract only the receive paths they test; unused helpers are fine.
    interval = re.search(r"^#define TPS_CALCULATION_INTERVAL_MS .*$", esb, re.MULTILINE).group(0)
    return "#pragma GCC diagnostic push\n#pragma GCC diagnostic ignored \"-Wunused-function\"\n" + \
        f"#ifndef TPS_CALCULATION_INTERVAL_MS\n{interval}\n#endif\n" * stats + "\n\n".join(_function(esb, name) for name in
                       ("packet_stats_refresh", "packet_stats_count") * stats + ("relay_multi_blocked", "relay_block_drop", "relay_sequence_check", "relay_take_caps",
                        "esb_receive_relay_diag")) + "\n#pragma GCC diagnostic pop\n"


def relay_section(esb, stats=True):
    return relay_declarations(esb) + relay_helpers(esb, stats)
