#ifndef SLIMENRF_RELAY_COORD_H
#define SLIMENRF_RELAY_COORD_H

#include <stdbool.h>
#include <stdint.h>

#include "relay_protocol.h"

/* Coordinator storage covers every protocol tracker ID on all chips. */
#define RC_MAX RELAY_MAX_TRACKERS
#if defined(CONFIG_RELAY_RAMFUNC) || !defined(__ZEPHYR__)
#define RELAY_RAMFUNC __ramfunc
#else
#define RELAY_RAMFUNC
#endif
#define RC_WINDOW_MS 500U
/* Any poor member can engage. Leaving needs RC_AUTO_RECOVER_MS with no member
 * weak or silent, and every member remembered as poor during the engagement
 * observed at RC_GOOD_PERMILLE or better across that window. A member joins the
 * remembered set when poor on the engaging tick, or once weak continuously
 * for RC_AUTO_ENTER_MS while engaged. */
#define RC_AUTO_ENTER_MS 1250U
#define RC_AUTO_OUTAGE_ENTER_MS 1000U
#define RC_AUTO_RECOVER_MS 1000U
#define RC_AUTO_HOLD_MS 1500U
#define RC_AUTO_REENTRY_MS 4000U
#define RC_AUTO_QUALITY_MAX_MS (RC_JUDGE_MAX_WINDOWS * RC_WINDOW_MS)

/* Weak: direct delivery below this share of expected frames (>= RC_MIN_EXPECTED
 * frames in the window) for RC_WEAK_WINDOWS consecutive windows, or direct
 * silence while the tracker is still a member. Healthy links on real hardware
 * lose 2-4% in the background, and the farthest tracker of an 11-tracker fleet
 * dips to 82-92% at idle; relaying those would cost head RX for nothing. */
#define RC_WEAK_PERMILLE 800U
#define RC_WEAK_WINDOWS 2U
#define RC_MIN_EXPECTED 20U
/* Windows with fewer expected frames (idle trackers send 10-20 per second)
 * are carried into the next judgement instead of being discarded, so weak
 * and good verdicts keep updating at idle; a partial count older than this
 * many windows is dropped. */
#define RC_JUDGE_MAX_WINDOWS 8U
/* Direct silence counts as weak this long after the last direct frame: below
 * the receiver's 15 s membership eviction, so a fully obstructed tracker gets
 * new heads (bounded by pair bans) until it is evicted. */
#define RC_SILENT_WEAK_MS 12000U
/* Direct silence is real loss only once the tracker has missed about
 * RC_MIN_EXPECTED frames at its own recent rate, and never under this. An
 * empty 500 ms window alone is normal for a still tracker, one throttled by
 * another tracker's OTA, or one rebooting, and must not force a relay. */
#define RC_SILENT_MIN_MS 500U
/* Stable recovery requires this pooled direct delivery share. */
#define RC_GOOD_PERMILLE 900U
/* Helpers must themselves meet the unchanged qualification threshold. */
#define RC_HEAD_MIN_PERMILLE 950U
_Static_assert(RC_HEAD_MIN_PERMILLE == 950U, "relay qualification threshold");
#define RC_OTA_DIRECT_FRESH_MS 1500U

enum relay_mode { RELAY_MODE_AUTO, RELAY_MODE_FORCE, RELAY_MODE_OFF };
enum rc_path { RC_DIRECT, RC_RELAYED };

/* check_packet_sequence() outcomes. */
enum rc_seq { RC_SEQ_NORMAL = 0, RC_SEQ_GAP = 1, RC_SEQ_OLD = 2, RC_SEQ_RESTART = 3, RC_SEQ_DUP = 4 };

struct rc_window {
	uint16_t direct;
	uint16_t relayed;
	uint16_t gaps;
	uint16_t direct_dup;
	uint16_t relayed_dup;

	uint32_t rssi_sum;
	uint16_t rssi_count;
};

struct rc_tracker {
	bool capable;
	uint8_t protocol_version;
	uint8_t recovery_expected; /* In-window sample count, capped at RC_MIN_EXPECTED. */
	uint32_t last_any_ms;
	uint32_t last_direct_ms;
	bool seen;
	bool direct_seen;
	uint16_t direct_permille; /* last judgement */
	uint32_t quality_ms; /* freshness of that judgement */
	uint16_t acc_direct;      /* frames carried towards the next judgement */
	uint16_t acc_expected;
	uint8_t acc_windows;
	bool direct_window_weak; /* latest stats window corroborates pooled weakness */
	uint16_t rate_x16;        /* frames per window x16, from non-empty windows */
	uint8_t rssi;             /* smoothed -dBm magnitude, 0 = unknown */
	uint8_t weak_windows;   /* consecutive lossy judgements */
};

struct rc_watch {
	uint32_t frame, session;
	uint16_t members, survey;
	uint16_t rescue_tried[RC_MAX];
	uint16_t r0_only[RC_MAX];
	uint16_t listeners[RC_MAX];
	uint8_t epoch;
	bool carried; /* Held roles, not a new control publication. */
	uint16_t primary_valid, backup_valid;
	uint8_t helpers[RC_MAX];
};
struct rc_receipt {
	uint32_t frame;
	uint16_t received;
};
/* Profile-1 PING/PONG in R0-R1: the tracker's PING interval, and the age at
 * which it displaces repairs. */
#define RC_MAINTENANCE_MS 997U
#define RC_MAINTENANCE_URGENT_MS 3000U
#define RC_LAYOUT_MAX_ACTIVATION_TICKS (UINT16_MAX * RELAY_ACTIVATION_UNIT_TICKS)
#define RC_LAYOUT_MAX_ACTIVATION_MS ((RC_LAYOUT_MAX_ACTIVATION_TICKS * 1000U + 32767U) / 32768U)

unsigned relay_stage_maintenance_stride(const struct relay_runtime_settings *applied);
uint32_t relay_stage_lead_ticks(const struct relay_runtime_settings *applied, uint8_t direct);

struct rc_peer {
	uint8_t expected, heard;
	uint16_t rssi_x8;
	uint32_t evidence_ms;
};
_Static_assert(sizeof(struct rc_peer) == 8, "compact directed peer evidence");
struct rc_choice {
	uint8_t helper, challenger, streak;
	bool valid;
	uint32_t since_ms;
};

/* Immutable policy snapshot for the bounded freeze step. */
struct rc_grant_plan {
	struct rc_watch watch[RELAY_CACHE_FRAMES];
	uint32_t children[RELAY_CACHE_FRAMES][RC_MAX];
	uint32_t candidates[3];
	uint8_t helper[3][32];
	uint32_t now_ms;
	uint16_t survey, rescue_child, fresh_survey;
	uint16_t next_listeners[RC_MAX];
	uint8_t rescue_helper, rescue_second, rescue_age;
	int8_t maintenance, reporter;
	uint8_t survey_next, valid;
	uint8_t receipt_index[RELAY_CACHE_MAX_FRAMES];
	bool urgent;
	uint16_t emergency_child, emergency_coverage;
	uint8_t emergency_helper;
	uint16_t demand_eligible[5];
	uint32_t fresh_candidates, fresh_coverage[RC_MAX];
	uint32_t planned_fresh;
	uint8_t fresh_helper[2 * RELAY_MAX_TRACKERS];
	uint32_t demand_candidates[5];
	uint32_t demand_children[5][RC_MAX];
	uint8_t demand_helper[5][32];
	struct rc_watch watch_extra[2];
};

static inline struct rc_watch *relay_plan_watch(struct rc_grant_plan *p, unsigned age)
{
	return age < 3 ? &p->watch[age] : &p->watch_extra[age - 3];
}
static inline const struct rc_watch *relay_plan_watch_const(const struct rc_grant_plan *p, unsigned age)
{
	return age < 3 ? &p->watch[age] : &p->watch_extra[age - 3];
}

struct relay_coord {
	bool auto_engaged, auto_applied;
	uint16_t auto_members, auto_poor_members;
	uint32_t auto_good_since, auto_applied_since, auto_native_since;
	/* Per member, over the recovery window: sum of 10 * direct - 9 * expected
	 * frames of each stats interval, clamped to int16. Until it first clamps,
	 * >= 0 means exactly pooled >= 900; a clamped sum keeps its sign at the
	 * clamp but may have a different sign from the unclamped sum. */
	int16_t auto_balance[RC_MAX];
	/* Per member: start of its current weak or outage streak, 0 = neither. */
	uint32_t auto_poor_since[RC_MAX];
	uint16_t auto_outage_members;
	bool auto_exit_pending;
	uint8_t cache_frames;
	struct rc_peer peer[RC_MAX][RC_MAX]; /* helper -> child */
	struct rc_choice choice[RC_MAX][2];
	uint16_t learn_members;
	uint8_t survey_next, report_next;
	uint32_t report_ms[RC_MAX];
	struct rc_watch watch[RELAY_CACHE_MAX_FRAMES];
	struct rc_receipt receipt[RELAY_CACHE_MAX_FRAMES];
	uint32_t maintenance_ms[RC_MAX];
	uint8_t maintenance_next;
	uint16_t native_test_members;
	uint32_t published[RC_MAX];
	uint32_t published_history[RC_MAX];
	uint16_t published_valid;
	bool enabled;
	struct rc_tracker t[RC_MAX];
	struct rc_window w[RC_MAX];
};

/* Independent working storage for learning and control frame planning. Publication and statistics state do not belong in these snapshots. */
struct rc_link_input {
	uint16_t direct_permille;
	uint8_t rssi;
};
struct rc_learn_snapshot {
	struct rc_peer peer[RC_MAX][RC_MAX];
	struct rc_choice choice[RC_MAX][2];
	struct rc_link_input t[RC_MAX];
	uint16_t learn_members;
};
struct rc_plan_snapshot {
	struct rc_learn_snapshot learn;
	struct rc_watch watch[RELAY_CACHE_MAX_FRAMES];
	struct rc_receipt receipt[RELAY_CACHE_MAX_FRAMES];
	uint32_t maintenance_ms[RC_MAX], report_ms[RC_MAX];
	uint8_t maintenance_next, report_next, survey_next, cache_frames;
};
void relay_learn_snapshot_tick(struct rc_learn_snapshot *rc, uint32_t now_ms);
void relay_plan_snapshot_watch(struct rc_plan_snapshot *rc, struct relay_control *c);
void relay_plan_snapshot_grants(struct rc_plan_snapshot *rc, struct relay_control *c,
	struct rc_grant_plan *p, uint32_t now_ms, bool stage_maintenance);

/* Background expiry; caller must serialize against report updates. */
void relay_coord_expire_peer(struct relay_coord *rc, unsigned helper,
	unsigned child, uint32_t now);
uint16_t relay_coord_peer_ratio(const struct rc_peer *p);
void relay_coord_learn_tick(struct relay_coord *rc, uint32_t now_ms);
void relay_coord_capture(struct relay_coord *rc, uint8_t helper, uint8_t child, uint8_t rssi, uint32_t now_ms);
/* Check the receive-time grant before applying any decoded observations. */
bool relay_coord_report_granted(const struct relay_control *grant, unsigned slot,
	const struct relay_peer_report *report);
/* The helper comes from an authorized report. */
void relay_coord_observe(struct relay_coord *rc, uint8_t helper, uint8_t child,
	uint8_t expected, uint8_t heard, uint8_t rssi, uint32_t now_ms);
void relay_coord_fast_reset(struct relay_coord *rc);
/* Record validated RF receipt before deferred publication. */
void relay_coord_receive(struct relay_coord *rc, uint8_t id, uint32_t frame);
bool relay_coord_was_published(const struct relay_coord *rc, uint8_t id, uint32_t frame);
bool relay_coord_publish(struct relay_coord *rc, uint8_t id, uint32_t frame);
/* Frames skipped since the last publication; call before relay_coord_publish(). */
uint8_t relay_coord_skipped(const struct relay_coord *rc, uint8_t id, uint32_t frame);
void relay_coord_plan_watch(struct relay_coord *rc, struct relay_control *control);
void relay_coord_plan_grants(struct relay_coord *rc, struct relay_control *c,
	struct rc_grant_plan *plan, uint32_t now_ms);
void relay_coord_finish_static_frozen(struct relay_control *c, const struct rc_grant_plan *p, uint8_t *wire);
void relay_coord_finish_frozen(struct relay_control *c, const struct rc_grant_plan *plan, uint8_t *wire);
void relay_coord_read_grants(struct relay_control *c, const uint8_t *wire);
/* Fill missing capture watches from published roles, before taking the plan snapshot. */
void relay_coord_fill_watch_gaps(struct relay_coord *rc, const struct relay_control *control);
/* Commit only after the first control copy has completed transmission. */
void relay_coord_commit_grants(struct relay_coord *rc, const struct relay_control *control,
	const struct rc_grant_plan *plan);
void relay_coord_init(struct relay_coord *rc);
void relay_coord_note_caps(struct relay_coord *rc, uint8_t id, uint8_t caps);
void relay_coord_forget(struct relay_coord *rc, uint8_t id, uint32_t now_ms);
/* One pose frame through check_packet_sequence(); gaps = gaps it revealed. */
void relay_coord_note_frame(struct relay_coord *rc, uint8_t id, enum rc_path path,
	int seq_result, uint8_t gaps, uint8_t rssi, uint32_t now_ms);

void relay_coord_auto_update(struct relay_coord *rc, uint16_t members, uint32_t now, bool applied, bool eligible);
void relay_coord_auto_native(struct relay_coord *rc, uint32_t now);
void relay_coord_fence_native(struct relay_coord *rc, uint32_t now);
void relay_coord_tick(struct relay_coord *rc, uint16_t members, uint32_t now);

/* Trailer for a capable recipient under the published layout. */
void relay_coord_trailer(const struct relay_coord *rc, uint8_t id, uint8_t epoch,
	uint16_t member_mask, struct relay_pong_trailer *out);

/* OTA must use fresh direct-link evidence. */
bool relay_coord_direct_ready(const struct relay_coord *rc, uint8_t id, uint32_t now_ms);
bool relay_coord_is_capable(const struct relay_coord *rc, uint8_t id);
void relay_coord_set_enabled(struct relay_coord *rc, bool enabled);
bool relay_coord_ready(const struct relay_coord *rc, uint16_t members);
/* Private profile requests also honor native-test intent in the selected roster. */
bool relay_coord_requested(const struct relay_coord *rc, enum relay_mode mode, uint16_t members);
/* Every member runs relay firmware, so an applied profile 1 can run. Permission does not gate an already applied profile; relay_coord_ready adds it. */
bool relay_coord_capable(const struct relay_coord *rc, uint16_t members);

#endif
