/* Receiver-side relay coordinator: pure policy, no Zephyr dependencies.
 *
 * Inputs (from the ESB RX path, ISR context, caller serialises):
 *   per accepted/duplicate pose frame, whether it arrived directly or inside
 *   a head's bundle, with the sequence-check outcome and the gap count it
 *   revealed; capability bytes; bundle overflow flags; trailer deliveries.
 * Every RC_WINDOW_MS the owner thread calls relay_coord_tick(), which decides
 *   - which trackers are weak (children) and which healthy, capable trackers
 *     relay them (heads: at most RELAY_MAX_HEADS, pinned ones preferred),
 *   - when an assignment failed (pair banned, next head tried),
 *   - when a head needs a forwarding slot appended to the TDMA frame, and the
 *     two-phase withdrawal of that slot (revoke in the trailer, wait out the
 *     head lease, only then drop it from the layout).
 * Output: per-recipient PONG trailers and the forwarding mask the layout uses.
 */
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
/* Release: direct share at or above this for RC_GOOD_WINDOWS judgements and
 * the assignment older than RC_MIN_HOLD_MS. Children keep transmitting
 * directly, so a relay only adds head RX cost: prefer direct and release
 * soon; the hold only damps flapping while a body keeps turning. */
#define RC_GOOD_PERMILLE 900U
#define RC_GOOD_WINDOWS 6U
#define RC_MIN_HOLD_MS 5000U
/* A head must itself be healthy. */
#define RC_HEAD_MIN_PERMILLE 950U
_Static_assert(RC_HEAD_MIN_PERMILLE == 950U, "relay qualification threshold");
#define RC_HEAD_DROP_PERMILLE 850U
/* Assignment checks: delivery of the trailer, then relay effectiveness. */
#define RC_DELIVERY_TIMEOUT_MS 3000U
#define RC_EVAL_AFTER_MS 1000U
#define RC_EVAL_MIN_LOST 10U
#define RC_EVAL_FAIL_PERMILLE 250U
#define RC_BAN_MS 20000U
/* Children silent on every path this long are released (dormant), counted
 * from the later of their last frame and the assignment (or its delivery). */
#define RC_CHILD_SILENT_MS 3000U
/* Try other heads while an obstructed child can still hold its membership.
 * A silent radio and a disappeared tracker cannot be distinguished here. */
#define RC_CHILD_RECOVERY_MS 15000U
#define RC_SILENT_BAN_MS 2000U
#define RC_OTA_DIRECT_FRESH_MS 1500U
/* Forwarding slots. */
#define RC_FWD_OVERFLOW_WINDOWS 2U
#define RC_FWD_IDLE_WINDOWS 20U
/* Lease (RELAY_HEAD_LEASE_MS on the tracker, 2500) + one PONG period + margin. */
#define RC_FWD_REVOKE_WAIT_MS 4000U
#define RC_MAX_TOTAL_SLOTS 16U

enum rc_path { RC_DIRECT, RC_RELAYED };

/* check_packet_sequence() outcomes. */
enum rc_seq { RC_SEQ_NORMAL = 0, RC_SEQ_GAP = 1, RC_SEQ_OLD = 2, RC_SEQ_RESTART = 3, RC_SEQ_DUP = 4 };

struct rc_window {
	uint16_t direct;
	uint16_t relayed;
	uint16_t gaps;
	uint16_t direct_dup;
	uint16_t relayed_dup;
	uint16_t overflow;
	uint32_t rssi_sum;
	uint16_t rssi_count;
};

struct rc_tracker {
	bool capable;
	uint8_t protocol_version;
	uint8_t role;           /* RC_ROLE_* */
	uint8_t head;           /* child: its head */
	uint16_t children;      /* head: its children */
	uint32_t last_any_ms;
	uint32_t last_direct_ms;
	bool seen;
	bool direct_seen;
	bool relay_needed;      /* survives head loss until direct recovery or dormancy */
	uint16_t direct_permille; /* last judgement */
	uint16_t acc_direct;      /* frames carried towards the next judgement */
	uint16_t acc_expected;
	uint8_t acc_windows;
	uint16_t rate_x16;        /* frames per window x16, from non-empty windows */
	uint8_t rssi;             /* smoothed -dBm magnitude, 0 = unknown */
	/* child assignment */
	uint32_t assigned_ms;
	uint32_t delivered_ms;
	bool delivered;
	uint16_t eval_relayed;
	uint16_t eval_gaps;
	uint8_t good_windows;
	uint8_t weak_windows;   /* consecutive lossy judgements */
	/* head forwarding slot */
	uint8_t overflow_windows;
	uint8_t fwd_idle_windows;
	bool fwd_wanted;
	bool fwd_requested;     /* info flag, also works when no bundle fits */
	bool fwd_revoking;
	uint32_t fwd_revoke_ms;
};

enum { RC_ROLE_NONE, RC_ROLE_HEAD, RC_ROLE_CHILD };

struct rc_watch {
	uint32_t frame, session;
	uint16_t members, survey;
	uint16_t rescue_tried[RC_MAX];
	uint16_t r0_only[RC_MAX];
	uint16_t listeners[RC_MAX];
	uint8_t epoch;
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

struct rc_peer {
	uint16_t expected, heard;
	uint16_t rssi_x8;
	uint32_t evidence_ms;
};
struct rc_choice {
	uint8_t helper, challenger, streak;
	bool valid;
	uint32_t since_ms;
};

/* Immutable policy snapshot for the bounded freeze step. */
struct rc_grant_plan {
	struct rc_watch watch[RELAY_V3_CACHE_FRAMES];
	uint32_t children[RELAY_V3_CACHE_FRAMES][RC_MAX];
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
	uint32_t published[RC_MAX];
	uint32_t published_history[RC_MAX];
	uint16_t published_valid;
	bool enabled;
	bool v3_requested;
	uint16_t pinned;            /* manual head preference */
	uint16_t relayed_mask;      /* children delivered by relay last window */
	uint16_t forward_mask;      /* heads whose forwarding slot is in the layout */
	struct rc_tracker t[RC_MAX];
	struct rc_window w[RC_MAX];
	uint32_t ban_until[RC_MAX][RC_MAX]; /* [child][head], 0 = none */
	uint16_t pair_score[RC_MAX][RC_MAX]; /* relay share history, permille */
	uint32_t assignments;
	uint32_t rotations;
	uint32_t releases;
};


/* Independent working storage for learning and B planning. No legacy role,
 * window, ban, publication or statistics state belongs in these snapshots. */
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
	struct rc_grant_plan *p, uint32_t now_ms);


/* Background expiry; caller must serialize against report updates. */
void relay_coord_expire_peer(struct relay_coord *rc, unsigned helper,
	unsigned child, uint32_t now);
uint16_t relay_coord_peer_ratio(const struct rc_peer *p);
void relay_coord_learn_tick(struct relay_coord *rc, uint32_t now_ms);
void relay_coord_capture(struct relay_coord *rc, uint8_t helper, uint8_t child, uint8_t rssi, uint32_t now_ms);
bool relay_coord_report(struct relay_coord *rc, const struct relay_control *c, unsigned slot,
	const struct relay_peer_report *report, uint32_t now_ms);
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
void relay_coord_commit_grants(struct relay_coord *rc, const struct relay_control *control,
	const struct rc_grant_plan *plan);
void relay_coord_init(struct relay_coord *rc);
void relay_coord_note_caps(struct relay_coord *rc, uint8_t id, uint8_t caps);
void relay_coord_forget(struct relay_coord *rc, uint8_t id, uint32_t now_ms);
/* One pose frame through check_packet_sequence(); gaps = gaps it revealed. */
void relay_coord_note_frame(struct relay_coord *rc, uint8_t id, enum rc_path path,
	int seq_result, uint8_t gaps, uint8_t rssi, uint32_t now_ms);
void relay_coord_note_bundle(struct relay_coord *rc, uint8_t head, bool overflow);
/* A trailer naming children was sent to a head. */
void relay_coord_note_trailer_sent(struct relay_coord *rc, uint8_t head, uint16_t children, uint32_t now_ms);

/* member_mask: direct TDMA members. Returns the desired forwarding mask; the
 * owner re-lays-out when it differs from the published one and then calls
 * relay_coord_set_forward_layout(). */
uint16_t relay_coord_tick(struct relay_coord *rc, uint16_t member_mask, uint32_t now_ms);
void relay_coord_set_forward_layout(struct relay_coord *rc, uint16_t forward_mask);
/* Desired forwarding mask given members (bounded by RC_MAX_TOTAL_SLOTS). */
uint16_t relay_coord_forward_wanted(const struct relay_coord *rc, uint16_t member_mask);

/* Trailer for a capable recipient under the published layout. */
void relay_coord_trailer(const struct relay_coord *rc, uint8_t id, uint8_t epoch,
	uint16_t member_mask, struct relay_pong_trailer *out);
/* Forwarding slot index of head in the published layout, or RELAY_NO_SLOT. */
uint8_t relay_coord_forward_slot(const struct relay_coord *rc, uint8_t head, uint16_t member_mask);

bool relay_coord_is_relayed(const struct relay_coord *rc, uint8_t id);
/* Role teardown during another OTA must not erase the direct-link gate. */
bool relay_coord_direct_ready(const struct relay_coord *rc, uint8_t id, uint32_t now_ms);
bool relay_coord_is_capable(const struct relay_coord *rc, uint8_t id);
void relay_coord_set_enabled(struct relay_coord *rc, bool enabled);
bool relay_coord_v3_ready(const struct relay_coord *rc, uint16_t members);
void relay_coord_pin(struct relay_coord *rc, uint8_t id, bool pinned);

#endif
