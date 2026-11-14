/* Host tests: relay wire format and the receiver relay coordinator policy. */
#include "relay_coord.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { if (!(cond)) { fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

static uint8_t normal_frame(uint8_t *out, uint8_t id, uint8_t seq)
{
	memset(out, 0, 17);
	out[0] = 1; /* quat+accel */
	out[1] = id;
	for (int i = 2; i < 16; i++) {
		out[i] = (uint8_t)(i * 7 + id);
	}
	out[16] = seq;
	return 17;
}

static void test_trailer(void)
{
	struct relay_pong_trailer in = {.epoch = 9, .children_mask = 0x0104, .forward_slot = 11,
		.member_mask = 0x07FF, .relayed_mask = 0x0004};
	uint8_t wire[RELAY_TRAILER_LEN];
	relay_trailer_encode(wire, &in);
	struct relay_pong_trailer out;
	CHECK(relay_trailer_decode(wire, &out));
	CHECK(out.epoch == 9 && out.children_mask == 0x0104 && out.forward_slot == 11);
	CHECK(out.member_mask == 0x07FF && out.relayed_mask == 0x0004);
	for (unsigned i = 0; i < RELAY_TRAILER_LEN; i++) {
		uint8_t bad[RELAY_TRAILER_LEN];
		memcpy(bad, wire, sizeof(bad));
		bad[i] ^= 0x10;
		CHECK(!relay_trailer_decode(bad, &out));
	}
	CHECK(RELAY_PONG_EXT_LEN == 23);
	CHECK(relay_trailer_encode(wire, &in) == RELAY_TRAILER_LEN);

	/* Pending block (staged layout activation): 17 bytes, own magic. */
	struct relay_pong_trailer pend = in;
	pend.pending = true;
	pend.pending_epoch = 10;
	pend.pending_total = 12;
	pend.pending_slot_ticks = 16;
	pend.pending_member_mask = 0x0FFF;
	pend.activation_units = 0xABCD;
	uint8_t wide[RELAY_TRAILER_PENDING_LEN];
	CHECK(relay_trailer_encode(wide, &pend) == RELAY_TRAILER_PENDING_LEN);
	CHECK(relay_trailer_decode_len(wide, sizeof(wide), &out));
	CHECK(out.pending && out.epoch == 9 && out.member_mask == 0x07FF && out.forward_slot == 11);
	CHECK(out.pending_epoch == 10 && out.pending_total == 12 && out.pending_slot_ticks == 16);
	CHECK(out.pending_member_mask == 0x0FFF && out.activation_units == 0xABCD);
	for (unsigned i = 0; i < RELAY_TRAILER_PENDING_LEN; i++) {
		uint8_t bad[RELAY_TRAILER_PENDING_LEN];
		memcpy(bad, wide, sizeof(bad));
		bad[i] ^= 0x10;
		CHECK(!relay_trailer_decode_len(bad, sizeof(bad), &out));
	}
	/* Lengths and magics must agree; the plain decoder rejects the pending form. */
	CHECK(!relay_trailer_decode_len(wide, RELAY_TRAILER_LEN, &out));
	uint8_t padded[RELAY_TRAILER_PENDING_LEN] = {0};
	memcpy(padded, wire, RELAY_TRAILER_LEN);
	CHECK(!relay_trailer_decode_len(padded, RELAY_TRAILER_PENDING_LEN, &out));
	CHECK(!relay_trailer_decode(wide, &out));
	CHECK(relay_trailer_decode_len(wire, RELAY_TRAILER_LEN, &out) && !out.pending);
	CHECK(RELAY_PONG_EXT_PENDING_LEN == 30);
	CHECK(relay_pong_is_extended(23) && relay_pong_is_extended(30));
	CHECK(!relay_pong_is_extended(13) && !relay_pong_is_extended(24));
}

static void test_member_slot(void)
{
	CHECK(relay_member_slot(0x002D, 0) == 0); /* members 0,2,3,5 */
	CHECK(relay_member_slot(0x002D, 2) == 1);
	CHECK(relay_member_slot(0x002D, 3) == 2);
	CHECK(relay_member_slot(0x002D, 5) == 3);
	CHECK(relay_member_slot(0x002D, 1) == -1);
	CHECK(relay_member_slot(0x002D, 16) == -1);
}

static void test_bundle(void)
{
	uint8_t own[17], c1[17], c2[17], c3[17], buf[RELAY_FRAME_MAX];
	normal_frame(own, 1, 10);
	normal_frame(c1, 4, 20);
	normal_frame(c2, 6, 30);
	normal_frame(c3, 7, 40);

	/* Own frame already in the output buffer (esb.c builds in place). */
	memcpy(buf, own, 17);
	struct relay_bundle_builder b;
	CHECK(relay_bundle_begin(&b, buf, sizeof(buf), 1, buf, 17, false));
	CHECK(relay_bundle_add(&b, c1, 17, 55));
	CHECK(relay_bundle_add(&b, c2, 17, 60));
	CHECK(!relay_bundle_add(&b, c3, 17, 61)); /* 3+17+3*19 = 77 > 72 */
	size_t len = relay_bundle_finish(&b);
	CHECK(len == 3 + 17 + 2 * 19);

	struct relay_bundle_view v;
	CHECK(relay_bundle_decode(buf, (uint8_t)len, &v));
	CHECK(v.head_id == 1 && !v.overflow && v.own_len == 17 && memcmp(v.own, own, 17) == 0);
	CHECK(v.record_count == 2);
	CHECK(v.records[0].length == 17 && v.records[0].rssi == 55 && memcmp(v.records[0].frame, c1, 17) == 0);
	CHECK(v.records[1].rssi == 60 && memcmp(v.records[1].frame, c2, 17) == 0);

	/* Relay-only bundle with three records and the overflow flag. */
	CHECK(relay_bundle_begin(&b, buf, sizeof(buf), 1, NULL, 0, true));
	CHECK(relay_bundle_add(&b, c1, 17, 50));
	CHECK(relay_bundle_add(&b, c2, 17, 51));
	CHECK(relay_bundle_add(&b, c3, 17, 52));
	len = relay_bundle_finish(&b);
	CHECK(len == 3 + 3 * 19);
	CHECK(relay_bundle_decode(buf, (uint8_t)len, &v));
	CHECK(v.overflow && v.own_len == 0 && v.record_count == 3);

	/* No record: nothing to send as a bundle. */
	CHECK(relay_bundle_begin(&b, buf, sizeof(buf), 1, own, 17, false));
	CHECK(relay_bundle_finish(&b) == 0);

	/* Rejections: truncated, head forwarding itself, duplicate child,
	 * non-pose payloads (PING, raw, event), bad own frame. */
	CHECK(relay_bundle_begin(&b, buf, sizeof(buf), 1, NULL, 0, false));
	CHECK(relay_bundle_add(&b, c1, 17, 50));
	len = relay_bundle_finish(&b);
	CHECK(!relay_bundle_decode(buf, (uint8_t)(len - 1), &v));
	uint8_t self[17];
	normal_frame(self, 1, 3);
	CHECK(relay_bundle_begin(&b, buf, sizeof(buf), 1, NULL, 0, false));
	CHECK(relay_bundle_add(&b, self, 17, 50));
	CHECK(!relay_bundle_decode(buf, (uint8_t)relay_bundle_finish(&b), &v));
	CHECK(relay_bundle_begin(&b, buf, sizeof(buf), 1, NULL, 0, false));
	CHECK(relay_bundle_add(&b, c1, 17, 50));
	CHECK(relay_bundle_add(&b, c1, 17, 50));
	CHECK(!relay_bundle_decode(buf, (uint8_t)relay_bundle_finish(&b), &v));
	uint8_t ping[17];
	normal_frame(ping, 4, 1);
	ping[0] = 0xF0;
	CHECK(relay_bundle_begin(&b, buf, sizeof(buf), 1, NULL, 0, false));
	CHECK(relay_bundle_add(&b, ping, 17, 50));
	CHECK(!relay_bundle_decode(buf, (uint8_t)relay_bundle_finish(&b), &v));
	uint8_t event[17];
	normal_frame(event, 4, 1);
	event[0] = 0xE0;
	CHECK(!relay_frame_is_pose(event, 17, 4));
	uint8_t raw[17];
	normal_frame(raw, 4, 1);
	raw[0] = 0x10;
	CHECK(!relay_frame_is_pose(raw, 17, 4));
	uint8_t composite[6] = {RELAY_COMPOSITE_TYPE, 4, 1, 3, 0x11, 9};
	CHECK(relay_frame_is_pose(composite, 6, 4));
	CHECK(!relay_frame_is_pose(composite, 6, 5));
}

/* ------------------------------------------------------------------ */

static struct relay_coord rc;
static uint32_t now;
static uint8_t seq[RC_MAX];
static uint8_t last[RC_MAX];
static bool started[RC_MAX];

/* One window of traffic: per tracker, direct deliveries out of `rate` frames
 * (the rest lost on both paths or relayed). */
struct traffic {
	uint8_t direct_pct; /* share delivered directly */
	uint8_t relayed_pct; /* share of the remainder delivered by relay */
	uint8_t rssi;
	bool silent;
};

static void run_window(struct traffic *traffic, uint8_t rate)
{
	for (uint8_t f = 0; f < rate; f++) {
		for (uint8_t id = 0; id < RC_MAX; id++) {
			struct traffic *t = &traffic[id];
			if (t->rssi == 0 || t->silent) {
				continue;
			}
			seq[id]++;
			/* Losses interleave with deliveries, as in a real fade. */
			bool direct = (uint32_t)((f * 61U + id * 17U) % 100U) < t->direct_pct;
			bool relayed = !direct && (uint32_t)((f * 37U + 11U) % 100U) < t->relayed_pct;
			if (direct || relayed) {
				uint8_t gaps = started[id] ? (uint8_t)(seq[id] - last[id] - 1U) : 0;
				relay_coord_note_frame(&rc, id, direct ? RC_DIRECT : RC_RELAYED,
					gaps ? RC_SEQ_GAP : RC_SEQ_NORMAL, gaps, t->rssi, now);
				last[id] = seq[id];
				started[id] = true;
			}
		}
	}
}

static uint16_t tick(uint16_t members)
{
	now += RC_WINDOW_MS;
	uint16_t fwd = relay_coord_tick(&rc, members, now);
	return fwd;
}

/* Loss-based weakness needs RC_WEAK_WINDOWS consecutive lossy windows. */
static uint16_t weak_windows(struct traffic *traffic, uint16_t members)
{
	uint16_t fwd = 0;
	for (unsigned w = 0; w < RC_WEAK_WINDOWS; w++) {
		run_window(traffic, 50);
		fwd = tick(members);
	}
	return fwd;
}

static void deliver_trailers(void)
{
	for (uint8_t h = 0; h < RC_MAX; h++) {
		if (rc.t[h].role == RC_ROLE_HEAD) {
			relay_coord_note_trailer_sent(&rc, h, rc.t[h].children, now);
		}
	}
}

static void reset_coord(uint16_t capable)
{
	relay_coord_init(&rc);
	now = 1000;
	memset(seq, 0, sizeof(seq));
	memset(last, 0, sizeof(last));
	memset(started, 0, sizeof(started));
	for (uint8_t i = 0; i < RC_MAX; i++) {
		if (capable & (1U << i)) {
			relay_coord_note_caps(&rc, i, RELAY_INFO_CAPS);
		}
	}
}

static void test_assignment_prefers_best_rssi_and_pin(void)
{
	reset_coord(0x003F);
	struct traffic t[RC_MAX] = {0};
	for (int i = 0; i < 6; i++) {
		t[i] = (struct traffic){.direct_pct = 100, .rssi = (uint8_t)(70 - i * 5)};
	}
	run_window(t, 50);
	tick(0x003F);
	CHECK(rc.assignments == 0);
	t[2].direct_pct = 40; /* tracker 2 obstructed */
	weak_windows(t, 0x003F);
	CHECK(rc.t[2].role == RC_ROLE_CHILD);
	CHECK(rc.t[2].head == 5); /* lowest RSSI magnitude = strongest link */
	CHECK(rc.t[5].role == RC_ROLE_HEAD && rc.t[5].children == (1U << 2));

	/* Pinned head wins over RSSI. */
	reset_coord(0x003F);
	relay_coord_pin(&rc, 0, true);
	t[2].direct_pct = 100;
	run_window(t, 50);
	tick(0x003F);
	t[2].direct_pct = 40;
	weak_windows(t, 0x003F);
	CHECK(rc.t[2].role == RC_ROLE_CHILD && rc.t[2].head == 0);
}

/* Hardware, RevF pair at -36/-43 dBm: 2-4% background loss is normal. It must
 * neither trigger relaying nor disqualify heads; one lossy window is a blip. */
static void test_background_loss_is_not_weak(void)
{
	reset_coord(0x000F);
	struct traffic t[RC_MAX] = {0};
	for (int i = 0; i < 4; i++) {
		t[i] = (struct traffic){.direct_pct = (uint8_t)(96 + i % 3), .rssi = (uint8_t)(36 + i * 3)};
	}
	for (int w = 0; w < 40; w++) {
		run_window(t, 50);
		tick(0x000F);
	}
	CHECK(rc.assignments == 0);
	t[1].direct_pct = 70; /* single lossy window */
	run_window(t, 50);
	tick(0x000F);
	t[1].direct_pct = 97;
	run_window(t, 50);
	tick(0x000F);
	CHECK(rc.assignments == 0);
	t[2].direct_pct = 30; /* sustained: assigned to a head with background loss */
	weak_windows(t, 0x000F);
	CHECK(rc.t[2].role == RC_ROLE_CHILD);
}

static void test_moderate_loss_stays_direct(void)
{
	reset_coord(0x000F);
	struct traffic t[RC_MAX] = {0};
	for (int i = 0; i < 4; i++) {
		t[i] = (struct traffic){.direct_pct = 98, .rssi = (uint8_t)(36 + i * 3)};
	}
	t[3].direct_pct = 85; /* far tracker with idle background loss */
	for (int w = 0; w < 40; w++) {
		run_window(t, 50);
		tick(0x000F);
	}
	CHECK(rc.assignments == 0);
	CHECK(rc.t[3].role == RC_ROLE_NONE);
	CHECK(relay_coord_direct_ready(&rc, 3, now));
}

static void test_non_capable_never_head_but_can_be_child(void)
{
	reset_coord(0x0002); /* only tracker 1 capable */
	struct traffic t[RC_MAX] = {0};
	t[0] = (struct traffic){.direct_pct = 100, .rssi = 40};
	t[1] = (struct traffic){.direct_pct = 100, .rssi = 60};
	t[2] = (struct traffic){.direct_pct = 30, .rssi = 80};
	weak_windows(t, 0x0007);
	CHECK(rc.t[2].role == RC_ROLE_CHILD && rc.t[2].head == 1);
	struct relay_pong_trailer tr;
	relay_coord_trailer(&rc, 1, 5, 0x0007, &tr);
	CHECK(tr.children_mask == (1U << 2) && tr.epoch == 5 && tr.member_mask == 0x0007);
}

static void test_head_and_children_limits(void)
{
	reset_coord(0xFFFF);
	struct traffic t[RC_MAX] = {0};
	for (int i = 0; i < 12; i++) {
		t[i] = (struct traffic){.direct_pct = 100, .rssi = (uint8_t)(50 + i)};
	}
	run_window(t, 50);
	tick(0x0FFF);
	for (int i = 3; i < 12; i++) {
		t[i].direct_pct = 20; /* nine weak trackers */
	}
	weak_windows(t, 0x0FFF);
	int heads = 0, children = 0;
	for (unsigned i = 0; i < RC_MAX; i++) {
		heads += rc.t[i].role == RC_ROLE_HEAD;
		children += rc.t[i].role == RC_ROLE_CHILD;
		if (rc.t[i].role == RC_ROLE_HEAD) {
			CHECK(relay_mask_count(rc.t[i].children) <= RELAY_MAX_CHILDREN);
		}
	}
	CHECK(heads == 3);
	CHECK(children == 9);
	CHECK(rc.t[0].role == RC_ROLE_HEAD && rc.t[1].role == RC_ROLE_HEAD && rc.t[2].role == RC_ROLE_HEAD);
}

static void test_silence_triggers_fast(void)
{
	reset_coord(0x0007);
	struct traffic t[RC_MAX] = {0};
	for (int i = 0; i < 3; i++) {
		t[i] = (struct traffic){.direct_pct = 100, .rssi = 50};
	}
	run_window(t, 50);
	tick(0x0007);
	t[1].silent = true; /* total uplink loss: no gaps can be counted */
	run_window(t, 50);
	tick(0x0007);
	CHECK(rc.t[1].role == RC_ROLE_CHILD);
}

static void test_failed_pair_rotates_and_bans(void)
{
	reset_coord(0x000F);
	struct traffic t[RC_MAX] = {0};
	for (int i = 0; i < 4; i++) {
		t[i] = (struct traffic){.direct_pct = 100, .rssi = (uint8_t)(50 + i)};
	}
	run_window(t, 50);
	tick(0x000F);
	t[3] = (struct traffic){.direct_pct = 30, .relayed_pct = 0, .rssi = 80};
	weak_windows(t, 0x000F);
	CHECK(rc.t[3].role == RC_ROLE_CHILD && rc.t[3].head == 0);
	deliver_trailers();
	/* Head 0 cannot hear tracker 3: nothing relayed. */
	for (int w = 0; w < 4; w++) {
		run_window(t, 50);
		tick(0x000F);
	}
	CHECK(rc.rotations >= 1);
	CHECK(rc.ban_until[3][0] != 0);
	CHECK(rc.t[3].role == RC_ROLE_CHILD && rc.t[3].head != 0);
	/* Now the new head hears it: relay recovers most frames. */
	t[3].relayed_pct = 95;
	deliver_trailers();
	uint32_t rotations = rc.rotations;
	for (int w = 0; w < 6; w++) {
		run_window(t, 50);
		tick(0x000F);
	}
	CHECK(rc.rotations == rotations);
	CHECK(rc.relayed_mask & (1U << 3));
	struct relay_pong_trailer tr;
	relay_coord_trailer(&rc, 2, 7, 0x000F, &tr); /* overheard by tracker 3 */
	CHECK(tr.relayed_mask & (1U << 3));
}

static void test_delivery_timeout_rotates(void)
{
	reset_coord(0x0007);
	struct traffic t[RC_MAX] = {0};
	for (int i = 0; i < 3; i++) {
		t[i] = (struct traffic){.direct_pct = 100, .rssi = (uint8_t)(50 + i)};
	}
	run_window(t, 50);
	tick(0x0007);
	t[2].direct_pct = 30;
	weak_windows(t, 0x0007);
	uint8_t first = rc.t[2].head;
	/* The head never PINGs (trailer never sent): give up after the timeout. */
	for (unsigned w = 0; w <= RC_DELIVERY_TIMEOUT_MS / RC_WINDOW_MS; w++) {
		run_window(t, 50);
		tick(0x0007);
	}
	CHECK(rc.t[2].role == RC_ROLE_CHILD && rc.t[2].head != first);
}

static void test_release_after_recovery_and_hold(void)
{
	reset_coord(0x0003);
	struct traffic t[RC_MAX] = {0};
	t[0] = (struct traffic){.direct_pct = 100, .rssi = 50};
	t[1] = (struct traffic){.direct_pct = 100, .rssi = 60};
	run_window(t, 50);
	tick(0x0003);
	t[1].direct_pct = 30;
	t[1].relayed_pct = 90;
	weak_windows(t, 0x0003);
	CHECK(rc.t[1].role == RC_ROLE_CHILD);
	deliver_trailers();
	t[1].direct_pct = 100;
	uint32_t assigned = rc.t[1].assigned_ms;
	unsigned windows = 0;
	while (rc.t[1].role == RC_ROLE_CHILD && windows < 200) {
		run_window(t, 50);
		tick(0x0003);
		windows++;
	}
	CHECK(rc.t[1].role == RC_ROLE_NONE);
	CHECK(now - assigned >= RC_MIN_HOLD_MS);
	CHECK(rc.t[0].role == RC_ROLE_NONE); /* last child gone: head role ends */
}

static void test_child_silent_released_and_head_loss(void)
{
	reset_coord(0x0007);
	struct traffic t[RC_MAX] = {0};
	for (int i = 0; i < 3; i++) {
		t[i] = (struct traffic){.direct_pct = 100, .rssi = (uint8_t)(50 + i)};
	}
	run_window(t, 50);
	tick(0x0007);
	t[2].silent = true;
	run_window(t, 50);
	tick(0x0007);
	CHECK(rc.t[2].role == RC_ROLE_CHILD);
	uint8_t silent_head = rc.t[2].head;
	uint32_t silent_assigned = rc.t[2].assigned_ms;
	/* Silence is counted from the assignment, not the last frame: the head
	 * gets the full RC_CHILD_SILENT_MS to start forwarding. */
	for (unsigned w = 0; w < RC_CHILD_SILENT_MS / RC_WINDOW_MS - 1; w++) {
		run_window(t, 50);
		tick(0x0007);
	}
	CHECK(rc.t[2].role == RC_ROLE_CHILD && rc.t[2].head == silent_head);
	for (unsigned w = 0; w < 3; w++) {
		run_window(t, 50);
		tick(0x0007);
	}
	/* That head never heard it: banned, and the still-member child is retried
	 * with the other candidate. */
	CHECK(rc.ban_until[2][silent_head] != 0);
	CHECK(rc.t[2].role == RC_ROLE_CHILD && rc.t[2].head != silent_head);
	CHECK(rc.t[2].assigned_ms != silent_assigned);
	/* Still a member but silent everywhere: retries are bounded, then dormant. */
	for (unsigned w = 0; w <= RC_CHILD_RECOVERY_MS / RC_WINDOW_MS + RC_CHILD_SILENT_MS / RC_WINDOW_MS + 1; w++) {
		run_window(t, 50);
		tick(0x0007);
	}
	CHECK(rc.t[2].role == RC_ROLE_NONE); /* bounded recovery attempts ended: dormant */

	/* A head that degrades loses its children (they get another head).
	 * Start independently of the failed-pair bans from the dormant case. */
	reset_coord(0x0007);
	t[2].silent = false;
	run_window(t, 50);
	tick(0x0007);
	t[2].direct_pct = 30;
	weak_windows(t, 0x0007);
	uint8_t head = rc.t[2].head;
	CHECK(rc.t[2].role == RC_ROLE_CHILD);
	t[head].direct_pct = 50;
	run_window(t, 50);
	tick(0x0007);
	CHECK(!(rc.t[head].role == RC_ROLE_HEAD && (rc.t[head].children & (1U << 2))));
}

static void test_forwarding_two_phase(void)
{
	reset_coord(0x001F);
	struct traffic t[RC_MAX] = {0};
	for (int i = 0; i < 5; i++) {
		t[i] = (struct traffic){.direct_pct = 100, .rssi = (uint8_t)(50 + i)};
	}
	run_window(t, 50);
	tick(0x001F);
	relay_coord_pin(&rc, 0, true);
	t[3].direct_pct = 30;
	t[4].direct_pct = 30;
	t[3].relayed_pct = t[4].relayed_pct = 95;
	uint16_t fwd = weak_windows(t, 0x001F);
	CHECK(rc.t[0].children == ((1U << 3) | (1U << 4)));
	CHECK(fwd == 0x0001); /* two children: one bundle per frame is not enough */
	/* Not in the published layout yet: no slot advertised. */
	CHECK(relay_coord_forward_slot(&rc, 0, 0x001F) == RELAY_NO_SLOT);
	relay_coord_set_forward_layout(&rc, fwd);
	CHECK(relay_coord_forward_slot(&rc, 0, 0x001F) == 5); /* after 5 direct slots */
	struct relay_pong_trailer tr;
	relay_coord_trailer(&rc, 0, 3, 0x001F, &tr);
	CHECK(tr.forward_slot == 5);

	/* Children recover: after the idle period the slot is revoked first
	 * (trailer says no slot) but stays in the layout for the lease wait. */
	t[3].direct_pct = t[4].direct_pct = 100;
	deliver_trailers();
	uint32_t revoke_seen = 0;
	for (int w = 0; w < 200; w++) {
		run_window(t, 50);
		fwd = tick(0x001F);
		if (!revoke_seen && rc.t[0].fwd_revoking) {
			revoke_seen = now;
			CHECK(relay_coord_forward_slot(&rc, 0, 0x001F) == RELAY_NO_SLOT);
			CHECK(fwd == 0x0001); /* still reserved while the lease runs out */
		}
		if (revoke_seen && fwd == 0) {
			CHECK(now - revoke_seen >= RC_FWD_REVOKE_WAIT_MS - RC_WINDOW_MS);
			break;
		}
		relay_coord_set_forward_layout(&rc, fwd);
	}
	CHECK(revoke_seen != 0);
	CHECK(fwd == 0);
}

static void test_forwarding_slot_cap(void)
{
	reset_coord(0xFFFF);
	struct traffic t[RC_MAX] = {0};
	for (int i = 0; i < 15; i++) {
		t[i] = (struct traffic){.direct_pct = 100, .rssi = (uint8_t)(50 + i)};
	}
	run_window(t, 50);
	tick(0x7FFF);
	for (int i = 3; i < 9; i++) {
		t[i].direct_pct = 30;
	}
	uint16_t fwd = weak_windows(t, 0x7FFF); /* 15 members: room for one forwarding slot */
	CHECK(relay_mask_count(fwd) == 1);
}

static void test_disable_and_forget(void)
{
	reset_coord(0x0007);
	struct traffic t[RC_MAX] = {0};
	for (int i = 0; i < 3; i++) {
		t[i] = (struct traffic){.direct_pct = 100, .rssi = (uint8_t)(50 + i)};
	}
	run_window(t, 50);
	tick(0x0007);
	t[2].direct_pct = 30;
	weak_windows(t, 0x0007);
	CHECK(relay_coord_is_relayed(&rc, 2));
	relay_coord_set_enabled(&rc, false); /* e.g. OTA in progress */
	CHECK(!relay_coord_is_relayed(&rc, 2));
	run_window(t, 50);
	tick(0x0007);
	CHECK(!relay_coord_is_relayed(&rc, 2));
	struct relay_pong_trailer tr;
	relay_coord_trailer(&rc, 0, 1, 0x0007, &tr);
	CHECK(tr.children_mask == 0);
	relay_coord_set_enabled(&rc, true);
	run_window(t, 50);
	tick(0x0007);
	CHECK(relay_coord_is_relayed(&rc, 2));
	uint8_t head = rc.t[2].head;
	relay_coord_forget(&rc, head, now);
	CHECK(!relay_coord_is_relayed(&rc, 2));
	CHECK(!relay_coord_is_capable(&rc, head));
}


static void test_zero_path_rotates(void)
{
 reset_coord(7);
 struct traffic t[RC_MAX] = {0};
 for(int i=0;i<3;i++) t[i]=(struct traffic){.direct_pct=100,.rssi=(uint8_t)(50+i)};
 run_window(t,50); tick(7);
 t[2].direct_pct=0;
 run_window(t,50); tick(7); deliver_trailers();
 CHECK(rc.t[2].role==RC_ROLE_CHILD);
 uint8_t first=rc.t[2].head;
 for(int w=0;w<8;w++){run_window(t,50);tick(7);deliver_trailers();}
 CHECK(rc.ban_until[2][first]!=0);
 CHECK(rc.t[2].role==RC_ROLE_CHILD && rc.t[2].head!=first);
}
static void test_long_relay_head_disappears(void)
{
 reset_coord(7);
 struct traffic t[RC_MAX]={0};
 for(int i=0;i<3;i++) t[i]=(struct traffic){.direct_pct=100,.rssi=(uint8_t)(50+i)};
 run_window(t,50);tick(7);
 t[2].direct_pct=0;t[2].relayed_pct=95;
 run_window(t,50);tick(7);deliver_trailers();
 uint8_t head=rc.t[2].head;
 /* Relay outlasts even the silent-member window: only relay_needed keeps
  * the child eligible once its head disappears. */
 for(unsigned w=0;w<RC_SILENT_WEAK_MS/RC_WINDOW_MS+4;w++){run_window(t,50);tick(7);deliver_trailers();}
 CHECK(rc.t[2].role==RC_ROLE_CHILD && rc.t[2].head==head);
 CHECK(now-rc.t[2].last_direct_ms>RC_SILENT_WEAK_MS+RC_WINDOW_MS);
 t[head].silent=true;t[2].relayed_pct=0;
 run_window(t,50);tick(7);
 CHECK(rc.t[2].role==RC_ROLE_CHILD && rc.t[2].head!=head);
}
static void test_single_child_capacity_request(void)
{
 reset_coord(7);
 struct traffic t[RC_MAX]={0};
 for(int i=0;i<3;i++) t[i]=(struct traffic){.direct_pct=100,.rssi=(uint8_t)(50+i)};
 run_window(t,50);tick(7);t[2].direct_pct=30;
 run_window(t,50);tick(7);deliver_trailers();
 uint8_t head=rc.t[2].head;
 relay_coord_note_caps(&rc,head,RELAY_INFO_CAPS|4U);
 run_window(t,50);uint16_t mask=tick(7);
 CHECK(mask==(1U<<head));
}


static void test_ota_gate_survives_role_pause(void)
{
 reset_coord(7);
 struct traffic t[RC_MAX]={0};
 for(int i=0;i<3;i++) t[i]=(struct traffic){.direct_pct=100,.rssi=(uint8_t)(50+i)};
 run_window(t,50);tick(7);CHECK(relay_coord_direct_ready(&rc,2,now));
 t[2].direct_pct=0;t[2].relayed_pct=95;
 run_window(t,50);tick(7);CHECK(relay_coord_is_relayed(&rc,2));
 relay_coord_set_enabled(&rc,false); /* OTA on a different tracker releases roles */
 CHECK(!relay_coord_is_relayed(&rc,2));
 CHECK(!relay_coord_direct_ready(&rc,2,now));
}


static void test_ota_gate_ignores_sparse_window_loss(void)
{
 reset_coord(7);
 struct traffic t[RC_MAX]={0};
 for(int i=0;i<3;i++) t[i]=(struct traffic){.direct_pct=100,.rssi=(uint8_t)(50+i)};
 run_window(t,50);tick(7);
 /* Idle rate: 10 expected frames per window, one lost. Not judged alone. */
 t[2].direct_pct=90;run_window(t,10);tick(7);
 CHECK(rc.t[2].weak_windows==0);
 CHECK(relay_coord_direct_ready(&rc,2,now));
 /* Sustained loss the coordinator would relay for blocks OTA. */
 t[2].direct_pct=50;
 for(unsigned w=0;w<RC_WEAK_WINDOWS;w++){run_window(t,50);tick(7);}
 CHECK(!relay_coord_direct_ready(&rc,2,now));
}

/* A tracker judged weak while moving, then still: sparse idle windows must
 * still clear the verdict (they used to be skipped, leaving OTA refused). */
static void test_idle_windows_still_judge(void)
{
 reset_coord(7);relay_coord_set_enabled(&rc,false);
 struct traffic t[RC_MAX]={0};
 for(int i=0;i<3;i++) t[i]=(struct traffic){.direct_pct=100,.rssi=(uint8_t)(50+i)};
 run_window(t,50);tick(7);
 t[2].direct_pct=50;
 for(unsigned w=0;w<RC_WEAK_WINDOWS;w++){run_window(t,50);tick(7);}
 CHECK(rc.t[2].weak_windows>=RC_WEAK_WINDOWS);
 CHECK(!relay_coord_direct_ready(&rc,2,now));
 t[2].direct_pct=100;
 for(unsigned w=0;w<RC_JUDGE_MAX_WINDOWS;w++){run_window(t,7);tick(7);}
 CHECK(rc.t[2].weak_windows==0);
 CHECK(relay_coord_direct_ready(&rc,2,now));
 /* A sparse partial count is dropped after the cap, never judged. */
 rc.t[2].acc_direct=0;rc.t[2].acc_expected=0;rc.t[2].acc_windows=0;
 for(unsigned w=0;w<RC_JUDGE_MAX_WINDOWS;w++){run_window(t,1);tick(7);}
 CHECK(rc.t[2].acc_windows==0 && rc.t[2].weak_windows==0);
}

/* An idle child (few frames per window) still collects good judgements and
 * is released once direct delivery recovers. */
static void test_idle_child_released(void)
{
 reset_coord(7);
 struct traffic t[RC_MAX]={0};
 for(int i=0;i<3;i++) t[i]=(struct traffic){.direct_pct=100,.rssi=(uint8_t)(50+i)};
 run_window(t,50);tick(7);
 t[2].direct_pct=30;t[2].relayed_pct=95;
 for(unsigned w=0;w<RC_WEAK_WINDOWS;w++){run_window(t,50);tick(7);deliver_trailers();}
 CHECK(rc.t[2].role==RC_ROLE_CHILD);
 t[2].direct_pct=100;
 unsigned windows=0;
 while(rc.t[2].role==RC_ROLE_CHILD && windows<400){run_window(t,8);tick(7);deliver_trailers();windows++;}
 CHECK(rc.t[2].role==RC_ROLE_NONE && !rc.t[2].relay_needed);
 CHECK(relay_coord_direct_ready(&rc,2,now));
}

/* Prefer direct: an empty window from an idle tracker is not loss. */
static void test_idle_gap_does_not_force_relay(void)
{
 reset_coord(7);
 struct traffic t[RC_MAX]={0};
 for(int i=0;i<3;i++) t[i]=(struct traffic){.direct_pct=100,.rssi=(uint8_t)(50+i)};
 for(int w=0;w<4;w++){run_window(t,5);tick(7);}
 t[2].silent=true;
 run_window(t,5);tick(7);
 CHECK(rc.assignments==0 && rc.t[2].role==RC_ROLE_NONE);
 /* About RC_MIN_EXPECTED missed frames at its own rate: real silence. */
 unsigned windows=1;
 while(rc.t[2].role==RC_ROLE_NONE && windows<20){run_window(t,5);tick(7);windows++;}
 CHECK(rc.t[2].role==RC_ROLE_CHILD);
 /* The harness notes a window's frames at its start, so gaps read one
  * window long: 20 frames at 5 per window = 2 s, seen after 3 windows. */
 CHECK(windows>=RC_MIN_EXPECTED/5-1);
}

/* Full-rate obstruction is still caught within one window. */
static void test_full_rate_silence_is_fast(void)
{
 reset_coord(7);
 struct traffic t[RC_MAX]={0};
 for(int i=0;i<3;i++) t[i]=(struct traffic){.direct_pct=100,.rssi=(uint8_t)(50+i)};
 run_window(t,50);tick(7);
 t[2].silent=true;
 run_window(t,50);tick(7);
 CHECK(rc.t[2].role==RC_ROLE_CHILD);
}

/* A child whose direct link recovers is released after the short hold. */
static void test_recovered_child_released_after_hold(void)
{
 reset_coord(7);
 struct traffic t[RC_MAX]={0};
 for(int i=0;i<3;i++) t[i]=(struct traffic){.direct_pct=100,.rssi=(uint8_t)(50+i)};
 run_window(t,50);tick(7);
 t[2].direct_pct=30;t[2].relayed_pct=95;
 weak_windows(t,7);deliver_trailers();
 CHECK(rc.t[2].role==RC_ROLE_CHILD);
 uint32_t assigned=rc.t[2].assigned_ms;
 t[2].direct_pct=100;
 while(rc.t[2].role==RC_ROLE_CHILD && now-assigned<60000){run_window(t,50);tick(7);deliver_trailers();}
 CHECK(rc.t[2].role==RC_ROLE_NONE);
 CHECK(now-assigned<=RC_MIN_HOLD_MS+RC_WINDOW_MS);
}

static void test_forget_starts_a_fresh_revoke_wait(void)
{
 reset_coord(7);rc.forward_mask=1;rc.t[0].fwd_wanted=true;
 now=10000;relay_coord_forget(&rc,0,now);
 CHECK(rc.t[0].fwd_revoking && rc.t[0].fwd_revoke_ms==now);
 CHECK(relay_coord_tick(&rc,7,now)==1);
 CHECK(relay_coord_tick(&rc,7,now+RC_FWD_REVOKE_WAIT_MS-1)==1);
 CHECK(relay_coord_tick(&rc,7,now+RC_FWD_REVOKE_WAIT_MS)==0);
}

int main(void)
{
	test_trailer();
	test_member_slot();
	test_bundle();
	test_assignment_prefers_best_rssi_and_pin();
	test_background_loss_is_not_weak();
	test_moderate_loss_stays_direct();
	test_non_capable_never_head_but_can_be_child();
	test_head_and_children_limits();
	test_silence_triggers_fast();
	test_failed_pair_rotates_and_bans();
	test_delivery_timeout_rotates();
	test_release_after_recovery_and_hold();
	test_child_silent_released_and_head_loss();
	test_forwarding_two_phase();
	test_forwarding_slot_cap();
	test_disable_and_forget();
	test_zero_path_rotates();
	test_long_relay_head_disappears();
	test_single_child_capacity_request();
	test_ota_gate_survives_role_pause();
	test_ota_gate_ignores_sparse_window_loss();
	test_idle_windows_still_judge();
	test_idle_child_released();
	test_idle_gap_does_not_force_relay();
	test_full_rate_silence_is_fast();
	test_recovered_child_released_after_hold();
	test_forget_starts_a_fresh_revoke_wait();
	if (failures) {
		fprintf(stderr, "relay coordinator: %d failure(s)\n", failures);
		return 1;
	}
	printf("relay coordinator: all tests passed\n");
	return 0;
}
