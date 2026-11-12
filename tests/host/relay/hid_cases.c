/* Included after the extracted receiver RX functions (see run_hid.py). */
static int failures;
#define CHECK(cond) do { if (!(cond)) { fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

static void reset_state(void)
{
	memset(tracker_slots, 0, sizeof(tracker_slots));
	memset(hid_out, 0, sizeof(hid_out));
	hid_count = 0;
	shadow_seen = 0;
	relay_coord_init(&relay_coord);
	relay_block_id = 0xFF;
	relay_block_percent = 0;
}

static void make_normal(struct esb_payload *p, uint8_t type, uint8_t id, uint8_t seq, int8_t rssi)
{
	memset(p, 0, sizeof(*p));
	p->length = 17;
	p->pipe = 1 + id % 7;
	p->rssi = rssi;
	p->data[0] = type;
	p->data[1] = id;
	for (int i = 2; i < 16; i++) {
		p->data[i] = (uint8_t)(0x30 + i * 3 + id);
	}
	p->data[16] = seq;
}

static void make_bundle(struct esb_payload *out, uint8_t head, const struct esb_payload *own,
	const struct esb_payload *record, uint8_t capture_rssi, int8_t head_rssi)
{
	memset(out, 0, sizeof(*out));
	struct relay_bundle_builder b;
	uint8_t own_len = own ? own->length : 0;
	CHECK(relay_bundle_begin(&b, out->data, sizeof(out->data), head, own ? own->data : NULL, own_len, false));
	CHECK(relay_bundle_add(&b, record->data, record->length, capture_rssi));
	out->length = (uint8_t)relay_bundle_finish(&b);
	out->pipe = 1 + head % 7;
	out->rssi = head_rssi;
}

/* The child's HID packet must match direct delivery byte for byte; only the
 * RSSI argument differs, reporting the weaker of the two hops. */
static void test_normal_parity(uint8_t type)
{
	struct esb_payload direct, child, head_own, bundle;

	reset_state();
	make_normal(&direct, type, 2, 5, 61);
	if (type == 0) {
		direct.data[7] = RELAY_INFO_CAPS;
	}
	esb_receive_normal(&direct, 0, false);
	CHECK(hid_count == 1);
	struct hid_record direct_out = hid_out[0];

	reset_state();
	make_normal(&child, type, 2, 5, 0);
	if (type == 0) {
		child.data[7] = RELAY_INFO_CAPS;
	}
	make_normal(&head_own, 1, 1, 9, 0);
	make_bundle(&bundle, 1, &head_own, &child, 72, 50);
	esb_receive_bundle(&bundle, 0);
	CHECK(hid_count == 2);
	CHECK(hid_out[0].pkt[1] == 1); /* head's own frame first, as received directly */
	CHECK(hid_out[0].rssi == 50);
	CHECK(memcmp(hid_out[1].pkt, direct_out.pkt, 16) == 0);
	CHECK(hid_out[1].rssi == 72); /* max(-50, -72 dBm magnitude) = weaker hop */
	CHECK(shadow_seen & (1U << 2)); /* relayed data keeps TDMA membership */
	if (type == 0) {
		CHECK(direct_out.pkt[RELAY_INFO_CAPS_INDEX] == 0); /* stock reserved byte */
		CHECK(relay_coord_is_capable(&relay_coord, 2));
	}
}

static void test_composite_parity(void)
{
	/* quat+accel (type 1, 14 bytes) and status (type 3, 2 bytes). */
	uint8_t frame[3 + 1 + 14 + 1 + 2 + 1] = {ESB_COMPOSITE_TYPE, 3, 2, 1};
	for (int i = 0; i < 14; i++) {
		frame[4 + i] = (uint8_t)(0x40 + i);
	}
	frame[18] = 3;
	frame[19] = 0x55;
	frame[20] = 0x66;
	frame[21] = 77; /* sequence */
	struct esb_payload direct = {.length = sizeof(frame), .pipe = 1 + 3 % 7, .rssi = 60};
	memcpy(direct.data, frame, sizeof(frame));

	reset_state();
	esb_receive_composite(&direct, 0, false);
	CHECK(hid_count == 2);
	struct hid_record d0 = hid_out[0], d1 = hid_out[1];

	reset_state();
	struct esb_payload record = direct, bundle;
	make_bundle(&bundle, 5, NULL, &record, 66, 40);
	esb_receive_bundle(&bundle, 0);
	CHECK(hid_count == 2);
	CHECK(memcmp(hid_out[0].pkt, d0.pkt, 16) == 0);
	CHECK(memcmp(hid_out[1].pkt, d1.pkt, 16) == 0); /* status counters too */
	CHECK(hid_out[0].rssi == 66);
}

static void test_duplicate_and_order(void)
{
	struct esb_payload direct, record, bundle, head_own;
	reset_state();
	make_normal(&direct, 1, 2, 5, 60);
	esb_receive_normal(&direct, 0, false);
	/* The head forwards the same frame: dropped as a duplicate. */
	make_normal(&record, 1, 2, 5, 0);
	make_normal(&head_own, 1, 1, 3, 0);
	make_bundle(&bundle, 1, &head_own, &record, 70, 50);
	esb_receive_bundle(&bundle, 0);
	CHECK(hid_count == 2); /* direct child + head own, no relayed duplicate */
	CHECK(hid_out[1].pkt[1] == 1);
	/* Next direct frame after a relayed one keeps the sequence going. */
	make_normal(&record, 1, 2, 6, 0);
	make_bundle(&bundle, 1, NULL, &record, 70, 50);
	esb_receive_bundle(&bundle, 0);
	make_normal(&direct, 1, 2, 7, 60);
	esb_receive_normal(&direct, 0, false);
	CHECK(hid_count == 4);
	CHECK(tracker_slots[2].stats.total_gaps == 0);
}

static void test_bundle_rejections(void)
{
	struct esb_payload record, bundle;
	reset_state();
	make_normal(&record, 1, 2, 5, 0);
	make_bundle(&bundle, 1, NULL, &record, 70, 50);
	bundle.pipe = 4; /* not the head's pipe */
	esb_receive_bundle(&bundle, 0);
	CHECK(hid_count == 0);
	CHECK(relay_rx_stats.bundle_invalid >= 1);

	make_normal(&record, 1, 12, 5, 0); /* not a stored tracker (8 stored) */
	make_bundle(&bundle, 1, NULL, &record, 70, 50);
	esb_receive_bundle(&bundle, 0);
	CHECK(hid_count == 0);
}

static void test_block_hook(void)
{
	struct esb_payload direct, record, bundle;
	reset_state();
	relay_block_id = 2;
	relay_block_percent = 100;
	make_normal(&direct, 1, 2, 5, 60);
	esb_receive_normal(&direct, 0, false);
	CHECK(hid_count == 0); /* direct uplink blocked */
	make_normal(&record, 1, 2, 5, 0);
	make_bundle(&bundle, 1, NULL, &record, 70, 50);
	esb_receive_bundle(&bundle, 0);
	CHECK(hid_count == 1); /* relay path unaffected */
	relay_block_id = 1; /* blocking the head blocks its bundles */
	make_normal(&record, 1, 2, 6, 0);
	make_bundle(&bundle, 1, NULL, &record, 70, 50);
	esb_receive_bundle(&bundle, 0);
	CHECK(hid_count == 1);
}

static void test_fast_repair(void)
{
	reset_state();
	relay_profile = RELAY_PROFILE_120HZ;
	relay_origin = 1000; relay_frame_anchor=0; relay_start_anchor=relay_origin;
	relay_session = 42;
	relay_coord_fast_reset(&relay_coord);
	tdma_active_mask = 7;
	relay_grants = (struct relay_control){.preamble_ticks=relay_frame_start(relay_origin,10)+RELAY_V3_B_PREAMBLE, .frame = 10, .session = 42, .epoch = 7, .repair_count = 5};
	relay_grants.grants[0] = (struct relay_grant){.owner = 0, .children = 4};
	relay_grants.grants[1] = (struct relay_grant){.owner = 1, .children = 4};
	relay_grants.missing[0] = relay_grants.missing[1] = 4;
	struct esb_payload source, packet = {.pipe = 1, .rssi = 50};
	make_normal(&source, 1, 2, 255, 60);
	struct relay_repair r = {.helper = 0, .epoch = 7, .session = 42, .count = 1};
	r.records[0] = (struct relay_repair_record){
	    .capture_frame = 10, .source = {.frame = source.data, .length = source.length, .rssi = 60}};
	packet.length = relay_repair_encode(packet.data, &r);
	uint32_t at = relay_frame_start(relay_origin, 10) + RELAY_V3_REPAIR_START + 10;
	esb_receive_repair(&packet, at + RELAY_V3_SLOT_TICKS);
	CHECK(hid_count == 0); /* wrong owner */
	r.session++;
	packet.length = relay_repair_encode(packet.data, &r);
	esb_receive_repair(&packet, at);
	CHECK(hid_count == 0);
	r.session--;
	packet.length = relay_repair_encode(packet.data, &r);
	source.data[1] = 3;
	packet.length = relay_repair_encode(packet.data, &r);
	esb_receive_repair(&packet, at);
	CHECK(hid_count == 0); /* wrong child */
	CHECK(shadow_seen == 0);
	source.data[1] = 2;
	/* Invalid copy must leave this capture publishable. */
	struct esb_payload bad = {.length=5, .data={ESB_COMPOSITE_TYPE, 2, 0, 0, 255}};
	r.records[0].source.frame=bad.data; r.records[0].source.length=bad.length;
	packet.length=relay_repair_encode(packet.data, &r);
	esb_receive_repair(&packet, at);
	CHECK(!relay_coord_was_published(&relay_coord, 2, 10));
	r.records[0].source.frame=source.data; r.records[0].source.length=source.length;
	packet.length = relay_repair_encode(packet.data, &r);
	esb_receive_repair(&packet, at);
	CHECK(hid_count == 1);
	CHECK(shadow_seen == (1U << 2)); /* valid repairs renew membership */
	CHECK(relay_coord.peer[0][2].rssi_x8 == 480 && !relay_coord.peer[0][2].expected);
	esb_receive_repair(&packet, at);
	CHECK(hid_count == 1);
	r.helper = 1;
	packet.pipe = 2;
	packet.length = relay_repair_encode(packet.data, &r);
	esb_receive_repair(&packet, at + RELAY_V3_SLOT_TICKS);
	CHECK(hid_count == 1);
	relay_direct_pose = true;
	relay_direct_start = relay_frame_start(relay_origin, 10) + 2 * RELAY_V3_SLOT_TICKS + 10;
	esb_receive_normal(&source, relay_direct_start, false);
	CHECK(hid_count == 1);
	source.data[16] = 0;
	relay_direct_start = relay_frame_start(relay_origin, 11) + 2 * RELAY_V3_SLOT_TICKS + 10;
	esb_receive_normal(&source, relay_direct_start, false);
	CHECK(hid_count == 2);
	r.helper = 0;
	r.records[0].capture_frame = 9;
	packet.pipe = 1;
	packet.length = relay_repair_encode(packet.data, &r);
	esb_receive_repair(&packet, at);
	CHECK(hid_count == 2); /* historical recovery */
	esb_receive_repair(&packet, at + RELAY_V3_FRAME_TICKS_MAX);
	CHECK(hid_count == 2);
	relay_coord_fast_reset(&relay_coord);
	r.helper = 1;
	r.records[0].capture_frame = 10;
	packet.pipe = 2;
	packet.length = relay_repair_encode(packet.data, &r);
	esb_receive_repair(&packet, at + RELAY_V3_SLOT_TICKS);
	CHECK(hid_count == 3);		      /* primary silent */
	relay_coord_fast_reset(&relay_coord); /* receiver/new layout resets frame order */
	CHECK(relay_coord_publish(&relay_coord, 2, 0));
	relay_profile = 0;
}

static void test_peer_report(void)
{
	reset_state();
	relay_coord_fast_reset(&relay_coord);
	relay_profile = 1; relay_origin = 1000; relay_frame_anchor=0; relay_start_anchor=relay_origin; relay_session = 42;
	relay_grants = (struct relay_control){.members=7,.preamble_ticks=relay_frame_start(relay_origin,10)+RELAY_V3_B_PREAMBLE,.frame=10,.session=42,.epoch=7,.repair_count=5};
	for (unsigned slot=0;slot<5;slot++) relay_grants.grants[slot].owner=RELAY_OWNER_IDLE;
	relay_grants.grants[2] = (struct relay_grant){.owner=0,.children=0};
	struct relay_peer_report report={.helper=0,.session=42,.epoch=7,.count=1};
	report.entries[0]=(struct relay_peer_observation){.child=2,.expected=120,.heard=0};
	struct esb_payload packet={.pipe=1,.rssi=40};
	packet.length=relay_peer_report_encode(packet.data,&report);
	uint32_t at=relay_frame_start(relay_origin,10)+RELAY_V3_REPAIR_START+2*RELAY_V3_SLOT_TICKS+2;
	uint32_t end=at+RELAY_V3_AIR_TICKS(packet.length);
	esb_receive_peer_report(&packet,end+RELAY_V3_SLOT_TICKS);
	CHECK(!relay_coord.peer[0][2].expected);
	packet.pipe=2; esb_receive_peer_report(&packet,end);
	CHECK(!relay_coord.peer[0][2].expected);
	packet.pipe=1; relay_grants.grants[2].children=4;
	esb_receive_peer_report(&packet,end);
	CHECK(!relay_coord.peer[0][2].expected);
	relay_grants.grants[2].children=0;
	esb_receive_peer_report(&packet,end);
	CHECK(relay_coord.peer[0][2].expected==120 && !relay_coord.peer[0][2].heard);
	CHECK(!relay_coord.peer[2][0].expected && hid_count==0);
	esb_receive_peer_report(&packet,end+RELAY_V3_FRAME_TICKS_MAX);
	CHECK(relay_coord.peer[0][2].expected==120);
	relay_profile=0;
}

static void test_composite_validation(void)
{
	struct composite_result d;
	struct esb_payload p = {.length=5, .data={ESB_COMPOSITE_TYPE, 2, 0, 0, 1}};
	CHECK(!composite_validate(&p, &d));
	p.data[2]=1; p.data[3]=1; /* known body truncated */
	CHECK(!composite_validate(&p, &d));
	p.data[3]=0xee; /* accepted unknown legacy tail, no business prefix */
	CHECK(composite_validate(&p, &d) && !d.business);
	p.length=19; p.data[3]=1; /* complete known pose prefix */
	CHECK(composite_validate(&p, &d) && d.business);
	p.data[2]=2; p.length=20; p.data[18]=0xee;
	CHECK(composite_validate(&p, &d) && d.business);
	p.data[18]=TRACKER_EVENT_ESB_TYPE;
	CHECK(!composite_validate(&p, &d));
	uint8_t event[TRACKER_EVENT_ESB_LEN];
	struct tracker_event e={.tracker_id=2,.nonce=1,.operation_id=1,.kind=1,.event=CAL_EVENT_BEGIN};
	CHECK(tracker_event_encode(event,&e));
	p=(struct esb_payload){.length=20,.data={ESB_COMPOSITE_TYPE,2,1,TRACKER_EVENT_ESB_TYPE}};
	memcpy(p.data+4,event+2,TRACKER_EVENT_BODY_LEN);
	CHECK(composite_validate(&p,&d) && !d.business && d.event_pos==4);
	p=(struct esb_payload){.length=35,.data={ESB_COMPOSITE_TYPE,2,2,1}};
	p.data[18]=TRACKER_EVENT_ESB_TYPE;
	memcpy(p.data+19,event+2,TRACKER_EVENT_BODY_LEN);
	CHECK(composite_validate(&p,&d) && d.business && d.event_pos==19);
	p.data[19]=0; CHECK(!composite_validate(&p,&d));
}

static void test_layout_frame_anchor(void)
{
	reset_state(); relay_profile=1; relay_origin=1000;
	relay_frame_anchor=15728640U;
	relay_start_anchor=relay_frame_start(relay_origin,relay_frame_anchor);
	/* A profile-1 layout publication clears grants, but not the clock grid. */
	relay_grants=(struct relay_control){0};
	tdma_active_mask=7;
	relay_direct_pose=true;
	relay_direct_start=relay_start_anchor+2*RELAY_V3_SLOT_TICKS+RELAY_V3_TX_OFFSET;
	struct esb_payload p; make_normal(&p,1,2,1,50);
	esb_receive_normal(&p,0,false);
	CHECK(relay_coord_was_published(&relay_coord,2,relay_frame_anchor));
	CHECK(hid_count==1);
	relay_profile=0;
}

int main(void)
{
	test_normal_parity(1);  /* quat + accel */
	test_normal_parity(0);  /* info: capability byte stripped in both paths */
	test_normal_parity(3);  /* status: receiver-filled loss counters */
	test_composite_parity();
	test_composite_validation();
	test_duplicate_and_order();
	test_bundle_rejections();
	test_block_hook();
	test_fast_repair();
	test_peer_report();
	test_layout_frame_anchor();
	if (failures) {
		fprintf(stderr, "relay HID parity: %d failure(s)\n", failures);
		return 1;
	}
	printf("relay HID parity: relayed frames produce the direct HID packets\n");
	return 0;
}
