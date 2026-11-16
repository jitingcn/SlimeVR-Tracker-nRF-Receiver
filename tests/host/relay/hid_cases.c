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



/* The child's HID packet must match direct delivery byte for byte; only the
 * RSSI argument differs, reporting the weaker of the two hops. */










static void test_normal_stats(void)
{
 reset_state(); relay_profile = 0;
 memset(&relay_rx_stats, 0, sizeof(relay_rx_stats));
 struct esb_payload p;
 make_normal(&p, 1, 2, 5, 60);
 esb_receive_normal(&p, 0, false);
 esb_receive_normal(&p, 0, false);
 p.data[16] = 4; esb_receive_normal(&p, 0, false);
 p.data[16] = 6; esb_receive_normal(&p, 0, false);
 CHECK(hid_count == 2);
 CHECK(tracker_slots[2].stats.total_received == 2);
 CHECK(tracker_slots[2].stats.packets_in_last_second == 2);
 CHECK(relay_rx_stats.arrivals == 4 && relay_rx_stats.duplicate_stale == 2);
 CHECK(relay_rx_stats.direct_accepted == 2 && !relay_rx_stats.repair_accepted);
}

static void test_fast_repair(void)
{
	reset_state();
	relay_profile = RELAY_PROFILE_120HZ;
	relay_origin = 1000; relay_frame_anchor=0; relay_start_anchor=relay_origin;
	relay_session = 42;
	relay_coord_fast_reset(&relay_coord);
	tdma_active_mask = 7;
	relay_grants = (struct relay_control){.preamble_ticks=relay_frame_start(relay_origin,10)+RELAY_CONTROL_PREAMBLE, .frame = 10, .session = 42, .epoch = 7, .repair_count = 5};
	relay_grants.grants[0] = (struct relay_grant){.owner = 0, .children = 4};
	relay_grants.grants[1] = (struct relay_grant){.owner = 1, .children = 4};
	relay_grants.missing[0] = relay_grants.missing[1] = 4;
	struct esb_payload source, packet = {.pipe = 1, .rssi = 50};
	make_normal(&source, 1, 2, 255, 60);
	struct relay_repair r = {.helper = 0, .epoch = 7, .session = 42, .count = 1};
	r.records[0] = (struct relay_repair_record){
	    .capture_frame = 10, .source = {.frame = source.data, .length = source.length, .rssi = 60}};
	packet.length = relay_repair_encode(packet.data, &r);
	uint32_t at = relay_frame_start(relay_origin, 10) + RELAY_REPAIR_START + 10;
	esb_receive_repair(&packet, at + RELAY_SLOT_TICKS);
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
	/* A published repair counts once in the host totals. */
	CHECK(tracker_slots[2].stats.total_received == 1 && tracker_slots[2].stats.status_received == 1);
	CHECK(relay_coord.peer[0][2].rssi_x8 == 480 && !relay_coord.peer[0][2].expected);
	shadow_seen = 0;
	esb_receive_repair(&packet, at);
	CHECK(hid_count == 1);
	CHECK(shadow_seen == 0); /* a duplicate renews nothing */
	CHECK(tracker_slots[2].stats.total_received == 1); /* nor counts */
	r.helper = 1;
	packet.pipe = 2;
	packet.length = relay_repair_encode(packet.data, &r);
	esb_receive_repair(&packet, at + RELAY_SLOT_TICKS);
	CHECK(hid_count == 1);
	relay_direct_pose = true;
	relay_direct_start = relay_frame_start(relay_origin, 10) + 2 * RELAY_SLOT_TICKS + 10;
	esb_receive_normal(&source, relay_direct_start, false);
	CHECK(hid_count == 1);
	source.data[16] = 0;
	relay_direct_start = relay_frame_start(relay_origin, 11) + 2 * RELAY_SLOT_TICKS + 10;
	esb_receive_normal(&source, relay_direct_start, false);
	CHECK(hid_count == 2);
	CHECK(tracker_slots[2].stats.total_received == 2 && !tracker_slots[2].stats.total_gaps);
	r.helper = 0;
	r.records[0].capture_frame = 9;
	packet.pipe = 1;
	packet.length = relay_repair_encode(packet.data, &r);
	esb_receive_repair(&packet, at);
	CHECK(hid_count == 2); /* historical recovery */
	esb_receive_repair(&packet, at + RELAY_FRAME_TICKS_MAX);
	CHECK(hid_count == 2);
	relay_coord_fast_reset(&relay_coord);
	r.helper = 1;
	r.records[0].capture_frame = 10;
	packet.pipe = 2;
	packet.length = relay_repair_encode(packet.data, &r);
	esb_receive_repair(&packet, at + RELAY_SLOT_TICKS);
	CHECK(hid_count == 3);		      /* primary silent */
	relay_coord_fast_reset(&relay_coord); /* receiver/new layout resets frame order */
	CHECK(relay_coord_publish(&relay_coord, 2, 0));
	relay_profile = 0;
}

/* Host loss totals: frames skipped on the private path count like skipped
 * native sequence numbers, so status packets report the same loss. */
static void test_private_host_stats(void)
{
	struct packet_stats native, priv;
	struct esb_payload p;
	reset_state();
	tdma_active_mask = 7;
	make_normal(&p, 1, 2, 0, 60);
	esb_receive_normal(&p, 0, false);
	p.data[16] = 3; /* two native sequence numbers lost */
	esb_receive_normal(&p, 0, false);
	esb_receive_normal(&p, 0, false); /* duplicate */
	native = tracker_slots[2].stats;
	CHECK(hid_count == 2 && native.total_received == 2 && native.total_gaps == 2 && native.status_lost == 2);

	reset_state();
	relay_profile = RELAY_PROFILE_120HZ;
	relay_origin = 1000; relay_frame_anchor = 0; relay_start_anchor = relay_origin;
	relay_coord_fast_reset(&relay_coord);
	tdma_active_mask = 7;
	relay_direct_pose = true;
	static const uint32_t frames[] = {10, 13, 13}; /* two frames lost, then a duplicate */
	for (unsigned i = 0; i < 3; i++) {
		make_normal(&p, 1, 2, (uint8_t)(200 + i), 60);
		relay_direct_start = relay_frame_start(relay_origin, frames[i]) + 2 * RELAY_SLOT_TICKS + 10;
		esb_receive_normal(&p, relay_direct_start, false);
	}
	priv = tracker_slots[2].stats;
	CHECK(hid_count == 2);
	CHECK(priv.total_received == native.total_received && priv.total_gaps == native.total_gaps);
	CHECK(priv.status_received == native.status_received && priv.status_lost == native.status_lost);
	CHECK(priv.gap_events == native.gap_events && priv.normal_packets == native.normal_packets);
	relay_direct_pose = false;
	relay_profile = 0;
}

static void test_peer_report(void)
{
	reset_state();
	relay_coord_fast_reset(&relay_coord);
	relay_profile = 1; relay_origin = 1000; relay_frame_anchor=0; relay_start_anchor=relay_origin; relay_session = 42;
	relay_grants = (struct relay_control){.members=7,.preamble_ticks=relay_frame_start(relay_origin,10)+RELAY_CONTROL_PREAMBLE,.frame=10,.session=42,.epoch=7,.repair_count=5};
	for (unsigned slot=0;slot<5;slot++) relay_grants.grants[slot].owner=RELAY_OWNER_IDLE;
	relay_grants.grants[2] = (struct relay_grant){.owner=0,.children=0};
	struct relay_peer_report report={.helper=0,.session=42,.epoch=7,.count=1};
	report.entries[0]=(struct relay_peer_observation){.child=2,.expected=120,.heard=0};
	struct esb_payload packet={.pipe=1,.rssi=40};
	packet.length=relay_peer_report_encode(packet.data,&report);
	uint32_t at=relay_frame_start(relay_origin,10)+RELAY_REPAIR_START+2*RELAY_SLOT_TICKS+2;
	uint32_t end=at+RELAY_AIR_TICKS(packet.length);
	esb_receive_peer_report(&packet,end+RELAY_SLOT_TICKS);
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
	esb_receive_peer_report(&packet,end+RELAY_FRAME_TICKS_MAX);
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
	relay_direct_start=relay_start_anchor+2*RELAY_SLOT_TICKS+RELAY_TX_OFFSET;
	struct esb_payload p; make_normal(&p,1,2,1,50);
	esb_receive_normal(&p,0,false);
	CHECK(relay_coord_was_published(&relay_coord,2,relay_frame_anchor));
	CHECK(hid_count==1);
	relay_profile=0;
}

int main(void)
{
	test_normal_stats();
	test_composite_validation();
	test_fast_repair();
	test_private_host_stats();
	test_peer_report();
	test_layout_frame_anchor();
	if (failures) {
		fprintf(stderr, "relay HID parity: %d failure(s)\n", failures);
		return 1;
	}
	printf("relay HID parity: relayed frames produce the direct HID packets\n");
	return 0;
}
