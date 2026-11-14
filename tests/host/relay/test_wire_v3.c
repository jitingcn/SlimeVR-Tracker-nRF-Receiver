#include <assert.h>
#include <stdio.h>
#include "relay_protocol.h"

static void control(void)
{
	uint8_t bytes[72], encoded[72];
	struct relay_control c = {
		.epoch = 255, .repair_count = 5, .session = 0x12345678,
		.frame = 0xFFFFFFFF, .preamble_ticks = 0xDEADBEEF,
		.members = 0x83FF, .expected = 0x83FF, .missing = {1, 2, 4},
		.primary_valid = 1, .backup_valid = 1, .helpers = {0x2F},
		.maintenance_owner = RELAY_OWNER_IDLE, .profile = RELAY_PROFILE_120HZ,
	};
	for (unsigned i = 0; i < 5; i++) {
		c.grants[i] = (struct relay_grant){15, 1};
	}
	assert(relay_control_encode(bytes, &c) == 72);
	assert(bytes[0] == 0xE8 && bytes[1] == RELAY_V3_VERSION && bytes[4] == 0x12 && bytes[7] == 0x78);
	assert(bytes[12] == 0xDE && bytes[15] == 0xEF && bytes[30] == 0x2F);
	assert(bytes[46] == 15 && bytes[51] == 0 && bytes[52] == 1);
	assert(RELAY_CONTROL_OFF_OWNERS == 46 && RELAY_CONTROL_OFF_CHILDREN == 51);
	assert(bytes[RELAY_CONTROL_OFF_OWNERS] == 15);
	assert(relay_get_be16(bytes + RELAY_CONTROL_OFF_CHILDREN) == 1);
	assert(RELAY_CONTROL_OFF_SURVEY == 61 && RELAY_CONTROL_OFF_FLAGS == 67);
	struct relay_control decoded;
	assert(relay_control_decode(bytes, 72, &decoded));
	assert(relay_control_encode(encoded, &decoded) == 72 && !memcmp(bytes, encoded, 72));
	for (unsigned i = 0; i < 72; i++) {
		bytes[i] ^= 1;
		assert(!relay_control_decode(bytes, 72, &decoded));
		bytes[i] ^= 1;
	}
	for (unsigned n = 0; n < 72; n++) {
		assert(!relay_control_decode(bytes, n, &decoded));
	}
	bytes[1] = 2; bytes[71] = relay_crc8(7, bytes, 71);
	assert(!relay_control_decode(bytes, 72, &decoded));
	c.grants[0].owner = 16;
	relay_control_encode(bytes, &c);
	assert(!relay_control_decode(bytes, 72, &decoded));
	c.grants[0].owner = 15; c.grants[0].children |= 0x8000;
	relay_control_encode(bytes, &c);
	assert(!relay_control_decode(bytes, 72, &decoded));
	assert(!relay_profile_valid(1, 0xFFF, 0xFFF, 16, 5));
	assert(!relay_profile_valid(1, 1, 2, 16, 5));
	assert(!relay_profile_valid(1, 1, 1, 17, 5));
	assert(!relay_profile_valid(1, 1, 1, 16, 6));
}

static void repairs(void)
{
	uint8_t bytes[72], frames[3][61] = {{1, 0}, {1, 1}, {1, 15}};
	struct relay_repair r = {.helper = 3, .epoch = 255, .session = 0xFFFFFFFF, .count = 3};
	for (unsigned i = 0; i < 3; i++) {
		frames[i][16] = (uint8_t)(255 + i);
		r.records[i] = (struct relay_repair_record){(uint16_t)(65535 + i), {frames[i], 17, 50}};
	}
	assert(relay_repair_encode(bytes, &r) == 71);
	struct relay_repair decoded;
	assert(relay_repair_decode(bytes, 71, &decoded));
	assert(decoded.records[2].source.frame[1] == 15 && decoded.records[0].capture_frame == 65535);
	for (unsigned n = 0; n < 71; n++) {
		assert(!relay_repair_decode(bytes, n, &decoded));
	}
	uint8_t saved = bytes[34];
	bytes[34] = bytes[13];
	relay_put_be16(bytes + 29, relay_get_be16(bytes + 8));
	assert(!relay_repair_decode(bytes, 71, &decoded));
	bytes[34] = saved;
	assert(relay_repair_encode(bytes, &r) == 71);
	r.records[1] = r.records[0];
	assert(relay_repair_encode(bytes, &r) == 71 && !relay_repair_decode(bytes, 71, &decoded));
	r.records[1] = (struct relay_repair_record){0, {frames[1], 17, 50}};
	bytes[3] |= 4;
	assert(!relay_repair_decode(bytes, 71, &decoded));
	r.count = 1; frames[0][0] = RELAY_COMPOSITE_TYPE; frames[0][2] = 1;
	r.records[0].source.length = 60;
	assert(relay_repair_encode(bytes, &r) == 72 && relay_repair_decode(bytes, 72, &decoded));
	r.records[0].source.length = 61;
	assert(!relay_repair_encode(bytes, &r));
	for (unsigned id = 0; id < 16; id++) {
		frames[0][0] = 1; frames[0][1] = id; r.helper = (id + 1) % 16;
		r.records[0].source.length = 17;
		assert(relay_repair_encode(bytes, &r) == 29 && relay_repair_decode(bytes, 29, &decoded));
	}
	assert(relay_capture_age(0, 65535) && relay_capture_age(1, 65535));
	assert(!relay_capture_age(2, 65535) && !relay_capture_age(65535, 0));
}

static void descriptors(void)
{
	uint8_t bytes[72];
	struct relay_layout_descriptor d = {.session = 0xFFFFFFFF, .epoch = 255, .profile = 1,
		.members = 0x8001, .expected = 0x8001, .slot_ticks = 16, .repair_count = 5,
		.origin_ticks = 0xFFFFFF80, .activation_ticks = 123, .frame_seed = 65535};
	struct relay_layout_descriptor decoded;
	assert(
		relay_layout_encode(bytes, &d) == RELAY_LAYOUT_LEN && relay_layout_decode(bytes, RELAY_LAYOUT_LEN, &decoded)
	);
	assert(decoded.session == d.session && decoded.frame_seed == d.frame_seed);
	for (unsigned i = 0; i < RELAY_LAYOUT_LEN; i++) {
		bytes[i] ^= 1;
		assert(!relay_layout_decode(bytes, RELAY_LAYOUT_LEN, &decoded));
		bytes[i] ^= 1;
	}
	struct relay_peer_report r = {.helper = 15, .epoch = 255, .session = 0x12345678, .count = 8};
	for (unsigned i = 0; i < 8; i++) {
		r.entries[i] = (struct relay_peer_observation){i, 65535, 120, i, 40, 0};
	}
	struct relay_peer_report report;
	assert(relay_peer_report_encode(bytes, &r) == 65 && relay_peer_report_decode(bytes, 65, &report));
	assert(report.entries[0].heard == 0 && report.entries[0].expected == 120);
	for (unsigned i = 0; i < 65; i++) {
		bytes[i] ^= 1; assert(!relay_peer_report_decode(bytes, 65, &report)); bytes[i] ^= 1;
	}
	r.entries[0].heard = 121;
	assert(relay_peer_report_encode(bytes, &r) == 65 && !relay_peer_report_decode(bytes, 65, &report));
}

static void cadence(void)
{
	for (unsigned phase = 0; phase < 15; phase++) {
		for (unsigned wrap = 0; wrap < 2; wrap++) {
			uint32_t origin = wrap ? UINT32_MAX - 200 + phase : phase;
			for (uint32_t f = 0; f < 120; f++) {
				uint32_t start = relay_frame_start(origin, f);
				uint32_t end = relay_frame_start(origin, f + 1);
				assert(end - start == 273 || end - start == 274);
				for (uint32_t t = 0; t < end - start; t++) {
					assert(relay_frame_at_tick(origin, start + t) == f);
				}
				assert(relay_frame_at_tick(origin, end) == f + 1);
			}
			assert(relay_frame_start(origin, 120) - origin == 32768);
		}
	}
	assert(relay_direct_payload_max(1, 10) == 32 && relay_direct_payload_max(1, 9) == 60);
	assert(relay_direct_payload_max(0, 10) == RELAY_RECORD_MAX_LEN);
}

static void freeze_mask_crc(void)
{
 uint8_t bytes[72]={0}, delta[71]={0};
 for(unsigned bit=0;bit<16;bit++) {
  memset(delta,0,sizeof(delta));
  delta[bit<8 ? 21 : 20]=1U<<(bit%8);
  uint8_t change=relay_crc8(0,delta,sizeof(delta));
  bytes[71]=relay_crc8(7,bytes,71);
  bytes[bit<8 ? 21 : 20]^=1U<<(bit%8);
  bytes[71]^=change;
  assert(bytes[71]==relay_crc8(7,bytes,71));
 }
}

static void frame_epochs(void)
{
	const uint64_t origin = 0xfffff000ULL;
	for (uint64_t t=0; t < (1ULL<<32)+8192; t+=4093) {
		uint32_t f=relay_frame_at_tick64(0,t);
		uint32_t start=relay_frame_start((uint32_t)origin,f);
		for (int d=-4; d<=550; d++) {
			uint64_t absolute=origin+(uint64_t)(f/15)*4096+(f%15)*4096/15+d;
			if (absolute<origin) continue;
			assert(relay_frame_near(f,start,(uint32_t)absolute)==relay_frame_at_tick64(origin,absolute));
		}
		if(t< (1ULL<<32)) assert(f==relay_frame_at_tick(0,(uint32_t)t));
	}
	/* Capture ages remain modular at the 16-bit boundary. */
	assert((uint16_t)(65537U-(uint16_t)65535U)==2);
}

static void control_coordinates(void)
{
	for (uint32_t f = 15728630; f < 15728660; f++) {
		uint32_t origin = 0xfffffff0U;
		uint32_t start = relay_frame_start(origin, f);
		uint32_t preamble = start + RELAY_V3_B_PREAMBLE;
		assert(relay_control_start(preamble) == start);
		assert(relay_control_origin(preamble, f) == origin);
		for (unsigned i = 0; i < RELAY_V3_REPAIR_SLOTS; i++) {
			uint32_t at = relay_repair_start(preamble, i);
			assert(at == start + RELAY_V3_REPAIR_START + i * RELAY_V3_SLOT_TICKS);
			assert(relay_frame_near(f, start, at) == f);
			assert(relay_repair_slot(relay_frame_phase(origin, f, at)) == i);
		}
	}
}

int main(void)
{
 control_coordinates(); frame_epochs();
	freeze_mask_crc(); control(); repairs(); descriptors(); cadence();
	puts("v3 wire and rational cadence: passed");
	return 0;
}
