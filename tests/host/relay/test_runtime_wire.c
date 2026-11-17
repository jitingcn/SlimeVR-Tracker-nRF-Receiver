#define CONFIG_RELAY_RUNTIME_LAYOUT 1
#include <assert.h>
#include <stdio.h>
#include "relay_protocol.h"
static struct relay_geometry geometry = {16, 1, 16, 120, 292, 1, 0};
const struct relay_geometry *relay_geometry_active = &geometry;
struct relay_timeline relay_timeline_active;
struct relay_runtime_anchor relay_direct_anchor;
static void reseal(uint8_t *wire)
{
	wire[71] = relay_crc8(7, wire, 71);
}

static void guarded_geometry(void)
{
	const uint32_t frames[] = {0U, 1U, 14U, 119U, UINT32_MAX};
	for (unsigned width = 15; width <= 16; ++width) {
		for (unsigned direct = 1; direct <= 16; ++direct) {
			for (unsigned mode = 0; mode <= 2; ++mode) {
				for (unsigned repairs = mode ? 2U : 0U; repairs <= (mode ? 4U : 5U); ++repairs) {
					assert(relay_geometry_make_control(&geometry, direct, repairs, width, 120, mode));
					unsigned control = direct * width + 4U;
					unsigned repair = control + width + (mode == 1U ? 2U * width + 4U : 0U);
					unsigned occupied = (direct + 1U + repairs + relay_control_repeat_slots(mode)) * width
						+ 4U * (1U + (mode != 0U));
					assert(RELAY_CONTROL_START == control && RELAY_REPAIR_START == repair);
					for (unsigned i = 0; i < direct; ++i) assert(relay_slot_offset(i) == i * width);
					for (unsigned i = 0; i < repairs; ++i)
						assert(relay_slot_offset(RELAY_REPAIR_FIRST_SLOT + i) == repair + i * width);
					assert(control + RELAY_TX_OFFSET + RELAY_CONTROL_AIR_TICKS <= repair);
					if (mode) {
						unsigned repeat = control + relay_control_repeat_offset(direct, repairs, width, mode);
						assert(repeat - 5U == (mode == 1U ? control + 2U * width - 1U
										       : repair + repairs * width - 1U));
						assert(repeat + RELAY_TX_OFFSET + RELAY_CONTROL_AIR_TICKS
						       <= (mode == 1U ? repair : occupied));
					}
					assert(control - 5U == direct * width - 1U);
					assert(repair + repairs * width <= occupied);
					for (unsigned f = 0; f < sizeof(frames) / sizeof(frames[0]); ++f) {
						uint32_t frame = frames[f], start = 0xfffffff0U;
						struct relay_runtime_anchor a;
						relay_runtime_anchor_set(&a, frame, start + control + RELAY_TX_OFFSET,
									 direct, repairs, repairs, repairs, width, 120, mode);
						assert(a.start == start);
						assert(a.next_start - start == relay_runtime_period(frame, direct, repairs, width, 120, mode));
						assert(a.following_start - a.next_start
						       == relay_runtime_period(frame + 1U, direct, repairs, width, 120, mode));
						assert(a.third_start - a.following_start
						       == relay_runtime_period(frame + 2U, direct, repairs, width, 120, mode));
						assert(a.next_start - start >= occupied);
					}
				}
			}
		}
	}
}

int main(void)
{
	for (unsigned direct = 1; direct <= 16; direct++) {
		for (unsigned repairs = 0; repairs <= 5; repairs++) {
			for (unsigned next = 0; next <= 5; next++) {
				for (unsigned next2 = 0; next2 <= 5; next2++) {
					for (unsigned width = 15; width <= 16; width++) {
						struct relay_geometry bootstrap;
						uint8_t code = relay_geometry_code(direct, repairs);
						assert(relay_geometry_from_bootstrap(code, width, &bootstrap));
						assert(
							bootstrap.direct == direct && bootstrap.repairs == repairs && bootstrap.slot_ticks == width
						);
						struct relay_control c
							= {.direct_count = direct,
							   .repair_count = repairs,
							   .slot_ticks = width,
							   .tps_max = 121,
							   .profile = 1,
							   .members = (uint16_t)((1U << direct) - 1U),
							   .maintenance_owner = RELAY_OWNER_IDLE,
							   .session = 0x12345678,
							   .epoch = 9,
							   .frame = 123,
							   .preamble_ticks = 0xfffffff0U};
						c.expected = c.members;
						for (unsigned i = 0; i < 5; i++) {
							c.grants[i].owner = RELAY_OWNER_IDLE;
						}
						relay_control_set_next_repairs(&c, next);
						relay_control_set_next2_repairs(&c, next2);
						uint8_t wire[72];
						struct relay_control decoded;
						assert(relay_control_encode(wire, &c) == 72 && wire[1] == RELAY_PROTOCOL_VERSION);
						assert(relay_control_decode(wire, 72, &decoded));
						assert(decoded.direct_count == direct && decoded.slot_ticks == width && decoded.tps_max == 121);
						assert(
							decoded.repair_count == repairs && relay_control_next_repairs(&decoded) == next
							&& relay_control_next2_repairs(&decoded) == next2
						);
						/* Frozen maintenance patch must retain the future announcement. */
						struct relay_control_crc_plan crc = relay_control_plan_crc(wire);
						c.flags |= RELAY_CONTROL_PENDING;
						relay_control_finish_encode(wire, &c, crc, false);
						assert(
							relay_control_decode(wire, 72, &decoded) && relay_control_next_repairs(&decoded) == next
							&& relay_control_next2_repairs(&decoded) == next2
						);
						wire[1] = RELAY_PROTOCOL_VERSION - 1U;
						reseal(wire);
						assert(!relay_control_decode(wire, 72, &decoded));
						wire[1] = RELAY_PROTOCOL_VERSION + 1U;
						reseal(wire);
						assert(!relay_control_decode(wire, 72, &decoded));
						wire[1] = RELAY_PROTOCOL_VERSION;
						wire[68] = 14;
						reseal(wire);
						assert(!relay_control_decode(wire, 72, &decoded));
						wire[68] = width;
						wire[67] = 6U << 2;
						reseal(wire);
						assert(!relay_control_decode(wire, 72, &decoded));
						wire[67] = 6U << 5;
						reseal(wire);
						assert(!relay_control_decode(wire, 72, &decoded));
						struct relay_layout_descriptor d
							= {.direct_count = direct,
							   .repair_count = repairs,
							   .slot_ticks = width,
							   .tps_max = 121,
							   .members = c.members,
							   .expected = c.members,
							   .profile = 1};
						uint8_t descriptor[30];
						struct relay_layout_descriptor parsed;
						assert(
							relay_layout_encode(descriptor, &d) == 30 && relay_layout_decode(descriptor, 30, &parsed)
						);
						assert(
							parsed.direct_count == direct && parsed.tps_max == 121 && parsed.repair_count == repairs
						);
					}
				}
			}
		}
	}
	guarded_geometry();
	assert(relay_geometry_make(&geometry, 16, 1, 16, 120));
	for (uint32_t cap = 1; cap <= 32768; cap++) {
		uint32_t frame = cap * 123457U;
		uint32_t exact = ((uint64_t)frame + 1U) * 32768U / cap - (uint64_t)frame * 32768U / cap;
		if (exact < 20U) {
			exact = 20U;
		}
		assert(relay_runtime_period(frame, 0, 0, 16, cap, 0) == exact);
	}
	uint32_t start = 0xfffffff0U;
	for (uint32_t frame = 0; frame < 10000; frame++) {
		unsigned repairs = frame % 6, next = (frame + 1) % 6;
		uint32_t next_start;
		relay_runtime_anchor_set(
			&relay_direct_anchor,
			frame,
			start + 16U * 16U + RELAY_CONTROL_ARM_GUARD_TICKS + RELAY_TX_OFFSET,
			16,
			repairs,
			next,
			(frame + 2U) % 6U,
			16,
			120, 0
		);
		relay_timeline_announce(&relay_timeline_active, &relay_direct_anchor);
		assert(relay_frame_start(0, frame) == start);
		assert(relay_frame_start(0, frame + 1) == relay_direct_anchor.next_start);
		assert(relay_frame_at_tick(0, start + 1) == frame);
		assert(relay_frame_near(frame, start, relay_direct_anchor.following_start + 1) == frame + 2);
		assert(relay_runtime_start(&relay_direct_anchor, frame + 3U, &next_start));
		assert(
			next_start
			== relay_direct_anchor.following_start + relay_runtime_period(frame + 2U, 16, (frame + 2U) % 6U, 16, 120, 0)
		);
		assert(relay_frame_near(frame, start, relay_direct_anchor.third_start + 1U) == frame + 3U);
		assert(relay_frame_at_tick(0, relay_direct_anchor.third_start + 1U) == frame + 3U);
		assert(relay_frame_at_tick64(0, relay_direct_anchor.third_start + 1U) == frame + 3U);
		assert(relay_runtime_direct_granted(&relay_direct_anchor, relay_direct_anchor.third_start + 255U));
		assert(!relay_runtime_direct_granted(&relay_direct_anchor, relay_direct_anchor.third_start + 256U));
		start += relay_runtime_period(frame, 16, repairs, 16, 120, 0);
	}
	puts("Wire runtime geometry, every current/next/next2 R, frozen CRC, descriptors and timeline passed");
}
