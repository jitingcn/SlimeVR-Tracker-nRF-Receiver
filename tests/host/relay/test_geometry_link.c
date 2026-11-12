#include <assert.h>
#include <stdio.h>
#include "relay_protocol.h"
void select_geometry(unsigned index);
int main(void)
{
	for (unsigned row = 0; row < 3; row++) {
		select_geometry(row);
		const struct relay_geometry *g = relay_geometry_current();
		const uint32_t deltas[] = {0, 1, 0x7fffffffU, 0x80000000U, 0xffffffffU};
		for (unsigned phase = 0; phase < g->denominator; phase++) {
			for (unsigned d = 0; d < sizeof(deltas) / sizeof(deltas[0]); d++) {
				uint32_t frame = 3000 * g->denominator + phase;
				int64_t delta = (int32_t)deltas[d] + (int64_t)(frame % g->denominator) * g->numerator / g->denominator;
				int64_t blocks = delta / g->numerator, rem = delta % g->numerator;
				if (rem < 0) {
					rem += g->numerator;
					blocks--;
				}
				uint32_t expected
					= frame - frame % g->denominator
					+ (uint32_t)(blocks * g->denominator + (rem * g->denominator + g->denominator - 1) / g->numerator);
				assert(relay_frame_near(frame, 100, deltas[d] + 100) == expected);
			}
		}
		for (unsigned frame = 1; frame < 10000; frame++) {
			uint32_t start = (uint32_t)((uint64_t)frame * g->numerator / g->denominator);
			assert(relay_frame_start(0, frame) == start);
			assert(relay_frame_at_tick(0, start) == frame);
			assert(relay_frame_at_tick64(0, start) == frame);
			assert(relay_frame_near(frame, start, start - 1) == frame - 1);
			assert(
				relay_frame_start_near(frame, start, frame + 1)
				== (uint32_t)((uint64_t)(frame + 1) * g->numerator / g->denominator)
			);
		}
	}
	puts("Cross-translation-unit receiver geometry dispatch: both periods passed");
}
