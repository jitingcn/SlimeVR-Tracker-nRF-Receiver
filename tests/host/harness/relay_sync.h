#pragma once

#include "relay_coord.h"

/* RX fixtures without a learning worker apply an admitted report immediately. */
static inline bool relay_defer_report(struct relay_coord *rc, const struct relay_control *grant,
	uint32_t frame, unsigned slot, const struct relay_peer_report *report, uint32_t now)
{
	if (grant->frame != frame || !relay_coord_report_granted(grant, slot, report)) {
		return false;
	}
	for (unsigned i = 0; i < report->count; i++) {
		const struct relay_peer_observation *e = &report->entries[i];
		relay_coord_observe(rc, report->helper, e->child, e->expected, e->heard, e->rssi, now);
	}
	return true;
}
