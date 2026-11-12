/* Local diagnostic ABI, never sent in relay protocol packets. */
#pragma once
#include <stdint.h>
enum ledger_event {
	LEDGER_SOURCE = 1,
	LEDGER_CAPTURE = 2,
	LEDGER_MISS = 3,
	LEDGER_SELECTED = 4,
	LEDGER_REPAIR = 5,
	LEDGER_B = 6,
	LEDGER_ACCEPT = 7,
	LEDGER_DUPLICATE = 8,
	LEDGER_OBSOLETE = 9,
	LEDGER_EXPIRE = 10,
	LEDGER_TARGET = 11,
	LEDGER_DIRECT_SEEN = 33,
	LEDGER_RESCUE_PLAN = 34,
	LEDGER_GRANT_HELPERS = 35,
	LEDGER_SURVEY_HISTORY = 36,
	LEDGER_CANDIDATE_FLAGS = 37,
	LEDGER_CANDIDATE_UPLINK = 38,
	LEDGER_CANDIDATE_RANK = 39,
};
#ifdef CONFIG_RELAY_FRAME_LEDGER
void ledger_note(uint32_t session, uint32_t frame, uint8_t child,
                 uint8_t event, int16_t detail, uint8_t peer, uint8_t age);
void ledger_command(int child);
void ledger_command_mask(uint16_t mask);
int ledger_child_enabled(unsigned child);
#else
#define ledger_note(...) ((void)0)
static inline void ledger_command(int child) { (void)child; }
static inline void ledger_command_mask(uint16_t mask) { (void)mask; }
#endif
