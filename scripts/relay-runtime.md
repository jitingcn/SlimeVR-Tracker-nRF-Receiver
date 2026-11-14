# Runtime repair policy

Defaults are max_trackers=16, max_repairs=5, repair_floor=5, slot_ticks=16,
tps_min=120 and tps_max=120. The member cap is capacity; the staged layout's
actual member count T sets R=clamp(floor(32768/tps_min/slot_ticks)-T-1,1,max_repairs).
Equal requested floor/ceiling means static allocation at that derived R.
`tps_min` is a repair-budget target, not a hard throughput guarantee.

`relay layout tps_min 100` selects the supported redundancy option. It keeps
the 120 TPS cap while spending more slots on repairs as membership grows.
At sixteen members, 120 mode uses R1 and redundancy uses R3. Every eighth
frame with nominal R1 still opens R2 for the two-attempt maintenance exchange.
This yields 273/274-tick frames except 288 ticks at T15 maintenance, about
119.185 TPS, and 288/304 ticks at T16, about 112.993 TPS. No guard is reduced.

An explicit lower `repair_floor` opts into experimental on-demand repair.
Its ceiling is the same per-member derived R, and its floor is clamped to
that ceiling. The queued current/next/next2 budgets remain immutable except
for the established future choice. Membership/settings publish atomically
through staged activation and reset the demand queue to the new layout.

`relay layout second_b 1` opts into experimental adjacent duplicate B;
`relay layout second_b 2` opts into experimental trailing duplicate B;
`relay layout second_b 0` disables it. It trades one nominal repair for B2
only when the one-B layout has at least R3, leaving at least R2. Otherwise
it is suspended for that membership, preserving the desired setting. Demand
mode's effective floor is at least R2 while two Bs are active. Adjacent B2
shifts repairs one slot later; trailing B2 follows the last repair. Helpers
retain the same direct capture windows, and the extra B never owns an R slot.

Both copies carry the first freeze's identical grants and announcements.
B2 changes only type, preamble timestamp and CRC. Its type is 0xEC; first B
remains 0xE8. Byte 68's upper two bits encode placement 0/1/2, its lower bits
retain static width or demand owner/width. The version remains local 7 or
upstream consolidated 4. Trackers normalize B2 back to the first B anchor.
A valid first copy allows repair service immediately. If it is absent or
rejected, the window stays/reopens through B2; only failure of both counts
one missed frame. A late trailing B can restore future announcements but
cannot recover repairs whose slots have passed. The third whole missed B
frame still fences direct transmission. Both copies missed while acquiring
continue the existing bounded broad acquisition with poses fenced.

Settings use NVS record v3, same ten bytes, with placement in the width byte's
upper bits. v1 records infer static floor=ceiling; v2 preserves explicit floor.
The exact old static default 11/5/16/100/120/floor5 in v1 or v2 migrates to
16/5/16/120/120/floor5. Other valid settings survive. Since old records carry
no intent bit, an explicit choice identical to that old default necessarily
migrates too. New v3 records preserve that tuple as an explicit redundancy
choice. Invalid/absent records use the new defaults. No NVS rewrite happens
at boot. Setting the ceiling while floor equals ceiling keeps static mode;
a lower explicit floor is preserved or clamped down to a lowered ceiling.
The ceiling must be 1-5; R0 is not a supported policy setting.
