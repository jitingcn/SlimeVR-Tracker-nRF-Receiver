# Runtime repair policy

In relay-enabled builds, `relay auto` is the boot default and switches profiles based on direct-link
quality. `relay force` keeps an eligible fleet in profile 1 through direct-link
recovery. `relay off` returns to direct TDMA by staged activation. `relay on`
remains an alias for `relay auto`. Mode selection lasts until reboot and
`relay` reports it in the `permission` field. Layout settings have their own
persistent record.

Defaults are max_trackers=16, max_repairs=5, repair_floor=5, slot_ticks=16,
tps_min=120 and tps_max=120. Native auto keeps the full live roster. An auto
entry needs every live tracker to support relay and the roster to fit the cap.
Before the repeat-control adjustment, reserve four ticks for arming. The slot budget is `floor((floor(32768 / tps_min) - 4) / slot_ticks)`. For T members the nominal repair count is `clamp(budget - T - 1, 1, max_repairs)`.
Equal requested floor/ceiling means static allocation at that derived R.
`tps_min` is a repair-budget target, not a hard throughput guarantee.

`relay layout tps_min 100` selects the supported redundancy option. It keeps
the 120 TPS cap while spending more slots on repairs as membership grows.
At sixteen members, 120 mode uses R1 and redundancy uses R3. Every eighth
frame with nominal R1 still opens R2 for the two-attempt maintenance exchange.
With the arming guard, T15/R1 uses 276-tick frames with 292 ticks for
maintenance, about 117.87 TPS. T16/R1 uses 292/308 ticks, about 111.46 TPS.
During staging, a relay layout with fewer than two guaranteed repair slots
reserves a maintenance pair every eighth frame. Layouts with at least two
guaranteed slots reserve one every fourth frame. The lead budgets three rounds
of PONGs at the applied layout's slowest cadence, plus two frames and 150 ms.
T16/R1 needs 3.615 s at a 120 TPS cap and 4.014 s at a 100 TPS cap. Native
TDMA keeps its 3.15 s lead.

An explicit lower `repair_floor` opts into experimental on-demand repair.
Its ceiling is the same per-member derived R, and its floor is clamped to
that ceiling. The queued current/next/next2 budgets remain immutable except
for the established future choice. Membership/settings publish atomically
through staged activation and reset the demand queue to the new layout.

`relay layout control_repeat_mode 1` enables an experimental repeat control
frame after the primary control frame, with one empty slot between them.
`relay layout control_repeat_mode 2` places the repeat control frame after the
repair area. `relay layout control_repeat_mode 0` disables repetition.
Repetition trades one nominal repair for the repeat control frame only when
the layout with a single control frame has at least R3, leaving at least R2.
Otherwise it is suspended for that membership, preserving the desired setting.
Demand mode's effective floor is at least R2 while two control frames are active.
Placement 1 puts the repair area after both control transmissions. Placement 2
puts the repeat control after the last repair. Each control transmission has
its own four-tick arming guard. Helpers retain the same direct capture windows;
neither a guard nor a control transmission owns a repair slot.

Both copies carry the first freeze's identical grants and announcements.
The repeat control frame changes only type, preamble timestamp and CRC. Its
type is 0xEC; the primary control frame remains 0xE8. Byte 68's upper two bits
encode placement 0/1/2, its lower bits retain static width or demand owner/width.
Both firmware repositories use wire version 5 and the same guarded geometry.
Trackers normalize the repeat control frame back to the primary control anchor,
including the repeat transmission's guard. Update trackers and receiver together.
A valid primary control frame allows repair service immediately. If it is
absent or rejected, the window stays/reopens through the repeat control frame;
only failure of both counts one missed frame. A late trailing repeat control
frame can restore future announcements but cannot recover repairs whose slots
have passed. The third whole missed control frame still fences direct
transmission. Both copies missed while acquiring continue the existing bounded
broad acquisition with poses fenced.

Settings use NVS record version 3, same ten bytes, with placement in the width byte's
upper bits. v1 records infer static floor=ceiling; v2 preserves explicit floor.
The exact old static default 11/5/16/100/120/floor5 in v1 or v2 migrates to
16/5/16/120/120/floor5. Other valid settings survive. Since old records carry
no intent bit, an explicit choice identical to that old default necessarily
migrates too. New version-3 records preserve that tuple as an explicit redundancy
choice. Invalid/absent records use the new defaults. No NVS rewrite happens
at boot. Setting the ceiling while floor equals ceiling keeps static mode;
a lower explicit floor is preserved or clamped down to a lowered ceiling.
The ceiling accepts 1 to 5. With repeat control off, `relay layout repair_floor 0` allows ordinary on-demand frames with no repair slots.
