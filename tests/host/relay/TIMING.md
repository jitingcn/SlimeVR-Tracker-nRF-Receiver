# Relay broadcast timing verification

The frame constants, RX lead, admission reserves, mask freeze at preamble minus
3 ticks and RTC launch at preamble minus 2 ticks are unchanged.

Hardware testing on 2026-11-02 disproved the previous full-path cycle estimate.
With all 11 trackers present, the original implementation sent 562 broadcasts
and failed 2,106, almost all with -ETIME. Its freeze-to-return maximum was 4,485
cycles, including cancellation. The advertised 1,544-cycle model was not a
hardware WCET bound. Tracker windows could not receive broadcasts that had been
cancelled before transmission.

Two RTC channels now serve distinct purposes. One compare wakes the freeze
callback ahead of the deadline, allowing for RTC ISR dispatch. The callback
polls the RTC counter until the original freeze tick, then freezes the receipt
masks. The other compare remains interrupt-masked and drives TIMER START through
PPI at the original launch tick. TIMER COMPARE1 triggers TXEN 21 us later; RADIO
then uses its existing 40 us fast ramp before START.

Radio arming does not depend on the encoded missing masks. Arm immediately after
freezing them, then finish the grant selection and CRC in the dedicated DMA
buffer while the hardware launch remains pending. IRQs remain locked through
publication. The measured arm maximum was 1,300 cycles and subsequent encoding
maximum 1,028 cycles. At 64 MHz, encoding takes less than the existing 21 us
TIMER delay, even if arming finishes immediately before the launch edge. It is
complete before TXEN, and therefore before START consumes the DMA payload.
The critical arm functions execute from RAM to avoid instruction-cache misses.

`freeze_arm_max` measures successful hardware arming, excluding subsequent
encoding and cancellation. `encode_max` measures the post-arm encoder. `B late`
counts rejected hardware launch deadlines. `entry_late` and `entry_max` measure
whole-tick lateness at the actual mask freeze. `grant_max` measures elapsed ticks
from the freeze to completion of the encoded payload. These are cumulative
hardware observations, not formal WCET guarantees. Compare counter deltas over
a stable fleet interval, excluding reset and profile acquisition.

The 16 KiB log buffer retains the complete relay diagnostic response, including
its timing summary and helper table. No timing window was widened.

The linked-firmware emulator checks DMA bytes against the former policy and
measures both deadlines separately. It counts RAM-resident instructions and
models DWT reads, with fixture RAM placed beyond the firmware data. The arm
assertion retains its original 1,562-cycle budget including caller allowance;
the post-arm encoding assertion requires completion within the existing 21 us
TIMER delay. A 10,000-case run measured 1,130 modeled arm cycles and 795 modeled
encoding cycles, with matching DMA bytes and passing ownership failure cases.
Hardware DWT measurements remain necessary because the model omits cache and
bus contention and interrupt-entry delays.

## Deferred RX work and validation

HID construction/filtering/RSSI smoothing/output enqueue run in a worker.
Capture and peer-report learning run in another worker with copied receive-time
grants and epoch/session fencing. TDMA phase, drift, histograms and EMA accounting
now run there too, from immutable timestamp/config/clock snapshots in a 64-entry
ring. Sequence checks, live receipts, admission counters and state needed by
radio/ACK handling remain synchronous. DWT RX max/average covers each EVENT IRQ
packet including dequeue, but excludes the separate RADIO/ACK ISR and worker time.

Queue overflow drains the oldest work synchronously, preserving data and order;
status exposes overflow counts. The diagnostic worker detects a sample already
consumed by overflow, and generation checks fence resets. Tests cover copies,
ordering, overflow, reset fencing and modeled timestamp preemption. Sustained
hardware load should verify that overflow remains zero and worker critical
sections do not delay freeze entry.

## Hardware acceptance status

The mixed-image diagnostic fleet sent 3,123 broadcasts with 15 failures and 12
late arms. Smol 2 subsequently received 8,977 B frames and missed 375 in a
stable interval (4.0% misses), with 102–112 receiver TPS in three samples and
zero maintenance lateness (maximum lag 3 ticks). Direct admission and final
validation refusals remained above the acceptance target.

After flashing the final tracker code to both RevF devices and all nine Smols,
Smol 5 repeatedly failed IMU discovery. Four reboots and a final console query
still reported `IMU: None`; its capability became zero. The receiver correctly
reported `v3 requested=on eligible=0`, holding the entire fleet in profile 0.
Thus full-fleet profile-1 throughput, all-tracker B/maintenance statistics,
learned helpers and forced-drop recovery have not passed acceptance. No forced
drops remain configured. Restore Smol 5's sensor detection before resuming the
full-fleet measurements; do not bypass capability eligibility.

Host suites retain the receiver 21/25 and tracker 21/42 baselines, including
protocol-header byte parity. All three firmware targets build. The final image
build stamps are receiver 14:22:57, Smols 14:25:59 and RevF 14:27:03–04 on
2026-11-02. Console commit identifiers are stale CMake-generated metadata;
these build stamps and the locally recorded firmware hashes identify the
images used for this run.
