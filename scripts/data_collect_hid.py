#!/usr/bin/env python
# /// script
# dependencies = [
#   "hidapi",
# ]
# ///

"""
SlimeVR Raw Sensor Data Collector (HID version)

Reads raw sensor data from the SlimeVR receiver's dedicated HID endpoint
and saves it to CSV for offline VQF parameter optimization.

HID report format (64 bytes):
  [0..N-1]    ESB payload (same format as over-the-air)
  [N]         RSSI
  [N+1..N+4]  rx_ticks (BE32, Zephyr uptime ticks)
  [N+5..63]   zero padding

Packet types in ESB payload (byte 0):
  0x10: Raw IMU (48 bytes) - float gyro+accel (+optional mag) + T-Cal temp
  0x11: Raw Mag (17 bytes) - float magnetometer
  0x12: Metadata (48 bytes) - ODR, range, sensor IDs

Output: CSV file with columns: seq,gx,gy,gz,ax,ay,az,mx,my,mz,temp
"""

import argparse
import math
import struct
import sys
import time
from pathlib import Path

# SlimeNRF Receiver USB IDs
SLIME_VID = 0x1209
SLIME_PID = 0x7690

# HID interface: vendor usage page 0xFF00 (data collection endpoint)
DC_USAGE_PAGE = 0xFF00


class SensorMetadata:
    """Stores metadata from type 0x12 packet."""

    def __init__(self):
        self.gyro_range = 0.0
        self.accel_range = 0.0
        self.gyro_odr = 0.0
        self.accel_odr = 0.0
        self.mag_odr = 0.0
        self.imu_id = 0
        self.mag_id = 0
        self.received = False

    def parse(self, payload: bytes):
        if len(payload) < 24:
            return
        self.gyro_range = struct.unpack_from("<f", payload, 2)[0]
        self.accel_range = struct.unpack_from("<f", payload, 6)[0]
        self.gyro_odr = struct.unpack_from("<f", payload, 10)[0]
        self.accel_odr = struct.unpack_from("<f", payload, 14)[0]
        self.mag_odr = struct.unpack_from("<f", payload, 18)[0]
        self.imu_id = payload[22]
        self.mag_id = payload[23]
        self.received = True

    def __str__(self):
        return (
            f"Gyro: {self.gyro_range:.0f} dps @ {self.gyro_odr:.0f} Hz, "
            f"Accel: {self.accel_range:.0f} g @ {self.accel_odr:.0f} Hz, "
            f"Mag: {self.mag_odr:.0f} Hz, "
            f"IMU ID: {self.imu_id}, Mag ID: {self.mag_id}"
        )


def parse_raw_imu(payload: bytes):
    """Parse type 0x10 raw IMU packet. Returns sample dict."""
    if len(payload) < 42:
        return None

    seq = struct.unpack_from(">H", payload, 2)[0]
    gx, gy, gz = struct.unpack_from("<fff", payload, 4)
    ax, ay, az = struct.unpack_from("<fff", payload, 16)

    flags = payload[40]
    temp_c = None
    if len(payload) >= 45:
        tcal_temp_c = struct.unpack_from("<f", payload, 41)[0]
        if math.isfinite(tcal_temp_c) and -100.0 < tcal_temp_c < 150.0:
            temp_c = tcal_temp_c

    mag = None
    if flags & 0x01:
        mx, my, mz = struct.unpack_from("<fff", payload, 28)
        mag = (mx, my, mz)

    return {
        "seq": seq,
        "gyro": (gx, gy, gz),
        "accel": (ax, ay, az),
        "mag": mag,
        "temp_c": temp_c,
    }


def find_data_hid(device_index=None):
    """Find the SlimeNRF data collection HID interface.

    The receiver has two HID interfaces:
      - Interface with standard usage page: tracker data
      - Interface with vendor usage page 0xFF00: data collection

    If multiple receivers are connected, lists them and requires
    device_index to select one.

    Returns (path, info_dict) or (None, None).
    """
    import hid

    devices = hid.enumerate(SLIME_VID, SLIME_PID)
    if not devices:
        return None, None

    # Filter for vendor-defined data collection interfaces
    dc_devices = [d for d in devices if d.get("usage_page") == DC_USAGE_PAGE]

    if not dc_devices:
        # Debug: show what was enumerated
        print("No device with vendor usage page 0xFF00 found.")
        print("Enumerated HID interfaces:")
        for d in devices:
            print(f"  interface={d.get('interface_number', '?')}, "
                  f"usage_page=0x{d.get('usage_page', 0):04X}, "
                  f"usage=0x{d.get('usage', 0):04X}")
        return None, None

    if len(dc_devices) == 1 and device_index is None:
        return dc_devices[0]["path"], dc_devices[0]

    if len(dc_devices) > 1:
        print(f"Found {len(dc_devices)} SlimeNRF data collection devices:")
        for i, d in enumerate(dc_devices):
            sn = d.get("serial_number", "?")
            mfr = d.get("manufacturer_string", "?")
            iface = d.get("interface_number", "?")
            print(f"  [{i}] SN={sn}  MFR={mfr}  interface={iface}")

        if device_index is None:
            print("\nUse --device N to select a device.")
            return None, None

    idx = device_index if device_index is not None else 0
    if idx >= len(dc_devices):
        print(f"Error: device index {idx} out of range (0-{len(dc_devices)-1})")
        return None, None

    return dc_devices[idx]["path"], dc_devices[idx]


def collect_hid(output_path, duration=None, device_index=None):
    """Main HID-based data collection loop with reorder buffer for ARQ retransmits."""
    import hid

    dev_path, dev_info = find_data_hid(device_index)
    if dev_path is None:
        print("Error: No SlimeNRF data collection HID endpoint found.")
        print("Make sure the receiver firmware was built with CONFIG_DATA_COLLECT=y")
        sys.exit(1)

    iface = dev_info.get("interface_number", "?")
    print(f"Found data HID: interface={iface}, "
          f"usage_page=0x{dev_info.get('usage_page', 0):04X}")

    h = hid.device()
    h.open_path(dev_path)

    meta = SensorMetadata()
    start_time = time.time()
    first_sample_time = None  # set when first data sample arrives
    frame_count = 0
    sample_count = 0
    last_status = start_time
    last_status_samples = 0
    last_rssi = 0
    retransmit_count = 0  # packets that filled a gap in the reorder buffer
    gap_count = 0  # sequence gaps that were never filled (actual loss)

    # Reorder buffer: holds samples until we can write them in order.
    # Keyed by sequence number; flushed when contiguous run available.
    REORDER_BUF_MAX = 128  # max buffered samples before force-flush
    reorder_buf = {}  # seq -> csv_line
    write_cursor = None  # next seq expected to be written

    base = Path(output_path)
    meta_path = base.with_suffix(".meta.txt")
    csv_path = base.with_suffix(".csv")

    print(f"Collecting data via HID...")
    print(f"Output: {csv_path}")
    if duration:
        print(f"Duration: {duration}s")
    print("Press Ctrl+C to stop\n")

    csv_file = open(csv_path, "w", encoding="utf-8")
    csv_file.write("seq,gx,gy,gz,ax,ay,az,mx,my,mz,temp\n")

    def flush_reorder_buf():
        """Write contiguous samples from write_cursor onwards."""
        nonlocal write_cursor, sample_count
        if write_cursor is None:
            return
        while write_cursor in reorder_buf:
            csv_file.write(reorder_buf.pop(write_cursor))
            sample_count += 1
            write_cursor = (write_cursor + 1) & 0xFFFF

    def force_flush_reorder_buf():
        """Force-flush oldest samples when buffer is too large."""
        nonlocal write_cursor, sample_count, gap_count
        if not reorder_buf:
            return
        # Sort remaining keys and write them all, counting gaps
        sorted_seqs = sorted(reorder_buf.keys(),
                             key=lambda s: (s - write_cursor) & 0xFFFF)
        for seq in sorted_seqs:
            # Count gaps between write_cursor and this seq
            gap = (seq - write_cursor) & 0xFFFF
            if gap > 0:
                gap_count += gap
                write_cursor = seq
            csv_file.write(reorder_buf[seq])
            sample_count += 1
            write_cursor = (write_cursor + 1) & 0xFFFF
        reorder_buf.clear()

    try:
        while True:
            report = h.read(64, timeout_ms=500)
            if not report:
                continue

            report = bytes(report)
            if len(report) < 2:
                continue

            pkt_type = report[0]
            frame_count += 1

            if pkt_type == 0x10:
                esb_len = 48
            elif pkt_type == 0x11:
                esb_len = 16
            elif pkt_type == 0x12:
                esb_len = 48
            else:
                continue

            esb_payload = report[:esb_len]
            if len(report) > esb_len:
                last_rssi = report[esb_len]

            if pkt_type == 0x12:  # Metadata
                meta.parse(esb_payload)
                print(f"\nMetadata received: {meta}")
                with open(meta_path, "w", encoding="utf-8") as f:
                    f.write(f"gyro_range_dps={meta.gyro_range}\n")
                    f.write(f"accel_range_g={meta.accel_range}\n")
                    f.write(f"gyro_odr_hz={meta.gyro_odr}\n")
                    f.write(f"accel_odr_hz={meta.accel_odr}\n")
                    f.write(f"mag_odr_hz={meta.mag_odr}\n")
                    f.write(f"imu_id={meta.imu_id}\n")
                    f.write(f"mag_id={meta.mag_id}\n")
                    f.write("temp_source=tcal_float_c\n")

            elif pkt_type == 0x10:  # Raw IMU
                s = parse_raw_imu(esb_payload)
                if s:
                    seq = s["seq"]

                    # Detect retransmit (seq < write_cursor or already in buffer)
                    if write_cursor is not None:
                        diff = (seq - write_cursor) & 0xFFFF
                        if diff > 0x8000:
                            # seq is behind write_cursor — late retransmit, already written
                            retransmit_count += 1
                            continue
                    if seq in reorder_buf:
                        # Duplicate
                        continue

                    # Initialize write cursor on first sample
                    if write_cursor is None:
                        write_cursor = seq
                        first_sample_time = time.time()

                    mag = s.get("mag") or (0.0, 0.0, 0.0)
                    temp = s.get("temp_c") or 0.0
                    csv_line = (
                        f"{seq},"
                        f"{s['gyro'][0]:.6f},{s['gyro'][1]:.6f},{s['gyro'][2]:.6f},"
                        f"{s['accel'][0]:.6f},{s['accel'][1]:.6f},{s['accel'][2]:.6f},"
                        f"{mag[0]:.6f},{mag[1]:.6f},{mag[2]:.6f},"
                        f"{temp:.6f}\n"
                    )

                    reorder_buf[seq] = csv_line

                    # Detect retransmit: a packet that fills a gap
                    # (write_cursor < seq, but seq is behind some already-buffered seqs)
                    diff = (seq - write_cursor) & 0xFFFF
                    if diff > 0 and diff < REORDER_BUF_MAX:
                        # Check if there are buffered seqs ahead of this one
                        # (meaning this seq was missing and just got retransmitted)
                        has_later = any(
                            0 < ((s2 - seq) & 0xFFFF) < REORDER_BUF_MAX
                            for s2 in reorder_buf if s2 != seq
                        )
                        if has_later:
                            retransmit_count += 1

                    # Flush contiguous samples from write cursor
                    flush_reorder_buf()

                    # Force flush if buffer is too large (gap not filled in time)
                    if len(reorder_buf) >= REORDER_BUF_MAX:
                        force_flush_reorder_buf()

            now = time.time()
            if now - last_status >= 2.0:
                csv_file.flush()
                elapsed = now - start_time
                period = now - last_status
                period_samples = sample_count - last_status_samples
                rate = period_samples / period if period > 0 else 0
                buffered = len(reorder_buf)
                retx_info = f", Retx: {retransmit_count}" if retransmit_count > 0 else ""
                buf_info = f", Buf: {buffered}" if buffered > 0 else ""
                # Loss = gaps that were never filled
                total = sample_count + gap_count
                loss_pct = gap_count / total * 100 if total > 0 else 0
                loss_info = f", Loss: {gap_count} ({loss_pct:.1f}%)" if gap_count > 0 else ""
                print(
                    f"\r[{elapsed:.1f}s] Samples: {sample_count} ({rate:.0f}/s)"
                    f"{retx_info}{buf_info}{loss_info}, RSSI: {last_rssi}   ",
                    end="",
                    flush=True,
                )
                last_status = now
                last_status_samples = sample_count

            if duration and (now - start_time >= duration):
                break

    except KeyboardInterrupt:
        print("\n\nStopping collection...")
    finally:
        # Flush remaining buffer
        if reorder_buf:
            force_flush_reorder_buf()
        csv_file.close()
        h.close()

    elapsed = time.time() - start_time
    data_duration = (time.time() - first_sample_time) if first_sample_time else elapsed
    print(f"\n\nCollection complete:")
    print(f"  Duration: {elapsed:.1f}s (data: {data_duration:.1f}s)")
    print(f"  Samples: {sample_count} -> {csv_path}")
    print(f"  Retransmits received: {retransmit_count}")
    total = sample_count + gap_count
    loss_pct = gap_count / total * 100 if total > 0 else 0
    print(f"  Gaps (lost): {gap_count} ({loss_pct:.2f}%)")
    if meta.received:
        with open(meta_path, "a", encoding="utf-8") as f:
            f.write(f"duration_s={data_duration:.3f}\n")
            f.write(f"sample_count={sample_count}\n")
        print(f"  Metadata: {meta_path}")


def main():
    parser = argparse.ArgumentParser(
        description="SlimeVR Raw Sensor Data Collector (HID)"
    )
    parser.add_argument(
        "-o", "--output", default="sensor_data",
        help="Output file base name (default: sensor_data)",
    )
    parser.add_argument(
        "-d", "--duration", type=float, default=None,
        help="Collection duration in seconds (default: until Ctrl+C)",
    )
    parser.add_argument(
        "--device", type=int, default=None,
        help="Device index when multiple receivers are connected (0-based)",
    )
    args = parser.parse_args()

    try:
        import hid  # noqa: F401
    except ImportError:
        print("Error: hidapi is required. Install with: pip install hidapi")
        sys.exit(1)

    collect_hid(args.output, args.duration, args.device)


if __name__ == "__main__":
    main()
