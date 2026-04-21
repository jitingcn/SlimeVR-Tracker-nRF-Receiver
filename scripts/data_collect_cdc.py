#!/usr/bin/env python
# /// script
# dependencies = [
#   "pyserial",
# ]
# ///

"""
SlimeVR Raw Sensor Data Collector

Reads raw sensor data from the SlimeVR receiver via CDC ACM serial port
and saves it to a binary file for offline VQF parameter optimization.

Binary framing protocol (from receiver):
  [0xAA][0x55][length][payload...][rssi][rx_ticks(4)][CRC-8]

Packet types in payload:
  0x10: Raw IMU (48 bytes) - 3 batched int16 gyro+accel samples
  0x11: Raw Mag (17 bytes) - int16 magnetometer
  0x12: Metadata (48 bytes) - ODR, range, sensor IDs (sent once)

Output: Binary file (.bin) + optional CSV conversion
"""

import argparse
import struct
import sys
import time
from pathlib import Path

# SlimeNRF Receiver USB IDs
SLIME_VID = 0x1209
SLIME_PID = 0x7690


def crc8_ccitt(data: bytes, init: int = 0x07) -> int:
    """CRC-8 CCITT (polynomial 0x07)."""
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


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


def parse_raw_imu(payload: bytes, meta: SensorMetadata):
    """Parse type 0x10 raw IMU packet (float format, no timestamp). Returns a single sample dict."""
    if len(payload) < 42:
        return None

    _tracker_id = payload[1]
    seq = struct.unpack_from(">H", payload, 2)[0]

    # Float gyro/accel (native byte order from ARM, little-endian)
    gx, gy, gz = struct.unpack_from("<fff", payload, 4)
    ax, ay, az = struct.unpack_from("<fff", payload, 16)

    flags = payload[40]
    temperature = payload[41]

    # Decode temperature
    temp_c = ((temperature - 128.5) / 2 + 25) if temperature > 0 else None

    # Mag (piggybacked if flag bit 0 is set)
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


def parse_raw_mag(payload: bytes, _meta: SensorMetadata):
    """Parse type 0x11 raw mag packet (float format)."""
    if len(payload) < 16:
        return None

    seq = struct.unpack_from(">H", payload, 2)[0]
    mx, my, mz = struct.unpack_from("<fff", payload, 4)

    return {
        "seq": seq,
        "mag": (mx, my, mz),
    }


def read_frames(port, baudrate=115200):
    """Generator that yields parsed frames from serial port."""
    import serial

    ser = serial.Serial(port, baudrate, timeout=0.1)
    buf = bytearray()

    try:
        while True:
            data = ser.read(256)
            if data:
                buf.extend(data)

            # Look for sync marker
            while len(buf) >= 4:
                # Find sync bytes
                idx = buf.find(b"\xAA\x55")
                if idx < 0:
                    # No sync found, keep last byte in case it's 0xAA
                    if len(buf) > 1:
                        buf = buf[-1:]
                    break

                if idx > 0:
                    buf = buf[idx:]

                if len(buf) < 3:
                    break

                frame_len = buf[2]  # payload length (includes rssi + ticks)
                total_frame = 3 + frame_len + 1  # sync(2) + len(1) + payload + crc(1)

                if len(buf) < total_frame:
                    break

                # Extract frame
                frame_data = bytes(buf[3 : 3 + frame_len])
                frame_crc = buf[3 + frame_len]

                # Verify CRC
                calc_crc = crc8_ccitt(bytes([frame_len]) + frame_data)
                if calc_crc != frame_crc:
                    # CRC mismatch, skip this sync and look for next
                    buf = buf[2:]
                    continue

                # Extract footer (rssi + rx_ticks)
                esb_len = frame_len - 5  # subtract rssi(1) + ticks(4)
                if esb_len < 2:
                    buf = buf[total_frame:]
                    continue

                esb_payload = frame_data[:esb_len]
                rssi = frame_data[esb_len]
                rx_ticks = struct.unpack_from(">I", frame_data, esb_len + 1)[0]

                yield esb_payload, rssi, rx_ticks
                buf = buf[total_frame:]

            if not data:
                time.sleep(0.001)
    finally:
        ser.close()


def collect(port, output_path, duration=None):
    """Main data collection loop with incremental file writing."""
    meta = SensorMetadata()
    start_time = time.time()
    first_sample_time = None
    frame_count = 0
    sample_count = 0
    last_status = start_time
    last_status_samples = 0
    last_rssi = 0
    retransmit_count = 0
    gap_count = 0  # sequence gaps never filled (actual loss)

    # Reorder buffer for ARQ retransmits
    REORDER_BUF_MAX = 64
    reorder_buf = {}  # seq -> csv_line
    write_cursor = None

    base = Path(output_path)
    meta_path = base.with_suffix(".meta.txt")
    csv_path = base.with_suffix(".csv")

    print(f"Collecting data from {port}...")
    print(f"Output: {csv_path}")
    if duration:
        print(f"Duration: {duration}s")
    print("Press Ctrl+C to stop\n")

    csv_file = open(csv_path, "w")
    csv_file.write("seq,gx,gy,gz,ax,ay,az,mx,my,mz,temp\n")

    try:
        for esb_payload, rssi, rx_ticks in read_frames(port):
            pkt_type = esb_payload[0]
            frame_count += 1
            last_rssi = rssi

            if pkt_type == 0x12:  # Metadata
                meta.parse(esb_payload)
                print(f"\nMetadata received: {meta}")
                with open(meta_path, "w") as f:
                    f.write(f"gyro_range_dps={meta.gyro_range}\n")
                    f.write(f"accel_range_g={meta.accel_range}\n")
                    f.write(f"gyro_odr_hz={meta.gyro_odr}\n")
                    f.write(f"accel_odr_hz={meta.accel_odr}\n")
                    f.write(f"mag_odr_hz={meta.mag_odr}\n")
                    f.write(f"imu_id={meta.imu_id}\n")
                    f.write(f"mag_id={meta.mag_id}\n")

            elif pkt_type == 0x10:  # Raw IMU
                s = parse_raw_imu(esb_payload, meta)
                if s:
                    seq = s["seq"]

                    # Detect late retransmit (behind write cursor)
                    if write_cursor is not None:
                        diff = (seq - write_cursor) & 0xFFFF
                        if diff > 0x8000:
                            retransmit_count += 1
                            continue
                    if seq in reorder_buf:
                        continue

                    if write_cursor is None:
                        write_cursor = seq
                        first_sample_time = time.time()

                    mag = s.get("mag") or (0.0, 0.0, 0.0)
                    temp = s.get('temp_c') or 0.0
                    csv_line = (
                        f"{seq},"
                        f"{s['gyro'][0]:.6f},{s['gyro'][1]:.6f},{s['gyro'][2]:.6f},"
                        f"{s['accel'][0]:.6f},{s['accel'][1]:.6f},{s['accel'][2]:.6f},"
                        f"{mag[0]:.6f},{mag[1]:.6f},{mag[2]:.6f},"
                        f"{temp:.2f}\n"
                    )

                    reorder_buf[seq] = csv_line

                    # Detect retransmit: packet filling a gap in reorder buffer
                    if (seq - write_cursor) & 0xFFFF > 0 and \
                       (seq - write_cursor) & 0xFFFF < REORDER_BUF_MAX:
                        has_later = any(
                            0 < ((s2 - seq) & 0xFFFF) < REORDER_BUF_MAX
                            for s2 in reorder_buf if s2 != seq
                        )
                        if has_later:
                            retransmit_count += 1

                    # Flush contiguous
                    while write_cursor in reorder_buf:
                        csv_file.write(reorder_buf.pop(write_cursor))
                        sample_count += 1
                        write_cursor = (write_cursor + 1) & 0xFFFF

                    # Force flush if too many buffered
                    if len(reorder_buf) >= REORDER_BUF_MAX:
                        sorted_seqs = sorted(reorder_buf.keys(),
                                             key=lambda s: (s - write_cursor) & 0xFFFF)
                        for sq in sorted_seqs:
                            gap = (sq - write_cursor) & 0xFFFF
                            if gap > 0:
                                gap_count += gap
                                write_cursor = sq
                            csv_file.write(reorder_buf[sq])
                            sample_count += 1
                            write_cursor = (write_cursor + 1) & 0xFFFF
                        reorder_buf.clear()

            # Flush periodically with status update
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
        # Flush remaining reorder buffer
        if reorder_buf and write_cursor is not None:
            sorted_seqs = sorted(reorder_buf.keys(),
                                 key=lambda s: (s - write_cursor) & 0xFFFF)
            for sq in sorted_seqs:
                gap = (sq - write_cursor) & 0xFFFF
                if gap > 0:
                    gap_count += gap
                    write_cursor = sq
                csv_file.write(reorder_buf[sq])
                sample_count += 1
                write_cursor = (write_cursor + 1) & 0xFFFF
            reorder_buf.clear()
        csv_file.close()

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
        # Append duration and actual ODR to metadata
        with open(meta_path, "a") as f:
            f.write(f"duration_s={data_duration:.3f}\n")
            f.write(f"sample_count={sample_count}\n")
        print(f"  Metadata: {meta_path}")


def find_data_port():
    """Find SlimeNRF CDC ports and list them for the user.

    With dual CDC ACM, the receiver enumerates two serial ports:
      - Lower interface/port number = console
      - Higher interface/port number = data collection

    Returns the device path if only one port found, otherwise None.
    """
    from serial.tools import list_ports

    candidates = []
    for p in list_ports.comports():
        if p.vid == SLIME_VID and p.pid == SLIME_PID:
            candidates.append(p)

    if not candidates:
        return None

    if len(candidates) == 1:
        print(f"Found one SlimeNRF port: {candidates[0].device}")
        return candidates[0].device

    # Sort by interface number or device path
    candidates.sort(key=lambda p: (getattr(p, 'interface_number', None) or 0, p.device))
    print("Found multiple SlimeNRF CDC ports:")
    for i, p in enumerate(candidates):
        iface = getattr(p, 'interface_number', '?')
        print(f"  [{i}] {p.device}  (interface={iface}, {p.description})")
    print(f"\nData port is usually the higher-numbered port.")
    print(f"Specify the port manually, e.g.: data_collect.py {candidates[-1].device}")
    return None


def main():
    parser = argparse.ArgumentParser(
        description="SlimeVR Raw Sensor Data Collector"
    )
    parser.add_argument(
        "port",
        nargs="?",
        default=None,
        help="CDC ACM serial port for data (e.g., /dev/ttyACM1). "
             "Auto-detected if omitted.",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="sensor_data",
        help="Output file base name (default: sensor_data)",
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=None,
        help="Collection duration in seconds (default: until Ctrl+C)",
    )
    parser.add_argument(
        "-b",
        "--baudrate",
        type=int,
        default=115200,
        help="Serial baudrate (default: 115200, CDC ACM ignores this)",
    )

    args = parser.parse_args()

    try:
        import serial  # noqa: F401
    except ImportError:
        print("Error: pyserial is required. Install with: pip install pyserial")
        sys.exit(1)

    port = args.port
    if port is None:
        port = find_data_port()
        if port is None:
            print("Error: No SlimeNRF receiver found. "
                  "Specify port manually, e.g.: data_collect.py /dev/ttyACM1")
            sys.exit(1)

    collect(port, args.output, args.duration)


if __name__ == "__main__":
    main()
