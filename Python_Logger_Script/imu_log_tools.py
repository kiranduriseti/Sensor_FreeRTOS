#!/usr/bin/env python3

'''
For test:
Syntetic log generation:
python imu_log_tools.py --make-test log.bin --seconds 10 --rate_hz 200
python imu_log_tools.py --bin log.bin
python imu_log_tools.py --bin log.bin --show
python imu_log_tools.py --bin log.bin --csv out.csv

For real logs:
python imu_log_tools.py --bin log.bin --show
'''

from __future__ import annotations

import argparse
import struct
import math
import random
from dataclasses import dataclass
from typing import Iterator, List, Optional, Tuple

import pandas as pd
import matplotlib.pyplot as plt

# ----------------------------
# Record format (your struct)
# ----------------------------
RECORD_FMT = "<Ihhhhhh"   # little-endian: uint32 + 6x int16
RECORD_SIZE = struct.calcsize(RECORD_FMT)

# Trailer format you write: [crc32][chunk_len] (both little-endian uint32)
TRAILER_FMT = "<II"
TRAILER_SIZE = struct.calcsize(TRAILER_FMT)  # 8

DEFAULT_ACCEL_RANGE_G = 2
DEFAULT_GYRO_RANGE_DPS = 250

ACC_LSB_PER_G = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
GYRO_LSB_PER_DPS = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}

@dataclass
class Record:
    t_ms: int
    ax: int
    ay: int
    az: int
    gx: int
    gy: int
    gz: int

# ----------------------------
# CRC-32/MPEG-2 (STM32 default CRC, bytes mode, no inversion)
# poly = 0x04C11DB7, init = 0xFFFFFFFF, refin/refout = False, xorout = 0
# ----------------------------
def crc32_mpeg2(data: bytes, init: int = 0xFFFFFFFF) -> int:
    poly = 0x04C11DB7
    crc = init & 0xFFFFFFFF
    for b in data:
        crc ^= (b << 24)  # non-reflected: byte goes into MSB
        for _ in range(8):
            if (crc & 0x80000000) != 0:
                crc = ((crc << 1) ^ poly) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc

def iter_records_from_payload(payload: bytes) -> Iterator[Record]:
    # payload is raw concatenation of packed records
    n = len(payload) // RECORD_SIZE
    for i in range(n):
        off = i * RECORD_SIZE
        t_ms, ax, ay, az, gx, gy, gz = struct.unpack_from(RECORD_FMT, payload, off)
        yield Record(t_ms, ax, ay, az, gx, gy, gz)

def read_chunks_backwards(path: str, verify_crc: bool = True) -> List[bytes]:
    """
    Returns payload chunks in chronological order (start->end),
    parsing file format: [payload][crc,len] repeated, but read backwards.
    """
    with open(path, "rb") as f:
        blob = f.read()

    pos = len(blob)
    chunks_rev: List[bytes] = []

    while pos >= TRAILER_SIZE:
        trailer = blob[pos - TRAILER_SIZE:pos]
        stored_crc, chunk_len = struct.unpack(TRAILER_FMT, trailer)

        payload_end = pos - TRAILER_SIZE
        payload_start = payload_end - chunk_len

        if payload_start < 0:
            raise ValueError(
                f"Corrupt file: chunk_len={chunk_len} goes before BOF at pos={pos}"
            )

        payload = blob[payload_start:payload_end]

        # Basic sanity checks
        if chunk_len != len(payload):
            raise ValueError("Internal length mismatch (should never happen).")
        if chunk_len % RECORD_SIZE != 0:
            # You *expect* chunk payload to be multiple of 16 (packed records)
            raise ValueError(
                f"Chunk payload not multiple of RECORD_SIZE ({RECORD_SIZE}). "
                f"chunk_len={chunk_len}"
            )

        if verify_crc:
            calc = crc32_mpeg2(payload)
            if calc != stored_crc:
                raise ValueError(
                    f"CRC mismatch for chunk ending at {pos}: "
                    f"stored=0x{stored_crc:08X} calc=0x{calc:08X} len={chunk_len}"
                )

        chunks_rev.append(payload)
        pos = payload_start  # jump back to previous chunk

        if pos == 0:
            break

    if pos != 0:
        raise ValueError("Corrupt file: did not land exactly on BOF after parsing.")

    # reverse to chronological order
    return list(reversed(chunks_rev))

def iter_records(path: str, verify_crc: bool = True) -> Iterator[Record]:
    chunks = read_chunks_backwards(path, verify_crc=verify_crc)
    for payload in chunks:
        yield from iter_records_from_payload(payload)

def records_to_df(records: List[Record],
                  accel_range_g: int,
                  gyro_range_dps: int) -> pd.DataFrame:
    if accel_range_g not in ACC_LSB_PER_G:
        raise ValueError(f"Unsupported accel_range_g={accel_range_g}. Use one of {sorted(ACC_LSB_PER_G.keys())}")
    if gyro_range_dps not in GYRO_LSB_PER_DPS:
        raise ValueError(f"Unsupported gyro_range_dps={gyro_range_dps}. Use one of {sorted(GYRO_LSB_PER_DPS.keys())}")

    a_scale = 1.0 / ACC_LSB_PER_G[accel_range_g]
    g_scale = 1.0 / GYRO_LSB_PER_DPS[gyro_range_dps]

    rows = []
    for r in records:
        rows.append({
            "t_ms": r.t_ms,
            "t_s": r.t_ms / 1000.0,
            "ax_raw": r.ax, "ay_raw": r.ay, "az_raw": r.az,
            "gx_raw": r.gx, "gy_raw": r.gy, "gz_raw": r.gz,
            "ax_g": r.ax * a_scale,
            "ay_g": r.ay * a_scale,
            "az_g": r.az * a_scale,
            "gx_dps": r.gx * g_scale,
            "gy_dps": r.gy * g_scale,
            "gz_dps": r.gz * g_scale,
        })

    df = pd.DataFrame(rows)
    if not df.empty:
        df["a_mag_g"] = (df["ax_g"]**2 + df["ay_g"]**2 + df["az_g"]**2) ** 0.5
        df["g_mag_dps"] = (df["gx_dps"]**2 + df["gy_dps"]**2 + df["gz_dps"]**2) ** 0.5
    return df

def plot_df(df: pd.DataFrame, title: str = "IMU Log") -> None:
    if df.empty:
        print("No data to plot.")
        return

    plt.figure()
    plt.plot(df["t_s"], df["ax_g"], label="ax (g)")
    plt.plot(df["t_s"], df["ay_g"], label="ay (g)")
    plt.plot(df["t_s"], df["az_g"], label="az (g)")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (g)")
    plt.title(title + " - Accel")
    plt.legend()

    plt.figure()
    plt.plot(df["t_s"], df["gx_dps"], label="gx (dps)")
    plt.plot(df["t_s"], df["gy_dps"], label="gy (dps)")
    plt.plot(df["t_s"], df["gz_dps"], label="gz (dps)")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular rate (deg/s)")
    plt.title(title + " - Gyro")
    plt.legend()

    plt.figure()
    plt.plot(df["t_s"], df["a_mag_g"], label="|a| (g)")
    plt.plot(df["t_s"], df["g_mag_dps"], label="|g| (dps)")
    plt.xlabel("Time (s)")
    plt.ylabel("Magnitude")
    plt.title(title + " - Magnitudes")
    plt.legend()

def make_test_log(path: str, seconds: float, rate_hz: float) -> None:
    """
    Writes chunk in
      [payload records][crc32][payload_len]
    """
    dt = 1.0 / rate_hz
    n = int(seconds * rate_hz)

    payload = bytearray()
    for i in range(n):
        t_s = i * dt
        t_ms = int(round(t_s * 1000.0))

        ax_g = 0.05 * math.sin(2 * math.pi * 1.0 * t_s)
        ay_g = 0.05 * math.cos(2 * math.pi * 1.0 * t_s)
        az_g = 1.0 + 0.02 * math.sin(2 * math.pi * 0.2 * t_s)

        gx_dps = 5.0 * math.sin(2 * math.pi * 0.5 * t_s)
        gy_dps = 3.0 * math.cos(2 * math.pi * 0.5 * t_s)
        gz_dps = 15.0 * math.sin(2 * math.pi * 0.1 * t_s)

        ax = int(round(ax_g * ACC_LSB_PER_G[DEFAULT_ACCEL_RANGE_G]))
        ay = int(round(ay_g * ACC_LSB_PER_G[DEFAULT_ACCEL_RANGE_G]))
        az = int(round(az_g * ACC_LSB_PER_G[DEFAULT_ACCEL_RANGE_G]))
        gx = int(round(gx_dps * GYRO_LSB_PER_DPS[DEFAULT_GYRO_RANGE_DPS]))
        gy = int(round(gy_dps * GYRO_LSB_PER_DPS[DEFAULT_GYRO_RANGE_DPS]))
        gz = int(round(gz_dps * GYRO_LSB_PER_DPS[DEFAULT_GYRO_RANGE_DPS]))

        def clamp_i16(v: int) -> int:
            return max(-32768, min(32767, v))

        payload += struct.pack(RECORD_FMT,
                               t_ms,
                               clamp_i16(ax), clamp_i16(ay), clamp_i16(az),
                               clamp_i16(gx), clamp_i16(gy), clamp_i16(gz))

    crc = crc32_mpeg2(payload)
    trailer = struct.pack(TRAILER_FMT, crc, len(payload))

    with open(path, "wb") as f:
        f.write(payload)
        f.write(trailer)

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", help="Input binary log (e.g., log.bin)")
    ap.add_argument("--csv", help="Optional output CSV path")
    ap.add_argument("--show", action="store_true", help="Show plots")
    ap.add_argument("--no-verify", action="store_true", help="Skip CRC verification")

    ap.add_argument("--make-test", dest="make_test", help="Generate a synthetic test log to this path")
    ap.add_argument("--seconds", type=float, default=10.0, help="Seconds for synthetic test log")
    ap.add_argument("--rate_hz", type=float, default=200.0, help="Sample rate for synthetic test log")

    args = ap.parse_args()

    if args.make_test:
        make_test_log(args.make_test, args.seconds, args.rate_hz)
        print(f"Wrote synthetic test log: {args.make_test}")
        return

    if not args.bin:
        ap.error("Provide --bin log.bin (or use --make-test ...)")

    recs = list(iter_records(args.bin, verify_crc=not args.no_verify))
    if not recs:
        print("No complete records found.")
        return

    df = records_to_df(recs, DEFAULT_ACCEL_RANGE_G, DEFAULT_GYRO_RANGE_DPS)

    print(f"Records: {len(df)}  Duration: {df['t_s'].iloc[-1] - df['t_s'].iloc[0]:.3f}s")
    print(df.head(5).to_string(index=False))

    if args.csv:
        df.to_csv(args.csv, index=False)
        print(f"Wrote CSV: {args.csv}")

    if args.show:
        plot_df(df, title=args.bin)
        plt.show()

if __name__ == "__main__":
    main()
