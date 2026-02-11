#!/usr/bin/env python3
"""
typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} data_read;

Defaults using mpu_init():
  - Accel: ±2g  (16384 LSB/g)
  - Gyro:  ±250 dps (131 LSB/(dps))

Usage:
  python imu_log_no_crc.py --bin log.bin
  python imu_log_no_crc.py --bin log.bin --csv out.csv
  python imu_log_no_crc.py --bin log.bin --show

Generator for synthetic test log:
  python imu_log_no_crc.py --make-test test_log.bin --seconds 10 --rate_hz 200
"""
from __future__ import annotations

import argparse
import struct
import math
import random
from dataclasses import dataclass
from typing import Iterator, Tuple, List, Optional

import pandas as pd
import matplotlib.pyplot as plt

# Your packed struct is 16 bytes
RECORD_FMT = "<Ihhhhhh"   # little-endian: uint32 + 6x int16
RECORD_SIZE = struct.calcsize(RECORD_FMT)

DEFAULT_ACCEL_RANGE_G = 2     # ±2g
DEFAULT_GYRO_RANGE_DPS = 250  # ±250 dps

ACC_LSB_PER_G = {
    2: 16384.0,
    4: 8192.0,
    8: 4096.0,
    16: 2048.0,
}
GYRO_LSB_PER_DPS = {
    250: 131.0,
    500: 65.5,
    1000: 32.8,
    2000: 16.4,
}

@dataclass
class Record:
    t_ms: int
    ax: int
    ay: int
    az: int
    gx: int
    gy: int
    gz: int

def iter_records(path: str) -> Iterator[Record]:
    with open(path, "rb") as f:
        data = f.read()

    n = len(data) // RECORD_SIZE
    if n == 0:
        return
    
    #cut off incomplete trailing record
    for i in range(n):
        off = i * RECORD_SIZE
        t_ms, ax, ay, az, gx, gy, gz = struct.unpack_from(RECORD_FMT, data, off)
        yield Record(t_ms, ax, ay, az, gx, gy, gz)

def records_to_df(records: List[Record],
                  accel_range_g: int,
                  gyro_range_dps: int) -> pd.DataFrame:
    if accel_range_g not in ACC_LSB_PER_G:
        raise ValueError(f"Unsupported accel_range_g={accel_range_g}. Use one of {sorted(ACC_LSB_PER_G.keys())}")
    if gyro_range_dps not in GYRO_LSB_PER_DPS:
        raise ValueError(f"Unsupported gyro_range_dps={gyro_range_dps}. Use one of {sorted(GYRO_LSB_PER_DPS.keys())}")

    a_scale = 1.0 / ACC_LSB_PER_G[accel_range_g]   # g per LSB
    g_scale = 1.0 / GYRO_LSB_PER_DPS[gyro_range_dps]  # dps per LSB

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

    # Accel
    plt.figure()
    plt.plot(df["t_s"], df["ax_g"], label="ax (g)")
    plt.plot(df["t_s"], df["ay_g"], label="ay (g)")
    plt.plot(df["t_s"], df["az_g"], label="az (g)")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (g)")
    plt.title(title + " - Accel")
    plt.legend()

    # Gyro
    plt.figure()
    plt.plot(df["t_s"], df["gx_dps"], label="gx (dps)")
    plt.plot(df["t_s"], df["gy_dps"], label="gy (dps)")
    plt.plot(df["t_s"], df["gz_dps"], label="gz (dps)")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular rate (deg/s)")
    plt.title(title + " - Gyro")
    plt.legend()

    # Magnitudes
    plt.figure()
    plt.plot(df["t_s"], df["a_mag_g"], label="|a| (g)")
    plt.plot(df["t_s"], df["g_mag_dps"], label="|g| (dps)")
    plt.xlabel("Time (s)")
    plt.ylabel("Magnitude")
    plt.title(title + " - Magnitudes")
    plt.legend()

def make_test_log(path: str, seconds: float, rate_hz: float,
                  accel_range_g: int = DEFAULT_ACCEL_RANGE_G,
                  gyro_range_dps: int = DEFAULT_GYRO_RANGE_DPS) -> None:
    """
    Generates a synthetic log that *resembles* a stationary board with:
      - ~1g on Z accel plus a small sinusoid on X/Y
      - A slow yaw rate sinusoid on gyro Z
    Data are written in the exact packed-binary format your STM32 uses.
    """
    a_lsb_per_g = ACC_LSB_PER_G[accel_range_g]
    g_lsb_per_dps = GYRO_LSB_PER_DPS[gyro_range_dps]

    dt = 1.0 / rate_hz
    n = int(seconds * rate_hz)

    with open(path, "wb") as f:
        for i in range(n):
            t_s = i * dt
            t_ms = int(round(t_s * 1000.0))

            # accel in g
            ax_g = 0.05 * math.sin(2 * math.pi * 1.0 * t_s)
            ay_g = 0.05 * math.cos(2 * math.pi * 1.0 * t_s)
            az_g = 1.0 + 0.02 * math.sin(2 * math.pi * 0.2 * t_s)

            # gyro in dps
            gx_dps = 5.0 * math.sin(2 * math.pi * 0.5 * t_s)
            gy_dps = 3.0 * math.cos(2 * math.pi * 0.5 * t_s)
            gz_dps = 15.0 * math.sin(2 * math.pi * 0.1 * t_s)

            # add small noise
            ax_g += random.uniform(-0.005, 0.005)
            ay_g += random.uniform(-0.005, 0.005)
            az_g += random.uniform(-0.005, 0.005)
            gx_dps += random.uniform(-0.2, 0.2)
            gy_dps += random.uniform(-0.2, 0.2)
            gz_dps += random.uniform(-0.2, 0.2)

            # convert to raw int16
            ax = int(round(ax_g * a_lsb_per_g))
            ay = int(round(ay_g * a_lsb_per_g))
            az = int(round(az_g * a_lsb_per_g))
            gx = int(round(gx_dps * g_lsb_per_dps))
            gy = int(round(gy_dps * g_lsb_per_dps))
            gz = int(round(gz_dps * g_lsb_per_dps))

            # clamp to int16 range
            def clamp_i16(v: int) -> int:
                return max(-32768, min(32767, v))

            rec = struct.pack(RECORD_FMT,
                              t_ms,
                              clamp_i16(ax), clamp_i16(ay), clamp_i16(az),
                              clamp_i16(gx), clamp_i16(gy), clamp_i16(gz))
            f.write(rec)

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", help="Input binary log (e.g., log.bin)")
    ap.add_argument("--csv", help="Optional output CSV path")
    ap.add_argument("--show", action="store_true", help="Show plots")

    ap.add_argument("--accel_range_g", type=int, default=DEFAULT_ACCEL_RANGE_G,
                    help="Accel full-scale range in g (defaults to 2 to match mpu_init)")
    ap.add_argument("--gyro_range_dps", type=int, default=DEFAULT_GYRO_RANGE_DPS,
                    help="Gyro full-scale range in deg/s (defaults to 250 to match mpu_init)")

    ap.add_argument("--make-test", dest="make_test", help="Generate a synthetic test log to this path")
    ap.add_argument("--seconds", type=float, default=10.0, help="Seconds for synthetic test log")
    ap.add_argument("--rate_hz", type=float, default=200.0, help="Sample rate for synthetic test log")

    args = ap.parse_args()

    if args.make_test:
        make_test_log(args.make_test, args.seconds, args.rate_hz,
                      accel_range_g=args.accel_range_g,
                      gyro_range_dps=args.gyro_range_dps)
        print(f"Wrote synthetic test log: {args.make_test}")
        return

    if not args.bin:
        ap.error("Provide --bin log.bin (or use --make-test ...)")

    recs = list(iter_records(args.bin))
    if not recs:
        print("No complete records found (file too small or empty).")
        return

    df = records_to_df(recs, args.accel_range_g, args.gyro_range_dps)

    # Basic sanity printout
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
