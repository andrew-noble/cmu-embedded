#!/usr/bin/env python3
"""Step open-loop motor PWM and record wheel RPM from calibration firmware."""

import argparse
import csv
import math
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports


def parse_steps(value):
    try:
        steps = [int(part.strip()) for part in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("steps must be comma-separated integers") from exc
    if not steps or any(step < 0 or step > 100 for step in steps):
        raise argparse.ArgumentTypeError("each duty must be between 0 and 100 percent")
    return steps


def send(port, command):
    port.write((command + "\n").encode("ascii"))
    port.flush()


def read_line(port):
    return port.readline().decode("ascii", errors="replace").strip()


def wait_for_arm(port, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    next_attempt = 0.0
    while time.monotonic() < deadline:
        if time.monotonic() >= next_attempt:
            send(port, "CAL ARM")
            next_attempt = time.monotonic() + 0.5
        line = read_line(port)
        if line.startswith("CAL_ARMED,max_duty="):
            return int(line.split("=", 1)[1])
        if line.startswith("CAL_ERROR,") and line != "CAL_ERROR,not_ready":
            raise RuntimeError(line)
    raise TimeoutError("calibration firmware did not arm; check the build and serial port")


def parse_data(line):
    if not line.startswith("CAL_DATA,"):
        if line.startswith("CAL_ERROR,"):
            raise RuntimeError(line)
        return None
    parts = line.split(",")
    if len(parts) != 7:
        raise RuntimeError(f"malformed calibration record: {line}")
    board_ms, duty, left_rpm, right_rpm, left_counts, right_counts = map(int, parts[1:])
    return {
        "board_ms": board_ms,
        "duty_percent": duty,
        "left_rpm": left_rpm,
        "right_rpm": right_rpm,
        "avg_rpm": (left_rpm + right_rpm) / 2,
        "left_counts": left_counts,
        "right_counts": right_counts,
    }


def collect_step(port, duty, dwell_s, settle_s, step_index, raw_writer, radius_m):
    start = time.monotonic()
    deadline = start + dwell_s
    next_heartbeat = start
    last_data = start
    steady = []
    all_samples = 0

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_heartbeat:
            send(port, f"CAL DUTY {duty}")
            next_heartbeat = now + 0.2  # firmware brakes after 0.5 s without commands

        line = read_line(port)
        sample = parse_data(line)
        if sample is not None:
            received = time.monotonic()
            if sample["duty_percent"] != duty:
                continue  # delayed sample from the previous step
            elapsed = received - start
            is_steady = elapsed >= settle_s
            raw_writer.writerow({
                "step": step_index,
                "requested_duty_percent": duty,
                "elapsed_s": f"{elapsed:.3f}",
                "steady": int(is_steady),
                **sample,
            })
            all_samples += 1
            last_data = received
            if is_steady:
                steady.append(sample)

        if time.monotonic() - last_data > 1.0:
            raise TimeoutError("no CAL_DATA for one second; firmware may have braked")

    if not steady:
        raise RuntimeError(f"no steady samples at duty {duty}; increase --dwell-s")

    mean_left = statistics.mean(row["left_rpm"] for row in steady)
    mean_right = statistics.mean(row["right_rpm"] for row in steady)
    mean_avg = statistics.mean(row["avg_rpm"] for row in steady)
    summary = {
        "step": step_index,
        "duty_percent": duty,
        "samples_total": all_samples,
        "samples_steady": len(steady),
        "left_rpm_mean": f"{mean_left:.2f}",
        "right_rpm_mean": f"{mean_right:.2f}",
        "avg_rpm_mean": f"{mean_avg:.2f}",
        "avg_rpm_stdev": f"{statistics.pstdev(row['avg_rpm'] for row in steady):.2f}",
        "speed_mps": f"{mean_avg * 2 * math.pi * radius_m / 60:.4f}" if radius_m else "",
    }
    print(f"{duty:3d}%: L {mean_left:7.2f} RPM, "
          f"R {mean_right:7.2f} RPM, average {mean_avg:7.2f} RPM")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list-ports", action="store_true", help="show available serial ports")
    parser.add_argument("--port", help="Nucleo ST-LINK USB serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--arm", action="store_true", help="explicitly allow motor movement")
    parser.add_argument("--steps", type=parse_steps,
                        default=parse_steps("0,10,20,30,40,50,60,70,80,90,100"),
                        help="PWM duties in percent (default: 0,10,...,100)")
    parser.add_argument("--dwell-s", type=float, default=3.0, help="seconds at each duty")
    parser.add_argument("--settle-s", type=float, default=1.5,
                        help="ignore this many seconds before averaging")
    parser.add_argument("--radius-m", type=float, help="optional wheel radius for m/s conversion")
    parser.add_argument("--output-dir", type=Path,
                        default=Path(__file__).resolve().parent / "data",
                        help="directory for timestamped CSV files")
    args = parser.parse_args()

    if args.list_ports:
        for port in list_ports.comports():
            print(f"{port.device}: {port.description}")
        return 0
    if not args.port or not args.arm:
        parser.error("supply --port and --arm after lifting both driven wheels")
    if args.dwell_s <= 0 or args.settle_s < 0 or args.settle_s >= args.dwell_s:
        parser.error("require 0 <= --settle-s < --dwell-s")
    if args.radius_m is not None and args.radius_m <= 0:
        parser.error("--radius-m must be positive")

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = args.output_dir / f"motor_pwm_samples_{stamp}.csv"
    summary_path = args.output_dir / f"motor_pwm_summary_{stamp}.csv"
    raw_fields = ["step", "requested_duty_percent", "elapsed_s", "steady", "board_ms",
                  "duty_percent", "left_rpm", "right_rpm", "avg_rpm", "left_counts", "right_counts"]
    summary_fields = ["step", "duty_percent", "samples_total",
                      "samples_steady", "left_rpm_mean", "right_rpm_mean",
                      "avg_rpm_mean", "avg_rpm_stdev", "speed_mps"]

    try:
        with serial.Serial(args.port, baudrate=115200, timeout=0.05) as port, \
             raw_path.open("w", newline="") as raw_file, \
             summary_path.open("w", newline="") as summary_file:
            raw_writer = csv.DictWriter(raw_file, fieldnames=raw_fields)
            summary_writer = csv.DictWriter(summary_file, fieldnames=summary_fields)
            raw_writer.writeheader()
            summary_writer.writeheader()
            port.reset_input_buffer()
            try:
                max_duty = wait_for_arm(port)
                if any(step > max_duty for step in args.steps):
                    raise ValueError(f"firmware caps duty at {max_duty}%; "
                                     "lower --steps or change CALIBRATION_MAX_DUTY_PERCENT "
                                     "in tools/motor_calibration/motor_calibration_config.h and rebuild")
                print(f"Calibration armed; firmware cap is {max_duty}%")
                for index, duty in enumerate(args.steps):
                    summary = collect_step(port, duty, args.dwell_s, args.settle_s,
                                           index, raw_writer, args.radius_m)
                    raw_file.flush()
                    summary_writer.writerow(summary)
                    summary_file.flush()
            finally:
                try:
                    send(port, "CAL STOP")
                except (serial.SerialException, OSError):
                    pass  # The firmware's 500 ms timeout still brakes on disconnect.
    except KeyboardInterrupt:
        print("Calibration interrupted; stop requested", file=sys.stderr)
        return 130
    except (serial.SerialException, OSError, RuntimeError, TimeoutError, ValueError) as exc:
        print(f"Calibration stopped: {exc}", file=sys.stderr)
        return 1

    print(f"Samples: {raw_path}\nSummary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
