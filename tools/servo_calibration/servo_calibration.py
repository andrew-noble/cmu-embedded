#!/usr/bin/env python3
"""Manually calibrate servo pulse widths using the separate servo firmware."""

# Run from the repository root after flashing the servo calibration firmware:
# source .venv/bin/activate
# python -u tools/servo_calibration/servo_calibration.py --port /dev/ttyACM0 --arm
# --arm commands the center position and enables manual servo movement.

import argparse
import json
from pathlib import Path
import select
import sys
import time

import serial
from serial.tools import list_ports


class Console:
    def __init__(self, port):
        self.port = port
        self.pending = bytearray()

    def send(self, command):
        self.port.write((command + "\n").encode("ascii"))
        self.port.flush()

    def read(self):
        self.pending.extend(self.port.readline())
        if b"\n" not in self.pending:
            if len(self.pending) > 512:
                raise RuntimeError("Invalid serial output; check baud rate/firmware")
            return ""
        line, _, rest = self.pending.partition(b"\n")
        self.pending = bytearray(rest)
        return line.decode("ascii", errors="replace").strip()


def check_limits(value, minimum, maximum):
    if not minimum <= value <= maximum:
        raise ValueError(f"Pulse must stay inside {minimum}..{maximum} us")
    return value


def save_marks(marks, output):
    if set(marks) != {"left", "center", "right"}:
        raise ValueError("Mark left, center, and right before saving")
    lo, hi = sorted((marks["left"], marks["right"]))
    if not lo < marks["center"] < hi:
        raise ValueError("Center must lie between distinct left/right pulse widths")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps({"model": "LD-1501MG", "period_us": 20000,
                                  "pulse_us": marks}, indent=2) + "\n")
    print(f"Saved {output}. Copy these measured values into src/config.h:", flush=True)
    for name in ("left", "center", "right"):
        print(f"#define SERVO_{name.upper()}_US {marks[name]}", flush=True)


def calibrate(port, args):
    console = Console(port)
    marks = {}
    pulse = None
    awaiting = None
    try:
        # Terminate any incomplete previous line and release any old session.
        console.send("\nSERVO STOP")
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            line = console.read()
            if line:
                print(f"Board: {line}", flush=True)
            if line == "SERVO_OFF":
                break
        else:
            raise TimeoutError("No SERVO_OFF acknowledgment; flash the servo calibration build")
        console.send("SERVO ARM")
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            line = console.read()
            if line:
                print(f"Board: {line}", flush=True)
            if line.startswith("SERVO_ERROR"):
                raise RuntimeError(line)
            if line.startswith("SERVO_ARMED,"):
                pulse = check_limits(int(line.split(",")[1]), args.min_us, args.max_us)
                break
        else:
            raise TimeoutError("Servo firmware did not arm")

        print("Commands: + / - (small step), us NUMBER, left, center, right, save, q", flush=True)
        print(f"Allowed range: {args.min_us}..{args.max_us} us; step {args.step_us} us.", flush=True)
        next_keep = 0
        last_reply = time.monotonic()
        while True:
            now = time.monotonic()
            if now >= next_keep:
                console.send("SERVO KEEP")
                next_keep = now + 0.2
            line = console.read()
            if line:
                if not line.startswith("SERVO_ALIVE,"):
                    print(f"Board: {line}", flush=True)
                if line.startswith("SERVO_ERROR") or line in ("SERVO_TIMEOUT", "SERVO_OFF"):
                    raise RuntimeError(line)
                if line.startswith(("SERVO_PULSE,", "SERVO_ALIVE,")):
                    actual = int(line.split(",")[1])
                    last_reply = time.monotonic()
                    if awaiting is None or actual == awaiting:
                        pulse = actual
                        awaiting = None
            if time.monotonic() - last_reply > 1.5:
                raise TimeoutError("No servo acknowledgment; stopping")
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            command = sys.stdin.readline()
            if not command or command.strip().lower() in ("q", "quit", "stop"):
                return
            command = command.strip().lower()
            try:
                if awaiting is not None:
                    raise ValueError("Wait for the previous pulse acknowledgment")
                if command in ("+", "-") or command.startswith("us "):
                    requested = (pulse + (args.step_us if command == "+" else -args.step_us)
                                 if command in ("+", "-") else int(command[3:]))
                    awaiting = check_limits(requested, args.min_us, args.max_us)
                    print(f"Requesting {awaiting} us", flush=True)
                    console.send(f"SERVO PULSE {awaiting}")
                elif command in ("left", "center", "right"):
                    marks[command] = pulse
                    print(f"Marked {command}: {pulse} us", flush=True)
                elif command == "save":
                    save_marks(marks, args.output)
                else:
                    print("Use +, -, us NUMBER, left, center, right, save, or q", flush=True)
            except ValueError as exc:
                print(exc, flush=True)
    finally:
        try:
            console.send("SERVO STOP")
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                if console.read() == "SERVO_OFF":
                    print("Board confirmed SERVO_OFF.", flush=True)
                    break
            else:
                print("STOP sent without acknowledgment; firmware timeout is 1 second.", flush=True)
        except (serial.SerialException, OSError):
            print("Serial connection lost; firmware timeout disables pulses.", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--port")
    parser.add_argument("--arm", action="store_true", help="allow centering and manual movement")
    parser.add_argument("--min-us", type=int, default=1400)
    parser.add_argument("--max-us", type=int, default=1600)
    parser.add_argument("--step-us", type=int, default=10)
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().parent / "data" / "endpoints.json")
    args = parser.parse_args()
    if args.list_ports:
        for port in list_ports.comports():
            print(f"{port.device}: {port.description}")
        return 0
    if not args.port or not args.arm:
        parser.error("Supply --port and --arm to permit servo movement")
    if not 500 <= args.min_us < args.max_us <= 2500 or not 1 <= args.step_us <= 25:
        parser.error("Require 500 <= min-us < max-us <= 2500 and step-us 1..25")
    try:
        with serial.Serial(args.port, 115200, timeout=0.02, exclusive=True) as port:
            calibrate(port, args)
    except KeyboardInterrupt:
        print("Calibration interrupted.")
        return 130
    except (serial.SerialException, OSError, ValueError, RuntimeError, TimeoutError) as exc:
        print(f"Calibration stopped: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
