#!/usr/bin/env python3
"""One read-only USB dashboard for normal-firmware telemetry and console output."""

# Run from the repository root:
# source .venv/bin/activate
# python -u tools/telemetry/telemetry.py --port /dev/ttyACM0

import argparse
import csv
from collections import deque
from datetime import datetime
from pathlib import Path
import re
import time

import serial

ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
MOTOR = re.compile(r"ENC L\s+(-?\d+)\s+(-?\d+) rpm\s*\| R\s+(-?\d+)\s+(-?\d+) rpm"
                   r"\s*\| target\s+(-?\d+) duty\s+(\d+)% (FWD|REV)")
COMMAND = re.compile(r"steer\s+(-?\d+)\s+thr\s+(-?\d+)\s+brk\s+(-?\d+)"
                     r"\s+buttons\s+(\d+)\s+seq\s+(\d+)\s+bad\s+(\d+)")
STATUS = re.compile(r"STATUS state=(\S+) \| current_mV L=(\S+) R=(\S+) S=(\S+)"
                    r" \| servo=(\S+) \| motor=(\S+) \| blinker=(\S+)")
FIELDS = ["host_time_s", "kind", "target_rpm", "left_rpm", "right_rpm", "average_rpm",
          "duty_percent", "direction", "left_counts", "right_counts", "steer_raw",
          "throttle_raw", "brake_raw", "buttons", "seq", "bad_frames", "state",
          "current_left_mv", "current_right_mv", "current_servo_mv", "servo",
          "servo_us", "motor_mode", "blinker", "event", "raw"]


def parse_line(line):
    """Return only values present in this line; missing readings stay missing."""
    line = ANSI.sub("", line).strip()
    row = dict(kind="console", raw=line)
    if match := MOTOR.search(line):
        lc, left, rc, right, target, duty = map(int, match.groups()[:6])
        direction = match.group(7)
        row.update(kind="motor", target_rpm=target, left_rpm=left, right_rpm=right,
                   average_rpm=(left + right) / 2 * (-1 if direction == "REV" else 1),
                   duty_percent=duty, direction=direction, left_counts=lc, right_counts=rc)
    elif match := COMMAND.search(line):
        row.update(kind="command", **dict(zip(
            ["steer_raw", "throttle_raw", "brake_raw", "buttons", "seq", "bad_frames"],
            map(int, match.groups()))))
    elif match := STATUS.search(line):
        state, left, right, servo_mv, pulse, motor, blinker = match.groups()
        try:
            currents = [None if v == "INVALID" else int(v) for v in (left, right, servo_mv)]
            # OFF/UNKNOWN are not measured positions and are not plotted as zero.
            pulse_us = int(pulse[:-2]) if pulse.endswith("us") else None
        except ValueError:
            return row
        row.update(kind="status", state=state, current_left_mv=currents[0],
                   current_right_mv=currents[1], current_servo_mv=currents[2], servo=pulse,
                   servo_us=pulse_us, motor_mode=motor, blinker=blinker)
    elif "Booting Zephyr" in line:
        row.update(kind="boot", event="BOARD RESTARTED")
    elif line.startswith(("LINK UP", "LINK LOST", "SELF-TEST", "ERROR:")):
        row.update(kind="event", event=line)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--window", type=float, default=30, help="visible history in seconds")
    parser.add_argument("--output", type=Path, help="CSV path; default: data/<timestamp>.csv")
    args = parser.parse_args()
    if not 0 < args.window < float("inf"):
        parser.error("--window must be a finite positive number")
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError:
        parser.error("Run: python -m pip install -r tools/telemetry/requirements.txt")

    output = args.output or Path(__file__).parent / "data" / (
        datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".csv")
    fig = plt.figure(figsize=(15, 10), layout="constrained")
    grid = fig.add_gridspec(4, 2, width_ratios=[1.25, 1])
    axes = [fig.add_subplot(grid[i, 0]) for i in range(4)]
    summary_ax = fig.add_subplot(grid[:2, 1])
    console_ax = fig.add_subplot(grid[2:, 1])
    for axis in (summary_ax, console_ax):
        axis.axis("off")
    summary = summary_ax.text(0, 1, "Waiting for telemetry…", va="top", fontsize=11,
                              family="monospace", transform=summary_ax.transAxes)
    console = console_ax.text(0, 1, "", va="top", fontsize=8, family="monospace",
                              transform=console_ax.transAxes, clip_on=True)
    definitions = [
        ("motor", [("target_rpm", "Target"), ("left_rpm", "Left"), ("right_rpm", "Right"),
                   ("average_rpm", "Average (direction-adjusted)")], "Speed (RPM)"),
        ("motor", [("duty_percent", "Drive PWM")], "Duty (%)"),
        ("status", [("servo_us", "Commanded servo pulse")], "Pulse (µs)"),
        ("status", [("current_left_mv", "Left motor"), ("current_right_mv", "Right motor"),
                    ("current_servo_mv", "Servo")], "Sensor voltage (mV)"),
    ]
    curves = []
    for axis, (_, fields, label) in zip(axes, definitions):
        curves.append([axis.plot([], [], label=name)[0] for _, name in fields])
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper left", fontsize=8)
    axes[1].set_ylim(-5, 105)
    axes[-1].set_xlabel("Host receive time (s)")
    title = fig.suptitle("STM32 telemetry — waiting for serial data")
    history = {key: deque(maxlen=100000) for key in ("motor", "status")}
    latest = {}
    recent = deque(maxlen=14)
    pending = bytearray()
    started = time.monotonic()
    last_received = None
    count = 0
    animation = None

    def section(kind, now):
        row = latest.get(kind)
        if row is None:
            return f"{kind.upper()}: not received"
        age = now - row["host_time_s"]
        return f"{kind.upper()}: age {age:.1f}s" + ("  STALE" if age > 2 else "")

    try:
        output.parent.mkdir(parents=True, exist_ok=True)
        with serial.Serial(args.port, args.baud, timeout=0, exclusive=True) as port, \
                output.open("x", newline="") as file:
            port.reset_input_buffer()
            writer = csv.DictWriter(file, fieldnames=FIELDS)
            writer.writeheader()
            print(f"Reading {args.port}; CSV: {output.resolve()}", flush=True)
            print("Close window or Ctrl+C to finish. This tool does not command or stop motors.", flush=True)

            def update(_):
                nonlocal last_received, count
                now = time.monotonic() - started
                try:
                    pending.extend(port.read(min(port.in_waiting, 65536)))
                except (serial.SerialException, OSError) as exc:
                    title.set_text("DISCONNECTED — displayed values are stale; CSV saved")
                    print(f"Serial disconnected: {exc}", flush=True)
                    animation.event_source.stop()
                    file.flush()
                    return
                while b"\n" in pending:
                    line, _, rest = pending.partition(b"\n")
                    pending[:] = rest
                    row = parse_line(line.decode("ascii", errors="replace"))
                    if not row["raw"]:
                        continue
                    row["host_time_s"] = now
                    writer.writerow(row)
                    count += 1
                    last_received = now
                    recent.append(f"{now:7.1f} {row['raw']}")
                    if row["kind"] == "boot":
                        latest.clear()
                        for values in history.values():
                            values.clear()
                    latest[row["kind"]] = row
                    if row["kind"] in history:
                        history[row["kind"]].append(row)
                if len(pending) > 65536:
                    pending.clear()
                    recent.append("Oversized incomplete line discarded")
                file.flush()
                for values in history.values():
                    while values and values[0]["host_time_s"] < now - args.window:
                        values.popleft()
                for axis, lines, (kind, fields, _) in zip(axes, curves, definitions):
                    values = history[kind]
                    times = [r["host_time_s"] for r in values]
                    for curve, (field, _) in zip(lines, fields):
                        curve.set_data(times, [r[field] if r[field] is not None else float("nan")
                                               for r in values])
                    if axis is not axes[1]:
                        axis.relim()
                        axis.autoscale_view(scalex=False)
                    axis.set_xlim(max(0, now - args.window), max(args.window, now))
                lines = [section("status", now)]
                if r := latest.get("status"):
                    def voltage(value):
                        return "INVALID" if value is None else str(value)
                    lines += [f"System: {r['state']}   Motor: {r['motor_mode']}",
                              f"Blinker: {r['blinker']}   Servo: {r['servo']}",
                              "Current sensors (mV, not amps):",
                              f"  L={voltage(r['current_left_mv'])} R={voltage(r['current_right_mv'])} S={voltage(r['current_servo_mv'])}"]
                lines += ["", section("motor", now)]
                if r := latest.get("motor"):
                    lines += [f"Target {r['target_rpm']} RPM   Duty {r['duty_percent']}% {r['direction']}",
                              f"RPM L={r['left_rpm']} R={r['right_rpm']} avg={r['average_rpm']:.1f}",
                              f"Counts L={r['left_counts']} R={r['right_counts']}"]
                lines += ["", section("command", now)]
                if r := latest.get("command"):
                    lines += [f"Steer={r['steer_raw']} Throttle={r['throttle_raw']}",
                              f"Brake={r['brake_raw']} Buttons={r['buttons']} (0x{r['buttons']:02x})",
                              f"Sequence={r['seq']} Rejected={r['bad_frames']}"]
                if r := latest.get("event"):
                    lines += ["", f"Last event ({now-r['host_time_s']:.1f}s ago):", r["event"]]
                summary.set_text("\n".join(lines))
                console.set_text("RAW CONSOLE (full lines saved in CSV)\n\n" + "\n".join(recent))
                age = "no data" if last_received is None else f"last line {now-last_received:.1f}s ago"
                title.set_text(f"STM32 telemetry — {count} lines — {age}")

            animation = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
            plt.show()
    except KeyboardInterrupt:
        pass
    except (serial.SerialException, OSError) as exc:
        print(f"Dashboard stopped: {exc}")
        return 1
    finally:
        if animation is not None and animation.event_source is not None:
            animation.event_source.stop()
        plt.close(fig)
    print(f"Saved {count} lines to {output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
