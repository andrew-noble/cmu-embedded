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
import signal
import textwrap
import time

import serial

ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
MOTOR = re.compile(r"ENC L\s+(-?\d+)\s+(-?\d+) rpm\s*\| R\s+(-?\d+)\s+(-?\d+) rpm"
                   r"\s*\| target\s+(-?\d+) duty\s+(\d+)% (FWD|REV)")
COMMAND = re.compile(r"steer\s+(-?\d+)\s+thr\s+(-?\d+)\s+brk\s+(-?\d+)"
                     r"\s+buttons\s+(\d+)\s+seq\s+(\d+)\s+bad\s+(\d+)")
STATUS = re.compile(r"STATUS state=(\S+) \| current_mV L=(\S+) R=(\S+) S=(\S+)"
                    r" \| servo=(\S+) \| motor=(\S+) \| blinker=(\S+)")
CURRENT_MA = re.compile(r" \| current_mA L=(\S+) R=(\S+) S=(\S+)")
WHEEL_PWM = re.compile(r" \| pwm L=(\d+) R=(\d+)")
FIELDS = ["host_time_s", "kind", "target_rpm", "left_rpm", "right_rpm", "average_rpm",
          "duty_percent", "direction", "left_counts", "right_counts", "steer_raw",
          "throttle_raw", "brake_raw", "buttons", "seq", "bad_frames", "state",
          "current_left_mv", "current_right_mv", "current_servo_mv", "servo",
          "servo_us", "motor_mode", "blinker", "current_left_a", "current_right_a",
          "current_servo_a", "left_duty_percent", "right_duty_percent", "event", "raw"]


def parse_line(line):
    """Return only values present in this line; missing readings stay missing."""
    line = ANSI.sub("", line).strip()
    row = dict(kind="console", raw=line)
    if match := MOTOR.search(line):
        lc, left, rc, right, target, duty = map(int, match.groups()[:6])
        direction = match.group(7)
        pwm = WHEEL_PWM.search(line)
        row.update(left_duty_percent=int(pwm[1]) if pwm else None,
                   right_duty_percent=int(pwm[2]) if pwm else None)
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
        amps = [None, None, None]
        if readings := CURRENT_MA.search(line):
            try:
                amps = [None if v == "INVALID" else int(v) / 1000 for v in readings.groups()]
            except ValueError:
                return row
        row.update(current_left_a=amps[0], current_right_a=amps[1], current_servo_a=amps[2])
        row.update(kind="status", state=state, current_left_mv=currents[0],
                   current_right_mv=currents[1], current_servo_mv=currents[2], servo=pulse,
                   servo_us=pulse_us, motor_mode=motor, blinker=blinker)
    elif "Booting Zephyr" in line:
        row.update(kind="boot", event="BOARD RESTARTED")
    elif line.startswith(("LINK UP", "LINK LOST", "SELF-TEST", "ERROR:", "CLAMP:")):
        row.update(kind="event", event=line)
    return row


def update_events(active, row):
    """Keep current conditions; recovery messages clear without being displayed."""
    message = row.get("event", "")
    if message.startswith("CLAMP:"):
        key = "clamp_" + message.split()[1]
        if "clamp cleared" in message:
            active.pop(key, None)
            return False
        active[key] = row
    elif message.startswith("LINK UP"):
        active.pop("link", None)
        active.pop("error", None)
        return False
    elif message.startswith("SELF-TEST exit"):
        active.pop("self_test", None)
        return False
    elif message.startswith("LINK LOST"):
        active["link"] = row
    elif message.startswith("SELF-TEST"):
        active["self_test"] = row
    elif message.startswith("ERROR:"):
        active["error"] = row
    return True


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
    fig = plt.figure(figsize=(15, 12), layout="constrained")
    grid = fig.add_gridspec(6, 2, width_ratios=[1.25, 1], height_ratios=[1, 1, 1, 1, 1, 1.6])
    axes = [fig.add_subplot(grid[i, 0]) for i in range(5)]
    summary_ax = fig.add_subplot(grid[:5, 1])
    console_ax = fig.add_subplot(grid[5, :])
    for axis in (summary_ax, console_ax):
        axis.axis("off")
    summary = summary_ax.text(0, 1, "Waiting for telemetry…", va="top", fontsize=11,
                              family="monospace", transform=summary_ax.transAxes)
    console = console_ax.text(0, 1, "", va="top", fontsize=11, family="monospace",
                              transform=console_ax.transAxes, clip_on=True)
    definitions = [
        ("motor", [("target_rpm", "Requested RPM"),
                   ("left_rpm", "Left measured RPM"),
                   ("right_rpm", "Right measured RPM")], "Speed (RPM)"),
        ("motor", [("left_duty_percent", "Left commanded PWM"),
                   ("right_duty_percent", "Right commanded PWM")], "Commanded PWM (%)"),
        ("status", [("servo_us", "Commanded servo pulse")], "Pulse (µs)"),
        ("status", [("current_left_a", "Left motor"), ("current_right_a", "Right motor")],
         "Motor current (A)"),
        ("status", [("current_servo_a", "Servo")], "Servo current (A)"),
    ]
    curves = []
    for axis, (_, fields, label) in zip(axes, definitions):
        curves.append([axis.plot([], [], label=name)[0] for _, name in fields])
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper left", fontsize=8)
    axes[1].set_ylim(-5, 105)
    curves[0][0].set_color("black")
    curves[0][0].set_linestyle("--")
    curves[0][0].set_linewidth(2)
    axes[0].legend(loc="upper left", fontsize=8)
    pwm_readout = axes[1].text(
        0.98, 0.96, "PWM command: waiting", ha="right", va="top",
        fontsize=11, family="monospace", transform=axes[1].transAxes,
        bbox=dict(facecolor="white", edgecolor="0.8", alpha=0.9))
    axes[-1].set_xlabel("Host receive time (s)")
    title = fig.suptitle("STM32 telemetry — waiting for serial data")
    history = {key: deque(maxlen=100000) for key in ("motor", "status")}
    latest = {}
    active_events = {}
    recent = deque(maxlen=200)
    events = deque(maxlen=200)
    show_raw = False
    console_paused = False

    def console_key(event):
        nonlocal show_raw, console_paused
        if event.key == "r":
            show_raw = not show_raw
        elif event.key == " ":
            console_paused = not console_paused

    fig.canvas.mpl_connect("key_press_event", console_key)
    pending = bytearray()
    started = time.monotonic()
    last_received = None
    count = 0
    animation = None
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    def check_stop():
        if stop_requested:
            plt.close(fig)

    # Let GUI callbacks finish instead of raising KeyboardInterrupt mid-draw.
    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    shutdown_timer = fig.canvas.new_timer(interval=100)
    shutdown_timer.add_callback(check_stop)

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
                    visible = row["kind"] != "event" or update_events(active_events, row)
                    if visible:
                        recent.append(f"{now:7.1f} {row['raw']}")
                    if visible and row["kind"] in ("event", "boot", "console"):
                        events.append(f"{now:7.1f} {row['raw']}")
                    if row["kind"] == "boot":
                        latest.clear()
                        active_events.clear()
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
                    lines[0] = f"STATUS: {r['state']}  |  " + section("status", now).split(": ", 1)[1]
                    lines += [f"Motor: {r['motor_mode']}   Blinker: {r['blinker']}",
                              f"Servo command: {r['servo']}", "",
                              "SENSOR OUTPUTS",
                              f"{'':12}{'Left':>9}{'Right':>9}{'Servo':>9}",
                              f"{'Voltage mV':12}{voltage(r['current_left_mv']):>9}{voltage(r['current_right_mv']):>9}{voltage(r['current_servo_mv']):>9}"]
                    def amps(value):
                        return "N/A" if value is None else f"{value:.3f}"
                    lines += [f"{'Estimate A':12}{amps(r['current_left_a']):>9}{amps(r['current_right_a']):>9}{amps(r['current_servo_a']):>9}"]
                lines += ["", section("motor", now)]
                if r := latest.get("motor"):
                    lines += [f"Requested: {r['target_rpm']} RPM   Direction: {r['direction']}",
                              f"{'':12}{'Left':>9}{'Right':>9}",
                              f"{'RPM':12}{r['left_rpm']:>9}{r['right_rpm']:>9}",
                              f"{'Counts':12}{r['left_counts']:>9}{r['right_counts']:>9}"]
                    def pwm_text(value):
                        return "N/A" if value is None else f"{value}%"
                    lines += [f"{'PWM command':12}{pwm_text(r['left_duty_percent']):>9}{pwm_text(r['right_duty_percent']):>9}"]
                    stale = "  STALE" if now - r["host_time_s"] > 2 else ""
                    pwm_readout.set_text(f"PWM command{stale}\n"
                                         f"L: {pwm_text(r['left_duty_percent'])}   R: {pwm_text(r['right_duty_percent'])}")
                else:
                    pwm_readout.set_text("PWM command: waiting")
                lines += ["", section("command", now)]
                if r := latest.get("command"):
                    lines += [f"Steering: {r['steer_raw']:>6}   Throttle: {r['throttle_raw']:>6}",
                              f"Brake:    {r['brake_raw']:>6}   Buttons: 0x{r['buttons']:02x}",
                              f"Sequence: {r['seq']:>6}   Rejected: {r['bad_frames']}"]
                lines += ["", "Event:"]
                width = max(20, int(summary_ax.bbox.width / (11 * fig.dpi / 72 * 0.62)))
                for r in active_events.values():
                    lines += textwrap.wrap(r["event"], width=width)
                summary.set_text("\n".join(lines))
                if not console_paused:
                    # Fit wrapped lines to the actual panel dimensions after resizing.
                    char_px = 11 * fig.dpi / 72 * 0.62
                    width = max(20, int(console_ax.bbox.width / char_px))
                    capacity = max(1, int(console_ax.bbox.height / (11 * fig.dpi / 72 * 1.3)) - 2)
                    wrapped = [part for entry in (recent if show_raw else events)
                               for part in textwrap.wrap(entry, width=width, subsequent_indent="        ")]
                    mode = "RAW" if show_raw else "EVENTS"
                    console.set_text(f"{mode} — R: toggle raw/events | Space: pause/resume console | CSV saves all lines\n\n"
                                     + "\n".join(wrapped[-capacity:]))
                age = "no data" if last_received is None else f"last line {now-last_received:.1f}s ago"
                title.set_text(f"STM32 telemetry — {count} lines — {age}")

            animation = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
            shutdown_timer.start()
            plt.show()
    except KeyboardInterrupt:
        pass
    except (serial.SerialException, OSError) as exc:
        print(f"Dashboard stopped: {exc}")
        return 1
    finally:
        shutdown_timer.stop()
        if animation is not None and animation.event_source is not None:
            animation.event_source.stop()
        plt.close(fig)
        signal.signal(signal.SIGINT, previous_sigint)
    print(f"Saved {count} lines to {output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
