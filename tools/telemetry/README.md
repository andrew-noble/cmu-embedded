# STM32 telemetry dashboard

One USB serial connection replaces the separate motor plot and miniterm view:

```bash
source .venv/bin/activate
python -m pip install -r tools/telemetry/requirements.txt
python -u tools/telemetry/telemetry.py --port /dev/ttyACM0
```

Close other serial monitors first. A desktop session with a Matplotlib GUI
backend is required. The tool reads only; closing it does not stop the motors.
Calibration tools remain separate because they command hardware using separate firmware.

The dashboard displays:

- Target RPM, both signed wheel speeds, direction-adjusted average, encoder counts.
- Drive PWM (braking reports zero drive, even though bridge enables are high).
- Commanded servo pulse; OFF/UNKNOWN are gaps in the plot, not measured positions.
- Three current-sensor voltages in mV, with INVALID readings shown explicitly.
- System state, motor/brake mode, requested blinker mode, and link/self-test events.
- Received steering, throttle, brake, buttons, sequence, and rejected-frame count.
- Recent raw console messages. Complete lines are retained in the CSV.

Each category has its own age indicator; values older than two seconds are marked
STALE. No received STATUS line is shown as "not received", not as zero. Reset
messages clear previous snapshots and plots. Link events are historical messages;
their age is shown, and absence of an event is not proof of link health.

CSV recordings go to `data/`, one row per received line. Columns absent from a
line stay blank (including invalid readings); the raw line preserves details.
Use `--window 60` for a longer display or `--output /tmp/run.csv` for a chosen file.
Existing files are not overwritten. Host receive timestamps include serial and
logging delays and cannot prove assignment response-time deadlines.

New firmware must be flashed to see the 500 ms STATUS summaries and the 200 RPM
target. Older firmware still displays available motor/command messages, with
missing status clearly labeled. Firmware output states are software commands,
not electrical feedback or measured servo position. Current scaling to amps
still needs sensor calibration. The UART binary packet to the Pi is unchanged.
