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

- Speed graph: requested RPM (dashed black), left measured RPM, and right
  measured RPM. Encoder counts appear in the summary; the direction-adjusted
  average is retained in CSV recordings.
- Separate left/right commanded drive PWM (braking reports zero drive,
  even though bridge enables are high). Older firmware has no per-wheel PWM.
  The legacy mean-duty field remains in CSV recordings but is not plotted.
- Commanded servo pulse; OFF/UNKNOWN are gaps in the plot, not measured positions.
- Three signed current estimates in amps, plus raw sensor voltages in mV.
  Motor currents share one plot; servo current has its own plot and vertical scale.
- ADC failures shown as INVALID; saturated samples remain numeric.
- System state, motor/brake mode, requested blinker mode, and link/self-test events.
- `CLAMP:` events identify inputs beyond configurable operational limits,
  showing received/applied values and the allowed range. Raw command summaries
  preserve the received values; motor targets reflect the clamped inputs.
  The **Event** summary shows active conditions and becomes blank when they
  clear. Recovery messages clear the corresponding condition without appearing
  in the displayed console; they remain in the CSV for diagnostics. Console
  history retains previous warnings.
- Received steering, throttle, brake, buttons, sequence, and rejected-frame count.
- A full-width event console with larger, wrapped text. Press **R** to toggle
  events/raw messages and **Space** to pause/resume the console while graphs
  and CSV recording continue. Click the figure first to give it keyboard focus.
  Complete raw lines are retained in the CSV in either mode.

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
not electrical feedback or measured servo position. Current uses nominal ACS712-05B
values: `(mV - 2500) / 185` amps, with per-channel settings in `src/config.h`.
There is no divider scaling or extrapolation: at a nominal 3.3 V ADC ceiling,
the estimate plateaus near +4.32 A even if actual current keeps increasing.
Zero offsets still need measurement; all three channels currently assume this
sensor model. Older firmware without `current_mA` displays no amp estimate.
The UART binary packet to the Pi is unchanged and still carries millivolts.
