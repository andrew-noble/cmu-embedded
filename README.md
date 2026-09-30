# cmu-embedded

CMU 18-449 Distributed Embedded Systems, Lab 2. Target: **NUCLEO-F401RE**
(`nucleo_f401re`). The merged Lab2 application receives framed UART commands
from the Pi and samples two wheel encoders using STM32 timer quadrature mode.

## Build and flash on this machine

Open a dedicated terminal:

```bash
cd /home/au/Desktop/cmu-embedded
source .venv/bin/activate
west build -b nucleo_f401re . -d build/lab2-sang-f401re
```

USB permissions were installed on this machine. On a fresh Linux host, install
the SDK's ST-LINK rule once (enter your Ubuntu administrator password locally):

```bash
sudo install -m 0644 .tools/zephyr-sdk-1.0.1/hosttools/sysroots/x86_64-pokysdk-linux/usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d/60-openocd.rules
sudo udevadm control --reload-rules
```

Connect or reconnect the board through its ST-LINK Mini-B USB connector with a
data cable, then flash:

```bash
west flash -d build/lab2-sang-f401re
```

This Lab2 application does not blink the onboard LED. Its serial output reports
the UART link state, decoded command values, and encoder counts/speeds. The
course's initial Blinky checkpoint should be built separately from the Zephyr
sample before attaching external hardware.

## Provisional motor control

Code map: [`src/main.c`](src/main.c) receives and checks UART commands;
[`src/encoder.c`](src/encoder.c) samples the two hardware encoder counters;
[`src/motor_control.c`](src/motor_control.c) runs the normal speed controller;
[`src/motor_driver.c`](src/motor_driver.c) applies PWM, direction, and braking;
[`src/blinker.c`](src/blinker.c) runs the blinker timer and GPIO outputs.
Their small `.h` files expose only the data and functions shared between
modules. Edit shared timing, pedal ranges, and motor tuning in
[`src/config.h`](src/config.h). Calibration-only settings live in
[`tools/motor_calibration/motor_calibration_config.h`](tools/motor_calibration/motor_calibration_config.h).
Zephyr pin assignments and PWM period
stay in [`boards/nucleo_f401re.overlay`](boards/nucleo_f401re.overlay), and
driver options stay in [`prj.conf`](prj.conf).

The motor code uses D3/D6 for L298N enable PWM and D2/D4/D10/D11 for direction
inputs. The two encoder counters still run in hardware. One control thread wakes
for a valid UART command or a 10 ms encoder sample. Brake takes priority and
puts both L298N channels in dynamic braking (equal inputs, enable held high).
The normal controller maps positive output into 20–100% actual PWM, configured
by `MOTOR_DUTY_OFFSET_PERCENT`: 50% controller output gives 60% PWM and 100%
gives 100%. Zero driving output coasts. Telemetry reports driving PWM; braking
reports zero drive even though the enables are held high. The separate
calibration tool sends direct percentages without this offset.
Throttle selects a target speed up to **200 RPM**, including at full pedal.
PID uses the average of the two wheel speeds throughout the driving range and
may use up to 100% PWM to correct speed under load. Brake, invalid input, link
loss, and motor/sensor faults still override throttle. The estimated unloaded
hardware maximum remains **317 RPM**, based on the recorded wheel-up calibration;
it is not a guaranteed loaded speed. The lower target leaves control headroom.
With a healthy link and brake released, releasing the accelerator disables
both bridge enables to coast and resets PID history. When
PID requests zero drive, the bridge also coasts. Brake-pedal requests,
invalid commands, link loss, and faults continue to apply active braking.
Clutch handling is scaffolded but disabled: the motor starts and stays
in forward direction. Once the Pi sends a real clutch value, a new press past
its threshold can toggle direction only at zero throttle with both wheels stopped.

The Pi's `/home/team5/proxy_receiver/bridge.c` sends a 13-byte UART frame:
start `A5`, ID `01 00`, DLC `08`, steering, throttle, and brake as 16-bit
little-endian fields, byte 10 as a buttons placeholder (currently zero),
sequence byte, and CRC-8. The Pi source documents pedal values as 0–32767,
which this firmware now accepts. A reported released value of -32767 conflicts
with that comment; such a reading is rejected until live Pi output resolves the
range. Steering drives the LD-1501MG on D15/PB8 (TIM4_CH3), at 50 Hz:
`-32767 -> 1000 us`, `0 -> 1500 us`, `32766 -> 2000 us`.
The inclusive -500..500 dead zone holds 1500 us; the remaining input range
is rescaled continuously to the endpoints (`STEER_DEAD_ZONE` in `src/config.h`).
Each valid Pi command updates the pulse through the existing command thread.
PWM starts disabled and is disabled on link loss or an invalid command.
Swap `SERVO_LEFT_US` and `SERVO_RIGHT_US` in `src/config.h` if steering is reversed.
Byte 10 is ignored for clutch until the Pi sender is extended.

Edit [the motor settings](src/config.h) after checking actual pedal
ranges, rest/pressed directions, motor polarity, counts per wheel revolution,
and safe maximum RPM. The throttle endpoints now match the Pi source's stated
0–32767 range. The measured brake range is 30454 released to 32767 fully
pressed, with braking active at 31610 or above. Brake input validation accepts
0–32767 because the released reading can fall below 30454; that measured rest
value is not a rejection boundary. Check the chosen midpoint threshold during
pedal testing; max RPM and PID gains
remain starting points to verify. `CLUTCH_CONTROL_ENABLED` remains `0` for forward-only mode.
`MOTOR_OUTPUTS_ENABLED` is `1`, and PWM duty may reach 100%; this is a software
output range, not a current limit. A valid throttle command above the 1% deadband can drive the
motors when brake is below its provisional threshold. Confirm pedal values and
motor direction before testing on the ground. The encoder counts-per-revolution setting is in
`boards/nucleo_f401re.overlay`; PWM pin mapping and 1 kHz period are there too.
Wheel-up motor response and manual encoder counts were measured; see
[the recorded measurements](tools/motor_calibration/MEASUREMENTS.md).
Loaded behavior and the 2 ms timing requirement remain unverified.
The link watchdog expires 95 ms after the last valid frame's reception, leaving
5 ms for output response before the stricter 100 ms unplug checkoff. Part 2 of
the handout instead says 150 ms; this firmware follows the checkoff. Expired
queued frames are rejected, and the command handler starts without a 10-second
delay. Command and fail-safe outputs are serialized so they cannot overwrite
one another midway through an update. Hardware response timing still needs measurement.

The blinker module uses a periodic Zephyr timer and a lower-priority thread,
with no delay in the UART or motor path. It supports 1 Hz left/right turns and
2 Hz hazards, both at 50% duty. Left LEDs use PC10/PC11; right LEDs use PC12/PD2.
Pi byte 10 uses bits 0/1/2 for self-test/left/right button states; the command
handler debounces all three buttons: presses act immediately; another press
requires release observed across valid samples for at least 30 ms. At 50 ms
packet intervals, allow two released samples between clicks. Self-test enters
on one press and exits on two additional presses within 500 ms; bounce and
held buttons do not count as additional clicks. A left/right press toggles that side, or switches
from the other side. Automatic cancellation arms after steering in the selected
direction beyond neutral by `BLINKER_TURN_MARGIN` (500 counts): left at -1000,
right at +1000. It cancels on return to the ±500 neutral zone, including crossing
past it between frames. Selecting a signal while centered does not cancel it.
Hazards override turn cancellation; changing modes clears the armed state.
See [TODO.md](TODO.md) for the missing hardware and Pi information.
The [servo calibration tool](tools/servo_calibration/servo_calibration.py)
uses a separate firmware build with drive motors disabled. Build it with
`-DSERVO_CALIBRATION=ON -DMOTOR_CALIBRATION=OFF` in a separate build directory.
The manual tool defaults to 1400–1600 us; pass `--min-us 500 --max-us 2500`
to allow the full pulse range. A 500–2500 us sweep has been commanded and
acknowledged; physical steering direction and linkage clearance still need
confirmation. The normal firmware now trials 1000–2000 us because steering
reached its physical limit at about half the steering-wheel travel.
Console output is buffered and drained by Zephyr’s logging thread at priority 5,
below the control threads; a full log buffer drops messages instead of blocking control.

## Measure PWM versus wheel speed

The normal firmware uses PID through full throttle (200 RPM target), so its
PWM duty changes as it corrects speed.
For an open-loop duty-versus-speed measurement, build the separate calibration
firmware. Its C sources and laptop script live together in
[`tools/motor_calibration/`](tools/motor_calibration/). CMake selects those test
sources only when `MOTOR_CALIBRATION=ON`; the normal motor controller contains
no calibration branches. The test firmware starts braked, uses forward direction
only, and accepts exact
`CAL ARM`, `CAL DUTY n`, and `CAL STOP` commands over the Nucleo ST-LINK USB
serial console (USART2, 115200 baud). It brakes if commands stop for 500 ms.
Duty commands use whole percentages from 0 to 100. Calibration now permits the
full range, as does the normal motor controller.

With both driven wheels raised, build and flash the calibration firmware:

```bash
source .venv/bin/activate
west build -b nucleo_f401re . -d build/motor-calibration-f401re -- -DMOTOR_CALIBRATION=ON
west flash -d build/motor-calibration-f401re -r stm32cubeprogrammer
python tools/motor_calibration/motor_calibration.py --list-ports
python tools/motor_calibration/motor_calibration.py --port /dev/ttyACM0 --arm
```

Use the actual port shown by `--list-ports`. The Python script sends fixed
0–100% duty steps in 10% increments, records both encoder positions and RPM every 100 ms, and
writes timestamped sample and summary CSV files under
`tools/motor_calibration/data/` (ignored by Git). It averages the
last 1.5 seconds of each three-second step. No wheel radius is needed to collect
RPM. Once the radius is known, pass `--radius-m 0.05` (replace `0.05` with the
measured radius) to add speed in m/s to the summary. Verify the encoder's
1316 counts-per-wheel-revolution setting against additional manual turns if
greater precision is needed; it is rounded from the measured average of 1316.2.
To use smaller steps, pass a list such as `--steps 0,5,10,15,20`. The current
sensors are not yet read by firmware, so this sweep records RPM only. After the
experiment, flash the normal build from
`build/lab2-sang-f401re` to restore Pi control.

The course's *Flashing and Debugging Your Nucleo* guide lists
**STM32CubeProgrammer as required** and notes that OpenOCD is included with
the Zephyr SDK on Linux. STM32CubeProgrammer **2.23.0 is installed** at
`.tools/STM32CubeProgrammer/`. This project's virtual environment activation makes its CLI available
in `PATH`. Verify it with `STM32_Programmer_CLI --version`. The F401RE default
runner is set in `CMakeLists.txt`. To name it explicitly for one flash:

```bash
west flash -d build/lab2-sang-f401re -r stm32cubeprogrammer
```

The guide's `source .venv/bin/active` line contains a typo. Use
`source .venv/bin/activate` here; this local activation also sets `ZEPHYR_BASE`, so a separate
`zephyr-env.sh` command is unnecessary. Do not replace the board's ST-LINK
firmware with J-Link firmware; the course guide explicitly prohibits that.

Subsequent builds can use `west build -d build/lab2-sang-f401re`. Activate the
environment in each new terminal. Run `deactivate` when finished; this local
activation restores the previous shell settings.

## Separate course environment

Everything installed specifically for this project is inside this directory:

| Path | Purpose |
| --- | --- |
| `.venv/` | West and Zephyr's Python build dependencies |
| `.tools/python/` | Independent Python 3.12.14 runtime |
| `.tools/zephyrproject/` | Independent West workspace: Zephyr and STM32 modules |
| `.tools/zephyr-sdk-1.0.1/` | ARM compiler and host tools, including OpenOCD |
| `.tools/STM32CubeProgrammer/` | ST's programmer and CLI (version 2.23.0) |
| `.tools/cache/` | Project build caches |
| `build/lab2-sang-f401re/` | Lab2 firmware and generated build files |

These local directories are ignored by Git. The tracked `zephyr-env.sh` records
the project paths; this machine's `.venv/bin/activate` sources it.
It restores the previous shell environment on `deactivate`. The setup does not depend on
`embedded-lab` or `advance-mechatronics`. Existing local source/toolchain copies
were used to seed independent copies; the SDK host tools were installed afresh
at their new path and the Python virtual environment was created afresh.
System utilities and Linux USB rules are machine-wide. No shell startup files
or global CMake package registrations were changed.

Lab 2 points to the standard [Zephyr getting-started guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)
and does not specify a release. This installation uses:

- Zephyr **v4.4.2**, commit `dccb09599635bdff17633fa7e9dab014b91dce90`.
- Zephyr SDK **1.0.1**, ARM GNU toolchain only, plus host tools.
- Python **3.12.14**, West **1.5.0**, CMake **3.31.10**, Ninja **1.13.2**.
- CMSIS, CMSIS 6, and STM32 HAL revisions pinned by Zephyr's `west.yml`.
- Python package versions recorded in `requirements-lock.txt`.

Only the modules needed for this STM32 application are populated. If later lab
features require more modules, update those explicitly in this workspace.

A fresh clone does not include the ignored environment. To recreate it, use
Python 3.12 to create `.venv`, install `requirements-lock.txt`, initialize
`.tools/zephyrproject` from the official Zephyr repository at `v4.4.2`, and run
`west update cmsis cmsis_6 hal_stm32` there. Install an independent SDK 1.0.1
(ARM toolchain and host tools) at `.tools/zephyr-sdk-1.0.1`. Obtain the
STM32CubeProgrammer Linux installer from
[ST's official download page](https://www.st.com/en/development-tools/stm32cubeprog.html)
and install it at `.tools/STM32CubeProgrammer/`. On a fresh clone, source
`zephyr-env.sh` after activating the new virtual environment, or configure
that virtual environment to source it automatically. The virtual environment
itself is not tracked by Git. No `west zephyr-export` is needed on this machine
because `zephyr-env.sh` sets `ZEPHYR_BASE` explicitly.

## Verification and remaining hardware step

Verified on this machine: complete Lab2 F401RE firmware build,
`python -m pip check`, and SDK OpenOCD startup. Firmware:
`build/lab2-sang-f401re/zephyr/zephyr.elf`.

USB rules are installed, and the Nucleo's ST-LINK/V2.1 was detected. The
STM32CubeProgrammer CLI version command passed. This build now selects
STM32CubeProgrammer as its default flash runner. Firmware has not been flashed
or checked on the physical LED. Do not run West with sudo; use USB rules.

## USB status monitor

After flashing the normal firmware, close other serial tools and run:

```bash
source .venv/bin/activate
python -m serial.tools.miniterm /dev/ttyACM0 115200
```

In addition to the existing motor and received-command lines, a `STATUS` line
prints every `USB_STATUS_PERIOD_MS` (500 ms) even when Pi commands stop.
It shows system state, three sensor voltages (`current_mV L/R/S`), commanded
servo pulse, motor output mode, and requested blinker mode. Servo `OFF` means
pulses disabled; `UNKNOWN/FAULT` means no confirmed pulse setting. ADC `INVALID`
means a read/setup failure. Motor modes distinguish `BRAKE`, `COAST`, and `DRIVE`,
with `UNKNOWN`/`FAULT` for unavailable/failed output state.

Sensor voltages are not calibrated amperes. Output modes and servo pulse are
software state, not measured electrical outputs or actual servo position.
Snapshots from separate components can briefly differ during transitions.
The existing Pi binary status packet remains unchanged, as does the `ENC` line
consumed by `tools/telemetry/telemetry.py`. Exit miniterm with Ctrl+].

For graphs and all status in one window, use the [telemetry dashboard](tools/telemetry/README.md)
instead of miniterm:

```bash
source .venv/bin/activate
python -u tools/telemetry/telemetry.py --port /dev/ttyACM0
```

This replaces the former `tools/motor_plot/` tool. Calibration tools remain
separate because they actively command their dedicated calibration firmware.
