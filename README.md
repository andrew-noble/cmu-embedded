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
[`src/motor.c`](src/motor.c) runs PWM, braking, clutch direction changes, and
PID. Their small `.h` files expose only the data and functions shared between
modules. Change pedal ranges and tuning in [`src/motor_config.h`](src/motor_config.h).

The motor code uses D3/D6 for L298N enable PWM and D2/D4/D12/D11 for direction
inputs. The two encoder counters still run in hardware. One control thread wakes
for a valid UART command or a 10 ms encoder sample. Brake takes priority and
puts both L298N channels in dynamic braking (equal inputs, enable held high).
Throttle selects a target RPM; a PID loop uses the average of the two wheel
speeds. Clutch is treated as an analog pedal: a new press past its threshold
toggles forward/reverse only when throttle is zero and both wheels are stopped.

The existing 13-byte UART frame is provisionally interpreted as: start `A5`,
ID `01 00`, DLC `08`, steering signed 16-bit little-endian, throttle unsigned
16-bit little-endian, brake unsigned 16-bit little-endian, clutch unsigned
8-bit, sequence unsigned 8-bit, CRC-8. Steering is decoded but steering servo
control is not implemented yet. **No Pi `bridge.c` exists in this repository or
its available branches**, so the sender must be checked against this layout.
An 8-bit clutch requires the Pi to scale its original pedal reading to 0–255.

Edit [the motor settings](src/motor_config.h) after checking actual pedal
ranges, rest/pressed directions, motor polarity, counts per wheel revolution,
and safe maximum RPM. The present 0–1000 throttle/brake ranges and PID gains
are initial placeholders. Set `THROTTLE_RAW_REST` and `THROTTLE_RAW_FULL` to
the actual readings; the code handles either direction. Brake and clutch each
have a threshold and `PRESSED_HIGH` switch. `MOTOR_OUTPUTS_ENABLED` starts at
`0`: firmware can
read commands and compute a duty value but holds both motors in dynamic braking.
Only set it to `1` after verifying the Pi packet and motor wiring with wheels
lifted. The encoder counts-per-revolution setting is in
`boards/nucleo_f401re.overlay`; PWM pin mapping and 1 kHz period are there too.
No physical motor response or 2 ms timing measurement has been performed.
The link watchdog uses 95 ms plus a 5 ms receive wait to target the PDF's
stricter 100 ms unplug checkoff; an earlier PDF paragraph says 150 ms.

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
