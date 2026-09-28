# cmu-embedded

CMU 18-449 Distributed Embedded Systems, Lab 2. Target: **NUCLEO-F401RE**
(`nucleo_f401re`). The application currently runs Zephyr Blinky.

## Build and flash on this machine

Open a dedicated terminal:

```bash
cd /home/au/Desktop/cmu-embedded
source .venv/bin/activate
west build -b nucleo_f401re . -d build/nucleo_f401re
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
west flash -d build/nucleo_f401re
```

The onboard green LED should toggle every second (a full on/off cycle takes two
seconds). Lab 2 requires this checkpoint before wiring external hardware. The
F401RE's LED alias comes from Zephyr; the Nordic overlay in `boards/` does not
apply to this target.

The course's *Flashing and Debugging Your Nucleo* guide lists
**STM32CubeProgrammer as required** and notes that OpenOCD is included with
the Zephyr SDK on Linux. STM32CubeProgrammer **2.23.0 is installed** at
`.tools/STM32CubeProgrammer/`. This project's virtual environment activation makes its CLI available
in `PATH`. Verify it with `STM32_Programmer_CLI --version`. The F401RE default
runner is set in `CMakeLists.txt`. To name it explicitly for one flash:

```bash
west flash -d build/nucleo_f401re -r stm32cubeprogrammer
```

The guide's `source .venv/bin/active` line contains a typo. Use
`source .venv/bin/activate` here; this local activation also sets `ZEPHYR_BASE`, so a separate
`zephyr-env.sh` command is unnecessary. Do not replace the board's ST-LINK
firmware with J-Link firmware; the course guide explicitly prohibits that.

Subsequent builds can use `west build -d build/nucleo_f401re`. Activate the
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
| `build/nucleo_f401re/` | Firmware and generated build files |

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

Verified on this machine: complete F401RE firmware build, `python -m pip check`,
and SDK OpenOCD startup. Firmware: `build/nucleo_f401re/zephyr/zephyr.elf`.

USB rules are installed, and the Nucleo's ST-LINK/V2.1 was detected. The
STM32CubeProgrammer CLI version command passed. This build now selects
STM32CubeProgrammer as its default flash runner. Firmware has not been flashed
or checked on the physical LED. Do not run West with sudo; use USB rules.
