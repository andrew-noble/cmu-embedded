# Motor measurements — 2026-09-29

## Encoder counts per wheel revolution

Five manual wheel revolutions per side, measured using accumulated encoder
counts. The wheel was turned in the negative encoder direction; calibration
uses the magnitude of the count difference.

| Wheel | Starting count | Ending count | Absolute difference | Turns | Counts/revolution |
| --- | ---: | ---: | ---: | ---: | ---: |
| Left | 135636 | 129044 | 6592 | 5 | 1318.4 |
| Right | 132052 | 125482 | 6570 | 5 | 1314.0 |

Average: `(1318.4 + 1314.0) / 2 = 1316.2` counts per wheel revolution.
Both encoder nodes use **1316**, rounded to the integer required by the
devicetree property. Manual start/end alignment limits measurement precision.

## PWM sweep with wheels lifted

Three seconds per step, averaging the final 1.5 seconds. Duty is the actual
enable-pin PWM percentage, without a controller offset. The reported RPM
below was collected using the old **3960** counts/revolution setting and is
retained as originally recorded; it is approximately three times too low.
For an approximate correction, multiply these RPM values by `3960 / 1316`.
Re-measure after rebuilding with the new encoder setting for updated results.

| Actual PWM | Original left RPM | Original right RPM |
| ---: | ---: | ---: |
| 0% | 0.00 | 0.00 |
| 5% | 0.00 | 0.00 |
| 10% | 0.00 | 0.00 |
| 20% | 0.00 | 0.00 |
| 30% | 44.13 | 48.00 |
| 40% | 60.53 | 65.53 |
| 50% | 72.53 | 77.33 |
| 60% | 82.07 | 84.40 |
| 70% | 89.07 | 89.53 |
| 80% | 93.80 | 93.00 |
| 90% | 97.33 | 96.40 |
| 100% | 106.33 | 104.80 |

Both wheels were observed rotating at 30% and stopping after the test.
20% did not start them in this test. No loaded-speed or current measurements
were taken. Original CSVs are stored locally in the ignored `data/` directory.

## Closed-loop baseline — 2026-09-30

Telemetry file: `tools/telemetry/data/20260930-181852-417269.csv` (local,
ignored). No link-loss events were recorded during this run. Near the initial
100–106 RPM request, average speed peaked around 143.5 RPM before slowly
settling. Near 200 RPM, average speed peaked around 207 RPM and settled around
200–205 RPM. These are logged samples, not guaranteed continuous-time peaks;
USB receive timestamps batch multiple firmware samples.

Holding approximately 106 RPM used about 23% actual PWM; holding approximately
200 RPM used about 39%. The previous feedforward alone requested 36% at
100 RPM and 52% at 200 RPM, leaving the integral term to remove excess drive.
The next tuning candidate uses piecewise feedforward effort of 0%, 4%, and 24%
at 0, 100, and 200 RPM, before the existing 20% offset. That maps the two
nonzero reference points to about 23% and 39% actual PWM. PI gains remain
Kp=0.2 and Ki=0.1; Kd=0. Repeat the same test before claiming reduced overshoot
or critical damping. Loaded operation still needs separate validation.

## Second run and independent-wheel candidate

`tools/telemetry/data/20260930-182539-465305.csv` recorded no link events.
At a 72 RPM target, the left encoder stayed at zero while the right reached
about 140 RPM at 26–28% shared PWM. The left began turning near 31% PWM as
the target increased. This is consistent with unequal starting requirements;
the shared average feedback hid the stationary wheel. The observation does
not by itself rule out an intermittent hardware or encoder problem.

The next candidate gives each wheel independent PI state and PWM, with a shared
target. Kp changes from 0.2 to 0.15 and Ki from 0.1 to 0.05; Kd remains zero.
The previous feedforward points are retained. Integral anti-windup prevents
accumulation farther into saturation. Separate left/right PWM fields expose
each controller's effort. Verify both wheels physically move, then repeat the
step test; these are candidate gains, not a verified critically damped tune.

## Left-wheel load test — 2026-09-30

`tools/telemetry/data/20260930-184347-522039.csv` has no recorded link events.
Under load, left speed stayed roughly 45–80 RPM versus a 198–200 RPM target,
with PWM plateauing around 75–76%; right speed remained near target. The old
integral cap (500000 RPM ms at Ki=0.05) limited integral effort to 25%, preventing
the output from reaching its nominal 100% limit. After load release, the left
peaked around 246 RPM and recovered slowly.

The next candidate keeps Kp=0.15, raises Ki to 0.10, and adds Kd=0.005 on
measurement with a 50 ms low-pass filter. The integral bound is now derived
from Ki to allow 100% integral effort. Anti-windup permits the update crossing
into saturation, then prevents further accumulation in that direction. A host
test of the production controller with fixed target 200 RPM and measured
50 RPM reaches 100% PWM in 3.57 s and holds without integral growth. This is
a controller calculation, not a prediction of physical loaded recovery.
Physical overshoot, startup and load-release response still need retesting.

## Faster recovery candidate

`tools/telemetry/data/20260930-185052-420415.csv` records no link events.
PWM reaches 100% under left-wheel loading, confirming the premature cap is
removed. Following release near 74 s, left speed peaks around 305 RPM against
a 200 RPM target and takes roughly 12 s to approach target. Host timestamps
and manually varying loads limit precise transient comparisons.

The next candidate raises Kp from 0.15 to 0.30, Ki from 0.10 to 0.30, and Kd
from 0.005 to 0.010, retaining the 50 ms derivative filter. This targets faster
load correction and integral unwinding after release. The integral storage
limit automatically rescales with Ki. These gains need physical verification
for oscillation and overshoot; critical damping has not been established.

`tools/telemetry/data/20260930-185356-675098.csv` shows no link events and
100% left PWM during loading. Speed remains below target while heavily loaded;
after release, left speed reaches roughly 259–264 RPM and takes several seconds
to recover. The next requested faster candidate is Kp=0.45, Ki=0.60, Kd=0.020,
with the 50 ms derivative filter unchanged. Higher gains cannot increase drive
beyond 100% during saturation. Physical oscillation and release overshoot must
be checked before accepting this tuning.
