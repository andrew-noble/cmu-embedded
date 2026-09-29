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
