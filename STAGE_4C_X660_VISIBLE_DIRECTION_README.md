# Stage 4C - visible X6-60 direction calibration

The first `+1.00 deg` raw-angle movement passed, but its physical direction was
too small to see. This second and final calibration movement commands exactly
`+5.00 deg` of raw encoder travel at `1 deg/s`. It exists only to distinguish
clockwise/right from anticlockwise/left when viewed from above.

Do not run `VanguardxMain_scheduler.py` or another CAN client at the same time.
Keep the scheduler configured as `x660-read-only` with `X660DirectionSign = 0`.

## Fixed test envelope

- One absolute-position command (`0xA4`)
- Raw target: current raw angle plus `5.00 deg`
- Speed limit: `1 deg/s`
- Travel guard: `5.50 deg` from the starting angle
- Reported-speed guard: `3 deg/s`
- Current guard: `2.00 A`
- Temperature guard: `50 C`
- Motor error flags checked continuously
- Movement timeout: `12 s`
- Emergency motor-stop (`0x81`) only after an abnormal armed run
- No configuration, encoder-zero, brake, CAN-ID, PID, acceleration, or ROM write
- Dry-run default; all three physical-safety acknowledgements are mandatory

## Before the dry run

Put a removable tape mark across the fixed and rotating sections so that a
five-degree movement and its direction are easy to see. Confirm at least six
degrees of free rotation from the present position. Keep power isolation within
immediate reach.

## Software checks and dry run

```bash
python3 -m py_compile \
    RunX660VisibleDirectionCalibration.py \
    TestX660VisibleDirectionCalibrationSafety.py

python3 -m unittest -v TestX660VisibleDirectionCalibrationSafety.py
python3 RunX660VisibleDirectionCalibration.py
```

The last command must finish with:

```text
DRY RUN: no CAN interface opened and no frame transmitted
```

Do not arm the live movement until the dry-run output has been reviewed. After
the live PASS, report whether increasing raw angle moved the antenna
clockwise/right or anticlockwise/left when viewed from above. Do not repeat the
movement and do not change `X660DirectionSign` until that observation is
reviewed with the telemetry output.
