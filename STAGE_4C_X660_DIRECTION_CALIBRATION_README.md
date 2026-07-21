# Stage 4C - X6-60 direction calibration

Stage 4C performs one independent, speed-limited movement to determine whether
increasing X6-60 raw angle moves the Vanguard X antenna clockwise or
anticlockwise when viewed from above.

The live scheduler remains in `x660-read-only` mode with
`X660DirectionSign = 0`.  Do not run `VanguardxMain_scheduler.py` at the same
time as this harness.

## Fixed test envelope

- One absolute-position command (`0xA4`)
- Raw target: current raw angle plus `1.00 deg`
- Speed limit: `1 deg/s`
- Travel guard: `1.50 deg` from the starting angle
- Current guard: `2.00 A`
- No configuration, encoder-zero, brake, CAN-ID, PID, acceleration, or ROM write
- Emergency motor-stop (`0x81`) only after an abnormal armed run
- Dry-run default; all three physical-safety acknowledgements are mandatory

The position-command layout follows the X6-60 CAN protocol: unsigned 16-bit
speed in `1 deg/s` units followed by a signed 32-bit absolute multi-turn angle
in `0.01 deg` units, all little-endian.

## Software checks and dry run

```bash
python3 -m py_compile \
    X660CalibrationProtocol.py \
    RunX660DirectionCalibration.py \
    TestX660DirectionCalibrationSafety.py

python3 -m unittest -v TestX660DirectionCalibrationSafety.py
python3 RunX660DirectionCalibration.py
```

The last command must finish with:

```text
DRY RUN: no CAN interface opened and no frame transmitted
```

## Armed run gate

Do not arm the test until all of the following are true:

- the antenna assembly is mechanically secured;
- the swept area is clear;
- there is at least two degrees of physical clearance in both directions;
- power isolation is within immediate reach;
- `can0` is already UP at 1 Mbit/s;
- the scheduler and every other CAN client are stopped.

Only then use the three explicit acknowledgement options shown by
`python3 RunX660DirectionCalibration.py --help`.

After a PASS, report whether the antenna moved clockwise/right or
anticlockwise/left when viewed from above.  Do not repeat the movement and do
not yet change `X660DirectionSign`; the observation and telemetry output must
be reviewed together first.
