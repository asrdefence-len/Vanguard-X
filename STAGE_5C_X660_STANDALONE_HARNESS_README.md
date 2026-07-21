# Stage 5C — Standalone guarded X6-60 motion harness

Stage 5C adds a standalone hardware harness for the first deliberately bounded
X6-60 motion check. It remains disconnected from `X660ReadOnlyController`,
`CreateX660Controller`, `PointingManager`, and the radar scheduler.

## Safety boundary

- Dry-run preview is the default and does not open `can0`.
- The only exposed motion is incremental position command `0xA8`.
- Increment magnitude is limited to 0.01–2.00 degrees.
- Speed is limited to 1–5 degrees per second.
- Default test is 1.00 degree at 2 degrees per second.
- Live mode requires four explicit acknowledgements.
- Every live attempt proceeds to `0x81` closed-loop stop and then `0x80` motor
  shutdown, including when the motion transaction fails.
- A failure to confirm `0x80` is reported as a fail-safe error.
- Continuous speed and absolute-position commands are not exposed.

## Software-only verification

These commands use only fake-bus tests and do not access CAN hardware:

```bash
python3 -m py_compile \
    RunX660GuardedMotionTest.py \
    TestRunX660GuardedMotionTest.py

python3 -m unittest -v \
    TestX660MotionProtocol.py \
    TestX660GuardedMotionTransport.py \
    TestRunX660GuardedMotionTest.py
```

## Dry-run preview

This command opens no CAN interface and transmits no frames:

```bash
python3 RunX660GuardedMotionTest.py
```

Expected command plan for node 1:

```text
A8 00 02 00 64 00 00 00   incremental +1.00 degree, maximum 2 deg/s
81 00 00 00 00 00 00 00   closed-loop stop
80 00 00 00 00 00 00 00   motor output shutdown
```

Do not run live motion merely because the software tests pass. Live execution
is a separate supervised checkpoint requiring a clear motion area, a supported
axis, an available emergency stop, and the exact acknowledgement flags printed
by `--help`.
