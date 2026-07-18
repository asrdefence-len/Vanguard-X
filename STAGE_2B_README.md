# Vanguard X Stage 2B — timing integration

Stage 2B makes the validated `RadarTiming` solution the source of the uniform
dwell parameters consumed by the executor, pulse plan, simulated source,
receive-only Ettus source, and radar processor.

The default search profile is:

- `Frank10_20MHz`
- fixed 40 MS/s
- 2 kHz PRF / 500 us PRI
- 32 pulses / 16 ms CPI
- 15 km instrumented range
- 6 us receive start delay
- 4043 complex receive samples per pulse

The receive delay is included in the range axis, and matched-filter output is
aligned to the echo leading edge. This preserves absolute target range.

## Install

Extract the package directly into `~/Projects/Software`, replacing the listed
files.

## Verify before running Main

```bash
cd ~/Projects/Software

python3 -m unittest -v \
    TestWaveformLibrary.py \
    TestSampledWaveformProcessing.py \
    TestRadarTiming.py \
    TestRadarTimingIntegration.py

python3 TestRadarPlans.py
python3 TestSchedulerIntegration.py
```

The unit-test command should report 27 passing tests. The two existing
regression scripts should also pass.

## Simulator integration run

`VanguardxMain_scheduler.py` now defaults to the simulated radar source. Start
it only after the tests pass:

```bash
python3 VanguardxMain_scheduler.py
```

At startup, confirm the timing line reports 40 MS/s, 2000 Hz PRF, 500 us PRI,
32 pulses, 16 ms CPI, 6 us RX start, 4043 RX samples, and 15 km maximum range.

This stage does not enable timed RF transmission. The Ettus path remains
receive-only; it schedules each finite RX window from the dwell's validated PRI
origin plus its derived receive-start delay.
