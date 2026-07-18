# Vanguard X Stage 2D — operator maximum range

This update adds maximum instrumented range to the compact timing controls.
The selectable range is 1–15 km, matching the existing PPI and Range Profile
extent; neither plot geometry is changed.

Waveform, PRF, pulses per CPI, and maximum range are applied as one validated
transaction at the next dwell boundary. Maximum range drives the receive-window
duration and RX sample count. Invalid combinations are rejected and the last
valid configuration is restored.

The timing model now also rejects a maximum range that does not exceed the
selected waveform's minimum full-echo range. For example, `Frank10_10MHz` has a
longer blind range than `Frank10_20MHz`.

## Install and verify

```bash
cd ~/Projects/Software

python3 -m unittest -v \
    TestWaveformLibrary.py \
    TestSampledWaveformProcessing.py \
    TestRadarTiming.py \
    TestRadarTimingIntegration.py \
    TestRadarTimingControls.py

python3 TestRadarPlans.py
python3 TestSchedulerIntegration.py
python3 VanguardxMain_scheduler.py
```

The unit-test command should report 34 passing tests. In Main, select maximum
range and press **Apply**. The derived status line and terminal message should
show the new range and corresponding RX sample count.
