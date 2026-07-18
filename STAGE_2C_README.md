# Vanguard X Stage 2C — compact operator timing controls

This update adds waveform, PRF, and pulses-per-CPI selection to the existing
top-right Controls box without changing the PPI or Range Profile layout.

The controls provide:

- waveform selection from the explicit 10/20 MHz catalogue;
- PRF selection from 1.00 to 4.00 kHz;
- pulses-per-CPI selection from 8 to 128;
- an explicit Apply timing action;
- compact derived PRI, CPI, and RX-sample feedback.

Selections are validated through `RadarExecutor` and applied for the next dwell.
An invalid selection is rejected and the previously valid configuration is
restored. The removed `Auto min` row is reused for timing feedback. The status
panel's redundant Sector and Step lines now report applied waveform and timing.

## Install and test

Extract the package into `~/Projects/Software`, then run:

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

The unit-test command should report 31 passing tests. In Main, change one timing
value and press **Apply timing**. The terminal should print the newly applied
waveform, PRF, pulse count, and CPI duration before the next dwell executes.
