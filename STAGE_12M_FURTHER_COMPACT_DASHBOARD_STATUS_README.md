# Vanguard X Stage 12M — Further Compact Dashboard Status

Date: 2 August 2026

## Purpose

Recover two more rows of vertical space for the range-profile plot while
retaining all dashboard status information.

## Changes

- Combined detection and plot counts, including their visibility state:

  `Dets: xx (shown/hidden), Plots: yy (shown/hidden)`

- Combined peak range and peak velocity, retaining their units:

  `Peak R: xxxx.x m, Peak V: x.x m/s`

Together with Stage 12L, the compact dashboard now saves five status-panel
rows without changing radar, tracker, pointing, Mission, display-toggle, or
range-profile processing.

## Verification

Run:

```bash
python3 -m unittest -v \
  TestCompactDashboardStatus.py \
  TestDashboardEarthReferencedScan.py \
  TestTacticalGreenDisplay.py
```

Then start Vanguard X and confirm that both combined rows update correctly and
that the range-profile plot has gained the additional vertical space.

The complete GUI-free operational regression passed 467 tests with two
expected skips on 2 August 2026. The headless Qt geometry test remains excluded
from this environment, as in the preceding stage.
