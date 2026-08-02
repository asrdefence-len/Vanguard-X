# Vanguard X Stage 12L — Compact Dashboard Status

Date: 2 August 2026

## Purpose

Recover vertical space for the range-profile plot by removing redundant rows
from the dashboard status panel.

## Changes

- Removed the separate `X6-60:` status row. The display continues to show the
  operational true beam bearing, and the X6-60 state remains available to the
  radar control path.
- Removed the separate `Y-axis:` limits row. Range-profile scaling and visible
  axis tick labels are unchanged.
- Combined the confirmed and tentative track counts into one row:

  `Tracks: xx, Tent: yy`

This saves three status-panel rows without changing radar, tracker, pointing,
Mission, or range-profile processing.

## Verification

Run:

```bash
python3 -m unittest -v \
  TestCompactDashboardStatus.py \
  TestDashboardEarthReferencedScan.py \
  TestTacticalGreenDisplay.py
```

Then start Vanguard X and confirm that the range-profile plot has more vertical
space and the compact track-count row updates correctly.
