# Vanguard X Stage 12C — Angular Detection Consolidation

## Purpose

Stage 12C removes high-SNR angular painting from the normal PPI without
changing Golay matched filtering or per-dwell range-Doppler CFAR.

The processing boundary is:

```text
Golay range-Doppler map
    -> per-dwell 2-D CFAR cells
    -> per-dwell range/Doppler component
    -> association through one true-bearing beam crossing
    -> built-in two-way antenna-pattern fit
    -> one consolidated plot
    -> legacy and Earth-referenced trackers
```

`AngularDetectionProcessor.py` recursively accumulates compact power samples
while the beam crosses a seeded target. It fits the same two-way `sinc^4`
response used by `TargetScenario.py`.

## Extended targets

The processor does not estimate or classify target width in Stage 12C.

A 200 m vessel is allowed to occupy multiple CFAR range cells. Those cells are
clustered within each dwell and their total response is fitted through the
antenna crossing. The output is one centroid plot with `RangeMinimumM`,
`RangeMaximumM`, and `RangeExtentM` retained as diagnostics.

`RangeExtentM` is not a claimed physical vessel length. A later high-resolution
classification stage may use calibrated angular and range-extent models.

## Display behaviour

The normal PPI now uses:

```python
"ShowRawDetections": False
"ShowTrackerPlots": True
```

Raw CFAR cells remain available to the logger and range-profile diagnostics.
Temporarily set `ShowRawDetections=True` only when inspecting CFAR behaviour;
doing so will intentionally restore the original high-SNR angular arc.

## Initial parameters

```python
"AngularDetectionProcessingEnabled": True
"AngularPerDwellClusterRangeGapM": 100.0
"AngularPerDwellClusterDopplerGapHz": 120.0
"AngularAssociationRangeGateM": 250.0
"AngularAssociationDopplerGateHz": 180.0
"AngularMaximumSampleGapDeg": 4.0
"AngularCloseAfterMissedDwells": 2
"AngularCloseAfterMissedAngleDeg": 5.0
"AngularMaximumMissedDwells": 12
"AngularMinimumFitSamples": 3
"AngularHypothesisStepDeg": 0.1
"AngularPowerRangeHalfWidthBins": 32
"AngularPowerDopplerHalfWidthBins": 1
"AngularPlotPersistenceSec": 30.0
```

The v2 crossing close rule is angular rather than dwell-count only. A crossing
may continue across a `ScanCycle` change at a sector reversal when the Mission
task is unchanged. It closes after the antenna has moved one configured
beamwidth beyond the last CFAR hit, with `AngularMaximumMissedDwells` retained
as a safety timeout for stopped or staring operation. This prevents one target
near a sector endpoint, or one target with a brief internal CFAR gap, from
creating separate leading-edge and trailing-edge plots.

These are simulation-stage values. `BeamwidthDeg=5.0` still means approximately
five degrees from boresight to the first null, not five degrees total.

## Validate after installation

```bash
cd ~/Projects/Software

python3 -m py_compile \
  AngularDetectionProcessor.py \
  CfarDetector.py \
  RadarRemoteDisplay.py \
  VanguardxMain_scheduler.py

python3 -m unittest -v \
  TestAngularDetectionProcessor.py \
  TestEarthReferencedMeasurements.py \
  TestEarthReferencedTracker.py \
  TestRadarLink.py \
  TestSchedulerIntegration.py
```

Then run the normal simulation and inspect the strong vessel near 344 degrees
true. The expected result is:

- raw cyan CFAR arc absent from the normal PPI;
- one white angular plot after the beam crossing closes;
- plot bearing between dwell-angle samples where supported by the fit;
- tracks seeded from consolidated plots rather than individual CFAR cells;
- `Dets` may still be large in console diagnostics because it remains the raw
  per-dwell CFAR count;
- `AngularCompletedPlotsThisDwell` becomes one when a crossing closes.

Do not commit until the simulated vessel produces one stable plot through a
complete scan and the normal tentative/confirmed track sequence still occurs.

## Calibration boundary

No antenna calibration file is introduced in Stage 12C. Simulation uses the
existing built-in pattern. The later installed-antenna calibration will replace
the pattern lookup without changing the crossing, display, or tracker
interfaces.
