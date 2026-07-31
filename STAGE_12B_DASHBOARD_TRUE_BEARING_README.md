# Vanguard X Stage 12B — Dashboard True-Bearing Scan

This focused update makes the main Dashboard scan controls geographic
true-bearing controls:

- 000 degrees true = North
- 090 degrees true = East
- 180 degrees true = South
- 270 degrees true = West

The Dashboard scan sector and PPI remain Earth-referenced while
`PointingManager` converts the true-bearing endpoints to vessel-relative
X6-60 commands using live ship heading.

Operator labels now distinguish the frames:

- `Start °T` and `Stop °T`
- `TRUE BRG ... deg T`
- `Ship hdg ... deg T`
- `X6-60 ... deg rel`

Mission sector bearings, detections, tracks, map data, and the PPI scan line
remain true/Earth referenced. The X6-60 encoder angle remains vessel-relative
for control and diagnostics.

## Install

From the Vanguard X software directory:

```bash
unzip -o \
  ~/Downloads/VanguardX_Stage12B_DashboardTrueBearing_20260731.zip \
  -d ~/Projects/Software
```

## Validate

```bash
cd ~/Projects/Software

python3 -m unittest -v \
  TestDashboardEarthReferencedScan.py \
  TestMovingPlatformCoordinateFrames.py \
  TestX660PointingControlLoop.py \
  TestMissionDashboardHandover.py

python3 MovingPlatformPpiOperationalValidation.py
python3 OperationalDisplayTrackValidation.py

python3 -m py_compile \
  RadarDisplayQt5.py \
  VanguardxMain_scheduler.py \
  TestDashboardEarthReferencedScan.py

git diff --check
```

## Expected runtime behaviour

For a Dashboard sector of 040 to 090 degrees true with ship heading
135 degrees true:

- the PPI sector remains 040 to 090 degrees true;
- the scan line is drawn in true bearing;
- the X6-60 endpoints are 265 and 315 degrees vessel-relative.

