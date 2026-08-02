# Vanguard X Stage 12K — Tactical Green Display Framework

Date: 2 August 2026

Stage 12K changes the structural radar-display symbology from white/grey to a
subdued tactical green on black, consistent with battle-management and combat
system displays.

## Changed to green

- PPI East/North axes, ticks and numeric tick labels
- PPI title and axis labels
- Cartesian grid references
- range rings and range-ring labels
- true-bearing degree and cardinal labels
- sector boundary lines
- range-profile axes, ticks, labels, title and grid
- the equivalent axes and grid in the fallback `SimpleDisplay`

## Operational colours deliberately retained

- confirmed tracks: white
- initiating/tentative tracks: amber
- antenna beam: yellow
- detections: cyan
- tracker plots: dim amber
- warnings and STOP controls: red
- map coastline and labels: separate map colours

This keeps display framework information visually subordinate to live radar
products and retains the established track-status hierarchy.

## Default palette

```text
Primary tactical green:  #00d060
Muted tactical green:    #65b883
Grid and range rings:    #176b3a
Sector boundaries:       #00a85a
```

The palette can be overridden from the display configuration with:

```python
"TacticalGreenColour": "#00d060",
"TacticalMutedGreenColour": "#65b883",
"TacticalGridColour": "#176b3a",
"RangeRingColour": "#176b3a",
"SectorBoundaryColour": "#00a85a",
```

## Install

From `~/Projects/Software`:

```bash
unzip -o \
  ~/Downloads/VanguardX_Stage12K_TacticalGreenDisplay_20260802.zip \
  -d ~/Projects/Software
```

Restart both the radar and remote UI processes after installation so the new
display module is loaded.

## Verification

- 463 GUI-free regression tests passed.
- Two expected skips remained.
- The known headless `TestRadarDisplayGeometry` import was excluded because
  the test environment cannot construct `PyQt5.QtWidgets.QWidget`.
- Four new tactical-palette regression tests passed.

Visually verify that green linework remains readable against both open water
and the land-map overlay before committing.
