# Vanguard X Mission — scheduler simulation increment

This source set starts Mission implementation on top of the hardware-tested
Phase 1B radar/UI separation.

## What is implemented

- Versioned, serialisable `MissionProfile` data classes.
- Ordered `360 Scan` and `Sector Scan` primary tasks.
- Continuous baseline scan with finite periodic scan interrupts.
- Mission-wide `Track Update` interrupt policy.
- Circular sector-angle handling, including sectors that cross 000 degrees.
- GUI-independent validation and derived timing/coverage calculations.
- Separate draft, validated and loaded immutable mission snapshots.
- A `Mission` tab in the existing Qt operator application.
- Add, remove, duplicate and reorder task controls.
- Selected 360/Sector task editors.
- Track Update policy editor.
- Mission JSON file open/save.
- Persistent system/TX/azimuth/mission/link header and STOP button on both tabs.
- Radar-side immutable mission execution controller.
- Versioned Load, Start, Pause, Resume and STOP commands over the Phase 1B link.
- Ordered enabled-task execution with active-task duration accounting.
- Safe-boundary periodic pre-emption and baseline resumption.
- STOP or REPEAT mission completion policy.
- Authoritative mission/task state returned to both local and remote UIs.
- Safe-boundary task transitions after a complete dwell.
- Communications-loss Mission STOP in addition to the Phase 1B TX/scan inhibit.

The existing PPI and range-profile widgets remain together and unchanged on the
`Radar` tab. Mission planning does not call the Ettus, scheduler, antenna, ATR,
TRM or other hardware interfaces.

## Current execution boundary

Mission execution is enabled only when the system mode is `SIM`. `Start
Mission` is rejected in `HARD` and `RF LOOPBACK`; this increment therefore
cannot start Ettus transmission or operational antenna motion.

The Mission workflow is:

1. Edit the draft.
2. Select **Validate Mission**.
3. Select **Load Mission** while the radar is stopped.
4. Wait for the authoritative `LOADED` state.
5. Select **Start Mission**.
6. Use **Pause** / **Resume** at safe dwell boundaries.
7. Use the persistent **STOP** control to abort and unload the active mission.

Task duration uses active surveillance time. Paused time is excluded. Draft
edits made during execution do not alter the radar-side loaded snapshot.

For continuous surveillance with a periodic focused scan, configure:

1. The first enabled task as `Continuous`.
2. Each later interrupt with a finite `Duration`.
3. Enable `Periodic interrupt` and set `Repeat` to an interval longer than
   its duration.

For example, a continuous 360 scan plus a 1 minute sector task repeated every
5 minutes runs the 360 scan as the baseline, changes to the sector only after a
completed dwell, then resumes the 360 scan after the sector duration. Pause
freezes both task duration and recurrence timing.

The X6-60 simulator executes a 360 task with an explicit continuous-rotation
pattern. It does not use 0/360 as reversing sector endpoints. CW and CCW
missions cross North without reversing, and `ScanCycle` advances once per
completed revolution. Sector tasks continue to use endpoint reversal.

The next Mission increment is Track Update simulation: eligible tagged-track
requests, safe-boundary pre-emption, CCW-to-CW gate nod, and suspended-scan
resumption. No Track Update interrupt is executed by this increment.

## Phase 1B checkpoint first

Commit the already hardware-tested Phase 1B files before copying the Mission
increment into the repository. The supplied `PHASE1B_BASELINE.sha256` records
the exact pre-Mission file versions in
`VanguardX_UI_Separation_Phase1B_Timing_Isolated.zip`.

In the full Vanguard X repository:

```bash
git status --short
git add VanguardxMain_scheduler.py RadarDisplayQt5.py \
    RadarRemoteDisplay.py RadarLinkClient.py RadarLinkProtocol.py \
    RunVanguardUiClient.py TestRadarLink.py VanguardUiSeparation_README.md
git commit -m "Separate operator UI with timing-isolated radar link"
```

Then copy in the Mission files and run:

```bash
python3 -m unittest -v TestMissionModelValidator.py
python3 -m unittest -v TestMissionExecution.py
python3 -m unittest -v TestRadarLink.py
```

The complete existing Vanguard X test suite should also be run in
`~/Projects/Software` before any hardware mode is used.

## Current validation limits

The validator uses the established 1–4 kHz PRF range, pulses/CPI range and
15 km maximum range. It deliberately warns that usable antenna beamwidth and
the future continuous-drive speed/acceleration limits are not yet
characterised. These warnings do not block scheduler simulation. They must not
be closed with assumed simulator values: verified antenna-pattern and drive
limits are required before operational Mission execution. The example 24 RPM
and sector values are draft examples, not approved operating limits.
