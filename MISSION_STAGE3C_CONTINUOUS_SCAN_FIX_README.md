# Vanguard X Mission Stage 3C continuous-scan correction

## Fault corrected

Mission Stage 3B represented a 360 task as the sector `0.000 -> 359.999`.
The persistent `SearchTask` remained in its default `SECTOR` pattern, so
PointingManager treated North as an endpoint and repeatedly reversed the X6-60
simulator around 0 degrees.

## Correct behaviour

- A 360 Scan selects `CONTINUOUS_CW` or `CONTINUOUS_CCW`.
- Continuous rotation never reverses at North.
- `ScanCycle` increments once per completed revolution.
- A Sector Scan selects `SECTOR` and retains endpoint reversal.
- Periodic Sector Scan interruption and continuous-baseline resumption are
  unchanged.
- Mission runtime status shows the active waveform, PRF, and pulses/CPI.

The initial `Search timing:` line printed during radar startup describes the
pre-mission default profile. The authoritative mission change is the later
`Operator timing applied:` line. The Mission status now also displays the
active values.

## Install

Commit or otherwise preserve the current Stage 3B working tree first. Then
extract this overlay into `~/Projects/Software`.

```bash
cd ~/Projects/Software

unzip -o ~/Downloads/VanguardX_Mission_Stage3C_Continuous_Scan_Fix.zip \
    -d ~/Projects/Software
```

## Test

```bash
python3 -m unittest -v TestMissionContinuousScan.py
python3 -m unittest -v TestMissionExecution.py
python3 -m unittest -v TestMissionModelValidator.py
python3 -m unittest -v TestRadarLink.py
python3 -m unittest discover -s . -p 'Test*.py' -v
```

Then run:

```bash
python3 VanguardxMain_scheduler.py --system-sim
```

Validate, load, and start the same Mission profile. For a clockwise 360 Scan,
the terminal must report:

```text
Mission: RUNNING_360
Operator timing applied: waveform=Frank10_20MHz, ...
PointingManager continuous scan start: CONTINUOUS_CW at ... deg/s
```

Success criteria:

- azimuth increases continuously through North;
- the direction does not alternate each dwell;
- no `0.00 -> 360.00` sector-start message appears for the 360 task;
- `ScanCycle` remains constant between North crossings and increments once
  per full revolution;
- a periodic sector task still reverses at its configured endpoints and then
  returns to continuous rotation.
