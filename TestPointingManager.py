"""
Regression tests for PointingManager.py using the X6-60 simulator.
"""

import time

from NavigationState import SimulatedNavigationSource
from PointingManager import PointingManager
from X660Controller import SimulatedX660Controller
from RadarTasks import (
    AngleFrame,
    MakeSearchTask,
    MakeTrackTaskFromRevisit,
    RevisitRequest,
)


x660 = SimulatedX660Controller(
    InitialAzimuthDeg=20.0,
    MaxPanRateDegPerSec=60.0,
    PositionToleranceDeg=0.2,
)

x660.Open()

nav = SimulatedNavigationSource(
    initial_heading_deg=30.0,
    turn_rate_deg_per_sec=0.0,
)

manager = PointingManager(
    x660=x660,
    endpoint_margin_deg=0.5,
    position_tolerance_deg=0.5,
)

# Coordinate conversion.
assert abs(manager.RelativeToTrueBearing(20.0, 30.0) - 50.0) < 1e-9
assert abs(manager.TrueToRelativeAzimuth(50.0, 30.0) - 20.0) < 1e-9

# Platform-fixed continuous search.
search = MakeSearchTask(
    TaskId=1,
    SectorStartDeg=20.0,
    SectorStopDeg=30.0,
    ScanRateDegPerSec=60.0,
    SectorFrame=AngleFrame.PLATFORM,
)

manager.ActivateTask(search, nav.get_attitude())

deadline = time.time() + 1.0
reversed_once = False

while time.time() < deadline:
    state = manager.Update(nav.get_attitude())
    if search.Sector.ScanCycle >= 2:
        reversed_once = True
        break
    time.sleep(0.01)

assert state.Valid
assert state.Mode.value == "CONTINUOUS_SCAN"
assert reversed_once

# Track task in true coordinates.
request = RevisitRequest(
    TrackId=12,
    RequestedTimeSec=time.time(),
    DeadlineTimeSec=time.time() + 2.0,
    PredictedRangeM=8000.0,
    PredictedTrueBearingDeg=70.0,
    Priority=100,
    RevisitIntervalSec=0.5,
)

track = MakeTrackTaskFromRevisit(
    TaskId=2,
    Request=request,
)

manager.ActivateTask(track, nav.get_attitude())

# Heading 30 true and target 70 true -> command relative 40 deg.
assert abs(manager.LastCommandedRelativeDeg - 40.0) < 0.01

deadline = time.time() + 1.0
track_ready = False

while time.time() < deadline:
    state = manager.Update(nav.get_attitude())
    if state.Ready:
        track_ready = True
        break
    time.sleep(0.01)

assert track_ready
assert state.ActiveTrackId == 12
assert abs(state.BeamBearingTrueDeg - 70.0) < 1.0

# Moving-platform correction: turn heading to 35 true.
nav.set_heading(35.0)
state = manager.Update(nav.get_attitude())

# True target remains 70, so relative command should move to 35.
assert abs(manager.LastCommandedRelativeDeg - 35.0) < 0.1

manager.ResumeSearch(nav.get_attitude())
state = manager.Update(nav.get_attitude())

assert state.Mode.value == "CONTINUOUS_SCAN"
assert state.ActiveTrackId is None

manager.Stop()
x660.Close()

print("PointingManager regression test passed")
