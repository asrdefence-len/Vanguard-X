"""
Basic regression tests for RadarTasks.py.
"""

from RadarTasks import (
    AngleFrame,
    MakeSearchTask,
    MakeTrackTaskFromRevisit,
    RadarTaskType,
    RevisitRequest,
)


search = MakeSearchTask(
    TaskId=1,
    SectorStartDeg=10.0,
    SectorStopDeg=120.0,
    ScanRateDegPerSec=14.0,
    SectorFrame=AngleFrame.PLATFORM,
)

assert search.TaskType == RadarTaskType.SEARCH
assert search.Sector.ActiveEndpointDeg == 120.0
assert search.Sector.ScanCycle == 1

search.Sector.Reverse()

assert search.Sector.ActiveEndpointDeg == 10.0
assert search.Sector.Direction == -1
assert search.Sector.ScanCycle == 2

request = RevisitRequest(
    TrackId=12,
    RequestedTimeSec=100.0,
    DeadlineTimeSec=100.5,
    PredictedRangeM=8200.0,
    PredictedTrueBearingDeg=68.4,
    Priority=100,
    RevisitIntervalSec=0.5,
)

track = MakeTrackTaskFromRevisit(
    TaskId=2,
    Request=request,
)

assert track.TaskType == RadarTaskType.TRACK
assert track.TrackId == 12
assert track.Pointing.TargetAzimuthDeg == 68.4
assert track.DeadlineTimeSec == 100.5
assert track.IsReady(100.0)
assert not track.IsOverdue(100.25)
assert track.IsOverdue(100.6)

print("Radar task regression test passed")
