"""
Regression tests for RadarScheduler.py.
"""

from RadarScheduler import RadarScheduler
from RadarTasks import (
    AngleFrame,
    MakeSearchTask,
    RadarTaskType,
    RevisitRequest,
    TaskStatus,
)


search = MakeSearchTask(
    TaskId=1,
    SectorStartDeg=10.0,
    SectorStopDeg=120.0,
    ScanRateDegPerSec=14.0,
    SectorFrame=AngleFrame.PLATFORM,
)

scheduler = RadarScheduler(search_task=search)

# With no track work, scheduler selects persistent search.
task = scheduler.GetNextTask(current_time_sec=100.0)
assert task.TaskType == RadarTaskType.SEARCH
assert task.Status == TaskStatus.ACTIVE

# One search dwell completes, but search remains available.
scheduler.CompleteActiveTask(current_time_sec=100.1)
assert search.Status == TaskStatus.QUEUED

# Add two revisits. Track 12 has the earlier deadline.
request_12 = RevisitRequest(
    TrackId=12,
    RequestedTimeSec=100.0,
    DeadlineTimeSec=100.5,
    PredictedRangeM=8000.0,
    PredictedTrueBearingDeg=68.4,
    Priority=100,
)

request_7 = RevisitRequest(
    TrackId=7,
    RequestedTimeSec=100.0,
    DeadlineTimeSec=101.0,
    PredictedRangeM=5000.0,
    PredictedTrueBearingDeg=40.0,
    Priority=100,
)

task_12 = scheduler.SubmitRevisitRequest(request_12)
task_7 = scheduler.SubmitRevisitRequest(request_7)

assert len(scheduler.GetQueuedTrackTasks()) == 2

selected = scheduler.GetNextTask(current_time_sec=100.25)
assert selected.TaskType == RadarTaskType.TRACK
assert selected.TrackId == 12
assert scheduler.SearchPaused

# Completing Track 12 allows Track 7 to run before search resumes.
scheduler.CompleteActiveTask(current_time_sec=100.3)

selected = scheduler.GetNextTask(current_time_sec=100.31)
assert selected.TrackId == 7

scheduler.CompleteActiveTask(current_time_sec=100.4)

# No more track tasks, so persistent search resumes.
selected = scheduler.GetNextTask(current_time_sec=100.41)
assert selected.TaskType == RadarTaskType.SEARCH
assert not scheduler.SearchPaused

scheduler.CompleteActiveTask(current_time_sec=100.5)

# Duplicate queued revisit updates rather than duplicates.
request_20_a = RevisitRequest(
    TrackId=20,
    RequestedTimeSec=102.0,
    DeadlineTimeSec=103.0,
    PredictedRangeM=6000.0,
    PredictedTrueBearingDeg=20.0,
    Priority=80,
)

request_20_b = RevisitRequest(
    TrackId=20,
    RequestedTimeSec=101.5,
    DeadlineTimeSec=102.5,
    PredictedRangeM=6100.0,
    PredictedTrueBearingDeg=22.0,
    Priority=90,
)

first = scheduler.SubmitRevisitRequest(request_20_a)
second = scheduler.SubmitRevisitRequest(request_20_b)

assert first is second
assert len(scheduler.GetQueuedTrackTasks()) == 1
assert second.Priority == 90
assert second.DeadlineTimeSec == 102.5
assert second.Pointing.TargetAzimuthDeg == 22.0

# Not ready before EarliestStartTimeSec.
selected = scheduler.GetNextTask(current_time_sec=101.0)
assert selected.TaskType == RadarTaskType.SEARCH
scheduler.CompleteActiveTask(current_time_sec=101.1)

# Ready afterwards.
selected = scheduler.GetNextTask(current_time_sec=101.6)
assert selected.TaskType == RadarTaskType.TRACK
assert selected.TrackId == 20

status = scheduler.GetStatus(current_time_sec=101.6)
assert status.ActiveTrackId == 20

scheduler.CompleteActiveTask(current_time_sec=101.7)

assert len(scheduler.GetQueuedTrackTasks()) == 0

print("RadarScheduler regression test passed")
