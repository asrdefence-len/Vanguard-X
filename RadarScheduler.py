"""
===============================================================================
Vanguard X Radar Scheduler
RadarScheduler.py
===============================================================================

Purpose
-------
Chooses the next high-level radar task.

The scheduler:
    - owns one persistent SEARCH task
    - accepts tracker RevisitRequest objects
    - converts revisits into TRACK tasks
    - prioritises overdue and near-deadline track work over search
    - pauses search while a track task is active
    - resumes the same search task after track completion
    - prevents duplicate queued track tasks for the same TrackId

The scheduler does not:
    - move the X6-60
    - build DwellPlan objects
    - process IQ
    - update tracks
    - control the Ettus or TRM

Those responsibilities remain in PointingManager, DwellPlanner/RadarExecutor,
RadarProcessor and the hardware interfaces.
===============================================================================
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional
import time

from RadarTasks import (
    MakeTrackTaskFromRevisit,
    RadarTask,
    RadarTaskType,
    RevisitRequest,
    SearchTask,
    TaskStatus,
    TrackTask,
)


@dataclass
class SchedulerStatus:
    CurrentTimeSec: float
    ActiveTaskId: Optional[int]
    ActiveTaskType: Optional[str]
    ActiveTrackId: Optional[int]
    SearchTaskId: Optional[int]
    SearchPaused: bool
    NumQueuedTrackTasks: int
    NextQueuedTrackId: Optional[int]


class RadarScheduler:
    """
    First Vanguard multifunction-radar scheduler.

    Scheduling policy
    -----------------
    1. Continue an already-active task until it is completed or failed.
    2. Otherwise choose the ready TRACK task with the strongest urgency.
    3. If no TRACK task is ready, run or resume persistent SEARCH.
    """

    def __init__(
        self,
        search_task: SearchTask,
        track_deadline_weight: float = 1000.0,
        debug: bool = False,
    ):
        if search_task.TaskType != RadarTaskType.SEARCH:
            raise ValueError("RadarScheduler requires a SEARCH task")

        self.SearchTask = search_task
        self.TrackDeadlineWeight = float(track_deadline_weight)
        self.Debug = bool(debug)

        self.ActiveTask: Optional[RadarTask] = None
        self.SearchPaused = False

        self._queued_track_tasks: List[TrackTask] = []
        self._track_task_by_track_id: Dict[int, TrackTask] = {}

        self._next_task_id = max(1, int(search_task.TaskId) + 1)

        self.CompletedTaskIds: List[int] = []
        self.FailedTaskIds: List[int] = []

    # ------------------------------------------------------------------
    # Tracker input
    # ------------------------------------------------------------------

    def SubmitRevisitRequest(self, request: RevisitRequest) -> TrackTask:
        """
        Add or update a TRACK task for one track.

        If a queued task for the same TrackId already exists, it is updated
        rather than duplicated. An active task for the same TrackId is left
        active; the new request becomes the next queued revisit only when its
        requested time is later than the active task's execution window.
        """
        track_id = int(request.TrackId)

        existing = self._track_task_by_track_id.get(track_id)

        if existing is not None and existing.Status in (
            TaskStatus.QUEUED,
            TaskStatus.PAUSED,
        ):
            self._update_track_task_from_request(existing, request)

            if self.Debug:
                print(
                    "Scheduler updated queued revisit: "
                    f"track={track_id} deadline={existing.DeadlineTimeSec:.3f}"
                )

            return existing

        task = MakeTrackTaskFromRevisit(
            TaskId=self._allocate_task_id(),
            Request=request,
        )

        self._queued_track_tasks.append(task)
        self._track_task_by_track_id[track_id] = task

        if self.Debug:
            print(
                "Scheduler queued TRACK task: "
                f"task={task.TaskId} track={track_id} "
                f"deadline={task.DeadlineTimeSec:.3f}"
            )

        return task

    def SubmitRevisitRequests(
        self,
        requests: List[RevisitRequest],
    ) -> List[TrackTask]:
        return [self.SubmitRevisitRequest(request) for request in requests]

    @staticmethod
    def _update_track_task_from_request(
        task: TrackTask,
        request: RevisitRequest,
    ) -> None:
        task.Priority = max(int(task.Priority), int(request.Priority))

        task.EarliestStartTimeSec = min(
            float(task.EarliestStartTimeSec),
            float(request.RequestedTimeSec),
        )

        if task.DeadlineTimeSec is None:
            task.DeadlineTimeSec = float(request.DeadlineTimeSec)
        else:
            task.DeadlineTimeSec = min(
                float(task.DeadlineTimeSec),
                float(request.DeadlineTimeSec),
            )

        task.PredictedRangeM = float(request.PredictedRangeM)
        task.PredictedTrueBearingDeg = float(
            request.PredictedTrueBearingDeg
        )
        task.PredictedElevationDeg = float(
            request.PredictedElevationDeg
        )
        task.PredictionTimeSec = float(request.RequestedTimeSec)
        task.RevisitIntervalSec = float(request.RevisitIntervalSec)
        task.WaveformProfileId = str(request.WaveformProfileId)

        task.Pointing.TargetAzimuthDeg = float(
            request.PredictedTrueBearingDeg
        )
        task.Pointing.TargetElevationDeg = float(
            request.PredictedElevationDeg
        )

        task.Metadata.update(dict(request.Metadata))

    # ------------------------------------------------------------------
    # Scheduling
    # ------------------------------------------------------------------

    def GetNextTask(
        self,
        current_time_sec: Optional[float] = None,
    ) -> Optional[RadarTask]:
        """
        Return the task that should own radar execution now.

        Calling this repeatedly does not create new tasks. The currently active
        task remains active until CompleteActiveTask() or FailActiveTask().
        """
        now = time.time() if current_time_sec is None else float(current_time_sec)

        if self.ActiveTask is not None:
            return self.ActiveTask

        track_task = self._select_best_ready_track_task(now)

        if track_task is not None:
            self._activate(track_task)

            if self.SearchTask.Status == TaskStatus.ACTIVE:
                self.SearchTask.Status = TaskStatus.PAUSED

            self.SearchPaused = True
            return track_task

        if self.SearchTask.Status in (
            TaskStatus.QUEUED,
            TaskStatus.PAUSED,
            TaskStatus.ACTIVE,
        ):
            self._activate(self.SearchTask)
            self.SearchPaused = False
            return self.SearchTask

        return None

    def _select_best_ready_track_task(
        self,
        now: float,
    ) -> Optional[TrackTask]:
        ready = [
            task
            for task in self._queued_track_tasks
            if task.IsReady(now)
            and task.Status in (TaskStatus.QUEUED, TaskStatus.PAUSED)
        ]

        if not ready:
            return None

        return max(
            ready,
            key=lambda task: self._track_task_score(task, now),
        )

    def _track_task_score(
        self,
        task: TrackTask,
        now: float,
    ) -> float:
        """
        Higher score wins.

        Base priority is augmented by deadline urgency. Overdue work receives
        a very large boost. This is deliberately simple and deterministic.
        """
        score = float(task.Priority)

        if task.DeadlineTimeSec is None:
            return score

        slack = float(task.DeadlineTimeSec) - now

        if slack <= 0.0:
            return score + self.TrackDeadlineWeight + abs(slack)

        return score + 1.0 / max(slack, 1e-6)

    def _activate(self, task: RadarTask) -> None:
        task.Status = TaskStatus.ACTIVE
        self.ActiveTask = task

        if self.Debug:
            if isinstance(task, TrackTask):
                detail = f" track={task.TrackId}"
            else:
                detail = ""

            print(
                f"Scheduler activated {task.TaskType.value} "
                f"task={task.TaskId}{detail}"
            )

    # ------------------------------------------------------------------
    # Task completion
    # ------------------------------------------------------------------

    def CompleteActiveTask(
        self,
        current_time_sec: Optional[float] = None,
    ) -> Optional[RadarTask]:
        """
        Mark the active task complete.

        TRACK tasks are one-shot and removed from the queue.

        SEARCH is persistent. Completing a search dwell releases ownership but
        returns the search task to QUEUED so it can be selected again.
        """
        if self.ActiveTask is None:
            return None

        now = time.time() if current_time_sec is None else float(current_time_sec)
        task = self.ActiveTask
        self.ActiveTask = None

        if task.TaskType == RadarTaskType.TRACK:
            task.Status = TaskStatus.COMPLETE
            self.CompletedTaskIds.append(int(task.TaskId))
            self._remove_track_task(task)

            self.SearchTask.Status = TaskStatus.PAUSED
            self.SearchPaused = True

            if self.Debug:
                print(
                    "Scheduler completed TRACK task: "
                    f"task={task.TaskId} track={task.TrackId} time={now:.3f}"
                )

        elif task.TaskType == RadarTaskType.SEARCH:
            task.Status = TaskStatus.QUEUED
            self.SearchPaused = False

        else:
            task.Status = TaskStatus.COMPLETE
            self.CompletedTaskIds.append(int(task.TaskId))

        return task

    def FailActiveTask(
        self,
        reason: str = "",
    ) -> Optional[RadarTask]:
        if self.ActiveTask is None:
            return None

        task = self.ActiveTask
        self.ActiveTask = None
        task.Status = TaskStatus.FAILED
        task.Metadata["FailureReason"] = str(reason)
        self.FailedTaskIds.append(int(task.TaskId))

        if task.TaskType == RadarTaskType.TRACK:
            self._remove_track_task(task)
            self.SearchTask.Status = TaskStatus.PAUSED
            self.SearchPaused = True

        if self.Debug:
            print(
                f"Scheduler failed task={task.TaskId}: {reason}"
            )

        return task

    def CancelTrackTask(self, track_id: int) -> bool:
        task = self._track_task_by_track_id.get(int(track_id))

        if task is None:
            return False

        if task is self.ActiveTask:
            self.ActiveTask = None

        task.Status = TaskStatus.CANCELLED
        self._remove_track_task(task)
        return True

    def _remove_track_task(self, task: TrackTask) -> None:
        self._queued_track_tasks = [
            queued
            for queued in self._queued_track_tasks
            if queued is not task
        ]

        current = self._track_task_by_track_id.get(int(task.TrackId))
        if current is task:
            del self._track_task_by_track_id[int(task.TrackId)]

    # ------------------------------------------------------------------
    # Status / inspection
    # ------------------------------------------------------------------

    def GetQueuedTrackTasks(self) -> List[TrackTask]:
        return list(self._queued_track_tasks)

    def GetStatus(
        self,
        current_time_sec: Optional[float] = None,
    ) -> SchedulerStatus:
        now = time.time() if current_time_sec is None else float(current_time_sec)

        queued_ready = sorted(
            [
                task
                for task in self._queued_track_tasks
                if task.Status in (TaskStatus.QUEUED, TaskStatus.PAUSED)
            ],
            key=lambda task: self._track_task_score(task, now),
            reverse=True,
        )

        active_task_id = None
        active_task_type = None
        active_track_id = None

        if self.ActiveTask is not None:
            active_task_id = int(self.ActiveTask.TaskId)
            active_task_type = self.ActiveTask.TaskType.value

            if isinstance(self.ActiveTask, TrackTask):
                active_track_id = int(self.ActiveTask.TrackId)

        next_track_id = (
            None if not queued_ready else int(queued_ready[0].TrackId)
        )

        return SchedulerStatus(
            CurrentTimeSec=now,
            ActiveTaskId=active_task_id,
            ActiveTaskType=active_task_type,
            ActiveTrackId=active_track_id,
            SearchTaskId=int(self.SearchTask.TaskId),
            SearchPaused=bool(self.SearchPaused),
            NumQueuedTrackTasks=len(queued_ready),
            NextQueuedTrackId=next_track_id,
        )

    def _allocate_task_id(self) -> int:
        task_id = self._next_task_id
        self._next_task_id += 1
        return task_id
