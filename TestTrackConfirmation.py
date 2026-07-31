"""Focused regressions for Stage 12E track confirmation and transition slew."""

import time
import unittest

from MissionProfile import TrackUpdatePolicy
from NavigationState import SimulatedNavigationSource
from PointingManager import PointingManager
from RadarExecutor import RadarExecutor
from RadarScheduler import RadarScheduler
from RadarTasks import (
    AngleFrame,
    MakeSearchTask,
    MakeTrackTaskFromRevisit,
)
from RadarTracker import RadarTrack, RadarTracker
from TrackConfirmation import (
    BuildTrackConfirmationRequest,
    FindTaskingTrack,
    TrackConfirmationController,
)
from X660Controller import SimulatedX660Controller


def confirmed_track(track_id=7, range_m=5000.0, azimuth_deg=343.0):
    return RadarTrack(
        TrackId=track_id,
        RangeM=range_m,
        AzimuthDeg=azimuth_deg,
        PredictedRangeM=range_m,
        PredictedAzimuthDeg=azimuth_deg,
        Status="CONFIRMED",
        IsConfirmed=True,
        Hits=5,
        Attempts=5,
        LastUpdateScan=3,
        LastHitScan=3,
    )


def tentative_track(
    track_id=9,
    range_m=5000.0,
    azimuth_deg=343.0,
    hits=1,
    attempts=1,
):
    return RadarTrack(
        TrackId=track_id,
        RangeM=range_m,
        AzimuthDeg=azimuth_deg,
        PredictedRangeM=range_m,
        PredictedAzimuthDeg=azimuth_deg,
        Status="TENTATIVE",
        IsConfirmed=False,
        Hits=hits,
        Attempts=attempts,
        LastUpdateScan=3,
        LastHitScan=3,
    )


class TrackConfirmationTests(unittest.TestCase):
    def setUp(self):
        self.policy = TrackUpdatePolicy(
            Enabled=True,
            GateHalfWidthDeg=5.0,
            MinimumGateDeg=1.0,
            MaximumGateDeg=8.0,
            SlewRateDegSec=40.0,
            NodRateDegSec=20.0,
            PointingToleranceDeg=0.25,
            MaximumUpdateDurationSec=20.0,
            MissPolicy="RETRY_WIDER_THEN_DELETE",
        )

    def test_request_carries_finite_nod_geometry(self):
        request = BuildTrackConfirmationRequest(
            confirmed_track(),
            self.policy,
            now_sec=100.0,
        )
        self.assertEqual(request.TrackId, 7)
        self.assertTrue(request.Metadata["TrackNoddyEnabled"])
        self.assertEqual(request.Metadata["TrackNodPasses"], 3)
        self.assertAlmostEqual(
            request.Metadata["TrackGateHalfWidthDeg"],
            5.0,
        )
        self.assertEqual(
            request.Metadata["TrackMissPolicy"],
            "RETRY_WIDER_THEN_DELETE",
        )

    def test_request_preserves_exact_earth_display_selection(self):
        request = BuildTrackConfirmationRequest(
            tentative_track(track_id=9),
            self.policy,
            now_sec=100.0,
            selected_source="EARTH",
            selected_track_id=41,
            selected_was_tentative=True,
        )

        self.assertEqual(request.TrackId, 9)
        self.assertEqual(
            request.Metadata["TrackSelectedSource"],
            "EARTH",
        )
        self.assertEqual(
            request.Metadata["TrackSelectedTrackId"],
            41,
        )
        self.assertTrue(
            request.Metadata["TrackSelectedWasTentative"]
        )

    def test_operator_request_queues_only_confirmed_track(self):
        search = MakeSearchTask(
            TaskId=1,
            SectorStartDeg=0.0,
            SectorStopDeg=90.0,
            ScanRateDegPerSec=10.0,
        )
        scheduler = RadarScheduler(search)
        controller = TrackConfirmationController()
        self.assertTrue(controller.HasPendingOperatorRequest({
            "TrackConfirmCommandId": 1,
        }))
        result = controller.ConsumeOperatorRequest(
            {
                "TrackConfirmCommandId": 1,
                "TrackConfirmTrackId": 7,
            },
            [confirmed_track()],
            scheduler,
            self.policy,
            now_sec=100.0,
        )
        self.assertTrue(result["Applied"])
        self.assertFalse(controller.HasPendingOperatorRequest({
            "TrackConfirmCommandId": 1,
        }))
        self.assertEqual(
            scheduler.GetQueuedTrackTasks()[0].TrackId,
            7,
        )
        self.assertTrue(
            scheduler.HasReadyOrActiveTrackTask(100.0)
        )
        selected = scheduler.GetNextTask(100.0)
        self.assertEqual(selected.TrackId, 7)
        self.assertTrue(scheduler.IsTrackTaskActive())
        scheduler.CompleteActiveTask(100.1)
        self.assertFalse(
            scheduler.HasReadyOrActiveTrackTask(100.1)
        )
        unchanged = controller.ConsumeOperatorRequest(
            {
                "TrackConfirmCommandId": 1,
                "TrackConfirmTrackId": 7,
            },
            [confirmed_track()],
            scheduler,
            self.policy,
            now_sec=101.0,
        )
        self.assertFalse(unchanged["Changed"])

    def test_operator_confirmation_upgrades_legacy_two_degree_gate(self):
        search = MakeSearchTask(
            TaskId=1,
            SectorStartDeg=0.0,
            SectorStopDeg=90.0,
            ScanRateDegPerSec=10.0,
        )
        scheduler = RadarScheduler(search)
        controller = TrackConfirmationController(
            minimum_operator_gate_half_width_deg=5.0,
        )
        result = controller.ConsumeOperatorRequest(
            {
                "TrackConfirmCommandId": 1,
                "TrackConfirmTrackId": 7,
            },
            [confirmed_track()],
            scheduler,
            TrackUpdatePolicy(
                Enabled=True,
                GateHalfWidthDeg=2.0,
                MinimumGateDeg=1.0,
                MaximumGateDeg=8.0,
            ),
            now_sec=100.0,
        )
        self.assertTrue(result["Applied"])
        task = scheduler.GetQueuedTrackTasks()[0]
        self.assertAlmostEqual(
            task.Metadata["TrackGateHalfWidthDeg"],
            5.0,
        )
        self.assertIn("+/-5 deg", result["Message"])

    def test_operator_request_queues_tentative_track_for_one_check(self):
        search = MakeSearchTask(
            TaskId=1,
            SectorStartDeg=0.0,
            SectorStopDeg=90.0,
            ScanRateDegPerSec=10.0,
        )
        scheduler = RadarScheduler(search)
        controller = TrackConfirmationController()
        result = controller.ConsumeOperatorRequest(
            {
                "TrackConfirmCommandId": 1,
                "TrackConfirmTrackId": 9,
            },
            [tentative_track()],
            scheduler,
            self.policy,
            now_sec=100.0,
        )
        self.assertTrue(result["Applied"])
        self.assertTrue(result["TrackWasTentative"])
        self.assertIn("initiation check queued", result["Message"])
        task = scheduler.GetQueuedTrackTasks()[0]
        self.assertEqual(task.TrackId, 9)
        self.assertTrue(task.Metadata["TrackWasTentativeAtRequest"])

    def test_earth_tentative_selection_resolves_to_legacy_tasking_track(self):
        legacy = tentative_track(
            track_id=9,
            range_m=5020.0,
            azimuth_deg=342.5,
        )
        resolved = FindTaskingTrack(
            [legacy],
            41,
            selected_source="EARTH",
            selected_range_m=5000.0,
            selected_azimuth_deg=343.0,
        )
        self.assertIs(resolved, legacy)

    def test_earth_tentative_selection_cannot_resolve_to_confirmed_neighbour(self):
        confirmed = confirmed_track(
            track_id=7,
            range_m=5000.0,
            azimuth_deg=343.0,
        )
        tentative = tentative_track(
            track_id=9,
            range_m=5030.0,
            azimuth_deg=343.4,
        )
        resolved = FindTaskingTrack(
            [confirmed, tentative],
            41,
            selected_source="EARTH",
            selected_range_m=5000.0,
            selected_azimuth_deg=343.0,
            selected_was_tentative=True,
        )
        self.assertIs(resolved, tentative)

    def test_earth_tentative_request_queues_tentative_legacy_neighbour(self):
        search = MakeSearchTask(
            TaskId=1,
            SectorStartDeg=0.0,
            SectorStopDeg=90.0,
            ScanRateDegPerSec=10.0,
        )
        scheduler = RadarScheduler(search)
        confirmed = confirmed_track(
            track_id=7,
            range_m=5000.0,
            azimuth_deg=343.0,
        )
        tentative = tentative_track(
            track_id=9,
            range_m=5030.0,
            azimuth_deg=343.4,
        )
        result = TrackConfirmationController().ConsumeOperatorRequest(
            {
                "TrackConfirmCommandId": 1,
                "TrackConfirmTrackId": 41,
                "TrackConfirmSource": "EARTH",
                "TrackConfirmRangeM": 5000.0,
                "TrackConfirmAzimuthDeg": 343.0,
                "TrackConfirmWasTentative": True,
            },
            [confirmed, tentative],
            scheduler,
            self.policy,
            now_sec=100.0,
        )
        self.assertTrue(result["Applied"])
        self.assertTrue(result["TrackWasTentative"])
        self.assertEqual(result["TrackId"], 9)
        task = scheduler.GetQueuedTrackTasks()[0]
        self.assertEqual(task.TrackId, 9)
        self.assertEqual(
            task.Metadata["TrackSelectedSource"],
            "EARTH",
        )
        self.assertEqual(
            task.Metadata["TrackSelectedTrackId"],
            41,
        )
        self.assertTrue(
            task.Metadata["TrackSelectedWasTentative"]
        )
        self.assertTrue(task.Metadata["TrackWasTentativeAtRequest"])

    def test_directed_hit_updates_nominated_track_and_merges_companion(self):
        tracker = RadarTracker({
            "AssociationRangeGateM": 300.0,
            "AssociationAzimuthGateDeg": 6.0,
            "DuplicateTrackRangeGateM": 200.0,
            "DuplicateTrackAzimuthGateDeg": 5.0,
        })
        nominated = confirmed_track(7, 5000.0, 343.0)
        companion = confirmed_track(8, 5050.0, 346.0)
        companion.Misses = 1
        tracker.Tracks = [nominated, companion]
        outcome = tracker.ApplyDirectedUpdate(
            7,
            [{
                "RangeM": 5010.0,
                "AzimuthDeg": 343.4,
                "SnrDb": 30.0,
            }],
            coverage_valid=True,
            miss_policy=self.policy.MissPolicy,
        )
        self.assertTrue(outcome["Hit"])
        self.assertEqual(outcome["DuplicatesMerged"], 1)
        self.assertEqual([track.TrackId for track in tracker.Tracks], [7])
        self.assertEqual(tracker.Tracks[0].DirectedUpdateMisses, 0)

    def test_first_complete_miss_retries_wider_then_second_deletes(self):
        tracker = RadarTracker({
            "DirectedTrackDeleteAfterMisses": 2,
        })
        track = confirmed_track()
        tracker.Tracks = [track]
        first = tracker.ApplyDirectedUpdate(
            7,
            [],
            coverage_valid=True,
            miss_policy=self.policy.MissPolicy,
        )
        self.assertFalse(first["Deleted"])

        search = MakeSearchTask(
            TaskId=1,
            SectorStartDeg=0.0,
            SectorStopDeg=90.0,
            ScanRateDegPerSec=10.0,
        )
        scheduler = RadarScheduler(search)
        retry = TrackConfirmationController().QueueRetryIfRequired(
            first,
            tracker.GetTracks(),
            scheduler,
            self.policy,
            now_sec=101.0,
        )
        self.assertIsNotNone(retry)
        self.assertAlmostEqual(retry["GateHalfWidthDeg"], 8.0)

        second = tracker.ApplyDirectedUpdate(
            7,
            [],
            coverage_valid=True,
            miss_policy=self.policy.MissPolicy,
        )
        self.assertTrue(second["Deleted"])
        self.assertEqual(tracker.GetTracks(), [])

    def test_incomplete_coverage_never_counts_as_a_miss(self):
        tracker = RadarTracker()
        track = confirmed_track()
        tracker.Tracks = [track]
        result = tracker.ApplyDirectedUpdate(
            7,
            [],
            coverage_valid=False,
            miss_policy="DELETE",
        )
        self.assertFalse(result["Deleted"])
        self.assertEqual(track.DirectedUpdateMisses, 0)
        self.assertEqual(track.Misses, 0)

    def test_tentative_three_pass_hit_counts_as_one_and_promotes(self):
        tracker = RadarTracker({
            "InitiationWindow": 3,
            "InitiationRequiredHits": 2,
        })
        track = tentative_track(hits=1, attempts=1)
        tracker.Tracks = [track]
        outcome = tracker.ApplyDirectedUpdate(
            9,
            [{
                "RangeM": 5010.0,
                "AzimuthDeg": 343.4,
                "SnrDb": 30.0,
            }],
            coverage_valid=True,
        )
        self.assertTrue(outcome["Hit"])
        self.assertTrue(outcome["Promoted"])
        self.assertEqual(outcome["Hits"], 2)
        self.assertEqual(outcome["Attempts"], 2)
        self.assertEqual(track.Status, "CONFIRMED")
        self.assertTrue(track.IsConfirmed)

    def test_tentative_complete_miss_is_one_retained_opportunity(self):
        tracker = RadarTracker({
            "InitiationWindow": 3,
            "InitiationRequiredHits": 2,
        })
        track = tentative_track(hits=1, attempts=1)
        tracker.Tracks = [track]
        outcome = tracker.ApplyDirectedUpdate(
            9,
            [],
            coverage_valid=True,
        )
        self.assertFalse(outcome["Hit"])
        self.assertFalse(outcome["Promoted"])
        self.assertFalse(outcome["Deleted"])
        self.assertEqual(outcome["Hits"], 1)
        self.assertEqual(outcome["Attempts"], 2)
        self.assertEqual(tracker.GetTracks(), [track])

    def test_tentative_final_miss_obeys_two_of_three(self):
        retained_tracker = RadarTracker()
        retained = tentative_track(hits=2, attempts=2)
        retained_tracker.Tracks = [retained]
        retained_outcome = retained_tracker.ApplyDirectedUpdate(
            9,
            [],
            coverage_valid=True,
        )
        self.assertTrue(retained_outcome["Promoted"])
        self.assertFalse(retained_outcome["Deleted"])
        self.assertEqual(retained.Status, "CONFIRMED")

        deleted_tracker = RadarTracker()
        deleted = tentative_track(hits=1, attempts=2)
        deleted_tracker.Tracks = [deleted]
        deleted_outcome = deleted_tracker.ApplyDirectedUpdate(
            9,
            [],
            coverage_valid=True,
        )
        self.assertFalse(deleted_outcome["Promoted"])
        self.assertTrue(deleted_outcome["Deleted"])
        self.assertEqual(deleted_tracker.GetTracks(), [])

    def test_incomplete_tentative_nod_does_not_consume_opportunity(self):
        tracker = RadarTracker()
        track = tentative_track(hits=1, attempts=1)
        tracker.Tracks = [track]
        outcome = tracker.ApplyDirectedUpdate(
            9,
            [],
            coverage_valid=False,
        )
        self.assertFalse(outcome["Deleted"])
        self.assertEqual(track.Hits, 1)
        self.assertEqual(track.Attempts, 1)
        self.assertEqual(track.Misses, 0)

    def test_tentative_miss_never_queues_confirmed_track_retry(self):
        tracker = RadarTracker()
        track = tentative_track(hits=1, attempts=1)
        tracker.Tracks = [track]
        outcome = tracker.ApplyDirectedUpdate(
            9,
            [],
            coverage_valid=True,
        )
        search = MakeSearchTask(
            TaskId=1,
            SectorStartDeg=0.0,
            SectorStopDeg=90.0,
            ScanRateDegPerSec=10.0,
        )
        scheduler = RadarScheduler(search)
        retry = TrackConfirmationController().QueueRetryIfRequired(
            outcome,
            tracker.GetTracks(),
            scheduler,
            self.policy,
            now_sec=101.0,
        )
        self.assertIsNone(retry)
        self.assertEqual(scheduler.GetQueuedTrackTasks(), [])

    def test_all_three_nod_pass_plot_evidence_is_retained(self):
        controller = TrackConfirmationController()
        controller.AccumulateTaskPlots(12, [{"PlotId": 1}])
        controller.AccumulateTaskPlots(12, [{"PlotId": 2}])
        plots = controller.FinalizeTaskPlots(12, [{"PlotId": 3}])
        self.assertEqual(
            [plot["PlotId"] for plot in plots],
            [1, 2, 3],
        )
        self.assertEqual(controller.FinalizeTaskPlots(12), [])


class PointingTransitionTests(unittest.TestCase):
    def setUp(self):
        self.x660 = SimulatedX660Controller(
            InitialAzimuthDeg=200.0,
            MaxPanRateDegPerSec=80.0,
            PositionToleranceDeg=0.10,
        )
        self.x660.Open()
        self.navigation = SimulatedNavigationSource(
            initial_heading_deg=0.0,
        )
        self.manager = PointingManager(
            self.x660,
            endpoint_margin_deg=0.25,
            position_tolerance_deg=0.25,
            transition_slew_rate_deg_per_sec=40.0,
        )

    def tearDown(self):
        self.manager.Stop()
        self.x660.Close()

    def _wait_for(self, predicate, timeout_sec=3.0):
        deadline = time.time() + timeout_sec
        state = None
        while time.time() < deadline:
            state = self.manager.Update(self.navigation.get_pose())
            if predicate(state):
                return state
            time.sleep(0.005)
        self.fail(
            f"pointing condition timed out; phase="
            f"{getattr(state, 'PointingPhase', None)}"
        )

    def test_sector_transition_slews_fast_then_scans_slow(self):
        search = MakeSearchTask(
            TaskId=1,
            SectorStartDeg=40.0,
            SectorStopDeg=60.0,
            ScanRateDegPerSec=10.0,
            SectorFrame=AngleFrame.TRUE,
        )
        search.Pointing.Metadata["SlewToSectorStart"] = True
        search.Pointing.Metadata[
            "TransitionSlewRateDegPerSec"
        ] = 40.0
        self.manager.ActivateTask(search, self.navigation.get_pose())
        self.assertEqual(self.manager.PointingPhase, "SLEW_TO_START")
        self.assertAlmostEqual(self.x660.GotoRateDegPerSec, 40.0)
        self.assertAlmostEqual(self.x660.State.CommandedAzimuthDeg, 40.0)

        state = self._wait_for(
            lambda item: item.PointingPhase == "SCANNING",
            timeout_sec=5.0,
        )
        self.assertTrue(state.Ready)
        self.assertAlmostEqual(self.x660.CommandedRateDegPerSec, 10.0)

    def test_track_confirmation_completes_three_gate_passes(self):
        request = BuildTrackConfirmationRequest(
            confirmed_track(7, 5000.0, 210.0),
            TrackUpdatePolicy(
                Enabled=True,
                GateHalfWidthDeg=5.0,
                SlewRateDegSec=60.0,
                NodRateDegSec=40.0,
                PointingToleranceDeg=0.25,
                MaximumUpdateDurationSec=10.0,
            ),
            now_sec=time.time(),
        )
        task = MakeTrackTaskFromRevisit(2, request)
        self.manager.ActivateTask(task, self.navigation.get_pose())
        self.assertEqual(
            self.manager.PointingPhase,
            "TRACK_SLEW_TO_GATE",
        )
        state = self._wait_for(
            lambda item: item.TrackUpdateComplete,
            timeout_sec=3.0,
        )
        self.assertTrue(state.TrackUpdateCoverageValid)
        self.assertEqual(state.TrackUpdatePass, 3)
        self.assertEqual(state.TrackUpdatePasses, 3)

    def test_track_gate_position_command_tracks_platform_heading(self):
        navigation = SimulatedNavigationSource(
            initial_heading_deg=0.0,
        )
        request = BuildTrackConfirmationRequest(
            confirmed_track(7, 5000.0, 210.0),
            TrackUpdatePolicy(
                Enabled=True,
                GateHalfWidthDeg=5.0,
                SlewRateDegSec=20.0,
                NodRateDegSec=5.0,
                PointingToleranceDeg=0.25,
                MaximumUpdateDurationSec=20.0,
            ),
            now_sec=time.time(),
        )
        task = MakeTrackTaskFromRevisit(2, request)
        self.manager.ActivateTask(task, navigation.get_pose())
        initial_command = self.manager.LastCommandedRelativeDeg
        self.assertAlmostEqual(initial_command, 205.0)

        # Reproduce the live circular-route failure: the X6-60 reaches the
        # originally commanded relative angle, but the platform heading has
        # changed before start-gate acquisition completes.
        navigation._heading = 2.0
        self.x660.UnwrappedAzimuthDeg = float(initial_command)
        self.x660.TargetUnwrappedAzimuthDeg = float(initial_command)
        self.x660.Stop()
        state = self.manager.Update(navigation.get_pose())

        self.assertFalse(state.Ready)
        self.assertEqual(state.PointingPhase, "TRACK_SLEW_TO_GATE")
        self.assertAlmostEqual(
            self.manager.LastCommandedRelativeDeg,
            203.0,
        )
        self.assertAlmostEqual(
            self.x660.State.CommandedAzimuthDeg,
            203.0,
        )

    def test_real_rate_nod_completes_while_platform_turns(self):
        x660 = SimulatedX660Controller(
            InitialAzimuthDeg=160.0,
            MaxPanRateDegPerSec=20.0,
            PositionToleranceDeg=0.75,
        )
        navigation = SimulatedNavigationSource(
            initial_heading_deg=0.0,
            turn_rate_deg_per_sec=1.0,
        )
        manager = PointingManager(
            x660,
            position_tolerance_deg=0.75,
        )
        request = BuildTrackConfirmationRequest(
            confirmed_track(7, 5000.0, 210.0),
            TrackUpdatePolicy(
                Enabled=True,
                GateHalfWidthDeg=5.0,
                SlewRateDegSec=20.0,
                NodRateDegSec=5.0,
                PointingToleranceDeg=0.25,
                MaximumUpdateDurationSec=20.0,
            ),
            now_sec=time.time(),
        )
        task = MakeTrackTaskFromRevisit(2, request)
        manager.ActivateTask(task, navigation.get_pose())

        deadline = time.time() + 12.0
        state = None
        while time.time() < deadline:
            state = manager.Update(navigation.get_pose())
            if state.TrackUpdateComplete:
                break
            time.sleep(0.01)

        manager.Stop()
        self.assertIsNotNone(state)
        self.assertTrue(state.TrackUpdateComplete)
        self.assertTrue(state.TrackUpdateCoverageValid)
        self.assertEqual(state.TrackUpdatePass, 3)
        self.assertEqual(state.TrackUpdatePasses, 3)

    def test_executor_reacquires_track_if_search_overrides_pointing(self):
        search = MakeSearchTask(
            TaskId=1,
            SectorStartDeg=40.0,
            SectorStopDeg=60.0,
            ScanRateDegPerSec=10.0,
            SectorFrame=AngleFrame.TRUE,
        )
        request = BuildTrackConfirmationRequest(
            confirmed_track(7, 5000.0, 210.0),
            TrackUpdatePolicy(
                Enabled=True,
                GateHalfWidthDeg=5.0,
                SlewRateDegSec=60.0,
                NodRateDegSec=40.0,
                PointingToleranceDeg=0.25,
                MaximumUpdateDurationSec=10.0,
            ),
            now_sec=time.time(),
        )
        track_task = MakeTrackTaskFromRevisit(2, request)
        executor = RadarExecutor(
            source=object(),
            pointing_manager=self.manager,
            config={},
        )
        executor._active_task_id = int(track_task.TaskId)

        self.manager.ActivateTask(search, self.navigation.get_pose())
        self.assertEqual(self.manager.ActiveTask.TaskId, search.TaskId)
        result = executor.ExecuteTaskStep(
            track_task,
            self.navigation.get_pose(),
        )
        self.assertEqual(self.manager.ActiveTask.TaskId, track_task.TaskId)
        self.assertEqual(
            self.manager.PointingPhase,
            "TRACK_SLEW_TO_GATE",
        )
        self.assertTrue(result.WaitingForPointing)


if __name__ == "__main__":
    unittest.main()
