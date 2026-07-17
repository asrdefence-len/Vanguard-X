"""
End-to-end simulator regression for the Vanguard scheduler architecture.

This test proves:
  SEARCH -> synthetic RevisitRequest -> TRACK -> resume SEARCH

It uses the existing simulated PTZ, SimulatedSource, RadarProcessor, CFAR and
tracker. No Ettus, TRM or real PTZ hardware is accessed.
"""

import time

from CfarDetector import CfarDetector
from NavigationState import SimulatedNavigationSource
from PointingManager import PointingManager
from PTZController import SimulatedPTZController
from RadarExecutor import RadarExecutor
from RadarProcessor import RadarProcessor
from RadarScheduler import RadarScheduler
from RadarTasks import AngleFrame, MakeSearchTask, RadarTaskType, RevisitRequest
from RadarTracker import RadarTracker
from SimulatedSource import SimulatedSource
from WaveformLibrary import WaveformLibrary


def main():
    config = {
        "SampleRate": 2.0e6,
	"EttusSampleRateHz":2.0e6,
        "NumSamples": 512,
        "NumPulses": 16,
        "PRI": 1.0e-3,
        "SearchWaveformId": "Barker13",
        "TrackWaveformId": "Frank10",
        "TrackNumPulses": 16,
        "TrackPRI": 1.0e-3,
        "RfFrequency": 9.4e9,
        "TransmitPowerW": 10.0,
        "AntennaGainDb": 20.0,
        "SystemLossDb": 6.0,
        "NoisePowerW": 1e-13,
        "TargetRangeM": 3000.0,
        "TargetVelocityMps": 0.0,
        "TargetRcsSqm": 1000.0,
        "TRMRxAttenuationDb": 0.0,
        "TRMTxAttenuationDb": 31.5,
        "TrainingCellsRange": 6,
        "TrainingCellsDoppler": 2,
        "GuardCellsRange": 2,
        "GuardCellsDoppler": 1,
        "CfarThresholdDb": 12.0,
        "MaxDetections": 20,
        "MinRangeM": 100.0,
        "MaxRangeM": 10000.0,
        "RangeBinM": 299792458.0 / (2.0 * 2.0e6),
        "ReturnBlobsForDebug": False,
    }

    waveforms = WaveformLibrary(config)
    waveforms.LoadDefaultWaveforms()

    source = SimulatedSource(config, waveforms)
    source.Initialise()

    processor = RadarProcessor(config, waveforms)
    detector = CfarDetector(config)
    tracker = RadarTracker(config)

    ptz = SimulatedPTZController(
        LeftLimitDeg=10.0,
        RightLimitDeg=120.0,
        InitialAzimuthDeg=10.0,
        MaxPanRateDegPerSec=90.0,
        PositionToleranceDeg=0.25,
    )
    ptz.Open()

    navigation = SimulatedNavigationSource(initial_heading_deg=0.0)
    pointing = PointingManager(
        ptz=ptz,
        left_limit_deg=10.0,
        right_limit_deg=120.0,
        endpoint_margin_deg=0.5,
        position_tolerance_deg=0.5,
    )

    search = MakeSearchTask(
        TaskId=1,
        SectorStartDeg=10.0,
        SectorStopDeg=120.0,
        ScanRateDegPerSec=45.0,
        SectorFrame=AngleFrame.PLATFORM,
        WaveformProfileId="SEARCH_DEFAULT",
    )

    scheduler = RadarScheduler(search_task=search, debug=False)
    executor = RadarExecutor(
        source=source,
        pointing_manager=pointing,
        config=config,
        debug=False,
    )

    search_dwells_before = 0
    search_dwells_after = 0
    track_dwells = 0
    revisit_submitted = False
    track_completed = False

    deadline = time.time() + 5.0

    try:
        while time.time() < deadline:
            now = time.time()

            if not revisit_submitted and search_dwells_before >= 5:
                scheduler.SubmitRevisitRequest(
                    RevisitRequest(
                        TrackId=12,
                        RequestedTimeSec=now,
                        DeadlineTimeSec=now + 1.0,
                        PredictedRangeM=3000.0,
                        PredictedTrueBearingDeg=70.0,
                        Priority=100,
                        RevisitIntervalSec=0.5,
                    )
                )
                revisit_submitted = True
                print("Synthetic revisit queued for Track 12 at 70.0 deg true")

            task = scheduler.GetNextTask(current_time_sec=now)
            assert task is not None

            result = executor.ExecuteTaskStep(
                task=task,
                navigation=navigation.get_attitude(),
                current_time_sec=now,
            )

            if not result.Executed:
                time.sleep(0.005)
                continue

            processed = processor.Process(result.Raw, result.Dwell)
            detections = detector.Detect(processed, result.Dwell)
            tracker.Update(detections, processed, result.Dwell)

            if task.TaskType == RadarTaskType.SEARCH:
                if not track_completed:
                    search_dwells_before += 1
                else:
                    search_dwells_after += 1

                if search_dwells_before in (1, 5) or search_dwells_after == 1:
                    print(
                        f"SEARCH dwell {result.Dwell.DwellId} at "
                        f"{result.Pointing.BeamBearingTrueDeg:.2f} deg true"
                    )

            elif task.TaskType == RadarTaskType.TRACK:
                track_dwells += 1
                track_completed = True
                print(
                    f"TRACK {task.TrackId} dwell {result.Dwell.DwellId} at "
                    f"{result.Pointing.BeamBearingTrueDeg:.2f} deg true"
                )

            scheduler.CompleteActiveTask(current_time_sec=time.time())
            executor.ReleaseTask(task)

            if task.TaskType == RadarTaskType.TRACK:
                pointing.ResumeSearch(navigation.get_attitude())

            if track_completed and search_dwells_after >= 3:
                break

            time.sleep(0.01)

        assert revisit_submitted, "Synthetic revisit was not submitted"
        assert track_dwells == 1, f"Expected one track dwell, got {track_dwells}"
        assert search_dwells_before >= 5
        assert search_dwells_after >= 3

        print("Scheduler integration regression test passed")

    finally:
        pointing.Stop()
        ptz.Close()
        source.Shutdown()


if __name__ == "__main__":
    main()
