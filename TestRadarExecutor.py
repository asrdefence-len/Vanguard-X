"""
Regression tests for RadarExecutor.py.
"""

import time

from NavigationState import SimulatedNavigationSource
from PointingManager import PointingManager
from PTZController import SimulatedPTZController
from RadarExecutor import RadarExecutor
from RadarTasks import (
    AngleFrame,
    MakeSearchTask,
    MakeTrackTaskFromRevisit,
    RevisitRequest,
)
from SimulatedSource import SimulatedSource
from WaveformLibrary import WaveformLibrary


config = {
    "SampleRate": 1.0e6,
    "NumSamples": 256,
    "NumPulses": 8,
    "PRI": 1.0e-3,

    "SearchWaveformId": "Barker13_20MHz",
    "TrackWaveformId": "Frank10_20MHz",

    "RfFrequency": 9.4e9,
    "TransmitPowerW": 1.0,
    "AntennaGainDb": 10.0,
    "SystemLossDb": 3.0,
    "NoisePowerW": 1e-12,

    "TargetRangeM": 1000.0,
    "TargetVelocityMps": 0.0,
    "TargetRcsSqm": 10.0,

    "TRMRxAttenuationDb": 0.0,
    "TRMTxAttenuationDb": 31.5,
}

waveforms = WaveformLibrary(config)
waveforms.LoadDefaultWaveforms()

source = SimulatedSource(config, waveforms)
source.Initialise()

ptz = SimulatedPTZController(
    LeftLimitDeg=10.0,
    RightLimitDeg=300.0,
    InitialAzimuthDeg=20.0,
    MaxPanRateDegPerSec=120.0,
    PositionToleranceDeg=0.2,
)

ptz.Open()

nav = SimulatedNavigationSource(
    initial_heading_deg=30.0,
    turn_rate_deg_per_sec=0.0,
)

pointing = PointingManager(
    ptz=ptz,
    left_limit_deg=10.0,
    right_limit_deg=300.0,
    endpoint_margin_deg=0.5,
    position_tolerance_deg=0.5,
)

executor = RadarExecutor(
    source=source,
    pointing_manager=pointing,
    config=config,
)

# ----------------------------------------------------------------------
# Search dwell executes while continuous scan is active.
# ----------------------------------------------------------------------

search = MakeSearchTask(
    TaskId=1,
    SectorStartDeg=20.0,
    SectorStopDeg=40.0,
    ScanRateDegPerSec=30.0,
    SectorFrame=AngleFrame.PLATFORM,
)

result = executor.ExecuteTaskStep(
    task=search,
    navigation=nav.get_attitude(),
)

assert result.Executed
assert result.Dwell.TaskType == "SEARCH"
assert result.Raw.Diagnostics["TaskType"] == "SEARCH"
assert "BeamBearingTrueDeg" in result.Raw.Diagnostics
assert result.Raw.Diagnostics["ScanCycle"] == 1

executor.ReleaseTask(search)

# ----------------------------------------------------------------------
# Track dwell waits for pointing and then executes.
# ----------------------------------------------------------------------

request = RevisitRequest(
    TrackId=12,
    RequestedTimeSec=time.time(),
    DeadlineTimeSec=time.time() + 2.0,
    PredictedRangeM=1000.0,
    PredictedTrueBearingDeg=70.0,
    Priority=100,
)

track = MakeTrackTaskFromRevisit(
    TaskId=2,
    Request=request,
)

track_result = None
deadline = time.time() + 2.0

while time.time() < deadline:
    track_result = executor.ExecuteTaskStep(
        task=track,
        navigation=nav.get_attitude(),
    )

    if track_result.Executed:
        break

    assert track_result.WaitingForPointing
    time.sleep(0.01)

assert track_result is not None
assert track_result.Executed
assert track_result.Dwell.TaskType == "TRACK"
assert track_result.Dwell.WaveformName == "Frank10_20MHz"
assert track_result.Raw.Diagnostics["TrackId"] == 12
assert abs(
    track_result.Raw.Diagnostics["BeamBearingTrueDeg"] - 70.0
) < 1.0

executor.ReleaseTask(track)

pointing.Stop()
ptz.Close()

print("RadarExecutor regression test passed")
