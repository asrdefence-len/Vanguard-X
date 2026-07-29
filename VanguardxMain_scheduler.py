"""
===============================================================================
ASR Defence X-Band Radar
VanguardxMain.py
===============================================================================

Foreword
--------
This is the main entry point for the ASR X-band radar prototype software.

The purpose of this file is to connect the major radar software modules together
in a simple, readable, top-level loop.

At this stage, the radar is Python-first and simulation-first. The Ettus hardware
backend will be added later, but it should eventually use the same interface as
the simulated source.

The intended radar processing chain is:

    Dwell definition
        ↓
    Source executes dwell
        ↓
    Raw IQ data returned
        ↓
    Pulse compression / radar processing
        ↓
    Detection
        ↓
    Tracking
        ↓
    Display / logging

Current status
--------------
This version adds the first scene-based scanning simulation:
    - multiple x/y scene objects
    - moving targets are supported
    - 0 to 90 degree search scan
    - 5 degree sinc-squared antenna beam pattern
    - all scene objects contribute through main beam and sidelobes
    - scene returns are passed into SimulatedSource for each dwell
===============================================================================
"""
from CoordinateFrames import GeodeticPosition
from EarthReferencedMeasurements import (
    AnnotateDetectionsWithEarthReference,
)
from NavigationState import (
    CircularRouteNavigationSource,
    SimulatedNavigationSource,
)
from PointingManager import PointingManager
from X660PointingControlLoop import X660PointingControlLoop
from RadarExecutor import RadarExecutor
from RadarScheduler import RadarScheduler
from RadarTasks import (
    AngleFrame,
    MakeSearchTask,
    RadarTaskType,
    SearchPattern,
)
from RadarTimingControls import ApplyTimingControlState
from MissionExecutionController import MissionExecutionController

from SimulatedSource import SimulatedSource
from WaveformLibrary import WaveformLibrary
from RadarProcessor import RadarProcessor
from CfarDetector import CfarDetector
from RadarTracker import RadarTracker
from SimpleDisplay import SimpleDisplay
from RadarRemoteDisplay import RadarRemoteDisplay
from DataLogger import DataLogger
from EttusRadarSource import EttusRadarSource
from EttusOperatingProfiles import (
    ApplyOperatingProfile,
    ApplyX660OperatingProfile,
    ParseOperatingProfileArguments,
)
import os
import sys
import time

try:
    from ReadIMU import IMUReader
except Exception:
    IMUReader = None

try:
    from X660Controller import CreateX660Controller
except Exception:
    CreateX660Controller = None

from TargetScenario import (
    create_default_scene,
    update_scene_objects,
    create_ping_pong_scan_angles,
    build_scene_returns_for_boresight,
    print_scene_returns,
)


# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------

def ParseUiArguments(CommandLineArguments=None):
    """Remove radar/UI transport options before operating-profile parsing.

    Local Qt remains the default. ``--remote-ui`` makes this process the
    headless radar TCP server; the separate RunVanguardUiClient.py process then
    owns Qt.  Binding defaults to loopback until authenticated LAN operation is
    introduced deliberately.
    """

    Arguments = list(
        sys.argv[1:] if CommandLineArguments is None else CommandLineArguments
    )
    Options = {
        "DisplayTransport": "LOCAL_QT",
        "RadarLinkHost": "127.0.0.1",
        "RadarLinkPort": 5810,
    }
    Filtered = []
    Index = 0
    while Index < len(Arguments):
        Argument = Arguments[Index]
        if Argument == "--remote-ui":
            Options["DisplayTransport"] = "TCP_SERVER"
            Index += 1
            continue
        if Argument in ("--radar-link-host", "--radar-link-port"):
            if Index + 1 >= len(Arguments):
                raise ValueError(f"{Argument} requires a value")
            Value = Arguments[Index + 1]
            if Argument == "--radar-link-host":
                Options["RadarLinkHost"] = str(Value)
            else:
                Options["RadarLinkPort"] = int(Value)
            Index += 2
            continue
        Filtered.append(Argument)
        Index += 1
    return Options, Filtered

def ParseSystemModeArguments(CommandLineArguments=None):
    """Remove the top-level SIM/HARD/RF LOOPBACK selection."""

    Arguments = list(
        sys.argv[1:] if CommandLineArguments is None else CommandLineArguments
    )
    SimSelected = "--system-sim" in Arguments
    HardSelected = "--system-hard" in Arguments
    LoopbackSelected = "--system-rf-loopback" in Arguments
    LoopbackSafetyConfirmed = "--i-confirm-rf-loopback-safety" in Arguments
    if sum((SimSelected, HardSelected, LoopbackSelected)) > 1:
        raise ValueError("Select only one system mode")
    if LoopbackSelected and not LoopbackSafetyConfirmed:
        raise ValueError(
            "RF LOOPBACK requires --i-confirm-rf-loopback-safety"
        )
    Filtered = [
        Argument for Argument in Arguments
        if Argument not in (
            "--system-sim", "--system-hard", "--system-rf-loopback",
            "--i-confirm-rf-loopback-safety",
        )
    ]
    if LoopbackSelected:
        return "RF LOOPBACK", Filtered
    return ("HARD" if HardSelected else "SIM"), Filtered

def BuildRadarParamsForScenario(Config, NavigationPose=None):
    """
    Convert the main Config dictionary into the lower-case keys used by
    TargetScenario.py.

    TargetScenario retains its legacy x/y names, where x is mission north and
    y is mission east.  A valid NavigationPose is therefore mapped from ENU as:

        x <- north
        y <- east

    Position and velocity come from the same timestamped pose so simulated
    range, bearing and Doppler remain mutually consistent on a moving platform.
    """

    RadarParams = {
        "carrier_frequency_hz": Config["RfFrequency"],
        "radar_x_m": Config.get("RadarXM", 0.0),
        "radar_y_m": Config.get("RadarYM", 0.0),
        "radar_vx_mps": Config.get("RadarVXMps", 0.0),
        "radar_vy_mps": Config.get("RadarVYMps", 0.0),
        "boresight_deg": Config["BoresightDeg"],
        "beamwidth_deg": Config["BeamwidthDeg"],
        "reference_range_m": Config.get("ReferenceRangeM", 8000.0),
        "target_amplitude_scale": Config.get("TargetAmplitudeScale", 1.0),
        "sidelobe_floor_db": Config.get("SidelobeFloorDb", -50.0),
    }

    if (
        NavigationPose is not None
        and bool(getattr(NavigationPose, "PositionValid", False))
    ):
        RadarParams["radar_x_m"] = float(
            NavigationPose.PositionEnu.north_m
        )
        RadarParams["radar_y_m"] = float(
            NavigationPose.PositionEnu.east_m
        )
        if bool(getattr(NavigationPose, "VelocityValid", False)):
            RadarParams["radar_vx_mps"] = float(
                NavigationPose.VelocityEnu.north_mps
            )
            RadarParams["radar_vy_mps"] = float(
                NavigationPose.VelocityEnu.east_mps
            )

    return RadarParams


def CreateNavigationSource(Config, time_source=time.time):
    """Create the real-contract navigation digital twin selected in Config."""

    MissionOrigin = GeodeticPosition(
        latitude_deg=float(
            Config.get(
                "MissionOriginLatitudeDeg",
                Config.get("MapLatitudeDeg", 0.0),
            )
        ),
        longitude_deg=float(
            Config.get(
                "MissionOriginLongitudeDeg",
                Config.get("MapLongitudeDeg", 0.0),
            )
        ),
        altitude_m=float(Config.get("MissionOriginAltitudeM", 0.0)),
    )
    Mode = str(
        Config.get("SimulatedNavigationMode", "STATIONARY")
    ).strip().upper()

    if Mode in ("CIRCLE", "CIRCULAR", "CIRCULAR_ROUTE"):
        Navigation = CircularRouteNavigationSource(
            mission_origin=MissionOrigin,
            centre_east_m=float(
                Config.get("SimulatedCircleCentreEastM", 0.0)
            ),
            centre_north_m=float(
                Config.get("SimulatedCircleCentreNorthM", 0.0)
            ),
            radius_m=float(
                Config.get("SimulatedCircleRadiusM", 1000.0)
            ),
            speed_mps=float(
                Config.get("SimulatedCircleSpeedMps", 5.0)
            ),
            clockwise=bool(
                Config.get("SimulatedCircleClockwise", True)
            ),
            initial_radial_bearing_deg=float(
                Config.get(
                    "SimulatedCircleInitialRadialBearingDeg",
                    0.0,
                )
            ),
            source_name="SIMULATED_CIRCULAR_ROUTE",
            time_source=time_source,
        )
        print(
            "Simulated platform navigation: CIRCLE "
            f"centre_E={Navigation.CentreEastM:.1f} m, "
            f"centre_N={Navigation.CentreNorthM:.1f} m, "
            f"radius={Navigation.RadiusM:.1f} m, "
            f"speed={Navigation.SpeedMps:.1f} m/s, "
            f"clockwise={Navigation.Clockwise}"
        )
        return Navigation

    if Mode not in ("STATIONARY", "LINEAR"):
        raise ValueError(
            f"Unsupported SimulatedNavigationMode: {Mode}"
        )

    return SimulatedNavigationSource(
        mission_origin=MissionOrigin,
        initial_heading_deg=float(
            Config.get("SimulatedInitialHeadingDeg", 0.0)
        ),
        velocity_east_mps=float(
            Config.get("SimulatedVelocityEastMps", 0.0)
        ),
        velocity_north_mps=float(
            Config.get("SimulatedVelocityNorthMps", 0.0)
        ),
        source_name=(
            "SIMULATED_LINEAR_ROUTE"
            if Mode == "LINEAR"
            else "SIMULATED_STATIONARY"
        ),
        time_source=time_source,
    )


def SelectDisplay(Config):
    """
    Create the selected display.

    RadarDisplay is the new operator-style display with SCAN/STARE controls.
    SimpleDisplay is kept as a fallback engineering display.
    """

    if str(Config.get("DisplayTransport", "LOCAL_QT")).upper() == "TCP_SERVER":
        return RadarRemoteDisplay(Config)

    if Config.get("DisplayType", "SimpleDisplay") == "RadarDisplay":
        # Keep Qt out of the radar-server process.  This local import is only
        # reached by the backwards-compatible single-process mode.
        from RadarDisplayQt5 import RadarDisplay
        return RadarDisplay(Config)

    return SimpleDisplay(Config)


def GetControlledBoresightDeg(Display, ScanBoresightDeg, ControlState=None):
    """
    Decide which boresight angle to use for this dwell.

    SCAN mode:
        Use the current angle from the ping-pong scan sequence.

    STARE mode or STOP:
        Hold the beam angle selected by the display.
    """

    if ControlState is None:
        ControlState = (
            Display.GetControlState()
            if hasattr(Display, "GetControlState")
            else None
        )

    if ControlState is None:
        return float(ScanBoresightDeg)

    DisplayMode = ControlState.get("DisplayMode", "SCAN")
    ScanEnabled = bool(ControlState.get("ScanEnabled", True))

    if DisplayMode == "SCAN" and ScanEnabled:
        BoresightDeg = float(ScanBoresightDeg)

        if hasattr(Display, "BeamAngleDeg"):
            Display.BeamAngleDeg = BoresightDeg

        return BoresightDeg

    return float(ControlState.get("BeamAngleDeg", ScanBoresightDeg))


def ApplyOperatorPointingCommand(
    Pointing,
    ControlState,
    DisplayMode,
    ScanEnabled,
    LastDisplayMode,
    LastScanEnabled,
    LastManualNudgeCommandId,
):
    """Apply one-shot STARE/STOP/nudge intent independently of RF state.

    This function must run before the radar-dwell enable/throttle gates. X6-60
    pointing is an operator control path and must not depend on TX being armed.
    Returns ``(last_nudge_id, action, commanded_relative_deg)``.
    """
    if Pointing is None:
        return LastManualNudgeCommandId, "NONE", None

    ManualNudgeCommandId = LastManualNudgeCommandId
    ManualNudgeDeltaDeg = 0.0
    if ControlState is not None:
        ManualNudgeCommandId = int(
            ControlState.get(
                "ManualNudgeCommandId",
                LastManualNudgeCommandId,
            )
        )
        ManualNudgeDeltaDeg = float(
            ControlState.get("ManualNudgeDeltaDeg", 0.0)
        )

    if (
        ManualNudgeCommandId != LastManualNudgeCommandId
        and abs(ManualNudgeDeltaDeg) > 0.0
    ):
        CommandedRelativeDeg = Pointing.Nudge(ManualNudgeDeltaDeg)
        return ManualNudgeCommandId, "NUDGE", CommandedRelativeDeg

    LeavingScan = bool(
        LastDisplayMode == "SCAN"
        and LastScanEnabled
        and not (DisplayMode == "SCAN" and ScanEnabled)
    )
    StopTransition = bool(
        DisplayMode == "STOP" and LastDisplayMode != "STOP"
    )
    if LeavingScan or StopTransition:
        Pointing.Stop()
        return LastManualNudgeCommandId, "HOLD", None

    return LastManualNudgeCommandId, "NONE", None


def RefreshX660MeasuredBeam(X660, Display):
    """Refresh the displayed beam directly from X6-60 encoder telemetry."""

    if X660 is None:
        return None

    State = X660.Update()
    if bool(getattr(State, "Valid", False)):
        AzimuthDeg = float(State.AzimuthDeg) % 360.0
        if hasattr(Display, "SetMeasuredBeamAngle"):
            Display.SetMeasuredBeamAngle(AzimuthDeg)
        elif hasattr(Display, "BeamAngleDeg"):
            Display.BeamAngleDeg = AzimuthDeg
    return State



def InitialiseX660AtCurrentPose(X660, Config, Display=None):
    """Adopt the measured X6-60 pose without commanding startup motion.

    The absolute encoder is the startup reference.  Opening Vanguard X must
    therefore leave the X6-60 where it is, initialise the boresight and PPI
    beam from valid telemetry, and wait for explicit operator scan, STARE, or
    nudge intent before issuing a motion command.
    """
    if X660 is None:
        return None

    State = X660.Update() if hasattr(X660, "Update") else X660.GetState()
    if not bool(getattr(State, "Valid", False)):
        raise RuntimeError(
            "X6-60 startup encoder telemetry is invalid; no motion commanded"
        )

    AzDeg = float(State.AzimuthDeg) % 360.0
    ElDeg = float(
        getattr(State, "ElevationDeg", Config.get("AntennaElDeg", 0.0))
    )

    Config["InitialBeamAngleDeg"] = AzDeg
    Config["BoresightDeg"] = AzDeg
    Config["AntennaAzDeg"] = AzDeg
    Config["AntennaElDeg"] = ElDeg
    Config["X660AzDeg"] = AzDeg
    Config["X660RawAngleDeg"] = getattr(State, "RawAngleDeg", None)

    if hasattr(Display, "SetMeasuredBeamAngle"):
        Display.SetMeasuredBeamAngle(AzDeg)
    elif hasattr(Display, "BeamAngleDeg"):
        Display.BeamAngleDeg = AzDeg

    print(
        "X6-60 startup position adopted from encoder: "
        f"AZ={AzDeg:.2f} deg (no startup motion commanded)"
    )
    return State





def ExecuteRadarDwell(
    Config,
    ScheduledTask,
    Executor,
    NavigationAttitude,
    Processor,
    Detector,
    Tracker,
    Display,
    Logger,
    ScanCycle,
    DisplayMode,
    BoresightDeg,
    CommandedBoresightDeg,
    MeasuredAntennaAzDeg,
    MeasuredAntennaElDeg,
    ImuValid,
    ImuSource,
    X660AzDeg,
    X660RateDegPerSec,
    X660Valid,
    X660Source,
    X660AtTarget,
    X660RawAngleDeg,
):
    """Execute one scheduled dwell and preserve the processing chain.

    RadarExecutor owns DwellPlan construction and source execution.
    PointingManager is the sole owner of continuous X6-60 search-scan movement.
    """

    T0 = time.perf_counter()
    BeginRadarTimingCritical = getattr(
        Display,
        "BeginRadarTimingCritical",
        None,
    )
    EndRadarTimingCritical = getattr(
        Display,
        "EndRadarTimingCritical",
        None,
    )
    if callable(BeginRadarTimingCritical):
        BeginRadarTimingCritical()
    try:
        ExecutionResult = Executor.ExecuteTaskStep(
            task=ScheduledTask,
            navigation=NavigationAttitude,
        )
    finally:
        if callable(EndRadarTimingCritical):
            EndRadarTimingCritical()
    T1 = time.perf_counter()

    if not ExecutionResult.Executed:
        return {
            "Executed": False,
            "WaitingForPointing": bool(ExecutionResult.WaitingForPointing),
            "Reason": str(ExecutionResult.Reason),
            "Pointing": ExecutionResult.Pointing,
            "T0": T0,
            "T1": T1,
        }

    Raw = ExecutionResult.Raw
    ThisDwell = ExecutionResult.Dwell

    EffectiveScanCycle = (
        int(ExecutionResult.Pointing.SearchCycle)
        if ExecutionResult.Pointing.SearchCycle is not None
        else int(ScanCycle)
    )

    # Preserve operator context without mutating core DwellPlan fields.
    ThisDwell.Metadata["ScanCycle"] = EffectiveScanCycle
    ThisDwell.Metadata["DisplayMode"] = str(DisplayMode)
    ThisDwell.Metadata["PointingControlOwner"] = "PointingManager"

    Processed = Processor.Process(Raw, ThisDwell)
    T2 = time.perf_counter()

    Processed.Diagnostics["BoresightDeg"] = float(BoresightDeg)
    Processed.Diagnostics["BeamAngleDeg"] = float(BoresightDeg)
    Processed.Diagnostics["CommandedBoresightDeg"] = float(CommandedBoresightDeg)
    Processed.Diagnostics["AntennaAzDeg"] = float(MeasuredAntennaAzDeg)
    Processed.Diagnostics["AntennaElDeg"] = float(MeasuredAntennaElDeg)
    Processed.Diagnostics["IMUValid"] = bool(ImuValid)
    Processed.Diagnostics["IMUSource"] = str(ImuSource)
    Processed.Diagnostics["X660AzDeg"] = float(X660AzDeg)
    Processed.Diagnostics["X660RateDegPerSec"] = float(X660RateDegPerSec)
    Processed.Diagnostics["X660Valid"] = bool(X660Valid)
    Processed.Diagnostics["X660Source"] = str(X660Source)
    Processed.Diagnostics["X660AtTarget"] = bool(X660AtTarget)
    Processed.Diagnostics["X660RawAngleDeg"] = (
        None if X660RawAngleDeg is None else float(X660RawAngleDeg)
    )
    Processed.Diagnostics["ScanCycle"] = EffectiveScanCycle
    Processed.Diagnostics["ScheduledTaskId"] = int(ScheduledTask.TaskId)
    Processed.Diagnostics["ScheduledTaskType"] = str(ScheduledTask.TaskType)
    Processed.Diagnostics["ScheduledWaveformProfileId"] = str(
        getattr(ScheduledTask, "WaveformProfileId", "")
    )

    Detections = Detector.Detect(Processed, ThisDwell)
    EarthReferencedMeasurements = AnnotateDetectionsWithEarthReference(
        Detections,
        NavigationAttitude,
    )

    # Stage 5 validation boundary: detections now carry a parallel mission-ENU
    # interpretation, but the established range/bearing tracker remains
    # authoritative until circular-platform trials validate this measurement
    # stream.  Tracker.Update deliberately receives the same legacy detections.
    Processed.Diagnostics["EarthReferenceAuthoritative"] = False
    Processed.Diagnostics["EarthReferencedDetectionCount"] = sum(
        1 for Measurement in EarthReferencedMeasurements
        if Measurement.Valid
    )
    Processed.Diagnostics["EarthReferencedVelocityCount"] = sum(
        1 for Measurement in EarthReferencedMeasurements
        if Measurement.Valid and Measurement.VelocityValid
    )
    Processed.Diagnostics["NavigationPoseSequenceNumber"] = getattr(
        NavigationAttitude,
        "SequenceNumber",
        None,
    )
    Processed.Diagnostics["NavigationPoseTimestampSec"] = getattr(
        NavigationAttitude,
        "TimestampSec",
        None,
    )

    if Config.get("TrackerEnabled", True):
        Tracks, Plots = Tracker.Update(Detections, Processed, ThisDwell)
        TrackerDebug = Tracker.GetDebugInfo() if hasattr(Tracker, "GetDebugInfo") else {}
    else:
        Tracks, Plots = [], []
        TrackerDebug = {}

    Processed.Diagnostics["TrackerCurrentScanPoints"] = int(TrackerDebug.get("CurrentScanPoints", 0))
    Processed.Diagnostics["TrackerLastCompletedBlobs"] = int(TrackerDebug.get("LastCompletedBlobs", 0))
    Processed.Diagnostics["TrackerTentativeTracks"] = int(TrackerDebug.get("TentativeTracks", 0))
    Processed.Diagnostics["TrackerConfirmedTracks"] = int(TrackerDebug.get("ConfirmedTracks", 0))

    T3 = time.perf_counter()
    Logger.log_dwell(Processed, Detections)
    T4 = time.perf_counter()
    Display.Update(Processed, Detections, Tracks=Tracks, Plots=Plots)
    T5 = time.perf_counter()

    return {
        "Executed": True,
        "WaitingForPointing": False,
        "Reason": "",
        "Pointing": ExecutionResult.Pointing,
        "ScanCycle": EffectiveScanCycle,
        "Dwell": ThisDwell,
        "Processed": Processed,
        "Detections": Detections,
        "EarthReferencedMeasurements": EarthReferencedMeasurements,
        "Tracks": Tracks,
        "Plots": Plots,
        "TrackerDebug": TrackerDebug,
        "T0": T0,
        "T1": T1,
        "T2": T2,
        "T3": T3,
        "T4": T4,
        "T5": T5,
    }

def Main(CommandLineArguments=None):
    """
    Main radar program.

    This function creates the configuration, initialises the radar modules,
    creates a target scene, scans the radar across a sector, executes a dwell at
    each scan angle, processes the result, detects targets, and displays output.
    """

    # -------------------------------------------------------------------------
    # Configuration dictionary
    # -------------------------------------------------------------------------

    UiOptions, ArgumentsWithoutUi = ParseUiArguments(CommandLineArguments)
    SystemMode, ProfileArguments = ParseSystemModeArguments(
        ArgumentsWithoutUi
    )
    OperatingArguments = ParseOperatingProfileArguments(ProfileArguments)

    Config = {
        # Operator timing selections. PRI, CPI duration, RX start and receive
        # sample count are derived by RadarTiming; they are not independent
        # configuration inputs.
        "MinPrfHz": 1000.0,
        "MaxPrfHz": 4000.0,
        # Operational Golay uses 64 physical A/B pulses at 4 kHz.  The
        # complementary processor produces 32 pair samples at a 2 kHz pair
        # rate, preserving the established 16 ms CPI and Doppler span.
        "SelectedPrfHz": 4000.0,
        "SelectedPulsesPerCpi": 64,
        "MinPulsesPerCpi": 8,
        "MaxPulsesPerCpi": 128,
        "InstrumentedMaxRangeM": 15000.0,
        "MinSelectableRangeM": 1000.0,
        "MaxSelectableRangeM": 15000.0,

        # Provisional RF recovery/margin values. These must be replaced by
        # oscilloscope measurements before high-power timed transmission.
        "ReceiverRecoveryTimeSec": 1.0e-6,
        "RxEndMarginSec": 2.0e-6,
        "NextTxGuardTimeSec": 2.0e-6,

        # ---------------------------------------------------------------------
        # Radar source
        # ---------------------------------------------------------------------

        "SystemMode": SystemMode,
        "RadarSource": "SIM",

        # Ettus configuration
        "EttusSerial": "34A0320",
        "EttusRxFrequencyHz": 1.0e9,
        "EttusRxGainDb": 10.0,
        "EttusRxAntenna": "RX2",
        "EttusRxChannel": 0,
        "EttusSampleRateHz": 40.0e6,
        "EttusMaxSampleRateHz": 40.0e6,
        "EttusReceiveTimeoutSec": 1.0,
        # No-profile startup remains fail-closed.  The Stage 3H profile enables
        # the CRO-verified ATR mapping on the safe attenuated SDR loopback.
        # The command lead is 5 ms and dwell cadence remains 100 ms.
        "EttusOperatingMode": "RECEIVE_ONLY",
        "EttusTimedTransmitEnabled": False,
        "EttusAtrGpioEnabled": False,
        "EttusAtrAllowOverlapForSimulation": False,
        "EttusTxLeadingZeroSamples": 8,
        "EttusCommandLeadTimeSec": 0.005,
        "EttusCommandQueueDepth": 20,
        "EttusRxWarmupEnabled": True,
        "EttusDebug": True,

        # Initial pulse-plan architecture. Search and track waveform selectors
        # are separate even though only SEARCH is scheduled in this version.
        "SearchWaveformId": "Golay64_20MHz",
        "TrackWaveformId": "Golay64_20MHz",

        # RF parameters
        "RfFrequency": 9.4e9,
        "TransmitPowerW": 50.0,
        "AntennaGainDb": 23.0,
        "SystemLossDb": 6.0,

        # Backwards-compatible single target fields.
        # These are still used if Config["SceneReturns"] is not provided.
        "TargetRangeM": 8000.0,
        "TargetVelocityMps": 5.0,
        "TargetRcsSqm": 1000,

        # Receiver / simulation noise
        "NoisePowerW": 1e-13,

        # Detection
        "ThresholdDb": -80.0,

        # ---------------------------------------------------------------------
        # Scene / scanning parameters
        # ---------------------------------------------------------------------
        "RadarXM": 0.0,
        "RadarYM": 0.0,
        "RadarVXMps": 0.0,
        "RadarVYMps": 0.0,
        # The simulator publishes the same timestamped NavigationPose contract
        # that the future operational GPS/WT901 source will publish.
        # STATIONARY preserves the current default. Set this to CIRCLE for the
        # deterministic offshore circular-route digital twin.
        "SimulatedNavigationMode": "STATIONARY",
        "MissionOriginLatitudeDeg": -34.368,
        "MissionOriginLongitudeDeg": 150.929,
        "MissionOriginAltitudeM": 0.0,
        "SimulatedInitialHeadingDeg": 0.0,
        "SimulatedVelocityEastMps": 0.0,
        "SimulatedVelocityNorthMps": 0.0,
        "SimulatedCircleCentreEastM": 5000.0,
        "SimulatedCircleCentreNorthM": 0.0,
        "SimulatedCircleRadiusM": 1000.0,
        "SimulatedCircleSpeedMps": 5.0,
        "SimulatedCircleClockwise": True,
        "SimulatedCircleInitialRadialBearingDeg": 0.0,
        "ScanStartDeg": 120.0,
        "ScanStopDeg": 10.0,
        "ScanStepDeg": 1,
        "BoresightDeg": 0.0,
        "BeamwidthDeg": 5.0,
        "SidelobeFloorDb": -50.0,
        "ReferenceRangeM": 8000.0,
        "TargetAmplitudeScale": 1.0,

        # CFAR parameters
        "TrainingCellsRange": 12,
        "TrainingCellsDoppler": 4,
        "GuardCellsRange": 4,
        "GuardCellsDoppler": 1,
        "CfarThresholdDb": 12.3,
        "MaxDetections": 200,
        # Operational detection blanking. Direct-path leakage and receiver
        # recovery currently contaminate the first 2 km of the cabled RF data.
        # The range profile remains unblanked for diagnostics, but CFAR emits
        # no detections here, so these cells cannot seed tracker plots/tracks.
        "MinRangeM": 2000.0,
        "MaxRangeM": 15000.0,

        # Tracker / plot extraction parameters
        "TrackerEnabled": True,
        "ReturnBlobsForDebug": True,
        "InitiationWindow": 3,
        "InitiationRequiredHits": 2,
        "InitiationRangeGateM": 300.0,
        "InitiationAzimuthGateDeg": 6.0,
        "AssociationRangeGateM": 300.0,
        "AssociationAzimuthGateDeg": 6.0,
        "ClusterRangeGapBins": 2,
        "ClusterDopplerGapBins": 2,
        "ClusterRangeGapM": 200.0,
        "ClusterDopplerGapHz": 120.0,
        "ClusterAzimuthGapDeg": 3.0,
        "TrackGateRangeM": 150.0,
        "TrackGateAzimuthDeg": 5.0,
        "TrackGateVelocityMps": 12.0,
        "TrackGateDopplerHz": 300.0,
        "TrackConfirmHits": 2,
        "TrackConfirmWindow": 3,
        "DeleteAfterMissesTentative": 2,
        "DeleteAfterMissesConfirmed": 5,

        # Debug controls
        "PrintSceneTruthTable": False,
        "LogoPath": "ASRDefenceLOGO.png",

        # ---------------------------------------------------------------------
        # Data logging controls
        # ---------------------------------------------------------------------
        # The display has a Save checkbox and File box. The main loop reads
        # those controls and opens/closes the HDF5 logger accordingly.
        "DataLoggingEnabled": False,
        "DataLogDirectory": "DataLogs",
        "DataLogFilename": "datafile1.h5",
        "DataLogOverwrite": True,
        "DataLogPrefix": "VanguardLog",
        "DataLogFlushEveryNDwells": 25,
        "LogDetections": True,
        "LogRangeDoppler": True,
        "LogRangeDopplerEveryNDwells": 10,
        "LogRangeDopplerMaxRangeM": 15000.0,
        "LogRangeDopplerFloat32": True,
        # ---------------------------------------------------------------------
        # Display controls
        # ---------------------------------------------------------------------
        "DisplayType": "RadarDisplay",
        "DisplayTransport": UiOptions["DisplayTransport"],
        "RadarLinkHost": UiOptions["RadarLinkHost"],
        "RadarLinkPort": UiOptions["RadarLinkPort"],
        # Prototype-safe policy: loss of the UI heartbeat stops scanning and
        # inhibits TX. This can later become a mission-level operating policy.
        "RadarLinkHeartbeatTimeoutSec": 2.0,
        "RadarLinkMaxRangeProfilePoints": 1500,

        "UpdatePlots": True,
        "ShowRangeProfile": True,
        "ShowRangeDopplerMap": False,
        "ShowPolarDetections": True,
        "ShowRawDetections": True,
        "ShowTrackerPlots": True,
        "ShowRangeDetectionMarkers": True,
        "ShowTracks": True,

        "PrintEveryNDwells": 1,
        "DetailPlotEveryNDwells": 1,
        "PolarPlotEveryNDwells": 1,

        "PlotPauseS": 0.001,
        "MaxDisplayRangeM": 15000.0,
        "PolarMaxRangeM": 15000.0,
        "MaxDetectionsPlottedPerDwell": 50,
        "MaxPolarDetections": 500,
        "RangeRingStepM": 2000.0,

        # Offline North-up map beneath the PPI. The radar remains at the
        # centre; live GPS will later replace these fixed Bellambi coordinates.
        "MapEnabled": True,
        "MapLatitudeDeg": -34.368,
        "MapLongitudeDeg": 150.929,
        "MapDatasetPath": "NSWCoast_Newcastle_to_BatemansBay_OSM_20260722.json",
        "MapLandColour": (92, 92, 92, 105),
        "MapCoastColour": (255, 255, 255, 190),
        "MapLabelColour": (255, 255, 255, 180),

        "RangeDopplerUpdateEveryNDwells": 0,
        "PolarUpdateEveryNDwells": 1,
        "PolarDetectionsUpdateEveryNDwells": 1,
        "RangeProfileUpdateEveryNDwells": 1,
        "RangeProfileDopplerMode": "MAX",
        "StatusUpdateEveryNDwells": 1,
        "QtProcessEventsEveryNDwells": 1,
        "QtRangeProfileDecimation": 1,

        # ---------------------------------------------------------------------
        # Operator display / scan controls
        # ---------------------------------------------------------------------
        "InitialDisplayMode": "STOP",
        "InitialScanEnabled": False,
        # Replaced from valid X6-60 encoder telemetry when the controller opens.
        "InitialBeamAngleDeg": 0.0,
        "ManualBeamStepDeg": 1.0,
        "MinScanRateDegPerSec": 1.0,

        # X6-60 motor/positioning unit: unlimited multi-turn azimuth.
        "EnableX660": True,
        "X660Mode": "x660-sim",
        "X660Debug": False,
        "X660PositionToleranceDeg": 0.75,
        "X660ScanEndpointMarginDeg": 1.0,
        "X660ScanSlewRateDegPerSec": 20.0,
        # Reverse the speed command before the sector boundary so the X6-60
        # planner decelerates through zero at the requested physical endpoint.
        # The 60 deg/s^2 value is the verified output-shaft equivalent of the
        # enforced 1140 motor-side planner setting.  The latency term accounts
        # for telemetry age, scheduler cadence, and CAN command response.
        "X660ScanBrakingEnabled": True,
        "X660ScanDecelerationDegPerSec2": 60.0,
        "X660ScanCommandLatencySec": 0.05,
        "X660ScanPattern": "SECTOR",
        "X660SimMaxRateDegPerSec": 60.0,
        # Endpoint control and the telemetry feeding it run at 50 Hz,
        # independently of the 10 Hz radar dwell.  Both stay in this scheduler
        # thread so CAN transactions remain serial.
        "X660PointingControlIntervalSec": 0.020,
        "X660PointingControlDebug": False,
        "X660TelemetryIntervalSec": 0.020,
        "RadarDwellIntervalSec": 0.10,
        # Stage 4B/4D SocketCAN telemetry-only integration.  The calibrated
        # mapping is North=000 deg, positive clockwise, naturally wrapping at
        # 360 deg while the raw multi-turn angle remains unchanged.
        "X660CanInterface": "can0",
        "X660NodeId": 1,
        "X660CanTimeoutSec": 0.25,
        "X660NorthRawAngleDeg": -361.53,
        "X660DirectionSign": +1,

        # Operational CAN motion remains opt-in. To connect the proven motion
        # transport to PointingManager, select x660-operational and set all
        # three session-authorisation values to True. The public coordinate
        # convention remains North=0 deg and clockwise-positive.
        "X660MotionEnabled": False,
        "X660IUnderstandMotionWillOccur": False,
        "X660IConfirmMotionAreaIsClear": False,
        "X660OperationalMaxRateDegPerSec": 60.0,
        "X660PositionCommandSpeedDegPerSec": 14,
        "X660MaximumNudgeDeg": 10.0,
        # Characterised X6-60 motor-side planner setting. With the unit's
        # 19:1 gearbox, 1140 produces approximately 60 deg/s^2 at the output.
        # Operational startup reads indexes 00-03, writes only mismatches,
        # verifies all four, and refuses MotorReady if verification fails.
        "X660PlannerInternalAccelerationDegPerSec2": 1140,

        # Antenna attitude / IMU controls
        "EnableIMU": False,
        "UseIMUForBeamAngle": False,
        "IMUDummyMode": True,
        "IMUAzimuthOffsetDeg": 0.0,
        "IMUElevationOffsetDeg": 0.0,
        "IMUInvertAzimuth": False,
        "IMUInvertElevation": False,
    }

    OperatingProfile = ApplyOperatingProfile(
        Config,
        OperatingArguments,
    )
    X660OperatingProfile = ApplyX660OperatingProfile(
        Config,
        OperatingArguments,
    )

    # The top-bar selector owns the complete adapter pairing.  HARD is
    # deliberately receive-only: it may move the X6-60, but never enables RF
    # transmission.  SIM never opens Ettus or SocketCAN.
    if SystemMode == "RF LOOPBACK":
        Config.update({
            "RadarSource": "ETTUS",
            "EttusOperatingMode": "TIMED_TX_RX",
            "EttusTimedTransmitEnabled": True,
            "EttusRfOutputAcknowledged": True,
            "EttusLoopbackConfirmed": True,
            "EttusExternalAttenuationDb": 30.0,
            # Preserve the verified Stage 3I RF path and gains.  This GUI
            # profile differs only in keeping both ATR outputs forced low.
            "EttusTxGainDb": 50.0,
            "EttusRxGainDb": 30.0,
            "EttusAtrGpioEnabled": False,
            "EttusAtrIsolationRequired": True,
            "EttusTrmPaAntennaIsolatedConfirmed": True,
            "EttusAtrAllowOverlapForSimulation": False,
            "Stage3E1LoopbackActive": True,
            # Reproduce the proven delayed RF target-emulator test.  A fixed
            # stationary target is emitted only while the simulated X6-60
            # boresight is within +/-2 degrees of 80 degrees.
            "EttusRfTargetEmulatorEnabled": True,
            "EttusRfTargetUseScenario": False,
            "EttusRfTargetRangeM": 6000.0,
            "EttusRfTargetBearingDeg": 80.0,
            "EttusRfTargetAngleHalfWidthDeg": 2.0,
            "EttusRfTargetRadialVelocityMps": 0.0,
            "EttusLoopbackHardwareDelaySamples": 166,
            "RangeProfileDopplerMode": "ZERO_DOPPLER",
            "InitialTransmitEnabled": False,
            "X660Mode": "x660-sim",
            "X660MotionEnabled": False,
        })
        OperatingProfile = "GUI_ATR_ISOLATED_LOOPBACK"
    elif SystemMode == "HARD":
        Config["RadarSource"] = "ETTUS"
        Config["EttusOperatingMode"] = "RECEIVE_ONLY"
        Config["EttusTimedTransmitEnabled"] = False
        Config["EttusAtrGpioEnabled"] = False
        Config["EttusAtrIsolationRequired"] = False
        Config["InitialTransmitEnabled"] = False
        Config["X660Mode"] = "x660-operational"
        Config["X660MotionEnabled"] = True
        Config["X660IUnderstandMotionWillOccur"] = True
        Config["X660IConfirmMotionAreaIsClear"] = True
    else:
        Config["RadarSource"] = "SIM"
        Config["X660Mode"] = "x660-sim"
        Config["X660MotionEnabled"] = False
    if X660OperatingProfile == "X660_OPERATIONAL":
        print(
            "X6-60 OPERATIONAL MOTION SELECTED "
            "(clockwise-positive, encoder beam feedback enabled)"
        )
    if OperatingProfile == "GUI_ATR_ISOLATED_LOOPBACK":
        print("GUI_ATR_ISOLATED_LOOPBACK GUARDED PROFILE SELECTED")
        print("  RF path:        TX/RX -> 30.0 dB or greater -> RX2")
        print("  RF TX / RX:     TIMED / ENABLED")
        print("  TX ATR:         FORCED LOW (manual GPIO, readback required)")
        print("  RX ATR:         FORCED LOW (manual GPIO, readback required)")
        print("  TRM / PA / ANT: DISCONNECTED OR PHYSICALLY ISOLATED")
        print("  X6-60:          STOPPED (simulated adapter; no CAN motion)")
        print("  TX/RX gains:    50.0 / 30.0 dB (verified Stage 3I values)")
        print("  RF target:      FIXED POINT 6.000 km at 80.00 deg, 0.00 m/s")
        print("  bearing gate:   80.00 deg +/-2.00 deg")
        print("  calibrated delay: 166 samples")
    if OperatingProfile in (
        "STAGE3E1_LOOPBACK",
        "STAGE3F_RF_TARGET",
        "STAGE3H_ATR_LOOPBACK",
        "STAGE3I_ATR_RF_TARGET_OVERLAP",
    ):
        print(
            f"{OperatingProfile} GUARDED FULL-APPLICATION PROFILE SELECTED"
        )
        print(
            "  RF path:        TX/RX -> "
            f"{Config['EttusExternalAttenuationDb']:.1f} dB -> RX2"
        )
        if OperatingProfile in (
            "STAGE3H_ATR_LOOPBACK",
            "STAGE3I_ATR_RF_TARGET_OVERLAP",
        ):
            print("  ATR:            ENABLED (CRO-verified FP0 mapping)")
            print("  GPIO:           J6-3 TX, J6-4 RX, J6-5 ATR_XX witness")
            print("  TRM / PA:       DISCONNECTED")
            print("  TX pre-roll:    8 zero samples / 0.200 us")
            if OperatingProfile == "STAGE3I_ATR_RF_TARGET_OVERLAP":
                print("  ATR_XX:         TX + RX + witness HIGH (SIMULATION ONLY)")
                print("  WARNING:        NEVER CONNECT TRM OR PA IN THIS PROFILE")
        else:
            print("  ATR / TRM / PA: DISABLED")
        print(
            f"  TX/RX gains:    {Config['EttusTxGainDb']:.1f} / "
            f"{Config['EttusRxGainDb']:.1f} dB"
        )
        print("  shutdown:       operator Stop / Exit")
        if OperatingProfile in (
            "STAGE3F_RF_TARGET",
            "STAGE3I_ATR_RF_TARGET_OVERLAP",
        ):
            if Config.get("EttusRfTargetUseScenario", False):
                print(
                    "  RF targets:     TargetScenario strongest parent "
                    "inside true bearing +/-2.00 deg"
                )
                print(
                    "  RF representation: one equivalent delayed pulse per PRI"
                )
            else:
                print(
                    "  RF target:      FIXED POINT "
                    f"{Config['EttusRfTargetRangeM'] / 1000.0:.3f} km at "
                    f"{Config['EttusRfTargetBearingDeg']:.2f} deg, "
                    f"{Config['EttusRfTargetRadialVelocityMps']:.2f} m/s"
                )
            print(
                "  calibrated delay: "
                f"{Config['EttusLoopbackHardwareDelaySamples']} samples"
            )

    # -------------------------------------------------------------------------
    # Waveform library
    # -------------------------------------------------------------------------

    TheWaveformLibrary = WaveformLibrary(Config)
    TheWaveformLibrary.LoadDefaultWaveforms()
    Config["AvailableWaveformIds"] = (
        TheWaveformLibrary.ListWaveforms()
    )

    # -------------------------------------------------------------------------
    # Source, processor, detector and display
    # -------------------------------------------------------------------------


    # -------------------------------------------------------------------------
    # Source selection
    # -------------------------------------------------------------------------

    RadarSourceType = Config.get("RadarSource", "SIM").upper()

    if RadarSourceType == "ETTUS":
        print("Using Ettus B200mini radar source")
        Source = EttusRadarSource(Config, TheWaveformLibrary)

    else:
        print("Using simulated radar source")
        Source = SimulatedSource(Config, TheWaveformLibrary)

    # Publish actual RF capability to the operator display. In Stage 3E0 the
    # Ettus source is receive-only, so Start may run RX dwells but must never
    # present transmit as enabled or available.
    Config["RfTransmitAvailable"] = bool(
        getattr(Source, "TimedTransmitEnabled", True)
    )
    # Even when the guarded loopback profile makes TX available, startup is
    # unarmed. The operator must press Start before the first timed dwell.
    Config["InitialTransmitEnabled"] = False
    ReceiveOnlyOperation = bool(
        RadarSourceType == "ETTUS"
        and str(getattr(Source, "OperatingMode", "")).upper()
        == "RECEIVE_ONLY"
    )

    Processor = RadarProcessor(Config, TheWaveformLibrary)

    Detector = CfarDetector(Config)
    Tracker = RadarTracker(Config)
    Display = SelectDisplay(Config)
    Logger = DataLogger(Config)

    # IMU / antenna attitude reader. Read once per dwell in the main loop.
    if Config.get("EnableIMU", False) and IMUReader is not None:
        Imu = IMUReader(
            dummy=bool(Config.get("IMUDummyMode", True)),
            az_offset_deg=float(Config.get("IMUAzimuthOffsetDeg", 0.0)),
            el_offset_deg=float(Config.get("IMUElevationOffsetDeg", 0.0)),
            invert_az=bool(Config.get("IMUInvertAzimuth", False)),
            invert_el=bool(Config.get("IMUInvertElevation", False)),
        )
        print("IMU reader enabled.")
    else:
        Imu = None
        if Config.get("EnableIMU", False):
            print("IMU requested, but ReadIMU.py / IMUReader could not be imported.")

    # X6-60 motor/positioning unit.  Only X6-60 CAN telemetry and the X6-60
    # software simulator are supported.
    if Config.get("EnableX660", False) and CreateX660Controller is not None:
        try:
            X660 = CreateX660Controller(Config)
            X660.Open()
            print(
                "X6-60 motor/positioning unit enabled. "
                f"Mode={Config.get('X660Mode', 'x660-read-only')}"
            )
            InitialiseX660AtCurrentPose(X660, Config, Display)
        except Exception as exc:
            X660 = None
            print(f"X6-60 motor/positioning unit failed to open: {exc}")
            if str(Config.get("X660Mode", "")).lower() in (
                "x660-operational",
                "x6-60-operational",
                "x660-live",
                "x6-60-live",
            ):
                raise RuntimeError(
                    "Vanguard X startup aborted: operational X6-60 planner "
                    "limits were not verified; TX and mission motion remain "
                    "inhibited"
                ) from exc
    else:
        X660 = None
        if Config.get("EnableX660", False):
            print("X6-60 requested, but X660Controller.py could not be imported.")

    Source.Initialise()

    # -------------------------------------------------------------------------
    # Create the 2D target / reflector scene
    # -------------------------------------------------------------------------

    SceneObjects = create_default_scene()

    # -------------------------------------------------------------------------
    # Execute continuous scan
    # -------------------------------------------------------------------------
    #
    # The scan angle is held as explicit state rather than by iterating over a
    # fixed list. This is important for operator Stop/Start behaviour:
    #   - Stop holds the current antenna angle.
    #   - Start resumes from that same angle.
    #   - The dummy IMU follows this commanded angle, rather than free-running.
    # -------------------------------------------------------------------------

    DwellId = 1
    ScanCycle = 1
    ExitRequested = False

    ScanStartDeg = float(Config.get("ScanStartDeg", -60.0))
    ScanStopDeg = float(Config.get("ScanStopDeg", 60.0))
    ScanStepDeg = abs(float(Config.get("ScanStepDeg", 1.0)))

    CurrentScanBoresightDeg = float(Config.get("InitialBeamAngleDeg", Config.get("BoresightDeg", 0.0)))
    CurrentScanBoresightDeg = max(min(CurrentScanBoresightDeg, ScanStopDeg), ScanStartDeg)
    LastManualNudgeCommandId = 0
    LastScanEnabled = False
    LastDisplayMode = "STOP"
    # Latched until PointingManager actually receives the search task.  The GUI
    # loop runs faster than the radar-dwell loop, so a one-iteration edge flag
    # can otherwise be consumed by the dwell-rate gate (especially after a
    # manual nudge leaves PointingManager in STARE mode).
    ScanStartPending = False
    LastRadarDwellTimeSec = 0.0
    LastPrintedBoresightDeg = None

    # -------------------------------------------------------------------------
    # Radar operating-system architecture
    #
    # Stage 3C.2:
    # RadarScheduler selects the high-level task, RadarExecutor constructs and
    # executes each dwell, and PointingManager owns continuous search movement,
    # endpoint reversal, and STOP handling. Manual nudge remains in Main until
    # Stage 3C.3.
    # -------------------------------------------------------------------------

    Navigation = CreateNavigationSource(Config)

    Pointing = PointingManager(
        x660=X660,
        endpoint_margin_deg=float(
            Config.get("X660ScanEndpointMarginDeg", 1.0)
        ),
        position_tolerance_deg=float(
            Config.get("X660PositionToleranceDeg", 0.75)
        ),
        # The current software simulator reverses rate instantaneously, so
        # hardware braking anticipation is enabled only for the operational
        # CAN controller.
        scan_braking_enabled=bool(
            Config.get("X660ScanBrakingEnabled", True)
        ) and str(Config.get("X660Mode", "")).lower() in (
            "x660-operational",
            "x6-60-operational",
            "x660-live",
            "x6-60-live",
        ),
        scan_deceleration_deg_per_sec2=float(
            Config.get("X660ScanDecelerationDegPerSec2", 60.0)
        ),
        scan_command_latency_sec=float(
            Config.get("X660ScanCommandLatencySec", 0.05)
        ),
    )
    PointingControl = X660PointingControlLoop(
        interval_sec=float(
            Config.get("X660PointingControlIntervalSec", 0.020)
        ),
        debug=bool(Config.get("X660PointingControlDebug", False)),
    )
    LastPointingControlError = None

    SearchTask = MakeSearchTask(
        TaskId=1,
        SectorStartDeg=float(ScanStartDeg),
        SectorStopDeg=float(ScanStopDeg),
        ScanRateDegPerSec=float(
            Config.get("X660ScanSlewRateDegPerSec", 14.0)
        ),
        SectorFrame=AngleFrame.PLATFORM,
        Pattern=SearchPattern(
            str(Config.get("X660ScanPattern", "SECTOR")).upper()
        ),
    )

    # Mission execution remains simulation-only in this stage.  The
    # controller owns the immutable loaded snapshot and emits task intent; it
    # has no source, UHD, Qt, or motion dependency.
    MissionExecution = MissionExecutionController(Config)
    LastMissionStatusRevision = -1
    LastMissionTaskActivationRevision = 0

    Scheduler = RadarScheduler(
        search_task=SearchTask,
        debug=False,
    )

    Executor = RadarExecutor(
        source=Source,
        pointing_manager=Pointing,
        config=Config,
        waveform_library=TheWaveformLibrary,
        debug=False,
    )

    LastAppliedTimingRevision = -1
    LastSystemModeRevision = 0
    RestartSystemMode = None
    InitialControlState = (
        Display.GetControlState()
        if hasattr(Display, "GetControlState")
        else None
    )
    InitialTimingApplication = ApplyTimingControlState(
        Config=Config,
        ControlState=InitialControlState,
        Executor=Executor,
        SearchTask=SearchTask,
        LastAppliedRevision=LastAppliedTimingRevision,
    )
    if InitialTimingApplication.Changed:
        LastAppliedTimingRevision = InitialTimingApplication.Revision
        if not InitialTimingApplication.Applied:
            raise RuntimeError(
                "Initial operator timing selection is invalid: "
                f"{InitialTimingApplication.Message}"
            )
        SearchProfile = InitialTimingApplication.Profile
    else:
        SearchProfile = Executor.GetExecutionProfile(SearchTask)

    if hasattr(Display, "SetTimingApplicationResult"):
        Display.SetTimingApplicationResult(
            Applied=True,
            Message="Applied",
            Profile=SearchProfile,
        )
    if hasattr(Display, "SetMissionRuntimeStatus"):
        Display.SetMissionRuntimeStatus(MissionExecution.GetStatus())
        LastMissionStatusRevision = MissionExecution.StatusRevision

    SearchTiming = SearchProfile.Timing
    print(
        "Search timing: "
        f"waveform={SearchProfile.WaveformId}, "
        f"sample_rate={SearchProfile.SampleRate / 1e6:.1f} MS/s, "
        f"PRF={SearchTiming.SelectedPrfHz:.0f} Hz, "
        f"PRI={SearchTiming.PriSec * 1e6:.3f} us, "
        f"pulses={SearchTiming.PulsesPerCpi}, "
        f"CPI={SearchTiming.CpiDurationSec * 1e3:.3f} ms, "
        f"TX_ATR={SearchTiming.TxAtrEnvelopeDurationSec * 1e6:.3f} us, "
        f"RF_start={SearchTiming.RfPulseStartDelaySec * 1e6:.3f} us, "
        f"RX_start={SearchTiming.RxStartDelaySec * 1e6:.3f} us, "
        f"RX_samples={SearchTiming.NumRxSamples}, "
        f"max_range={SearchTiming.MaximumRangeM / 1e3:.3f} km"
    )

    try:
        while not ExitRequested:

                # -------------------------------------------------------------
                # Read display/operator controls.
                # -------------------------------------------------------------

                ControlState = (
                    Display.GetControlState()
                    if hasattr(Display, "GetControlState")
                    else None
                )

                if ControlState is not None:
                    RequestedModeRevision = int(
                        ControlState.get("SystemModeRevision", 0)
                    )
                    if RequestedModeRevision != LastSystemModeRevision:
                        LastSystemModeRevision = RequestedModeRevision
                        RequestedMode = str(
                            ControlState.get("RequestedSystemMode", SystemMode)
                        ).upper()
                        IsStopped = bool(
                            str(ControlState.get("DisplayMode", "STOP")) == "STOP"
                            and not ControlState.get("ScanEnabled", False)
                        )
                        if RequestedMode not in ("SIM", "HARD", "RF LOOPBACK"):
                            if hasattr(Display, "SetSystemModeApplicationResult"):
                                Display.SetSystemModeApplicationResult(
                                    False, "Invalid mode"
                                )
                        elif not IsStopped:
                            if hasattr(Display, "SetSystemModeApplicationResult"):
                                Display.SetSystemModeApplicationResult(
                                    False, "Stop radar first"
                                )
                        elif RequestedMode != SystemMode:
                            RestartSystemMode = RequestedMode
                            if hasattr(Display, "SetSystemModeApplicationResult"):
                                Display.SetSystemModeApplicationResult(
                                    True, f"Restarting in {RequestedMode}"
                                )
                            print(
                                f"System mode change requested: "
                                f"{SystemMode} -> {RequestedMode}"
                            )
                            break

                # -------------------------------------------------------------
                # Mission requests and duration transitions are consumed at the
                # top of the scheduler loop, after the preceding dwell has
                # completed and before any new timed work can be armed.
                # -------------------------------------------------------------

                MissionResult = MissionExecution.ApplyControlState(
                    ControlState,
                    system_mode=SystemMode,
                    now_sec=time.monotonic(),
                )
                ControlState = MissionExecution.BuildEffectiveControlState(
                    ControlState
                )
                if (
                    MissionExecution.StatusRevision
                    != LastMissionStatusRevision
                ):
                    LastMissionStatusRevision = (
                        MissionExecution.StatusRevision
                    )
                    MissionStatus = MissionExecution.GetStatus()
                    if hasattr(Display, "SetMissionRuntimeStatus"):
                        Display.SetMissionRuntimeStatus(MissionStatus)
                    print(
                        "Mission: "
                        f"{MissionStatus['State']} - "
                        f"{MissionStatus['Message']}"
                    )

                if (
                    MissionExecution.IsRunning
                    and MissionExecution.TaskActivationRevision
                    != LastMissionTaskActivationRevision
                ):
                    LastMissionTaskActivationRevision = (
                        MissionExecution.TaskActivationRevision
                    )
                    # A new primary task or Resume is a safe-boundary motion
                    # reactivation.  Stop the previous search intent before the
                    # updated SearchTask is handed to PointingManager below.
                    try:
                        Pointing.Stop()
                    except Exception:
                        pass
                    ScanStartPending = True

                TimingApplication = ApplyTimingControlState(
                    Config=Config,
                    ControlState=ControlState,
                    Executor=Executor,
                    SearchTask=SearchTask,
                    LastAppliedRevision=LastAppliedTimingRevision,
                )
                if TimingApplication.Changed:
                    LastAppliedTimingRevision = TimingApplication.Revision
                    if hasattr(Display, "SetTimingApplicationResult"):
                        Display.SetTimingApplicationResult(
                            Applied=TimingApplication.Applied,
                            Message=TimingApplication.Message,
                            Profile=TimingApplication.Profile,
                        )
                    if TimingApplication.Applied:
                        AppliedTiming = TimingApplication.Profile.Timing
                        print(
                            "Operator timing applied: "
                            f"waveform={TimingApplication.Profile.WaveformId}, "
                            f"PRF={AppliedTiming.SelectedPrfHz:.0f} Hz, "
                            f"pulses={AppliedTiming.PulsesPerCpi}, "
                            f"CPI={AppliedTiming.CpiDurationSec * 1e3:.3f} ms, "
                            f"max_range={AppliedTiming.MaximumRangeM / 1e3:.3f} km"
                        )
                    else:
                        print(
                            "Operator timing rejected: "
                            f"{TimingApplication.Message}"
                        )
                        if MissionExecution.IsRunning:
                            MissionExecution.Fault(
                                "Mission timing rejected: "
                                f"{TimingApplication.Message}",
                                now_sec=time.monotonic(),
                            )
                            ControlState = (
                                MissionExecution.BuildEffectiveControlState(
                                    ControlState
                                )
                            )
                            LastMissionStatusRevision = (
                                MissionExecution.StatusRevision
                            )
                            if hasattr(Display, "SetMissionRuntimeStatus"):
                                Display.SetMissionRuntimeStatus(
                                    MissionExecution.GetStatus()
                                )

                # -------------------------------------------------------------
                # Apply operator data-logging controls from the display.
                # Save checkbox ON  -> open/log to the requested HDF5 file.
                # Save checkbox OFF -> close file and stop logging.
                # -------------------------------------------------------------

                Logger.update_control_state(ControlState)

                # -------------------------------------------------------------
                # Exit cleanly if requested from the display.
                # -------------------------------------------------------------

                if ControlState is not None and ControlState.get("ExitRequested", False):
                    print("")
                    print("Exit requested from display.")
                    ExitRequested = True
                    break

                # -------------------------------------------------------------
                # Select commanded scan/stare target.
                # -------------------------------------------------------------

                CommandedBoresightDeg = GetControlledBoresightDeg(
                    Display=Display,
                    ScanBoresightDeg=CurrentScanBoresightDeg,
                    ControlState=ControlState,
                )

                PreviousScanEnabled = bool(LastScanEnabled)

                if ControlState is not None:
                    DisplayMode = str(ControlState.get("DisplayMode", "STOP"))
                    ScanEnabled = bool(ControlState.get("ScanEnabled", False))
                    ScanStartDeg = float(ControlState.get("ScanStartDeg", ScanStartDeg))
                    ScanStopDeg = float(ControlState.get("ScanStopDeg", ScanStopDeg))
                    ScanStepDeg = abs(float(ControlState.get("ScanStepDeg", ScanStepDeg)))
                    if "ScanRateDegPerSec" in ControlState:
                        RequestedDashboardRate = abs(float(
                            ControlState["ScanRateDegPerSec"]
                        ))
                        MinimumDashboardRate = float(
                            Config.get("MinScanRateDegPerSec", 1.0)
                        )
                        MaximumDashboardRate = float(
                            Config.get(
                                "X660OperationalMaxRateDegPerSec",
                                60.0,
                            )
                        )
                        Config["X660ScanSlewRateDegPerSec"] = min(
                            max(RequestedDashboardRate, MinimumDashboardRate),
                            MaximumDashboardRate,
                        )

                    Config["ScanStartDeg"] = ScanStartDeg
                    Config["ScanStopDeg"] = ScanStopDeg
                    Config["ScanStepDeg"] = ScanStepDeg
                else:
                    DisplayMode = "STOP"
                    ScanEnabled = False

                ScanJustStarted = bool(
                    DisplayMode == "SCAN"
                    and ScanEnabled
                    and not (LastDisplayMode == "SCAN" and PreviousScanEnabled)
                )
                if ScanJustStarted:
                    ScanStartPending = True
                elif DisplayMode != "SCAN" or not ScanEnabled:
                    # Cancel an unissued start if the operator stops again
                    # before the next eligible radar dwell.
                    ScanStartPending = False

                # -------------------------------------------------------------
                # Apply operator pointing intent before any RF/dwell gate.
                # Nudge and STARE/STOP transitions must work while TX is off.
                # This path issues commands only on a new nudge ID or mode
                # transition, so it does not add continuous X660 serial traffic.
                # -------------------------------------------------------------
                try:
                    (
                        LastManualNudgeCommandId,
                        PointingAction,
                        NudgeTargetRelativeDeg,
                    ) = ApplyOperatorPointingCommand(
                        Pointing=Pointing if X660 is not None else None,
                        ControlState=ControlState,
                        DisplayMode=DisplayMode,
                        ScanEnabled=ScanEnabled,
                        LastDisplayMode=LastDisplayMode,
                        LastScanEnabled=LastScanEnabled,
                        LastManualNudgeCommandId=(
                            LastManualNudgeCommandId
                        ),
                    )
                    if PointingAction == "NUDGE":
                        print(
                            "Operator nudge: "
                            f"{float(NudgeTargetRelativeDeg):.2f} deg relative"
                        )
                except Exception as exc:
                    if ControlState is not None:
                        LastManualNudgeCommandId = int(
                            ControlState.get(
                                "ManualNudgeCommandId",
                                LastManualNudgeCommandId,
                            )
                        )
                    print(f"Operator pointing command failed: {exc}")

                # Consume the operator mode transition immediately.  Do not
                # wait until the next radar dwell, otherwise the fast GUI loop
                # can issue the same X660 Stop command repeatedly while the
                # dwell-rate throttle is active.
                LastScanEnabled = bool(ScanEnabled)
                LastDisplayMode = str(DisplayMode)

                # X6-60 endpoint control is independent of radar dwells.  This
                # single-threaded 50 Hz tick reads fresh encoder angle/speed and
                # lets PointingManager issue a braking/reversal command as soon
                # as the calculated threshold is reached.  Missed periods are
                # skipped, never replayed as a burst of CAN transactions.
                if X660 is not None:
                    try:
                        FastPointingState = PointingControl.TickIfDue(
                            pointing=Pointing,
                            navigation=Navigation.get_pose(),
                            x660=X660,
                            display=Display,
                            config=Config,
                            now_monotonic_sec=time.monotonic(),
                        )
                        if FastPointingState is not None:
                            CurrentScanBoresightDeg = float(
                                FastPointingState.BeamBearingTrueDeg
                            ) % 360.0
                            LastPointingControlError = None
                    except Exception as exc:
                        PointingControlError = str(exc)
                        Config["X660Valid"] = False
                        Config["X660Source"] = (
                            f"POINTING CONTROL ERROR: {PointingControlError}"
                        )
                        if PointingControlError != LastPointingControlError:
                            print(
                                "X6-60 50 Hz pointing-control update failed: "
                                f"{PointingControlError}"
                            )
                            LastPointingControlError = PointingControlError

                # -------------------------------------------------------------
                # Timed radar dwell scheduler.
                #
                # The active-dwell X6-60 state snapshot remains after this gate
                # for dwell diagnostics.  Its adapter-level query is throttled,
                # so it normally reuses the fresh 50 Hz state without another
                # CAN transaction.
                # -------------------------------------------------------------

                NowDwellSec = time.time()
                RadarDwellIntervalSec = float(Config.get("RadarDwellIntervalSec", 0.10))
                DwellWallDtSec = NowDwellSec - LastRadarDwellTimeSec if LastRadarDwellTimeSec > 0.0 else 0.0

                TransmitEnabled = True if ControlState is None else bool(ControlState.get("TransmitEnabled", True))
                RadarDwellEnabled = bool(
                    TransmitEnabled or ReceiveOnlyOperation
                )

                if DisplayMode == "STOP" or not RadarDwellEnabled:
                    if hasattr(Display, "App"):
                        Display.App.processEvents()
                    time.sleep(0.005)
                    continue

                if LastRadarDwellTimeSec > 0.0 and DwellWallDtSec < RadarDwellIntervalSec:
                    # Between dwell instants, keep the GUI responsive.  The
                    # independent 50 Hz block above continues X6-60 telemetry
                    # and endpoint control while radar work remains at 10 Hz.
                    if hasattr(Display, "App"):
                        Display.App.processEvents()
                    time.sleep(0.002)
                    continue

                LastRadarDwellTimeSec = NowDwellSec
                NavigationPose = Navigation.get_pose()

                # -------------------------------------------------------------
                # X660 / X6-60 state and operator controls.
                #
                # Continuous SCAN, endpoint reversal, STOP, and manual nudge are
                # now owned exclusively by PointingManager. Main only forwards
                # operator intent and consumes measured pointing state.
                # -------------------------------------------------------------

                X660Valid = False
                X660Source = "DISABLED"
                X660AzDeg = float(CommandedBoresightDeg)
                X660RateDegPerSec = 0.0
                X660AtTarget = False
                X660RawAngleDeg = None

                if X660 is not None:
                    try:
                        # Query/update position. This is throttled inside X660Controller.
                        X660State = X660.Update()
                        X660AzDeg = float(X660State.AzimuthDeg)
                        X660RateDegPerSec = float(X660State.PanRateDegPerSec)
                        X660Valid = bool(X660State.Valid)
                        X660Source = str(X660State.Source)
                        X660AtTarget = bool(
                            getattr(X660State, "AtTarget", False)
                        )
                        X660RawAngleDeg = getattr(
                            X660State,
                            "RawAngleDeg",
                            None,
                        )

                        if X660Valid:
                            CurrentScanBoresightDeg = (
                                Pointing.RelativeToTrueBearing(
                                    X660AzDeg,
                                    NavigationPose.HeadingTrueDeg,
                                )
                            )
                            if hasattr(Display, "BeamAngleDeg"):
                                Display.BeamAngleDeg = float(
                                    CurrentScanBoresightDeg
                                )

                    except Exception as exc:
                        X660Valid = False
                        X660Source = f"ERROR: {exc}"

                LastScanEnabled = bool(ScanEnabled)
                LastDisplayMode = str(DisplayMode)

                # -------------------------------------------------------------
                # Read actual antenna AZ/EL from the IMU once per dwell.
                # -------------------------------------------------------------

                MeasuredAntennaAzDeg = float(CommandedBoresightDeg)
                MeasuredAntennaElDeg = 0.0
                ImuValid = False
                ImuSource = "DISABLED"

                if Imu is not None:
                    try:
                        if hasattr(Imu, "SetSimulatedAntennaPosition"):
                            Imu.SetSimulatedAntennaPosition(
                                AzimuthDeg=float(X660AzDeg if X660Valid else CommandedBoresightDeg),
                                ElevationDeg=float(Config.get("AntennaElDeg", 0.0)),
                            )

                        ImuData = Imu.Read()
                        MeasuredAntennaAzDeg = float(ImuData.AzimuthDeg)
                        MeasuredAntennaElDeg = float(ImuData.ElevationDeg)
                        ImuValid = bool(ImuData.Valid)
                        ImuSource = str(ImuData.Source)
                    except Exception as exc:
                        ImuValid = False
                        ImuSource = f"ERROR: {exc}"

                # Boresight source priority:
                #   IMU if explicitly enabled and valid,
                #   else X660 measured position,
                #   else commanded fallback.
                if Config.get("UseIMUForBeamAngle", False) and ImuValid:
                    BoresightDeg = float(MeasuredAntennaAzDeg)
                elif X660Valid:
                    BoresightDeg = Pointing.RelativeToTrueBearing(
                        X660AzDeg,
                        NavigationPose.HeadingTrueDeg,
                    )
                else:
                    BoresightDeg = float(CommandedBoresightDeg)

                Config["BoresightDeg"] = float(BoresightDeg)
                Config["CommandedBoresightDeg"] = float(CommandedBoresightDeg)
                Config["AntennaAzDeg"] = float(MeasuredAntennaAzDeg)
                Config["AntennaElDeg"] = float(MeasuredAntennaElDeg)
                Config["IMUValid"] = bool(ImuValid)
                Config["IMUSource"] = str(ImuSource)
                Config["X660AzDeg"] = float(X660AzDeg)
                Config["X660BeamBearingTrueDeg"] = float(BoresightDeg)
                Config["X660RateDegPerSec"] = float(X660RateDegPerSec)
                Config["X660Valid"] = bool(X660Valid)
                Config["X660Source"] = str(X660Source)
                Config["X660AtTarget"] = bool(X660AtTarget)
                Config["X660RawAngleDeg"] = X660RawAngleDeg
                Config["ScanCycle"] = int(ScanCycle)

                # -------------------------------------------------------------
                # Build the scene returns for this beam position.
                # -------------------------------------------------------------

                RadarParams = BuildRadarParamsForScenario(
                    Config,
                    NavigationPose,
                )

                SceneReturns = build_scene_returns_for_boresight(
                    SceneObjects,
                    RadarParams,
                )

                # Pass the current boresight scene returns into the simulated
                # source. SimulatedSource will insert all of these returns into
                # the received IQ.
                Config["SceneReturns"] = SceneReturns

                if Config["PrintSceneTruthTable"]:
                    print_scene_returns(SceneReturns, BoresightDeg)

                # -------------------------------------------------------------
                # Stage 3C.2: RadarScheduler selects the task. RadarExecutor
                # executes it, while PointingManager owns continuous scan slew,
                # endpoint reversal, and stop handling for the X6-60.
                # -------------------------------------------------------------

                NavigationAttitude = NavigationPose

                # Keep the persistent scheduler search task aligned with the
                # operator-selected sector and scan rate.
                SearchTask.Sector.StartDeg = float(ScanStartDeg)
                SearchTask.Sector.StopDeg = float(ScanStopDeg)
                RequestedScanRateDegPerSec = (
                    abs(float(ControlState["MissionScanRateDegSec"]))
                    if (
                        ControlState is not None
                        and "MissionScanRateDegSec" in ControlState
                    )
                    else abs(float(
                        Config.get("X660ScanSlewRateDegPerSec", 20.0)
                    ))
                )
                SearchTask.Sector.ScanRateDegPerSec = min(
                    RequestedScanRateDegPerSec,
                    float(Config.get(
                        "X660OperationalMaxRateDegPerSec",
                        60.0,
                    )),
                )
                RequestedScanPattern = str(
                    ControlState.get(
                        "MissionScanPattern",
                        Config.get("X660ScanPattern", "SECTOR"),
                    )
                    if ControlState is not None
                    else Config.get("X660ScanPattern", "SECTOR")
                ).upper()
                SearchTask.Sector.Pattern = SearchPattern(
                    RequestedScanPattern
                )

                # Pointing.Stop() clears the active task. Reissue the search
                # command on every transition into active SCAN, including from
                # STOP or STARE when ScanEnabled remained true.
                if ScanStartPending:
                    # A task change starts a fresh measured-crossing history.
                    # This prevents the final sector sample from being treated
                    # as a North crossing in a new continuous task.
                    SearchTask.Sector.LastMeasuredAzimuthDeg = None
                    if SearchTask.Sector.Pattern == SearchPattern.SECTOR:
                        print(
                            f"PointingManager sector scan start: "
                            f"{ScanStartDeg:.2f} -> {ScanStopDeg:.2f}"
                        )
                    else:
                        print(
                            "PointingManager continuous scan start: "
                            f"{SearchTask.Sector.Pattern.value} at "
                            f"{SearchTask.Sector.ScanRateDegPerSec:.2f} deg/s"
                        )
                    Pointing.ActivateTask(SearchTask, NavigationAttitude)
                    ScanStartPending = False

                ScheduledTask = Scheduler.GetNextTask(
                    current_time_sec=NowDwellSec,
                )

                if ScheduledTask is None:
                    if hasattr(Display, "App"):
                        Display.App.processEvents()
                    time.sleep(0.002)
                    continue

                # -------------------------------------------------------------
                # Execute one complete radar dwell through the extracted helper.
                # -------------------------------------------------------------

                DwellResult = ExecuteRadarDwell(
                    Config=Config,
                    ScheduledTask=ScheduledTask,
                    Executor=Executor,
                    NavigationAttitude=NavigationAttitude,
                    Processor=Processor,
                    Detector=Detector,
                    Tracker=Tracker,
                    Display=Display,
                    Logger=Logger,
                    ScanCycle=ScanCycle,
                    DisplayMode=DisplayMode,
                    BoresightDeg=BoresightDeg,
                    CommandedBoresightDeg=CommandedBoresightDeg,
                    MeasuredAntennaAzDeg=MeasuredAntennaAzDeg,
                    MeasuredAntennaElDeg=MeasuredAntennaElDeg,
                    ImuValid=ImuValid,
                    ImuSource=ImuSource,
                    X660AzDeg=X660AzDeg,
                    X660RateDegPerSec=X660RateDegPerSec,
                    X660Valid=X660Valid,
                    X660Source=X660Source,
                    X660AtTarget=X660AtTarget,
                    X660RawAngleDeg=X660RawAngleDeg,
                )

                if not DwellResult["Executed"]:
                    if hasattr(Display, "App"):
                        Display.App.processEvents()
                    time.sleep(0.002)
                    continue

                ScanCycle = int(DwellResult["ScanCycle"])
                ThisDwell = DwellResult["Dwell"]
                Processed = DwellResult["Processed"]
                Detections = DwellResult["Detections"]
                Tracks = DwellResult["Tracks"]
                Plots = DwellResult["Plots"]
                TrackerDebug = DwellResult["TrackerDebug"]
                T0 = DwellResult["T0"]
                T1 = DwellResult["T1"]
                T2 = DwellResult["T2"]
                T3 = DwellResult["T3"]
                T4 = DwellResult["T4"]
                T5 = DwellResult["T5"]

                # One scheduler task currently corresponds to one completed
                # legacy dwell. SEARCH is persistent, so completing it simply
                # returns it to the queued state for the next dwell.
                Scheduler.CompleteActiveTask(
                    current_time_sec=time.time(),
                )

                # Mission duration is active surveillance time.  Account it
                # only after this complete CPI/dwell, then publish the latest
                # task/remaining-time status.  Any task transition selected
                # here is applied before the next dwell is armed.
                MissionExecution.Advance(time.monotonic())
                if hasattr(Display, "SetMissionRuntimeStatus"):
                    Display.SetMissionRuntimeStatus(
                        MissionExecution.GetStatus()
                    )

                if DwellId % 1 == 0:
                    TimedTransportSummary = ""
                    if Config.get("Stage3E1LoopbackActive", False):
                        TargetSummary = ""
                        if Processed.Diagnostics.get(
                            "RfTargetEmulatorActive", False
                        ):
                            TargetName = str(
                                Processed.Diagnostics.get(
                                    "RfTargetName", "Target"
                                )
                            )
                            TargetSummary = (
                                " TARGET "
                                f"{TargetName} "
                                f"{float(Processed.Diagnostics.get('RfTargetRangeM', 0.0)) / 1000.0:.2f}km"
                            )
                        TimedTransportSummary = (
                            " | TX "
                            f"{int(Processed.Diagnostics.get('TransmitCommandCount', 0))} "
                            "ACK "
                            f"{int(Processed.Diagnostics.get('TransmitBurstAcknowledgementCount', 0))}"
                            f"{TargetSummary}"
                        )
                    if LastPrintedBoresightDeg is None:
                        PrintedAzStepDeg = 0.0
                    else:
                        PrintedAzStepDeg = BoresightDeg - LastPrintedBoresightDeg
                        while PrintedAzStepDeg > 180.0:
                            PrintedAzStepDeg -= 360.0
                        while PrintedAzStepDeg < -180.0:
                            PrintedAzStepDeg += 360.0
                    LastPrintedBoresightDeg = float(BoresightDeg)

                    print(
                        f"Dwell {DwellId:5d} | "
                        f"WallDt {1000*DwellWallDtSec:7.2f} ms | "
                        f"AzStep {PrintedAzStepDeg:6.2f} deg | "
                        f"Collect {1000*(T1-T0):7.2f} ms | "
                        f"Process {1000*(T2-T1):7.2f} ms | "
                        f"Detect+Track {1000*(T3-T2):7.2f} ms | "
                        f"Log {1000*(T4-T3):7.2f} ms | "
                        f"Display {1000*(T5-T4):7.2f} ms | "
                        f"Total {1000*(T5-T0):7.2f} ms | "
                        f"X6-60 {X660AzDeg:7.2f} deg {X660Source} | "
                        f"Mode {DisplayMode} Scan {ScanEnabled} | "
                        f"Dets {len(Detections):3d} Plots {len(Plots):3d} Tracks {len(Tracks):3d} | "
                        f"Pts {int(TrackerDebug.get('CurrentScanPoints', 0)):3d} "
                        f"Blobs {int(TrackerDebug.get('LastCompletedBlobs', 0)):3d} "
                        f"Tent {int(TrackerDebug.get('TentativeTracks', 0)):3d} "
                        f"Conf {int(TrackerDebug.get('ConfirmedTracks', 0)):3d} "
                        f"ScanCycle {ScanCycle}"
                        f"{TimedTransportSummary}"
                    )


                # -------------------------------------------------------------
                # Move targets forward by one dwell time.
                # -------------------------------------------------------------

                DwellTimeS = ThisDwell.NumPulses * ThisDwell.PRI
                update_scene_objects(SceneObjects, DwellTimeS)

                DwellId += 1

                # -------------------------------------------------------------
                # Update scan limits from display/operator controls.
                #
                # IMPORTANT:
                # Do not free-run CurrentScanBoresightDeg here. PointingManager
                # advances the search sector only after measured X6-60 pointing
                # reaches or crosses the active endpoint.
                # -------------------------------------------------------------

                ScanStartDeg = float(Config.get("ScanStartDeg", ScanStartDeg))
                ScanStopDeg = float(Config.get("ScanStopDeg", ScanStopDeg))
                ScanStepDeg = abs(float(Config.get("ScanStepDeg", ScanStepDeg)))



    except KeyboardInterrupt:
        print("")
        print("Scan stopped by user.")

    finally:
        # ---------------------------------------------------------------------
        # Shutdown source
        # ---------------------------------------------------------------------

        try:
            Logger.close()
        except Exception:
            pass

        try:
            if X660 is not None:
                Pointing.Stop()
                X660.Close()
        except Exception:
            pass

        try:
            if hasattr(Display, "Shutdown"):
                Display.Shutdown()
        except Exception:
            pass

        Source.Shutdown()

    if RestartSystemMode is not None:
        ModeArgument = {
            "HARD": "--system-hard",
            "RF LOOPBACK": "--system-rf-loopback",
        }.get(RestartSystemMode, "--system-sim")
        RestartArguments = [ModeArgument]
        if RestartSystemMode == "RF LOOPBACK":
            RestartArguments.append("--i-confirm-rf-loopback-safety")
        if str(Config.get("DisplayTransport", "LOCAL_QT")).upper() == "TCP_SERVER":
            RestartArguments.extend([
                "--remote-ui",
                "--radar-link-host", str(Config.get("RadarLinkHost", "127.0.0.1")),
                "--radar-link-port", str(int(Config.get("RadarLinkPort", 5810))),
            ])
        os.execv(
            sys.executable,
            [sys.executable, os.path.abspath(__file__)] + RestartArguments,
        )


if __name__ == "__main__":
    Main()
