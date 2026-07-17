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
from NavigationState import SimulatedNavigationSource
from PointingManager import PointingManager
from RadarExecutor import RadarExecutor
from RadarScheduler import RadarScheduler
from RadarTasks import AngleFrame, MakeSearchTask, RadarTaskType

from SimulatedSource import SimulatedSource
from WaveformLibrary import WaveformLibrary
from RadarProcessor import RadarProcessor
from CfarDetector import CfarDetector
from RadarTracker import RadarTracker
from SimpleDisplay import SimpleDisplay
from RadarDisplayQt5 import RadarDisplay
from DataLogger import DataLogger
from EttusRadarSource import EttusRadarSource
import time

try:
    from ReadIMU import IMUReader
except Exception:
    IMUReader = None

try:
    from PTZController import CreatePTZController
except Exception:
    CreatePTZController = None

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

def BuildRadarParamsForScenario(Config):
    """
    Convert the main Config dictionary into the lower-case keys used by
    TargetScenario.py.
    """

    return {
        "carrier_frequency_hz": Config["RfFrequency"],
        "radar_x_m": Config.get("RadarXM", 0.0),
        "radar_y_m": Config.get("RadarYM", 0.0),
        "boresight_deg": Config["BoresightDeg"],
        "beamwidth_deg": Config["BeamwidthDeg"],
        "reference_range_m": Config.get("ReferenceRangeM", 8000.0),
        "target_amplitude_scale": Config.get("TargetAmplitudeScale", 1.0),
        "sidelobe_floor_db": Config.get("SidelobeFloorDb", -50.0),
    }


def SelectDisplay(Config):
    """
    Create the selected display.

    RadarDisplay is the new operator-style display with SCAN/STARE controls.
    SimpleDisplay is kept as a fallback engineering display.
    """

    if Config.get("DisplayType", "SimpleDisplay") == "RadarDisplay":
        return RadarDisplay(Config)

    return SimpleDisplay(Config)


def ClampDeg(Value, MinValue, MaxValue):
    return max(float(MinValue), min(float(MaxValue), float(Value)))




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



def InitialisePTZToStartupPose(Ptz, Config, Display=None):
    """Command the PTZ to the configured safe startup AZ/EL pose.

    Default startup pose is reported EL=55 deg and AZ=200 deg.  This runs before the
    radar dwell loop so the antenna starts from a known mechanical attitude.
    """
    if Ptz is None or not bool(Config.get("PTZStartupEnabled", True)):
        return

    TargetAzDeg = ClampDeg(
        float(Config.get("PTZStartupAzimuthDeg", 200.0)),
        float(Config.get("PTZLeftLimitDeg", 10.0)),
        float(Config.get("PTZRightLimitDeg", 300.0)),
    )
    TargetElDeg = ClampDeg(float(Config.get("PTZStartupElevationDeg", 45.0)), -90.0, 90.0)
    TimeoutSec = float(Config.get("PTZStartupTimeoutSec", 20.0))
    ToleranceDeg = float(Config.get("PTZStartupPositionToleranceDeg", Config.get("PTZPositionToleranceDeg", 1.0)))

    print(f"PTZ startup initialise: AZ={TargetAzDeg:.2f} deg, EL={TargetElDeg:.2f} deg")

    if hasattr(Ptz, "CommandPosition"):
        Ptz.CommandPosition(TargetAzDeg, TargetElDeg)
    else:
        if hasattr(Ptz, "SetPanPositionNative"):
            Ptz.SetPanPositionNative(TargetAzDeg)
        if hasattr(Ptz, "SetTiltPositionNative"):
            Ptz.SetTiltPositionNative(TargetElDeg)

    StartSec = time.time()
    LastPrintSec = 0.0

    while (time.time() - StartSec) < TimeoutSec:
        try:
            State = Ptz.Update() if hasattr(Ptz, "Update") else Ptz.GetState()
            AzDeg = float(getattr(State, "AzimuthDeg", TargetAzDeg))
            ElDeg = float(getattr(State, "ElevationDeg", TargetElDeg))
        except Exception as exc:
            print(f"PTZ startup initialise warning: {exc}")
            break

        if hasattr(Display, "App"):
            Display.App.processEvents()

        AzOk = abs(AzDeg - TargetAzDeg) <= ToleranceDeg
        ElOk = abs(ElDeg - TargetElDeg) <= ToleranceDeg

        NowSec = time.time()
        if NowSec - LastPrintSec > 1.0:
            print(f"PTZ startup position: AZ={AzDeg:.2f} deg, EL={ElDeg:.2f} deg")
            LastPrintSec = NowSec

        if AzOk and ElOk:
            break

        time.sleep(0.05)

    try:
        Ptz.Stop()
    except Exception:
        pass

    Config["InitialBeamAngleDeg"] = TargetAzDeg
    Config["BoresightDeg"] = TargetAzDeg
    Config["AntennaElDeg"] = TargetElDeg
    if hasattr(Display, "BeamAngleDeg"):
        Display.BeamAngleDeg = TargetAzDeg





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
    PtzAzDeg,
    PtzRateDegPerSec,
    PtzValid,
    PtzSource,
    PtzAtTarget,
):
    """Execute one scheduled dwell and preserve the processing chain.

    RadarExecutor owns DwellPlan construction and source execution.
    PointingManager is the sole owner of continuous X6-60 search-scan movement.
    """

    T0 = time.perf_counter()
    ExecutionResult = Executor.ExecuteTaskStep(
        task=ScheduledTask,
        navigation=NavigationAttitude,
    )
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
    Processed.Diagnostics["PTZAzDeg"] = float(PtzAzDeg)
    Processed.Diagnostics["PTZRateDegPerSec"] = float(PtzRateDegPerSec)
    Processed.Diagnostics["PTZValid"] = bool(PtzValid)
    Processed.Diagnostics["PTZSource"] = str(PtzSource)
    Processed.Diagnostics["PTZAtTarget"] = bool(PtzAtTarget)
    Processed.Diagnostics["ScanCycle"] = EffectiveScanCycle
    Processed.Diagnostics["ScheduledTaskId"] = int(ScheduledTask.TaskId)
    Processed.Diagnostics["ScheduledTaskType"] = str(ScheduledTask.TaskType)
    Processed.Diagnostics["ScheduledWaveformProfileId"] = str(
        getattr(ScheduledTask, "WaveformProfileId", "")
    )

    Detections = Detector.Detect(Processed, ThisDwell)

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

def Main():
    """
    Main radar program.

    This function creates the configuration, initialises the radar modules,
    creates a target scene, scans the radar across a sector, executes a dwell at
    each scan angle, processes the result, detects targets, and displays output.
    """

    # -------------------------------------------------------------------------
    # Configuration dictionary
    # -------------------------------------------------------------------------

    Config = {
        "NumSamples": 4096,
        "NumPulses": 32,
        "PRI": 200e-6,

        # ---------------------------------------------------------------------
        # Radar source
        # ---------------------------------------------------------------------

        "RadarSource": "ETTUS",      # "SIM" or "ETTUS"

        # Ettus configuration
        "EttusSerial": "34A0320",
        "EttusRxFrequencyHz": 1.0e9,
        "EttusRxGainDb": 10.0,
        "EttusRxAntenna": "RX2",
        "EttusRxChannel": 0,
        "EttusSampleRateHz": 40.0e6,
        "EttusMaxSampleRateHz": 40.0e6,
        "EttusReceiveTimeoutSec": 1.0,
        "EttusCommandLeadTimeSec": 0.005,
        "EttusDebug": True,

        # Initial pulse-plan architecture. Search and track waveform selectors
        # are separate even though only SEARCH is scheduled in this version.
        "SearchWaveformId": "Frank10",
        "TrackWaveformId": "Barker13",

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
        "MinRangeM": 100.0,
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

        "RangeDopplerUpdateEveryNDwells": 0,
        "PolarUpdateEveryNDwells": 1,
        "PolarDetectionsUpdateEveryNDwells": 1,
        "RangeProfileUpdateEveryNDwells": 1,
        "StatusUpdateEveryNDwells": 1,
        "QtProcessEventsEveryNDwells": 1,
        "QtRangeProfileDecimation": 1,

        # ---------------------------------------------------------------------
        # Operator display / scan controls
        # ---------------------------------------------------------------------
        "InitialDisplayMode": "STOP",
        "InitialScanEnabled": False,
        "InitialBeamAngleDeg": 200.0,
        "ManualBeamStepDeg": 1.0,

        # PTZ controls "pelco" or "sim"
        "EnablePTZ": True,
        "PTZMode": "sim", #"pelco"
        "PTZPort": "/dev/ttyACM0",
        "PTZBaudRate": 9600,
        "PTZAddress": 1,
        "PTZPanSpeed": 0x5F,
        "PTZTiltSpeed": 0x3F,
        "PTZLeftLimitDeg": 10.0,
        "PTZRightLimitDeg": 300.0,
        "PTZLimitMarginDeg": 1.0,
        "PTZTimeoutSec": 0.3,
        "PTZQueryIntervalSec": 0.10,
        "PTZQueryTiltInUpdate": False,
        "PTZDebug": False,
        "PTZPositionToleranceDeg": 0.75,
        "PTZScanReverseLockoutSec": 0.8,
        "PTZScanEndpointMarginDeg": 1.0,
        "PTZScanSlewRateDegPerSec": 14.0,
        "PTZSimPanRateDegPerSec": 14.0,
        "PTZSimWrapMode": False,
        "RadarDwellIntervalSec": 0.10,
        "PTZScanContinuousToEndpoint": True,
        "PTZStartupEnabled": True,
        "PTZStartupAzimuthDeg": 200.0,
        "PTZStartupElevationDeg": 60.0,
        "PTZStartupTimeoutSec": 20.0,
        "PTZStartupPositionToleranceDeg": 1.0,

        # Antenna attitude / IMU controls
        "EnableIMU": False,
        "UseIMUForBeamAngle": False,
        "IMUDummyMode": True,
        "IMUAzimuthOffsetDeg": 0.0,
        "IMUElevationOffsetDeg": 0.0,
        "IMUInvertAzimuth": False,
        "IMUInvertElevation": False,
    }

    # -------------------------------------------------------------------------
    # Waveform library
    # -------------------------------------------------------------------------

    TheWaveformLibrary = WaveformLibrary(Config)
    TheWaveformLibrary.LoadDefaultWaveforms()

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

    # PTZ controller. In hardware mode this opens /dev/ttyACM0 and uses
    # native Pelco-D pan position commands.
    if Config.get("EnablePTZ", False) and CreatePTZController is not None:
        try:
            Ptz = CreatePTZController(Config)
            Ptz.Open()
            print(f"PTZ controller enabled. Mode={Config.get('PTZMode', 'pelco')}")
            InitialisePTZToStartupPose(Ptz, Config, Display)
        except Exception as exc:
            Ptz = None
            print(f"PTZ requested but failed to open: {exc}")
    else:
        Ptz = None
        if Config.get("EnablePTZ", False):
            print("PTZ requested, but PTZController.py could not be imported.")

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
    LastStopCommandId = 0
    PtzStopped = False
    LastScanEnabled = False
    LastDisplayMode = "STOP"
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

    Navigation = SimulatedNavigationSource(
        initial_heading_deg=0.0,
    )

    Pointing = PointingManager(
        ptz=Ptz,
        left_limit_deg=float(Config.get("PTZLeftLimitDeg", 10.0)),
        right_limit_deg=float(Config.get("PTZRightLimitDeg", 300.0)),
        endpoint_margin_deg=float(
            Config.get("PTZScanEndpointMarginDeg", 1.0)
        ),
        position_tolerance_deg=float(
            Config.get("PTZPositionToleranceDeg", 0.75)
        ),
    )

    SearchTask = MakeSearchTask(
        TaskId=1,
        SectorStartDeg=float(ScanStartDeg),
        SectorStopDeg=float(ScanStopDeg),
        ScanRateDegPerSec=float(
            Config.get("PTZScanSlewRateDegPerSec", 14.0)
        ),
        SectorFrame=AngleFrame.PLATFORM,
    )

    Scheduler = RadarScheduler(
        search_task=SearchTask,
        debug=False,
    )

    Executor = RadarExecutor(
        source=Source,
        pointing_manager=Pointing,
        config=Config,
        debug=False,
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

                # -------------------------------------------------------------
                # Timed radar dwell scheduler.
                #
                # IMPORTANT:
                # This gate is deliberately BEFORE the PTZ serial query/update.
                # The Pelco-D query can block on serial timeout, so putting the
                # gate after Ptz.Update() makes the radar dwell rate depend on
                # PTZ serial latency.  Gate first, then do one PTZ update and
                # one radar dwell when a dwell is actually due.
                # -------------------------------------------------------------

                NowDwellSec = time.time()
                RadarDwellIntervalSec = float(Config.get("RadarDwellIntervalSec", 0.10))
                DwellWallDtSec = NowDwellSec - LastRadarDwellTimeSec if LastRadarDwellTimeSec > 0.0 else 0.0

                TransmitEnabled = True if ControlState is None else bool(ControlState.get("TransmitEnabled", True))

                if DisplayMode == "STOP" or not TransmitEnabled:
                    # If the operator stops the radar, stop the PTZ immediately;
                    # do not wait for the next dwell slot.
                    if Ptz is not None and (LastDisplayMode != "STOP" or LastScanEnabled):
                        try:
                            Pointing.Stop()
                        except Exception:
                            pass

                    LastScanEnabled = bool(ScanEnabled)
                    LastDisplayMode = str(DisplayMode)

                    if hasattr(Display, "App"):
                        Display.App.processEvents()
                    time.sleep(0.005)
                    continue

                if LastRadarDwellTimeSec > 0.0 and DwellWallDtSec < RadarDwellIntervalSec:
                    # Between dwell instants, keep the GUI responsive but do not
                    # spend time on PTZ serial queries.  The PTZ continues slewing
                    # from the last command.
                    if hasattr(Display, "App"):
                        Display.App.processEvents()
                    time.sleep(0.002)
                    continue

                LastRadarDwellTimeSec = NowDwellSec

                # -------------------------------------------------------------
                # PTZ / X6-60 state and operator controls.
                #
                # Continuous SCAN, endpoint reversal, STOP, and manual nudge are
                # now owned exclusively by PointingManager. Main only forwards
                # operator intent and consumes measured pointing state.
                # -------------------------------------------------------------

                PtzValid = False
                PtzSource = "DISABLED"
                PtzAzDeg = float(CommandedBoresightDeg)
                PtzRateDegPerSec = 0.0
                PtzAtTarget = False

                if Ptz is not None:
                    try:
                        # Query/update position. This is throttled inside PTZController.
                        PtzState = Ptz.Update()
                        PtzAzDeg = float(PtzState.AzimuthDeg)
                        PtzRateDegPerSec = float(PtzState.PanRateDegPerSec)
                        PtzValid = bool(PtzState.Valid)
                        PtzSource = str(PtzState.Source)

                        # Manual positioning is owned by PointingManager.
                        ManualNudgeCommandId = None
                        ManualNudgeDeltaDeg = 0.0
                        if ControlState is not None:
                            ManualNudgeCommandId = ControlState.get("ManualNudgeCommandId", None)
                            ManualNudgeDeltaDeg = float(ControlState.get("ManualNudgeDeltaDeg", 0.0))

                        if not hasattr(Main, "_LastConsumedNudgeId"):
                            Main._LastConsumedNudgeId = None

                        if (
                            ManualNudgeCommandId is not None
                            and ManualNudgeCommandId != Main._LastConsumedNudgeId
                            and abs(ManualNudgeDeltaDeg) > 0.0
                        ):
                            Main._LastConsumedNudgeId = ManualNudgeCommandId
                            Pointing.Nudge(ManualNudgeDeltaDeg)

                        elif not (DisplayMode == "SCAN" and ScanEnabled):
                            # PointingManager owns the stop transition. This
                            # clears its active task as well as stopping motion.
                            if LastDisplayMode == "SCAN" and LastScanEnabled:
                                Pointing.Stop()

                        if PtzValid:
                            CurrentScanBoresightDeg = float(PtzAzDeg)
                            if hasattr(Display, "BeamAngleDeg"):
                                Display.BeamAngleDeg = float(PtzAzDeg)

                    except Exception as exc:
                        PtzValid = False
                        PtzSource = f"ERROR: {exc}"

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
                                AzimuthDeg=float(PtzAzDeg if PtzValid else CommandedBoresightDeg),
                                ElevationDeg=float(Config.get("AntennaElDeg", Config.get("PTZStartupElevationDeg", 55.0))),
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
                #   else PTZ measured position,
                #   else commanded fallback.
                if Config.get("UseIMUForBeamAngle", False) and ImuValid:
                    BoresightDeg = float(MeasuredAntennaAzDeg)
                elif PtzValid:
                    BoresightDeg = float(PtzAzDeg)
                else:
                    BoresightDeg = float(CommandedBoresightDeg)

                Config["BoresightDeg"] = float(BoresightDeg)
                Config["CommandedBoresightDeg"] = float(CommandedBoresightDeg)
                Config["AntennaAzDeg"] = float(MeasuredAntennaAzDeg)
                Config["AntennaElDeg"] = float(MeasuredAntennaElDeg)
                Config["IMUValid"] = bool(ImuValid)
                Config["IMUSource"] = str(ImuSource)
                Config["PTZAzDeg"] = float(PtzAzDeg)
                Config["PTZRateDegPerSec"] = float(PtzRateDegPerSec)
                Config["PTZValid"] = bool(PtzValid)
                Config["PTZSource"] = str(PtzSource)
                Config["PTZAtTarget"] = bool(PtzAtTarget)
                Config["ScanCycle"] = int(ScanCycle)

                # -------------------------------------------------------------
                # Build the scene returns for this beam position.
                # -------------------------------------------------------------

                RadarParams = BuildRadarParamsForScenario(Config)

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

                NavigationAttitude = Navigation.get_attitude()

                # Keep the persistent scheduler search task aligned with the
                # operator-selected sector and scan rate.
                SearchTask.Sector.StartDeg = float(ScanStartDeg)
                SearchTask.Sector.StopDeg = float(ScanStopDeg)
                SearchTask.Sector.ScanRateDegPerSec = abs(
                    float(Config.get("PTZScanSlewRateDegPerSec", 14.0))
                )

                # Pointing.Stop() clears the active task. Reissue the search
                # command on every transition into active SCAN, including from
                # STOP or STARE when ScanEnabled remained true.
                if ScanJustStarted:
                    print(
                        f"PointingManager scan start: "
                        f"{ScanStartDeg:.2f} -> {ScanStopDeg:.2f}"
                    )
                    Pointing.ActivateTask(SearchTask, NavigationAttitude)

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
                    PtzAzDeg=PtzAzDeg,
                    PtzRateDegPerSec=PtzRateDegPerSec,
                    PtzValid=PtzValid,
                    PtzSource=PtzSource,
                    PtzAtTarget=PtzAtTarget,
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

                if DwellId % 1 == 0:
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
                        f"PTZ {PtzAzDeg:7.2f} deg {PtzSource} | "
                        f"Mode {DisplayMode} Scan {ScanEnabled} | "
                        f"Dets {len(Detections):3d} Plots {len(Plots):3d} Tracks {len(Tracks):3d} | "
                        f"Pts {int(TrackerDebug.get('CurrentScanPoints', 0)):3d} "
                        f"Blobs {int(TrackerDebug.get('LastCompletedBlobs', 0)):3d} "
                        f"Tent {int(TrackerDebug.get('TentativeTracks', 0)):3d} "
                        f"Conf {int(TrackerDebug.get('ConfirmedTracks', 0)):3d} "
                        f"ScanCycle {ScanCycle}"
                    )


                # -------------------------------------------------------------
                # Move targets forward by one dwell time.
                # -------------------------------------------------------------

                DwellTimeS = Config["NumPulses"] * Config["PRI"]
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
            if Ptz is not None:
                Pointing.Stop()
                Ptz.Close()
        except Exception:
            pass

        Source.Shutdown()


if __name__ == "__main__":
    Main()
