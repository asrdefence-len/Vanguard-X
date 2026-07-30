"""
===============================================================================
ASR Defence X-Band Radar Prototype
RadarDisplayQt5_TracksOnly_TentativeYellow.py
===============================================================================

Fast PyQtGraph operator-style display for Vanguard X.

This module keeps the same public interface as the matplotlib RadarDisplay class:

    Display = RadarDisplay(Config)
    Display.Update(Processed, Detections, Tracks=None, Plots=None)
    ControlState = Display.GetControlState()

It is intended as a faster live display replacement for RadarDisplay.py.
It uses PyQtGraph/Qt rather than matplotlib.

Install requirements if needed:

    pip install pyqtgraph PyQt6

or:

    pip install pyqtgraph PySide6

Notes:
    - The PPI/sector display is drawn in Cartesian x/y space for speed.
    - Static range rings and sector boundaries are drawn once.
    - Each dwell updates only beam line, detection scatter, range profile, and text.
    - Range-Doppler plotting is deliberately omitted for speed.
===============================================================================
"""

import sys
import math
import os
import numpy as np

from RangeProfileScaling import (
    CalculateNoiseReferencedRangeProfileLimits,
    SelectRangeProfileDb,
)
from RadarMapOverlay import RadarCentredMap
from MissionPage import MissionPage

DISPLAY_VERSION = "tracks-white-surface-symbol-v10-radar-link"

# Force pyqtgraph to use the conda PyQt5 binding.
# This avoids macOS/Anaconda failures when a pip PyQt6 install is also present.
os.environ.setdefault("PYQTGRAPH_QT_LIB", "PyQt5")

try:
    import pyqtgraph as pg
    from PyQt5 import QtCore, QtGui, QtWidgets
except Exception as Error:  # pragma: no cover - user environment issue
    raise ImportError(
        "RadarDisplayQt requires pyqtgraph and a Qt binding. Install with:\n"
        "    conda install -c conda-forge pyqtgraph pyqt\n"
        f"\nOriginal import error: {Error}"
    )


class RadarDisplay:
    """
    Fast PyQtGraph display with the same interface expected by VanguardxMain.py.
    """

    def __init__(self, Config):
        self.Config = Config

        # ------------------------------------------------------------------
        # Display / control state
        # ------------------------------------------------------------------
        self.TransmitAvailable = bool(
            self.Config.get("RfTransmitAvailable", True)
        )
        self.TransmitEnabled = bool(
            self.Config.get(
                "InitialTransmitEnabled",
                self.TransmitAvailable,
            )
        ) and self.TransmitAvailable
        self.ExitRequested = False
        self.UpdateCounter = 0
        self.RadarLinkStatus = str(Config.get("RadarLinkStatus", "LOCAL"))
        self.DisplayTrackSourceRequested = str(
            Config.get("DisplayTrackSource", "LEGACY")
        ).upper()
        self.DisplayTrackSourceApplied = self.DisplayTrackSourceRequested
        self.DisplayTrackSourceFallback = False
        self.DisplayTrackSourceReason = "INITIAL_CONFIG"

        self.DisplayMode = self.Config.get("InitialDisplayMode", "STOP")
        self.ScanEnabled = bool(self.Config.get("InitialScanEnabled", False))

        # Complete system selection.  Main consumes a new revision only while
        # STOPPED, shuts down cleanly, and restarts with matching radar and
        # X6-60 adapters.
        self.SystemMode = str(Config.get("SystemMode", "SIM")).upper()
        self.RequestedSystemMode = self.SystemMode
        self.SystemModeRevision = 0

        self.BeamAngleDeg = float(Config.get("InitialBeamAngleDeg", 0.0))
        self.ManualBeamStepDeg = float(Config.get("ManualBeamStepDeg", 2.0))

        # One-shot X6-60/manual command events. These are consumed by the
        # main loop exactly once. Do not use BeamAngleDeg itself as a
        # command event, because BeamAngleDeg is also overwritten by the
        # measured X6-60 boresight for display.
        self.ManualNudgeCommandId = 0
        self.ManualNudgeDeltaDeg = 0.0
        self.StopCommandId = 0

        self.ScanStartDeg = float(Config.get("ScanStartDeg", -60.0))
        self.ScanStopDeg = float(Config.get("ScanStopDeg", 60.0))
        self.ScanStepDeg = float(Config.get("ScanStepDeg", 1.0))
        self.MinScanRateDegPerSec = float(
            Config.get("MinScanRateDegPerSec", 1.0)
        )
        self.MaxScanRateDegPerSec = float(
            Config.get("X660OperationalMaxRateDegPerSec", 60.0)
        )
        self.ScanRateDegPerSec = float(
            Config.get("X660ScanSlewRateDegPerSec", 20.0)
        )
        self.ScanRateDegPerSec = min(
            max(self.ScanRateDegPerSec, self.MinScanRateDegPerSec),
            self.MaxScanRateDegPerSec,
        )

        # Operator timing selections. Editing widgets does not change the
        # running radar until Apply is pressed and Main validates the new
        # profile for the next dwell boundary.
        self.AvailableWaveformIds = list(Config.get(
            "AvailableWaveformIds",
            [
                "Barker13_10MHz",
                "Barker13_20MHz",
                "Frank10_10MHz",
                "Frank10_20MHz",
            ],
        ))
        self.SelectedWaveformId = str(Config.get(
            "SearchWaveformId",
            "Frank10_20MHz",
        ))
        if self.SelectedWaveformId not in self.AvailableWaveformIds:
            self.AvailableWaveformIds.append(self.SelectedWaveformId)
        self.SelectedPrfHz = float(Config.get("SelectedPrfHz", 2000.0))
        self.SelectedPulsesPerCpi = int(Config.get(
            "SelectedPulsesPerCpi",
            32,
        ))
        self.SelectedMaximumRangeM = float(Config.get(
            "InstrumentedMaxRangeM",
            15000.0,
        ))
        self.MinPrfHz = float(Config.get("MinPrfHz", 1000.0))
        self.MaxPrfHz = float(Config.get("MaxPrfHz", 4000.0))
        self.MinPulsesPerCpi = int(Config.get("MinPulsesPerCpi", 8))
        self.MaxPulsesPerCpi = int(Config.get("MaxPulsesPerCpi", 128))
        self.MinSelectableRangeM = float(Config.get(
            "MinSelectableRangeM",
            1000.0,
        ))
        self.MaxSelectableRangeM = float(Config.get(
            "MaxSelectableRangeM",
            Config.get("MaxDisplayRangeM", 15000.0),
        ))
        self.TimingSelectionRevision = 0
        self.TimingApplicationMessage = "Configured"

        self.AppliedWaveformId = self.SelectedWaveformId
        self.AppliedPrfHz = self.SelectedPrfHz
        self.AppliedPulsesPerCpi = self.SelectedPulsesPerCpi
        self.AppliedMaximumRangeM = self.SelectedMaximumRangeM
        self.AppliedCpiDurationSec = (
            self.AppliedPulsesPerCpi / self.AppliedPrfHz
        )
        self.AppliedRxSamples = None

        self.MaxDisplayRangeM = float(Config.get("MaxDisplayRangeM", 15000.0))
        self.PolarMaxRangeM = float(Config.get("PolarMaxRangeM", 15000.0))
        self.RangeRingStepM = float(Config.get("RangeRingStepM", 2000.0))
        self.DataLogFilename = str(Config.get("DataLogFilename", "datafile1.h5"))
        self.SaveDataEnabled = bool(Config.get("DataLoggingEnabled", False))

        self.PolarUpdateEveryNDwells = int(Config.get("PolarUpdateEveryNDwells", 1))
        self.PolarDetectionsUpdateEveryNDwells = int(
            Config.get("PolarDetectionsUpdateEveryNDwells", 1)
        )
        self.RangeProfileUpdateEveryNDwells = int(
            Config.get("RangeProfileUpdateEveryNDwells", 3)
        )
        self.StatusUpdateEveryNDwells = int(Config.get("StatusUpdateEveryNDwells", 1))

        # Degree labels around the PPI/polar plot. This is a Cartesian PPI,
        # so labels are drawn manually around the outside range ring.
        self.ShowPolarDegreeLabels = bool(Config.get("ShowPolarDegreeLabels", True))
        self.ShowPolarCardinalLabels = bool(Config.get("ShowPolarCardinalLabels", True))
        self.PolarDegreeLabelStepDeg = float(Config.get("PolarDegreeLabelStepDeg", 10.0))
        self.PolarDegreeLabelRadiusFraction = float(Config.get("PolarDegreeLabelRadiusFraction", 0.985))

        self.QtProcessEventsEveryNDwells = int(
            Config.get("QtProcessEventsEveryNDwells", 1)
        )

        self.MaxPolarDetections = int(Config.get("MaxPolarDetections", 500))
        self.MaxDetectionsPlottedPerDwell = int(
            Config.get("MaxDetectionsPlottedPerDwell", 20)
        )
        self.DecimateRangeProfile = int(Config.get("QtRangeProfileDecimation", 2))
        self.MaxRangeProfilePoints = int(Config.get("QtMaxRangeProfilePoints", 1500))
        self.RangeProfileDopplerMode = str(
            Config.get("RangeProfileDopplerMode", "MAX")
        ).upper()
        if self.RangeProfileDopplerMode not in ("MAX", "ZERO_DOPPLER"):
            raise ValueError(
                "RangeProfileDopplerMode must be MAX or ZERO_DOPPLER"
            )

        # Range-profile vertical scale. By default both limits follow a robust
        # estimate of the displayed noise floor and the current strongest
        # return. Editing Max dB switches only the upper limit to manual.
        self.RangeProfileMinDb = float(Config.get("RangeProfileMinDb", -110.0))
        self.RangeProfileMaxDb = float(Config.get("RangeProfileMaxDb", -20.0))
        self.RangeProfileAutoScale = bool(Config.get("RangeProfileAutoScale", False))

        # Optional robust automatic lower Y-limit based on estimated noise floor.
        # This is different from full autoscale: the max remains operator-set,
        # while the min follows the estimated noise floor.
        self.RangeProfileAutoMinDb = bool(Config.get("RangeProfileAutoMinDb", True))
        self.RangeProfileAutoMaxDb = bool(Config.get("RangeProfileAutoMaxDb", True))
        self.RangeProfileNoiseMarginDb = float(Config.get("RangeProfileNoiseMarginDb", 5.0))
        self.RangeProfileMinimumSpanAboveNoiseDb = float(
            Config.get("RangeProfileMinimumSpanAboveNoiseDb", 20.0)
        )
        self.RangeProfilePeakHeadroomDb = float(
            Config.get("RangeProfilePeakHeadroomDb", 3.0)
        )
        self.RangeProfileNoisePercentile = float(Config.get("RangeProfileNoisePercentile", 70.0))
        self.RangeProfileNoiseAlpha = float(Config.get("RangeProfileNoiseAlpha", 0.15))
        self.RangeProfileDetectionExclusionBins = int(Config.get("RangeProfileDetectionExclusionBins", 6))
        self.EstimatedNoiseFloorDb = None
        self.EstimatedRangeProfilePeakDb = None
        self.RangeProfileDisplayMinDb = self.RangeProfileMinDb
        self.RangeProfileDisplayMaxDb = self.RangeProfileMaxDb

        self.PolarDetectionHistory = []
        self.LatestProcessed = None
        self.LatestDetections = []
        self.LatestTracks = []
        self.LatestPlots = []

        # ------------------------------------------------------------------
        # Operator target selection
        # ------------------------------------------------------------------
        # Left-click near a displayed track on the PPI to select it.  The
        # target details panel then follows that TrackId on each update.
        self.SelectedTrackId = None
        self.SelectedTrack = None
        self.TrackClickGateM = float(Config.get("TrackClickGateM", 350.0))
        self.TargetDetailsLabel = None

        # ------------------------------------------------------------------
        # Tracker-display layer controls
        # ------------------------------------------------------------------
        # For tracker testing, raw detections/blob plots must be hidden.
        # Otherwise the display's own detection history looks like tracker
        # persistence and makes it impossible to debug initiation/deletion.
        self.ShowRawDetections = bool(Config.get("ShowRawDetections", False))
        self.ShowRangeDetectionMarkers = bool(Config.get("ShowRangeDetectionMarkers", False))
        self.ShowTrackerPlots = bool(Config.get("ShowTrackerPlots", False))
        self.ShowTracks = bool(Config.get("ShowTracks", True))

        # Keep this at zero by default so raw detections cannot persist on the
        # PPI.  Set ShowRawDetections=True and MaxPolarDetections>0 only for
        # CFAR debugging, not for tracker debugging.
        if not self.ShowRawDetections:
            self.MaxPolarDetections = 0

        # ------------------------------------------------------------------
        # Colours
        # ------------------------------------------------------------------
        self.BackgroundColour = "#000000"
        self.PanelColour = "#050505"
        self.GridColour = "#303030"
        self.TextColour = "#ffffff"
        self.MutedTextColour = "#bfbfbf"
        self.TraceColour = "#00ff66"
        self.DetectionColour = "#00ffff"
        self.PlotColour = "#ffcc00"

        # Track/display hierarchy:
        #   - raw detections / plots: deliberately dim
        #   - tentative tracks: pale yellow / amber
        #   - confirmed tracks: white tactical symbol
        # Use RGBA tuples for PyQtGraph so alpha can be controlled.
        self.RawDetectionBrushColour = Config.get("RawDetectionBrushColour", (0, 255, 255, 45))
        self.PlotBrushColour = Config.get("PlotBrushColour", (255, 220, 80, 35))
        self.PlotPenColour = Config.get("PlotPenColour", (255, 220, 80, 100))
        self.TentativeTrackPenColour = Config.get("TentativeTrackPenColour", (255, 225, 100, 135))
        self.TentativeTrackBrushColour = Config.get("TentativeTrackBrushColour", (255, 225, 100, 25))
        self.ConfirmedTrackPenColour = Config.get("ConfirmedTrackPenColour", (255, 255, 255, 235))
        self.ConfirmedTrackBrushColour = Config.get("ConfirmedTrackBrushColour", (255, 255, 255, 0))
        self.TrackTextColour = Config.get("TrackTextColour", (230, 230, 230, 220))
        self.TentativeTrackTextColour = Config.get("TentativeTrackTextColour", (255, 225, 100, 150))

        # Backwards-compatible names used elsewhere in this file.
        self.TentativeTrackColour = self.TentativeTrackPenColour
        self.ConfirmedTrackColour = self.ConfirmedTrackPenColour
        self.BeamColour = "#ffff00"
        self.BoundaryColour = "#808080"
        self.RingColour = Config.get("RangeRingColour", "#505050")

        # Offline, North-up, radar-centred map. Live GPS can later call
        # SetRadarPosition() without changing the display architecture.
        self.MapEnabled = bool(Config.get("MapEnabled", False))
        self.MapLatitudeDeg = float(Config.get("MapLatitudeDeg", -34.368))
        self.MapLongitudeDeg = float(Config.get("MapLongitudeDeg", 150.929))
        self.MapDatasetPath = str(Config.get("MapDatasetPath", "BellambiRegionalMap.json"))
        self.MapLandColour = Config.get("MapLandColour", (92, 92, 92, 105))
        self.MapCoastColour = Config.get("MapCoastColour", (255, 255, 255, 190))
        self.MapLabelColour = Config.get("MapLabelColour", (255, 255, 255, 180))
        self.MapOverlay = None
        self.MapGraphicsItems = []

        # Optional logo. Path is relative to the folder you run the script from,
        # or it may be an absolute path.
        self.LogoPath = Config.get("LogoPath", "")
        self.LogoWidthPx = int(Config.get("LogoWidthPx", 160))

        # PyQtGraph global style.
        pg.setConfigOptions(antialias=False, background=self.BackgroundColour, foreground=self.TextColour)

        # QApplication must exist before widgets.
        self.App = QtWidgets.QApplication.instance()
        if self.App is None:
            self.App = QtWidgets.QApplication(sys.argv)

        self.Window = None
        self.PpiPlot = None
        self.RangePlot = None
        self.StatusLabel = None
        self.TargetDetailsLabel = None

        self.BeamLine = None
        self.SectorBoundaryLines = []
        self.PolarDegreeLabelItems = []
        self.DetectionScatter = None
        self.PlotScatter = None
        self.TentativeTrackScatter = None
        self.ConfirmedTrackScatter = None
        self.TrackTextItems = []
        self.RangeProfileCurve = None
        self.ControlWidgets = {}
        self.TimingFeedbackLabel = None
        self.TimingSummaryLabel = None
        self.MissionPage = None
        self.OperatorHeaderModeLabel = None
        self.OperatorHeaderTxLabel = None
        self.OperatorHeaderAntennaLabel = None
        self.OperatorHeaderMissionLabel = None
        self.OperatorHeaderWarningLabel = None

        self.InitialiseWindow()

    # ------------------------------------------------------------------
    # Public interface used by VanguardxMain.py
    # ------------------------------------------------------------------

    def Update(self, Processed, Detections, Tracks=None, Plots=None):
        """
        Update display with latest processed dwell.

        Tracks and Plots are optional so this display remains compatible with
        older Vanguard main loops that only passed Processed and Detections.
        """
        self.UpdateCounter += 1
        self.LatestProcessed = Processed
        self.LatestDetections = Detections if Detections is not None else []
        self.LatestTracks = Tracks if Tracks is not None else []
        self.LatestPlots = Plots if Plots is not None else []

        Diagnostics = getattr(Processed, "Diagnostics", {})
        self.BeamAngleDeg = float(Diagnostics.get("BoresightDeg", self.BeamAngleDeg))
        self.DisplayTrackSourceRequested = str(
            Diagnostics.get(
                "DisplayTrackSourceRequested",
                self.DisplayTrackSourceRequested,
            )
        ).upper()
        self.DisplayTrackSourceApplied = str(
            Diagnostics.get(
                "DisplayTrackSourceApplied",
                self.DisplayTrackSourceApplied,
            )
        ).upper()
        self.DisplayTrackSourceFallback = bool(
            Diagnostics.get(
                "DisplayTrackSourceFallback",
                self.DisplayTrackSourceFallback,
            )
        )
        self.DisplayTrackSourceReason = str(
            Diagnostics.get(
                "DisplayTrackSourceReason",
                self.DisplayTrackSourceReason,
            )
        )

        if self.ShowRawDetections:
            self.AppendPolarDetections(Detections)
        else:
            self.PolarDetectionHistory = []

        if self.PolarUpdateEveryNDwells > 0 and self.UpdateCounter % self.PolarUpdateEveryNDwells == 0:
            self.UpdateBeamLine()

        if (
            self.PolarDetectionsUpdateEveryNDwells > 0
            and self.UpdateCounter % self.PolarDetectionsUpdateEveryNDwells == 0
        ):
            self.UpdateDetectionScatter()
            self.UpdateTrackScatter()

        if (
            self.RangeProfileUpdateEveryNDwells > 0
            and self.UpdateCounter % self.RangeProfileUpdateEveryNDwells == 0
        ):
            self.UpdateRangeProfile()

        # Keep the selected target tied to the latest Track object from the
        # tracker output.  The tracker passes a fresh list each dwell.
        self.RefreshSelectedTrack()
        self.UpdateTargetDetailsPanel()

        if self.StatusUpdateEveryNDwells > 0 and self.UpdateCounter % self.StatusUpdateEveryNDwells == 0:
            self.UpdateStatusPanel()

        if self.QtProcessEventsEveryNDwells > 0 and self.UpdateCounter % self.QtProcessEventsEveryNDwells == 0:
            self.App.processEvents(QtCore.QEventLoop.ProcessEventsFlag.AllEvents, 1)

    def GetControlState(self):
        """
        Return current operator control state.
        """
        state = {
            "DisplayMode": self.DisplayMode,
            "ScanEnabled": self.ScanEnabled,
            "BeamAngleDeg": self.BeamAngleDeg,
            "ScanStartDeg": self.ScanStartDeg,
            "ScanStopDeg": self.ScanStopDeg,
            "ScanStepDeg": self.ScanStepDeg,
            "ScanRateDegPerSec": self.ScanRateDegPerSec,
            "TransmitAvailable": self.TransmitAvailable,
            "TransmitEnabled": self.TransmitEnabled,
            "ExitRequested": self.ExitRequested,
            "DataLogFilename": self.DataLogFilename,
            "SaveDataEnabled": self.SaveDataEnabled,
            "ManualNudgeCommandId": self.ManualNudgeCommandId,
            "ManualNudgeDeltaDeg": self.ManualNudgeDeltaDeg,
            "StopCommandId": self.StopCommandId,
            "SelectedWaveformId": self.SelectedWaveformId,
            "SelectedPrfHz": self.SelectedPrfHz,
            "SelectedPulsesPerCpi": self.SelectedPulsesPerCpi,
            "SelectedMaximumRangeM": self.SelectedMaximumRangeM,
            "TimingSelectionRevision": self.TimingSelectionRevision,
            "SystemMode": self.SystemMode,
            "RequestedSystemMode": self.RequestedSystemMode,
            "SystemModeRevision": self.SystemModeRevision,
        }
        if self.MissionPage is not None:
            state.update(self.MissionPage.GetMissionControlState())
        return state

    def SetMissionRuntimeStatus(self, Status):
        """Apply authoritative mission status returned by the radar process."""

        status = dict(Status or {})
        state = str(status.get("State", "STOPPED")).upper()
        if self.MissionPage is not None:
            self.MissionPage.SetRuntimeStatus(status)
        if state in ("RUNNING_360", "RUNNING_SECTOR"):
            self.DisplayMode = "SCAN"
            self.ScanEnabled = True
            if self.TransmitAvailable:
                self.TransmitEnabled = True
        elif state in (
            "LOADED", "PAUSED", "COMPLETED", "ABORTED", "FAULTED",
        ):
            self.DisplayMode = "STOP"
            self.ScanEnabled = False
            self.TransmitEnabled = False
        self.UpdateStatusPanel()

    def SetSystemModeApplicationResult(self, Applied, Message):
        """Report a system-mode request accepted or rejected by Main."""

        if hasattr(self, "SystemModeFeedbackLabel"):
            Colour = "#00ff66" if Applied else "#ff6666"
            self.SystemModeFeedbackLabel.setStyleSheet(f"color: {Colour};")
            self.SystemModeFeedbackLabel.setText(str(Message))

    def SetRadarLinkStatus(self, Connected, Message):
        """Update the operator-visible radar server connection state."""

        self.RadarLinkStatus = str(Message)
        self.UpdateStatusPanel()

    def SetMeasuredBeamAngle(self, AzimuthDeg):
        """Refresh the PPI beam from measured X6-60 encoder telemetry.

        This lightweight path is independent of radar dwell processing, so
        the beam remains live while the operator display is in STOP.
        """

        self.BeamAngleDeg = float(AzimuthDeg) % 360.0
        if self.BeamLine is not None:
            self.UpdateBeamLine()
        if self.StatusLabel is not None:
            self.UpdateStatusPanel()

    def SetTimingApplicationResult(
        self,
        Applied,
        Message,
        Profile=None,
    ):
        """Show the result of Main's dwell-boundary timing validation."""

        self.TimingApplicationMessage = str(Message)
        if self.TimingFeedbackLabel is not None:
            Colour = "#00ff66" if Applied else "#ff6666"
            self.TimingFeedbackLabel.setStyleSheet(f"color: {Colour};")
            self.TimingFeedbackLabel.setText(self.TimingApplicationMessage)

        if Applied and Profile is not None:
            Timing = Profile.Timing
            self.AppliedWaveformId = str(Profile.WaveformId)
            self.AppliedPrfHz = float(Timing.SelectedPrfHz)
            self.AppliedPulsesPerCpi = int(Timing.PulsesPerCpi)
            self.AppliedMaximumRangeM = float(Timing.MaximumRangeM)
            self.AppliedCpiDurationSec = float(Timing.CpiDurationSec)
            self.AppliedRxSamples = int(Timing.NumRxSamples)
            self.UpdateTimingSummary(Profile)

        self.UpdateStatusPanel()

    def UpdateTimingSummary(self, Profile=None):
        """Update the compact timing line without resizing the plot area."""

        PrfHz = float(self.SelectedPrfHz)
        Pulses = int(self.SelectedPulsesPerCpi)
        PriUs = 1.0e6 / PrfHz
        CpiMs = 1.0e3 * Pulses / PrfHz
        MaximumRangeKm = self.SelectedMaximumRangeM / 1000.0
        RxText = ""

        if Profile is not None:
            Timing = Profile.Timing
            PriUs = float(Timing.PriSec) * 1.0e6
            CpiMs = float(Timing.CpiDurationSec) * 1.0e3
            MaximumRangeKm = float(Timing.MaximumRangeM) / 1000.0
            RxText = f"  |  RX {int(Timing.NumRxSamples)}"

        if self.TimingSummaryLabel is not None:
            self.TimingSummaryLabel.setText(
                f"PRI {PriUs:.1f} us  |  CPI {CpiMs:.1f} ms{RxText}  |  "
                f"R {MaximumRangeKm:.1f} km"
            )

    # ------------------------------------------------------------------
    # Window construction
    # ------------------------------------------------------------------

    def InitialiseWindow(self):
        self.Window = QtWidgets.QMainWindow()
        self.Window.setWindowTitle("Vanguard X Radar and Mission Display")
        # The original Radar page remains a 1500 x 900 layout.  The additional
        # height accommodates the persistent status header and tab bar rather
        # than taking space from the proven PPI/range-profile geometry.
        self.Window.resize(1500, 970)
        self.Window.setStyleSheet(
            "QMainWindow { background-color: black; }"
            "QWidget { background-color: black; color: white; font-family: Arial; }"
            "QPushButton { background-color: #202020; color: white; border: 1px solid #505050; padding: 3px 6px; }"
            "QPushButton:hover { background-color: #404040; }"
            "QLineEdit { background-color: #101010; color: white; border: 1px solid #505050; padding: 2px 4px; }"
            "QComboBox, QSpinBox, QDoubleSpinBox { background-color: #101010; color: white; border: 1px solid #505050; padding: 2px 4px; }"
            "QTableWidget, QPlainTextEdit { background-color: #050505; color: white; border: 1px solid #303030; }"
            "QHeaderView::section { background-color: #202020; color: white; border: 1px solid #404040; padding: 4px; }"
            "QTabWidget::pane { border: 1px solid #303030; }"
            "QTabBar::tab { background-color: #151515; color: #bfbfbf; border: 1px solid #404040; padding: 7px 18px; }"
            "QTabBar::tab:selected { background-color: #303030; color: white; }"
            "QLabel { color: white; }"
        )

        CentralWidget = QtWidgets.QWidget()
        MainLayout = QtWidgets.QHBoxLayout(CentralWidget)
        MainLayout.setContentsMargins(8, 8, 8, 8)
        MainLayout.setSpacing(10)

        # Left: PPI display.
        self.PpiPlot = pg.PlotWidget()
        self.PpiPlot.setBackground(self.PanelColour)
        self.PpiPlot.setAspectLocked(True)
        self.PpiPlot.showGrid(x=True, y=True, alpha=0.25)
        self.PpiPlot.setLabel("bottom", "East / range", units="m")
        self.PpiPlot.setLabel("left", "North / range", units="m")
        self.PpiPlot.setXRange(-self.PolarMaxRangeM, self.PolarMaxRangeM, padding=0.02)
        self.PpiPlot.setYRange(-self.PolarMaxRangeM, self.PolarMaxRangeM, padding=0.02)
        self.PpiPlot.setTitle("VANGUARD X Sector Scan")
        self.PpiPlot.scene().sigMouseClicked.connect(self.OnPpiMouseClicked)

        MainLayout.addWidget(self.PpiPlot, stretch=3)

        # Right: controls, status, range profile.
        RightPanel = QtWidgets.QWidget()
        RightLayout = QtWidgets.QVBoxLayout(RightPanel)
        RightLayout.setContentsMargins(0, 0, 0, 0)
        RightLayout.setSpacing(8)
        MainLayout.addWidget(RightPanel, stretch=2)

        HeaderWidget = QtWidgets.QWidget()
        HeaderLayout = QtWidgets.QHBoxLayout(HeaderWidget)
        HeaderLayout.setContentsMargins(0, 0, 0, 0)
        HeaderLayout.setSpacing(8)

        Title = QtWidgets.QLabel("VANGUARD X\nMultifunction Tactical Radar Display")
        Title.setStyleSheet("font-size: 20px; font-weight: bold; color: white;")
        HeaderLayout.addWidget(Title, stretch=1)

        self.LogoLabel = QtWidgets.QLabel()
        self.LogoLabel.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.LoadLogo()
        HeaderLayout.addWidget(self.LogoLabel, stretch=0)

        RightLayout.addWidget(HeaderWidget)

        Controls = self.CreateControls()
        RightLayout.addWidget(Controls)

        # Status and selected target are shown side-by-side so the selected
        # target panel does not push the range profile down.
        StatusTargetWidget = QtWidgets.QWidget()
        StatusTargetLayout = QtWidgets.QHBoxLayout(StatusTargetWidget)
        StatusTargetLayout.setContentsMargins(0, 0, 0, 0)
        StatusTargetLayout.setSpacing(8)

        self.StatusLabel = QtWidgets.QLabel()
        self.StatusLabel.setStyleSheet(
            "background-color: #050505; border: 1px solid #303030; "
            "font-family: Menlo, Consolas, monospace; font-size: 12px; padding: 8px;"
        )
        self.StatusLabel.setMinimumHeight(220)
        self.StatusLabel.setAlignment(QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignTop)
        StatusTargetLayout.addWidget(self.StatusLabel, stretch=1)

        self.TargetDetailsLabel = QtWidgets.QLabel()
        self.TargetDetailsLabel.setStyleSheet(
            "background-color: #050505; border: 1px solid #303030; "
            "font-family: Menlo, Consolas, monospace; font-size: 12px; padding: 8px;"
        )
        self.TargetDetailsLabel.setMinimumHeight(220)
        self.TargetDetailsLabel.setMinimumWidth(260)
        self.TargetDetailsLabel.setAlignment(QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignTop)
        self.TargetDetailsLabel.setText("SELECTED TARGET\nNone")
        StatusTargetLayout.addWidget(self.TargetDetailsLabel, stretch=1)

        RightLayout.addWidget(StatusTargetWidget)

        self.RangePlot = pg.PlotWidget()
        self.RangePlot.setBackground(self.PanelColour)
        self.RangePlot.showGrid(x=True, y=True, alpha=0.25)
        self.RangePlot.setLabel("bottom", "Range", units="m")
        self.RangePlot.setLabel("left", "Magnitude", units="dB")
        self.RangePlot.setXRange(0, self.MaxDisplayRangeM, padding=0.0)
        self.RangePlot.setTitle(
            "Range Profile (0 Hz Doppler)"
            if self.RangeProfileDopplerMode == "ZERO_DOPPLER"
            else "Range Profile (Max Doppler Envelope)"
        )
        self.ApplyRangeProfileScale()
        RightLayout.addWidget(self.RangePlot, stretch=1)

        # Range-Doppler panel removed for compact high-speed display.

        # The existing display is kept intact as the Radar page. Mission
        # planning lives in the same Qt application but in a separate tab.
        # MissionPage has no direct radar, antenna, or hardware interface.
        RootWidget = QtWidgets.QWidget()
        RootLayout = QtWidgets.QVBoxLayout(RootWidget)
        RootLayout.setContentsMargins(6, 6, 6, 6)
        RootLayout.setSpacing(5)
        RootLayout.addWidget(self.CreatePersistentOperatorHeader())

        self.OperatorTabs = QtWidgets.QTabWidget()
        self.OperatorTabs.addTab(CentralWidget, "Radar")
        self.MissionPage = MissionPage(self.Config)
        self.MissionPage.statusChanged.connect(self.UpdatePersistentOperatorHeader)
        self.OperatorTabs.addTab(self.MissionPage, "Mission")
        RootLayout.addWidget(self.OperatorTabs, stretch=1)
        self.Window.setCentralWidget(RootWidget)

        self.CreateStaticPpiItems()
        self.RangeProfileCurve = self.RangePlot.plot(
            [], [], pen=pg.mkPen(self.TraceColour, width=1)
        )

        # Current dwell detection markers on range profile.
        self.RangeDetectionScatter = pg.ScatterPlotItem(
            size=9,
            brush=pg.mkBrush(255, 255, 0),
            pen=pg.mkPen(255, 255, 255, width=1),
        )
        self.RangePlot.addItem(self.RangeDetectionScatter)
        self.UpdateStatusPanel()

        self.Window.show()
        self.App.processEvents()

    def CreatePersistentOperatorHeader(self):
        """Create the compact status and STOP strip visible from both tabs."""

        header = QtWidgets.QFrame()
        header.setStyleSheet(
            "QFrame { background-color: #050505; border: 1px solid #303030; }"
            "QLabel { border: none; font-family: Menlo, Consolas, monospace; }"
        )
        layout = QtWidgets.QHBoxLayout(header)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(16)

        title = QtWidgets.QLabel("VANGUARD X")
        title.setStyleSheet(
            "border: none; font-size: 15px; font-weight: bold; color: white;"
        )
        layout.addWidget(title)

        self.OperatorHeaderModeLabel = QtWidgets.QLabel()
        self.OperatorHeaderTxLabel = QtWidgets.QLabel()
        self.OperatorHeaderAntennaLabel = QtWidgets.QLabel()
        self.OperatorHeaderMissionLabel = QtWidgets.QLabel()
        self.OperatorHeaderWarningLabel = QtWidgets.QLabel()
        for label in (
            self.OperatorHeaderModeLabel,
            self.OperatorHeaderTxLabel,
            self.OperatorHeaderAntennaLabel,
            self.OperatorHeaderMissionLabel,
        ):
            layout.addWidget(label)
        layout.addWidget(self.OperatorHeaderWarningLabel, stretch=1)

        stop_button = QtWidgets.QPushButton("STOP")
        stop_button.setMinimumWidth(110)
        stop_button.setStyleSheet(
            "QPushButton { background-color: #8b0000; color: white; "
            "font-weight: bold; border: 2px solid #ff6666; padding: 5px 18px; }"
            "QPushButton:hover { background-color: #c00000; }"
        )
        stop_button.clicked.connect(self.OnStop)
        layout.addWidget(stop_button)
        self.UpdatePersistentOperatorHeader()
        return header

    def UpdatePersistentOperatorHeader(self):
        """Refresh persistent operator state without touching radar geometry."""

        if self.OperatorHeaderModeLabel is None:
            return
        tx_text = (
            "RX ONLY" if not self.TransmitAvailable
            else ("TX ENABLED" if self.TransmitEnabled else "TX OFF")
        )
        mission_text = (
            self.MissionPage.MissionStatusText()
            if self.MissionPage is not None else "MISSION: unavailable"
        )
        self.OperatorHeaderModeLabel.setText(
            f"SYSTEM {self.SystemMode} | {self.DisplayMode}"
        )
        self.OperatorHeaderTxLabel.setText(tx_text)
        self.OperatorHeaderTxLabel.setStyleSheet(
            "border: none; color: "
            + ("#ff6666;" if self.TransmitEnabled else "#bfbfbf;")
        )
        self.OperatorHeaderAntennaLabel.setText(
            f"AZ {self.BeamAngleDeg:06.2f} deg"
        )
        self.OperatorHeaderMissionLabel.setText(mission_text)
        if "LOST" in self.RadarLinkStatus.upper():
            self.OperatorHeaderWarningLabel.setText(
                "LINK LOST - RADAR FAIL-STOP"
            )
            self.OperatorHeaderWarningLabel.setStyleSheet(
                "border: none; color: #ff6666; font-weight: bold;"
            )
        else:
            self.OperatorHeaderWarningLabel.setText(
                f"LINK {self.RadarLinkStatus}"
            )
            self.OperatorHeaderWarningLabel.setStyleSheet(
                "border: none; color: #bfbfbf;"
            )

    def LoadLogo(self):
        """Load the ASR logo into the header if the file exists."""
        self.LogoLabel.clear()

        if not self.LogoPath:
            return

        LogoPath = self.LogoPath
        if not os.path.isabs(LogoPath):
            LogoPath = os.path.abspath(LogoPath)

        if not os.path.exists(LogoPath):
            self.LogoLabel.setText("")
            return

        Pixmap = QtGui.QPixmap(LogoPath)
        if Pixmap.isNull():
            self.LogoLabel.setText("")
            return

        Pixmap = Pixmap.scaledToWidth(
            self.LogoWidthPx,
            QtCore.Qt.TransformationMode.SmoothTransformation,
        )
        self.LogoLabel.setPixmap(Pixmap)
        self.LogoLabel.setMaximumWidth(self.LogoWidthPx + 8)

    def CreateControls(self):
        Box = QtWidgets.QGroupBox("Controls")
        Layout = QtWidgets.QGridLayout(Box)
        Layout.setContentsMargins(6, 6, 6, 6)
        Layout.setHorizontalSpacing(5)
        Layout.setVerticalSpacing(5)

        def CompactButton(Label, Width=64):
            Button = QtWidgets.QPushButton(Label)
            Button.setMinimumWidth(Width)
            Button.setMaximumWidth(Width)
            Button.setMinimumHeight(26)
            return Button

        def CompactEdit(Text, Width=58):
            Edit = QtWidgets.QLineEdit(Text)
            Edit.setMinimumWidth(Width)
            Edit.setMaximumWidth(Width)
            Edit.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight)
            return Edit

        StartButton = CompactButton("Start", 58)
        StopButton = CompactButton("Stop", 58)
        StopTxButton = CompactButton("Tx Off", 58)
        if not self.TransmitAvailable:
            StopTxButton.setText("TX N/A")
            StopTxButton.setEnabled(False)
        ExitButton = CompactButton("Exit", 58)
        ScanButton = CompactButton("Scan", 58)
        StareButton = CompactButton("Stare", 58)
        LeftButton = CompactButton("◀", 40)
        RightButton = CompactButton("▶", 40)

        StartButton.clicked.connect(self.OnStartScan)
        StopButton.clicked.connect(self.OnStop)
        StopTxButton.clicked.connect(self.OnStopTransmit)
        ExitButton.clicked.connect(self.OnExit)
        ScanButton.clicked.connect(self.OnStartScan)
        StareButton.clicked.connect(self.OnStartStare)
        LeftButton.clicked.connect(self.OnBeamLeft)
        RightButton.clicked.connect(self.OnBeamRight)

        Layout.addWidget(StartButton, 0, 0)
        Layout.addWidget(StopButton, 0, 1)
        Layout.addWidget(StopTxButton, 0, 2)
        Layout.addWidget(ExitButton, 0, 3)
        Layout.addWidget(ScanButton, 1, 0)
        Layout.addWidget(StareButton, 1, 1)
        Layout.addWidget(LeftButton, 1, 2)
        Layout.addWidget(RightButton, 1, 3)

        self.ControlWidgets["ScanStart"] = CompactEdit(str(self.ScanStartDeg))
        self.ControlWidgets["ScanStop"] = CompactEdit(str(self.ScanStopDeg))
        self.ControlWidgets["ScanStep"] = CompactEdit(str(self.ScanStepDeg), 50)
        self.ControlWidgets["ScanRate"] = QtWidgets.QDoubleSpinBox()
        self.ControlWidgets["ScanRate"].setRange(
            self.MinScanRateDegPerSec,
            self.MaxScanRateDegPerSec,
        )
        self.ControlWidgets["ScanRate"].setDecimals(1)
        self.ControlWidgets["ScanRate"].setSingleStep(1.0)
        self.ControlWidgets["ScanRate"].setSuffix(" °/s")
        self.ControlWidgets["ScanRate"].setValue(self.ScanRateDegPerSec)
        self.ControlWidgets["ScanRate"].setMinimumWidth(82)
        self.ControlWidgets["ScanRate"].setMaximumWidth(96)
        self.ControlWidgets["RangeProfileMaxDb"] = CompactEdit(str(self.RangeProfileMaxDb), 50)
        self.ControlWidgets["DataLogFilename"] = QtWidgets.QLineEdit(self.DataLogFilename)
        self.ControlWidgets["DataLogFilename"].setMinimumWidth(145)
        self.ControlWidgets["DataLogFilename"].setMaximumWidth(190)
        self.ControlWidgets["DataLogFilename"].setAlignment(QtCore.Qt.AlignmentFlag.AlignLeft)
        self.ControlWidgets["SaveDataEnabled"] = QtWidgets.QCheckBox("Save")
        self.ControlWidgets["SaveDataEnabled"].setChecked(self.SaveDataEnabled)
        self.ControlWidgets["SaveDataEnabled"].setMaximumWidth(70)

        self.ControlWidgets["SystemMode"] = QtWidgets.QComboBox()
        self.ControlWidgets["SystemMode"].addItems(["SIM", "HARD", "RF LOOPBACK"])
        self.ControlWidgets["SystemMode"].setCurrentText(self.SystemMode)
        self.ControlWidgets["SystemMode"].setMinimumWidth(72)
        self.ControlWidgets["SystemMode"].setMaximumWidth(125)
        self.ControlWidgets["SystemMode"].setStyleSheet(
            "QComboBox { color: #00ff66; font-weight: bold; }"
            if self.SystemMode == "SIM"
            else "QComboBox { color: #ffb347; font-weight: bold; }"
        )
        self.ControlWidgets["SystemMode"].currentTextChanged.connect(
            self.OnSystemModeChanged
        )
        self.SystemModeFeedbackLabel = QtWidgets.QLabel("Active")
        self.SystemModeFeedbackLabel.setStyleSheet("color: #bfbfbf;")

        self.ControlWidgets["WaveformId"] = QtWidgets.QComboBox()
        self.ControlWidgets["WaveformId"].addItems(
            self.AvailableWaveformIds
        )
        self.ControlWidgets["WaveformId"].setCurrentText(
            self.SelectedWaveformId
        )
        self.ControlWidgets["WaveformId"].setMinimumWidth(125)
        self.ControlWidgets["WaveformId"].setMaximumWidth(145)

        self.ControlWidgets["PrfKHz"] = QtWidgets.QDoubleSpinBox()
        self.ControlWidgets["PrfKHz"].setRange(
            self.MinPrfHz / 1000.0,
            self.MaxPrfHz / 1000.0,
        )
        self.ControlWidgets["PrfKHz"].setDecimals(2)
        self.ControlWidgets["PrfKHz"].setSingleStep(0.10)
        self.ControlWidgets["PrfKHz"].setSuffix(" kHz")
        self.ControlWidgets["PrfKHz"].setValue(
            self.SelectedPrfHz / 1000.0
        )
        self.ControlWidgets["PrfKHz"].setMinimumWidth(92)
        self.ControlWidgets["PrfKHz"].setMaximumWidth(105)

        self.ControlWidgets["PulsesPerCpi"] = QtWidgets.QSpinBox()
        self.ControlWidgets["PulsesPerCpi"].setRange(
            self.MinPulsesPerCpi,
            self.MaxPulsesPerCpi,
        )
        self.ControlWidgets["PulsesPerCpi"].setValue(
            self.SelectedPulsesPerCpi
        )
        self.ControlWidgets["PulsesPerCpi"].setMinimumWidth(68)
        self.ControlWidgets["PulsesPerCpi"].setMaximumWidth(80)

        self.ControlWidgets["MaximumRangeKm"] = QtWidgets.QDoubleSpinBox()
        self.ControlWidgets["MaximumRangeKm"].setRange(
            self.MinSelectableRangeM / 1000.0,
            self.MaxSelectableRangeM / 1000.0,
        )
        self.ControlWidgets["MaximumRangeKm"].setDecimals(1)
        self.ControlWidgets["MaximumRangeKm"].setSingleStep(0.5)
        self.ControlWidgets["MaximumRangeKm"].setSuffix(" km")
        self.ControlWidgets["MaximumRangeKm"].setValue(
            self.SelectedMaximumRangeM / 1000.0
        )
        self.ControlWidgets["MaximumRangeKm"].setMinimumWidth(92)
        self.ControlWidgets["MaximumRangeKm"].setMaximumWidth(105)

        TimingApplyButton = QtWidgets.QPushButton("Apply")
        TimingApplyButton.setMinimumHeight(26)
        TimingApplyButton.setMinimumWidth(68)
        TimingApplyButton.setMaximumWidth(78)
        TimingApplyButton.clicked.connect(self.OnTimingApply)

        self.TimingFeedbackLabel = QtWidgets.QLabel(
            self.TimingApplicationMessage
        )
        self.TimingFeedbackLabel.setStyleSheet("color: #bfbfbf;")
        self.TimingSummaryLabel = QtWidgets.QLabel()
        self.TimingSummaryLabel.setStyleSheet(
            "color: #bfbfbf; font-family: Menlo, Consolas, monospace;"
        )

        self.ControlWidgets["ScanStart"].editingFinished.connect(self.OnScanStartChanged)
        self.ControlWidgets["ScanStop"].editingFinished.connect(self.OnScanStopChanged)
        self.ControlWidgets["ScanStep"].editingFinished.connect(self.OnScanStepChanged)
        self.ControlWidgets["ScanRate"].valueChanged.connect(
            self.OnScanRateChanged
        )
        self.ControlWidgets["RangeProfileMaxDb"].editingFinished.connect(self.OnRangeProfileMaxDbChanged)
        self.ControlWidgets["DataLogFilename"].editingFinished.connect(self.OnDataLogFilenameChanged)
        self.ControlWidgets["SaveDataEnabled"].stateChanged.connect(self.OnSaveDataEnabledChanged)

        StartLabel = QtWidgets.QLabel("Start")
        StopLabel = QtWidgets.QLabel("Stop")
        StepLabel = QtWidgets.QLabel("Step")
        MaxDbLabel = QtWidgets.QLabel("Max dB")
        FileLabel = QtWidgets.QLabel("File")
        WaveformLabel = QtWidgets.QLabel("Waveform")
        PrfLabel = QtWidgets.QLabel("PRF")
        PulsesLabel = QtWidgets.QLabel("Pulses/CPI")
        ScanRateLabel = QtWidgets.QLabel("Scan rate")
        MaximumRangeLabel = QtWidgets.QLabel("Max range")

        Layout.addWidget(StartLabel, 2, 0)
        Layout.addWidget(self.ControlWidgets["ScanStart"], 2, 1)
        Layout.addWidget(StopLabel, 2, 2)
        Layout.addWidget(self.ControlWidgets["ScanStop"], 2, 3)
        Layout.addWidget(StepLabel, 3, 0)
        Layout.addWidget(self.ControlWidgets["ScanStep"], 3, 1)
        Layout.addWidget(MaxDbLabel, 3, 2)
        Layout.addWidget(self.ControlWidgets["RangeProfileMaxDb"], 3, 3)
        Layout.addWidget(FileLabel, 4, 0)
        Layout.addWidget(self.ControlWidgets["DataLogFilename"], 4, 1, 1, 2)
        Layout.addWidget(self.ControlWidgets["SaveDataEnabled"], 4, 3)
        Layout.addWidget(QtWidgets.QLabel("Mode"), 4, 6)
        Layout.addWidget(self.ControlWidgets["SystemMode"], 4, 7)
        Layout.addWidget(self.SystemModeFeedbackLabel, 4, 8)

        # A compact third functional column uses the previously empty width.
        # The sixth row replaces the removed Auto-min control, keeping the
        # Controls box at the same row count and preserving plot geometry.
        Layout.addWidget(WaveformLabel, 0, 4)
        Layout.addWidget(self.ControlWidgets["WaveformId"], 0, 5)
        Layout.addWidget(PrfLabel, 1, 4)
        Layout.addWidget(self.ControlWidgets["PrfKHz"], 1, 5)
        Layout.addWidget(PulsesLabel, 2, 4)
        Layout.addWidget(self.ControlWidgets["PulsesPerCpi"], 2, 5)
        Layout.addWidget(ScanRateLabel, 2, 6)
        Layout.addWidget(self.ControlWidgets["ScanRate"], 2, 7)
        Layout.addWidget(MaximumRangeLabel, 3, 4)
        Layout.addWidget(self.ControlWidgets["MaximumRangeKm"], 3, 5)
        Layout.addWidget(TimingApplyButton, 4, 4)
        Layout.addWidget(self.TimingFeedbackLabel, 4, 5)
        Layout.addWidget(self.TimingSummaryLabel, 5, 0, 1, 6)

        # Prevent the grid columns expanding controls across the full side panel.
        for Column in range(9):
            Layout.setColumnStretch(Column, 0)

        self.UpdateTimingSummary()

        return Box

    def CreateStaticPpiItems(self):
        self.CreateMapOverlay()

        # Static range rings.
        if self.RangeRingStepM > 0:
            RingRanges = np.arange(
                self.RangeRingStepM,
                self.PolarMaxRangeM + 0.5 * self.RangeRingStepM,
                self.RangeRingStepM,
            )
            Theta = np.linspace(0, 2 * np.pi, 241)
            RingPen = pg.mkPen(self.RingColour, width=1)
            for RingM in RingRanges:
                X = RingM * np.cos(Theta)
                Y = RingM * np.sin(Theta)
                self.PpiPlot.plot(X, Y, pen=RingPen)

                Label = pg.TextItem(f"{int(RingM / 1000)} km", color=self.MutedTextColour, anchor=(0, 0.5))
                Label.setPos(-0.72 * RingM, 0.72 * RingM)
                self.PpiPlot.addItem(Label)

        # Sector boundaries are dynamic because the operator can change
        # ScanStartDeg / ScanStopDeg from the GUI after startup.
        self.UpdateSectorBoundaryLines()
        self.UpdatePolarDegreeLabels()

        # Dynamic beam line, optional CFAR/blobs, and track scatter.
        # By default only tracks are visible.
        self.BeamLine = self.PpiPlot.plot([], [], pen=pg.mkPen(self.BeamColour, width=2))

        # Raw CFAR cell history. Cyan, small and faint.
        self.DetectionScatter = pg.ScatterPlotItem(
            size=5,
            brush=pg.mkBrush(*self.RawDetectionBrushColour),
            pen=None,
        )
        self.PpiPlot.addItem(self.DetectionScatter)

        # Current clustered plots/blobs. Dim yellow/amber so they do not
        # dominate the tracker layer.
        self.PlotScatter = pg.ScatterPlotItem(
            size=10,
            brush=pg.mkBrush(*self.PlotBrushColour),
            pen=pg.mkPen(*self.PlotPenColour, width=1),
        )
        self.PpiPlot.addItem(self.PlotScatter)

        # Tentative / initiating tracks. Pale yellow open-ish markers.
        # These are first-hit / 1-of-3 / initiation-stage objects.
        self.TentativeTrackScatter = pg.ScatterPlotItem(
            size=14,
            symbol="o",
            brush=pg.mkBrush(*self.TentativeTrackBrushColour),
            pen=pg.mkPen(*self.TentativeTrackPenColour, width=1),
        )
        self.PpiPlot.addItem(self.TentativeTrackScatter)

        # Confirmed tracks. White tactical surface-vessel style marker.
        # The custom symbol is intentionally unfilled so confirmed tracks are
        # crisp without becoming solid blobs.
        self.SurfaceVesselSymbol = self.CreateSurfaceVesselSymbolPath()
        self.ConfirmedTrackScatter = pg.ScatterPlotItem(
            size=24,
            symbol=self.SurfaceVesselSymbol,
            brush=pg.mkBrush(*self.ConfirmedTrackBrushColour),
            pen=pg.mkPen(*self.ConfirmedTrackPenColour, width=2),
        )
        self.PpiPlot.addItem(self.ConfirmedTrackScatter)

        self.UpdateBeamLine()

    def CreateMapOverlay(self):
        """Draw the cached regional map beneath all radar PPI layers."""
        if not self.MapEnabled:
            return

        dataset_path = self.MapDatasetPath
        if not os.path.isabs(dataset_path):
            dataset_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), dataset_path)

        try:
            self.MapOverlay = RadarCentredMap(
                dataset_path,
                self.MapLatitudeDeg,
                self.MapLongitudeDeg,
            )
            coastlines = self.MapOverlay.ProjectCoastlines()
            land_polygons = self.MapOverlay.ProjectLandPolygons()
        except Exception as error:
            print(f"Map overlay disabled: {error}")
            self.MapOverlay = None
            return

        if not coastlines:
            return

        clip_path = QtGui.QPainterPath()
        radius_m = self.PolarMaxRangeM
        clip_path.addEllipse(QtCore.QPointF(0.0, 0.0), radius_m, radius_m)
        for polygon in land_polygons:
            if len(polygon) < 3:
                continue
            land_path = QtGui.QPainterPath(QtCore.QPointF(*polygon[0]))
            for east_m, north_m in polygon[1:]:
                land_path.lineTo(east_m, north_m)
            land_path.closeSubpath()
            land_item = QtWidgets.QGraphicsPathItem(land_path.intersected(clip_path))
            land_item.setPen(pg.mkPen(None))
            land_item.setBrush(pg.mkBrush(*self.MapLandColour))
            land_item.setZValue(-100.0)
            self.PpiPlot.addItem(land_item)
            self.MapGraphicsItems.append(land_item)

        for coastline in coastlines:
            coast_item = self.PpiPlot.plot(
                [point[0] for point in coastline],
                [point[1] for point in coastline],
                pen=pg.mkPen(self.MapCoastColour, width=1),
            )
            coast_item.setZValue(-90.0)
            self.MapGraphicsItems.append(coast_item)

        for name, east_m, north_m in self.MapOverlay.ProjectLabels(self.PolarMaxRangeM):
            label = pg.TextItem(name, color=self.MapLabelColour, anchor=(0.5, 0.5))
            label.setPos(east_m, north_m)
            label.setZValue(-80.0)
            self.PpiPlot.addItem(label)
            self.MapGraphicsItems.append(label)

        attribution = pg.TextItem(
            "Map: © OpenStreetMap contributors",
            color=(185, 185, 185, 150),
            anchor=(0.0, 1.0),
        )
        attribution.setPos(-0.96 * radius_m, -0.96 * radius_m)
        attribution.setZValue(-70.0)
        self.PpiPlot.addItem(attribution)
        self.MapGraphicsItems.append(attribution)

    def SetRadarPosition(self, latitude_deg, longitude_deg):
        """Future GPS entry point; keep radar centred and move the map."""
        self.MapLatitudeDeg = float(latitude_deg)
        self.MapLongitudeDeg = float(longitude_deg)
        if self.MapOverlay is None:
            return
        for item in self.MapGraphicsItems:
            try:
                self.PpiPlot.removeItem(item)
            except Exception:
                pass
        self.MapGraphicsItems = []
        self.CreateMapOverlay()

    @staticmethod
    def CreateSurfaceVesselSymbolPath():
        """
        Return a simple NATO-inspired surface vessel symbol for PyQtGraph.

        The path is defined in normalized symbol coordinates and then scaled
        by ScatterPlotItem(size=...). It is deliberately simple: open frame,
        small mast tick, and hull/base cue. This keeps it readable on the PPI
        without relying on external icon files or painter-based drawing.
        """
        Path = QtGui.QPainterPath()

        # Main tactical frame.
        Path.addRect(QtCore.QRectF(-0.64, -0.42, 1.28, 0.84))

        # Larger internal surface-combatant / ship glyph.
        # This is deliberately line-based, not filled, because the PPI uses a
        # transparent symbol brush.  It therefore remains readable on a black
        # background while looking close to the NATO-style surface-combatant cue.

        # Bow / superstructure cue.
        Path.moveTo(0.00, -0.20)
        Path.lineTo(-0.18, 0.02)
        Path.lineTo(0.18, 0.02)
        Path.lineTo(0.00, -0.20)

        # Main hull, made larger than the previous small base cue.
        Path.moveTo(-0.34, 0.10)
        Path.lineTo(-0.22, 0.24)
        Path.lineTo(0.22, 0.24)
        Path.lineTo(0.34, 0.10)
        Path.lineTo(-0.34, 0.10)

        # Short centre mast line.
        Path.moveTo(0.00, -0.30)
        Path.lineTo(0.00, -0.18)

        return Path

    # ------------------------------------------------------------------
    # Dynamic updates
    # ------------------------------------------------------------------

    def UpdatePolarDegreeLabels(self):
        """Draw North-up, clockwise-positive bearing labels around the PPI."""
        if self.PpiPlot is None:
            return

        for Item in getattr(self, "PolarDegreeLabelItems", []):
            try:
                self.PpiPlot.removeItem(Item)
            except Exception:
                pass
        self.PolarDegreeLabelItems = []

        if not self.ShowPolarDegreeLabels and not self.ShowPolarCardinalLabels:
            return

        StepDeg = max(1.0, float(self.PolarDegreeLabelStepDeg))
        LabelRadius = float(self.PolarMaxRangeM) * float(self.PolarDegreeLabelRadiusFraction)
        CardinalLabels = {
            0.0: "N\n0°",
            90.0: "E\n90°",
            180.0: "S\n180°",
            270.0: "W\n270°",
        }

        # Keep labels just inside the outside ring so they remain visible
        # inside the fixed plot range. Cardinal labels are drawn separately so
        # N/E/S/W remain visible even when the degree-label interval changes.
        if self.ShowPolarDegreeLabels:
            AngleDeg = 0.0
            while AngleDeg < 360.0:
                NormalisedAngleDeg = float(AngleDeg % 360.0)
                IsCardinal = any(
                    math.isclose(NormalisedAngleDeg, CardinalDeg, abs_tol=1.0e-9)
                    for CardinalDeg in CardinalLabels
                )
                if not (self.ShowPolarCardinalLabels and IsCardinal):
                    X, Y = self.AngleRangeToXY(NormalisedAngleDeg, LabelRadius)
                    Label = pg.TextItem(
                        f"{int(round(NormalisedAngleDeg))}°",
                        color=self.MutedTextColour,
                        anchor=(0.5, 0.5),
                    )
                    Label.setPos(X, Y)
                    self.PpiPlot.addItem(Label)
                    self.PolarDegreeLabelItems.append(Label)
                AngleDeg += StepDeg

        if self.ShowPolarCardinalLabels:
            for AngleDeg, LabelText in CardinalLabels.items():
                X, Y = self.AngleRangeToXY(AngleDeg, LabelRadius)
                Label = pg.TextItem(
                    LabelText,
                    color=self.TextColour,
                    anchor=(0.5, 0.5),
                )
                Label.setPos(X, Y)
                self.PpiPlot.addItem(Label)
                self.PolarDegreeLabelItems.append(Label)

    def UpdateSectorBoundaryLines(self):
        """Redraw the scan-sector boundary lines after scan limits change."""
        if self.PpiPlot is None:
            return

        # Remove old boundary lines. Range rings are left untouched.
        for Line in getattr(self, "SectorBoundaryLines", []):
            try:
                self.PpiPlot.removeItem(Line)
            except Exception:
                pass
        self.SectorBoundaryLines = []

        BoundaryPen = pg.mkPen(
            self.BoundaryColour,
            width=1,
            style=QtCore.Qt.PenStyle.DashLine,
        )

        for AngleDeg in [self.ScanStartDeg, self.ScanStopDeg]:
            X, Y = self.AngleRangeToXY(AngleDeg, self.PolarMaxRangeM)
            Line = self.PpiPlot.plot([0, X], [0, Y], pen=BoundaryPen)
            self.SectorBoundaryLines.append(Line)

    def UpdateBeamLine(self):
        X, Y = self.AngleRangeToXY(self.BeamAngleDeg, self.PolarMaxRangeM)
        self.BeamLine.setData([0, X], [0, Y])

    def UpdateDetectionScatter(self):
        if not self.ShowRawDetections:
            self.DetectionScatter.setData([], [])
            return

        if len(self.PolarDetectionHistory) == 0:
            self.DetectionScatter.setData([], [])
            return

        XValues = []
        YValues = []
        for Record in self.PolarDetectionHistory:
            X, Y = self.AngleRangeToXY(Record["AzimuthDeg"], Record["RangeM"])
            XValues.append(X)
            YValues.append(Y)

        self.DetectionScatter.setData(XValues, YValues)


    def UpdateTrackScatter(self):
        """Draw current clustered plots and tentative/confirmed tracks on the PPI."""
        # Optional plot/blob centroids for debugging only.
        # Keep disabled during tracker testing, otherwise uninitiated detections
        # appear as fading/vanishing objects outside the tracker.
        PlotX = []
        PlotY = []
        if self.ShowTrackerPlots:
            for Plot in self.LatestPlots:
                try:
                    X, Y = self.AngleRangeToXY(float(Plot.AzimuthDeg), float(Plot.RangeM))
                    PlotX.append(X)
                    PlotY.append(Y)
                except Exception:
                    continue

        if self.PlotScatter is not None:
            self.PlotScatter.setData(PlotX, PlotY)

        TentX = []
        TentY = []
        ConfX = []
        ConfY = []

        # Remove previous text labels before adding current labels.
        for Item in self.TrackTextItems:
            try:
                self.PpiPlot.removeItem(Item)
            except Exception:
                pass
        self.TrackTextItems = []

        if not self.ShowTracks:
            if self.TentativeTrackScatter is not None:
                self.TentativeTrackScatter.setData([], [])
            if self.ConfirmedTrackScatter is not None:
                self.ConfirmedTrackScatter.setData([], [])
            return

        for Track in self.LatestTracks:
            try:
                X, Y = self.AngleRangeToXY(float(Track.AzimuthDeg), float(Track.RangeM))
                Status = str(getattr(Track, "Status", "TENTATIVE")).upper()
                TrackId = int(getattr(Track, "TrackId", 0))

                # Treat anything not explicitly confirmed as an initiation/tentative
                # track.  This covers tracker Status values such as TENTATIVE,
                # CANDIDATE, INITIATING, PENDING, FIRST_HIT, or simply
                # IsConfirmed=False.
                IsConfirmed = bool(getattr(Track, "IsConfirmed", False)) or Status == "CONFIRMED"

                Hits = int(getattr(Track, "Hits", getattr(Track, "HitCount", 1)))

                IsSelected = (
                    self.SelectedTrackId is not None
                    and int(TrackId) == int(self.SelectedTrackId)
                )

                if IsConfirmed:
                    ConfX.append(X)
                    ConfY.append(Y)
                    LabelColour = self.TrackTextColour
                    LabelText = f"> T{TrackId} <" if IsSelected else f"T{TrackId}"
                else:
                    TentX.append(X)
                    TentY.append(Y)
                    LabelColour = self.TentativeTrackTextColour
                    LabelText = f"> t{TrackId} {Hits}/3 <" if IsSelected else f"t{TrackId} {Hits}/3"

                Label = pg.TextItem(LabelText, color=LabelColour, anchor=(-0.2, 1.2))
                Label.setPos(X, Y)
                self.PpiPlot.addItem(Label)
                self.TrackTextItems.append(Label)
            except Exception:
                continue

        if self.TentativeTrackScatter is not None:
            self.TentativeTrackScatter.setData(TentX, TentY)
        if self.ConfirmedTrackScatter is not None:
            self.ConfirmedTrackScatter.setData(ConfX, ConfY)

    def ApplyRangeProfileScale(self):
        """Apply manual or noise-referenced range-profile Y limits."""
        if self.RangePlot is None:
            return

        if self.RangeProfileAutoScale:
            self.RangePlot.enableAutoRange(axis="y", enable=True)
            return

        # Use robust noise-tracking for the lower limit if enabled.
        DisplayMinDb = self.RangeProfileMinDb
        if self.RangeProfileAutoMinDb and self.EstimatedNoiseFloorDb is not None:
            DisplayMinDb = float(self.EstimatedNoiseFloorDb) - self.RangeProfileNoiseMarginDb

        DisplayMaxDb = self.RangeProfileMaxDb
        if (
            self.RangeProfileAutoMaxDb
            and self.EstimatedNoiseFloorDb is not None
            and self.EstimatedRangeProfilePeakDb is not None
        ):
            DisplayMinDb, DisplayMaxDb = (
                self.CalculateNoiseReferencedRangeProfileLimits(
                    NoiseFloorDb=self.EstimatedNoiseFloorDb,
                    PeakDb=self.EstimatedRangeProfilePeakDb,
                    NoiseMarginDb=self.RangeProfileNoiseMarginDb,
                    MinimumSpanAboveNoiseDb=(
                        self.RangeProfileMinimumSpanAboveNoiseDb
                    ),
                    PeakHeadroomDb=self.RangeProfilePeakHeadroomDb,
                )
            )

        # Guard against accidental reversed/zero span limits.
        if DisplayMaxDb <= DisplayMinDb:
            DisplayMaxDb = DisplayMinDb + 10.0

        self.RangeProfileDisplayMinDb = float(DisplayMinDb)
        self.RangeProfileDisplayMaxDb = float(DisplayMaxDb)

        self.RangePlot.enableAutoRange(axis="y", enable=False)
        self.RangePlot.setYRange(
            DisplayMinDb,
            DisplayMaxDb,
            padding=0.0,
        )

        # Show the active automatic upper limit in the existing Max dB box.
        # Do not overwrite text while the operator is editing it.
        MaxWidget = self.ControlWidgets.get("RangeProfileMaxDb")
        if (
            self.RangeProfileAutoMaxDb
            and MaxWidget is not None
            and not MaxWidget.hasFocus()
        ):
            MaxWidget.setText(f"{DisplayMaxDb:.1f}")

    @staticmethod
    def CalculateNoiseReferencedRangeProfileLimits(
        NoiseFloorDb,
        PeakDb,
        NoiseMarginDb=5.0,
        MinimumSpanAboveNoiseDb=20.0,
        PeakHeadroomDb=3.0,
    ):
        """Return stable display limits referenced to noise and current peak."""
        return CalculateNoiseReferencedRangeProfileLimits(
            NoiseFloorDb=NoiseFloorDb,
            PeakDb=PeakDb,
            NoiseMarginDb=NoiseMarginDb,
            MinimumSpanAboveNoiseDb=MinimumSpanAboveNoiseDb,
            PeakHeadroomDb=PeakHeadroomDb,
        )

    def EstimateRangeProfileNoiseFloorDb(self, RangeAxisM, RangeProfileDb):
        """
        Estimate the noise floor robustly from the range profile.

        Strong targets are handled by:
          1. Removing bins close to current dwell detections.
          2. Using only the lower/central part of the remaining distribution,
             rather than the whole mean, so large peaks do not pull the estimate up.
        """
        try:
            Values = np.asarray(RangeProfileDb, dtype=float)
            Ranges = np.asarray(RangeAxisM, dtype=float)
            Mask = np.isfinite(Values)

            # Exclude bins around current detections so real targets do not bias
            # the noise-floor estimate upward.
            if self.LatestDetections is not None and len(self.LatestDetections) > 0:
                for Detection in self.LatestDetections:
                    DetectionRangeM = float(getattr(Detection, "RangeM", np.nan))
                    if not np.isfinite(DetectionRangeM):
                        continue
                    Idx = int(np.argmin(np.abs(Ranges - DetectionRangeM)))
                    StartIdx = max(0, Idx - self.RangeProfileDetectionExclusionBins)
                    StopIdx = min(Mask.size, Idx + self.RangeProfileDetectionExclusionBins + 1)
                    Mask[StartIdx:StopIdx] = False

            CandidateValues = Values[Mask]
            CandidateValues = CandidateValues[np.isfinite(CandidateValues)]
            if CandidateValues.size < 20:
                CandidateValues = Values[np.isfinite(Values)]
            if CandidateValues.size == 0:
                return

            Percentile = np.clip(self.RangeProfileNoisePercentile, 10.0, 95.0)
            CutoffDb = np.nanpercentile(CandidateValues, Percentile)
            NoiseBins = CandidateValues[CandidateValues <= CutoffDb]
            if NoiseBins.size < 10:
                NoiseBins = CandidateValues

            NoiseFloorDb = float(np.nanmean(NoiseBins))

            # Smooth the display estimate so the axis does not jump every dwell.
            Alpha = float(np.clip(self.RangeProfileNoiseAlpha, 0.01, 1.0))
            if self.EstimatedNoiseFloorDb is None or not np.isfinite(self.EstimatedNoiseFloorDb):
                self.EstimatedNoiseFloorDb = NoiseFloorDb
            else:
                self.EstimatedNoiseFloorDb = (1.0 - Alpha) * self.EstimatedNoiseFloorDb + Alpha * NoiseFloorDb

        except Exception:
            return

    def UpdateRangeProfile(self):
        if self.LatestProcessed is None:
            return

        Processed = self.LatestProcessed
        HasRemoteProfile = hasattr(Processed, "DisplayRangeProfileDb")
        if (
            not hasattr(Processed, "RangeAxisM")
            or (not HasRemoteProfile and not hasattr(Processed, "MagnitudeDb"))
        ):
            return

        try:
            if HasRemoteProfile:
                # The radar-server worker has already selected and peak-pooled
                # a bounded display profile. Raw IQ/range-Doppler data never
                # crosses the operator link.
                RangeProfileDb = np.asarray(
                    Processed.DisplayRangeProfileDb,
                    dtype=float,
                )
            else:
                RangeProfileDb = SelectRangeProfileDb(
                    Processed.MagnitudeDb,
                    getattr(Processed, "DopplerAxisHz", None),
                    Mode=self.RangeProfileDopplerMode,
                )
            RangeAxisM = np.asarray(Processed.RangeAxisM)
        except Exception:
            return

        RangeMask = RangeAxisM <= self.MaxDisplayRangeM
        RangeAxisM = RangeAxisM[RangeMask]
        RangeProfileDb = RangeProfileDb[RangeMask]

        if RangeAxisM.size == 0:
            return

        # Downsample safely for display.
        #
        # IMPORTANT:
        # Do NOT use simple decimation such as RangeProfileDb[::Step].
        # Radar targets can be only one range bin wide, so simple decimation can
        # skip the exact target bin and make a real detection disappear from the
        # range-profile display. Use max-pooling instead so narrow peaks survive.
        RequestedStep = max(1, self.DecimateRangeProfile)

        # If MaxRangeProfilePoints is set, choose a display step, but still use
        # max-pooling. Set QtMaxRangeProfilePoints to 0 or a large number to
        # force full-resolution plotting.
        if self.MaxRangeProfilePoints is not None and self.MaxRangeProfilePoints > 0:
            AutoStep = int(math.ceil(RangeAxisM.size / float(self.MaxRangeProfilePoints)))
        else:
            AutoStep = 1

        Step = max(1, RequestedStep, AutoStep)

        if Step <= 1:
            X = RangeAxisM
            Y = RangeProfileDb
        else:
            NumBins = (RangeProfileDb.size // Step) * Step

            if NumBins <= 0:
                X = RangeAxisM
                Y = RangeProfileDb
            else:
                ProfileBlocks = RangeProfileDb[:NumBins].reshape(-1, Step)
                RangeBlocks = RangeAxisM[:NumBins].reshape(-1, Step)

                # Preserve one-bin peaks by selecting the maximum bin inside
                # each displayed block.
                MaxIndex = np.nanargmax(ProfileBlocks, axis=1)
                RowIndex = np.arange(ProfileBlocks.shape[0])

                Y = ProfileBlocks[RowIndex, MaxIndex]
                X = RangeBlocks[RowIndex, MaxIndex]

                # Include leftover bins at the end of the array.
                if NumBins < RangeProfileDb.size:
                    TailProfile = RangeProfileDb[NumBins:]
                    TailRange = RangeAxisM[NumBins:]
                    if TailProfile.size > 0:
                        TailIndex = int(np.nanargmax(TailProfile))
                        Y = np.append(Y, TailProfile[TailIndex])
                        X = np.append(X, TailRange[TailIndex])

        # Estimate the noise floor from the full-resolution visible profile,
        # not from the downsampled display curve.
        if self.RangeProfileAutoMinDb or self.RangeProfileAutoMaxDb:
            self.EstimateRangeProfileNoiseFloorDb(RangeAxisM, RangeProfileDb)

        FiniteProfile = RangeProfileDb[np.isfinite(RangeProfileDb)]
        if FiniteProfile.size > 0:
            self.EstimatedRangeProfilePeakDb = float(
                np.max(FiniteProfile)
            )

        self.RangeProfileCurve.setData(X, Y)
        self.ApplyRangeProfileScale()

        # Optional current-dwell detection markers on the range profile.
        # Disabled by default during tracker testing so raw CFAR detections do
        # not appear to be tracker objects.
        MarkerX = []
        MarkerY = []
        if self.ShowRangeDetectionMarkers:
            for Detection in self.LatestDetections:
                DetectionRangeM = float(getattr(Detection, "RangeM", np.nan))
                if not np.isfinite(DetectionRangeM):
                    continue
                if DetectionRangeM < 0 or DetectionRangeM > self.MaxDisplayRangeM:
                    continue

                Idx = int(np.argmin(np.abs(RangeAxisM - DetectionRangeM)))
                MarkerX.append(DetectionRangeM)
                MarkerY.append(float(RangeProfileDb[Idx]))

        if hasattr(self, "RangeDetectionScatter") and self.RangeDetectionScatter is not None:
            self.RangeDetectionScatter.setData(MarkerX, MarkerY)
    def OnPpiMouseClicked(self, MouseEvent):
        """
        Select the nearest displayed track when the operator left-clicks on the PPI.

        The click is converted from Qt scene coordinates into the PPI's x/y metre
        coordinates, then compared with the latest track positions.  Clicking empty
        space clears the selection.
        """
        try:
            # PyQt5 compatibility: Qt.LeftButton is always present; newer bindings
            # may also expose Qt.MouseButton.LeftButton.
            LeftButton = getattr(QtCore.Qt, "LeftButton", None)
            if LeftButton is None:
                LeftButton = QtCore.Qt.MouseButton.LeftButton
            if MouseEvent.button() != LeftButton:
                return

            if self.PpiPlot is None or self.PpiPlot.plotItem is None:
                return

            ScenePos = MouseEvent.scenePos()
            ViewBox = self.PpiPlot.plotItem.vb
            if not ViewBox.sceneBoundingRect().contains(ScenePos):
                return

            MousePoint = ViewBox.mapSceneToView(ScenePos)
            ClickX = float(MousePoint.x())
            ClickY = float(MousePoint.y())

            BestTrack = None
            BestDistanceM = None

            for Track in self.LatestTracks:
                try:
                    TrackX, TrackY = self.AngleRangeToXY(
                        float(getattr(Track, "AzimuthDeg")),
                        float(getattr(Track, "RangeM")),
                    )
                    DistanceM = math.hypot(TrackX - ClickX, TrackY - ClickY)

                    if DistanceM <= self.TrackClickGateM:
                        if BestDistanceM is None or DistanceM < BestDistanceM:
                            BestDistanceM = DistanceM
                            BestTrack = Track
                except Exception:
                    continue

            if BestTrack is None:
                self.SelectedTrack = None
                self.SelectedTrackId = None
            else:
                self.SelectedTrack = BestTrack
                self.SelectedTrackId = int(getattr(BestTrack, "TrackId", -1))

            self.UpdateTargetDetailsPanel()
            self.UpdateTrackScatter()

        except Exception:
            # Display mouse handling must never stop the radar update loop.
            return

    def RefreshSelectedTrack(self):
        """
        Keep the selected target reference tied to the latest tracker output.
        """
        if self.SelectedTrackId is None:
            self.SelectedTrack = None
            return

        for Track in self.LatestTracks:
            try:
                if int(getattr(Track, "TrackId", -1)) == int(self.SelectedTrackId):
                    self.SelectedTrack = Track
                    return
            except Exception:
                continue

        # The selected track has disappeared or been deleted by the tracker.
        self.SelectedTrack = None
        self.SelectedTrackId = None

    def UpdateTargetDetailsPanel(self):
        """
        Print selected target details in the side panel.
        """
        if self.TargetDetailsLabel is None:
            return

        if self.SelectedTrack is None:
            self.TargetDetailsLabel.setText(
                "SELECTED TARGET\n"
                "None\n\n"
                "Left-click near a track on the PPI."
            )
            return

        Track = self.SelectedTrack

        def GetFloat(*Names, Default=0.0):
            for Name in Names:
                if hasattr(Track, Name):
                    try:
                        return float(getattr(Track, Name))
                    except Exception:
                        pass
            return float(Default)

        def GetInt(*Names, Default=0):
            for Name in Names:
                if hasattr(Track, Name):
                    try:
                        return int(getattr(Track, Name))
                    except Exception:
                        pass
            return int(Default)

        TrackId = GetInt("TrackId", Default=-1)
        Status = str(getattr(Track, "Status", "UNKNOWN")).upper()
        IsConfirmed = bool(getattr(Track, "IsConfirmed", False)) or Status == "CONFIRMED"
        TrackType = str(getattr(Track, "TrackType", "SURFACE VESSEL"))
        TrackSource = str(
            getattr(Track, "TrackSource", self.DisplayTrackSourceApplied)
        ).upper()

        RangeM = GetFloat("RangeM")
        AzimuthDeg = GetFloat("AzimuthDeg")
        RangeRateMps = GetFloat("RangeRateMps", "RangeRate", "VelocityMps")
        AzimuthRateDps = GetFloat("AzimuthRateDps", "AngleRateDps", "BearingRateDps")
        Hits = GetInt("Hits", "HitCount")
        Misses = GetInt("Misses", "MissedCount")
        Age = GetInt("Age", "ScanAge")

        Text = (
            "SELECTED TARGET\n"
            "---------------\n"
            f"Track ID:   T{TrackId}\n"
            f"Status:     {'CONFIRMED' if IsConfirmed else Status}\n"
            f"Source:     {TrackSource}\n"
            f"Type:       {TrackType}\n"
            f"Range:      {RangeM:8.1f} m\n"
            f"Azimuth:    {AzimuthDeg:8.2f} deg\n"
            f"R-rate:     {RangeRateMps:8.2f} m/s\n"
            f"Az-rate:    {AzimuthRateDps:8.3f} deg/s\n"
            f"Hits:       {Hits}\n"
            f"Misses:     {Misses}\n"
            f"Age:        {Age}"
        )
        self.TargetDetailsLabel.setText(Text)

    def UpdateStatusPanel(self):
        NumDetections = len(self.LatestDetections)
        NumPlots = len(self.LatestPlots)
        NumConfirmed = sum(
            1 for Track in self.LatestTracks
            if bool(getattr(Track, "IsConfirmed", False)) or str(getattr(Track, "Status", "")).upper() == "CONFIRMED"
        )
        NumTentative = max(0, len(self.LatestTracks) - NumConfirmed)
        if not self.TransmitAvailable:
            TransmitStatus = "INHIBITED (RX ONLY)"
        elif self.TransmitEnabled:
            TransmitStatus = "ENABLED"
        else:
            TransmitStatus = "OFF"

        Lines = [
            "Vanguard X Radar",
            "----------------",
            f"Link:   {self.RadarLinkStatus}",
            f"Mode:   {self.DisplayMode}",
            f"Scan:   {self.ScanEnabled}",
            f"Tx:     {TransmitStatus}",
            f"Beam:   {self.BeamAngleDeg:.1f} deg",
            f"Track src: {self.DisplayTrackSourceApplied}",
            f"Wave:   {self.AppliedWaveformId}",
            (
                f"Timing: {self.AppliedPrfHz / 1000.0:.2f} kHz, "
                f"{self.AppliedPulsesPerCpi} p, "
                f"{self.AppliedCpiDurationSec * 1e3:.1f} ms, "
                f"R{self.AppliedMaximumRangeM / 1000.0:.1f}k"
            ),
            f"Dwell:  {self.UpdateCounter}",
            f"Dets:   {NumDetections} ({'shown' if self.ShowRawDetections else 'hidden'})",
            f"Plots:  {NumPlots} ({'shown' if self.ShowTrackerPlots else 'hidden'})",
            f"Tent:   {NumTentative}",
            f"Tracks: {NumConfirmed}",
        ]

        if self.DisplayTrackSourceFallback:
            Lines.append(
                "Track fallback: "
                f"{self.DisplayTrackSourceRequested} -> "
                f"{self.DisplayTrackSourceApplied} "
                f"({self.DisplayTrackSourceReason})"
            )

        if self.LatestProcessed is not None:
            Diagnostics = getattr(self.LatestProcessed, "Diagnostics", {})
            Lines.append(f"Peak R: {Diagnostics.get('PeakRangeM', 0.0):.1f} m")
            Lines.append(f"Peak V: {Diagnostics.get('PeakVelocityMps', 0.0):.1f} m/s")

        if self.EstimatedNoiseFloorDb is not None:
            Lines.append(f"Noise:  {self.EstimatedNoiseFloorDb:.1f} dB")
            Lines.append(
                f"Y-axis: {self.RangeProfileDisplayMinDb:.1f} to "
                f"{self.RangeProfileDisplayMaxDb:.1f} dB"
            )

        self.StatusLabel.setText("\n".join(Lines))
        self.UpdatePersistentOperatorHeader()

    # ------------------------------------------------------------------
    # Detection history
    # ------------------------------------------------------------------

    def AppendPolarDetections(self, Detections):
        if not self.ShowRawDetections or self.MaxPolarDetections <= 0:
            self.PolarDetectionHistory = []
            return

        for Detection in Detections[: self.MaxDetectionsPlottedPerDwell]:
            AzimuthDeg = getattr(Detection, "AzimuthDeg", self.BeamAngleDeg)
            RangeM = getattr(Detection, "RangeM", None)
            AmplitudeDb = getattr(Detection, "AmplitudeDb", 0.0)
            if RangeM is None:
                continue

            self.PolarDetectionHistory.append(
                {
                    "AzimuthDeg": float(AzimuthDeg),
                    "RangeM": float(RangeM),
                    "AmplitudeDb": float(AmplitudeDb),
                }
            )

        if len(self.PolarDetectionHistory) > self.MaxPolarDetections:
            self.PolarDetectionHistory = self.PolarDetectionHistory[-self.MaxPolarDetections :]

    # ------------------------------------------------------------------
    # Controls
    # ------------------------------------------------------------------

    def OnStartScan(self):
        # Operator pressed Start. This must always start scanning,
        # independent of the initial startup setting.
        self.DisplayMode = "SCAN"
        self.ScanEnabled = True
        if self.TransmitAvailable:
            self.TransmitEnabled = True
        self.ManualNudgeDeltaDeg = 0.0
        self.UpdateStatusPanel()

    def OnStartStare(self):
        # STARE is an explicit radar operating command, parallel to SCAN.
        # Hold the measured beam position and arm RF only when the configured
        # source has declared transmit capability.
        self.DisplayMode = "STARE"
        self.ScanEnabled = False
        if self.TransmitAvailable:
            self.TransmitEnabled = True
        self.ManualNudgeDeltaDeg = 0.0
        self.UpdateStatusPanel()

    def OnStop(self):
        # STOP is a hard one-shot operator event. The main loop uses
        # StopCommandId to send a real X6-60 stop and then remains in idle.
        if self.MissionPage is not None:
            self.MissionPage.RequestStopFromPersistentStop()
        self.DisplayMode = "STOP"
        self.ScanEnabled = False
        self.TransmitEnabled = False
        self.StopCommandId += 1
        self.UpdateStatusPanel()

    def OnStopTransmit(self):
        self.TransmitEnabled = False
        self.UpdateStatusPanel()

    def OnExit(self):
        self.ExitRequested = True
        self.ScanEnabled = False
        self.TransmitEnabled = False
        self.UpdateStatusPanel()
        self.Window.close()

    def OnModeChanged(self, Label):
        # Retained for compatibility with callers outside this display.
        self.DisplayMode = Label
        self.ScanEnabled = True if Label == "SCAN" else False
        self.UpdateStatusPanel()

    def OnBeamLeft(self):
        # One-shot manual nudge event. BeamAngleDeg is display state and may be
        # overwritten by measured X6-60 angle, so the main loop must consume this
        # explicit delta instead of inferring from BeamAngleDeg.  The visible
        # Step control is the operator's nudge increment as well as the sector
        # scan step setting.
        self.DisplayMode = "STARE"
        self.ScanEnabled = False
        if self.TransmitAvailable:
            self.TransmitEnabled = True
        self.ManualNudgeDeltaDeg = -abs(float(self.ScanStepDeg))
        self.ManualNudgeCommandId += 1
        self.UpdateStatusPanel()

    def OnBeamRight(self):
        # One-shot manual nudge event.
        self.DisplayMode = "STARE"
        self.ScanEnabled = False
        if self.TransmitAvailable:
            self.TransmitEnabled = True
        self.ManualNudgeDeltaDeg = abs(float(self.ScanStepDeg))
        self.ManualNudgeCommandId += 1
        self.UpdateStatusPanel()

    def OnScanStartChanged(self):
        try:
            self.ScanStartDeg = float(self.ControlWidgets["ScanStart"].text())
            self.Config["ScanStartDeg"] = self.ScanStartDeg
            self.UpdateSectorBoundaryLines()
            self.UpdateStatusPanel()
        except ValueError:
            self.ControlWidgets["ScanStart"].setText(str(self.ScanStartDeg))

    def OnScanStopChanged(self):
        try:
            self.ScanStopDeg = float(self.ControlWidgets["ScanStop"].text())
            self.Config["ScanStopDeg"] = self.ScanStopDeg
            self.UpdateSectorBoundaryLines()
            self.UpdateStatusPanel()
        except ValueError:
            self.ControlWidgets["ScanStop"].setText(str(self.ScanStopDeg))

    def OnScanStepChanged(self):
        try:
            NewStepDeg = float(self.ControlWidgets["ScanStep"].text())
            if NewStepDeg <= 0:
                raise ValueError
            self.ScanStepDeg = NewStepDeg
            self.Config["ScanStepDeg"] = self.ScanStepDeg
            self.UpdateStatusPanel()
        except ValueError:
            self.ControlWidgets["ScanStep"].setText(str(self.ScanStepDeg))

    def OnScanRateChanged(self, NewRateDegPerSec):
        """Apply the front-dashboard sector scan-rate selection."""

        self.ScanRateDegPerSec = min(
            max(float(NewRateDegPerSec), self.MinScanRateDegPerSec),
            self.MaxScanRateDegPerSec,
        )
        self.Config["X660ScanSlewRateDegPerSec"] = self.ScanRateDegPerSec
        self.UpdateStatusPanel()

    def OnTimingApply(self):
        """Queue the visible timing selection for Main to validate."""

        self.SelectedWaveformId = str(
            self.ControlWidgets["WaveformId"].currentText()
        )
        self.SelectedPrfHz = float(
            self.ControlWidgets["PrfKHz"].value()
        ) * 1000.0
        self.SelectedPulsesPerCpi = int(
            self.ControlWidgets["PulsesPerCpi"].value()
        )
        self.SelectedMaximumRangeM = float(
            self.ControlWidgets["MaximumRangeKm"].value()
        ) * 1000.0
        self.TimingSelectionRevision += 1
        self.TimingApplicationMessage = "Pending next dwell"
        self.TimingFeedbackLabel.setStyleSheet("color: #ffcc00;")
        self.TimingFeedbackLabel.setText(self.TimingApplicationMessage)
        self.UpdateTimingSummary()
        self.UpdateStatusPanel()

    def OnSystemModeChanged(self, Mode):
        """Request a complete system-mode restart, but only from STOP."""

        Requested = str(Mode).upper()
        if Requested == self.SystemMode:
            return

        if self.DisplayMode != "STOP" or self.ScanEnabled:
            QtWidgets.QMessageBox.warning(
                None,
                "Stop radar first",
                "System mode can only be changed while the radar is stopped.",
            )
            self.ControlWidgets["SystemMode"].blockSignals(True)
            self.ControlWidgets["SystemMode"].setCurrentText(self.SystemMode)
            self.ControlWidgets["SystemMode"].blockSignals(False)
            return

        if Requested == "HARD":
            Answer = QtWidgets.QMessageBox.question(
                None,
                "Enable Vanguard X hardware",
                "HARD will restart the application, connect the Ettus in "
                "receive-only mode, and enable operational X6-60 motion.\n\n"
                "TX remains inhibited. Confirm the X6-60 motion area is clear "
                "and the hardware is ready.",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if Answer != QtWidgets.QMessageBox.Yes:
                self.ControlWidgets["SystemMode"].blockSignals(True)
                self.ControlWidgets["SystemMode"].setCurrentText(self.SystemMode)
                self.ControlWidgets["SystemMode"].blockSignals(False)
                return

        if Requested == "RF LOOPBACK":
            Answer = QtWidgets.QMessageBox.warning(
                None,
                "Enable guarded RF loopback",
                "RF LOOPBACK enables timed Ettus RF transmission.\n\n"
                "Confirm ALL of the following:\n"
                "• TX/RX is connected to RX2 through at least 30 dB attenuation.\n"
                "• TRM, PA and antenna are disconnected or physically isolated.\n"
                "• The X6-60 will remain stopped.\n"
                "• Both external TX ATR and RX ATR must be forced low and "
                "verified before TX is prepared.\n\n"
                "Continue?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if Answer != QtWidgets.QMessageBox.Yes:
                self.ControlWidgets["SystemMode"].blockSignals(True)
                self.ControlWidgets["SystemMode"].setCurrentText(self.SystemMode)
                self.ControlWidgets["SystemMode"].blockSignals(False)
                return

        self.RequestedSystemMode = Requested
        self.SystemModeRevision += 1
        self.SystemModeFeedbackLabel.setStyleSheet("color: #ffb347;")
        self.SystemModeFeedbackLabel.setText("Restart requested")

    def OnRangeProfileMaxDbChanged(self):
        try:
            NewMaxDb = float(self.ControlWidgets["RangeProfileMaxDb"].text())
            CurrentMinDb = self.RangeProfileMinDb
            if self.RangeProfileAutoMinDb and self.EstimatedNoiseFloorDb is not None:
                CurrentMinDb = self.EstimatedNoiseFloorDb - self.RangeProfileNoiseMarginDb
            if NewMaxDb <= CurrentMinDb:
                raise ValueError
            self.RangeProfileMaxDb = NewMaxDb
            self.Config["RangeProfileMaxDb"] = self.RangeProfileMaxDb
            self.RangeProfileAutoScale = False
            self.Config["RangeProfileAutoScale"] = False
            self.RangeProfileAutoMaxDb = False
            self.Config["RangeProfileAutoMaxDb"] = False
            self.ApplyRangeProfileScale()
            self.UpdateStatusPanel()
        except ValueError:
            self.ControlWidgets["RangeProfileMaxDb"].setText(str(self.RangeProfileMaxDb))

    def OnSaveDataEnabledChanged(self):
        """Update the operator logging request from the Save checkbox."""
        self.SaveDataEnabled = bool(self.ControlWidgets["SaveDataEnabled"].isChecked())
        self.Config["DataLoggingEnabled"] = self.SaveDataEnabled

    def OnDataLogFilenameChanged(self):
        NewFilename = self.ControlWidgets["DataLogFilename"].text().strip()
        if NewFilename == "":
            NewFilename = "datafile1.h5"
        if not NewFilename.lower().endswith(".h5"):
            NewFilename += ".h5"
        self.DataLogFilename = NewFilename
        self.Config["DataLogFilename"] = self.DataLogFilename
        self.ControlWidgets["DataLogFilename"].setText(self.DataLogFilename)
        self.UpdateStatusPanel()

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    @staticmethod
    def AngleRangeToXY(AngleDeg, RangeM):
        """Convert radar bearing/range to East/North Cartesian coordinates.

        Vanguard X uses the conventional North-up PPI convention: 0 degrees
        is North (positive Y), 90 degrees is East (positive X), and bearing
        increases clockwise.
        """
        AngleRad = np.deg2rad(AngleDeg)
        X = RangeM * np.sin(AngleRad)
        Y = RangeM * np.cos(AngleRad)
        return float(X), float(Y)
