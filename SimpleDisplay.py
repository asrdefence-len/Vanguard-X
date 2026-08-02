"""
===============================================================================
ASR Defence X-Band Radar Prototype
SimpleDisplay.py
===============================================================================

Foreword
--------
This file contains a simple live display module.

Current display behaviour
-------------------------
    - compact terminal status, not a wall of text every dwell
    - reuses the same range profile figure
    - reuses the same range-Doppler figure
    - maintains a live polar detection plot during scan

The polar display uses the current radar boresight angle as the azimuth of each
CFAR detection. That is suitable for this first scanning simulation because the
receiver is currently a mechanically/electronically scanned pencil beam model.
===============================================================================
"""

import numpy as np
import matplotlib.pyplot as plt


class SimpleDisplay:
    """
    Simple terminal and matplotlib display.
    """

    def __init__(self, Config):
        self.Config = Config
        self.UpdateCounter = 0

        self.RangeProfileFigure = None
        self.RangeProfileAxes = None
        self.RangeProfileLine = None
        self.RangeProfileThresholdLine = None

        self.RangeDopplerFigure = None
        self.RangeDopplerAxes = None
        self.RangeDopplerImage = None
        self.RangeDopplerColorbar = None

        self.PolarFigure = None
        self.PolarAxes = None
        self.PolarScatter = None
        self.PolarBeamLines = []
        self.PolarBoresightLine = None
        self.PolarBeamTip = None
        self.PolarAnglesRad = []
        self.PolarRangesM = []
        self.PolarAmplitudesDb = []

        if self.Config.get("UpdatePlots", True):
            plt.ion()

    # -------------------------------------------------------------------------
    # Public update function
    # -------------------------------------------------------------------------

    def Update(self, Processed, Detections):
        """
        Display processed data and detections.
        """

        self.UpdateCounter += 1

        PrintEveryNDwells = int(self.Config.get("PrintEveryNDwells", 1))
        ShouldPrint = (
            PrintEveryNDwells > 0
            and (self.UpdateCounter == 1 or self.UpdateCounter % PrintEveryNDwells == 0)
        )

        if ShouldPrint:
            self.PrintCompactStatus(Processed, Detections)

        if not self.Config.get("UpdatePlots", True):
            return

        DetailPlotEveryNDwells = int(self.Config.get("DetailPlotEveryNDwells", 20))
        PolarPlotEveryNDwells = int(self.Config.get("PolarPlotEveryNDwells", 1))

        ShouldUpdateDetailPlots = (
            DetailPlotEveryNDwells > 0
            and (self.UpdateCounter == 1 or self.UpdateCounter % DetailPlotEveryNDwells == 0)
        )

        ShouldUpdatePolarPlot = (
            PolarPlotEveryNDwells > 0
            and (self.UpdateCounter == 1 or self.UpdateCounter % PolarPlotEveryNDwells == 0)
        )

        if ShouldUpdateDetailPlots and self.Config.get("ShowRangeProfile", True):
            self.PlotRangeProfile(Processed)

        if ShouldUpdateDetailPlots and self.Config.get("ShowRangeDopplerMap", True):
            self.PlotRangeDopplerMap(Processed)

        if ShouldUpdatePolarPlot and self.Config.get("ShowPolarDetections", True):
            self.UpdatePolarDetections(Processed, Detections)

        plt.pause(float(self.Config.get("PlotPauseS", 0.001)))

    # -------------------------------------------------------------------------
    # Terminal output
    # -------------------------------------------------------------------------

    def PrintCompactStatus(self, Processed, Detections):
        """
        Print a compact status line rather than a full dwell report each time.
        """

        Diagnostics = Processed.Diagnostics

        BoresightDeg = Diagnostics.get("BoresightDeg", self.Config.get("BoresightDeg", 0.0))
        ScanCycle = Diagnostics.get("ScanCycle", 1)
        PeakRangeM = Diagnostics.get("PeakRangeM", 0.0)
        PeakDopplerHz = Diagnostics.get("PeakDopplerHz", 0.0)
        PeakVelocityMps = Diagnostics.get("PeakVelocityMps", 0.0)
        InputSnrDb = Diagnostics.get("InputSnrDb", None)

        if InputSnrDb is None:
            SnrText = ""
        else:
            SnrText = f", input SNR {InputSnrDb:.1f} dB"

        print(
            f"Dwell {Processed.DwellId:5d} | "
            f"scan {ScanCycle:3d} | "
            f"az {BoresightDeg:6.1f} deg | "
            f"detections {len(Detections):3d} | "
            f"peak {PeakRangeM:8.1f} m, "
            f"{PeakDopplerHz:8.1f} Hz, "
            f"{PeakVelocityMps:6.2f} m/s"
            f"{SnrText}"
        )

    # -------------------------------------------------------------------------
    # Range profile
    # -------------------------------------------------------------------------

    def PlotRangeProfile(self, Processed):
        """
        Reuse one range profile figure.
        """

        RangeProfileDb = np.max(Processed.MagnitudeDb, axis=0)

        if self.RangeProfileFigure is None:
            self.RangeProfileFigure, self.RangeProfileAxes = plt.subplots(figsize=(10, 5))

            (self.RangeProfileLine,) = self.RangeProfileAxes.plot(
                Processed.RangeAxisM,
                RangeProfileDb,
                linewidth=1.2,
                label="Max over Doppler",
            )

            if "ThresholdDb" in self.Config:
                self.RangeProfileThresholdLine = self.RangeProfileAxes.axhline(
                    self.Config["ThresholdDb"],
                    linestyle="--",
                    linewidth=1.0,
                    label="Detection threshold",
                )

            self.RangeProfileAxes.set_xlabel("Range (m)")
            self.RangeProfileAxes.set_ylabel("Magnitude (dB)")
            self.RangeProfileAxes.grid(True)
            self.RangeProfileAxes.set_xlim(0, 15000)
            self.RangeProfileAxes.legend()
            self.RangeProfileFigure.tight_layout()

        self.RangeProfileLine.set_ydata(RangeProfileDb)

        YMin = np.nanmin(RangeProfileDb)
        YMax = np.nanmax(RangeProfileDb)
        if np.isfinite(YMin) and np.isfinite(YMax) and YMax > YMin:
            Margin = 5.0
            self.RangeProfileAxes.set_ylim(YMin - Margin, YMax + Margin)

        BoresightDeg = Processed.Diagnostics.get("BoresightDeg", self.Config.get("BoresightDeg", 0.0))
        self.RangeProfileAxes.set_title(
            f"Range Profile - Dwell {Processed.DwellId}, Az {BoresightDeg:.1f} deg"
        )

        self.RangeProfileFigure.canvas.draw_idle()

    # -------------------------------------------------------------------------
    # Range-Doppler map
    # -------------------------------------------------------------------------

    def PlotRangeDopplerMap(self, Processed):
        """
        Reuse one range-Doppler map figure.
        """

        MaxRangeM = 15000.0
        RangeMask = Processed.RangeAxisM <= MaxRangeM

        PlotData = Processed.MagnitudeDb[:, RangeMask]
        PlotRanges = Processed.RangeAxisM[RangeMask]
        PlotVelocities = Processed.VelocityAxisMps

        if self.RangeDopplerFigure is None:
            self.RangeDopplerFigure, self.RangeDopplerAxes = plt.subplots(figsize=(10, 6))

            self.RangeDopplerImage = self.RangeDopplerAxes.imshow(
                PlotData,
                aspect="auto",
                origin="lower",
                extent=[
                    PlotRanges[0],
                    PlotRanges[-1],
                    PlotVelocities[0],
                    PlotVelocities[-1],
                ],
            )

            self.RangeDopplerAxes.set_xlabel("Range (m)")
            self.RangeDopplerAxes.set_ylabel("Radial velocity (m/s)")
            self.RangeDopplerColorbar = self.RangeDopplerFigure.colorbar(
                self.RangeDopplerImage,
                ax=self.RangeDopplerAxes,
                label="Magnitude (dB)",
            )
            self.RangeDopplerFigure.tight_layout()

        self.RangeDopplerImage.set_data(PlotData)

        VMin = np.nanpercentile(PlotData, 5)
        VMax = np.nanpercentile(PlotData, 99.5)
        if np.isfinite(VMin) and np.isfinite(VMax) and VMax > VMin:
            self.RangeDopplerImage.set_clim(VMin, VMax)

        BoresightDeg = Processed.Diagnostics.get("BoresightDeg", self.Config.get("BoresightDeg", 0.0))
        self.RangeDopplerAxes.set_title(
            f"Range-Doppler Map - Dwell {Processed.DwellId}, Az {BoresightDeg:.1f} deg"
        )

        self.RangeDopplerFigure.canvas.draw_idle()

    # -------------------------------------------------------------------------
    # Polar detection display
    # -------------------------------------------------------------------------
    # PATCH_MARKER_DARK_POLAR_DISPLAY_V1

    def UpdatePolarDetections(self, Processed, Detections):
        """
        Accumulate detections onto a live tactical-style polar plot.

        Since this is a scanning pencil-beam simulation, each detection is plotted
        at the current boresight angle.
        """

        BoresightDeg = Processed.Diagnostics.get("BoresightDeg", self.Config.get("BoresightDeg", 0.0))
        BoresightRad = np.deg2rad(BoresightDeg)

        DetectionsToPlot = list(Detections)
        MaxPerDwell = int(self.Config.get("MaxDetectionsPlottedPerDwell", 30))

        if len(DetectionsToPlot) > MaxPerDwell:
            DetectionsToPlot = sorted(
                DetectionsToPlot,
                key=lambda d: getattr(d, "AmplitudeDb", -300.0),
                reverse=True,
            )[:MaxPerDwell]

        for Det in DetectionsToPlot:
            RangeM = float(getattr(Det, "RangeM", 0.0))
            AmpDb = float(getattr(Det, "AmplitudeDb", 0.0))

            if RangeM <= 0.0:
                continue

            if RangeM > float(self.Config.get("PolarMaxRangeM", 15000.0)):
                continue

            self.PolarAnglesRad.append(BoresightRad)
            self.PolarRangesM.append(RangeM)
            self.PolarAmplitudesDb.append(AmpDb)

        MaxPolarDetections = int(self.Config.get("MaxPolarDetections", 3000))
        if len(self.PolarRangesM) > MaxPolarDetections:
            self.PolarAnglesRad = self.PolarAnglesRad[-MaxPolarDetections:]
            self.PolarRangesM = self.PolarRangesM[-MaxPolarDetections:]
            self.PolarAmplitudesDb = self.PolarAmplitudesDb[-MaxPolarDetections:]

        if self.PolarFigure is None:
            self._CreatePolarFigure()

        if len(self.PolarRangesM) > 0:
            Offsets = np.column_stack((self.PolarAnglesRad, self.PolarRangesM))
            self.PolarScatter.set_offsets(Offsets)

            Amplitudes = np.array(self.PolarAmplitudesDb)
            if len(Amplitudes) > 0:
                Sizes = 22.0 + 4.0 * np.clip(Amplitudes - np.nanmin(Amplitudes), 0.0, 20.0)
                self.PolarScatter.set_sizes(Sizes)

        self._UpdatePolarBeam(BoresightDeg)

        TacticalGreen = self.Config.get("TacticalGreenColour", "#00d060")
        self.PolarAxes.set_title(
            f"Accumulated Scan Detections - Dwell {Processed.DwellId}, Az {BoresightDeg:.1f} deg",
            color=TacticalGreen,
        )

        self.PolarFigure.canvas.draw_idle()

    def _CreatePolarFigure(self):
        """
        Create the tactical-style polar display once.
        """

        self.PolarFigure = plt.figure(figsize=(7, 7), facecolor="black")
        self.PolarAxes = self.PolarFigure.add_subplot(111, projection="polar")
        self.PolarAxes.set_facecolor("black")

        self.PolarAxes.set_theta_zero_location("E")
        self.PolarAxes.set_theta_direction(1)
        self.PolarAxes.set_thetamin(float(self.Config.get("ScanStartDeg", 0.0)))
        self.PolarAxes.set_thetamax(float(self.Config.get("ScanStopDeg", 90.0)))
        self.PolarAxes.set_rlim(0, float(self.Config.get("PolarMaxRangeM", 15000.0)))

        TacticalGreen = self.Config.get("TacticalGreenColour", "#00d060")
        TacticalGrid = self.Config.get("TacticalGridColour", "#176b3a")
        self.PolarAxes.grid(True, color=TacticalGrid, alpha=0.55, linewidth=0.8)
        self.PolarAxes.tick_params(colors=TacticalGreen)
        self.PolarAxes.spines["polar"].set_color(TacticalGreen)

        self.PolarScatter = self.PolarAxes.scatter(
            [],
            [],
            s=28,
            c="white",
            edgecolors="cyan",
            linewidths=0.9,
            alpha=0.95,
            zorder=6,
        )

        self.PolarFigure.tight_layout()

    def _UpdatePolarBeam(self, BoresightDeg):
        """
        Draw a visible fuzzy scan beam on the polar display.
        """

        if self.PolarAxes is None:
            return

        MaxRangeM = float(self.Config.get("PolarMaxRangeM", 15000.0))
        BeamwidthDeg = float(self.Config.get("BeamwidthDeg", self.Config.get("AntennaBeamwidthDeg", 5.0)))
        BoresightRad = np.deg2rad(BoresightDeg)

        # Remove the previous fuzzy beam lines.
        for Line in self.PolarBeamLines:
            try:
                Line.remove()
            except ValueError:
                pass
        self.PolarBeamLines = []

        # Draw several semi-transparent lines to create a fuzzy beam.
        BeamOffsetsDeg = np.linspace(-BeamwidthDeg, BeamwidthDeg, 21)

        for OffsetDeg in BeamOffsetsDeg:
            OffsetFraction = abs(OffsetDeg) / max(BeamwidthDeg, 1e-6)
            Alpha = 0.50 * (1.0 - OffsetFraction) + 0.05
            LineWidth = 1.0 + 4.0 * (1.0 - OffsetFraction)
            Theta = np.deg2rad(BoresightDeg + OffsetDeg)

            (Line,) = self.PolarAxes.plot(
                [Theta, Theta],
                [0.0, MaxRangeM],
                color="cyan",
                alpha=Alpha,
                linewidth=LineWidth,
                zorder=3,
            )
            self.PolarBeamLines.append(Line)

        # Bright centre boresight line.
        if self.PolarBoresightLine is None:
            (self.PolarBoresightLine,) = self.PolarAxes.plot(
                [BoresightRad, BoresightRad],
                [0.0, MaxRangeM],
                color="white",
                linewidth=3.0,
                alpha=1.0,
                zorder=5,
            )
        else:
            self.PolarBoresightLine.set_data([BoresightRad, BoresightRad], [0.0, MaxRangeM])

        # Dot at the beam tip.
        if self.PolarBeamTip is None:
            (self.PolarBeamTip,) = self.PolarAxes.plot(
                [BoresightRad],
                [MaxRangeM],
                marker="o",
                color="cyan",
                markeredgecolor="white",
                markersize=8,
                zorder=7,
            )
        else:
            self.PolarBeamTip.set_data([BoresightRad], [MaxRangeM])
