#!/usr/bin/env python3
"""Single-threaded 50 Hz control clock for Vanguard X X6-60 pointing.

The radar dwell cadence and the motor endpoint-control cadence are deliberately
independent.  ``TickIfDue`` is called from the existing scheduler loop, so all
SocketCAN access remains serial.  Missed deadlines are skipped rather than
replayed, preventing a delayed radar dwell from causing a burst of CAN queries.
"""

from __future__ import annotations

import math
import time


class X660PointingControlLoop:
    """Run PointingManager updates at a bounded monotonic-clock cadence."""

    def __init__(
        self,
        interval_sec: float = 0.020,
        debug: bool = False,
        debug_report_every_ticks: int = 250,
    ):
        self.IntervalSec = float(interval_sec)
        if not math.isfinite(self.IntervalSec) or self.IntervalSec <= 0.0:
            raise ValueError("interval_sec must be greater than zero")

        self.Debug = bool(debug)
        self.DebugReportEveryTicks = max(1, int(debug_report_every_ticks))

        self.NextDueMonotonicSec = None
        self.LastTickMonotonicSec = None
        self.LastActualIntervalSec = None
        self.TickCount = 0
        self.MissedTickCount = 0
        self.MaximumAbsoluteJitterSec = 0.0
        self.TotalActualIntervalSec = 0.0
        self.ActualIntervalCount = 0

    def TickIfDue(
        self,
        pointing,
        navigation,
        x660=None,
        display=None,
        config=None,
        now_monotonic_sec=None,
    ):
        """Update pointing once when the 20 ms control deadline is due.

        Returns the new PointingState when an update ran, otherwise ``None``.
        The next deadline is based on completion time rather than replaying
        missed periods; this keeps CAN traffic bounded after a slow dwell.
        """

        now = (
            time.monotonic()
            if now_monotonic_sec is None
            else float(now_monotonic_sec)
        )
        if not math.isfinite(now):
            raise ValueError("now_monotonic_sec must be finite")

        if self.NextDueMonotonicSec is None:
            self.NextDueMonotonicSec = now

        if now < self.NextDueMonotonicSec:
            return None

        lateness_sec = max(0.0, now - self.NextDueMonotonicSec)
        self.MissedTickCount += int(lateness_sec // self.IntervalSec)

        if self.LastTickMonotonicSec is not None:
            actual_interval_sec = now - self.LastTickMonotonicSec
            self.LastActualIntervalSec = actual_interval_sec
            self.TotalActualIntervalSec += actual_interval_sec
            self.ActualIntervalCount += 1
            self.MaximumAbsoluteJitterSec = max(
                self.MaximumAbsoluteJitterSec,
                abs(actual_interval_sec - self.IntervalSec),
            )

        # Advance before the CAN transaction so an exception cannot create a
        # tight retry loop.  Delayed periods are skipped, never replayed.
        self.LastTickMonotonicSec = now
        self.NextDueMonotonicSec = now + self.IntervalSec
        self.TickCount += 1

        pointing_state = pointing.Update(navigation)
        measured_state = self._GetMeasuredState(pointing, x660)

        encoder_azimuth_deg = float(
            getattr(
                measured_state,
                "AzimuthDeg",
                pointing_state.AntennaAzimuthRelativeDeg,
            )
        ) % 360.0
        beam_bearing_true_deg = float(
            getattr(
                pointing_state,
                "BeamBearingTrueDeg",
                encoder_azimuth_deg,
            )
        ) % 360.0

        if display is not None:
            if hasattr(display, "SetMeasuredBeamAngle"):
                display.SetMeasuredBeamAngle(beam_bearing_true_deg)
            elif hasattr(display, "BeamAngleDeg"):
                display.BeamAngleDeg = beam_bearing_true_deg

        if config is not None:
            config["BoresightDeg"] = beam_bearing_true_deg
            config["X660BeamBearingTrueDeg"] = beam_bearing_true_deg
            config["X660AzDeg"] = encoder_azimuth_deg
            config["X660RateDegPerSec"] = float(
                getattr(
                    measured_state,
                    "PanRateDegPerSec",
                    pointing_state.PanRateDegPerSec,
                )
            )
            config["X660Valid"] = bool(
                getattr(measured_state, "Valid", pointing_state.Valid)
            )
            config["X660Source"] = str(
                getattr(measured_state, "Source", pointing_state.Source)
            )
            config["X660AtTarget"] = bool(
                getattr(measured_state, "AtTarget", pointing_state.Ready)
            )
            config["X660RawAngleDeg"] = getattr(
                measured_state,
                "RawAngleDeg",
                None,
            )
            config["X660PointingControlActualIntervalSec"] = (
                self.LastActualIntervalSec
            )
            config["X660PointingControlMissedTickCount"] = (
                self.MissedTickCount
            )

        if (
            self.Debug
            and self.TickCount % self.DebugReportEveryTicks == 0
        ):
            diagnostics = self.GetTimingDiagnostics()
            print(
                "X6-60 control clock: "
                f"last={1000.0 * diagnostics['LastIntervalSec']:.2f} ms, "
                f"mean={1000.0 * diagnostics['MeanIntervalSec']:.2f} ms, "
                f"max_jitter="
                f"{1000.0 * diagnostics['MaximumAbsoluteJitterSec']:.2f} ms, "
                f"missed={diagnostics['MissedTickCount']}"
            )

        return pointing_state

    def GetTimingDiagnostics(self):
        mean_interval_sec = (
            self.TotalActualIntervalSec / self.ActualIntervalCount
            if self.ActualIntervalCount > 0
            else self.IntervalSec
        )
        return {
            "TargetIntervalSec": self.IntervalSec,
            "LastIntervalSec": (
                self.LastActualIntervalSec
                if self.LastActualIntervalSec is not None
                else self.IntervalSec
            ),
            "MeanIntervalSec": mean_interval_sec,
            "MaximumAbsoluteJitterSec": self.MaximumAbsoluteJitterSec,
            "TickCount": self.TickCount,
            "MissedTickCount": self.MissedTickCount,
        }

    @staticmethod
    def _GetMeasuredState(pointing, x660):
        positioner = x660 if x660 is not None else getattr(
            pointing,
            "Positioner",
            None,
        )
        if positioner is None:
            return None
        if hasattr(positioner, "GetState"):
            return positioner.GetState()
        if hasattr(positioner, "GetLastKnownState"):
            return positioner.GetLastKnownState()
        return getattr(positioner, "State", None)
