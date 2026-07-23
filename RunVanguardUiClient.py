"""Run the existing Vanguard X Qt5 display as a TCP client."""

from __future__ import annotations

import argparse
import os
import sys
import time
from types import SimpleNamespace

import numpy as np

from RadarLinkClient import RadarLinkClient


def _Namespace(data):
    return SimpleNamespace(**dict(data or {}))


def _Processed(data):
    values = dict(data or {})
    for name in ("RangeAxisM", "DisplayRangeProfileDb"):
        if name in values:
            values[name] = np.asarray(values[name], dtype=float)
    return SimpleNamespace(**values)


def ApplyServerMessage(display, message):
    message_type = message["type"]
    payload = message.get("payload", {})
    if message_type == "server_hello":
        config = payload.get("display_config", {})
        system_mode = str(config.get("SystemMode", display.SystemMode)).upper()
        display.SystemMode = system_mode
        display.RequestedSystemMode = system_mode
        display.TransmitAvailable = bool(config.get("RfTransmitAvailable", False))
        display.TransmitEnabled = False
        system_widget = display.ControlWidgets.get("SystemMode")
        if system_widget is not None:
            system_widget.blockSignals(True)
            system_widget.setCurrentText(system_mode)
            system_widget.blockSignals(False)
        display.SetRadarLinkStatus(True, "CONNECTED")
    elif message_type == "radar_snapshot":
        display.Update(
            _Processed(payload.get("processed")),
            [_Namespace(item) for item in payload.get("detections", [])],
            Tracks=[_Namespace(item) for item in payload.get("tracks", [])],
            Plots=[_Namespace(item) for item in payload.get("plots", [])],
        )
    elif message_type == "beam_angle":
        display.SetMeasuredBeamAngle(float(payload.get("azimuth_deg", 0.0)))
    elif message_type == "system_mode_result":
        display.SetSystemModeApplicationResult(
            bool(payload.get("applied", False)), str(payload.get("message", ""))
        )
    elif message_type == "timing_result":
        profile_data = payload.get("profile")
        profile = None
        if profile_data:
            timing = _Namespace(profile_data.get("Timing"))
            profile = SimpleNamespace(WaveformId=profile_data.get("WaveformId", ""), Timing=timing)
        display.SetTimingApplicationResult(
            bool(payload.get("applied", False)),
            str(payload.get("message", "")),
            Profile=profile,
        )


def Main(arguments=None):
    parser = argparse.ArgumentParser(description="Vanguard X remote Qt operator UI")
    parser.add_argument("--radar-host", default="127.0.0.1")
    parser.add_argument("--radar-port", type=int, default=5810)
    parser.add_argument(
        "--ui-nice",
        type=int,
        default=10,
        help=(
            "Linux scheduling priority increment for the UI process "
            "(default: 10; use 0 to disable)"
        ),
    )
    options = parser.parse_args(arguments)

    if options.ui_nice < 0 or options.ui_nice > 19:
        parser.error("--ui-nice must be between 0 and 19")
    if options.ui_nice and hasattr(os, "nice"):
        try:
            applied_nice = os.nice(options.ui_nice)
            print(f"UI process scheduling priority: nice {applied_nice}")
        except OSError as error:
            print(f"Warning: could not lower UI process priority: {error}")

    client = RadarLinkClient(options.radar_host, options.radar_port)
    client.Start()
    try:
        config = client.WaitForDisplayConfig(timeout_sec=5.0)
    except Exception as error:
        client.Close()
        raise SystemExit(f"Could not connect to Vanguard X radar server: {error}")

    # Import Qt only in the UI process.  The radar server can therefore run
    # without importing or executing the graphical stack.
    from RadarDisplayQt5 import RadarDisplay
    from PyQt5 import QtCore

    config["QtProcessEventsEveryNDwells"] = 0
    config["RadarLinkStatus"] = "CONNECTED"
    display = RadarDisplay(config)

    last_link_state = [True]

    def PollLink():
        for message in client.DrainMessages():
            ApplyServerMessage(display, message)
        client.SendControlState(display.GetControlState())
        connected = bool(client.Connected)
        if connected != last_link_state[0]:
            display.SetRadarLinkStatus(
                connected,
                "CONNECTED" if connected else "LOST - RADAR FAIL-STOP",
            )
            last_link_state[0] = connected
        if not connected:
            display.TransmitEnabled = False
            display.ScanEnabled = False
            display.DisplayMode = "STOP"
            display.UpdateStatusPanel()

    timer = QtCore.QTimer()
    timer.timeout.connect(PollLink)
    timer.start(50)
    try:
        exec_method = getattr(display.App, "exec", None) or display.App.exec_
        return int(exec_method())
    finally:
        # Make an explicit final STOP best-effort; a lost connection also
        # activates the radar-side heartbeat fail-stop.
        display.OnStop()
        client.SendControlState(display.GetControlState())
        time.sleep(0.10)
        client.Close()


if __name__ == "__main__":
    sys.exit(Main())
