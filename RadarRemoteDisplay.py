"""Headless display adapter used by the Vanguard X radar process.

It implements the same small interface as RadarDisplayQt5 while keeping socket
I/O on a background thread.  Display-product preparation and JSON encoding are
completed by the radar thread immediately after a dwell, before the next timed
acquisition can begin.  The socket thread therefore never touches the large
processed radar arrays and cannot hold the Python GIL while UHD is arming a
timed TX/RX sequence.
"""

from __future__ import annotations

from collections import deque
import select
import socket
import threading
import time
from typing import Any, Dict, Iterable, Optional

import numpy as np

from RadarLinkProtocol import (
    EncodeMessage,
    FilterPublicConfig,
    FrameDecoder,
    MakeMessage,
    ProtocolError,
)


CONTROL_KEYS = {
    "DisplayMode",
    "ScanEnabled",
    "BeamAngleDeg",
    "ScanStartDeg",
    "ScanStopDeg",
    "ScanStepDeg",
    "ScanRateDegPerSec",
    "TransmitAvailable",
    "TransmitEnabled",
    "ExitRequested",
    "DataLogFilename",
    "SaveDataEnabled",
    "ManualNudgeCommandId",
    "ManualNudgeDeltaDeg",
    "StopCommandId",
    "SelectedWaveformId",
    "SelectedPrfHz",
    "SelectedPulsesPerCpi",
    "SelectedMaximumRangeM",
    "TimingSelectionRevision",
    "SystemMode",
    "RequestedSystemMode",
    "SystemModeRevision",
    "MissionCommandRevision",
    "MissionCommand",
    "MissionProfile",
}


PUBLIC_CONFIG_KEYS = {
    "RfTransmitAvailable", "InitialTransmitEnabled", "InitialDisplayMode",
    "InitialScanEnabled", "SystemMode", "InitialBeamAngleDeg",
    "ManualBeamStepDeg", "ScanStartDeg", "ScanStopDeg", "ScanStepDeg",
    "MinScanRateDegPerSec", "X660ScanSlewRateDegPerSec",
    "X660OperationalMaxRateDegPerSec",
    "AvailableWaveformIds", "SearchWaveformId", "SelectedPrfHz",
    "SelectedPulsesPerCpi", "InstrumentedMaxRangeM", "MinPrfHz",
    "MaxPrfHz", "MinPulsesPerCpi", "MaxPulsesPerCpi",
    "MinSelectableRangeM", "MaxSelectableRangeM", "MaxDisplayRangeM",
    "PolarMaxRangeM", "RangeRingStepM", "DataLogFilename",
    "DataLoggingEnabled", "PolarUpdateEveryNDwells",
    "PolarDetectionsUpdateEveryNDwells", "RangeProfileUpdateEveryNDwells",
    "StatusUpdateEveryNDwells", "MaxPolarDetections",
    "MaxDetectionsPlottedPerDwell", "QtRangeProfileDecimation",
    "QtMaxRangeProfilePoints", "RangeProfileDopplerMode",
    "RangeProfileMinDb", "RangeProfileMaxDb", "RangeProfileAutoScale",
    "RangeProfileAutoMinDb", "RangeProfileAutoMaxDb",
    "RangeProfileNoiseMarginDb", "RangeProfileMinimumSpanAboveNoiseDb",
    "RangeProfilePeakHeadroomDb", "RangeProfileNoisePercentile",
    "RangeProfileNoiseAlpha", "RangeProfileDetectionExclusionBins",
    "ShowRawDetections", "ShowRangeDetectionMarkers", "ShowTrackerPlots",
    "ShowTracks", "TrackClickGateM", "MapEnabled", "MapLatitudeDeg",
    "MapLongitudeDeg", "MapDatasetPath", "MapLandColour",
    "MapCoastColour", "MapLabelColour", "LogoPath", "LogoWidthPx",
    "RadarDwellIntervalSec", "MissionUsableBeamwidthDeg",
    "AntennaMaximumRotationRpm", "AntennaMaximumScanRateDegSec",
}


def _InitialControlState(config: Dict[str, Any]) -> Dict[str, Any]:
    transmit_available = bool(config.get("RfTransmitAvailable", False))
    return {
        "DisplayMode": "STOP",
        "ScanEnabled": False,
        "BeamAngleDeg": float(config.get("InitialBeamAngleDeg", 0.0)),
        "ScanStartDeg": float(config.get("ScanStartDeg", 0.0)),
        "ScanStopDeg": float(config.get("ScanStopDeg", 0.0)),
        "ScanStepDeg": float(config.get("ScanStepDeg", 1.0)),
        "ScanRateDegPerSec": float(
            config.get("X660ScanSlewRateDegPerSec", 20.0)
        ),
        "TransmitAvailable": transmit_available,
        "TransmitEnabled": False,
        "ExitRequested": False,
        "DataLogFilename": str(config.get("DataLogFilename", "datafile1.h5")),
        "SaveDataEnabled": bool(config.get("DataLoggingEnabled", False)),
        "ManualNudgeCommandId": 0,
        "ManualNudgeDeltaDeg": 0.0,
        "StopCommandId": 0,
        "SelectedWaveformId": str(config.get("SearchWaveformId", "Frank10_20MHz")),
        "SelectedPrfHz": float(config.get("SelectedPrfHz", 2000.0)),
        "SelectedPulsesPerCpi": int(config.get("SelectedPulsesPerCpi", 32)),
        "SelectedMaximumRangeM": float(config.get("InstrumentedMaxRangeM", 15000.0)),
        "TimingSelectionRevision": 0,
        "SystemMode": str(config.get("SystemMode", "SIM")).upper(),
        "RequestedSystemMode": str(config.get("SystemMode", "SIM")).upper(),
        "SystemModeRevision": 0,
        "MissionCommandRevision": 0,
        "MissionCommand": "",
        "MissionProfile": None,
    }


def _ObjectFields(item: Any, field_names: Iterable[str]) -> Dict[str, Any]:
    result = {}
    for name in field_names:
        if hasattr(item, name):
            value = getattr(item, name)
            if value is not None:
                result[name] = value
    return result


def _DisplayProfile(processed: Any, mode: str, maximum_points: int):
    magnitude = np.asarray(getattr(processed, "MagnitudeDb"), dtype=float)
    ranges = np.asarray(getattr(processed, "RangeAxisM"), dtype=float)
    if magnitude.ndim == 1:
        profile = magnitude
    else:
        doppler = np.asarray(getattr(processed, "DopplerAxisHz", []), dtype=float)
        # Support either [doppler, range] or [range, doppler] products. The
        # current display helper hides this orientation, so the network layer
        # derives it from the published axes instead of assuming one layout.
        if magnitude.shape[-1] == ranges.size:
            range_dimension = 1
            doppler_dimension = 0
        elif magnitude.shape[0] == ranges.size:
            range_dimension = 0
            doppler_dimension = 1
        else:
            raise ValueError("MagnitudeDb does not contain the RangeAxisM dimension")

        if str(mode).upper() == "ZERO_DOPPLER":
            doppler_count = magnitude.shape[doppler_dimension]
            index = int(np.argmin(np.abs(doppler))) if doppler.size == doppler_count else doppler_count // 2
            profile = magnitude[index, :] if doppler_dimension == 0 else magnitude[:, index]
        else:
            profile = np.nanmax(magnitude, axis=doppler_dimension)

    count = min(ranges.size, profile.size)
    ranges = ranges[:count]
    profile = profile[:count]
    step = max(1, int(np.ceil(count / float(max(1, maximum_points)))))
    if step == 1:
        return ranges, profile

    # Vectorised peak-preserving reduction.  The former Python loop called
    # nanargmax more than a thousand times for a normal 4043-bin profile and
    # could consume tens of milliseconds on the radar MiniPC.
    block_count = int(np.ceil(count / float(step)))
    padded_count = block_count * step
    finite = np.isfinite(profile)
    safe_profile = np.full(padded_count, -np.inf, dtype=float)
    safe_profile[:count] = np.where(finite, profile, -np.inf)
    blocks = safe_profile.reshape(block_count, step)
    offsets = np.argmax(blocks, axis=1)
    indices = np.arange(block_count, dtype=int) * step + offsets
    valid_blocks = np.any(blocks != -np.inf, axis=1)
    indices = indices[valid_blocks]
    return ranges[indices], profile[indices]


def BuildDisplaySnapshot(config, processed, detections, tracks, plots):
    """Create a bounded display-only snapshot; raw IQ is intentionally absent."""

    processed_fields = {
        "Diagnostics": dict(getattr(processed, "Diagnostics", {}) or {}),
    }
    try:
        ranges, profile = _DisplayProfile(
            processed,
            config.get("RangeProfileDopplerMode", "MAX"),
            int(config.get("RadarLinkMaxRangeProfilePoints", 1500)),
        )
        processed_fields["RangeAxisM"] = ranges
        processed_fields["DisplayRangeProfileDb"] = profile
    except Exception:
        pass

    detection_fields = (
        "RangeM", "AzimuthDeg", "AmplitudeDb", "DopplerHz", "VelocityMps",
    )
    track_fields = (
        "TrackId", "Status", "IsConfirmed", "TrackType", "RangeM",
        "AzimuthDeg", "RangeRateMps", "RangeRate", "VelocityMps",
        "AzimuthRateDps", "AngleRateDps", "BearingRateDps", "Hits",
        "HitCount", "Misses", "MissedCount", "Age", "ScanAge",
    )
    plot_fields = ("RangeM", "AzimuthDeg", "AmplitudeDb", "DopplerHz")
    return {
        "timestamp_unix_sec": time.time(),
        "processed": processed_fields,
        "detections": [_ObjectFields(item, detection_fields) for item in (detections or [])],
        "tracks": [_ObjectFields(item, track_fields) for item in (tracks or [])],
        "plots": [_ObjectFields(item, plot_fields) for item in (plots or [])],
    }


class RadarRemoteDisplay:
    """Radar-side adapter compatible with the current Main display calls."""

    def __init__(self, config: Dict[str, Any]):
        self.Config = config
        self.Host = str(config.get("RadarLinkHost", "127.0.0.1"))
        self.Port = int(config.get("RadarLinkPort", 5810))
        self.HeartbeatTimeoutSec = float(config.get("RadarLinkHeartbeatTimeoutSec", 2.0))
        self.BeamAngleDeg = float(config.get("InitialBeamAngleDeg", 0.0))
        self._control_lock = threading.Lock()
        self._control_state = _InitialControlState(config)
        self._snapshot_lock = threading.Lock()
        self._pending_snapshot_frame = None
        self._events_lock = threading.Lock()
        self._events = deque(maxlen=64)
        self._sequence_lock = threading.Lock()
        self._sequence = 0
        self._stop_event = threading.Event()
        # Cleared only around the time-critical Ettus ExecuteTaskStep call.
        # While it is clear the socket thread waits without decoding commands,
        # encoding display products, or otherwise competing for the GIL.
        self._radar_idle = threading.Event()
        self._radar_idle.set()
        self._radar_timing_lock = threading.Lock()
        self._connected = threading.Event()
        self._ready = threading.Event()
        self._startup_error = None
        self._last_client_message_monotonic = 0.0
        self.BoundPort = self.Port
        self._thread = threading.Thread(target=self._Run, name="RadarUiServer", daemon=True)
        self._thread.start()
        if not self._ready.wait(2.0):
            raise RuntimeError("timed out starting radar UI server")
        if self._startup_error is not None:
            raise RuntimeError(f"could not start radar UI server: {self._startup_error}")

    def GetControlState(self):
        self._ApplyHeartbeatPolicy()
        with self._control_lock:
            return dict(self._control_state)

    def Update(self, processed, detections, Tracks=None, Plots=None):
        # Prepare the complete bounded wire frame at the known-safe end of the
        # dwell.  Deferring this work to the socket thread caused intermittent
        # UHD late-command errors when it overlapped the following 5 ms-ahead
        # timed TX queue.  Replacing one prepared frame still never waits for
        # the UI or for socket I/O.
        snapshot = BuildDisplaySnapshot(
            self.Config,
            processed,
            detections,
            Tracks,
            Plots,
        )
        frame = self._EncodeFrame("radar_snapshot", snapshot)
        with self._snapshot_lock:
            self._pending_snapshot_frame = frame

    def BeginRadarTimingCritical(self):
        """Pause radar-side link work while a timed SDR dwell is armed/run."""
        self._radar_idle.clear()
        # The lock closes the small race in which the link thread passed its
        # idle-event check just before the event was cleared.  Waiting here is
        # safe: the Ettus command time is chosen only after this method returns.
        self._radar_timing_lock.acquire()

    def EndRadarTimingCritical(self):
        """Release deferred link work after the timed SDR dwell completes."""
        self._radar_timing_lock.release()
        self._radar_idle.set()

    def SetMeasuredBeamAngle(self, azimuth_deg):
        self.BeamAngleDeg = float(azimuth_deg) % 360.0
        with self._control_lock:
            self._control_state["BeamAngleDeg"] = self.BeamAngleDeg
        self._QueueEvent("beam_angle", {"azimuth_deg": self.BeamAngleDeg})

    def SetSystemModeApplicationResult(self, Applied, Message):
        self._QueueEvent("system_mode_result", {"applied": bool(Applied), "message": str(Message)})

    def SetTimingApplicationResult(self, Applied, Message, Profile=None):
        payload = {"applied": bool(Applied), "message": str(Message), "profile": None}
        if Profile is not None:
            timing = Profile.Timing
            payload["profile"] = {
                "WaveformId": str(Profile.WaveformId),
                "Timing": {
                    "SelectedPrfHz": float(timing.SelectedPrfHz),
                    "PulsesPerCpi": int(timing.PulsesPerCpi),
                    "MaximumRangeM": float(timing.MaximumRangeM),
                    "CpiDurationSec": float(timing.CpiDurationSec),
                    "NumRxSamples": int(timing.NumRxSamples),
                    "PriSec": float(timing.PriSec),
                },
            }
        self._QueueEvent("timing_result", payload)

    def SetMissionRuntimeStatus(self, Status):
        self._QueueEvent("mission_status", dict(Status or {}))

    def Shutdown(self):
        self._stop_event.set()
        self._radar_idle.set()
        self._thread.join(timeout=2.0)

    def _QueueEvent(self, message_type: str, payload: Dict[str, Any]):
        frame = self._EncodeFrame(message_type, payload)
        with self._events_lock:
            self._events.append(frame)

    def _NextSequence(self):
        with self._sequence_lock:
            self._sequence += 1
            return self._sequence

    def _EncodeFrame(self, message_type: str, payload: Dict[str, Any]):
        return EncodeMessage(
            MakeMessage(message_type, payload, self._NextSequence())
        )

    def _NextFrame(self):
        with self._events_lock:
            if self._events:
                return self._events.popleft()
        with self._snapshot_lock:
            pending_frame = self._pending_snapshot_frame
            self._pending_snapshot_frame = None
        return pending_frame

    def _ApplyHeartbeatPolicy(self):
        if not self._connected.is_set():
            return
        if time.monotonic() - self._last_client_message_monotonic <= self.HeartbeatTimeoutSec:
            return
        self._FailStop("UI heartbeat timeout")

    def _FailStop(self, reason: str):
        was_connected = self._connected.is_set()
        self._connected.clear()
        with self._control_lock:
            state = self._control_state
            already_stopped = state.get("DisplayMode") == "STOP" and not state.get("TransmitEnabled", False)
            state["DisplayMode"] = "STOP"
            state["ScanEnabled"] = False
            state["TransmitEnabled"] = False
            state["ExitRequested"] = False
            if not already_stopped:
                state["StopCommandId"] = int(state.get("StopCommandId", 0)) + 1
            state["MissionCommandRevision"] = (
                int(state.get("MissionCommandRevision", 0)) + 1
            )
            state["MissionCommand"] = "STOP"
            state["MissionProfile"] = None
        if was_connected:
            print(f"Radar UI link fail-stop: {reason}")

    def _ApplyClientMessage(self, message):
        self._last_client_message_monotonic = time.monotonic()
        message_type = message["type"]
        payload = message.get("payload", {})
        if message_type == "heartbeat":
            return
        if message_type != "control_state":
            return
        filtered = {key: value for key, value in payload.items() if key in CONTROL_KEYS}
        # The backend owns actual TX capability; a client cannot create it.
        transmit_available = bool(self.Config.get("RfTransmitAvailable", False))
        filtered["TransmitAvailable"] = transmit_available
        filtered["TransmitEnabled"] = bool(filtered.get("TransmitEnabled", False)) and transmit_available
        with self._control_lock:
            self._control_state.update(filtered)
            self.BeamAngleDeg = float(self._control_state.get("BeamAngleDeg", self.BeamAngleDeg))

    def _Run(self):
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            listener.bind((self.Host, self.Port))
        except OSError as error:
            self._startup_error = error
            self._ready.set()
            listener.close()
            return
        listener.listen(1)
        listener.settimeout(0.25)
        self.BoundPort = int(listener.getsockname()[1])
        self._ready.set()
        print(f"Radar UI server listening on {self.Host}:{self.BoundPort}")
        try:
            while not self._stop_event.is_set():
                try:
                    client, address = listener.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                self._ServeClient(client, address)
        finally:
            listener.close()
            self._FailStop("server shutdown")

    def _ServeClient(self, client: socket.socket, address):
        client.setblocking(False)
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        decoder = FrameDecoder()
        self._connected.set()
        self._last_client_message_monotonic = time.monotonic()
        hello_payload = {
            "server": "Vanguard X radar",
            "display_config": FilterPublicConfig(self.Config, PUBLIC_CONFIG_KEYS),
            "fail_stop_timeout_sec": self.HeartbeatTimeoutSec,
        }
        outgoing = bytearray(self._EncodeFrame("server_hello", hello_payload))
        print(f"Radar UI connected from {address[0]}:{address[1]}")
        try:
            while not self._stop_event.is_set():
                # The main radar thread clears this event immediately before
                # ExecuteTaskStep and sets it when acquisition returns.  A
                # 16--25 ms wait is well inside the two-second heartbeat limit.
                if not self._radar_idle.wait(timeout=0.05):
                    continue
                with self._radar_timing_lock:
                    if not self._radar_idle.is_set():
                        continue
                    if not outgoing:
                        frame = self._NextFrame()
                        if frame is not None:
                            outgoing.extend(frame)
                readable, writable, exceptional = select.select(
                    [client], [client] if outgoing else [], [client], 0.05
                )
                if exceptional:
                    break
                if readable:
                    with self._radar_timing_lock:
                        data = client.recv(65536)
                        if not data:
                            break
                        for message in decoder.Feed(data):
                            self._ApplyClientMessage(message)
                if writable and outgoing:
                    with self._radar_timing_lock:
                        sent = client.send(outgoing)
                        del outgoing[:sent]
                with self._radar_timing_lock:
                    self._ApplyHeartbeatPolicy()
                if not self._connected.is_set():
                    break
        except (ConnectionError, OSError, ProtocolError, TypeError, ValueError) as error:
            print(f"Radar UI link closed: {error}")
        finally:
            try:
                client.close()
            except Exception:
                pass
            self._FailStop("UI disconnected")
