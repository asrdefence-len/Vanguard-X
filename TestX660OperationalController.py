#!/usr/bin/env python3
"""Software-only regressions for operational X6-60 controller integration."""

from __future__ import annotations

from collections import deque
from pathlib import Path
from types import ModuleType, SimpleNamespace
import struct
import sys
import unittest


try:
    import NavigationState  # noqa: F401
except ImportError:
    NavigationState = ModuleType("NavigationState")

    class PlatformAttitude:
        def __init__(self, HeadingTrueDeg=0.0, PitchDeg=0.0, Valid=True):
            self.HeadingTrueDeg = float(HeadingTrueDeg)
            self.PitchDeg = float(PitchDeg)
            self.Valid = bool(Valid)

    NavigationState.PlatformAttitude = PlatformAttitude
    sys.modules["NavigationState"] = NavigationState

import X660GuardedMotionTransport as GuardedTransport
import X660MotionProtocol as MotionProtocol
import X660PlannerProtocol as PlannerProtocol
from EttusOperatingProfiles import (
    ApplyOperatingProfile,
    ApplyX660OperatingProfile,
    ParseOperatingProfileArguments,
)
from NavigationState import PlatformAttitude
from PointingManager import PointingManager
from RadarTasks import AngleFrame, MakeSearchTask, SearchPattern
from X660Controller import CreateX660Controller
from X660OperationalController import X660OperationalController
from X660ReadOnlyController import X660ReadOnlyState


class FakeBus:
    def __init__(self):
        self.shutdown_count = 0
        self.planner_values = [1140, 1140, 1140, 1140]
        self.requests = []
        self.replies = deque()

    def send(self, request, timeout):
        data = bytes(request.data)
        self.requests.append(data)
        command = data[0]
        index = data[1]
        if command == PlannerProtocol.READ_PLANNER_PARAMETER:
            reply_data = (
                bytes([command, index, 0, 0])
                + int(self.planner_values[index]).to_bytes(4, "little")
            )
        elif command == PlannerProtocol.WRITE_PLANNER_PARAMETER:
            self.planner_values[index] = int.from_bytes(data[4:8], "little")
            reply_data = data
        else:
            raise AssertionError(f"unexpected fake-bus command 0x{command:02X}")
        self.replies.append(
            GuardedTransport.CanFrame(
                arbitration_id=0x241,
                data=reply_data,
                is_extended_id=False,
            )
        )

    def recv(self, timeout):
        return self.replies.popleft() if self.replies else None

    def shutdown(self):
        self.shutdown_count += 1


class FakeTelemetry:
    def __init__(
        self,
        RawAngleDeg=-361.91,
        AzimuthDeg=359.62,
        PanRateDegPerSec=0.0,
    ):
        self.Bus = FakeBus()
        self.CanModule = SimpleNamespace(Message=GuardedTransport.CanFrame)
        self.Calibrated = True
        self.IsOpen = False
        self.open_count = 0
        self.close_count = 0
        self.State = X660ReadOnlyState(
            AzimuthDeg=float(AzimuthDeg),
            ElevationDeg=0.0,
            Valid=True,
            PanRateDegPerSec=float(PanRateDegPerSec),
            Direction="stopped",
            AtTarget=False,
            Source="FAKE_X660_TELEMETRY",
            RawAngleDeg=float(RawAngleDeg),
            ErrorFlags=0,
        )

    def Open(self):
        self.IsOpen = True
        self.open_count += 1

    def Close(self):
        self.IsOpen = False
        self.close_count += 1

    def Update(self):
        return self.State

    def GetLastKnownState(self):
        return self.State


class RecordingTransport:
    def __init__(self):
        self.Payloads = []
        self.FailNextNonStop = False

    def Transact(self, Payload):
        Data = bytes(Payload)
        self.Payloads.append(Data)
        if self.FailNextNonStop and Data[0] != MotionProtocol.MOTOR_STOP_COMMAND:
            self.FailNextNonStop = False
            raise TimeoutError("injected motion timeout")
        return SimpleNamespace(Command=Data[0], ReplyData=Data)


class RecordingTransportFactory:
    def __init__(self, Transport):
        self.Transport = Transport
        self.Arguments = None

    def __call__(self, **Arguments):
        self.Arguments = Arguments
        return self.Transport


class RecordingPlannerInitialiserFactory:
    def __init__(self, Failure):
        self.Arguments = None
        self.Failure = Failure

    def __call__(self, **Arguments):
        self.Arguments = Arguments
        Failure = self.Failure

        class FailingPlannerInitialiser:
            def Initialise(self):
                raise Failure

        return FailingPlannerInitialiser()


class RecordingShutdown:
    def __init__(self):
        self.Calls = []

    def __call__(self, Bus, CanModule, NodeId, Payload, TimeoutSec):
        self.Calls.append((Bus, CanModule, NodeId, bytes(Payload), TimeoutSec))
        return bytes(Payload)


def MakeController(DirectionSign=+1, **Overrides):
    Telemetry = Overrides.pop("TelemetryController", FakeTelemetry())
    Transport = Overrides.pop("Transport", RecordingTransport())
    Shutdown = Overrides.pop("ShutdownTransaction", RecordingShutdown())
    Factory = RecordingTransportFactory(Transport)
    Controller = X660OperationalController(
        DirectionSign=DirectionSign,
        MotionEnabled=True,
        IUnderstandMotionWillOccur=True,
        IConfirmMotionAreaIsClear=True,
        TelemetryController=Telemetry,
        MotionTransportFactory=Factory,
        ShutdownTransaction=Shutdown,
        **Overrides,
    )
    return Controller, Telemetry, Transport, Factory, Shutdown


class TestX660OperationalController(unittest.TestCase):
    def test_live_session_requires_all_three_explicit_authorisations(self):
        Cases = (
            {},
            {"MotionEnabled": True},
            {
                "MotionEnabled": True,
                "IUnderstandMotionWillOccur": True,
            },
        )
        for Arguments in Cases:
            with self.subTest(Arguments=Arguments):
                Controller = X660OperationalController(
                    DirectionSign=+1,
                    TelemetryController=FakeTelemetry(),
                    **Arguments,
                )
                with self.assertRaises(RuntimeError):
                    Controller.Open()

    def test_open_joins_telemetry_bus_to_live_guarded_transport(self):
        Controller, Telemetry, _, Factory, _ = MakeController()
        Controller.Open()
        self.assertTrue(Controller.IsOpen)
        self.assertEqual(Telemetry.open_count, 1)
        self.assertIs(Factory.Arguments["Bus"], Telemetry.Bus)
        self.assertFalse(Factory.Arguments["DryRun"])
        self.assertTrue(Factory.Arguments["IUnderstandMotionWillOccur"])
        self.assertTrue(Factory.Arguments["IConfirmMotionAreaIsClear"])
        self.assertTrue(Controller.MotorReady)
        self.assertEqual(
            Controller.PlannerInitialisationResult.Values,
            PlannerProtocol.VANGUARD_X_REQUIRED_VALUES,
        )

    def test_open_fails_closed_when_planner_initialisation_fails(self):
        PlannerFactory = RecordingPlannerInitialiserFactory(
            TimeoutError("injected planner timeout")
        )
        Controller, Telemetry, Transport, Factory, _ = MakeController(
            PlannerInitialiserFactory=PlannerFactory,
        )
        with self.assertRaisesRegex(TimeoutError, "planner timeout"):
            Controller.Open()
        self.assertFalse(Controller.IsOpen)
        self.assertFalse(Controller.MotorReady)
        self.assertIsNone(Controller.Transport)
        self.assertIsNone(Factory.Arguments)
        self.assertEqual(Transport.Payloads, [])
        self.assertEqual(Telemetry.close_count, 1)

    def test_clockwise_positive_slew_maps_through_calibrated_direction(self):
        for DirectionSign, ExpectedRawRate in ((+1, +6.0), (-1, -6.0)):
            with self.subTest(DirectionSign=DirectionSign):
                Controller, _, Transport, _, _ = MakeController(DirectionSign)
                Controller.Open()
                Controller.CommandSlew(+6.0)
                Payload = Transport.Payloads[-1]
                self.assertEqual(Payload[0], MotionProtocol.SPEED_CONTROL_COMMAND)
                self.assertEqual(struct.unpack("<i", Payload[4:8])[0], int(ExpectedRawRate * 100))
                self.assertEqual(Controller.MotionMode, "slew")

    def test_operational_rate_limit_is_enforced_before_transport(self):
        Controller, _, Transport, _, _ = MakeController(
            MaxPanRateDegPerSec=14.0
        )
        Controller.Open()
        with self.assertRaisesRegex(ValueError, "14 deg/s"):
            Controller.CommandSlew(14.01)
        self.assertEqual(Transport.Payloads, [])

    def test_goto_uses_nearest_unlimited_multiturn_raw_target(self):
        Controller, _, Transport, _, _ = MakeController(+1)
        Controller.Open()
        Controller.SetPanPositionNative(10.0)
        Payload = Transport.Payloads[-1]
        self.assertEqual(Payload[0], MotionProtocol.ABSOLUTE_POSITION_COMMAND)
        self.assertEqual(int.from_bytes(Payload[2:4], "little"), 14)
        self.assertEqual(struct.unpack("<i", Payload[4:8])[0], -35153)
        self.assertAlmostEqual(Controller.TargetAzimuthDeg, 10.0)

    def test_nudge_and_stop_use_proven_incremental_and_hold_commands(self):
        Controller, _, Transport, _, _ = MakeController(+1)
        Controller.Open()
        Controller.NudgePanPositionNative(+1.0)
        Controller.Stop()
        self.assertEqual(
            Transport.Payloads[0],
            MotionProtocol.BuildIncrementalPositionRequest(+1.0, 14),
        )
        self.assertEqual(
            Transport.Payloads[1],
            MotionProtocol.BuildMotorStopRequest(),
        )
        self.assertEqual(Controller.MotionMode, "hold")

    def test_motion_failure_attempts_stop_and_latches_until_reopen(self):
        Controller, _, Transport, _, _ = MakeController()
        Controller.Open()
        Transport.FailNextNonStop = True
        with self.assertRaisesRegex(RuntimeError, "fault latched"):
            Controller.CommandSlew(4.0)
        self.assertTrue(Controller.MotionFaulted)
        self.assertEqual(
            Transport.Payloads[-1],
            MotionProtocol.BuildMotorStopRequest(),
        )
        with self.assertRaisesRegex(RuntimeError, "fault-latched"):
            Controller.CommandSlew(2.0)

    def test_close_always_stops_then_confirms_motor_output_shutdown(self):
        Controller, Telemetry, Transport, _, Shutdown = MakeController()
        Controller.Open()
        Controller.CommandSlew(3.0)
        Controller.Close()
        self.assertEqual(
            Transport.Payloads[-1],
            MotionProtocol.BuildMotorStopRequest(),
        )
        self.assertEqual(len(Shutdown.Calls), 1)
        self.assertEqual(Shutdown.Calls[0][3], bytes.fromhex("80 00 00 00 00 00 00 00"))
        self.assertEqual(Telemetry.close_count, 1)
        self.assertFalse(Controller.IsOpen)
        self.assertFalse(Controller.MotorReady)

    def test_reported_motor_error_stops_and_invalidates_state(self):
        Controller, Telemetry, Transport, _, _ = MakeController()
        Controller.Open()
        Telemetry.State.ErrorFlags = 0x0004
        with self.assertRaisesRegex(RuntimeError, "0x0004"):
            Controller.Update()
        self.assertEqual(
            Transport.Payloads[-1],
            MotionProtocol.BuildMotorStopRequest(),
        )
        self.assertTrue(Controller.MotionFaulted)
        self.assertFalse(Telemetry.State.Valid)

    def test_factory_exposes_operational_mode_without_changing_default(self):
        Default = CreateX660Controller({"X660DirectionSign": +1})
        self.assertFalse(Default.MotionCommandsEnabled)
        Operational = CreateX660Controller({
            "X660Mode": "x660-operational",
            "X660DirectionSign": +1,
            "X660MotionEnabled": True,
            "X660IUnderstandMotionWillOccur": True,
            "X660IConfirmMotionAreaIsClear": True,
        })
        self.assertIsInstance(Operational, X660OperationalController)
        self.assertTrue(Operational.MotionCommandsEnabled)
        self.assertFalse(Operational.SupportsElevation)

    def test_pointing_manager_drives_motion_and_reports_measured_beam(self):
        Controller, Telemetry, Transport, _, _ = MakeController()
        Controller.Open()
        Pointing = PointingManager(x660=Controller)
        Navigation = PlatformAttitude(
            HeadingTrueDeg=20.0,
            PitchDeg=0.0,
            Valid=True,
        )
        Search = MakeSearchTask(
            TaskId=1,
            SectorStartDeg=0.0,
            SectorStopDeg=0.0,
            ScanRateDegPerSec=6.0,
            SectorFrame=AngleFrame.PLATFORM,
            Pattern=SearchPattern.CONTINUOUS_CW,
        )
        Pointing.ActivateTask(Search, Navigation)
        self.assertEqual(
            Transport.Payloads[-1],
            MotionProtocol.BuildSpeedControlRequest(+6.0),
        )

        Telemetry.State.AzimuthDeg = 42.0
        Telemetry.State.PanRateDegPerSec = 6.0
        State = Pointing.Update(Navigation)
        self.assertAlmostEqual(State.AntennaAzimuthRelativeDeg, 42.0)
        self.assertAlmostEqual(State.BeamBearingTrueDeg, 62.0)
        self.assertTrue(State.Valid)

    def test_scheduler_source_selects_operational_mode_explicitly(self):
        Source = (Path(__file__).parent / "VanguardxMain_scheduler.py").read_text()
        self.assertIn('"X660Mode": "x660-sim"', Source)
        self.assertIn('"X660MotionEnabled": False', Source)
        self.assertIn('"X660OperationalMaxRateDegPerSec": 60.0', Source)
        self.assertIn(
            '"X660PlannerInternalAccelerationDegPerSec2": 1140',
            Source,
        )
        self.assertIn(
            "Vanguard X startup aborted: operational X6-60 planner",
            Source,
        )
        self.assertIn("ApplyX660OperatingProfile", Source)

    def test_command_line_profile_enables_one_operational_session(self):
        Arguments = ParseOperatingProfileArguments([
            "--x660-operational",
            "--i-understand-x660-motion-will-occur",
            "--i-confirm-x660-motion-area-is-clear",
        ])
        Config = {
            "EttusOperatingMode": "RECEIVE_ONLY",
            "EttusTimedTransmitEnabled": False,
        }
        self.assertEqual(
            ApplyOperatingProfile(Config, Arguments),
            "RECEIVE_ONLY",
        )
        self.assertEqual(
            ApplyX660OperatingProfile(Config, Arguments),
            "X660_OPERATIONAL",
        )
        self.assertEqual(Config["X660Mode"], "x660-operational")
        self.assertTrue(Config["X660MotionEnabled"])

    def test_x660_command_line_profile_is_fail_closed(self):
        for ArgumentsList in (
            ["--i-understand-x660-motion-will-occur"],
            ["--i-confirm-x660-motion-area-is-clear"],
            ["--x660-operational"],
            [
                "--x660-operational",
                "--i-understand-x660-motion-will-occur",
            ],
        ):
            with self.subTest(ArgumentsList=ArgumentsList):
                Arguments = ParseOperatingProfileArguments(ArgumentsList)
                with self.assertRaises(ValueError):
                    ApplyX660OperatingProfile({}, Arguments)

    def test_no_live_option_preserves_software_simulator(self):
        Arguments = ParseOperatingProfileArguments([])
        Config = {"X660Mode": "x660-sim"}
        self.assertEqual(
            ApplyX660OperatingProfile(Config, Arguments),
            "X660_SIMULATED",
        )
        self.assertEqual(Config["X660Mode"], "x660-sim")
        self.assertFalse(Config["X660MotionEnabled"])


if __name__ == "__main__":
    unittest.main()
