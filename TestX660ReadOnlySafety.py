"""Hardware-free regression tests for the Stage 4A X6-60 harness."""

import unittest

from RunX660ReadOnlyHardwareTest import (
    ParseArguments,
    READ_SEQUENCE,
    ValidateArguments,
)
from X660CanProtocol import (
    BuildReadRequest,
    DecodeMultiTurnAngleDeg,
    DecodeSoftwareVersion,
    DecodeStatus1,
    DecodeStatus2,
    READ_MULTI_TURN_ANGLE,
    READ_ONLY_COMMANDS,
    READ_SOFTWARE_VERSION,
    READ_STATUS_1,
    READ_STATUS_2,
    ReplyArbitrationId,
    RequestArbitrationId,
    ValidateReply,
)


class TestX660ReadOnlySafety(unittest.TestCase):
    def test_node_one_uses_protocol_ids(self):
        self.assertEqual(RequestArbitrationId(1), 0x141)
        self.assertEqual(ReplyArbitrationId(1), 0x241)

    def test_only_documented_read_commands_are_permitted(self):
        self.assertEqual(
            READ_ONLY_COMMANDS,
            frozenset({0xB2, 0x92, 0x9A, 0x9C}),
        )
        self.assertEqual(set(READ_SEQUENCE), set(READ_ONLY_COMMANDS))
        for ProhibitedCommand in (0x80, 0x81, 0xA1, 0xA2, 0xA4, 0xB3):
            with self.subTest(command=ProhibitedCommand):
                with self.assertRaisesRegex(ValueError, "not permitted"):
                    BuildReadRequest(ProhibitedCommand)

    def test_read_requests_are_exact_eight_byte_frames(self):
        for Command in READ_SEQUENCE:
            with self.subTest(command=Command):
                self.assertEqual(
                    BuildReadRequest(Command),
                    bytes([Command, 0, 0, 0, 0, 0, 0, 0]),
                )

    def test_manual_examples_decode_exactly(self):
        self.assertEqual(
            DecodeSoftwareVersion(bytes.fromhex("B2 00 00 00 2E 89 34 01")),
            20220206,
        )
        self.assertEqual(
            DecodeMultiTurnAngleDeg(bytes.fromhex("92 00 00 00 A0 8C 00 00")),
            360.0,
        )
        Status1 = DecodeStatus1(bytes.fromhex("9A 32 00 01 E5 01 04 00"))
        self.assertEqual(Status1.TemperatureC, 50)
        self.assertTrue(Status1.BrakeReleased)
        self.assertEqual(Status1.BusVoltageV, 48.5)
        self.assertEqual(Status1.ErrorFlags, 0x0004)
        self.assertEqual(Status1.ErrorNames, ("low voltage",))
        Status2 = DecodeStatus2(bytes.fromhex("9C 32 64 00 F4 01 2D 00"))
        self.assertEqual(Status2.TemperatureC, 50)
        self.assertEqual(Status2.TorqueCurrentA, 1.0)
        self.assertEqual(Status2.SpeedDegPerSec, 500)
        self.assertEqual(Status2.ShaftAngleDeg, 45)

    def test_reply_id_command_frame_type_and_dlc_are_enforced(self):
        Data = bytes.fromhex("92 00 00 00 A0 8C 00 00")
        self.assertEqual(ValidateReply(1, 0x92, 0x241, Data), Data)
        Cases = (
            (0x242, Data, False, "reply ID"),
            (0x241, bytes.fromhex("94 00 00 00 A0 8C 00 00"), False, "command"),
            (0x241, Data[:-1], False, "DLC"),
            (0x241, Data, True, "standard"),
        )
        for ArbitrationId, ReplyData, Extended, Pattern in Cases:
            with self.subTest(pattern=Pattern):
                with self.assertRaisesRegex(ValueError, Pattern):
                    ValidateReply(
                        1, 0x92, ArbitrationId, ReplyData, Extended,
                    )

    def test_hardware_access_defaults_off(self):
        Arguments = ParseArguments([])
        ValidateArguments(Arguments)
        self.assertFalse(Arguments.execute_read_only)

    def test_hardware_access_requires_both_acknowledgements(self):
        Base = ["--execute-read-only"]
        for Extra, Pattern in (
            ([], "secured-and-area-clear"),
            (["--i-confirm-x6-60-is-secured-and-area-clear"],
             "no-motion-or-configuration"),
        ):
            with self.subTest(extra=Extra):
                with self.assertRaisesRegex(ValueError, Pattern):
                    ValidateArguments(ParseArguments(Base + Extra))
        ValidateArguments(ParseArguments(Base + [
            "--i-confirm-x6-60-is-secured-and-area-clear",
            "--i-confirm-no-motion-or-configuration-commands",
        ]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
