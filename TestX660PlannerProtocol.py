#!/usr/bin/env python3
"""Regression tests for X6-60 planner configuration payloads."""

import unittest

import X660PlannerProtocol as Protocol


class TestX660PlannerProtocol(unittest.TestCase):
    def test_vanguard_x_value_encodes_as_verified_can_frames(self):
        self.assertEqual(
            Protocol.BuildPlannerReadRequest(0x02),
            bytes.fromhex("42 02 00 00 00 00 00 00"),
        )
        self.assertEqual(
            Protocol.BuildPlannerWriteRequest(0x02, 1140),
            bytes.fromhex("43 02 00 00 74 04 00 00"),
        )

    def test_all_four_required_values_are_1140(self):
        self.assertEqual(
            Protocol.VANGUARD_X_REQUIRED_VALUES,
            (1140, 1140, 1140, 1140),
        )

    def test_read_reply_decodes_little_endian_value(self):
        self.assertEqual(
            Protocol.DecodePlannerReadReply(
                0x03,
                bytes.fromhex("42 03 00 00 74 04 00 00"),
            ),
            1140,
        )

    def test_write_reply_must_echo_exact_index_and_value(self):
        with self.assertRaisesRegex(ValueError, "echo mismatch"):
            Protocol.ValidatePlannerWriteReply(
                0x01,
                1140,
                bytes.fromhex("43 01 00 00 64 00 00 00"),
            )

    def test_only_documented_indices_and_values_are_accepted(self):
        for Index in (-1, 4, True):
            with self.subTest(Index=Index):
                with self.assertRaises(ValueError):
                    Protocol.BuildPlannerReadRequest(Index)
        for Value in (99, 60001, 1140.5, True):
            with self.subTest(Value=Value):
                with self.assertRaises(ValueError):
                    Protocol.BuildPlannerWriteRequest(0, Value)


if __name__ == "__main__":
    unittest.main()
