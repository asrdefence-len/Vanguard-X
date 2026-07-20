"""Hardware-free tests for Vanguard X command-line operating profiles."""

import unittest

from EttusOperatingProfiles import (
    ApplyOperatingProfile,
    ParseOperatingProfileArguments,
)


class TestEttusOperatingProfiles(unittest.TestCase):
    @staticmethod
    def _required_arguments(profile, *gain_arguments):
        arguments = [
            profile,
            "--attenuation-db", "30",
            *gain_arguments,
            "--i-understand-rf-output-is-enabled",
            "--i-confirm-txrx-to-rx2-loopback",
        ]
        if profile in (
            "--stage3h-atr-loopback",
            "--stage3i-atr-rf-target-overlap",
        ):
            arguments.extend([
                "--i-confirm-trm-pa-disconnected",
                "--i-confirm-atr-cro-verified",
            ])
        else:
            arguments.append("--i-confirm-atr-trm-pa-disabled")
        return arguments

    @staticmethod
    def _base_config():
        return {
            "EttusOperatingMode": "RECEIVE_ONLY",
            "EttusTimedTransmitEnabled": False,
            "EttusAtrGpioEnabled": False,
            "EttusRxFrequencyHz": 1.0e9,
            "EttusRxChannel": 0,
        }

    def test_no_profile_remains_receive_only(self):
        arguments = ParseOperatingProfileArguments([])
        config = self._base_config()

        profile = ApplyOperatingProfile(config, arguments)

        self.assertEqual(profile, "RECEIVE_ONLY")
        self.assertFalse(config["EttusTimedTransmitEnabled"])
        self.assertFalse(config["EttusAtrGpioEnabled"])

    def test_stage3i_uses_verified_target_gain_defaults(self):
        arguments = ParseOperatingProfileArguments(
            self._required_arguments("--stage3i-atr-rf-target-overlap")
        )
        config = self._base_config()

        profile = ApplyOperatingProfile(config, arguments)

        self.assertEqual(profile, "STAGE3I_ATR_RF_TARGET_OVERLAP")
        self.assertEqual(config["EttusTxGainDb"], 50.0)
        self.assertEqual(config["EttusRxGainDb"], 30.0)

    def test_other_loopback_profiles_keep_conservative_defaults(self):
        for option, expected_profile in (
            ("--stage3e1-loopback", "STAGE3E1_LOOPBACK"),
            ("--stage3f-rf-target", "STAGE3F_RF_TARGET"),
            ("--stage3h-atr-loopback", "STAGE3H_ATR_LOOPBACK"),
        ):
            with self.subTest(profile=expected_profile):
                arguments = ParseOperatingProfileArguments(
                    self._required_arguments(option)
                )
                config = self._base_config()

                profile = ApplyOperatingProfile(config, arguments)

                self.assertEqual(profile, expected_profile)
                self.assertEqual(config["EttusTxGainDb"], 0.0)
                self.assertEqual(config["EttusRxGainDb"], 10.0)

    def test_explicit_stage3i_gains_override_defaults(self):
        arguments = ParseOperatingProfileArguments(
            self._required_arguments(
                "--stage3i-atr-rf-target-overlap",
                "--tx-gain-db", "12",
                "--rx-gain-db", "22",
            )
        )
        config = self._base_config()

        ApplyOperatingProfile(config, arguments)

        self.assertEqual(config["EttusTxGainDb"], 12.0)
        self.assertEqual(config["EttusRxGainDb"], 22.0)


if __name__ == "__main__":
    unittest.main()
