"""Hardware-free tests for guarded full-application Ettus profiles."""

from pathlib import Path
import unittest

from EttusOperatingProfiles import (
    ApplyOperatingProfile,
    ParseOperatingProfileArguments,
)


def receive_only_config():
    return {
        "RadarSource": "ETTUS",
        "EttusOperatingMode": "RECEIVE_ONLY",
        "EttusTimedTransmitEnabled": False,
        "EttusAtrGpioEnabled": False,
        "EttusRxFrequencyHz": 1.0e9,
        "EttusRxChannel": 0,
    }


class TestEttusOperatingProfiles(unittest.TestCase):
    def test_normal_startup_remains_receive_only(self):
        config = receive_only_config()
        arguments = ParseOperatingProfileArguments([])

        profile = ApplyOperatingProfile(config, arguments)

        self.assertEqual(profile, "RECEIVE_ONLY")
        self.assertFalse(config["EttusTimedTransmitEnabled"])
        self.assertFalse(config["Stage3E1LoopbackActive"])

    def test_loopback_options_without_profile_are_rejected(self):
        arguments = ParseOperatingProfileArguments([
            "--attenuation-db", "30",
        ])
        with self.assertRaisesRegex(ValueError, "require"):
            ApplyOperatingProfile(receive_only_config(), arguments)

    def test_loopback_requires_all_acknowledgements(self):
        argument_sets = (
            [
                "--stage3e1-loopback",
                "--attenuation-db", "30",
            ],
            [
                "--stage3e1-loopback",
                "--attenuation-db", "30",
                "--i-understand-rf-output-is-enabled",
            ],
            [
                "--stage3e1-loopback",
                "--attenuation-db", "30",
                "--i-understand-rf-output-is-enabled",
                "--i-confirm-txrx-to-rx2-loopback",
            ],
        )
        for argv in argument_sets:
            with self.subTest(argv=argv):
                with self.assertRaises(ValueError):
                    ApplyOperatingProfile(
                        receive_only_config(),
                        ParseOperatingProfileArguments(argv),
                    )

    def test_loopback_rejects_insufficient_attenuation(self):
        arguments = self.valid_arguments()
        arguments.attenuation_db = 29.9
        with self.assertRaisesRegex(ValueError, "at least 30"):
            ApplyOperatingProfile(receive_only_config(), arguments)

    def test_valid_loopback_profile_is_bounded_and_keeps_atr_off(self):
        config = receive_only_config()
        arguments = self.valid_arguments()

        profile = ApplyOperatingProfile(config, arguments)

        self.assertEqual(profile, "STAGE3E1_LOOPBACK")
        self.assertEqual(config["EttusOperatingMode"], "TIMED_TX_RX")
        self.assertTrue(config["EttusTimedTransmitEnabled"])
        self.assertTrue(config["EttusRfOutputAcknowledged"])
        self.assertTrue(config["EttusLoopbackConfirmed"])
        self.assertEqual(config["EttusExternalAttenuationDb"], 30.0)
        self.assertEqual(config["EttusTxGainDb"], 0.0)
        self.assertEqual(config["EttusRxGainDb"], 30.0)
        self.assertFalse(config["EttusAtrGpioEnabled"])
        self.assertEqual(config["Stage3E1MaximumTimedDwells"], 10)

    def test_main_has_automatic_timed_dwell_shutdown(self):
        main_text = (
            Path(__file__).with_name("VanguardxMain_scheduler.py")
            .read_text(encoding="utf-8")
        )
        self.assertIn("CompletedTimedDwells += 1", main_text)
        self.assertIn("automatic timed-dwell limit reached", main_text)
        self.assertIn('Config["InitialTransmitEnabled"] = False', main_text)

    @staticmethod
    def valid_arguments():
        return ParseOperatingProfileArguments([
            "--stage3e1-loopback",
            "--attenuation-db", "30",
            "--tx-gain-db", "0",
            "--rx-gain-db", "30",
            "--maximum-timed-dwells", "10",
            "--i-understand-rf-output-is-enabled",
            "--i-confirm-txrx-to-rx2-loopback",
            "--i-confirm-atr-trm-pa-disabled",
        ])


if __name__ == "__main__":
    unittest.main()
