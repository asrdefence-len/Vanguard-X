import unittest
import RunEttusCroTransmitTest as Program


class TestCroTransmit(unittest.TestCase):
    def test_safety_flags_required(self):
        a = Program.arguments(["--attenuation-db", "30"])
        with self.assertRaises(ValueError): Program.validate(a)

    def test_default_observed_carrier_is_100_mhz(self):
        a = Program.arguments([
            "--attenuation-db", "30",
            "--i-understand-rf-output-is-enabled",
            "--i-confirm-scope-is-50-ohm-terminated",
        ])
        Program.validate(a)
        self.assertEqual(a.centre_frequency_mhz+a.baseband_tone_mhz, 100.0)

    def test_burst_must_fit_pri(self):
        a = Program.arguments([
            "--attenuation-db", "30", "--burst-us", "20000",
            "--i-understand-rf-output-is-enabled",
            "--i-confirm-scope-is-50-ohm-terminated",
        ])
        with self.assertRaises(ValueError): Program.validate(a)


if __name__ == "__main__": unittest.main(verbosity=2)
