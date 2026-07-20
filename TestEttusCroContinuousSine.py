import unittest
import RunEttusCroContinuousSine as Program


class TestContinuousSine(unittest.TestCase):
    def test_interlocks_required(self):
        a = Program.parse(["--attenuation-db", "30"])
        with self.assertRaises(ValueError): Program.validate(a)

    def test_defaults_make_100_mhz(self):
        a = Program.parse([
            "--attenuation-db", "30",
            "--i-understand-rf-output-is-enabled",
            "--i-confirm-scope-is-50-ohm-terminated",
        ])
        Program.validate(a)
        self.assertEqual(a.centre_frequency_mhz+a.baseband_tone_mhz, 100)


if __name__ == "__main__": unittest.main(verbosity=2)
