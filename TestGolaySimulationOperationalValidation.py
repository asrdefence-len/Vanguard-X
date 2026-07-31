"""Regression for the complete simulated Golay operational gate."""

import unittest

from GolaySimulationOperationalValidation import RunValidation


class TestGolaySimulationOperationalValidation(unittest.TestCase):
    def test_complete_simulated_pair_chain_passes(self):
        Result = RunValidation()

        self.assertTrue(Result["Passed"], Result["Checks"])
        self.assertLess(Result["CorrectedPslrDb"], -50.0)
        self.assertGreater(Result["AOnlyPslrDb"], -15.0)
        self.assertEqual(
            Result["PulseWaveformIds"][:4],
            [
                "Golay64A_20MHz",
                "Golay64B_20MHz",
                "Golay64A_20MHz",
                "Golay64B_20MHz",
            ],
        )


if __name__ == "__main__":
    unittest.main()
