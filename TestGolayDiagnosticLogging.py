"""Hardware-free checks for one-shot Golay A/B HDF5 diagnostics."""

from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

import numpy as np

from DataLogger import DataLogger

try:
    import h5py
    from PlotGolayDiagnostic import CalculateGolayDiagnostic
except ImportError:
    h5py = None
    CalculateGolayDiagnostic = None


def MakeProcessed(DwellId=1, Scenario=False):
    NumPairs = 4
    NumSamples = 32
    RawA = np.zeros((NumPairs, NumSamples), dtype=np.complex64)
    RawB = np.zeros_like(RawA)
    CompressedA = np.zeros_like(RawA)
    CompressedB = np.zeros_like(RawB)
    CompressedA[:, 12] = 1.0 + 0.0j
    CompressedB[:, 12] = 1.0 + 0.0j
    return SimpleNamespace(
        DwellId=DwellId,
        RangeAxisM=np.arange(NumSamples, dtype=float) * 3.75,
        VelocityAxisMps=np.arange(NumPairs, dtype=float),
        MagnitudeDb=np.zeros((NumPairs, NumSamples), dtype=float),
        Diagnostics={
            "ProcessingMode": "GOLAY_COMPLEMENTARY",
            "RfTargetEmulatorActive": True,
            "RfTargetUseScenario": Scenario,
            "RfTargetRangeM": 6000.0,
            "RfTargetBearingDeg": 80.0,
            "RfTargetRadialVelocityMps": 0.0,
            "TransmitCommandCount": 8,
            "TransmitBurstAcknowledgementCount": 8,
            "TxGainDb": 40.0,
            "RxGainDb": 30.0,
            "ExternalAttenuationDb": 30.0,
        },
        GolayDiagnostic={
            "RawA": RawA,
            "RawB": RawB,
            "CompressedA": CompressedA,
            "CompressedB": CompressedB,
            "PulseValid": np.ones(8, dtype=bool),
            "WaveformAId": "Golay64A_20MHz",
            "WaveformBId": "Golay64B_20MHz",
            "PhysicalPriSec": 250.0e-6,
            "PairPriSec": 500.0e-6,
            "SampleRateHz": 40.0e6,
            "SamplesPerChip": 2,
        },
    )


@unittest.skipIf(h5py is None, "h5py is not installed")
class TestGolayDiagnosticLogging(unittest.TestCase):
    def test_first_complete_fixed_target_dwell_is_written_once(self):
        with tempfile.TemporaryDirectory() as Directory:
            Config = {
                "DataLoggingEnabled": True,
                "DataLogDirectory": Directory,
                "DataLogFilename": "golay.h5",
                "DataLogOverwrite": True,
                "LogGolayDiagnosticOnce": True,
                "LogRangeDoppler": False,
            }
            Logger = DataLogger(Config)
            Logger.log_dwell(MakeProcessed(DwellId=7), [])
            Logger.log_dwell(MakeProcessed(DwellId=8), [])
            Filename = Path(Logger.Filename)
            Logger.close()

            with h5py.File(Filename, "r") as File:
                self.assertIn("golay_diagnostic", File)
                Group = File["golay_diagnostic"]
                self.assertEqual(Group.attrs["dwell_id"], 7)
                self.assertEqual(Group["raw_a"].shape, (4, 32))
                self.assertEqual(Group["raw_b"].shape, (4, 32))
                self.assertEqual(Group["compressed_a"].shape, (4, 32))
                self.assertEqual(Group["compressed_b"].shape, (4, 32))
                Result = CalculateGolayDiagnostic(Group)

            self.assertEqual(Result["PeakIndex"], 12)
            self.assertEqual(Result["PeakRangeDifferenceSamples"], 0)
            self.assertAlmostEqual(Result["GainDifferenceDb"], 0.0)
            self.assertAlmostEqual(Result["PhaseDifferenceDeg"], 0.0)
            self.assertEqual(Result["ClippedComponentCount"], 0)

    def test_scenario_dwell_is_skipped_until_fixed_target_arrives(self):
        with tempfile.TemporaryDirectory() as Directory:
            Logger = DataLogger({
                "DataLoggingEnabled": True,
                "DataLogDirectory": Directory,
                "DataLogFilename": "golay.h5",
                "LogGolayDiagnosticOnce": True,
                "LogRangeDoppler": False,
            })
            Logger.log_dwell(MakeProcessed(DwellId=1, Scenario=True), [])
            self.assertNotIn("golay_diagnostic", Logger.File)

            Logger.log_dwell(MakeProcessed(DwellId=2, Scenario=False), [])
            self.assertIn("golay_diagnostic", Logger.File)
            self.assertEqual(
                Logger.File["golay_diagnostic"].attrs["dwell_id"],
                2,
            )
            Logger.close()


if __name__ == "__main__":
    unittest.main()
