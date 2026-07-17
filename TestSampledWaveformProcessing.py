"""Integration regression for sampled waveform pulse compression metadata."""

import unittest

import numpy as np

from DataTypes import RawDwellData
from RadarPlans import make_uniform_dwell_plan
from RadarProcessor import RadarProcessor
from WaveformLibrary import WaveformLibrary


class TestSampledWaveformProcessing(unittest.TestCase):
    def test_frank10_20mhz_uses_200_sample_filter_but_100_chip_gain(self):
        SampleRateHz = 40.0e6
        NumPulses = 8
        NumSamples = 1024
        PriSec = 500.0e-6
        WaveformId = "Frank10_20MHz"

        Library = WaveformLibrary(
            {"EttusSampleRateHz": SampleRateHz}
        )
        Library.LoadDefaultWaveforms()
        TxWaveform = Library.Get(WaveformId)

        Iq = np.zeros((NumPulses, NumSamples), dtype=np.complex64)
        StartIndex = 200
        Iq[:, StartIndex:StartIndex + len(TxWaveform)] = TxWaveform

        Raw = RawDwellData(
            DwellId=1,
            IQ=Iq,
            SampleRate=SampleRateHz,
            PRI=PriSec,
            TimeStamp=0.0,
        )
        Dwell = make_uniform_dwell_plan(
            dwell_id=1,
            task_id=1,
            task_type="SEARCH",
            waveform_id=WaveformId,
            sample_rate=SampleRateHz,
            num_samples=NumSamples,
            num_pulses=NumPulses,
            pri_sec=PriSec,
        )
        Processor = RadarProcessor(
            {"RfFrequency": 9.4e9},
            Library,
        )

        Processed = Processor.Process(Raw, Dwell)
        Diagnostics = Processed.Diagnostics

        self.assertEqual(Processed.RangeCompressed.shape, Iq.shape)
        self.assertEqual(
            Processed.RangeDopplerMap.shape,
            Iq.shape,
        )
        self.assertEqual(Diagnostics["WaveformId"], WaveformId)
        self.assertEqual(Diagnostics["CodeLength"], 100)
        self.assertEqual(Diagnostics["ChipCount"], 100)
        self.assertEqual(Diagnostics["SampledWaveformLength"], 200)
        self.assertEqual(Diagnostics["SamplesPerChip"], 2)
        self.assertEqual(Diagnostics["ChipRateHz"], 20.0e6)
        self.assertAlmostEqual(
            Diagnostics["PulseDurationSec"],
            5.0e-6,
            places=15,
        )
        self.assertAlmostEqual(
            Diagnostics["IdealProcessingGainDb"],
            20.0,
            places=12,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
