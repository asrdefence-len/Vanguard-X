"""Basic regression tests for the first pulse-plan architecture."""

import numpy as np

from RadarPlans import make_uniform_dwell_plan
from WaveformLibrary import WaveformLibrary
from SimulatedSource import SimulatedSource
from RadarProcessor import RadarProcessor


def main():
    config = {
        "SampleRate": 1e6,
        "NumSamples": 512,
        "NumPulses": 16,
        "PRI": 1e-3,
        "RfFrequency": 9.4e9,
        "TransmitPowerW": 1.0,
        "AntennaGainDb": 0.0,
        "SystemLossDb": 0.0,
        "TargetRangeM": 3000.0,
        "TargetVelocityMps": 5.0,
        "TargetRcsSqm": 100.0,
        "NoisePowerW": 1e-15,
        "BoresightDeg": 0.0,
        "BeamwidthDeg": 5.0,
    }

    plan = make_uniform_dwell_plan(
        dwell_id=1,
        task_id=1,
        task_type="SEARCH",
        waveform_id="Barker13_20MHz",
        sample_rate=config["SampleRate"],
        num_samples=config["NumSamples"],
        num_pulses=config["NumPulses"],
        pri_sec=config["PRI"],
    )

    assert plan.NumPulses == 16
    assert plan.WaveformName == "Barker13_20MHz"
    assert np.isclose(plan.PRI, 1e-3)
    assert all(p.WaveformId == "Barker13_20MHz" for p in plan.PulsePlans)

    waveforms = WaveformLibrary(config)
    waveforms.LoadDefaultWaveforms()
    source = SimulatedSource(config, waveforms)
    processor = RadarProcessor(config, waveforms)

    raw = source.ExecuteDwell(plan)
    processed = processor.Process(raw, plan)

    assert raw.IQ.shape == (16, 512)
    assert raw.PulseTimesSec.shape == (16,)
    assert raw.PulsePriSec.shape == (16,)
    assert raw.PulseWaveformIds == ["Barker13_20MHz"] * 16
    assert processed.RangeDopplerMap.shape == (16, 512)
    assert processed.Diagnostics["ProcessingMode"] == "UNIFORM_PRI_FFT"

    print("Pulse-plan regression test passed")


if __name__ == "__main__":
    main()
