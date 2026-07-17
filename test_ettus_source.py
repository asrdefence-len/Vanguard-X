#!/usr/bin/env python3

import numpy as np

from EttusRadarSource import EttusRadarSource
from RadarPlans import make_uniform_dwell_plan
from WaveformLibrary import WaveformLibrary


CONFIG = {
    "EttusSerial": "34A0320",
    "EttusRxFrequencyHz": 1.0e9,
    "EttusRxGainDb": 10.0,
    "EttusRxAntenna": "RX2",
    "EttusRxChannel": 0,
    "EttusReceiveTimeoutSec": 1.0,
    "EttusCommandLeadTimeSec": 0.10,
    "EttusDebug": True,

    # Initial conservative test values
    "SampleRate": 1.0e6,
    "NumSamples": 1000,
    "NumPulses": 8,
    "PRI": 0.005,
}


def main():
    waveforms = WaveformLibrary(CONFIG)
    waveforms.LoadDefaultWaveforms()

    dwell = make_uniform_dwell_plan(
        dwell_id=1,
        task_id=1,
        task_type="SEARCH",
        waveform_id="Barker13",
        sample_rate=CONFIG["SampleRate"],
        num_samples=CONFIG["NumSamples"],
        num_pulses=CONFIG["NumPulses"],
        pri_sec=CONFIG["PRI"],
        azimuth_deg=0.0,
        elevation_deg=0.0,
        rx_attenuation_db=0.0,
        tx_attenuation_db=31.5,
        metadata={"Test": "Stage4A2ReceiveOnly"},
    )

    source = EttusRadarSource(CONFIG, waveforms)

    try:
        source.Initialise()
        raw = source.ExecuteDwell(dwell)

        print()
        print("Capture complete")
        print(f"IQ shape:          {raw.IQ.shape}")
        print(f"IQ dtype:          {raw.IQ.dtype}")
        print(f"Sample rate:       {raw.SampleRate}")
        print(f"Pulse valid:       {raw.PulseValid}")
        print(f"Mean magnitude:    {np.mean(np.abs(raw.IQ)):.6e}")
        print(f"Maximum magnitude: {np.max(np.abs(raw.IQ)):.6e}")

        expected_shape = (
            CONFIG["NumPulses"],
            CONFIG["NumSamples"],
        )

        if raw.IQ.shape != expected_shape:
            raise RuntimeError(
                f"Unexpected IQ shape {raw.IQ.shape}; "
                f"expected {expected_shape}"
            )

        if not np.all(raw.PulseValid):
            print("WARNING: one or more PRI captures were invalid")
            for index, diag in enumerate(
                raw.Diagnostics.get("PulseDiagnostics", [])
            ):
                print(
                    f"PRI {index}: "
                    f"received={diag.get('ReceivedSamples')}, "
                    f"error={diag.get('LastMetadataError')}, "
                    f"late={diag.get('LateCommandCount')}, "
                    f"timeout={diag.get('TimeoutCount')}, "
                    f"overflow={diag.get('OverflowCount')}"
                )
        else:
            print("PASS: every PRI returned the requested IQ samples")

    finally:
        source.Shutdown()


if __name__ == "__main__":
    main()
