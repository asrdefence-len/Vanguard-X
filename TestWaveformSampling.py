"""
Regression test for sampled waveform construction.

Initial Vanguard X waveform baseline:
    SDR sample rate: 40 MS/s
    Chip rate:       20 Mchip/s
    Samples/chip:    2
"""

import numpy as np
from WaveformLibrary import WaveformLibrary


def main():
    config = {
        "EttusSampleRateHz": 40.0e6,
        "DefaultChipRateHz": 20.0e6,
    }

    library = WaveformLibrary(config)
    library.LoadDefaultWaveforms()

    # Barker-13: 13 chips, two samples per chip.
    assert library.GetCodeLength("Barker13") == 13
    assert library.GetSamplesPerChip("Barker13") == 2
    assert library.GetSampleLength("Barker13") == 26
    assert len(library.Get("Barker13")) == 26
    assert abs(library.GetPulseDurationSec("Barker13") - 0.65e-6) < 1e-12

    # Frank-10: 10 x 10 = 100 chips, two samples per chip.
    assert library.GetCodeLength("Frank10") == 100
    assert library.GetSamplesPerChip("Frank10") == 2
    assert library.GetSampleLength("Frank10") == 200
    assert len(library.Get("Frank10")) == 200
    assert abs(library.GetPulseDurationSec("Frank10") - 5.0e-6) < 1e-12

    # Each 20 Mchip/s chip must be represented by two identical samples.
    barker_samples = library.Get("Barker13")
    frank_samples = library.Get("Frank10")

    assert np.array_equal(
        barker_samples[0::2],
        barker_samples[1::2]
    )
    assert np.array_equal(
        frank_samples[0::2],
        frank_samples[1::2]
    )

    # A waveform-specific 10 Mchip/s definition uses four samples per chip.
    ten_mchip_config = {
        "EttusSampleRateHz": 40.0e6,
        "DefaultChipRateHz": 20.0e6,
        "WaveformChipRatesHz": {
            "Frank10": 10.0e6,
        },
    }

    ten_mchip_library = WaveformLibrary(ten_mchip_config)
    ten_mchip_library.LoadDefaultWaveforms()

    assert ten_mchip_library.GetSamplesPerChip("Frank10") == 4
    assert ten_mchip_library.GetSampleLength("Frank10") == 400
    assert abs(
        ten_mchip_library.GetPulseDurationSec("Frank10") - 10.0e-6
    ) < 1e-12


    # Processing gain must remain based on chip count, not sampled length.
    assert abs(library.GetIdealProcessingGainDb("Barker13") - 11.1394335) < 1e-6
    assert abs(library.GetIdealProcessingGainDb("Frank10") - 20.0) < 1e-12

    print("Waveform sampling regression test passed")


if __name__ == "__main__":
    main()
