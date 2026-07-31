# Stage 12A — Moving-platform target geometry

This focused update moves the default simulated targets away from the
1 km-radius platform route centred 5 km east of the mission origin.

## Revised scene

| Target | Initial range | Initial bearing | Speed | Heading |
| --- | ---: | ---: | ---: | ---: |
| `Container_Vessel_200m` | 7.0 km | 30° true | 5 m/s | 30° true |
| `Container_Vessel1_200m` | 10.0 km | 110° true | 5 m/s | 110° true |
| `Reflector_-35deg` | 11.0 km | 70° true | stationary | — |

The extended-vessel axes follow their headings, retaining long range profiles.
The moving vessels travel generally away from the platform route.

`TestDefaultSceneSeparation.py` checks every simulated scatterer at ten-second
intervals for twenty minutes. At each epoch it considers every possible
platform position on the circular route, not only the simultaneous platform
position.

Expected result:

```text
Default scene separation: minimum=4.499 km, maximum=12.504 km, duration=20 min
```

This removes near-target geometry as a confounding factor. It does not change
the operational 2 km CFAR blanking, waveform processing, CFAR configuration,
target RCS, platform route, or Stage 12 Golay correction.

## Install

Extract the archive into the Vanguard X software directory:

```bash
cd ~/Projects/Software
unzip -o ~/Downloads/VanguardX_Stage12A_TargetGeometry_20260730.zip \
  -d ~/Projects/Software
```

## Validate

```bash
python3 -m unittest -v \
  TestDefaultSceneSeparation.py \
  TestTargetScenarioPlatformMotion.py

python3 -m py_compile \
  TargetScenario.py \
  TestDefaultSceneSeparation.py

git diff --check
git status --short
```
