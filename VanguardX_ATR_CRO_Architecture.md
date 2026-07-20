# Vanguard X ATR architecture and CRO verification

Status: Stage 3G CRO baseline complete; Stage 3H guarded loopback integration  
Hardware baseline: Ettus B200mini, serial `34A0320`  
Safety boundary: TRM and PA physically disconnected

## 1. Purpose and boundary

Stage 3G measured the B200mini Automatic Transmit/Receive (ATR) GPIO signals
before they are permitted to control the Vanguard X TRM or PA.  Stage 3H now
integrates that verified mapping into `EttusRadarSource.py` and
`VanguardxMain_scheduler.py` behind the explicit `--stage3h-atr-loopback`
profile.  Selecting that profile runs real finite timed TX/RX on the attenuated
SDR loopback; it is not a receive-only test.  No-profile startup remains
fail-closed with timed TX and ATR disabled.

The harness sends finite zero-IQ bursts.  Zero IQ does not make the transmit
chain safe or inactive: the TX/RX RF connector must be fitted with a 50-ohm RF
load, TX gain is fixed at 0 dB, and the TRM and PA must remain disconnected.

## 2. Verified connector and UHD mapping

The official B2xx hardware reference defines the B200mini J6 pinout as:
pin 1 = 3.3 V, pin 2 = GPIO_0, pin 3 = GPIO_1, pin 4 = GPIO_2,
pin 5 = GPIO_3, pin 6 = ground, pin 7 = 3.3 V, pins 8-11 = GPIO_4-7,
and pin 12 = ground.

UHD exposes this connector as GPIO bank `FP0`.  `CTRL=1` selects ATR control,
`DDR=1` makes a pin an output, and the four state registers mean:

| UHD register | Radio state |
| --- | --- |
| `ATR_0X` | idle |
| `ATR_RX` | receive only |
| `ATR_TX` | transmit only |
| `ATR_XX` | simultaneous transmit and receive |

## 3. Vanguard X Stage 3G state encoding

| Radio state | J6 pin 3 TX | J6 pin 4 RX | J6 pin 5 overlap witness |
| --- | ---: | ---: | ---: |
| Idle (`ATR_0X`) | 0 | 0 | 0 |
| RX only (`ATR_RX`) | 0 | 1 | 0 |
| TX only (`ATR_TX`) | 1 | 0 | 0 |
| TX and RX (`ATR_XX`) | 0 | 0 | 1 |

TX and RX are provisionally active high.  J6 pin 5 / GPIO_3 is diagnostic only
and is never an operational TRM/PA control.  It solves an observability problem:
the fail-low operational encoding makes idle and unintended full duplex look
identical on the first two channels.  The third channel positively identifies
any `ATR_XX` interval while both operational outputs remain inhibited.

## 4. Safe GPIO transition order

On setup the harness and Stage 3H operational source:

1. preloads manual `OUT=0`;
2. selects manual control with `CTRL=0`;
3. enables output direction with `DDR=1`;
4. writes and reads back all four ATR state words; and
5. transfers ownership to ATR with `CTRL=1` as the final step.

On normal exit, exception, or Ctrl-C, both paths return all three pins to
manual driven-low state and holds the B200mini handle for the configured
shutdown observation time.

This controlled shutdown is not the same as an unpowered or released device.
Ettus documents the B2xxmini power-on and initial GPIO state as high impedance
with internal pull-ups.  Therefore, before the TRM is ever connected, the
external interface must provide fail-low biasing and an independent PA inhibit.
The final design must not rely on B200mini firmware or an open-circuit GPIO to
hold TX disabled.

## 5. Fixed timing under test

| Parameter | Stage 3G default |
| --- | ---: |
| Sample rate | 40.000 MS/s |
| PRF / PRI | 2 kHz / 500 us |
| Pulses / CPI | 32 / 16 ms |
| Dwell cadence | 100 ms |
| Timed-pair queue | depth 20 |
| Command lead | 5 ms |
| TX ATR envelope | 208 samples / 5.200 us |
| Leading zero pre-roll | 8 samples / 0.200 us |
| Frank10 RF waveform interval | 0.200-5.200 us; 200 samples / 5.000 us |
| RX start | 6.200 us from PRI; 6.000 us from RF waveform origin |
| RX sample count | 4043 |
| Nominal RX interval | 6.200-107.275 us |
| Nominal TX-to-RX data guard | 1.000 us |

ATR marks UHD radio state, not merely the host command call.  Consequently the
oscilloscope measurements—not the nominal data intervals above—become the
authoritative TX enable width, receiver-enable timing, and switching guard used
for the TRM interface design.

The measured GPIO edge rise time is approximately 100 ns.  Stage 3G therefore
adds an 8-sample / 200 ns zero pre-roll before the unchanged 200-sample Frank10
waveform.  This padding belongs to the timed-transmit transport envelope and
must not be included in the waveform definition or matched-filter reference.
The RF and range time origin is the first non-zero waveform sample at +200 ns.
Otherwise the pre-roll would introduce a 200 ns delay, equivalent to a 30 m
two-way radar range bias.  RX moves by the same 200 ns so the measured 1 us
TX-to-RX guard and the RF-relative receiver timing are preserved.

## 6. CRO connection and setup

- Fit a 50-ohm RF load to the B200mini TX/RX RF connector.
- Keep the TRM, PA, up/down converter, antenna, and X6-60 motor/positioning unit
  out of this electrical test path.
- Use 10x high-impedance probes.  Never select a 50-ohm CRO input for a GPIO;
  it would overload the 3.3 V LVCMOS output.
- Connect CH1 to J6 pin 3, CH2 to J6 pin 4, CH3 to J6 pin 5, and the probe
  grounds to J6 pin 6 or pin 12.
- DC couple all channels and trigger on the CH1 rising edge.
- First view one 500 us PRI, then expand around the TX-to-RX transition, then
  view the 16 ms CPI and 100 ms dwell cadence.

## 7. Run command

```bash
python3 RunEttusAtrCroVerification.py \
    --serial 34A0320 \
    --prf-hz 2000 \
    --pulses 32 \
    --queue-depth 20 \
    --lead-ms 5 \
    --continuous \
    --i-confirm-trm-and-pa-disconnected \
    --i-confirm-txrx-terminated-50-ohm \
    --i-confirm-cro-inputs-high-impedance \
    --i-understand-zero-iq-still-enables-tx-chain
```

Continuous mode starts a new CPI on the established 100 ms dwell cadence until
Ctrl-C.  The `finally` block must force and read back manual low before the
B200mini handle is released.  There is no shutdown observation delay by
default; add `--shutdown-observation-sec N` only when a timed safe-low hold is
specifically required.

### Stage 3H main-program loopback command

With TX/RX connected through at least 30 dB attenuation to RX2, and with the
TRM and PA disconnected, run:

```bash
python3 VanguardxMain_scheduler.py \
    --stage3h-atr-loopback \
    --attenuation-db 30 \
    --tx-gain-db 0 \
    --rx-gain-db 10 \
    --i-understand-rf-output-is-enabled \
    --i-confirm-txrx-to-rx2-loopback \
    --i-confirm-trm-pa-disconnected \
    --i-confirm-atr-cro-verified
```

The program starts in operator STOP with transmission unarmed.  Press Start to
begin the 100 ms cadence timed TX/RX dwells.  The Stage 3H profile fixes the
verified mapping to FP0 bits 1, 2, and 3; requires the eight-sample pre-roll;
reads back the GPIO state words before enabling FPGA ATR ownership; and forces
manual driven-low GPIO state during controlled shutdown.

### Stage 3I simulation-only overlapping ATR target command

The delayed loopback target is generated by transmitting during the active RX
window.  That is intentionally unlike a final half-duplex radar and forces UHD
into `ATR_XX`.  For isolated SDR-loopback target testing only, Stage 3I changes
the `ATR_XX` word from witness-only (`0b1000`) to TX + RX + witness (`0b1110`):

```bash
python3 VanguardxMain_scheduler.py \
    --stage3i-atr-rf-target-overlap \
    --attenuation-db 30 \
    --loopback-hardware-delay-samples 166 \
    --i-understand-rf-output-is-enabled \
    --i-confirm-txrx-to-rx2-loopback \
    --i-confirm-trm-pa-disconnected \
    --i-confirm-atr-cro-verified
```

When the gain options are omitted, Stage 3I uses the verified protected-loopback
target settings of 50 dB TX gain and 30 dB RX gain.  Explicit `--tx-gain-db`
and `--rx-gain-db` values continue to override those profile defaults.  These
high-gain defaults apply only to Stage 3I; Stage 3H and the other guarded
loopback profiles retain the conservative 0 dB TX / 10 dB RX defaults.

The TRM and PA must remain disconnected in Stage 3I.  This overlapping state
is prohibited from the final product profile and does not change the normal
Stage 3H fail-low `ATR_XX` encoding.

## 8. Measurement worksheet and acceptance

Do not mark Stage 3G passed from console output alone.  Save CRO screenshots and
enter the measurements below.

| Measurement | Expected behaviour | Measured | Pass/Fail |
| --- | --- | --- | --- |
| Configured idle, CH1/CH2/CH3 | all low |  |  |
| TX active polarity and high level | CH1 active high |  |  |
| RX active polarity and high level | CH2 active high |  |  |
| Initial TX ATR width | 5.000 us before pre-roll decision | 5.000 us | Pass |
| GPIO edge rise time | record measured 10-90% edge | approximately 100 ns | Pass |
| Revised TX ATR width | stable; record actual against 5.200 us envelope |  |  |
| RF waveform start from TX ATR rise | 0.200 us by 8-sample construction |  |  |
| RX rising edge from PRI reference | stable; record actual against 6.200 us nominal |  |  |
| RX ATR width | stable; record actual against 101.075 us data window |  |  |
| TX-to-RX low guard | 1.000 us | 1.000 us before pre-roll change | Pass; recheck revised envelope |
| CH1 and CH2 simultaneous high | never observed |  |  |
| CH3 overlap witness | never high during the test |  |  |
| PRI recurrence | 500 us |  |  |
| CPI envelope | 32 pulses / 16 ms |  |  |
| Dwell recurrence | 100 ms |  |  |
| Normal controlled shutdown | all three driven low |  |  |
| Ctrl-C controlled shutdown | all three driven low |  |  |
| Process/device release | record high-Z/pull-up behaviour; not an operational pass state |  |  |

Stage 3G passes only when polarity, timing, non-overlap, and both controlled
shutdown paths are captured and repeatable.  Any CH3 pulse is a hard failure in
Stage 3G or normal Stage 3H.  CH3 is deliberately high during the explicitly
selected Stage 3I simulated-target overlap interval.
The measured TX and RX edge timings must be reviewed against TRM switching,
receiver recovery, and PA sequencing requirements before the TRM or PA is
connected.  Stage 3H authorizes ATR only for the acknowledged attenuated SDR
loopback profile; it does not authorize connection of the TRM or PA.

## 9. Sources

- Ettus B2xx hardware reference: https://files.ettus.com/manual/page_usrp_b200.html
- UHD GPIO API and ATR state definitions: https://files.ettus.com/manual/page_gpio_api.html
- UHD `multi_usrp` GPIO attributes: https://files.ettus.com/manual/classuhd_1_1usrp_1_1multi__usrp.html
- Ettus B2xx GPIO power-on state: https://kb.ettus.com/B200/B210/B200mini/B205mini/B206mini
