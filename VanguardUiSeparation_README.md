# Vanguard X UI separation — Phase 1B timing-isolated implementation

This source set separates the current Qt5 operator display from the radar and
hardware process over a localhost TCP connection.  It intentionally preserves
the existing `RadarDisplay` interface used by `VanguardxMain_scheduler.py`.

## Process boundary

- Radar process: scheduler, Ettus, ATR, processing, CFAR, tracker, logger,
  pointing hardware, mission safety and the TCP server adapter.
- UI process: Qt5/PyQtGraph PPI, operator controls and future Mission tabs.
- Link data: control state, command results, measured beam angle, diagnostics,
  detections, tracks and a bounded peak-preserving range profile.
- Excluded from the link: raw IQ and the full range-Doppler cube.

At the end of each completed dwell, the radar thread reduces the display
products and prepares the complete bounded JSON frame. Socket transmission
then occurs in a background thread. The link thread is explicitly suspended
while the Ettus timed TX/RX dwell is being armed and captured. If the UI cannot
keep up, old prepared display snapshots are discarded rather than queued behind
radar acquisition.

The separate UI starts at Linux nice level `+10` by default so Qt/PyQtGraph
redraws yield CPU time to the radar backend. Use `--ui-nice 0` only for a
controlled comparison; it is not the recommended hardware-running setting on
the shared MiniPC.

## Files

- `VanguardxMain_scheduler.py` — updated main program with `--remote-ui` mode.
- `RadarDisplayQt5.py` — accepts local or network-prepared range profiles and
  displays radar-link status.
- `RadarRemoteDisplay.py` — headless radar-side display adapter/server.
- `RadarLinkClient.py` — reconnecting non-blocking UI-side client.
- `RadarLinkProtocol.py` — strict framed JSON protocol.
- `RunVanguardUiClient.py` — separate Qt5 UI entry point.
- `TestRadarLink.py` — protocol, bounded-product, round-trip and fail-stop tests.

Place all Python files in the existing Vanguard X software directory, replacing
the two existing files of the same name after retaining the normal git history.

## First run on the same MiniPC

Use simulation first. In terminal 1:

```bash
python3 VanguardxMain_scheduler.py --remote-ui --system-sim
```

The backend prints:

```text
Radar UI server listening on 127.0.0.1:5810
```

In terminal 2:

```bash
python3 RunVanguardUiClient.py
```

The existing PPI should appear. Start, Stop, timing Apply, track display and
range-profile display travel through the localhost link.

Run the focused tests with:

```bash
python3 -m unittest -v TestRadarLink.py
```

The five focused tests include verification that processed radar arrays are
accessed only by the radar calling thread and that prepared snapshot transport
is deferred throughout a simulated time-critical acquisition interval.

## Guarded RF-loopback verification

Keep the established cable, 30 dB-or-greater attenuation and guarded operating
procedure unchanged. Start directly in RF-loopback mode.

Terminal 1:

```bash
python3 VanguardxMain_scheduler.py \
    --remote-ui \
    --system-rf-loopback \
    --i-confirm-rf-loopback-safety
```

Terminal 2:

```bash
python3 RunVanguardUiClient.py
```

The radar terminal should continue to report `TX 64 ACK 64`. Any UHD
`time_error`, late-packet marker, or incomplete acknowledgement count remains a
fail-stop condition; do not compensate by changing RF gain or attenuation.

## Safety behavior

- Radar-side startup control state is always `STOP`, with TX disabled.
- The backend owns actual transmit capability; a UI cannot manufacture it.
- Loss of heartbeat or TCP connection forces `STOP`, disables scanning and
  inhibits TX. The current prototype timeout is two seconds.
- A UI backlog cannot stall acquisition; only the newest display snapshot is
  retained.
- The radar link thread is quiesced around the timed source-execution call.
- UI rendering runs below the radar backend's normal Linux scheduling priority.
- Local single-process Qt operation remains the default when `--remote-ui` is
  omitted.

## Moving the UI later

This first implementation binds only to `127.0.0.1` by default. Before binding
the command link to a LAN address, add the production security layer (mutual
authentication, command authorization and encryption) and test communication-
loss behavior on the isolated radar network. The Qt application itself then
moves unchanged; only its `--radar-host` address changes.

## GPS placement

GPS may be physically attached to the UI MiniPC if it is used only to centre
the operator map. If radar detections/tracks require georeferencing, ownship
motion compensation or north stabilisation, the radar backend must also receive
timestamped position/heading/quality messages. That navigation input should be
a separate validated data stream, not ordinary display state.
