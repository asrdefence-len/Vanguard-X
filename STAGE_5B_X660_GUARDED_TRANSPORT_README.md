# Stage 5B — X6-60 guarded motion transport

Stage 5B adds a transport boundary for the four motion payloads proven in
Stage 5A. It does **not** connect motion to `X660ReadOnlyController`,
`CreateX660Controller`, `PointingManager`, the radar scheduler, or a command-line
launcher.

## Safety boundary

`X660GuardedMotionTransport`:

- defaults to `DryRun=True`;
- cannot open `can0` or any other CAN interface;
- imports neither `python-can` nor SocketCAN support;
- requires the caller to inject a bus;
- requires both `IUnderstandMotionWillOccur=True` and
  `IConfirmMotionAreaIsClear=True` before the injected bus can be used;
- accepts only `0x81`, `0xA2`, `0xA4`, and `0xA8` eight-byte payloads;
- rejects shutdown, brake, reset, configuration, and telemetry commands;
- always uses a standard (not extended) request frame;
- validates node ID, request ID, payload, reply ID, reply command, frame type,
  and reply DLC.

The existing real controller remains telemetry-only. Its scheduler-facing
motion methods and `Stop()` remain locked and transmit nothing.

## Software-only verification

Run:

```bash
python3 -m py_compile \
    X660GuardedMotionTransport.py \
    TestX660GuardedMotionTransport.py

python3 -m unittest -v \
    TestX660MotionProtocol.py \
    TestX660GuardedMotionTransport.py \
    TestX660ReadOnlySafety.py \
    TestX660ReadOnlyController.py \
    TestX660UnlimitedAzimuth.py
```

The Stage 5B tests exercise live-mode logic only with an in-memory fake bus.
They do not import `python-can`, inspect `can0`, transmit a real CAN frame, or
move the X6-60.

## Explicitly deferred

The following remain outside Stage 5B:

- opening a real CAN interface;
- a real-motor motion launcher;
- operational speed and travel envelopes;
- motion watchdog and emergency-stop policy;
- scheduler/factory integration;
- enabling any motion method on the read-only controller.

Those require a later, separately reviewed stage.
