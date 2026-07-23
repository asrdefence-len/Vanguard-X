"""Non-blocking network client used by the separate Vanguard X Qt UI."""

from __future__ import annotations

from collections import deque
import select
import socket
import threading
import time

from RadarLinkProtocol import EncodeMessage, FrameDecoder, MakeMessage, ProtocolError


class RadarLinkClient:
    def __init__(self, host="127.0.0.1", port=5810, heartbeat_interval_sec=0.5):
        self.Host = str(host)
        self.Port = int(port)
        self.HeartbeatIntervalSec = float(heartbeat_interval_sec)
        self.DisplayConfig = None
        self.FailStopTimeoutSec = None
        self.LastError = ""
        self._incoming_lock = threading.Lock()
        self._incoming = deque(maxlen=64)
        self._outgoing_lock = threading.Lock()
        self._pending_control = None
        self._sequence = 0
        self._connected = threading.Event()
        self._hello = threading.Event()
        self._stop = threading.Event()
        self._thread = None

    @property
    def Connected(self):
        return self._connected.is_set()

    def Start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._Run, name="RadarUiClient", daemon=True)
        self._thread.start()

    def WaitForDisplayConfig(self, timeout_sec=5.0):
        if not self._hello.wait(float(timeout_sec)):
            raise TimeoutError(self.LastError or "timed out waiting for radar server")
        if self.DisplayConfig is None:
            raise ConnectionError(self.LastError or "radar server did not provide configuration")
        return dict(self.DisplayConfig or {})

    def SendControlState(self, control_state):
        with self._outgoing_lock:
            self._pending_control = dict(control_state)

    def DrainMessages(self):
        with self._incoming_lock:
            messages = list(self._incoming)
            self._incoming.clear()
        return messages

    def Close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _TakeControlMessage(self):
        with self._outgoing_lock:
            payload = self._pending_control
            self._pending_control = None
        if payload is None:
            return None
        self._sequence += 1
        return MakeMessage("control_state", payload, self._sequence)

    def _StoreIncoming(self, message):
        if message["type"] == "server_hello":
            payload = message["payload"]
            is_reconnect = self.DisplayConfig is not None
            self.DisplayConfig = dict(payload.get("display_config", {}))
            self.FailStopTimeoutSec = payload.get("fail_stop_timeout_sec")
            self._hello.set()
            if is_reconnect:
                with self._incoming_lock:
                    self._incoming.append(message)
            return
        with self._incoming_lock:
            self._incoming.append(message)

    def _Run(self):
        while not self._stop.is_set():
            connection = None
            try:
                connection = socket.create_connection((self.Host, self.Port), timeout=2.0)
                connection.setblocking(False)
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.LastError = ""
                self._connected.set()
                decoder = FrameDecoder()
                outgoing = bytearray()
                last_heartbeat = 0.0
                while not self._stop.is_set():
                    if not outgoing:
                        message = self._TakeControlMessage()
                        now = time.monotonic()
                        if message is None and now - last_heartbeat >= self.HeartbeatIntervalSec:
                            self._sequence += 1
                            message = MakeMessage("heartbeat", {}, self._sequence)
                            last_heartbeat = now
                        if message is not None:
                            outgoing.extend(EncodeMessage(message))
                    readable, writable, exceptional = select.select(
                        [connection], [connection] if outgoing else [], [connection], 0.05
                    )
                    if exceptional:
                        raise ConnectionError("socket exception")
                    if readable:
                        data = connection.recv(65536)
                        if not data:
                            raise ConnectionError("radar server disconnected")
                        for message in decoder.Feed(data):
                            self._StoreIncoming(message)
                    if writable and outgoing:
                        sent = connection.send(outgoing)
                        del outgoing[:sent]
            except (OSError, ConnectionError, ProtocolError) as error:
                self.LastError = str(error)
            finally:
                self._connected.clear()
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
            if not self._stop.wait(0.5):
                continue
