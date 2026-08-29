"""
TRMInterface.py
Linux-side interface for the Vanguard X STM32 TRM shift controller.

The Linux PC builds the complete 28-bit Shinewave control word. The STM32 only
receives PING, STATUS and WORD commands, then clocks the word to the TRM.

Fast T and R switching remains under Ettus GPIO/ATR control.

to test: (with the STM32 connected)
python3 TRMInterface.py --port /dev/ttyACM0 --debug --rx-att 0.0 --tx-att 31.5 --force

"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

try:
    import serial
except ImportError as exc:
    serial = None
    _SERIAL_IMPORT_ERROR = exc
else:
    _SERIAL_IMPORT_ERROR = None


WORD_MASK = 0x0FFFFFFF
BIT_D1_FIXED = 1 << 1
BIT_RX_PATH_ENABLE = 1 << 2
BIT_TX_PATH_ENABLE = 1 << 27
RX_HIGH_BITS_ACTIVE_LOW = True


@dataclass
class TRMStatus:
    word: int
    raw_response: str


class TRMInterface:
    def __init__(
        self,
        port: str = (
            "/dev/serial/by-id/"
            "usb-STMicroelectronics_STM32_STLink_0671FF564953856767104019-if02"
        ),
        baudrate: int = 115200,
        timeout_s: float = 1.0,
        startup_wait_s: float = 0.5,
        debug: bool = False,
    ):
        self.port = str(port)
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self.startup_wait_s = float(startup_wait_s)
        self.debug = bool(debug)
        self.serial_port = None
        self.last_word: Optional[int] = None

    def open(self) -> None:
        if serial is None:
            raise ImportError(
                "TRMInterface requires pyserial. Install with: "
                "sudo apt install python3-serial"
            ) from _SERIAL_IMPORT_ERROR

        if self.serial_port is not None and self.serial_port.is_open:
            return

        self.serial_port = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout_s,
            write_timeout=self.timeout_s,
        )
        time.sleep(self.startup_wait_s)
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

    def close(self) -> None:
        if self.serial_port is not None:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            finally:
                self.serial_port = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def _ensure_open(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            self.open()

    def _command(self, command: str, timeout_s: Optional[float] = None) -> str:
        self._ensure_open()
        command = command.strip()
        if not command:
            raise ValueError("empty TRM command")

        if self.debug:
            print("TRM TX:", command)

        self.serial_port.write((command + "\n").encode("ascii"))
        self.serial_port.flush()

        deadline = time.monotonic() + (
            self.timeout_s if timeout_s is None else float(timeout_s)
        )

        while time.monotonic() < deadline:
            raw = self.serial_port.readline()
            if not raw:
                continue

            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue

            if self.debug:
                print("TRM RX:", line)

            if line.startswith("OK "):
                return line
            if line.startswith("ERR "):
                raise RuntimeError(line[4:])

        raise TimeoutError(
            f"No response from TRM controller on {self.port} for command: {command}"
        )

    def ping(self) -> str:
        return self._command("PING")

    def get_status(self) -> TRMStatus:
        response = self._command("STATUS")
        for token in response.split():
            if token.startswith("WORD="):
                word = int(token.split("=", 1)[1], 16)
                self.last_word = word
                return TRMStatus(word=word, raw_response=response)
        raise RuntimeError(f"Could not parse TRM status: {response}")

    @staticmethod
    def _validate_attenuation(value_db: float) -> int:
        value_db = float(value_db)
        steps = int(round(value_db * 2.0))
        if steps < 0 or steps > 63:
            raise ValueError("attenuation must be between 0.0 and 31.5 dB")
        if abs(value_db * 2.0 - steps) > 1e-6:
            raise ValueError("attenuation must use 0.5 dB increments")
        return steps

    @classmethod
    def _encode_tx_attenuation(cls, value_db: float) -> int:
        steps = cls._validate_attenuation(value_db)
        word = 0
        for mask, bit_position in (
            (1, 15), (2, 16), (4, 20),
            (8, 17), (16, 19), (32, 18),
        ):
            if steps & mask:
                word |= 1 << bit_position
        return word

    @classmethod
    def _encode_rx_attenuation(cls, value_db: float) -> int:
        steps = cls._validate_attenuation(value_db)
        word = 0

        for mask, bit_position in ((1, 9), (2, 10), (8, 11)):
            if steps & mask:
                word |= 1 << bit_position

        unusual = ((4, 14), (16, 13), (32, 12))
        if RX_HIGH_BITS_ACTIVE_LOW:
            for mask, bit_position in unusual:
                word |= 1 << bit_position
                if steps & mask:
                    word &= ~(1 << bit_position)
        else:
            for mask, bit_position in unusual:
                if steps & mask:
                    word |= 1 << bit_position

        return word

    @classmethod
    def build_word(
        cls,
        rx_att_db: float,
        tx_att_db: float,
        rx_path_enabled: bool = True,
        tx_path_enabled: bool = True,
    ) -> int:
        """
        Build the complete 28-bit TRM word.

        rx_path_enabled and tx_path_enabled are the slow serial standby bits
        D2 and D27, not the fast Ettus T/R signals.
        """
        word = BIT_D1_FIXED
        if rx_path_enabled:
            word |= BIT_RX_PATH_ENABLE
        if tx_path_enabled:
            word |= BIT_TX_PATH_ENABLE
        word |= cls._encode_rx_attenuation(rx_att_db)
        word |= cls._encode_tx_attenuation(tx_att_db)
        return word & WORD_MASK

    def write_word(self, word: int, force: bool = False) -> str:
        word = int(word)
        if word < 0 or word > WORD_MASK:
            raise ValueError("TRM word must be a 28-bit value")

        if not force and self.last_word == word:
            return f"UNCHANGED WORD={word:07X}"

        response = self._command(f"WORD {word:07X}")
        self.last_word = word
        return response

    def configure(
        self,
        rx_att_db: float,
        tx_att_db: float,
        force: bool = False,
    ) -> str:
        """Set attenuation while leaving both slow path-enable bits active."""
        word = self.build_word(
            rx_att_db=rx_att_db,
            tx_att_db=tx_att_db,
            rx_path_enabled=True,
            tx_path_enabled=True,
        )
        return self.write_word(word, force=force)

    def standby(self, force: bool = False) -> str:
        """
        Disable both paths through the serial standby bits.

        The Ettus T and R outputs should already be low before this is called.
        """
        word = self.build_word(
            rx_att_db=0.0,
            tx_att_db=31.5,
            rx_path_enabled=False,
            tx_path_enabled=False,
        )
        return self.write_word(word, force=force)


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Configure the Vanguard X Shinewave TRM"
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--rx-att", type=float, default=None)
    parser.add_argument("--tx-att", type=float, default=None)
    parser.add_argument("--standby", action="store_true")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    with TRMInterface(port=args.port, debug=args.debug) as trm:
        print(trm.ping())

        if args.standby:
            print(trm.standby(force=True))
        elif args.rx_att is not None or args.tx_att is not None:
            if args.rx_att is None or args.tx_att is None:
                parser.error("--rx-att and --tx-att must be supplied together")
            print(
                trm.configure(
                    rx_att_db=args.rx_att,
                    tx_att_db=args.tx_att,
                    force=args.force,
                )
            )

        print(trm.get_status())


if __name__ == "__main__":
    main()
