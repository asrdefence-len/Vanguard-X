"""
TRMController.py
MicroPython firmware for the Vanguard X Shinewave SW-TRM-93009500-47C.

Target: STM32 NUCLEO-F446RE running MicroPython.

Arduino header assignments:
    D2 = SEL  = PA10
    D3 = CLK  = PB3
    D4 = DATA = PB5
    D5 = LOAD = PB4

Commands over USB serial:
    PING
    STATUS
    SAFE
    WORD 08000002
    CONFIG 1 1 10.0 6.5
    RXEN 1
    TXEN 0
    RXATT 10.0
    TXATT 6.5
    APPLY
    HELP
"""

from machine import Pin, ADC
import sys
import time

try:
    import uselect as select
except ImportError:
    import select

SEL = Pin("A10", Pin.OUT)
CLK = Pin("B3", Pin.OUT)
DATA = Pin("B5", Pin.OUT)
LOAD = Pin("B4", Pin.OUT)

# TRM temperature monitor. Arduino A1 on the NUCLEO-F446RE is PA1.
# The Shinewave interface documentation identifies a TMP36-style monitor.
# TMP36 nominal transfer: 500 mV at 0 C, 10 mV/C.
try:
    TEMP_ADC = ADC(Pin("A1"))
except Exception:
    TEMP_ADC = None

ADC_REFERENCE_V = 3.3
ADC_FULL_SCALE = 65535.0
TMP36_ZERO_C_V = 0.500
TMP36_VOLTS_PER_C = 0.010
TEMP_AVERAGE_SAMPLES = 16

WORD_MASK = 0x0FFFFFFF
RX_HIGH_BITS_ACTIVE_LOW = True

DATA_SETUP_US = 2
CLOCK_HIGH_US = 2
CLOCK_LOW_US = 2
LOAD_SETUP_US = 2
LOAD_PULSE_US = 5
SEL_HOLD_US = 2

BIT_D1_FIXED = 1 << 1
BIT_RX_ENABLE = 1 << 2
BIT_TX_ENABLE = 1 << 27


def idle_pins():
    SEL.value(0)
    CLK.value(0)
    DATA.value(0)
    LOAD.value(0)


idle_pins()


def validate_attenuation(value_db):
    value_db = float(value_db)
    steps = int(round(value_db * 2.0))
    if steps < 0 or steps > 63:
        raise ValueError("attenuation must be between 0.0 and 31.5 dB")
    return steps / 2.0


def encode_tx_attenuation(value_db):
    half_db_steps = int(round(validate_attenuation(value_db) * 2.0))
    word = 0
    for mask, bit_position in (
        (1, 15), (2, 16), (4, 20),
        (8, 17), (16, 19), (32, 18),
    ):
        if half_db_steps & mask:
            word |= 1 << bit_position
    return word


def encode_rx_attenuation(value_db):
    half_db_steps = int(round(validate_attenuation(value_db) * 2.0))
    word = 0

    # Documented active-high RX bits.
    for mask, bit_position in ((1, 9), (2, 10), (8, 11)):
        if half_db_steps & mask:
            word |= 1 << bit_position

    # Vendor table marks D14=2 dB, D13=8 dB and D12=16 dB active-low.
    unusual = ((4, 14), (16, 13), (32, 12))
    if RX_HIGH_BITS_ACTIVE_LOW:
        for mask, bit_position in unusual:
            word |= 1 << bit_position
            if half_db_steps & mask:
                word &= ~(1 << bit_position)
    else:
        for mask, bit_position in unusual:
            if half_db_steps & mask:
                word |= 1 << bit_position

    return word



def read_temperature():
    """Return (temperature_c, adc_voltage_v) from the TRM A1 monitor.

    The conversion assumes the TMP36 monitor is connected directly to A1
    with a 3.3 V ADC reference and no external divider.  Averaging is used
    only to reduce ADC noise; the RF timing path is independent of this read.
    """
    if TEMP_ADC is None:
        return None, None
    total = 0
    count = max(1, int(TEMP_AVERAGE_SAMPLES))
    for _ in range(count):
        total += int(TEMP_ADC.read_u16())
    raw = float(total) / float(count)
    voltage_v = raw * ADC_REFERENCE_V / ADC_FULL_SCALE
    temperature_c = (voltage_v - TMP36_ZERO_C_V) / TMP36_VOLTS_PER_C
    return float(temperature_c), float(voltage_v)


def decode_tx_attenuation(word):
    steps = 0
    for mask, bit_position in (
        (1, 15), (2, 16), (4, 20),
        (8, 17), (16, 19), (32, 18),
    ):
        if int(word) & (1 << bit_position):
            steps |= mask
    return steps / 2.0


def decode_rx_attenuation(word):
    word = int(word)
    steps = 0
    for mask, bit_position in ((1, 9), (2, 10), (8, 11)):
        if word & (1 << bit_position):
            steps |= mask
    unusual = ((4, 14), (16, 13), (32, 12))
    if RX_HIGH_BITS_ACTIVE_LOW:
        for mask, bit_position in unusual:
            if not (word & (1 << bit_position)):
                steps |= mask
    else:
        for mask, bit_position in unusual:
            if word & (1 << bit_position):
                steps |= mask
    return steps / 2.0


def build_word(rx_enable=False, tx_enable=False,
               rx_att_db=0.0, tx_att_db=0.0):
    word = BIT_D1_FIXED
    if bool(rx_enable):
        word |= BIT_RX_ENABLE
    if bool(tx_enable):
        word |= BIT_TX_ENABLE
    word |= encode_rx_attenuation(rx_att_db)
    word |= encode_tx_attenuation(tx_att_db)
    return word & WORD_MASK


def write_word(word):
    """Transmit exactly 28 bits, D0 first, then pulse LOAD."""
    word = int(word) & WORD_MASK

    CLK.value(0)
    LOAD.value(0)
    DATA.value(0)
    SEL.value(1)
    time.sleep_us(SEL_HOLD_US)

    for bit_index in range(28):
        DATA.value((word >> bit_index) & 1)
        time.sleep_us(DATA_SETUP_US)

        CLK.value(1)
        time.sleep_us(CLOCK_HIGH_US)

        # TRM samples DATA on this falling edge.
        CLK.value(0)
        time.sleep_us(CLOCK_LOW_US)

    DATA.value(0)
    time.sleep_us(LOAD_SETUP_US)

    # TRM latches the shift register on the rising edge of LOAD.
    LOAD.value(1)
    time.sleep_us(LOAD_PULSE_US)
    LOAD.value(0)
    time.sleep_us(SEL_HOLD_US)

    SEL.value(0)
    DATA.value(0)
    return word


class TRMController:
    def __init__(self):
        self.rx_enable = False
        self.tx_enable = False
        self.rx_att_db = 0.0
        self.tx_att_db = 0.0
        self.last_word = build_word()

    def apply(self):
        self.last_word = build_word(
            self.rx_enable,
            self.tx_enable,
            self.rx_att_db,
            self.tx_att_db,
        )
        write_word(self.last_word)
        return self.last_word

    def safe(self):
        self.rx_enable = False
        self.tx_enable = False
        self.rx_att_db = 0.0
        self.tx_att_db = 0.0
        return self.apply()

    def apply_raw_word(self, word):
        self.last_word = write_word(word)
        # Decode for truthful STATUS reporting.  Linux remains authoritative
        # for word construction; this only mirrors the latched command state.
        self.rx_enable = bool(self.last_word & BIT_RX_ENABLE)
        self.tx_enable = bool(self.last_word & BIT_TX_ENABLE)
        self.rx_att_db = decode_rx_attenuation(self.last_word)
        self.tx_att_db = decode_tx_attenuation(self.last_word)
        return self.last_word

    def status_text(self):
        temperature_c, voltage_v = read_temperature()
        temp_text = "NA" if temperature_c is None else "{:.1f}".format(temperature_c)
        voltage_text = "NA" if voltage_v is None else "{:.4f}".format(voltage_v)
        return (
            "STATUS WORD={:07X} RXEN={} TXEN={} "
            "RXATT={:.1f} TXATT={:.1f} RX_ACTIVE_LOW={} "
            "TEMP={} A1V={}"
        ).format(
            self.last_word,
            int(self.rx_enable),
            int(self.tx_enable),
            self.rx_att_db,
            self.tx_att_db,
            int(RX_HIGH_BITS_ACTIVE_LOW),
            temp_text,
            voltage_text,
        )


HELP_TEXT = (
    "COMMANDS: PING | STATUS | SAFE | WORD <hex> | "
    "CONFIG <rxen> <txen> <rxatt_db> <txatt_db> | "
    "RXEN <0|1> | TXEN <0|1> | RXATT <dB> | "
    "TXATT <dB> | APPLY | HELP"
)


def parse_bool(token):
    value = token.strip().lower()
    if value in ("1", "on", "true", "yes"):
        return True
    if value in ("0", "off", "false", "no"):
        return False
    raise ValueError("expected 0/1, on/off, true/false")


def process_command(controller, line):
    parts = line.strip().split()
    if not parts:
        return None

    command = parts[0].upper()

    if command == "PING":
        return "OK PONG"

    if command == "HELP":
        return "OK " + HELP_TEXT

    if command == "STATUS":
        return "OK " + controller.status_text()

    if command == "SAFE":
        word = controller.safe()
        return "OK SAFE WORD={:07X}".format(word)

    if command == "WORD":
        if len(parts) != 2:
            raise ValueError("usage: WORD <hex>")
        token = parts[1].lower()
        if token.startswith("0x"):
            token = token[2:]
        word = int(token, 16)
        applied = controller.apply_raw_word(word)
        return "OK WORD={:07X}".format(applied)

    if command == "CONFIG":
        if len(parts) != 5:
            raise ValueError(
                "usage: CONFIG <rxen> <txen> <rxatt_db> <txatt_db>"
            )
        controller.rx_enable = parse_bool(parts[1])
        controller.tx_enable = parse_bool(parts[2])
        controller.rx_att_db = validate_attenuation(parts[3])
        controller.tx_att_db = validate_attenuation(parts[4])
        word = controller.apply()
        return "OK CONFIG WORD={:07X}".format(word)

    if command == "RXEN":
        if len(parts) != 2:
            raise ValueError("usage: RXEN <0|1>")
        controller.rx_enable = parse_bool(parts[1])
        return "OK RXEN={}".format(int(controller.rx_enable))

    if command == "TXEN":
        if len(parts) != 2:
            raise ValueError("usage: TXEN <0|1>")
        controller.tx_enable = parse_bool(parts[1])
        return "OK TXEN={}".format(int(controller.tx_enable))

    if command == "RXATT":
        if len(parts) != 2:
            raise ValueError("usage: RXATT <dB>")
        controller.rx_att_db = validate_attenuation(parts[1])
        return "OK RXATT={:.1f}".format(controller.rx_att_db)

    if command == "TXATT":
        if len(parts) != 2:
            raise ValueError("usage: TXATT <dB>")
        controller.tx_att_db = validate_attenuation(parts[1])
        return "OK TXATT={:.1f}".format(controller.tx_att_db)

    if command == "APPLY":
        word = controller.apply()
        return "OK APPLY WORD={:07X}".format(word)

    raise ValueError("unknown command: " + command)


def run_server():
    controller = TRMController()

    # Safe power-up state: serial RX and TX path-enable bits both low.
    controller.safe()

    print("TRM READY")
    print(controller.status_text())

    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)

    # Read raw bytes rather than sys.stdin.readline().  On STM32 MicroPython,
    # the USB serial stream may contain a stray control or non-UTF8 byte when a
    # host program opens the port.  readline() attempts UTF-8 decoding inside
    # MicroPython and can raise UnicodeError before our command parser sees it.
    rx_buffer = bytearray()

    while True:
        events = poller.poll(100)
        if not events:
            continue

        try:
            chunk = sys.stdin.buffer.read(1)
        except AttributeError:
            # Fallback for ports/builds without sys.stdin.buffer.
            try:
                import os
                chunk = os.read(0, 1)
            except Exception:
                chunk = b""

        if not chunk:
            time.sleep_ms(1)
            continue

        byte_value = chunk[0]

        # Ignore carriage return.  Newline terminates one command.
        if byte_value == 13:
            continue

        if byte_value == 10:
            if not rx_buffer:
                continue

            line = bytes(rx_buffer).decode("ascii", "ignore").strip()
            rx_buffer = bytearray()

            if not line:
                continue

            try:
                response = process_command(controller, line)
                if response:
                    print(response)
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                print("ERR " + str(exc))
            continue

        # Accept printable ASCII only. This discards USB/terminal control bytes.
        if 32 <= byte_value <= 126:
            if len(rx_buffer) < 128:
                rx_buffer.append(byte_value)
            else:
                rx_buffer = bytearray()
                print("ERR command too long")


if __name__ == "__main__":
    run_server()
