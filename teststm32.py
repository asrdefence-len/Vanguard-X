from machine import Pin
import time

# Vanguard X / Shinewave TRM control pins
SEL  = Pin("D2", Pin.OUT, value=0)
CLK  = Pin("D3", Pin.OUT, value=0)
DATA = Pin("D4", Pin.OUT, value=0)
LOAD = Pin("D5", Pin.OUT, value=0)

# Slow timing for CRO inspection.
# 50 us high + 50 us low = approx 10 kHz clock.
HALF_CLOCK_US = 50

# Delay between complete 28-bit transactions.
REPEAT_MS = 500


def send_trm_word(word):
    """
    Send one 28-bit TRM control word.
    Transmission order is D0 first through D27.
    DATA is stable before the falling edge of CLK.
    """

    # Known idle state
    CLK.value(0)
    LOAD.value(0)
    DATA.value(0)

    # Enable serial interface
    SEL.value(1)
    time.sleep_us(100)

    for bit_number in range(28):

        bit = (word >> bit_number) & 0x1

        # Set DATA before the clock falling edge
        DATA.value(bit)

        # Clock high
        CLK.value(1)
        time.sleep_us(HALF_CLOCK_US)

        # Falling edge: TRM samples DATA here
        CLK.value(0)
        time.sleep_us(HALF_CLOCK_US)

    # Keep a little separation after final clock
    time.sleep_us(100)

    # LOAD rising edge latches the 28-bit shift register
    LOAD.value(1)
    time.sleep_us(300)
    LOAD.value(0)

    time.sleep_us(100)

    # Disable serial interface
    SEL.value(0)

    DATA.value(0)


def make_test_word():
    """
    Known initial test:
      D0-D1 = 01
      D2     = 1  RX enabled
      D3-D8  = 000000
      D9-D14 = 000000  RX attenuation = 0 dB
      D15-D20= 000000  TX attenuation = 0 dB
      D21-D26= 000000
      D27    = 1  TX enabled
    """

    word = 0

    # D0-D1 must be 01.
    # Since D0 is the low bit:
    word |= (1 << 0)

    # RX normal / enabled
    word |= (1 << 2)

    # TX normal / enabled
    word |= (1 << 27)

    return word


TEST_WORD = make_test_word()

print("TRM CRO test")
print("28-bit word: 0x{:07X}".format(TEST_WORD))

print("Transmission order D0 -> D27:")
for i in range(28):
    print("D{:02d} = {}".format(i, (TEST_WORD >> i) & 1))

print("Sending repeatedly every {} ms".format(REPEAT_MS))


while True:
    send_trm_word(TEST_WORD)
    time.sleep_ms(REPEAT_MS)