from machine import Pin
import time

# NUCLEO-F446RE Arduino header assignments
# D2 = SEL, D3 = CLK, D4 = DATA, D5 = LOAD
sel = Pin("A10", Pin.OUT)
clk = Pin("B3", Pin.OUT)
data = Pin("B5", Pin.OUT)
load = Pin("B4", Pin.OUT)

# Safe idle state
sel.low()
clk.low()
data.low()
load.low()

# Four-bit test pattern, transmitted in this order
bits = [1, 0, 1, 1]


def send_test_pattern():
    # Enable the serial interface
    sel.high()
    time.sleep_ms(10)

    # Send four bits
    for bit in bits:
        data.value(bit)
        time.sleep_ms(10)

        clk.high()
        time.sleep_ms(10)

        # Falling edge: the TRM samples DATA here
        clk.low()
        time.sleep_ms(10)

    # Return DATA low before latching
    data.low()
    time.sleep_ms(10)

    # Rising edge of LOAD latches the shifted data
    load.high()
    time.sleep_ms(20)

    load.low()
    time.sleep_ms(10)

    # Disable the serial interface
    sel.low()


print("Starting repeating 4-bit TRM test pattern")
print("Pattern: 1, 0, 1, 1")
print("Press Ctrl-C in mpremote to stop")

while True:
    send_test_pattern()
    time.sleep_ms(100)