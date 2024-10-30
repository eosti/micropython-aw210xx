from machine import I2C, Pin
from utime import sleep

from aw210xx import AW210xx


def step_through(leds: AW210xx) -> None:
    for i in range(leds.NUM_CHANNELS):
        leds.br(i, 255)
        leds.update()
        sleep(0.5)
        leds.br(i, 0)


def main():
    i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
    leds = AW210xx(i2c, r_ext=3650, model="AW21024")

    leds.reset()
    leds.chip_enabled(True)
    print(f"Driver ID: 0x{leds.get_id():02X}, Driver Version: 0x{leds.get_version():02X}")
    leds.global_current(255)

    print(f"Maximum current per channel is {leds.max_global_current() * 1000:.3f}mA")

    for i in range(leds.NUM_CHANNELS):
        leds.col(i, 255)

    while True:
        step_through(leds)


if __name__ == "__main__":
    main()
