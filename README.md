# micropython-aw210xx
> Driver for Awinic's AW210xx line of 8-bit RGB drivers

## Getting Started
This package is installable with `mip`: 

```python
mpremote mip install github:eosti/micropython-aw210xx
```

This driver is compatible with any of the AW210xx devices, i.e. the 09, 12, 18, 24, and 36-channel versions. 
While TI's LP50xx is pin-compatible with these devices, the I2C interface sadly is not. 
All this driver requires is the I2C object, as well as a bit of information about your specific configuration:

```python
from machine import I2C, Pin
from aw210xx import AW210xx

# To use default interface on RPi Pico:
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)

# Using the AW21024 with address pins pulled low and 3.65kOhm external resistor
leds = AW210xx(i2c, addr=0x30, model="AW21024", r_ext=3650)
```

This library implements most of the features available on the chip, including basic LED operation, PWM adjustments, protections, and open/short checks. 
Some examples of these features have been put in `/examples` for your viewing convenience. 

Note that the datasheet labels the pins LED1 through LEDn, but registers REG0 through REGn-1. 
This driver will zero-index all channel numbers to match the register values, so don't be confused when you want to turn on LED1 and writing to channel 1 turns on LED2 instead. 

### Help, it's not working! 
I suggest checking out the datasheet first, but here's a TL;DR of what you need to do to get anything outputting: 

There are four factors that go into output brightness: global current control, white balance, channel current control, and channel brightness. 
All four need to be non-zero in order to get anything output -- white balance is already at maximum on reset, but the other three are zeroed. 
Additionally, the chip needs to be enabled, and the chip need to be told to update after brightness is set. 
Here's what that looks like: 

```python
leds.chip_enabled(True)
leds.global_current(255)    # Current is now limited by R_ext
leds.col(0, 255)            # Channel 0, no current limit
leds.br(0, 255)             # Channel 0, all the bright
leds.update()               # Let there be light!
```

## Organization
Since there seems to be no major consensus on MicroPython driver organization, I looked at a number of existing libraries and tried to match them somewhat. 
Importantly: no properties :(. 
This comes straight from [MicroPython's driver guidelines](https://github.com/micropython/micropython/wiki/Driver-API-Design-Guidelines) for optimization reasons. 
This means that most functions act as both a setter and a getter: don't pass an argument to use it as a getter, pass an argument to use it as a setter. 
It makes me uncomfy but makes MicroPython happy so who am I to disagree?

Most magic numbers such as register addresses and bitfields are stored in enums. 
MicroPython doesn't support enums, so unless you're typechecking during development, they act as normal objects. 
For basic operations you will not need to interface with any enums, but for more advanced functionality you may need to, in which case import the appropriate enum before using. 

For organizational convenience, the `AW210xx` object is structured as a mixin, using multiple-inheritance from subclasses that implement feature-specific methods. 
I hope this makes it easier to parse!

## Contributing
If you run into an issue or want to see a feature added, feel free to drop me an issue, or better yet, a PR! 

For PRs, please do some basic linting before opening the PR: `isort . && black . && pylint .`. 
Typechecking can be enabled by setting `TYPE_CHECKING = True` at the top of the file, and then running `mypy .`.
I'm not too strict with `pylint` issues, just clean up the obviously non-MicroPython-related ones/don't make any new ones please!
