from typing import Optional
from machine import I2C
from AW210xx_constants import *

class AW210xxBase:
    """Base class for AW210xx with init and R/W functions"""

    def __init__(
        self, i2c: I2C, addr: int = 0x30, model: str = "AW21024", r_ext: float = 4000
    ):
        self.i2c = i2c
        self.addr = addr
        self.r_ext = 4000
        self.model = model

        if model == "AW21009":
            self.MAX_CHANNEL = 8
        elif model == "AW21012":
            self.MAX_CHANNEL = 11
        elif model == "AW21018":
            self.MAX_CHANNEL = 17
        elif model == "AW21024":
            self.MAX_CHANNEL = 23
        elif model == "AW21036":
            self.MAX_CHANNEL = 35
        else:
            raise ValueError("Unknown AW210xx model")

    def _read_register(self, register: int) -> int:
        return int(self.i2c.readfrom_mem(self.addr, register, 1)[0])

    def _write_register(self, register: int, val: int) -> None:
        self.i2c.writeto_mem(self.addr, register, val)

    def _read_bits(self, register: int, pos: int, numbits: int) -> int:
        """Read from certain bits in a register"""
        if pos + numbits > 8:
            raise ValueError("Cannot read more than 8 bits from a byte")
        mask = (2**numbits) - 1 << pos
        reg = self.i2c.readfrom_mem(self.addr, register, 1)
        return (reg & mask) >> pos

    def _write_bits(self, register: int, pos: int, numbits: int, state: int) -> None:
        """Write to certain bits in a register"""
        if pos + numbits > 8:
            raise ValueError("Cannot write more than 8 bits to a byte")
        mask = (2**numbits) - 1 << pos
        reg = self.i2c.readfrom_mem(self.addr, register, 1)
        retval = (reg[0] & ~mask) | ((state << pos) & mask)
        self.i2c.writeto_mem(self.addr, register, retval)


class AW210xxBasic(AW210xxBase):
    """Collection of basic functions of the chip"""

    def get_version(self) -> int:
        """Get the chip version"""
        return self._read_register(AW210xxRegisters.VER)

    def get_id(self) -> int:
        """Get the chip ID = 0x18"""
        return self._read_register(AW210xxRegisters.RESET)

    def reset(self) -> None:
        """Reset registers to default state"""
        self._write_register(AW210xxRegisters.RESET, 0x00)

    def chip_enable(self, state: Optional[bool] = None) -> bool:
        """Set or get chip enable state"""
        if state is None:
            return bool(self._read_bits(AW210xxRegisters.GCR, 0, 1))
        self._write_bits(AW210xxRegisters.GCR, 0, 1, state)
        return state

    def update(self):
        """Update output to match current state of BR registers"""
        self._write_register(AW210xxRegisters.UPDATE, 0x00)


class AW210xxLED(AW210xxBase):
    """Collection of functions changing the control of LEDs"""

    def rgb_mode(self, state: Optional[bool] = None) -> bool:
        """Set or get the state of RGB mode (every 3 channels share common brightness)"""
        if state is None:
            return bool(self._read_bits(AW210xxRegisters.GCR2, 0, 1))
        self._write_bits(AW210xxRegisters.GCR2, 0, 1, state)
        return state

    def white_scaling(self, wb: Optional[tuple] = None) -> tuple:
        """Set or get the output white balance as a tuple (R, G, B)"""
        if wb is None:
            r_wb = self._read_register(AW210xxRegisters.WBR)
            g_wb = self._read_register(AW210xxRegisters.WBG)
            b_wb = self._read_register(AW210xxRegisters.WBB)
            return (r_wb, g_wb, b_wb)

        self._write_register(AW210xxRegisters.WBR, wb[0])
        self._write_register(AW210xxRegisters.WBG, wb[1])
        self._write_register(AW210xxRegisters.WBB, wb[2])
        return wb

    def rise_slewrate(
        self, rate: Optional[AW210xxRisingSlewRate] = None
    ) -> AW210xxRisingSlewRate:
        """Set or get the LED rising edge slew rate"""
        if rate is None:
            return AW210xxRisingSlewRate(self._read_bits(AW210xxRegisters.GCR4, 2, 1))
        self._write_bits(AW210xxRegisters.GCR4, 2, 1, rate)
        return rate

    def fall_slewrate(
        self, rate: Optional[AW210xxFallingSlewRate] = None
    ) -> AW210xxFallingSlewRate:
        """Set or get the LED falling edge slew rate"""
        if rate is None:
            return AW210xxFallingSlewRate(self._read_bits(AW210xxRegisters.GCR4, 0, 2))
        self._write_bits(AW210xxRegisters.GCR4, 0, 2, rate)
        return rate

    def br(self, channel: int, val: Optional[int]) -> int:
        """Set or get the PWM brightness for a certain channel"""
        if channel < 0 or channel > self.MAX_CHANNEL:
            raise ValueError(f"Channel number must be between 0 and {self.MAX_CHANNEL}")

        if val is None:
            return self._read_register(AW210xxRegisters.BR_BASE + channel)

        if val < 0 or val > 255:
            raise ValueError(
                "Channel brightness scalar must be between 0 and 255 inclusive"
            )

        self._write_register(AW210xxRegisters.BR_BASE + channel, val)
        return val


class AW210xxOpenShortChecking(AW210xxBase):
    """Collection of functions related to the open/short circuit detection"""

    def open_threshold(
        self, thresh: Optional[AW210xxOpenThreshold]
    ) -> AW210xxOpenThreshold:
        """Set or get the open circuit threshold"""
        if thresh is None:
            return AW210xxOpenThreshold(self._read_bits(AW210xxRegisters.OSDCR, 3, 1))
        self._write_bits(AW210xxRegisters.OSDCR, 3, 1, thresh)
        return thresh

    def short_threshold(
        self, thresh: Optional[AW210xxShortThreshold]
    ) -> AW210xxShortThreshold:
        """Set or get the short circuit threshold"""
        if thresh is None:
            return AW210xxShortThreshold(self._read_bits(AW210xxRegisters.OSDCR, 2, 1))
        self._write_bits(AW210xxRegisters.OSDCR, 2, 1, thresh)
        return thresh

    def open_short_detect(
        self, thresh: Optional[AW210xxOpenShortDetect]
    ) -> AW210xxOpenShortDetect:
        """Set or get the short/open detect state"""
        if thresh is None:
            return AW210xxOpenShortDetect(self._read_bits(AW210xxRegisters.OSDCR, 0, 2))
        self._write_bits(AW210xxRegisters.OSDCR, 0, 2, thresh)
        return thresh

    def get_open_short_status(self) -> int:
        """Reads open/short status register and returns concatenated bitfield"""
        # TODO: do smaller chips just return zero for these fields?
        state = 0
        for i in range(5):
            state = state | self._read_register(AW210xxRegisters.OSST_BASE + i) << (
                8 * i
            )
        return state


class AW210xxPWM(AW210xxBase):
    """Collection of functions relating to PWM configuration"""

    def pwm_sel(
        self, freq: Optional[AW210xxPWMFrequency] = None
    ) -> AW210xxPWMFrequency:
        """Set or get output PWM frequency"""
        if freq is None:
            return AW210xxPWMFrequency(self._read_bits(AW210xxRegisters.GCR, 4, 3))
        self._write_bits(AW210xxRegisters.GCR, 4, 3, freq)
        return freq

    def pwm_phase_delay(self, state: Optional[bool] = None) -> bool:
        """Sets or gets the PWM phase delay state"""
        if state is None:
            return bool(self._read_bits(AW210xxRegisters.PHCR, 7, 1))
        self._write_bits(AW210xxRegisters.PHCR, 7, 1, state)
        return state

    def set_pwm_phase_invert(self, state: bool) -> None:
        """
        Sets the PWM phase inversion state. I'm lazy so this will
        toggle all inversions at once, i.e. all even channels will become inverted.
        """
        if state is True:
            state_val = 0b11111111
        else:
            state_val = 0
        # TODO: is writing to all the fields ok on smaller chips?
        self._write_bits(AW210xxRegisters.PHCR, 1, 6, state_val)

    def spread_spectrum_enable(self, state: Optional[bool] = None) -> bool:
        """Sets or gets if spread spectrum is enabled"""
        if state is None:
            return bool(self._read_bits(AW210xxRegisters.SSCR, 4, 1))
        self._write_bits(AW210xxRegisters.SSCR, 4, 1, state)
        return state

    def spread_spectrum_range(
        self, state: Optional[AW210xxSpreadSpectrumRange] = None
    ) -> AW210xxSpreadSpectrumRange:
        """Sets or gets the frequency range that the frequency will spread across"""
        if state is None:
            return AW210xxSpreadSpectrumRange(
                self._read_bits(AW210xxRegisters.SSCR, 2, 2)
            )
        self._write_bits(AW210xxRegisters.SSCR, 2, 2, state)
        return state

    def spread_spectrum_cycletime(
        self, state: Optional[AW210xxSpreadSpectrumTime] = None
    ) -> AW210xxSpreadSpectrumTime:
        """Sets or gets the cycle time of spread spectrum"""
        if state is None:
            return AW210xxSpreadSpectrumTime(
                self._read_bits(AW210xxRegisters.SSCR, 0, 2)
            )
        self._write_bits(AW210xxRegisters.SSCR, 0, 2, state)
        return state

    def fixed_on(self, state: bool) -> None:
        """Sets all LEDs to max BRightness"""
        if state:
            write_var = 0b111
        else:
            write_var = 0b000
        self._write_bits(AW210xxRegisters.SSCR, 5, 3, write_var)


class AW210xxPower(AW210xxBase):
    """Collection of functions related to power and power protection"""

    def power_saving(self, state: Optional[bool] = None) -> bool:
        """Set or get power saving state"""
        if state is None:
            return bool(self._read_bits(AW210xxRegisters.GCR, 7, 1))
        self._write_bits(AW210xxRegisters.GCR, 7, 1, state)
        return state

    def global_current(self, val: Optional[int] = None) -> int:
        """Set or get the global current scalar (0-255)"""
        if val is None:
            return self._read_register(AW210xxRegisters.GCCR)

        if val < 0 or val > 255:
            raise ValueError(
                "Global current scalar must be between 0 and 255 inclusive"
            )

        self._write_register(AW210xxRegisters.GCCR, val)
        return val

    def col(self, channel: int, val: Optional[int]) -> int:
        """Set or get the constant current parameter for a certain channel"""
        if channel < 0 or channel > self.MAX_CHANNEL:
            raise ValueError(f"Channel number must be between 0 and {self.MAX_CHANNEL}")

        if val is None:
            return self._read_register(AW210xxRegisters.COL_BASE + channel)

        if val < 0 or val > 255:
            raise ValueError(
                "Channel current scalar must be between 0 and 255 inclusive"
            )

        self._write_register(AW210xxRegisters.COL_BASE + channel, val)
        return val

    def uvlo_detected(self) -> bool:
        """Reads if an UVLO has been detected"""
        return bool(self._read_bits(AW210xxRegisters.UVCR, 5, 1))

    def por_status(self) -> bool:
        """Reads power on reset state. Note: cleared on read"""
        return bool(self._read_bits(AW210xxRegisters.UVCR, 4, 1))

    def rext_status(self) -> AW210xxREXTStatus:
        """Reads state of R_ext"""
        return AW210xxREXTStatus(self._read_bits(AW210xxRegisters.UVCR, 7, 2))

    def ocp_threshold(
        self, state: Optional[AW210xxOCPThreshold] = None
    ) -> AW210xxOCPThreshold:
        """Set or get the overcurrent protection threshold"""
        if state is None:
            return AW210xxOCPThreshold(self._read_bits(AW210xxRegisters.UVCR, 3, 1))
        self._write_bits(AW210xxRegisters.UVCR, 3, 1, state)
        return state

    def ocp_enable(self, state: Optional[bool] = None) -> bool:
        """Set or get the overcurrent protection state"""
        if state is None:
            return bool(self._read_bits(AW210xxRegisters.UVCR, 2, 1))
        self._write_bits(AW210xxRegisters.UVCR, 2, 1, state)
        return state

    def uvlo_disable_protection(self, state: Optional[bool] = None) -> bool:
        """
        Set or get if the UVLO protection (i.e. action) is disabled
        When tripped, the UVLO protection disables the chip.
        To function, the UVLO detection must also be enabled.
        """
        if state is None:
            return bool(self._read_bits(AW210xxRegisters.UVCR, 1, 1))
        self._write_bits(AW210xxRegisters.UVCR, 1, 1, state)
        return state

    def uvlo_disable_detection(self, state: Optional[bool] = None) -> bool:
        """
        Set or get if the UVLO detection is disabled
        When tripped, the UVLO detection will set undervoltage bit, see `uvlo_detected()`
        """
        if state is None:
            return bool(self._read_bits(AW210xxRegisters.UVCR, 0, 1))
        self._write_bits(AW210xxRegisters.UVCR, 0, 1, state)
        return state


class AW210xxThermalProtection(AW210xxBase):
    """Collection of functions for configuring overtemperature protection"""

    def ot_detected(self) -> bool:
        """Reads if there is an overtemperature event"""
        return bool(self._read_bits(AW210xxRegisters.OTCR, 4, 1))

    def ot_disable_protection(self, state: Optional[bool] = None) -> bool:
        """
        Set or get if the overtemperature protection (i.e. action) is disabled
        When tripped, the OT protection disables the chip.
        The setpoint of overtemperature is 150C.
        To function, the OT detection must also be enabled.
        """
        if state is None:
            return bool(self._read_bits(AW210xxRegisters.OTCR, 3, 1))
        self._write_bits(AW210xxRegisters.OTCR, 3, 1, state)
        return state

    def ot_disable_detection(self, state: Optional[bool] = None) -> bool:
        """
        Set or get if the overtemperature protection (i.e. action) is disabled
        When tripped, the OT detection will set OT bit, see `ot_detected()`
        The setpoint of overtemperature is 150C.
        """
        if state is None:
            return bool(self._read_bits(AW210xxRegisters.OTCR, 2, 1))
        self._write_bits(AW210xxRegisters.OTCR, 2, 1, state)
        return state

    def thermal_rolloff(self) -> bool:
        """Reads if there is thermal roll-off occurring"""
        return bool(self._read_bits(AW210xxRegisters.OTCR, 5, 1))

    def thermal_rolloff_threshold(
        self, temp: Optional[AW210xxThermalThreshold]
    ) -> AW210xxThermalThreshold:
        """Set or get the thermal threshold for current rolloff"""
        if temp is None:
            return AW210xxThermalThreshold(self._read_bits(AW210xxRegisters.OTCR, 0, 2))
        self._write_bits(AW210xxRegisters.OTCR, 0, 2, temp)
        return temp

    def thermal_rolloff_percent(
        self, temp: Optional[AW210xxThermalRollOffPercentage]
    ) -> AW210xxThermalRollOffPercentage:
        """Set or get the percentage of Iout to roll-off once the threshold is exceeded"""
        if temp is None:
            return AW210xxThermalRollOffPercentage(
                self._read_bits(AW210xxRegisters.OTCR, 6, 2)
            )
        self._write_bits(AW210xxRegisters.OTCR, 6, 2, temp)
        return temp


class AW210xxPattern(AW210xxBase):
    """Collection of functions related to the built-in effects engine"""

    pass


class AW210xxGroups(AW210xxBase):
    """Collection of functions related to grouping channels"""

    pass


# This is called a mixin, by the way
class AW210xx(
    AW210xxBasic,
    AW210xxLED,
    AW210xxOpenShortChecking,
    AW210xxPWM,
    AW210xxPower,
    AW210xxThermalProtection,
    AW210xxPattern,
    AW210xxGroups,
):
    """Top-level class of AW210xx with all functionality"""

    pass
