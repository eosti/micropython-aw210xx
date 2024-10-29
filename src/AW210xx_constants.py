TYPE_CHECKING = True
if TYPE_CHECKING:
    from enum import IntEnum
else:
    IntEnum = object


class AW210xxRegisters(IntEnum):
    """
    Enum for all registers on a AW210xx. Any _BASE registers represent the
    0th register with that name, the number of additional registers is dependent on model
    """

    GCR = 0x00
    BR_BASE = 0x01
    UPDATE = 0x49
    COL_BASE = 0x4A
    GCCR = 0x6E
    PHCR = 0x70
    OSDCR = 0x71
    OSST_BASE = 0x72
    OTCR = 0x77
    SSCR = 0x78
    UVCR = 0x79
    GCR2 = 0x7A
    GCR4 = 0x7C
    VER = 0x7E
    RESET = 0x7F
    WBR = 0x90
    WBG = 0x91
    WBB = 0x92
    PATCFG = 0xA0
    PATGO = 0xA1
    PATT0 = 0xA2
    PATT1 = 0xA3
    PATT2 = 0xA4
    PATT3 = 0xA5
    FADEH = 0xA6
    FADEL = 0xA7
    GCOLR = 0xA8
    GCALG = 0xA9
    GCOLB = 0xAA
    GCFG0 = 0xAB
    GCFG1 = 0xAC


class AW210xxPWMFrequency(IntEnum):
    """
    Enum for PWM frequency selection
    The PWM values are a function of the OSC frequency selection,
        which is actually being changed with these
    """

    FREQ_62K = 0b000
    FREQ_32K = 0x001
    FREQ_4K = 0b010
    FREQ_2K = 0b011
    FREQ_1K = 0b100
    FREQ_500 = 0b101
    FREQ_244 = 0b110
    FREQ_122 = 0b111


class AW210xxOpenShortDetect(IntEnum):
    """Enum for open/short detection mode selection"""

    DISABLE = 0b00
    SHORT_ENABLE = 0b10
    OPEN_ENABLE = 0b11


class AW210xxOpenThreshold(IntEnum):
    """Enum for open-circuit threshold, relative to above GND"""

    VOLTAGE_100MV = 0
    VOLTAGE_200MV = 1


class AW210xxShortThreshold(IntEnum):
    """Enum for short-circuit threshold, relative to below VDD"""

    VOLTAGE_1V = 0
    VOLTAGE_500MV = 1


class AW210xxRisingSlewRate(IntEnum):
    """Enum for the rising time of the LED"""

    RATE_1NS = 0
    RATE_6NS = 1


class AW210xxFallingSlewRate(IntEnum):
    """Enum for the falling time of the LED"""

    RATE_1NS = 0b00
    RATE_3NS = 0b01
    RATE_6NS = 0b10
    RATE_10NS = 0b11


class AW210xxThermalThreshold(IntEnum):
    """Enum for the thermal roll-off threshold"""

    TEMP_140C = 0b00
    TEMP_120C = 0b01
    TEMP_100C = 0b10
    TEMP_90C = 0b11


class AW210xxThermalRollOffPercentage(IntEnum):
    """Enum for the IOUT roll-off thermal protection"""

    PERCENT_100 = 0b00
    PERCENT_75 = 0b01
    PERCENT_55 = 0b10
    PERCENT_30 = 0b11


class AW210xxSpreadSpectrumRange(IntEnum):
    """Enum for the percent frequency deviation during spread spectrum"""

    PERCENT_05 = 0b00
    PERCENT_15 = 0b01
    PERCENT_25 = 0b10
    PERCENT_35 = 0b11


class AW210xxSpreadSpectrumTime(IntEnum):
    """Enum for the cycle time of spread spectrum"""

    TIME_1980US = 0b00
    TIME_1200US = 0b01
    TIME_820US = 0b10
    TIME_660US = 0b11


class AW210xxREXTStatus(IntEnum):
    """Enum for the status of R_ext"""

    NORMAL = 0b00
    OPEN = 0b01
    SHORT_OCP = 0b10
    UNDEFINED = 0b11


class AW210xxOCPThreshold(IntEnum):
    """Enum for the OCP threshold"""

    OCP_85MA = 0
    OCP_55MA = 1
