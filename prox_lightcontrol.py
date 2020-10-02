#!/usr/bin/env python3

import smbus
from RPi import GPIO

GPIO.setmode(GPIO.BOARD)

class VCNL4010:
    I2C_ADDR = 0x13

    def __init__(self, smbus):
        self.bus = smbus

    def read(self, addr, nbytes=None):
        if nbytes is None:
            return self.bus.read_i2c_block_data(self.I2C_ADDR, addr, 1)[0]
        else:
            return self.bus.read_i2c_block_data(self.I2C_ADDR, addr, nbytes)

    def write(self, addr, towrite_bytes):
        if isinstance(towrite_bytes, int):
            towrite_bytes = [towrite_bytes]
        self.bus.write_i2c_block_data(self.I2C_ADDR, addr, towrite_bytes)

    def set_led_current(self, ma):
        if ma > 200:
            raise ValueError('current cannot go above 200')
        self.write(0x83, ma // 10)

    def read_prox(self):
        h, l = self.read(0x87, 2)
        return l + (h << 8)

    def read_light(self):
        h, l = self.read(0x85, 2)
        return l + (h << 8)

    def measure_prox(self):
        """
        Trigger a single proximity measurement. This also switches to on-demand
        mode for both light and proximity.
        """
        self.write(0x80, 0b00001000)

    def measure_light(self):
        """
        Trigger a single light measurement. This also switches to on-demand
        mode for both light and proximity.
        """
        self.write(0x80, 0b00010000)

    def measure_both(self):
        """
        Trigger a single light and prox measurement. This also switches to
        on-demand mode for both light and proximity.
        """
        self.write(0x80, 0b00011000)

    LIGHT_RATE_OPTIONS = {0b000: 1,
                    0b001: 2,
                    0b010: 3,
                    0b011: 4,
                    0b100: 5,
                    0b101: 6,
                    0b110: 8,
                    0b111: 10}
    def periodic_light(self, measpersec=None):
        """
        Set light to periodic mode.
        """
        cmd_reg = self.read(0x80)

        if not cmd_reg & 0b1:
            self.write(0x80, 0b00000001 | (cmd_reg & 0b11111000))

        self.write(0x80, 0b00000101 | cmd_reg)

        if measpersec is not None:
            diffs = [r-measpersec for r in self.LIGHT_RATE_OPTIONS.values()]
            idx = min(range(len(diffs)), key=lambda k: abs(diffs[k]))
            rate_byte = list(self.LIGHT_RATE_OPTIONS.keys())[idx]
            amb_reg = self.read(0x84)
            self.write(0x84, (rate_byte << 4) | (amb_reg & 0b10001111))
            return self.LIGHT_RATE_OPTIONS[rate_byte]


    PROX_RATE_OPTIONS = {0b000: 1.95,
                    0b001: 3.90625,
                    0b010: 7.8125,
                    0b011: 16.625,
                    0b100: 31.25,
                    0b101: 62.5,
                    0b110: 125,
                    0b111: 250}
    def periodic_prox(self, measpersec=None):
        """
        Set proximity to periodic mode.
        """
        cmd_reg = self.read(0x80)

        if not cmd_reg & 0b1:
            self.write(0x80, 0b00000001 | (cmd_reg & 0b11111000))

        self.write(0x80, 0b00000011 | cmd_reg)

        if measpersec is not None:
            diffs = [r-measpersec for r in self.PROX_RATE_OPTIONS.values()]
            idx = min(range(len(diffs)), key=lambda k: abs(diffs[k]))
            rate_byte = list(self.PROX_RATE_OPTIONS.keys())[idx]
            self.write(0x82, rate_byte)
            return self.PROX_RATE_OPTIONS[rate_byte]


def main(int_pin=7):

    sm = smbus.SMBus(1)

    vcnl = VCNL4010(sm)

    GPIO.setup(int_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)


if __name__ == '__main__':
    main()