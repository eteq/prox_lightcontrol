#!/usr/bin/env python3

import time
import json
import smbus
import socket
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


class WizLight:
    CONFIG_MESSAGE = r'{"method":"getSystemConfig","params":{}}'.encode('utf-8')

    def __init__(self, ip, port=38899):
        self.ip = ip
        self.port = port

    @property
    def _socket_address(self):
        return (self.ip, self.port)

    def _send_message(self, msg):
        if isinstance(msg, dict):
            msg = json.dumps(msg).encode('utf-8')

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.2)
        sent = sock.sendto(msg, self._socket_address)
        assert sent > 0, 'no bytes sent!'
        return sock.recvfrom(4096)[0]

    def check_config(self):
        return self._send_message(self.CONFIG_MESSAGE)

    def set_color(self, r, g, b):
        self._send_message({"method":"setPilot",
                            "params":{"r": r, "g": g, "b": b}})

    def set_scene(self, scene):
        self._send_message({"method":"setPilot",  "params":{"sceneId": scene}})

    def set_brightness(self, val):
        self._send_message({"method":"setPilot",
                            "params":{"dimming": int(val*100)}})

    def turn_on(self):
        self._send_message({"method":"setPilot","params":{"state":True}})

    def turn_off(self):
        self._send_message({"method":"setPilot","params":{"state":False}})


def main(light_ip, int_pin=7, which_smbus=1, poll_time_s=0.1, prox_thresh=5000):
    #GPIO.setup(int_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    vcnl = VCNL4010(smbus.SMBus(which_smbus))
    light = WizLight(light_ip)
    colors = [(0, 255, 0), (255, 100, 0), (255, 0, 0), None]

    # pooling approach instead of
    rate = vcnl.periodic_prox(2/poll_time_s)
    if rate < 2/poll_time_s:
        print("Warning: real rate below 1/poll_time:",  rate, 1/poll_time_s)

    last_prox_high = None
    color_idx = 0
    while True:
        prox = vcnl.read_prox()
        if prox > prox_thresh:
            if last_prox_high is False:
                print('transitioned high')
                color = colors[color_idx % len(colors)]
                if color is None:
                    light.turn_off()
                else:
                    light.turn_on()
                    light.set_color(*color)
                color_idx += 1
            last_prox_high = True
        else:
            if last_prox_high is True:
                print('transitioned low')
            last_prox_high = False

        time.sleep(poll_time_s)


if __name__ == '__main__':
    main()