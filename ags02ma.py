# Author: Tom Øyvind Hogstad - 2023
#
# This module is a standalone Micropython library for the AGS02MA TVOC / Gas sensor.
# It borrows from the Adafruit AGS02MA Circuit Python library. Original
# Copyright notices and documentation are reproduced below.
#
# The standalone sensor need pull-up resistors on SDA and SCL.
# The sensors values need some time to stabilize ¨~30+ minutes
#
# Github Adafruit: https://github.com/adafruit/Adafruit_CircuitPython_AGS02MA
# Github Arduino library: https://github.com/RobTillaart/AGS02MA
# Product page and datasheet: http://www.aosong.com/m/en/products-33.html


"""
`adafruit_ags02ma`
================================================================================

AGS02MA TVOC / Gas sensor


* Author(s): ladyada

Implementation Notes
--------------------

**Hardware:**

* `AGS02MA Gas Sensor <http://www.adafruit.com/products/5593>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

The MIT License (MIT)

Copyright (c) 2022 ladyada for Adafruit Industries

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import time
import struct
from micropython import const
#from adafruit_bus_device import i2c_device

#__version__ = "0.0.0+auto.0"
#__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_AGS02MA.git"

AGS02MA_I2CADDR_DEFAULT: int = const(0x1A)  # Default I2C address
_AGS02MA_TVOCSTAT_REG = const(0x00)
_AGS02MA_VERSION_REG = const(0x11)
_AGS02MA_GASRES_REG = const(0x20)
_AGS02MA_SETADDR_REG = const(0x21)
_AGS02MA_CRC8_INIT = const(0xFF)
_AGS02MA_CRC8_POLYNOMIAL = const(0x31)


def _generate_crc(data):
    """8-bit CRC algorithm for checking data

    :param bytearray data: The data to generate a CRC for
    """

    crc = _AGS02MA_CRC8_INIT
    # calculates 8-Bit checksum with given polynomial
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ _AGS02MA_CRC8_POLYNOMIAL
            else:
                crc <<= 1
        crc &= 0xFF
    return crc & 0xFF


class AGS02MA:
    """Driver for the AGS02MA air quality sensor

    :param I2C the AGS02MA is connected to
    :param int address: The I2C device address. Defaults to :const:`0x1A`
    """

    def __init__(self, i2c, address=AGS02MA_I2CADDR_DEFAULT):
        self.i2c = i2c
        self.address = address

        self._buf = bytearray(5)
        self._addr = bytearray(1)
        try:
            fwv = self.firmware_version()
            
        except RuntimeError as exc:  # a CRC error or something!
            raise RuntimeError("Failed to find AGS02MA - check your wiring!") from exc

    def firmware_version(self):
        """Return 24-bit value which contains the firmware version"""
        return self._read_reg(_AGS02MA_VERSION_REG, 30)
 
    @property
    def gas_resistance(self):
        """The resistance of the MEMS gas sensor"""
        return self._read_reg(_AGS02MA_GASRES_REG, 1500) * 100  # in 0.1Kohm

    @property
    def TVOC(self):
        """The calculated Total Volatile Organic Compound measurement, in ppb"""
        val = self._read_reg(_AGS02MA_TVOCSTAT_REG, 1500)
        status = val >> 24
        # print(hex(status))
        if status & 0x1:
            raise RuntimeError("Sensor still preheating")
        return val & 0xFFFFFF

    def set_address(self, new_addr):
        # Not implemented. Use the Arduino library to set another address?
        """Set the address for the I2C interface, from 0x0 to 0x7F

        :param int new_addr: THe new address
        """
        """
        _buf = bytearray(
            [
                _AGS02MA_SETADDR_REG,
                new_addr,
                ~new_addr & 0xFF,
                new_addr,
                ~new_addr & 0xFF,
                0,
            ]
        )
        _buf[5] = _generate_crc(_buf[1:5])
        with self.i2c_device as i2c:
            i2c.write(_buf)
         """
 
    def _read_reg(self, addr, delayms):
        """Read a register

        :param int addr: The address to read
        :param int delayms: The delay between writes and reads, in milliseconds
        """
        self._addr[0] = addr
        self.i2c.writeto(self.address, self._addr)
        time.sleep(delayms / 1000)
        data = self.i2c.readfrom(self.address, len(self._buf))
        if _generate_crc(data) != 0:
            raise RuntimeError("CRC check failed")
        val, crc = struct.unpack(">IB", data)        
        return val
    

if __name__ == '__main__':
    # Test code
    import machine
    i2c = machine.SoftI2C(scl=machine.Pin(22), sda=machine.Pin(21), freq=20_000)
    devices = i2c.scan()
    print('Devices:', devices)

    ags = AGS02MA(i2c)
    print('Firmware version:', ags.firmware_version()&0x0000FF)
    while True:
        print('Gas resistance:', ags.gas_resistance)
        print('TVOC:', ags.TVOC)
        time.sleep(10)




# i2c adresse 0x1A = 20
    
    

