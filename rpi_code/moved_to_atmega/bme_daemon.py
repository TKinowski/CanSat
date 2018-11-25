#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Defines the bmeReader class that runs as a separate thread and provides data from the BME280
"""

import datetime
from time import sleep
import threading
import smbus2
import sys

class reader(object):
    """
    Wraps a I2C SMBus instance to provide methods for reading
    signed/unsigned bytes and 16-bit words
    """
    def __init__(self, bus, address):
        self._bus = bus
        self._address = address

    def unsigned_short(self, register):
        return self._bus.read_word_data(self._address, register) & 0xffff

    def signed_short(self, register):
        word = self.unsigned_short(register)
        return word if word < 0x8000 else word - 0x10000

    def unsigned_byte(self, register):
        return self._bus.read_byte_data(self._address, register) & 0xff

    def signed_byte(self, register):
        byte = self.unsigned_byte(register) & 0xff
        return byte if byte < 0x80 else byte - 0x100

class oversampling:
    x1 =  1
    x2 =  2
    x4 =  3
    x8 =  4
    x16 = 5


class bmeReader(threading.Thread):

    def __init__(self, threadID, name, address=0x76):
        threading.Thread.__init__(self)
        self.lock = threading.RLock()
        self.threadID = threadID
        self.address = address
        self.name = name
        self.bus = smbus2.SMBus(1)
        self.__cache_calibration_params = self.load_calibration_params()
        self.temperature = 0.0
        self.pressure = 0.0
        self.humidity = 0.0
        self.altitude = 0.0
        self.sealevel_pa=101325.0
        self._end = False

    def endThread(self):
        self._end = True

    def run(self):
        while not self._end:
            try:
                data = self.sample(self.__cache_calibration_params)
                with self.lock:
                    self.temperature = data.temperature
                    self.pressure = data.pressure
                    self.humidity = data.humidity
                    self.altitude = self.calc_altitude(self.pressure)
            finally:
                pass
    

    def load_calibration_params(self):
        """
        The BME280 output consists of the ADC output values. However, each sensing
        element behaves differently. Therefore, the actual pressure and temperature
        must be calculated using a set of calibration parameters.

        The calibration parameters are subsequently used to with some compensation
        formula to perform temperature readout in degC, humidity in % and pressure
        in hPA.
        """

        self.read = reader(self.bus, self.address)
        self.compensation_params = params()

        # Temperature trimming params
        self.compensation_params.dig_T1 = self.read.unsigned_short(0x88)
        self.compensation_params.dig_T2 = self.read.signed_short(0x8A)
        self.compensation_params.dig_T3 = self.read.signed_short(0x8C)

        # Pressure trimming params
        self.compensation_params.dig_P1 = self.read.unsigned_short(0x8E)
        self.compensation_params.dig_P2 = self.read.signed_short(0x90)
        self.compensation_params.dig_P3 = self.read.signed_short(0x92)
        self.compensation_params.dig_P4 = self.read.signed_short(0x94)
        self.compensation_params.dig_P5 = self.read.signed_short(0x96)
        self.compensation_params.dig_P6 = self.read.signed_short(0x98)
        self.compensation_params.dig_P7 = self.read.signed_short(0x9A)
        self.compensation_params.dig_P8 = self.read.signed_short(0x9C)
        self.compensation_params.dig_P9 = self.read.signed_short(0x9E)

        # Humidity trimming params
        self.compensation_params.dig_H1 = self.read.unsigned_byte(0xA1)
        self.compensation_params.dig_H2 = self.read.signed_short(0xE1)
        self.compensation_params.dig_H3 = self.read.signed_byte(0xE3)

        self.e4 = self.read.signed_byte(0xE4)
        self.e5 = self.read.signed_byte(0xE5)
        self.e6 = self.read.signed_byte(0xE6)

        self.compensation_params.dig_H4 = self.e4 << 4 | self.e5 & 0x0F
        self.compensation_params.dig_H5 = ((self.e5 >> 4) & 0x0F) | (self.e6 << 4)
        self.compensation_params.dig_H6 = self.read.signed_byte(0xE7)

        return self.compensation_params


    
    def calc_altitude(self, pressure):
        """Calculates the altitude in meters."""
        altitude = 44330.0 * (1.0 - pow(pressure / self.sealevel_pa, (1.0/5.255)))
        return altitude


    def __calc_delay(self, t_oversampling, h_oversampling, p_oversampling):
        t_delay = 0.000575 + 0.0023 * (1 << t_oversampling)
        h_delay = 0.000575 + 0.0023 * (1 << h_oversampling)
        p_delay = 0.001250 + 0.0023 * (1 << p_oversampling)
        return t_delay + h_delay + p_delay


    def sample(self, compensation_params, sampling=oversampling.x8):
        """
        Primes the sensor for reading (defaut: x1 oversampling), pauses for a set
        amount of time so that the reading stabilizes, and then returns a
        compensated reading object with the following attributes:
            * timestamp (Python's datetime object) when reading was taken.
            * temperature, in degrees Celcius.
            * humidity, in % relative humidity.
            * pressure, in hPa.
        """

        self.mode = 1  # forced
        self.t_oversampling = sampling or oversampling.x1
        self.h_oversampling = sampling or oversampling.x1
        self.p_oversampling = sampling or oversampling.x1

        self.bus.write_byte_data(self.address, 0xF2, self.h_oversampling)  # ctrl_hum
        self.bus.write_byte_data(self.address, 0xF4, self.t_oversampling << 5 | self.p_oversampling << 2 | self.mode)  # ctrl
        self.delay = self.__calc_delay(self.t_oversampling, self.h_oversampling, self.p_oversampling)
        sleep(self.delay)

        block = self.bus.read_i2c_block_data(self.address, 0xF7, 8)
        self.raw_data = uncompensated_readings(block)
        return compensated_readings(self.raw_data, self.compensation_params)


class uncompensated_readings(object):

    def __init__(self, block):
        self._block = block
        self.pressure = (block[0] << 16 | block[1] << 8 | block[2]) >> 4
        self.temperature = (block[3] << 16 | block[4] << 8 | block[5]) >> 4
        self.humidity = block[6] << 8 | block[7]

    def __repr__(self):
        return "uncompensated_reading(temp=0x{0:08X}, pressure=0x{1:08X}, humidity=0x{2:08X}, block={3})".format(
            self.temperature, self.pressure, self.humidity,
            ":".join("{0:02X}".format(c) for c in self._block))


class compensated_readings(object):
    """
    Compensation formulas translated from Appendix A (8.1) of BME280 datasheet

    """
    def __init__(self, raw_readings, compensation_params):
        self._comp = compensation_params
        self.uncompensated = raw_readings
        self.temperature = self.__tfine(raw_readings.temperature) / 5120.0
        self.humidity = self.__calc_humidity(raw_readings.humidity,
                                             raw_readings.temperature)
        self.pressure = self.__calc_pressure(raw_readings.pressure,
                                             raw_readings.temperature)

    def __tfine(self, t):
        v1 = (t / 16384.0 - self._comp.dig_T1 / 1024.0) * self._comp.dig_T2
        v2 = ((t / 131072.0 - self._comp.dig_T1 / 8192.0) ** 2) * self._comp.dig_T3
        return v1 + v2

    def __calc_humidity(self, h, t):
        res = self.__tfine(t) - 76800.0
        res = (h - (self._comp.dig_H4 * 64.0 + self._comp.dig_H5 / 16384.0 * res)) * \
            (self._comp.dig_H2 / 65536.0 * (1.0 + self._comp.dig_H6 / 67108864.0 * res *
                                            (1.0 + self._comp.dig_H3 / 67108864.0 * res)))
        res = res * (1.0 - (self._comp.dig_H1 * res / 524288.0))
        return max(0.0, min(res, 100.0))

    def __calc_pressure(self, p, t):
        v1 = self.__tfine(t) / 2.0 - 64000.0
        v2 = v1 * v1 * self._comp.dig_P6 / 32768.0
        v2 = v2 + v1 * self._comp.dig_P5 * 2.0
        v2 = v2 / 4.0 + self._comp.dig_P4 * 65536.0
        v1 = (self._comp.dig_P3 * v1 * v1 / 524288.0 + self._comp.dig_P2 * v1) / 524288.0
        v1 = (1.0 + v1 / 32768.0) * self._comp.dig_P1

        # Prevent divide by zero
        if v1 == 0:
            return 0

        res = 1048576.0 - p
        res = ((res - v2 / 4096.0) * 6250.0) / v1
        v1 = self._comp.dig_P9 * res * res / 2147483648.0
        v2 = res * self._comp.dig_P8 / 32768.0
        res = res + (v1 + v2 + self._comp.dig_P7) / 16.0
        return res

    def __repr__(self):
        return "compensated_reading(temp={0:0.3f} °C, pressure={1} Pa, humidity={2:0.2f} % rH)".format(
             self.temperature, self.pressure, self.humidity)


class params(dict):
    __getattr__ = dict.__getitem__
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__


class memoize:
    def __init__(self, f):
        self.f = f
        self.memo = {}

    def __call__(self, *args):
        if args not in self.memo:
            self.memo[args] = self.f(*args)
        return self.memo[args]

if __name__ == "__main__":
    bme = bmeReader(1, "bme")
    bme.start()
    try:
        bme.calibrate(134)
        while True:
            sleep(0.1)
            with bme.lock:
                sys.stdout.write("\033[2Jtmp: " + str(bme.temperature))
                sys.stdout.write("\nhum: " + str(bme.humidity))
                sys.stdout.write("\npres: " + str(bme.pressure))
                sys.stdout.write("\npres-alt: " +str(bme.altitude) + "\n")
    except KeyboardInterrupt:
        pass
    finally:
        bme.endThread()
        bme.join()
