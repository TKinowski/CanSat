""" Defines the BOARD class that contains the board pin mappings and RF module HF/LF info. """


import RPi.GPIO as GPIO
import spidev

import time


class BOARD:
    """ Board initialisation/teardown and pin configuration is kept here.
        Also, information about the RF module is kept here.
        This is the Raspberry Pi board with a RFM98W.
    """
    # Note that the BCOM numbering for the GPIOs is used.
    DIO0 = 24   # RaspPi GPIO 24
    RESET = 25  #RasPi GPIO 25

    # The spi object is kept here
    spi = None
    
    # tell pySX127x here whether the attached RF module uses low-band (RF*_LF pins) or high-band (RF*_HF pins).
    # low band (called band 1&2) are 137-175 and 410-525
    # high band (called band 3) is 862-1020
    low_band = True

    @staticmethod
    def setup():
        """ Configure the Raspberry GPIOs
        :rtype : None
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        #reset
        GPIO.setup(BOARD.RESET, GPIO.OUT)

        # DIOx
        GPIO.setup(BOARD.DIO0, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        
        GPIO.output(BOARD.RESET, 0)
        time.sleep(0.01)
        GPIO.output(BOARD.RESET, 1)
        time.sleep(0.01)

    @staticmethod
    def teardown():
        """ Cleanup GPIO and SpiDev """
        GPIO.cleanup()
        BOARD.spi.close()

    @staticmethod
    def SpiDev(spi_bus=0, spi_cs=0):
        """ Init and return the SpiDev object
        :return: SpiDev object
        :param spi_bus: The RPi SPI bus to use: 0 or 1
        :param spi_cs: The RPi SPI chip select to use: 0 or 1
        :rtype: SpiDev
        """
        BOARD.spi = spidev.SpiDev()
        BOARD.spi.open(spi_bus, spi_cs)
        BOARD.spi.max_speed_hz = 8000000
        return BOARD.spi

    @staticmethod
    def add_event_detect(dio_number, callback):
        """ Wraps around the GPIO.add_event_detect function
        :param dio_number: DIO pin 0...5
        :param callback: The function to call when the DIO triggers an IRQ.
        :return: None
        """
        GPIO.add_event_detect(dio_number, GPIO.RISING, callback=callback)

    @staticmethod
    def add_events(cb_dio0, cb_dio1, cb_dio2, cb_dio3, cb_dio4, cb_dio5):
        BOARD.add_event_detect(BOARD.DIO0, callback=cb_dio0)

