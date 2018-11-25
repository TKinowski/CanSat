"""Defines the bmpReader class which runs in a separate thread and reads from the BMP180"""

import threading
import BMP180
import sys
from time import sleep

class bmpReader (threading.Thread, BMP180.BMP180):

    """Thread of bmp reader, accessing data requires acquiring the lock"""

    def __init__(self, threadID, name):
        BMP180.BMP180.__init__(self, mode=BMP180.BMP180_ULTRAHIGHRES)
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.lock = threading.Lock()
        self.temperature = 0.0
        self.pressure = 0
        self.altitude = 0.0
        self.end = False
        
    def run(self):
        
        while not self.end:
            with self.lock:
                self.temperature = self.read_temperature()
                self.pressure = self.read_pressure()
                self.altitude = self.read_altitude()
            sleep(0.05)


