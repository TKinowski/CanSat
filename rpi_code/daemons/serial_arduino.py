"""Defines the gpsReader class which runs in a separate thread and reads data from a gps module"""

import pynmea2
import sys
import time
import threading
import serial



class gpsReader (threading.Thread, pynmea2.NMEAStreamReader):
    """thread of gps reader, requires lock to access data"""
    def __init__(self, threadID, name, port="/dev/ttyAMA0"):
        threading.Thread.__init__(self)
        pynmea2.NMEAStreamReader.__init__(self)
        self.lock = threading.RLock()
        self.threadID = threadID
        self.name = name
        self.initVariables()
        self._end = False
        self.port = port
        self.ser = serial.Serial(self.port, baudrate = 9600, timeout = 0)
        

    def endThread(self):
        self._end = True

    def initVariables (self):
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.num_sats = 0

    def setup_gps(self):

        if True:
            if not self.ser.isOpen():
                self.ser.open()

            #baudrate 115200
            self.ser.write([0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc4, 0x96, 0xb5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22])
            time.sleep(0.2)

            self.ser = serial.Serial(self.port, baudrate = 115200, timeout = 0)
            self.ser.close()
            self.ser.open()

            #disable GSV
            self.ser.write([0xB5 ,0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39]) 
            #disable GSA
            self.ser.write([0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32])
            #disable GLL
            self.ser.write([0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B])

            #disable RMC
            self.ser.write([0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40])

            #rate = 10hz
            self.ser.write([0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12])

            time.sleep(0.2)


    def run(self):   #======================================================================MAIN FUNCTION

        self.setup_gps()
        self.ser.read_until('$')
        while not self._end:
            data = self.ser.read_all()
            try:
                self.encode(data)
            except UnicodeDecodeError:
                pass
            except pynmea2.ParseError:
                pass
            
    def encode(self, c):
        msgs = self.next(c)
        for msg in msgs:
            with self.lock:
                if isinstance(msg, pynmea2.types.talker.GGA):
                    self.latitude = msg.latitude
                    self.longitude = msg.longitude
                    if msg.altitude != None:
                        self.altitude = msg.altitude
                    self.num_sats = int(msg.num_sats)
                if isinstance(msg, pynmea2.types.talker.VTG):
                    pass
            if __name__ == "__main__":
                sys.stdout.write("\033[2Jlat: " + str(self.latitude))
                sys.stdout.write("\nlon: " + str(self.longitude))
                sys.stdout.write("\nalt: " + str(self.altitude))
                sys.stdout.write("\nquality: " + str(self.num_sats))
                sys.stdout.flush()


if __name__ == "__main__":
    gps = gpsReader(1, "gps")
    gps.daemon = True
    gps.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.flush()
        sys.stdout.write("\r\n")
        sys.stdout.flush()
        gps.endThread()
        gps.join()
