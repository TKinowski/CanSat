import sys
import time
import serial
import threading

ser = serial.Serial("/dev/ttyAMA0", baudrate = 9600, timeout = 0)

ser.write("hello from pi!")

def stob (string):
    return [ord(c) for c in ''.join(list(string))]

def send (message):
    ser.write(stob(message))

class serial_printer(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)

        self.threadID = threadID
        self.name = name
        self._end = False

    def endThread(self):
        self._end = True

    def run(self):
        while not self._end:
            sys.stdout.write(ser.read_all())

ppp = serial_printer(1, "ppp")
ppp.start()

try:
    while True:
        a = raw_input()
        send(a)
except KeyboardInterrupt:
    pass
finally:
    ppp.endThread()
    pass