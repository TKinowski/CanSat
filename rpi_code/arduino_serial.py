import sys
import time
import serial

ser = serial.Serial("/dev/ttyAMA0", baudrate = 9600, timeout = 0)

ser.write("hello from pi!")


while True:
    a = ser.read_all()
    sys.stdout.write(a)
    ser.write(a)
    