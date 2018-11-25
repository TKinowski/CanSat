from servo import servo
import RPi.GPIO as GPIO
from time import sleep
import argparse

# argument parsing
parser = argparse.ArgumentParser()
parser.add_argument("rotation", help="rotation", type=float, default=90)
parser.add_argument("elevation", help="elevation", type=float, default=84)
args = parser.parse_args()

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

srv_r = servo(12)
srv_l = servo (13)

srv_r.rotate_to(args.rotation)
srv_l.rotate_to(args.elevation)
sleep(0.6)
srv_r.stop()

if False:
    try:
        pass
    except KeyboardInterrupt:
        pass

GPIO.cleanup()
