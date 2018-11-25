"""
This script interfaces Raspberry pi zero with:
BME280 temperature/pressure/humidity sensor
MPU9250 imu
SX127x LoRa radio
Neo6m GPS reciever
PiCamera v2

Lora packet structure - every variable consists of 4 bytes (MSBF):

-transmission number
-4 bits showing state of: bme, gps, imu and bat.
-temperature
-humidity
-pressure
-altitude
-latitude
-longitude
-roll
-pitch
-yaw
-heading

verbosity settings:
0 - only log
1 - log + initialization output
2 - log + init & tx communicates
3 - log + printing data

data file (data.csv) structure:
temp, hum, press, alt(press), alt(GPS), lat, lon, sats, roll, pitch, yaw, heading
"""
import sys
import datetime
import argparse
import os
from time import sleep
import threading
import psutil

start_time = datetime.datetime.now()

logging_delay = 1
LoRa_frequency = 433.0

#logging setup
def timestamp():
    return "{0:.3f}".format((datetime.datetime.now()-start_time).total_seconds())

def log(msg):
    if not msg.endswith('\n'):
        msg += '\n'
    log_file.write("[{0}] ".format(timestamp()) + msg)

#debug output setup
def out(msg, min_verbosity = 1, save_log = False):
    if verbosity >= min_verbosity:
        sys.stdout.write(msg)
        sys.stdout.flush()
    if save_log:
        log(msg)



#argument parsing
parser = argparse.ArgumentParser()
parser.add_argument("-d", "--dir", help="directory to use as home", type=str, default='/mnt/fat/')
parser.add_argument("-v", "--verbosity", help="verbosity level, defaults to 2", type=int, choices=[0, 1, 2, 3], default=2)
parser.add_argument("-c", "--cam", help="whether to use camera, defaults to True", action="store_false")
parser.add_argument("-i", "--imu", help="whether to use IMU, defaults to True", action="store_false")
parser.add_argument("-a", "--air", help="whether to use BME280 sensor, defaults to True", action="store_false")
parser.add_argument("-r", "--radio", help="whether to use radio, defaults to True", action="store_false")
parser.add_argument("-g", "--gps", help="whether to use GPS, defaults to True", action="store_false")
parser.add_argument("-b", "--battery", help="whether to use battery reader, defaults to True", action="store_false")
parser.add_argument("-s", "--sleep", help="logging delay in seconds", type=float, default=0.2)
args = parser.parse_args()


verbosity = args.verbosity
use_bme = args.air
use_imu = args.imu
use_lora = args.radio
use_gps = args.gps
use_cam = args.cam
use_bat = args.battery
logging_delay = args.sleep
workingDirectory = args.dir

if not workingDirectory.endswith('/'):
    workingDirectory += '/'
if not workingDirectory.startswith('/'):
    out("directory should be absolute! Falling back to default.")
    workingDirectory = '/mnt/fat/'



#files setup
directory = workingDirectory + start_time.strftime("%Y-%m-%d_%H-%M-%S")
os.mkdir(directory)
directory += '/'
log_file = open(directory + "log.txt", 'w')
data_file = open(directory + "data.csv", 'w')



#=============================================
#========== PROGRAM STARTS HERE ==============
#=============================================

out("CanOS 0.2\n")
out("verbosity: " + str(verbosity) + '\n')


if use_lora:
    out("loading module LoRa... ")
    from SX127x.LoRa import *
    from SX127x.LoRaArgumentParser import LoRaArgumentParser
    from SX127x.board_config import BOARD
    out("done.\n")
    log("module LoRa loaded")


if use_bme:
    out("loading module BME... ")
    import bme_daemon.bme_daemon as bme_daemon
    out("done.\n")
    log("module BME loaded")

if use_gps:
    out("loading module GPS... ")
    import gps_daemon.gps_daemon as gps_daemon
    out("done.\n")
    log("module GPS loaded")

if use_imu:
    out("loading module IMU... ")
    import imu_daemon
    out("done.\n")
    log("module IMU loaded")

if use_cam:
    out("loading module Cam... ")
    import cam_daemon
    out("done.\n")
    log("module Cam loaded")

if use_bat:
    out("loading module BAT... ")
    import battery_daemon
    out("done.\n")
    log("module BAT loaded")


#conversion functions============================
def ftoi (value, dp):
    return int(round(value, dp)*(10**dp))

def itob (value):
    return  list((int(value)>>(8*i))&0xff for i in range(3,-1,-1))

def stob (string):
    return [ord(c) for c in ''.join(list(string))]

#lora class=====================================================================================
if use_lora:
    class LoraTx(LoRa, threading.Thread): 

        def __init__(self, _verbosity=False):
            BOARD.setup()
            LoRa.__init__(self, _verbosity)
            
            self.set_mode(MODE.SLEEP)
            self.set_dio_mapping([1,0,0,0,0,0])
            self.tx_counter = 0
            self.set_freq(LoRa_frequency)
            self.set_agc_auto_on(True)
            self.set_pa_config(pa_select=1, output_power=15)
            self._end = False

        def end(self):
            self._end = True

        def test(self):
            initial_ctr = self.tx_counter
            self.transmit([0x00, 0x01])
            sleep(0.2)
            return self.tx_counter>initial_ctr

        def createPacket(self):
            payload = []
            payload += itob(self.tx_counter)
            payload += [use_bme, use_gps, use_imu, use_bat]
            if use_bme:
                with bme.lock:
                    payload += itob(ftoi(bme.temperature, 2))
                    payload += itob(ftoi(bme.humidity, 2))
                    payload += itob(ftoi(bme.pressure, 0))
                    payload += itob(ftoi(bme.altitude, 2))
            if use_gps:
                with gps.lock:
                    payload += itob(ftoi(gps.latitude, 6))
                    payload += itob(ftoi(gps.longitude, 6))

            if use_imu:
                with imu.lock:
                    payload += itob(ftoi(imu.roll, 2))
                    payload += itob(ftoi(imu.pitch, 2))
                    payload += itob(ftoi(imu.yaw, 2))
                    payload += itob(ftoi(imu.heading, 2))

            if use_bat:
                with bat.lock:
                    payload += itob(ftoi(bat.v1, 3))

            return payload

        def on_tx_done(self):
            if self._end:
                return
            self.tx_counter += 1
            self.set_mode(MODE.STDBY)
            self.transmit(self.createPacket())





#OBJECTS SETUP================================
if use_bme:
    out("starting bme... ")
    bme=bme_daemon.bmeReader(1, "bme")
    bme.daemon = True
    bme.start()
    out("done.\n")
    log("module BME started")
if use_gps:
    out("starting GPS... ")
    gps = gps_daemon.gpsReader(2, "gps")
    gps.daemon = True
    gps.start()
    out("done.\n")
    log("module GPS started")
if use_imu:
    out("starting IMU... ")
    imu = imu_daemon.imuReader(3, "imu")
    imu.daemon = True
    imu.start()
    out("done.\n")
    log("module IMU started")
if use_bat:
    out ("starting battery daemon...")
    bat = battery_daemon.battery_daemon(4, "bat")
    bat.daemon = True
    bat.start()
    out ("done.\n")
    log("module BAT started")
if use_cam:
    out ("starting camera...")
    cam = cam_daemon.camera(5, "cam", cam_dir=directory, preview=False, verbose=False, delay=2)
    cam.daemon = True
    cam.start()
    out ("done.\n")
    log("module CAM started")
if use_lora:
    out("starting LoRa... ")
    lora = LoraTx()
    if not lora.test():
        out("failed!\n")
        log ("[!] failed to start module LoRa")
    else:
        out("done.\n")
        log("module LoRa started")



def printData():
    if use_bme:
        with bme.lock:
            out("\ntmp: " + str(bme.temperature))
            out("\nhum: " + str(bme.humidity))
            out("\npres: " + str(bme.pressure))
            out("\npres-alt: " +str(bme.altitude))
    if use_gps:
        with gps.lock:
            out("\nalt: " + str(gps.altitude))
            out("\nlat: " + str(gps.latitude))
            out("\nlon: " + str(gps.longitude))
            out("\nsats: " + str(gps.num_sats))

    if use_imu:
        with imu.lock:
            out("\nroll: " + str(imu.roll))
            out("\npitch: " + str(imu.pitch))
            out("\nyaw: " + str(imu.yaw))
            out("\nhead: " + str(imu.heading))

    if use_bat:
        with bat.lock:
            out("\nv1: " + str(bat.v1))
            out("\nv2: " + str(bat.v2))

def saveData():

    data_file.write(timestamp() + ',')
    if use_bme:
        with bme.lock:
            data_file.write("{0:.2f}".format(bme.temperature) + ',')
            data_file.write("{0:.2f}".format(bme.humidity) + ',')
            data_file.write("{0:.0f}".format(bme.pressure) + ',')
            data_file.write("{0:.2f}".format(bme.altitude) + ',')
    if use_gps:
        with gps.lock:
            data_file.write("{0:.2f}".format(gps.altitude) + ',')
            data_file.write("{0:.6f}".format(gps.latitude) + ',')
            data_file.write("{0:.6f}".format(gps.longitude) + ',')
            data_file.write("{0:d}".format(gps.num_sats) + ',')

    if use_imu:
        with imu.lock:
            data_file.write("{0:.2f}".format(imu.roll) + ',')
            data_file.write("{0:.2f}".format(imu.pitch) + ',')
            data_file.write("{0:.2f}".format(imu.yaw) + ',')
            data_file.write("{0:.2f}".format(imu.heading) + ',')
    
    if use_bat:
        with bat.lock:
            data_file.write("{0:.3f}".format(bat.v1) + ',')
            data_file.write("{0:.3f}".format(bat.v2) + ',')

    data_file.write('\n')
    data_file.flush()


#MAIN LOOP=====================================
try:
    out ("Setup complete, entering main loop\n", 1)
    while True:
        sleep(logging_delay)
        if verbosity >= 3:
            printData()
        saveData()

except KeyboardInterrupt:
    sys.stdout.flush()
    out("\rKeyboard Interrupt!\n\r")
    sys.stdout.flush()
finally:
    if use_lora:
        lora.end()
    if use_cam:
        cam.endThread()
        cam.join()
    if use_bat:
        bat.endThread()
        bat.join()
    if use_bme:
        bme.endThread()
        bme.join()
    if use_imu:
        imu.endThread()
        imu.join()
    if use_gps:
        gps.endThread()
        gps.join()





