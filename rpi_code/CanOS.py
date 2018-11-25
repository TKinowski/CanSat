"""
This is the program for raspberry pi during the flight phase of CanSat, 
using MPU9250, PiCamera v2, servos and serial connection with atmega328p.


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
import time
from time import sleep
import threading
import psutil
import serial

start_time = datetime.datetime.now()


def timestamp():
    """returns time from start in seconds with 3 decimal points"""
    return "{0:.3f}".format((datetime.datetime.now()-start_time).total_seconds())

def log(msg):
    """writes the message to the log file, always skips to next line"""
    if not msg.endswith('\n'):
        msg += '\n'
    log_file.write("[{0}] ".format(timestamp()) + msg)

def out(msg, min_verbosity = 1, save_log = False):
    """prints the message if verbosity is not lower than <<min_verbosity>>, optionally saves to log"""
    if verbosity >= min_verbosity:
        sys.stdout.write(msg)
    if save_log:
        log(msg)


# ONLY FOR DEBUG
#argument parsing
parser = argparse.ArgumentParser()
parser.add_argument("-d", "--dir", help="directory to use as home", type=str, default='/mnt/fat/')
parser.add_argument("-v", "--verbosity", help="verbosity level, defaults to 2", type=int, choices=[0, 1, 2, 3], default=2)
parser.add_argument("-c", "--cam", help="whether to use camera, defaults to True", action="store_false")
parser.add_argument("-i", "--imu", help="whether to use IMU, defaults to True", action="store_false")
parser.add_argument("-s", "--sleep", help="logging delay in seconds", type=float, default=0.2)
args = parser.parse_args()


verbosity = args.verbosity
use_imu = args.imu
use_cam = args.cam
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

if use_imu:
    out("loading module IMU... ")
    import imu_daemon
    out("done.\n")
    log("module IMU loaded")

if use_cam:
    out("loading module Cam... ")
    import daemons.cam_daemon as cam_daemon
    out("done.\n")
    log("module Cam loaded")


#conversion functions============================
def ftoi (value, dp):
    return int(round(value, dp)*(10**dp))

def itob (value):
    return  list((int(value)>>(8*i))&0xff for i in range(3,-1,-1))

def stob (string):
    return [ord(c) for c in ''.join(list(string))]



#OBJECTS SETUP================================

if use_imu:
    out("starting IMU... ")
    imu = imu_daemon.imuReader(3, "imu")
    if(not imu.state):
        out("failed [!]\n")
        log("module IMU failed to start")
    else:
        imu.daemon = True
        imu.start()
        out("done.\n")
        log("module IMU started")

if use_cam:
    out ("starting camera...")
    cam = cam_daemon.camera(5, "cam", cam_dir=directory, preview=False, verbose=False, delay=2)
    cam.daemon = True
    cam.start()
    out ("done.\n")
    log("module CAM started")

class bme_data:
    def __init__ (self):
        self.temperature=0
        self.humidity=0
        self.pressure=0
        self.altitude=0
bme = bme_data()

class gps_data:
    def __init__ (self):
        self.latitude=0
        self.longitude=0
        self.altitude=0
        self.num_sats=0
gps = gps_data()

def printData():
    if use_imu:
        with imu.lock:
            out("\nroll: " + str(imu.roll))
            out("\npitch: " + str(imu.pitch))
            out("\nyaw: " + str(imu.yaw))
            out("\nhead: " + str(imu.heading))

def saveData():
    """saves current data to a csv file"""
    data_file.write(timestamp() + ',')
    if False:
        data_file.write("{0:.2f}".format(bme.temperature) + ',')
        data_file.write("{0:.2f}".format(bme.humidity) + ',')
        data_file.write("{0:.0f}".format(bme.pressure) + ',')
        data_file.write("{0:.2f}".format(bme.altitude) + ',')

    if False:
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

    data_file.write('\n')
    data_file.flush()




#CONTROL LOOP=====================================
try:
    out ("Setup complete, entering main loop\n", 1)
    


        
        
        

except KeyboardInterrupt:
    sys.stdout.flush()
    out("\rKeyboard Interrupt!\n\r")
    sys.stdout.flush()
finally:
    if use_imu:
        imu.endThread()
        imu.join()

from arduino_comm import arduino_comm




#FLIGHT MODE STARTS HERE ================================================================================

out("entering flight mode", save_log=True)

from servo import servo
import RPi.GPIO as GPIO

lastSave = time.time()

if use_imu:
    out("starting IMU... ")
    imu = imu_daemon.imuReader(3, "imu")
    if(not imu.state):
        out("failed [!]\n")
        log("module IMU failed to start")
    else:
        imu.daemon = True
        imu.start()
        out("done.\n")
        log("module IMU started")

if use_cam:
    out ("starting camera...")
    cam = cam_daemon.camera(5, "cam", cam_dir=directory, preview=False, verbose=False, delay=2)
    cam.daemon = True
    cam.start()
    out ("done.\n")
    log("module CAM started")

#flight loop here

    while True:
        if (time.time() - lastSave > logging_delay):
            #saveData()
            pass
    #get commands from arudino

    #parse data from commands

    #save all data to csv file

    #do photos if orientation is good

    #if NCS finished with the last batch send it the most recent photo

    #if we got data from NCS make next packet a special one

    #if more than 0.9 seconds passed from last packet being sent & the radio is free send a packet

    #given the orientation and position do something with the servos

