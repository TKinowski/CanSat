import sys

sys.path.append('.')
import RTIMU
import os.path
import time
import math
import operator
import threading
import pdb

class imu(object):

    def __init__(self, settings="RTIMULib"):
        
        self.settings=settings
        
        self.s = RTIMU.Settings(self.settings)
        self.imu = RTIMU.RTIMU(self.s)
        self.state=0

        #position variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.heading = 0.0
        self.rollrate = 0.0
        self.pitchrate = 0.0
        self.yawrate = 0.0
        self.magnetic_deviation = 6

        # dampening variables
        
        self.t_one = 0
        self.t_three = 0
        self.roll_total = 0.0
        self.roll_run = [0] * 3
        self.heading_cos_total = 0.0
        self.heading_sin_total = 0.0
        self.heading_cos_run = [0] * 10
        self.heading_sin_run = [0] * 10

        self.try_setup()
        
        
    def try_setup(self):
        self.state = self.imu.IMUInit()
        if (not self.state):
            return 1

        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)
        
        self.poll_interval = self.imu.IMUGetPollInterval()
        self.t_damp = time.time()
        
        
        return 0


    def sample(self):
        try:
            if self.imu.IMURead():

                data = self.imu.getIMUData()
                fusionPose = data["fusionPose"]
                Gyro = data["gyro"]

                self.roll = round(math.degrees(fusionPose[0]), 1)
                self.pitch = round(math.degrees(fusionPose[1]), 1)
                self.yaw = round(math.degrees(fusionPose[2]), 1)
                self.rollrate = round(math.degrees(Gyro[0]), 1)
                self.pitchrate = round(math.degrees(Gyro[1]), 1)
                self.yawrate = round(math.degrees(Gyro[2]), 1)
                self.heading = self.yaw + self.magnetic_deviation + 285
                if self.heading > 360.0:
                    self.heading = self.heading - 360.0

                # Dampening functions
                self.roll_total = self.roll_total - self.roll_run[self.t_one]
                self.roll_run[self.t_one] = self.roll
                self.roll_total = self.roll_total + self.roll_run[self.t_one]
                self.roll = self.roll_total / 3

                if False:
                    self.heading_cos_total = self.heading_cos_total - self.heading_cos_run[self.t_three]
                    self.heading_sin_total = self.heading_sin_total - self.heading_sin_run[self.t_three]
                    self.heading_cos_run[self.t_three] = math.cos(math.radians(self.heading))
                    self.heading_sin_run[self.t_three] = math.sin(math.radians(self.heading))
                    self.heading_cos_total = self.heading_cos_total + self.heading_cos_run[self.t_three]
                    self.heading_sin_total = self.heading_sin_total + self.heading_sin_run[self.t_three]
                    self.heading = round(math.degrees(math.atan2(self.heading_sin_total/10,self.heading_cos_total/10)),1)

                if self.heading < 0.001:
                    self.heading = self.heading + 360.0

                self.t_one += 1
                if self.t_one == 3:
                    self.t_one = 0
                self.t_three += 1
                if self.t_three == 10:
                    self.t_three = 0

                self.state = 1
        except:
            self.state = 0

            

        

class imuReader(threading.Thread, imu):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        imu.__init__(self)

        self.lock = threading.RLock()
        self.threadID = threadID
        self.name = name
        self._end = False

    def endThread(self):
        self._end = True

    def run(self):
        while not self._end:
            if not self.state: #if not working try to init every 0.5s
                self.try_setup()
                time.sleep(0.5)
            else:
                with self.lock:
                    self.sample()

if __name__=="__main__":
    imu = imuReader(1, "imu")
    imu.start()
    try:
        while True:
            if True:
                sys.stdout.write("\033[2J")
                sys.stdout.flush()
                with imu.lock:
                    print("roll: " + str(imu.roll))
                    print("pitch: " + str(imu.pitch))
                    print("yaw: " + str(imu.yaw))
                    print("roll rate: " + str(imu.rollrate))
                    print("pitch rate: " + str(imu.pitchrate))
                    print("yaw rate: " + str(imu.yawrate))
                    print("heading: " + str(imu.heading))
            time.sleep(0.1)
    except KeyboardInterrupt:
        sys.stdout.flush()
        sys.stdout.write("\r\n")
        sys.stdout.flush()
    finally:
        imu.endThread()
        imu.join()