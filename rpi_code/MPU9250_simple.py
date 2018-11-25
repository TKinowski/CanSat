import sys

sys.path.append('.')
import RTIMU
import os.path
import time
import math
import operator

SETTINGS_FILE = "RTIMULib"

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

# timers
t_print = time.time()
t_damp = time.time()
t_fail = time.time()
t_fail_timer = 0.0
t_shutdown = 0

if (not imu.IMUInit()):
    hack = time.time()
    print ("failed to initialize mpu 9250!")
    exit()

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()

# data variables
roll = 0.0
pitch = 0.0
yaw = 0.0
heading = 0.0
rollrate = 0.0
pitchrate = 0.0
yawrate = 0.0
magnetic_deviation = -13.7


# dampening variables
t_one = 0
t_three = 0
roll_total = 0.0
roll_run = [0] * 10
heading_cos_total = 0.0
heading_sin_total = 0.0
heading_cos_run = [0] * 30
heading_sin_run = [0] * 30


while True:
    hack = time.time()

    if imu.IMURead():
        data = imu.getIMUData()
        fusionPose = data["fusionPose"]
        Gyro = data["gyro"]
        t_fail_timer = 0.0

        if (hack - t_damp) > .1:
            roll = round(math.degrees(fusionPose[0]), 1)
            pitch = round(math.degrees(fusionPose[1]), 1)
            yaw = round(math.degrees(fusionPose[2]), 1)
            rollrate = round(math.degrees(Gyro[0]), 1)
            pitchrate = round(math.degrees(Gyro[1]), 1)
            yawrate = round(math.degrees(Gyro[2]), 1)

            if yaw < 90.01:
                    heading = yaw + 270 - magnetic_deviation
            else:
                heading = yaw - 90 - magnetic_deviation
            if heading > 360.0:
                heading = heading - 360.0


            # Dampening functions
            roll_total = roll_total - roll_run[t_one]
            roll_run[t_one] = roll
            roll_total = roll_total + roll_run[t_one]
            roll = roll_total / 3
            heading_cos_total = heading_cos_total - heading_cos_run[t_three]
            heading_sin_total = heading_sin_total - heading_sin_run[t_three]
            heading_cos_run[t_three] = math.cos(math.radians(heading))
            heading_sin_run[t_three] = math.sin(math.radians(heading))
            heading_cos_total = heading_cos_total + heading_cos_run[t_three]
            heading_sin_total = heading_sin_total + heading_sin_run[t_three]
            heading = round(math.degrees(math.atan2(heading_sin_total/30,heading_cos_total/30)),1)

            if heading < 0.01:
                heading = heading + 360.0

            t_damp = hack
            t_one += 1
            if t_one == 3:
                t_one = 0
            t_three += 1
            if t_three == 30:
                t_three = 0

            if (hack - t_print) > 0.2:
                sys.stdout.write("\033[2J")
                print("roll: " + str(roll))
                print("pitch: " + str(pitch))
                print("yaw: " + str(yaw))
                print("roll rate: " + str(rollrate))
                print("pitch rate: " + str(pitchrate))
                print("yaw rate: " + str(yawrate))
                print("heading: " + str(heading))
                t_print = hack

    time.sleep(poll_interval*0.001)