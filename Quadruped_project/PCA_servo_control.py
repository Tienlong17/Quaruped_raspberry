# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import math 
from board import SCL, SDA
import busio

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 50

servo0 = servo.Servo(pca.channels[0], min_pulse=440, max_pulse=2400)
servo1 = servo.Servo(pca.channels[1], min_pulse=440, max_pulse=2400)
servo2 = servo.Servo(pca.channels[2], min_pulse=440, max_pulse=2400)
servo3 = servo.Servo(pca.channels[3], min_pulse=440, max_pulse=2400)
servo4 = servo.Servo(pca.channels[4], min_pulse=440, max_pulse=2400)
servo5 = servo.Servo(pca.channels[5], min_pulse=440, max_pulse=2400)
servo6 = servo.Servo(pca.channels[6], min_pulse=440, max_pulse=2400)
servo7 = servo.Servo(pca.channels[7], min_pulse=440, max_pulse=2400)
servo8 = servo.Servo(pca.channels[8], min_pulse=440, max_pulse=2400)
servo9 = servo.Servo(pca.channels[9], min_pulse=440, max_pulse=2400)
servo10 = servo.Servo(pca.channels[10], min_pulse=440, max_pulse=2400)
servo11 = servo.Servo(pca.channels[11], min_pulse=440, max_pulse=2400)
servo12 = servo.Servo(pca.channels[12], min_pulse=440, max_pulse=2400)
servo13 = servo.Servo(pca.channels[13], min_pulse=440, max_pulse=2400)
servo14 = servo.Servo(pca.channels[14], min_pulse=440, max_pulse=2400)
servo15 = servo.Servo(pca.channels[15], min_pulse=440, max_pulse=2400)
# We sleep in the loops to give the servo time to move into position.
time_stop = 0.007
def Rot_Arm_RF(angle0, angle1, angle2):
    servo0.angle = 90 + angle0 #chua fix
    time.sleep(time_stop)
    servo1.angle = 90 - angle1
    time.sleep(time_stop)
    servo2.angle = 180- angle2
    time.sleep(time_stop)

def Rot_Arm_LF(angle0, angle1, angle2):
    servo4.angle = 90 + angle0 #chua fix
    time.sleep(time_stop)
    servo5.angle = 90 + angle1
    time.sleep(time_stop)
    servo6.angle = angle2
    time.sleep(time_stop)

def Rot_Arm_RH(angle0, angle1, angle2):
    servo8.angle = 90 + angle0 #chua fix
    time.sleep(time_stop)
    servo9.angle = 90 - angle1
    time.sleep(time_stop)
    servo10.angle = 180- angle2
    time.sleep(time_stop)

def Rot_Arm_LH(angle0, angle1, angle2):
    servo12.angle = 90 + angle0 #chua fix
    time.sleep(time_stop)
    servo13.angle = 90 + angle1
    time.sleep(time_stop)
    servo14.angle = angle2
    time.sleep(time_stop)

def Rot_Arm_Rot_Forward_First(angle0, angle1, angle2):
    Rot_Arm_RF(angle0, angle1, angle2)
    time.sleep(0.01)
    Rot_Arm_LH(angle0, angle1, angle2)

def Rot_Arm_Rot_Forward_Second(angle0, angle1, angle2):
    Rot_Arm_LF(angle0, angle1, angle2)
    time.sleep(0.01)
    Rot_Arm_RH(angle0, angle1, angle2)
    time.sleep(0.01)

pca.deinit()

