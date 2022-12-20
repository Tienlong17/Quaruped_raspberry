# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
import numpy as np
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
time_stop = 0.05      # 0.02
spam_time = 0.01
def Rot_Angle(a, b):
    a.angle = b
    time.sleep(time_stop)

#RF

def RF_Angle_0(angle):
    servo0.angle = 90 - angle
    time.sleep(time_stop)
def RF_Angle_1(angle):
    servo1.angle = 90 - angle
    time.sleep(time_stop)
def RF_Angle_2(angle):
    servo2.angle = 180- angle
    time.sleep(time_stop)
#LF
def LF_Angle_0(angle):
    servo4.angle = 90 + angle
    time.sleep(time_stop)
def LF_Angle_1(angle):
    servo5.angle = 90 + angle
    time.sleep(time_stop)
def LF_Angle_2(angle):
    servo6.angle = angle
    time.sleep(time_stop)
#RH
def RH_Angle_0(angle):
    servo8.angle = 90 + angle
    time.sleep(time_stop)
def RH_Angle_1(angle):
    servo9.angle = 90 - angle
    time.sleep(time_stop)
def RH_Angle_2(angle):
    servo10.angle = 180- angle
    time.sleep(time_stop)
#LH
def LH_Angle_0(angle):
    servo12.angle = 90 - angle
    time.sleep(time_stop)
def LH_Angle_1(angle):
    servo13.angle = 90 + angle
    time.sleep(time_stop)
def LH_Angle_2(angle):
    servo14.angle = angle
    time.sleep(time_stop)    

def Rot_Arm_RF(angle0, angle1, angle2):
    servo0.angle = 90 - angle0 #chua fix
    time.sleep(time_stop)
    servo1.angle = 90 - angle1
    time.sleep(time_stop)
    servo2.angle = 180- angle2
    time.sleep(time_stop)
def Rot_Arm_RF_2(angle1, angle2):
    servo1.angle = 90 - angle1
    time.sleep(time_stop)
    servo2.angle = 180- angle2
    time.sleep(time_stop)

def Rot_Arm_LF(angle0, angle1, angle2):
    servo4.angle = 90 + angle0 
    time.sleep(time_stop)
    servo5.angle = 90 + angle1
    time.sleep(time_stop)
    servo6.angle = angle2
    time.sleep(time_stop)
def Rot_Arm_LF_2(angle1, angle2):
    servo5.angle = 90 + angle1
    time.sleep(time_stop)
    servo6.angle = angle2
    time.sleep(time_stop)

def Rot_Arm_RH(angle0, angle1, angle2):
    servo8.angle = 90 + angle0 
    time.sleep(time_stop)
    servo9.angle = 90 - angle1
    time.sleep(time_stop)
    servo10.angle = 180- angle2
    time.sleep(time_stop)
def Rot_Arm_RH_2(angle1, angle2):
    servo9.angle = 90 - angle1
    time.sleep(time_stop)
    servo10.angle = 180- angle2
    time.sleep(time_stop)

def Rot_Arm_LH(angle0, angle1, angle2):
    servo12.angle = 90 - angle0 #chua fix
    time.sleep(time_stop)
    servo13.angle = 90 + angle1
    time.sleep(time_stop)
    servo14.angle = angle2
    time.sleep(time_stop)
def Rot_Arm_LH_2(angle1, angle2):
    servo13.angle = 90 + angle1
    time.sleep(time_stop)
    servo14.angle = angle2
    time.sleep(time_stop)

def Rot_Arm_Rot_Forward_First(angle0, angle1, angle2):
    Rot_Arm_RF(angle0, angle1, angle2)
    time.sleep(time_stop)
    Rot_Arm_LH(angle0, angle1, angle2)

def Rot_Arm_Rot_Forward_Second(angle0, angle1, angle2):
    Rot_Arm_LF(angle0, angle1, angle2)
    time.sleep(time_stop)
    Rot_Arm_RH(angle0, angle1, angle2)
    time.sleep(time_stop)
def Default_legs_1():
    Rot_Angle(servo0,90)
    time.sleep(0.05)
    Rot_Angle(servo4,90)
    time.sleep(0.05)
    Rot_Angle(servo8,90)
    time.sleep(0.05)
    Rot_Angle(servo12,90)
    time.sleep(0.05)
def Default_legs_2():
    Rot_Angle(servo1,180)
    time.sleep(0.05)
    Rot_Angle(servo5,0)
    time.sleep(0.05)
    Rot_Angle(servo9,180)
    time.sleep(0.05)
    Rot_Angle(servo13,0)
    time.sleep(0.05)
def Default_legs_3():
    Rot_Angle(servo2,0)
    time.sleep(0.05)
    Rot_Angle(servo6,180)
    time.sleep(0.05)
    Rot_Angle(servo10,0)
    time.sleep(0.05)
    Rot_Angle(servo14,180)
    time.sleep(0.05)
def Default_Stand_Up():
    Default_legs_1()
    Rot_Angle(servo1, 135)
    Rot_Angle(servo5, 45)
    Rot_Angle(servo9, 135)
    Rot_Angle(servo13, 45)
    time.sleep(spam_time)
    Rot_Angle(servo2, 68)
    Rot_Angle(servo6, 112)
    Rot_Angle(servo10, 68)
    Rot_Angle(servo14, 112)
    
def Controll_Downup():
    anpha = 45
    for i in np.arange(0,anpha,1):
        # khau 2
        servo1.angle = (180-anpha) + i # tuc la chay toi 180 -anpha do
        servo5.angle = anpha - i
        servo9.angle = (180-anpha) + i
        servo13.angle = anpha - i
        time.sleep(spam_time)
        # khau 3
        servo2.angle = 1.5*(anpha-i)
        servo6.angle = (180 - 1.5*anpha) + 1.5*i
        time.sleep(spam_time)
        servo10.angle = 1.5*(anpha - i)
        servo14.angle = (180 - 1.5*anpha) + 1.5*i
        time.sleep(spam_time)
def Controll_Sitdown():
    anpha = 45
    for i in np.arange(0,anpha,1):
        # khau 2
        #servo1.angle = (180-anpha) + i # tuc la chay toi 180 -anpha do
        #servo5.angle = anpha - i
        servo9.angle = (180-anpha) + i
        servo13.angle = anpha - i
        time.sleep(spam_time)
        # khau 3
        #servo2.angle = 1.5*(anpha-i)
        #servo6.angle = (180 - 1.5*anpha) + 1.5*i
        #time.sleep(spam_time)
        servo10.angle = 1.5*(anpha - i)
        servo14.angle = (180 - 1.5*anpha) + 1.5*i
        time.sleep(spam_time)
def Controll_Stand_Sit():
    spam_time = 0.05
    anpha = 45
    time_anpha = 1.5
    for i in np.arange(0,anpha,1.5): # <2
        # khau 2
        #servo1.angle = 180 - i # tuc la chay toi 120 do
        #servo5.angle = i
        servo9.angle = 180 - i
        servo13.angle = i
        # khau 3
        #servo2.angle = time_anpha*i
        #servo6.angle = 180 - time_anpha*i
        servo10.angle = time_anpha*i
        servo14.angle = 180 - time_anpha*i
        time.sleep(spam_time)
def Standup():
    spam_time = 0.05
    anpha = 45
    time_anpha = 1.5
    for i in np.arange(0,anpha,1.5): # <2
        # khau 2
        servo1.angle = 180 - i # tuc la chay toi 120 do
        servo5.angle = i
        servo9.angle = 180 - i
        servo13.angle = i
        # khau 3
        servo2.angle = time_anpha*i
        servo6.angle = 180 - time_anpha*i
        servo10.angle = time_anpha*i
        servo14.angle = 180 - time_anpha*i
        time.sleep(spam_time)

def Downup():
    Default_legs_1()
    time.sleep(spam_time)
    Controll_Downup()

def Sitdown():
    Default_legs_1()
    time.sleep(spam_time)
    Controll_Sitdown()
def Stand_Sit():
    Default_legs_1()
    time.sleep(spam_time)
    Controll_Stand_Sit()
pca.deinit()