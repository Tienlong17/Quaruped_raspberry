import Math_auto_quadruped
from math import pi, sin, cos, asin, acos, atan2, sqrt
import numpy
import PCA_servo_control
import RPi.GPIO as GPIO
import time

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

#set GPIO Pins
trigger = [12,16,20,21]
echo =  [6,13,19,26]

 
#set GPIO direction (IN / OUT)
for i in range(len(echo)):
    GPIO.setup(trigger[i],GPIO.OUT)
    GPIO.setup(echo[i],GPIO.IN)
    
def Check_distance_SCR(number):
    GPIO.output(trigger[number-1], True)
    MaxTime = 0.04
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(trigger[number-1], False)
 
    StartTime = time.time()
    TimeOut = StartTime + MaxTime
    # save StartTime
    while GPIO.input(echo[number-1]) == 0: # tai sao ko dung if
        if(StartTime <= TimeOut):
            StartTime = time.time()
        else:
            return False
    TimeEnd = 0.04
    StopTime = time.time()
    TimeEnd = StopTime + TimeEnd
    # save time of arrival
    while GPIO.input(echo[number-1]) == 1:
        if(StopTime <= TimeEnd):
            StopTime = time.time()
        else:
            return False
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    # Viet ham check distance 
    if 0 < distance <= 15: # co vat can 
        return True
    else: 
        return False  

def Call_SR4(dire):
    t = False 
    if dire == 1 or dire == 5: # len 
        t = Check_distance_SCR(1) # ham nay se khac nhau 
        return t
    elif dire == 2 or dire == 6: # qua phai
        t = Check_distance_SCR(2)
        return t 
    elif dire == 3 : # xuong 
        t = Check_distance_SCR(3)
        return t
    elif dire == 4 or dire == 0: # qua trai
        t = Check_distance_SCR(4)
        return t
def Check_Object(dist):
    check_object1 = Call_SR4(dist)
    if check_object1 == False: # False nghia la khong co vat can va Tru la co vat can 
        return (dist,1) # khong co vat can thi duoc di <- se de ham di chuyen 
    else:
        check_object2 = Call_SR4(dist - 1)
        if check_object2 == False: # False nghia la khong co vat can va Tru la co vat can 
            print("thuc hien chuyen dong lan 2, huong:",dist - 1) # khong co vat can thi duoc di
            return (dist - 1,3)
        else:
            check_object3 = Call_SR4(dist + 1)
            if check_object3 == False: # False nghia la khong co vat can va Tru la co vat can 
                print("thuc hien chuyen dong lan 3, huong:",(dist + 1)) # khong co vat can thi duoc di
                return (dist + 1,3)
            else:
                check_object4 = Call_SR4(dist + 2)
                if check_object4 == False: # False nghia la khong co vat can va Tru la co vat can 
                    print("thuc hien chuyen dong lan 4, huong:",dist + 2) # khong co vat can thi duoc di
                    return (dist + 2,2)
                else:
                    return (dist,0)

#information detail Robot
L1 = 7
L2 = 10
L3 = 10
HIGH_stand = 12#day la chieu cao dat lam gia tri khi dung 

# information about action of Robot
s_step_trot = 2 # khoang cach sai buoc chan
high_step_trot = 2 # do cao nhac chan len de buoc tiep


s_step__crawl = 3 # khoang cach sai buoc chan
high_step_crawl = 12 # do cao nhac chan len de buoc tiep 
sampling_time_crawl  = 0.1# thoi gian lay mau 



def Move_Robot(type_move: int, direction: int, direc: int):
    global TM_trot, s_step_trot, high_step_trot, TM_crawl, s_step__crawl, high_step_crawl, sampling_time_crawl, L1, L2, L3, HIGH_stand
    for i in range(0,direc,1):
        if type_move == 1:
            Math_auto_quadruped.Type_Trot(direction, s_step_trot, high_step_trot, L1, L2, L3, HIGH_stand)
        if type_move == 2:
            Math_auto_quadruped.Type_Crawl(direction, s_step__crawl, high_step_crawl, L1, L2, L3, HIGH_stand)
def Default_0_degree():
    PCA_servo_control.Default_legs_3()
    PCA_servo_control.Default_legs_2()
    PCA_servo_control.Default_legs_1()
def Default_Stand_Up():
    PCA_servo_control.Default_Stand_Up()
def Stand_Robot(isStand):
    if isStand == 2:
        PCA_servo_control.Standup()
    elif isStand == 0:
        Default_0_degree()
        time.sleep(0.4)
        PCA_servo_control.Standup()
    elif isStand == 3:
        PCA_servo_control.Stand_Sit()
def Spin_Right():
    Math_auto_quadruped.Spin_Right(2, 3, L1, L2, L3, HIGH_stand)
def Spin_Left():
    Math_auto_quadruped.Spin_Left(2, 3, L1, L2, L3, HIGH_stand)
def Lie_Robot(isStand):
    if isStand ==1:
        PCA_servo_control.Downup()
def Sitdown(isStand):
    if isStand ==1:
        PCA_servo_control.Sitdown()
def Tempt():
    Math_auto_quadruped.set_stand(s_step_trot, high_step_trot, L1, L2, L3, HIGH_stand)