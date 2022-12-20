from array import array
from math import degrees, pi, sin, cos, sin, asin, acos, atan, atan2, sqrt, radians
import numpy as np
import time
import PCA_servo_control
number_round = 0
#delta_x = 1
delta_y = 1
delta_z = -2
p_strong = 1
spam_time = 0.08
def IK(x: float, y: float , z: float, L1: float, L2: float, L3: float, H: float):
    theta = []
    try:
        if x > 0:
            f11 = ( -atan(y/x) - atan(sqrt(x**2 + y**2  - L1**2)/L1))
        else:
            f11 = ( atan(y/x) - atan(sqrt(x**2 + y**2  - L1**2)/L1))
        theta_11 = round(degrees(f11), number_round) #lam tron 2 chu so
        
        try:
            f13 = acos((x**2 + y**2 + z**2 - L1**2 - L2**2 - L3**2)/(2*L2*L3))       
        except:
            f13 = 0
        theta_13 = round(degrees(f13),number_round)

        f12 = atan(z/(sqrt(x**2 + y**2 + z**2 - L1**2))) - atan((L3*sin(radians(theta_13)))/(L2 + L3*cos(radians(theta_13))))
        theta_12 = round(degrees(f12),number_round)
        theta = [theta_11, theta_12, theta_13]
        #print('Chan truoc Theo truc do toa do: goc0 =',90 - theta_11,'Chan truoc: goc1 =',90 - theta_12,'Chan truoc: goc1 =',180 - theta_13)
        #print('Chan truoc Theo goc nhap: goc0 =',theta_11,'Chan truoc: goc1 =',theta_12,'Chan truoc: goc1 =',theta_13)
        
        return theta

    except:
        print("Viet ham dua cac chan de robot 4 chan dung im") 
    return(theta)
def Forward(s, h, L1, L2, L3, H):
    h = 6
    s = 3
    a = -4*h/(s*s)
    b = -a*s
    c = -H
    value_increase = 1.5
    delta_z = -2
    delta_y = 2
    for z in np.arange(0,s + value_increase, value_increase):
        theta = IK(-L1, a*z**2 + b*z + c +delta_y,z -delta_z,L1, L2, L3, H)
        PCA_servo_control.RF_Angle_2(theta[2])
        PCA_servo_control.RF_Angle_1(theta[1])
        PCA_servo_control.LH_Angle_2(theta[2])
        PCA_servo_control.LH_Angle_1(theta[1])
        theta = IK(-L1,-H  + delta_y,-z  +delta_y,L1, L2, L3, H)
        PCA_servo_control.LF_Angle_2(theta[2])
        PCA_servo_control.LF_Angle_1(theta[1])
        theta = IK(-L1,-H + delta_y,-z  -delta_z,L1, L2, L3, H)
        PCA_servo_control.RH_Angle_2(theta[2])
        PCA_servo_control.RH_Angle_1(theta[1])
    time.sleep(0.2)
    for z in np.arange(-s,0 + value_increase/2, value_increase):
        theta = IK(-L1,-H+ delta_y,- z  -delta_z,L1, L2, L3, H)
        PCA_servo_control.RF_Angle_2(theta[2])
        PCA_servo_control.RF_Angle_1(theta[1])
        PCA_servo_control.LH_Angle_2(theta[2])
        PCA_servo_control.LH_Angle_1(theta[1])
        theta = IK(-L1, a*(z)**2 -b*(z) + c + delta_y,z -delta_z,L1, L2, L3, H)
        PCA_servo_control.LF_Angle_1(theta[1])
        PCA_servo_control.LF_Angle_2(theta[2])
        time.sleep(0.2)
        theta = IK(-L1, a*(z)**2 -b*(z) + c + delta_y,z  -delta_z,L1, L2, L3, H)
        PCA_servo_control.RH_Angle_2(theta[2])
        PCA_servo_control.RH_Angle_1(theta[1])
def Backward(s, h, L1, L2, L3, H):
    h = 6
    s = 4
    a = -4*h/(s*s)
    b = -a*s
    c = -H
    value_increase = 1.5
    delta_z = -2
    delta_y = 1
    delta_h = -1
    theta = IK(-L1,-H + delta_h + delta_y, -delta_z,L1, L2, L3, H)
    PCA_servo_control.LH_Angle_2(theta[2])
    PCA_servo_control.LH_Angle_1(theta[1])
    PCA_servo_control.RH_Angle_2(theta[2])
    PCA_servo_control.RH_Angle_1(theta[1])
    time.sleep(0.1)
    for z in np.arange(0,s + value_increase/2, value_increase):
        theta = IK(-L1, a*(-z)**2 - b*(-z) + c + delta_h +delta_y,-z -delta_z,L1, L2, L3, H)
        PCA_servo_control.RH_Angle_2(theta[2])
        PCA_servo_control.RH_Angle_1(theta[1])
        time.sleep(0.1)
        theta = IK(-L1, a*(-z)**2 - b*(-z) + c +delta_y,-z -delta_z,L1, L2, L3, H)
        PCA_servo_control.LF_Angle_2(theta[2])
        PCA_servo_control.LF_Angle_1(theta[1])
        theta = IK(-L1,-H + delta_h  + delta_y,z  -delta_z,L1, L2, L3, H)
        PCA_servo_control.LH_Angle_2(theta[2])
        PCA_servo_control.LH_Angle_1(theta[1])
        time.sleep(0.1)
        theta = IK(L1,-H  + delta_y,z  -delta_z,L1, L2, L3, H)
        PCA_servo_control.RF_Angle_1(theta[1])
        PCA_servo_control.RF_Angle_2(theta[2])      
    time.sleep(0.2)
    value_increase = 1.5
    for z in np.arange(-s,0 + value_increase/2, value_increase):
        theta = IK(-L1,-H  + delta_y, z  -delta_z,L1, L2, L3, H)
        PCA_servo_control.LF_Angle_2(theta[2])
        PCA_servo_control.LF_Angle_1(theta[1])
        theta = IK(-L1,-H + delta_h + delta_y, z  -delta_z,L1, L2, L3, H)
        PCA_servo_control.RH_Angle_2(theta[2])
        PCA_servo_control.RH_Angle_1(theta[1])
        
        theta = IK(-L1, a*(-z)**2 +b*(-z) + c + delta_h + delta_y,-z  -delta_z,L1, L2, L3, H)
        PCA_servo_control.LH_Angle_2(theta[2])
        PCA_servo_control.LH_Angle_1(theta[1])
        time.sleep(0.2)
        theta = IK(-L1, a*(-z)**2 +b*(-z) + c + delta_y,-z -delta_z,L1, L2, L3, H)
        PCA_servo_control.RF_Angle_1(theta[1])
        PCA_servo_control.RF_Angle_2(theta[2])
    time.sleep(0.2)
    PCA_servo_control.Default_Stand_Up()
################################################################################################################# Xoay vong
def Spin_Right(s, h, L1, L2, L3, H):
    h = 3 # max 6 
    s = 2
    a = -4*h/(s*s)
    b = -a*s
    c = -H
    value_increase = 1
    delta_z = -2
    delta_y = 1
    for z in np.arange(0,s + value_increase, value_increase):
        theta = IK(-L1-z, a*z**2 + b*z + c -delta_z,-z -delta_z,L1, L2, L3, H)
        PCA_servo_control.RF_Angle_2(theta[2])
        PCA_servo_control.RF_Angle_0(theta[0])
        PCA_servo_control.RF_Angle_1(theta[1])
        theta = IK(L1-z, a*z**2 + b*z + c -delta_z,z -delta_z,L1, L2, L3, H)
        PCA_servo_control.LH_Angle_2(theta[2])
        PCA_servo_control.LH_Angle_0(theta[0])
        PCA_servo_control.LH_Angle_1(theta[1])
    for z in np.arange(0,s + value_increase, value_increase):
        theta = IK(-L1-z, a*z**2 + b*z + c -delta_z,-z -delta_z,L1, L2, L3, H)
        PCA_servo_control.RH_Angle_2(theta[2])
        PCA_servo_control.RH_Angle_0(theta[0])
        PCA_servo_control.RH_Angle_1(theta[1])
        time.sleep(0.1)
        theta = IK(L1-z, a*z**2 + b*z + c -delta_z,z -delta_z,L1, L2, L3, H)
        PCA_servo_control.LF_Angle_2(theta[2])
        PCA_servo_control.LF_Angle_0(theta[0])
        PCA_servo_control.LF_Angle_1(theta[1])
    time.sleep(0.2)
    PCA_servo_control.Default_Stand_Up()
    
def Spin_Left(s, h, L1, L2, L3, H):
    h = 6
    s = 2
    a = -4*h/(s*s)
    b = -a*s
    c = -H
    value_increase = 1
    delta_z = -2
    delta_y = 1
    for z in np.arange(0,s + value_increase, value_increase):
        theta = IK(-L1+z, a*z**2 + b*z + c -delta_z,z -delta_z,L1, L2, L3, H)
        PCA_servo_control.RF_Angle_2(theta[2])
        PCA_servo_control.RF_Angle_0(theta[0])
        PCA_servo_control.RF_Angle_1(theta[1])
        theta = IK(L1-z, a*z**2 + b*z + c -delta_z,-z -delta_z,L1, L2, L3, H)
        PCA_servo_control.LH_Angle_2(theta[2])
        PCA_servo_control.LH_Angle_0(theta[0])
        PCA_servo_control.LH_Angle_1(theta[1])
    for z in np.arange(0,s + value_increase, value_increase):
        theta = IK(-L1-z, a*z**2 + b*z + c -delta_z,z -delta_z,L1, L2, L3, H)
        PCA_servo_control.RH_Angle_2(theta[2])
        PCA_servo_control.RH_Angle_0(theta[0])
        PCA_servo_control.RH_Angle_1(theta[1])
        time.sleep(0.1)
        theta = IK(L1-z, a*z**2 + b*z + c -delta_z,-z -delta_z,L1, L2, L3, H)
        PCA_servo_control.LF_Angle_2(theta[2])
        PCA_servo_control.LF_Angle_0(theta[0])
        PCA_servo_control.LF_Angle_1(theta[1])     
    time.sleep(0.2)
    PCA_servo_control.Default_Stand_Up()
################################################################################################################# Di Kieu Trot
def Type_Trot(direction, s, h, L1, L2, L3, H):
    if direction == 1 or direction == 5:
        G0_Forward_Trot(s, h, L1, L2, L3, H)
    elif direction == 3:
        Go_Backward_Trot(s, h, L1, L2, L3, H)
    elif direction == 2 or direction == 5:
        Go_Rightward_Trot(s, h, L1, L2, L3, H)
    elif direction == 4 or direction == 0:
        Go_Lefttward_Trot(s, h, L1, L2, L3, H)
    else:
        print("Dung im")
def G0_Forward_Trot(s, h, L1, L2, L3, H):
    #chuyen toa do
    #dua cac chan ve vi tri ban dau
    #chan 1 vs chan 4 di cung luc
    theta = IK(-L1,-H+h + delta_y,(s/2)- delta_z,L1, L2, L3, H)
    PCA_servo_control.RF_Angle_2(150)
    PCA_servo_control.RF_Angle_1(theta[1])
    time.sleep(0.02)
    PCA_servo_control.LH_Angle_2(150)
    PCA_servo_control.LH_Angle_1(theta[1])
    
    theta = IK(-L1,-H + delta_y, -s - delta_z, L1, L2, L3, H) # Cap chan di sau day phia sau
    PCA_servo_control.LF_Angle_1(theta[1])
    PCA_servo_control.LF_Angle_2(theta[2])
    PCA_servo_control.RH_Angle_1(theta[1])
    PCA_servo_control.RH_Angle_2(theta[2])

    theta = IK(-L1,-H + delta_y, s- delta_z,L1, L2, L3, H)
    PCA_servo_control.RF_Angle_2(theta[2])
    PCA_servo_control.RF_Angle_1(theta[1])
    PCA_servo_control.LH_Angle_2(theta[2])
    PCA_servo_control.LH_Angle_1(theta[1])
    time.sleep(0.5)
    
    #chan 2 va chan 3 di cung luc 
    theta = IK(-L1,-H+h + delta_y, - (s/2) - delta_z,L1, L2, L3, H) # cap chan sau 
    PCA_servo_control.LF_Angle_1(theta[1] +10)
    PCA_servo_control.LF_Angle_2(160)
    time.sleep(0.07)
    PCA_servo_control.RH_Angle_2(140)
    PCA_servo_control.RH_Angle_1(theta[1])
    
    theta = IK(-L1,-H + delta_y,- delta_z,L1, L2, L3, H) #Cap chan di dau tien day ra sau
    PCA_servo_control.RF_Angle_1(theta[1])
    PCA_servo_control.RF_Angle_2(theta[2])
    PCA_servo_control.LH_Angle_1(theta[1])
    PCA_servo_control.LH_Angle_2(theta[2])

    theta = IK(-L1,-H + delta_y,- delta_z,L1, L2, L3, H)
    PCA_servo_control.RH_Angle_1(theta[1])
    PCA_servo_control.RH_Angle_2(theta[2])
    PCA_servo_control.LF_Angle_2(theta[2])
    PCA_servo_control.LF_Angle_1(theta[1])
    time.sleep(0.08)
def Go_Backward_Trot(s, h, L1, L2, L3, H):
    delta_h = 0
    delta_y = 1
    s = 3
    time.sleep(0.08)
    theta = IK(-L1 ,-H + h + delta_h + delta_y,- delta_z,L1, L2, L3, H) #Cap chan dau tien di
    PCA_servo_control.LF_Angle_2(theta[2])
    PCA_servo_control.LF_Angle_1(theta[1])
    time.sleep(0.15)
    PCA_servo_control.RH_Angle_2(theta[2]+20)
    PCA_servo_control.RH_Angle_1(theta[1]-20)
    
    theta = IK(-L1,-H + delta_h + delta_y, s - delta_z, L1, L2, L3, H) # Cap chan di sau day phia sau
    PCA_servo_control.RF_Angle_2(theta[2])
    PCA_servo_control.RF_Angle_1(theta[1])
    PCA_servo_control.LH_Angle_2(theta[2])
    PCA_servo_control.LH_Angle_1(theta[1])
    time.sleep(0.05)

    theta = IK(-L1,-H + delta_h + delta_y, - delta_z,L1, L2, L3, H) # quay ve
    PCA_servo_control.LF_Angle_1(theta[1])
    PCA_servo_control.LF_Angle_2(theta[2])
    theta = IK(-L1,-H +1+ delta_h + delta_y, - delta_z,L1, L2, L3, H) # quay ve
    PCA_servo_control.RH_Angle_1(theta[1])
    PCA_servo_control.RH_Angle_2(theta[2])
    time.sleep(0.1)
    ############################################################
    
    theta = IK(-L1,-H +h+ delta_h + delta_y, s/2 - delta_z, L1, L2, L3, H) # Cap chan di sau day phia sau
    PCA_servo_control.LH_Angle_2(theta[2]+40)
    PCA_servo_control.LH_Angle_1(theta[1]-40)
    time.sleep(0.1)
    PCA_servo_control.RF_Angle_2(theta[2]+10)
    PCA_servo_control.RF_Angle_1(theta[1]-20)
    #print(theta)
    theta = IK(-L1,-H + delta_h + delta_y,  - delta_z,L1, L2, L3, H)
    PCA_servo_control.LF_Angle_1(theta[1])
    PCA_servo_control.LF_Angle_2(theta[2])
    PCA_servo_control.RH_Angle_2(theta[2])
    PCA_servo_control.RH_Angle_1(theta[1])
    
    time.sleep(0.1)
    theta = IK(-L1,-H + delta_h + delta_y, - delta_z,L1, L2, L3, H)
    PCA_servo_control.RF_Angle_2(theta[2])
    PCA_servo_control.RF_Angle_1(theta[1])
    PCA_servo_control.LH_Angle_2(theta[2])
    PCA_servo_control.LH_Angle_1(theta[1])
    time.sleep(0.01)
    PCA_servo_control.Default_Stand_Up()
def Go_Lefttward_Trot(s, h, L1, L2, L3, H):
    h = 3
    s = 2
    delta_h = 4
    theta = IK(L1, -H + delta_h + delta_z, -delta_z,L1, L2, L3,H)
    PCA_servo_control.LF_Angle_0(theta[0])
    PCA_servo_control.RF_Angle_0(theta[0])
    PCA_servo_control.RF_Angle_1(theta[1])
    PCA_servo_control.LF_Angle_1(theta[1])
    PCA_servo_control.RF_Angle_2(theta[2])
    PCA_servo_control.LF_Angle_2(theta[2])
    time.sleep(2)
    theta = IK(-L1 + s/2,-H + delta_h + h + delta_y, - delta_z, L1, L2, L3, H)#cap chan di truoc 
    PCA_servo_control.RF_Angle_0(theta[0])
    PCA_servo_control.RF_Angle_2(135)
    time.sleep(0.01)
    theta = IK(L1 + s/2,-H + h + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.LH_Angle_0(theta[0])
    PCA_servo_control.LH_Angle_2(135)
    
    theta = IK(L1 - s,-H + delta_h + delta_y, - delta_z, L1, L2, L3, H) # Cap chan di sau day phia kia
    PCA_servo_control.LF_Angle_0(theta[0])
    PCA_servo_control.LF_Angle_1(theta[1])
    PCA_servo_control.LF_Angle_2(theta[2])
    theta = IK(-L1 - s,-H + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.RH_Angle_0(theta[0])
    PCA_servo_control.RH_Angle_1(theta[1])
    PCA_servo_control.RH_Angle_2(theta[2])
    
    theta = IK(L1 + s,-H + delta_y,- delta_z, L1, L2, L3, H)
    PCA_servo_control.LH_Angle_0(theta[0])
    PCA_servo_control.LH_Angle_1(theta[1])
    PCA_servo_control.LH_Angle_2(theta[2])
    theta = IK(-L1 + s,-H + delta_h + delta_y,- delta_z,L1, L2, L3, H)
    PCA_servo_control.RF_Angle_0(theta[0])
    PCA_servo_control.RF_Angle_1(theta[1])
    PCA_servo_control.RF_Angle_2(theta[2])
    time.sleep(0.1)
    ############################################################
    theta = IK(L1 - s/2,-H + delta_h + h + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.LF_Angle_0(theta[0])
    PCA_servo_control.LF_Angle_1(theta[1])
    PCA_servo_control.LF_Angle_2(140)
    time.sleep(0.2)
    theta = IK(-L1 - s/2,-H +2 + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.RH_Angle_2(150)
    PCA_servo_control.RH_Angle_0(theta[0]+20)
    PCA_servo_control.RH_Angle_1(theta[1])
    
    
    theta = IK(-L1,-H + delta_h + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.RF_Angle_0(theta[0])
    PCA_servo_control.RF_Angle_1(theta[1])
    PCA_servo_control.RF_Angle_2(theta[2])
    
    theta = IK(L1 ,-H + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.LH_Angle_0(theta[0])
    PCA_servo_control.LH_Angle_1(theta[1])
    PCA_servo_control.LH_Angle_2(theta[2])
    theta = IK(-L1 ,-H + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.RH_Angle_0(theta[0])
    PCA_servo_control.RH_Angle_1(theta[1])
    PCA_servo_control.RH_Angle_2(theta[2])

    theta = IK(L1 ,-H+ delta_h + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.LF_Angle_0(theta[0])
    PCA_servo_control.LF_Angle_1(theta[1])
    PCA_servo_control.LF_Angle_2(theta[2])
    time.sleep(0.5)
def set_stand(s, h, L1, L2, L3, H):
    theta = IK(-L1,-H + delta_y, 0*(- delta_z), L1, L2, L3, H)#cap chan di truoc 
    PCA_servo_control.RF_Angle_0(theta[0])
    PCA_servo_control.RF_Angle_1(theta[1])
    PCA_servo_control.RF_Angle_2(theta[2])
def Go_Rightward_Trot(s, h, L1, L2, L3, H):
    #chan 1 vs chan 4 di cung luc
    delta_x = 1
    h = 3
    s = 2
    theta = IK(-L1 - s/2,-H + h + delta_y, - delta_z, L1, L2, L3, H)#cap chan di truoc 
    PCA_servo_control.RF_Angle_0(theta[0])
    PCA_servo_control.RF_Angle_2(135)
    time.sleep(0.0)
    theta = IK(L1 - s/2,-H + h + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.LH_Angle_0(theta[0])
    PCA_servo_control.LH_Angle_2(135)
    
    theta = IK(L1 +s,-H + delta_y, - delta_z, L1, L2, L3, H) # Cap chan di sau day phia kia
    PCA_servo_control.LF_Angle_0(theta[0])
    PCA_servo_control.LF_Angle_1(theta[1])
    PCA_servo_control.LF_Angle_2(theta[2])
    theta = IK(-L1 +s,-H + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.RH_Angle_0(theta[0])
    PCA_servo_control.RH_Angle_1(theta[1])
    PCA_servo_control.RH_Angle_2(theta[2])
    
    
    theta = IK(L1 - s,-H + delta_y,- delta_z, L1, L2, L3, H)
    PCA_servo_control.LH_Angle_0(theta[0])
    PCA_servo_control.LH_Angle_1(theta[1])
    PCA_servo_control.LH_Angle_2(theta[2])
    theta = IK(-L1 - s,-H + delta_y,- delta_z,L1, L2, L3, H)
    PCA_servo_control.RF_Angle_0(theta[0])
    PCA_servo_control.RF_Angle_1(theta[1])
    PCA_servo_control.RF_Angle_2(theta[2])
    time.sleep(0.1)
    ############################################################
    
    theta = IK(-L1 + s/2,-H +h + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.RH_Angle_0(theta[0]-20)
    PCA_servo_control.RH_Angle_2(140)
    time.sleep(0.02)
    theta = IK(L1 + s/2,-H + h + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.LF_Angle_0(theta[0]+20)
    PCA_servo_control.LF_Angle_2(150)
    
    theta = IK(-L1,-H + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.RF_Angle_0(theta[0])
    PCA_servo_control.RF_Angle_1(theta[1])
    PCA_servo_control.RF_Angle_2(theta[2])
    
    theta = IK(L1 ,-H + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.LH_Angle_0(theta[0])
    PCA_servo_control.LH_Angle_1(theta[1])
    PCA_servo_control.LH_Angle_2(theta[2])
    
    theta = IK(-L1 ,-H + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.RH_Angle_0(theta[0])
    PCA_servo_control.RH_Angle_1(theta[1])
    PCA_servo_control.RH_Angle_2(theta[2])

    theta = IK(L1 ,-H + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.LF_Angle_0(theta[0])
    PCA_servo_control.LF_Angle_1(theta[1])
    PCA_servo_control.LF_Angle_2(theta[2])
    time.sleep(0.5)
    

################################################################################################################# D/N servo

def RF_Servo(x1,y1,z1, L1, L2, L3, H): # quay toi goc dua vao toa do nhap vao
    theta = IK(x1, y1, z1, L1, L2, L3, H)
    PCA_servo_control.Rot_Arm_RF(0, theta[1], theta[2])
    #print(theta)
    time.sleep(spam_time)
    
def LH_Servo(x1,y1,z1, L1, L2, L3, H):
    theta = IK(x1, y1, z1, L1, L2, L3, H)
    PCA_servo_control.Rot_Arm_LH(theta[0], theta[1], theta[2])
    time.sleep(spam_time)
    
def RH_Servo(x1,y1,z1, L1, L2, L3, H):
    beta = IK(x1, y1, z1, L1, L2, L3, H)
    PCA_servo_control.Rot_Arm_RH(beta[0], beta[1], beta[2])
    time.sleep(spam_time)
    
def LF_Servo(x1,y1,z1, L1, L2, L3, H):
    beta = IK(x1,y1,z1, L1, L2, L3, H)
    PCA_servo_control.Rot_Arm_LF(beta[0], beta[1], beta[2])
    time.sleep(spam_time)
################################################################################################################# Di Crawl
def Type_Crawl(direction, s, h, L1, L2, L3, H):
    if direction == 1 or direction == 5:
        G0_Forward_Crawl(s, h, L1, L2, L3, H)
    elif direction == 3:
        Go_Backward_Crawl(s, h, L1, L2, L3, H)
    elif direction == 2 or direction == 5:
        Go_Rightward_Crawl(s, h, L1, L2, L3, H)
    elif direction == 4 or direction == 0:
        Go_Lefttward_Crawl(s, h, L1, L2, L3, H)
    else:
        print("Dung im")
def G0_Forward_Crawl(s, h, L1, L2, L3, H):
    #chan 1 vs chan 4 di cung luc
    theta = IK(-L1,-H+h + delta_y,(s/2)- delta_z,L1, L2, L3, H)
    PCA_servo_control.RF_Angle_2(140)
    PCA_servo_control.RF_Angle_1(theta[1])
    time.sleep(0.02)
    PCA_servo_control.LH_Angle_2(140)
    PCA_servo_control.LH_Angle_1(theta[1])
    

    theta = IK(-L1,-H + delta_y, s- delta_z,L1, L2, L3, H)
    PCA_servo_control.RF_Angle_2(theta[2])
    PCA_servo_control.RF_Angle_1(theta[1])
    PCA_servo_control.LH_Angle_2(theta[2])
    PCA_servo_control.LH_Angle_1(theta[1])
    time.sleep(0.5)
    ######################################
    #chan 2 va chan 3 di cung luc 
    theta = IK(-L1,-H+h + delta_y,(s/2) - delta_z,L1, L2, L3, H)
    PCA_servo_control.LF_Angle_1(theta[1])
    PCA_servo_control.LF_Angle_2(140)
    time.sleep(0.15)
    PCA_servo_control.RH_Angle_1(theta[1])
    PCA_servo_control.RH_Angle_2(140)
    

    theta = IK(-L1,-H + delta_y,s - delta_z,L1, L2, L3, H)
    PCA_servo_control.RH_Angle_1(theta[1])
    PCA_servo_control.RH_Angle_2(theta[2])
    PCA_servo_control.LF_Angle_2(theta[2])
    PCA_servo_control.LF_Angle_1(theta[1])
    value_increase = 1
    for i in np.arange(0,s + value_increase, value_increase):
        RF_Servo(-L1, -H + delta_y, s - delta_z -i, L1, L2, L3, H)
        LH_Servo(-L1, -H +delta_y, s - delta_z -i, L1, L2, L3, H)
        LF_Servo(-L1, -H + delta_y, s - delta_z -i, L1, L2, L3, H)
        RH_Servo(-L1, -H +delta_y, s - delta_z -i, L1, L2, L3, H)
    time.sleep(0.08)
def Go_Backward_Crawl(s, h, L1, L2, L3, H):
    s = 4
    value_increase = 1
    h = 6
    delta_h = 0
    for i in np.arange(0,s + value_increase, value_increase):
        RF_Servo(-L1, -H + delta_y + delta_h, - delta_z +i, L1, L2, L3, H)
        LF_Servo(-L1, -H + delta_y + delta_h,- delta_z +i, L1, L2, L3, H)
        LH_Servo(-L1, -H + delta_y ,- delta_z +i, L1, L2, L3, H)
        RH_Servo(-L1, -H + delta_y , - delta_z +i, L1, L2, L3, H)
    time.sleep(0.08)
    #chan 1 vs chan 4 di cung luc
    theta = IK(-L1,-H+h + delta_h + delta_y,(s/2)- delta_z,L1, L2, L3, H)
    PCA_servo_control.RF_Angle_2(theta[2]+10)
    PCA_servo_control.RF_Angle_1(theta[1]-10)
    time.sleep(0.3)
    theta = IK(-L1,-H + delta_y, - delta_z,L1, L2, L3, H)
    PCA_servo_control.RF_Angle_2(theta[2])
    PCA_servo_control.RF_Angle_1(theta[1])
    time.sleep(0.5)
    
    theta = IK(-L1,-H+ h + delta_h + delta_y,(s/2) - delta_z,L1, L2, L3, H)
    PCA_servo_control.LF_Angle_2(theta[2])
    PCA_servo_control.LF_Angle_1(theta[1]-20)
    time.sleep(0.01)
    theta = IK(-L1,-H + delta_y, -delta_z,L1, L2, L3, H)
    PCA_servo_control.LF_Angle_2(theta[2])
    PCA_servo_control.LF_Angle_1(theta[1])
    time.sleep(0.5)
    theta = IK(-L1,-H + h  + delta_h ,(s/2) - delta_z,L1, L2, L3, H)
    PCA_servo_control.RH_Angle_2(theta[2]+20)
    PCA_servo_control.RH_Angle_1(-90)
    time.sleep(0.6)
    theta = IK(-L1,-H + delta_y, -delta_z,L1, L2, L3, H)
    PCA_servo_control.RH_Angle_1(theta[1])
    PCA_servo_control.RH_Angle_2(theta[2])
    time.sleep(0.5)
    theta = IK(-L1,-H+h + delta_y,(s/2)- delta_z,L1, L2, L3, H)
    PCA_servo_control.LH_Angle_2(theta[2]+20)
    PCA_servo_control.LH_Angle_1(-90)
    time.sleep(0.6)
    theta = IK(-L1,-H + delta_y,- delta_z,L1, L2, L3, H)
    PCA_servo_control.LH_Angle_2(theta[2])
    PCA_servo_control.LH_Angle_1(theta[1])
    time.sleep(0.5)
    
def Go_Rightward_Crawl(s, h, L1, L2, L3, H):
    #chan 1 vs chan 4 di cung luc
    h = 3
    s = 3
    theta = IK(-L1 - s/2,-H + h + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.RF_Angle_0(theta[0])
    PCA_servo_control.RF_Angle_2(150)
    time.sleep(0.015)
    theta = IK(L1 - s/2,-H + h + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.LH_Angle_0(theta[0])
    PCA_servo_control.LH_Angle_2(150)
    time.sleep(0.02)
    theta = IK(L1 - s,-H + delta_y,- delta_z, L1, L2, L3, H)
    PCA_servo_control.LH_Angle_0(theta[0])
    PCA_servo_control.LH_Angle_1(theta[1])
    PCA_servo_control.LH_Angle_2(theta[2])
    theta = IK(-L1 - s,-H + delta_y,- delta_z,L1, L2, L3, H)
    PCA_servo_control.RF_Angle_0(theta[0])
    PCA_servo_control.RF_Angle_1(theta[1])
    PCA_servo_control.RF_Angle_2(theta[2])
    time.sleep(0.5)
    ############################################################
    
    theta = IK(-L1 - s/2,-H +h + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.RH_Angle_0(theta[0])
    PCA_servo_control.RH_Angle_2(170)
    time.sleep(0.02)
    theta = IK(L1 - s/2,-H + h + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.LF_Angle_0(theta[0])
    PCA_servo_control.LF_Angle_2(170)
    
    theta = IK(-L1 - s,-H + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.RH_Angle_0(theta[0])
    PCA_servo_control.RH_Angle_2(theta[2])
    theta = IK(L1 - s,-H + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.LF_Angle_0(theta[0])
    PCA_servo_control.LF_Angle_2(theta[2])
    time.sleep(0.5)
    value_increase = 1
    for i in np.arange(0,s + value_increase, value_increase):
        RF_Servo(-L1-s +i, -H + delta_y,- delta_z, L1, L2, L3, H)
        LH_Servo(L1-s +i, -H + delta_y,- delta_z, L1, L2, L3, H)
        LF_Servo(L1-s +i, -H + delta_y, - delta_z, L1, L2, L3, H)
        RH_Servo(-L1-s +i, -H + delta_y,- delta_z, L1, L2, L3, H)
    time.sleep(0.08)
    
def Go_Lefttward_Crawl(s, h, L1, L2, L3, H):
    #chan 1 vs chan 4 di cung luc
    h = 3
    s = 3
    theta = IK(-L1 + s/2,-H + h + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.RF_Angle_0(theta[0])
    PCA_servo_control.RF_Angle_2(150)
    time.sleep(0.02)
    theta = IK(L1 + s/2,-H + h + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.LH_Angle_0(theta[0])
    PCA_servo_control.LH_Angle_2(150)
    time.sleep(0.02)
    theta = IK(L1 + s,-H + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.LH_Angle_0(theta[0])
    PCA_servo_control.LH_Angle_2(theta[2])
    theta = IK(-L1 + s,-H + delta_y,- delta_z,L1, L2, L3, H)
    PCA_servo_control.RF_Angle_0(theta[0])
    PCA_servo_control.RF_Angle_2(theta[2])
    time.sleep(0.5)
    ############################################################
    theta = IK(L1 + s/2,-H + h + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.LF_Angle_0(theta[0])
    PCA_servo_control.LF_Angle_2(170)
    theta = IK(-L1 + s/2,-H + h + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.RH_Angle_0(theta[0])
    PCA_servo_control.RH_Angle_2(170)
    
    theta = IK(-L1 + s,-H + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.RH_Angle_0(theta[0])
    PCA_servo_control.RH_Angle_1(theta[1])
    PCA_servo_control.RH_Angle_2(theta[2])
    time.sleep(0.02)
    theta = IK(L1 + s,-H + delta_y, - delta_z, L1, L2, L3, H)
    PCA_servo_control.LF_Angle_0(theta[0])
    PCA_servo_control.LF_Angle_1(theta[1])
    PCA_servo_control.LF_Angle_2(theta[2])
    time.sleep(0.5)
    value_increase = 1
    for i in np.arange(0,s + value_increase, value_increase):
        RF_Servo(-L1+s -i, -H + delta_y,- delta_z, L1, L2, L3, H)
        LH_Servo(L1+s -i, -H + delta_y,- delta_z, L1, L2, L3, H)
        LF_Servo(L1+s -i, -H + delta_y, - delta_z, L1, L2, L3, H)
        RH_Servo(-L1+s -i, -H + delta_y,- delta_z, L1, L2, L3, H)
    time.sleep(0.08)
