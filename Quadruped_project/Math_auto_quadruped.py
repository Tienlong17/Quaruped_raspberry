from array import array
from math import degrees, pi, sin, cos, sin, asin, acos, atan, atan2, sqrt, radians
import numpy as np
import time
import PCA_servo_control

def IK(x: float, y: float , z: float, L1: float, L2: float, L3: float, H: float):
    theta = []
    try:
        if x > 0:
            f11 = ( -atan(y/x) - atan(sqrt(x**2 + y**2  - L1**2)/L1))
        else:
            f11 = ( atan(y/x) - atan(sqrt(x**2 + y**2  - L1**2)/L1))
        theta_11 = round(degrees(f11), 2) #lam tron 2 chu so
        
        try:
            f13 = acos((x**2 + y**2 + z**2 - L1**2 - L2**2 - L3**2)/(2*L2*L3))       
        except:
            f13 = 0
        theta_13 = round(degrees(f13),2)

        f12 = atan(z/(sqrt(x**2 + y**2 + z**2 - L1**2))) - atan((L3*sin(radians(theta_13)))/(L2 + L3*cos(radians(theta_13))))
        theta_12 = round(degrees(f12),2)
        theta = [theta_11, theta_12, theta_13]
        return theta

    except:
        print("Viet ham dua cac chan de robot 4 chan dung im")

    
    return(theta)


def Type_Trot(direction, TM, s, h, sampling_time, L1, L2, L3, H):
    # do phan giai thoi gian 
    '''N/v: goi ham toa do de tra ve cac goc cho canh tay'''
    array =  np.arange(0, 2*TM + sampling_time, sampling_time)
    if direction == 5:
        print("dung 5")
    elif direction == 1:
        for t in array:
            time.sleep(sampling_time)
            G0_Forward(TM, s, h, sampling_time, t, L1, L2, L3, H)
    elif direction == 2:
        for t in array:
            time.sleep(sampling_time)
            Go_Backward(TM, s, h, sampling_time, t, L1, L2, L3, H)
    else:
        print("dung 0")
    
    
def G0_Forward(TM, s, h, sampling_time, t, L1, L2, L3, H):
    Arm_Forward_Trot_Frist(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.07)
    Arm_Forward_Trot_Second(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.07)
    Arm_Forward_Trot_Third(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.07)
    Arm_Forward_Trot_Fourth(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.07)
def Arm_Forward_Trot_Frist(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x1 = -L1
    z1 = round(Congthuc_toado_ditoi_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    y1 = round(Congthuc_toado_ditoi_chantruoc( TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua khau cuoi 1: x =", x1,", y =", y1,", z =", z1)
    theta = IK(x1, y1, z1, L1, L2, L3, H)
    print('theta0 =',theta[0], 'theta1 =', theta[1], 'theta2 =', theta[2] )
    PCA_servo_control.Rot_Arm_RF(theta[0], theta[1], theta[2])
def Arm_Forward_Trot_Second(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x1 = -L1
    z1 = round(Congthuc_toado_ditoi_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    y1 = round(Congthuc_toado_ditoi_chantruoc( TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua khau cuoi 1: x =", x1,", y =", y1,", z =", z1)
    theta = IK(x1, y1, z1, L1, L2, L3, H)
    print('theta0 =',theta[0], 'theta1 =', theta[1], 'theta2 =', theta[2] )
    PCA_servo_control.Rot_Arm_LH(theta[0], theta[1], theta[2])
def Arm_Forward_Trot_Third(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x2 = L1
    z2 = round(Congthuc_toado_ditoi_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    y2 = round(Congthuc_toado_ditoi_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua khau cuoi 2: x =", x2,", y =", y2,", z =", z2)
    beta = IK(x2, y2, z2, L1, L2, L3, H)
    print('beta0 =',beta[0], 'beta1 =', beta[1], 'beta2 =', beta[2] )
    PCA_servo_control.Rot_Arm_LF(beta[0], beta[1], beta[2])
def Arm_Forward_Trot_Fourth(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x2 = L1
    z2 = round(Congthuc_toado_ditoi_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    y2 = round(Congthuc_toado_ditoi_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua khau cuoi 2: x =", x2,", y =", y2,", z =", z2)
    beta = IK(x2, y2, z2, L1, L2, L3, H)
    print('beta0 =',beta[0], 'beta1 =', beta[1], 'beta2 =', beta[2] )
    PCA_servo_control.Rot_Arm_RH(beta[0], beta[1], beta[2])
def Go_Backward(TM, s, h, sampling_time, t, L1, L2, L3, H):
    '''Neu a == 2 thi di lui '''
    x1 = -L1
    z1 = round(Congthuc_toado_dilui_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    y1 = round(Congthuc_toado_dilui_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua khau cuoi 1: x =", x1,", y =", y1,", z =", z1)
    IK(x1, y1, z1, L1, L2, L3, H)


    x2 = L1
    z2 = round(Congthuc_toado_ditoi_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    y2 = round(Congthuc_toado_ditoi_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua khau cuoi 2: x =", x2,", y =", y2,", z =", z2)


  

def Congthuc_toado_ditoi_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H):
    '''Ham nay tra toa do tinh ra tu quy dao ve cac bien x,y,z'''    
    if (t < TM/2):
        Ps = s*(t/TM - 1/(4*pi)*sin(4*pi*t/TM)) - s/2
        Ph = - H + 2*h*(t/TM - 1/(4*pi)*sin(4*pi*t/TM))
    elif (TM/2 <= t < TM):
        Ps = s*(t/TM - 1/(4*pi)*sin(4*pi*t/TM)) - s/2
        Ph = - H + 2*h*(1 - t/TM + 1/(4*pi)*sin(4*pi*t/TM))
    elif (TM <= t < (3*TM/2)):
        Ps = s*((2*TM - t)/TM - 1/(4*pi)*sin(4*pi*(2*TM - t)/TM)) - s/2
        Ph = - H 
    elif ((3*TM/2) <= t <= 2*TM + sampling_time):
        Ps = s*((2*TM - t)/TM - 1/(4*pi)*sin(4*pi*(2*TM - t)/TM)) - s/2
        Ph = - H 
    toa_do = [Ps,Ph]
    return toa_do

def Congthuc_toado_ditoi_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H):
    '''Ham nay tra toa do tinh ra tu quy dao ve cac bien x,y,z'''
    Ps = 0
    Ph = 0
    if (t < TM):
        Ps = -s*(t/TM - 1/(4*pi)*sin(4*pi*t/TM)) + s/2
        Ph = - H 
    if (TM <= t < 3*TM/2 ):
        Ps = s*((t - TM)/TM - 1/(4*pi)*sin(4*pi*(t - TM)/TM)) - s/2
        Ph = - H + 2*h*((t - TM)/TM - 1/(4*pi)*sin(4*pi*(t - TM)/TM))
    if ((3*TM/2) <= t <= 2*TM + sampling_time):
        Ps = s*((t - TM)/TM - 1/(4*pi)*sin(4*pi*(t - TM)/TM)) - s/2
        Ph = - H + 2*h*(1 - (t - TM)/TM + 1/(4*pi)*sin(4*pi*(t - TM)/TM))
    toa_do = [Ps,Ph]
    return toa_do

def Congthuc_toado_dilui_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H):
    '''Ham nay tra toa do tinh ra tu quy dao ve cac bien x,y,z'''    
    if (t < TM/2):
        Ps = -s*(t/TM - 1/(4*pi)*sin(4*pi*t/TM)) + s/2
        Ph = - H + 2*h*(t/TM - 1/(4*pi)*sin(4*pi*t/TM))
    if (t >= TM/2 and t <= TM ):
        Ps = -s*(t/TM - 1/(4*pi)*sin(4*pi*t/TM)) + s/2
        Ph = - H + 2*h*(1 - t/TM + 1/(4*pi)*sin(4*pi*t/TM))
    if (t  >= TM ):
        Ps = -s*((2*TM - t)/TM - 1/(4*pi)*sin(4*pi*(2*TM - t)/TM)) + s/2
        Ph = - H 
    toa_do = [Ps,Ph]
    return toa_do

def Congthuc_toado_dilui_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H):
    '''Ham nay tra toa do tinh ra tu quy dao ve cac bien x,y,z'''    
    if (t < TM):
        Ps = s*(t/TM - 1/(4*pi)*sin(4*pi*t/TM)) - s/2
        Ph = - H 
    if (t >= TM and t < 3*TM/2):
        Ps = -s*((t - TM)/TM - 1/(4*pi)*sin(4*pi*(t - TM)/TM)) + s/2
        Ph = - H + 2*h*((t - TM)/TM - 1/(4*pi)*sin(4*pi*(t - TM)/TM))
    if (t  >= 3*TM/2):
        Ps = -s*((t - TM)/TM - 1/(4*pi)*sin(4*pi*(t - TM)/TM)) + s/2
        Ph = - H + 2*h*(1 - (t - TM)/TM + 1/(4*pi)*sin(4*pi*(t - TM)/TM))
    toa_do = [Ps,Ph]
    return toa_do

def Type_Crawl(direction, TM, s, h, sampling_time, L1, L2, L3, H):
    array =  np.arange(0, 2*TM + sampling_time, sampling_time)
    if direction == 0:
        print(" crawl dang dung")
    elif direction == 1:
        G0_Forward_Crawl(TM, s, h, sampling_time, L1, L2, L3, H)
    elif direction == 2:
        Go_Backward_Crawl(TM, s, h, sampling_time, L1, L2, L3, H)
    elif direction == 3:
        Go_Rightward_Crawl(TM, s, h, sampling_time, L1, L2, L3, H)
    elif direction == 4:
        Go_Lefttward_Crawl(TM, s, h, sampling_time, L1, L2, L3, H)
    else:
        print("dung 0")
def G0_Forward_Crawl(TM, s, h, sampling_time, L1, L2, L3, H):
    for t in np.arange(0, TM + sampling_time, sampling_time):
        Arm_Forward_Crawl_Frist(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(0, TM + sampling_time, sampling_time):
        Arm_Forward_Crawl_Second(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(TM, 2*TM + sampling_time, sampling_time):
        Arm_Forward_Crawl_Third(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(TM, 2*TM + sampling_time, sampling_time):
        Arm_Forward_Crawl_Fourth(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(0, TM + sampling_time, sampling_time):
        Arm_Forward_Crawl_Frist(TM, s, h, sampling_time, t + TM, L1, L2, L3, H)
        Arm_Forward_Crawl_Second(TM, s, h, sampling_time, t + TM, L1, L2, L3, H)
        Arm_Forward_Crawl_Third(TM, s, h, sampling_time, t, L1, L2, L3, H)
        Arm_Forward_Crawl_Fourth(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
def Arm_Forward_Crawl_Frist(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x1 = -L1
    z1 = round(Congthuc_toado_ditoi_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    y1 = round(Congthuc_toado_ditoi_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 1: x =", x1,", y =", y1,", z =", z1)
    theta = IK(x1, y1, z1, L1, L2, L3, H)
    print('theta0 =',theta[0], 'theta1 =', theta[1], 'theta2 =', theta[2] )
    PCA_servo_control.Rot_Arm_RF(theta[0], theta[1], theta[2])
def Arm_Forward_Crawl_Second(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x1 = L1
    z1 = round(Congthuc_toado_ditoi_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    y1 = round(Congthuc_toado_ditoi_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 2: x =", x1,", y =", y1,", z =", z1)
    theta = IK(x1, y1, z1, L1, L2, L3, H)
    print('theta0 =',theta[0], 'theta1 =', theta[1], 'theta2 =', theta[2] )
    PCA_servo_control.Rot_Arm_LH(theta[0], theta[1], theta[2])
def Arm_Forward_Crawl_Third(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x2 = L1
    z2 = round(Congthuc_toado_ditoi_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    y2 = round(Congthuc_toado_ditoi_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 3: x =", x2,", y =", y2,", z =", z2)
    beta = IK(x2, y2, z2, L1, L2, L3, H)
    print('beta0 =',beta[0], 'beta1 =', beta[1], 'beta2 =', beta[2] , t)
    PCA_servo_control.Rot_Arm_LF(beta[0], beta[1], beta[2])
def Arm_Forward_Crawl_Fourth(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x2 = - L1
    z2 = round(Congthuc_toado_ditoi_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    y2 = round(Congthuc_toado_ditoi_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 4: x =", x2,", y =", y2,", z =", z2)
    beta = IK(x2, y2, z2, L1, L2, L3, H)
    print('beta0 =',beta[0], 'beta1 =', beta[1], 'beta2 =', beta[2] )
    PCA_servo_control.Rot_Arm_RH(beta[0], beta[1], beta[2])
def Go_Backward_Crawl(TM, s, h, sampling_time, L1, L2, L3, H):
    for t in np.arange(0, TM + sampling_time, sampling_time):
        Arm_Backward_Crawl_Frist(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(0, TM + sampling_time, sampling_time):
        Arm_Backward_Crawl_Second(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(TM, 2*TM + sampling_time, sampling_time):
        Arm_Backward_Crawl_Third(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(TM, 2*TM + sampling_time, sampling_time):
        Arm_Backward_Crawl_Fourth(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(0, TM + sampling_time, sampling_time):
        Arm_Backward_Crawl_Frist(TM, s, h, sampling_time, t + TM, L1, L2, L3, H)
        Arm_Backward_Crawl_Second(TM, s, h, sampling_time, t + TM, L1, L2, L3, H)
        Arm_Backward_Crawl_Third(TM, s, h, sampling_time, t, L1, L2, L3, H)
        Arm_Backward_Crawl_Fourth(TM, s, h, sampling_time, t, L1, L2, L3, H)
def Arm_Backward_Crawl_Frist(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x1 = -L1
    z1 = round(Congthuc_toado_dilui_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    y1 = round(Congthuc_toado_dilui_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 1: x =", x1,", y =", y1,", z =", z1)
    theta = IK(x1, y1, z1, L1, L2, L3, H)
    print('theta0 =',theta[0], 'theta1 =', theta[1], 'theta2 =', theta[2] )
    PCA_servo_control.Rot_Arm_RF(theta[0], theta[1], theta[2])
def Arm_Backward_Crawl_Second(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x1 = L1
    z1 = round(Congthuc_toado_dilui_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    y1 = round(Congthuc_toado_dilui_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 2: x =", x1,", y =", y1,", z =", z1)
    theta = IK(x1, y1, z1, L1, L2, L3, H)
    print('theta0 =',theta[0], 'theta1 =', theta[1], 'theta2 =', theta[2] )
    PCA_servo_control.Rot_Arm_LH(theta[0], theta[1], theta[2])
def Arm_Backward_Crawl_Third(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x2 = L1
    z2 = round(Congthuc_toado_dilui_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    y2 = round(Congthuc_toado_dilui_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 3: x =", x2,", y =", y2,", z =", z2)
    beta = IK(x2, y2, z2, L1, L2, L3, H)
    print('beta0 =',beta[0], 'beta1 =', beta[1], 'beta2 =', beta[2] , t)
    PCA_servo_control.Rot_Arm_LF(beta[0], beta[1], beta[2])
def Arm_Backward_Crawl_Fourth(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x2 = -L1
    z2 = round(Congthuc_toado_dilui_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    y2 = round(Congthuc_toado_dilui_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 4: x =", x2,", y =", y2,", z =", z2)
    beta = IK(x2, y2, z2, L1, L2, L3, H)
    print('beta0 =',beta[0], 'beta1 =', beta[1], 'beta2 =', beta[2] )
    PCA_servo_control.Rot_Arm_RH(beta[0], beta[1], beta[2])
def Go_Rightward_Crawl(TM, s, h, sampling_time, L1, L2, L3, H):
    for t in np.arange(0, TM + sampling_time, sampling_time):
        Arm_Rightward_Crawl_Frist(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(0, TM + sampling_time, sampling_time):
        Arm_Rightward_Crawl_Second(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(TM, 2*TM + sampling_time, sampling_time):
        Arm_Rightward_Crawl_Third(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(TM, 2*TM + sampling_time, sampling_time):
        Arm_Rightward_Crawl_Fourth(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(0, TM + sampling_time, sampling_time):
        Arm_Rightward_Crawl_Frist(TM, s, h, sampling_time, t + TM, L1, L2, L3, H)
        Arm_Rightward_Crawl_Second(TM, s, h, sampling_time, t + TM, L1, L2, L3, H)
        Arm_Rightward_Crawl_Third(TM, s, h, sampling_time, t, L1, L2, L3, H)
        Arm_Rightward_Crawl_Fourth(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
def Arm_Rightward_Crawl_Frist(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x1 = -L1 + round(Congthuc_toado_ditoi_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    z1 = 0
    y1 = round(Congthuc_toado_ditoi_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 1: x =", x1,", y =", y1,", z =", z1)
    theta = IK(x1, y1, z1, L1, L2, L3, H)
    print('theta0 =',theta[0], 'theta1 =', theta[1], 'theta2 =', theta[2] )
    PCA_servo_control.Rot_Arm_RF(theta[0], theta[1], theta[2])
def Arm_Rightward_Crawl_Second(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x1 = -L1 + round(Congthuc_toado_ditoi_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    z1 = 0
    y1 = round(Congthuc_toado_ditoi_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 2: x =", x1,", y =", y1,", z =", z1)
    theta = IK(x1, y1, z1, L1, L2, L3, H)
    print('theta0 =',theta[0], 'theta1 =', theta[1], 'theta2 =', theta[2] )
    PCA_servo_control.Rot_Arm_LH(theta[0], theta[1], theta[2])
def Arm_Rightward_Crawl_Third(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x2 = L1 + round(Congthuc_toado_ditoi_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    z2 = 0
    y2 = round(Congthuc_toado_ditoi_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 3: x =", x2,", y =", y2,", z =", z2)
    beta = IK(x2, y2, z2, L1, L2, L3, H)
    print('beta0 =',beta[0], 'beta1 =', beta[1], 'beta2 =', beta[2] , t)
    PCA_servo_control.Rot_Arm_LF(beta[0], beta[1], beta[2])
def Arm_Rightward_Crawl_Fourth(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x2 = L1 + round(Congthuc_toado_ditoi_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    z2 = 0 
    y2 = round(Congthuc_toado_ditoi_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 4: x =", x2,", y =", y2,", z =", z2)
    beta = IK(x2, y2, z2, L1, L2, L3, H)
    print('beta0 =',beta[0], 'beta1 =', beta[1], 'beta2 =', beta[2] )
    PCA_servo_control.Rot_Arm_RH(beta[0], beta[1], beta[2])
def Go_Lefttward_Crawl(TM, s, h, sampling_time, L1, L2, L3, H):
    for t in np.arange(0, TM + sampling_time, sampling_time):
        Arm_Leftward_Crawl_Frist(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(0, TM + sampling_time, sampling_time):
        Arm_Leftward_Crawl_Second(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(TM, 2*TM + sampling_time, sampling_time):
        Arm_Leftward_Crawl_Third(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(TM, 2*TM + sampling_time, sampling_time):
        Arm_Leftward_Crawl_Fourth(TM, s, h, sampling_time, t, L1, L2, L3, H)
    time.sleep(0.1)
    for t in np.arange(0, TM + sampling_time, sampling_time):
        Arm_Leftward_Crawl_Frist(TM, s, h, sampling_time, t + TM, L1, L2, L3, H)
        Arm_Leftward_Crawl_Second(TM, s, h, sampling_time, t + TM, L1, L2, L3, H)
        Arm_Leftward_Crawl_Third(TM, s, h, sampling_time, t, L1, L2, L3, H)
        Arm_Leftward_Crawl_Fourth(TM, s, h, sampling_time, t, L1, L2, L3, H)
def Arm_Leftward_Crawl_Frist(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x1 = -L1 + round(Congthuc_toado_dilui_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    z1 = 0
    y1 = round(Congthuc_toado_ditoi_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 1: x =", x1,", y =", y1,", z =", z1)
    theta = IK(x1, y1, z1, L1, L2, L3, H)
    print('theta0 =',theta[0], 'theta1 =', theta[1], 'theta2 =', theta[2] )
    PCA_servo_control.Rot_Arm_RF(theta[0], theta[1], theta[2])
def Arm_Leftward_Crawl_Second(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x1 = -L1 + round(Congthuc_toado_dilui_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    z1 = 0
    y1 = round(Congthuc_toado_ditoi_chantruoc(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 2: x =", x1,", y =", y1,", z =", z1)
    theta = IK(x1, y1, z1, L1, L2, L3, H)
    print('theta0 =',theta[0], 'theta1 =', theta[1], 'theta2 =', theta[2] )
    PCA_servo_control.Rot_Arm_LH(theta[0], theta[1], theta[2])
def Arm_Leftward_Crawl_Third(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x2 = L1 + round(Congthuc_toado_dilui_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    z2 = 0
    y2 = round(Congthuc_toado_dilui_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 3: x =", x2,", y =", y2,", z =", z2)
    beta = IK(x2, y2, z2, L1, L2, L3, H)
    print('beta0 =',beta[0], 'beta1 =', beta[1], 'beta2 =', beta[2] , t)
    PCA_servo_control.Rot_Arm_LF(beta[0], beta[1], beta[2])
def Arm_Leftward_Crawl_Fourth(TM, s, h, sampling_time, t, L1, L2, L3, H):
    x2 = L1 + round(Congthuc_toado_dilui_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[0],2)
    z2 = 0 
    y2 = round(Congthuc_toado_dilui_chansau(TM, s, h, t, sampling_time, L1, L2, L3, H)[1],2)
    print("     toa do cua lan 4: x =", x2,", y =", y2,", z =", z2)
    beta = IK(x2, y2, z2, L1, L2, L3, H)
    print('beta0 =',beta[0], 'beta1 =', beta[1], 'beta2 =', beta[2] )
    PCA_servo_control.Rot_Arm_RH(beta[0], beta[1], beta[2])
