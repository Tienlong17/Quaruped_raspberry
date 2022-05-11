import Math_auto_quadruped
from math import pi, sin, cos, asin, acos, atan2, sqrt
import numpy

#information detail Robot
L1 = 8
L2 = 18.5
L3 = 21.5
H = 30 #day la chieu cao dat lam gia tri khi dung tinh tu khop 1 den khau cuoi cung


TM = 1 # chu ki buoc
s = 15 # khoang cach sai buoc chan
h = 6 # do cao nhac chan len
sampling_time  = 0.1# thoi gian lay mau 



def Move_Robot(type_move: int, direction: int):
    if type_move == 1:
        Math_auto_quadruped.Type_Trot(direction, TM, s, h, sampling_time, L1, L2, L3, H)
    if type_move == 2:
        Math_auto_quadruped.Type_Crawl(direction, TM, s, h, sampling_time, L1, L2, L3, H)