import Math_auto_quadruped
from math import pi, sin, cos, asin, acos, atan2, sqrt
import numpy

#information detail Robot
L1 = 8
L2 = 18.5
L3 = 21.5
H = 30 #day la chieu cao dat lam gia tri khi dung tinh tu khop 1 den khau cuoi cung


TM = 2 # chu ki buoc
s = 25 # khoang cach sai buoc chan
h = 12 # do cao nhac chan len
sampling_time  = 0.1# thoi gian lay mau 



def Move_robot(type_move: int):
    Math_auto_quadruped.Type_trot(type_move, TM, s, h, sampling_time, L1, L2, L3, H)

