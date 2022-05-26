import Math_auto_quadruped
from math import pi, sin, cos, asin, acos, atan2, sqrt
import numpy

#information detail Robot
L1 = 8
L2 = 18.5
L3 = 21.5
HIGH_stand = 30 #day la chieu cao dat lam gia tri khi dung 

# information about action of Robot
TM_trot = 2
s_step_trot = 10 # khoang cach sai buoc chan
high_step_trot = 6 # do cao nhac chan len de buoc tiep

TM_crawl = 1.5 # chu ki buoc
s_step__crawl = 10 # khoang cach sai buoc chan
high_step_crawl = 6 # do cao nhac chan len de buoc tiep 
sampling_time  = 0.05# thoi gian lay mau 



def Move_Robot(type_move: int, direction: int):
    global TM_trot, s_step_trot, high_step_trot, TM_crawl, s_step__crawl, high_step_crawl, sampling_time, L1, L2, L3, HIGH_stand
    if type_move == 1:
        Math_auto_quadruped.Type_Trot(direction, TM_trot, s_step_trot, high_step_trot, sampling_time, L1, L2, L3, HIGH_stand)
    if type_move == 2:
        Math_auto_quadruped.Type_Crawl(direction, TM_crawl, s_step__crawl, high_step_crawl, sampling_time, L1, L2, L3, HIGH_stand)
