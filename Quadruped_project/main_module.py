import Math_auto_quadruped
from math import pi, sin, cos, asin, acos, atan2, sqrt
import numpy

#information detail Robot
L1 = 5
L2 = 10
L3 = 10
HIGH_stand = 30 #day la chieu cao dat lam gia tri khi dung 

# information about action of Robot
TM_trot = 2
s_step_trot = 3 # khoang cach sai buoc chan
high_step_trot = 3 # do cao nhac chan len de buoc tiep
sampling_time_trot  = 0.5# thoi gian lay mau 

TM_crawl = 1.5 # chu ki buoc
s_step__crawl = 3 # khoang cach sai buoc chan
high_step_crawl = 3 # do cao nhac chan len de buoc tiep 
sampling_time_crawl  = 0.05# thoi gian lay mau 



def Move_Robot(type_move: int, direction: int):
    global TM_trot, s_step_trot, high_step_trot, TM_crawl, s_step__crawl, high_step_crawl,sampling_time_trot, sampling_time_crawl, L1, L2, L3, HIGH_stand
    if type_move == 1:
        Math_auto_quadruped.Type_Trot(direction, TM_trot, s_step_trot, high_step_trot, sampling_time_trot, L1, L2, L3, HIGH_stand)
    if type_move == 2:
        Math_auto_quadruped.Type_Crawl(direction, TM_crawl, s_step__crawl, high_step_crawl, sampling_time_crawl, L1, L2, L3, HIGH_stand)