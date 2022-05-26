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
s_trot_step = 10 # khoang cach sai buoc chan
high_trot_step = 6 # do cao nhac chan len de buoc tiep

TM_crawl = 1.5 # chu ki buoc
s_crawl_step = 10 # khoang cach sai buoc chan
high_crawl_step = 6 # do cao nhac chan len de buoc tiep 
sampling_time  = 0.05# thoi gian lay mau 



def Move_Robot(type_move: int, direction: int):
    global TM_trot, s_trot_step, high_trot_step, TM_crawl, s_crawl_step, high_crawl_step, sampling_time, L1, L2, L3, HIGH_stand
    if type_move == 1:
        Math_auto_quadruped.Type_Trot(direction, TM_trot, s_trot_step, high_trot_step, sampling_time, L1, L2, L3, HIGH_stand)
    if type_move == 2:
        Math_auto_quadruped.Type_Crawl(direction, TM_crawl, s_crawl_step, high_crawl_step, sampling_time, L1, L2, L3, HIGH_stand)