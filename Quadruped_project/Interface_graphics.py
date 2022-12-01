import pygame
import main_module

import RPi.GPIO as GPIO
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


pygame.init()
pygame.display.set_caption('Code-mau-giao-dien')
screen = pygame.display.set_mode((900, 600))



running = True
Color_BackGorund = (23, 62, 130)
color_button1 = (232, 53, 21)
color_button2 = (232, 232, 21)
blue_light = (21, 232, 204)
blue_drark = (2, 38, 102)

direction = -1
typeMove = 0
def Create_Text_Word(a : str):
    '''Ham de tao ghi chu~'''
    font = pygame.font.SysFont("sans",30)
    return  font.render(a,True, blue_light)
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

#set GPIO Pins
trigger = [6,13,19,26]
echo = [12,16,20,21]

 
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
    if 0 < distance <= 10: # co vat can 
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
        return dist # khong co vat can thi duoc di <- se de ham di chuyen 
    else:
        check_object2 = Call_SR4(dist - 1)
        if check_object2 == False: # False nghia la khong co vat can va Tru la co vat can 
            print("thuc hien chuyen dong lan 2, huong:",dist - 1) # khong co vat can thi duoc di
            return dist - 1
        else:
            check_object3 = Call_SR4(dist + 1)
            if check_object3 == False: # False nghia la khong co vat can va Tru la co vat can 
                print("thuc hien chuyen dong lan 3, huong:",(dist + 1)) # khong co vat can thi duoc di
                return dist + 1
            else:
                check_object4 = Call_SR4(dist + 2)
                if check_object4 == False: # False nghia la khong co vat can va Tru la co vat can 
                    print("thuc hien chuyen dong lan 4, huong:",dist + 2) # khong co vat can thi duoc di
                    return dist + 2
                else:
                    return -1


clock = pygame.time.Clock()
try:
    while running:
        clock.tick(60) # 60
        screen.fill(Color_BackGorund)

        mouse_x,mouse_y = pygame.mouse.get_pos()
        '''Giao dien man hinh'''
        # button  moving function 
        pygame.draw.rect(screen,color_button2,(120,350,50,50))
        pygame.draw.rect(screen,color_button1,(120,430,50,50))
        pygame.draw.rect(screen,color_button2,(120,500,50,50))
        pygame.draw.rect(screen,color_button2,(50,430,50,50))
        pygame.draw.rect(screen,color_button2,(190,430,50,50))

        #draw information
        screen.blit(Create_Text_Word('Name Robot:'),(12,12))
        screen.blit(Create_Text_Word('Quadruped'),(200,12))
        screen.blit(Create_Text_Word('Walking Style:'),(11,42))
        
        # draw funtion type of movement 
        screen.blit(Create_Text_Word('1.Trotting'),(205,45))
        screen.blit(Create_Text_Word('2.Crawling'),(360,45))
        direction = -1 
        for event in pygame.event.get():
            # Event by Type of walking Keyboard kieu di 
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    typeMove = 1 # di kieu trot 
                if event.key == pygame.K_2:
                    typeMove = 2 # di kieu crawl 
                # Movement Function 
                if event.key == pygame.K_LEFT: 
                    #pygame.draw.rect(screen,blue_drark,(50,430,50,50)) # show di len di xuong
                    direction = 4
                if event.key == pygame.K_RIGHT: 
                    direction = 2
                    #pygame.draw.rect(screen,blue_drark,(190,430,50,50))
                if event.key == pygame.K_UP: 
                    direction = 1
                    #pygame.draw.rect(screen,blue_drark,(120,350,50,50))
                if event.key == pygame.K_DOWN: 
                    direction = 3
                    #pygame.draw.rect(screen,blue_drark,(120,500,50,50))
                if event.key == pygame.K_p: # 7 dung stop 
                    #pygame.draw.rect(screen,blue_drark,(120,430,50,50))
                    direction = -1
        if 0< typeMove <3: # khong truyen so khong thi co di chuyen khong 
            #print('typeMove =',typeMove) Ve giao dien dang di theo kieu nao
            if 0 <= direction < 7:
                direction = Check_Object(direction)
                main_module.Move_Robot(typeMove,direction) # 60s 
        
        if event.type == pygame.QUIT:
            running = False
            print('Exit program')
            GPIO.cleanup()# giai phong bo nho GPIO

        pygame.display.flip()

    pygame.quit()
except KeyboardInterrupt:
    print('Interupt Ctrl + C')
    GPIO.cleanup()#giai phong bo nho GPIO
    print('Exit program')
