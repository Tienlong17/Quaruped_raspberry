import pygame
import main_module

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
typeMove = 1
isStand = 0
isCheck = 0
#isClose = 0
def Create_Text_Word(a : str, color):
    '''Ham de tao ghi chu~'''
    font = pygame.font.SysFont("sans",30)
    return  font.render(a,True, color)

clock = pygame.time.Clock()
try:
    while running:
        clock.tick(60) # 60
        screen.fill(Color_BackGorund)

        mouse_x,mouse_y = pygame.mouse.get_pos()
        '''Giao dien man hinh'''
        #Draw pannel
        pygame.draw.rect(screen,blue_drark,(12,74,600,170))#chuc nang
        pygame.draw.rect(screen,blue_drark,(12,300,250,35))#kiem tra vat can
        if isCheck == 0:
            pygame.draw.rect(screen,color_button1,(137,300,125,35))
        else:
            pygame.draw.rect(screen,color_button1,(12,300,125,35))
        # button  moving function 
        pygame.draw.rect(screen,color_button2,(120,350,50,50))
        pygame.draw.rect(screen,color_button1,(120,430,50,50))
        pygame.draw.rect(screen,color_button2,(120,500,50,50))
        pygame.draw.rect(screen,color_button2,(50,430,50,50))
        pygame.draw.rect(screen,color_button2,(190,430,50,50))

        #draw information
        screen.blit(Create_Text_Word('Name Robot: Quadruped',blue_light),(300,12))
        screen.blit(Create_Text_Word('Walking Style:',blue_light),(11,42))
        
        screen.blit(Create_Text_Word('Button O: Stand Up',blue_light),(20,80))
        screen.blit(Create_Text_Word('Button  I : Set Lying Coordinates ',blue_light),(20,114))
        screen.blit(Create_Text_Word('Button U: Set Standing Coordinates ',blue_light),(20,148))
        screen.blit(Create_Text_Word('Button L: Lie Down',blue_light),(20,182))
        screen.blit(Create_Text_Word('Button P: Sit Down',blue_light),(300,80))
        
        screen.blit(Create_Text_Word('Button A: Rotate Left,',blue_light),(20,212))
        screen.blit(Create_Text_Word('Button D: Rotate Right',blue_light),(310,212))
        screen.blit(Create_Text_Word('Check Oject:',blue_light),(20,270))
        screen.blit(Create_Text_Word('On',blue_light),(50,305))
        screen.blit(Create_Text_Word('OFF',blue_light),(180,305))
        # draw funtion type of movement
        if typeMove == 1:
            pygame.draw.rect(screen,color_button2,(205,45,150,27))
            screen.blit(Create_Text_Word('1.Trotting',blue_drark),(205,45))
            screen.blit(Create_Text_Word('2.Crawling',blue_light),(360,45))
            screen.blit(Create_Text_Word('3.Trot-Parabol',blue_light),(520,45))
        elif typeMove == 2:
            screen.blit(Create_Text_Word('1.Trotting',blue_light),(205,45))
            pygame.draw.rect(screen,color_button2,(360,45,150,27))
            screen.blit(Create_Text_Word('2.Crawling',blue_drark),(360,45))
            screen.blit(Create_Text_Word('3.Trot-Parabol',blue_light),(520,45))
        elif typeMove == 3:
            screen.blit(Create_Text_Word('1.Trotting',blue_light),(205,45))
            screen.blit(Create_Text_Word('2.Crawling',blue_light),(360,45))
            pygame.draw.rect(screen,color_button2,(510,45,200,27))
            screen.blit(Create_Text_Word('3.Trot-Parabol',blue_drark),(520,45))
        elif 0< typeMove < 4:
            screen.blit(Create_Text_Word('1.Trotting',blue_light),(205,45))
            screen.blit(Create_Text_Word('2.Crawling',blue_light),(360,45))
            screen.blit(Create_Text_Word('3.Trot-Parabol',blue_light),(520,45))
        direction = -1
        
        for event in pygame.event.get():
            # Event by Type of walking Keyboard kieu di 
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    typeMove = 1 # di kieu trot 
                if event.key == pygame.K_2:
                    typeMove = 2 # di kieu crawl
                if event.key == pygame.K_3:
                    typeMove = 3
                # Movement Function 
                if event.key == pygame.K_LEFT: 
                    direction = 4
                if event.key == pygame.K_RIGHT: 
                    direction = 2
                if event.key == pygame.K_UP: 
                    direction = 1
                if event.key == pygame.K_DOWN: 
                    direction = 3
                ##########################################################    
                if event.key == pygame.K_q:         # xoay trai
                    main_module.Spin_Left()
                    pygame.draw.rect(screen,color_button2,(16,212,132,27))
                if event.key == pygame.K_e:         # xoay phai
                    main_module.Spin_Right()
                    pygame.draw.rect(screen,color_button2,(302,210,132,27))
                if event.key == pygame.K_w:
                    main_module.Forward()
                if event.key == pygame.K_s:
                    main_module.Backward()
                if event.key == pygame.K_a:
                    main_module.Left()
                if event.key == pygame.K_d:
                    main_module.Right()
                ###########################################################    
                if event.key == pygame.K_i:         # dua ve vi tri nam
                    pygame.draw.rect(screen,color_button2,(16,80,132,27))
                    main_module.Default_0_degree()
                    isStand = 0
                    
                if event.key == pygame.K_u:         # dua ve vi tri di dung
                    main_module.Default_Stand_Up()
                    isStand = 1
                    pygame.draw.rect(screen,color_button2,(16,148,132,27))
                if event.key == pygame.K_o:         # dung len
                    pygame.draw.rect(screen,color_button2,(16,80,132,27))
                    main_module.Stand_Robot(isStand)
                    isStand = 1
                if event.key == pygame.K_l:         # nam xuong 
                    main_module.Lie_Robot(isStand)
                    isStand = 2
                    pygame.draw.rect(screen,color_button2,(16,180,132,27))
                if event.key == pygame.K_p:         # ngoi 2 chan
                    main_module.Sitdown(isStand)
                    isStand = 3
                #if event.key == pygame.K_t:
                #    main_module.Tempt()
                if event.key == pygame.K_y:
                    if isCheck == 0:
                        isCheck = 1
                    else:
                        isCheck = 0
        if 0< typeMove <4 and isStand == 1 and 0 < direction < 5: 
            if isCheck == 1:
                direc = main_module.Check_Object(direction)
                main_module.Move_Robot(typeMove,direc[0],direc[1]) # 60s 
            else:
                main_module.Move_Robot(typeMove,direction,1)
        if event.type == pygame.QUIT:
            main_module.Lie_Robot(isStand)
            running = False
            print('Exit program')
            main_module.GPIO.cleanup()# giai phong bo nho GPIO

        pygame.display.flip()

    pygame.quit()
except KeyboardInterrupt:
    main_module.Lie_Robot(isStand)
    print('Interupt Ctrl + C')
    main_module.GPIO.cleanup()#giai phong bo nho GPIO
    print('Exit program')