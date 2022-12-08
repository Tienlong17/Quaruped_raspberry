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
typeMove = 2
def Create_Text_Word(a : str):
    '''Ham de tao ghi chu~'''
    font = pygame.font.SysFont("sans",30)
    return  font.render(a,True, blue_light)

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
                    direction = 4
                if event.key == pygame.K_RIGHT: 
                    direction = 2
                if event.key == pygame.K_UP: 
                    direction = 1
                if event.key == pygame.K_DOWN: 
                    direction = 3
                if event.key == pygame.K_d:         # xoay phai
                    main_module.Spin_Right()
                if event.key == pygame.K_i:         # dua ve vi tri nam
                    main_module.Default_0_degree() 
                if event.key == pygame.K_u:         # dua ve vi tri di dung
                    main_module.Default_Stand_Up()              
                if event.key == pygame.K_o:         # dung len
                    main_module.Stand_Robot()  
                if event.key == pygame.K_p:         # nam xuong 
                    main_module.Down_Robot()
        if 0< typeMove <3: # khong truyen so khong thi co di chuyen khong 
            #print('typeMove =',typeMove) Ve giao dien dang di theo kieu nao
            if 0 <= direction < 7:
                #direction = main_module.Check_Object(direction)
                main_module.Move_Robot(typeMove,direction) # 60s 
        
        if event.type == pygame.QUIT:
            running = False
            print('Exit program')
            main_module.GPIO.cleanup()# giai phong bo nho GPIO

        pygame.display.flip()

    pygame.quit()
except KeyboardInterrupt:
    print('Interupt Ctrl + C')
    main_module.GPIO.cleanup()#giai phong bo nho GPIO
    print('Exit program')