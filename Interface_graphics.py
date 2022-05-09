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

def creat_text_word(a : str):
    '''Ham de tao ghi chu~'''
    font = pygame.font.SysFont("sans",30)
    return  font.render(a,True, blue_light)

clock = pygame.time.Clock()

def get_Button(mouse_x,mouse_y):
    x = 0
    if (120 < mouse_x < 170) and (350 < mouse_y < 400):
        x = 1

    if (120 < mouse_x < 170) and (500 < mouse_y < 550):
        x = 2

    if (50 < mouse_x < 100) and (430 < mouse_y < 480):
        x = 3

    if (190 < mouse_x < 240) and (430 < mouse_y < 480):
        x = 4

    if (120 < mouse_x < 170) and (430 < mouse_y < 480):
        x = 0
    return x

huong =0
isClick = 0

while running:
    clock.tick(60) # 60
    screen.fill(Color_BackGorund)

    mouse_x,mouse_y = pygame.mouse.get_pos()
    '''Giao dien man hinh'''
    # button chuc nang di chuyen 
    pygame.draw.rect(screen,color_button2,(120,350,50,50))
    pygame.draw.rect(screen,color_button1,(120,430,50,50))
    pygame.draw.rect(screen,color_button2,(120,500,50,50))
    pygame.draw.rect(screen,color_button2,(50,430,50,50))
    pygame.draw.rect(screen,color_button2,(190,430,50,50))

    #Ve cac information
    screen.blit(creat_text_word('Status Robot:'),(12,12))
    screen.blit(creat_text_word('Walking Style:'),(11,42))


    for event in pygame.event.get():
        if event.type == pygame.MOUSEBUTTONDOWN:
            isClick = 1
        if event.type == pygame.MOUSEBUTTONUP:
            isClick = 0

    if isClick == 1:
        huong = get_Button(mouse_x,mouse_y)
    else:
        huong = 0

    print(huong)

    main_module.Move_robot(huong) # 60s 


    
    if event.type == pygame.QUIT:
        running = False

    pygame.display.flip()

pygame.quit()


