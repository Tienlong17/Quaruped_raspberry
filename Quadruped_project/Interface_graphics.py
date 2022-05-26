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

direction =0
isClick = 0
typeMove = 0
def Create_Text_Word(a : str):
    '''Ham de tao ghi chu~'''
    font = pygame.font.SysFont("sans",30)
    return  font.render(a,True, blue_light)
def Create_Button(name: str, position_x: float,position_y: float):
    text = Create_Text_Word(name)
    text_box = text.get_rect()
    pygame.draw.rect(screen,blue_drark,(position_x,position_y,text_box[2],text_box[3]))
    screen.blit(text, (position_x,position_y))
    if is_mouse_on_text(position_x, position_y, text_box[2],text_box[3]):
        pygame.draw.rect(screen,(100,100,100), (position_x,position_y,text_box[2],text_box[3]))
        pygame.draw.line(screen, (0,0,255), (position_x, position_y + text_box[3]), (position_x +text_box[2], position_y + text_box[3]))
        screen.blit(text, (position_x,position_y))

def is_mouse_on_text(position_x,position_y,position_z,position_t):
        mouse_x, mouse_y = pygame.mouse.get_pos()
        if position_x < mouse_x < position_x + position_z and position_y < mouse_y < position_y + position_t:
            return True
        else:
            return False
        
def Click_Button():
    text1 = Create_Text_Word(' Trotting ')
    text_box1 = text1.get_rect()
    position_x1,position_y1 = 200,42
    text2 = Create_Text_Word(' Crawling ')
    text_box2 = text2.get_rect()
    position_x2,position_y2 = 320,42
    global typeMove
    if position_x1 < mouse_x < position_x1 + text_box1[2] and position_y1 < mouse_y < position_y1 + text_box1[3]:
        typeMove = 1
    if position_x2 < mouse_x < position_x2 + text_box2[2] and position_y2< mouse_y < position_y2 + text_box2[3]:
        typeMove = 2

clock = pygame.time.Clock()

def Get_Button(mouse_x,mouse_y):
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
    
    # draw button
    Create_Button(' Trotting ',200,42)
    Create_Button(' Crawling ',320,42)
    for event in pygame.event.get():
        if event.type == pygame.MOUSEBUTTONDOWN:
            isClick = 1
        if event.type == pygame.MOUSEBUTTONUP:
            isClick = 0

    if isClick == 1:
        direction = Get_Button(mouse_x,mouse_y)
        Click_Button()
    else:
        direction = 0

    print('direction',direction)
    print('typeMove',typeMove)
    main_module.Move_Robot(typeMove,direction) # 60s 


    
    if event.type == pygame.QUIT:
        running = False

    pygame.display.flip()

pygame.quit()
