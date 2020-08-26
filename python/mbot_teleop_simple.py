import pygame
from pygame.locals import *
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
import sys
sys.path.append("lcmtypes")
import lcm
from lcmtypes import mbot_motor_pwm_t
FWD_PWM_CMD = 0.3
TURN_PWM_CMD = 0.3

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
pygame.init()
pygame.display.set_caption("MBot TeleOp")
screen = pygame.display.set_mode([640,480])
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.5)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    screen.fill([0,0,0])
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = image.swapaxes(0,1)
    image = cv2.flip(image, -1)
    image = pygame.surfarray.make_surface(image)
    screen.blit(image, (0,0))
    pygame.display.update()
    fwd = 0.0
    turn = 0.0
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            pygame.quit()
            sys.exit()
            cv2.destroyAllWindows()
    key_input = pygame.key.get_pressed()  
    if key_input[pygame.K_LEFT]:
        turn += 1.0
    if key_input[pygame.K_UP]:
        fwd +=1.0
    if key_input[pygame.K_RIGHT]:
        turn -= 1.0
    if key_input[pygame.K_DOWN]:
        fwd -= 1.0
    command = mbot_motor_pwm_t()
    command.left_motor_pwm =  fwd * FWD_PWM_CMD - turn * TURN_PWM_CMD
    command.right_motor_pwm = fwd * FWD_PWM_CMD + turn * TURN_PWM_CMD
    lc.publish("MBOT_MOTOR_PWM",command.encode())
    rawCapture.truncate(0)
