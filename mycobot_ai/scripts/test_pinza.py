# encoding:utf-8

from tokenize import Pointfloat

import time
import json
import os
import rospy

import RPi.GPIO as GPIO
from moving_utils import Movement
from pymycobot import MyCobot
from pymycobot import MycobotCommandGenerater
from pymycobot.error import calibration_parameters



class Object_detect(Movement, MycobotCommandGenerater):

    def __init__(self, set_gripper_state=(1,20), camera_x=180, camera_y=0): # it was (self, camera_x=150, camera_y=-10)
        # inherit the parent class
        super(Object_detect, self).__init__()
        # get path of file
        dir_path = os.path.dirname(__file__)
        # 移动角度
        
        # which robot: USB* is m5; ACM* is wio; AMA* is raspi
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(20,GPIO.OUT)
        GPIO.setup(21,GPIO.OUT)
        GPIO.output(20,1)
        GPIO.output(21,1)

        calibration_parameters=0
        MycobotCommandGenerater.set_gripper_ini(self)
        MycobotCommandGenerater.set_gripper_state(self, 1, 20)
        MycobotCommandGenerater.set_gripper_state(self, 0, 20)

        
    def gpio_status(self, flag):
        if flag:
            GPIO.output(20, 0)
            GPIO.output(21, 0)
        else:
            GPIO.output(20, 1)
            GPIO.output(21, 1)

    def gripper_status(self, flag):
        if flag:
            self.set_gripper_state(0, 20)
        else:
            self.set_gripper_state(1, 20)
            

        # open pump
        self.gripper_status(True) # era self.gpio_status(True)
        print("estado pinza verdadero")
        time.sleep(0.5)

        # close pump
        self.gripper_status(False) # era self.gpio_status(False)
        print("estado pinza falso")

        self.set_gripper_ini()
        self.gripper_status(False) # era self.gpio_status(False)
        print("estado pinza falso")

