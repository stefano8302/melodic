from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle

from pymycobot.generate import MycobotCommandGenerater
from pymycobot.error import calibration_parameters

import time
mc = MyCobot("/dev/ttyAMA0", 115200)

def gripper_test(mc):
    print("Start check IO part of api\n")
    flag = mc.is_gripper_moving()
    print("Is gripper moving: {}".format(flag))
    time.sleep(1)
    mc.set_encoder(1, 2048)
    mc.set_encoder(2, 2048)
    mc.set_encoder(3, 2048)
    mc.set_encoder(4, 2048)
    mc.set_encoder(5, 2000)
    mc.set_encoder(6, 2000)
    time.sleep(2)
    
    #mc.set_encoders([1024, 1024, 1024, 1024, 1024, 1024], 20)
    #time.sleep(3)

    print(mc.get_encoder(7))

    #MycobotCommandGenerater.set_gripper_ini()
    #MycobotCommandGenerater.set_gripper_state(1, 20)
    #MycobotCommandGenerater.set_gripper_state(0, 20)

    mc.set_gripper_ini()
    print("inicializado")
    time.sleep(3)
    mc.set_gripper_state(50, 80)
    print("estado 1")
    time.sleep(3)
    mc.set_gripper_state(80, 20)
    print("estado 0")

    #mc.set_encoder(7, 1550) 
    time.sleep(3)
  
    #mc.set_encoder(7, 2000)
    #time.sleep(3)
 
    #mc.set_gripper_value(100, 20)
    #time.sleep(3)
 
if __name__ == "__main__":
 
    mc = MyCobot("/dev/ttyAMA0", 115200)
    mc.set_encoders([2048, 2048, 2048, 2048, 2048, 2048], 10)
    time.sleep(3)
    gripper_test(mc)


