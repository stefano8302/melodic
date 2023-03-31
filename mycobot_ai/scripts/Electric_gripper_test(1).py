from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
import time
mc = MyCobot("/dev/ttyAMA0", 115200)

def gripper_test(mc):
    print("Start check IO part of api\n")
    flag = mc.is_gripper_moving()
    print("Is gripper moving: {}".format(flag))
    time.sleep(1)
    #mc.set_encoder(1, 2048)
    #time.sleep(2)
    
    #mc.set_encoders([1024, 1024, 1024, 1024, 1024, 1024], 20)
    #time.sleep(3)

    print(mc.get_encoder(1))

    mc.set_encoder(7, 2048) 
    time.sleep(3)
  
    mc.set_encoder(7, 1300)
    time.sleep(3)
 
    mc.set_gripper_value(2048, 70)
    time.sleep(3)
 
if __name__ == "__main__":
 
    mc = MyCobot("AMA0", 115200)
    #mc.set_encoders([2048, 2048, 2048, 2048, 2048, 2048], 20)
    time.sleep(3)
    gripper_test(mc)


