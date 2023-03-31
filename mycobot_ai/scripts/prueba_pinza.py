from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
import time

mc = MyCobot("/dev/ttyAMA0", 115200) # era mc = MyCobot("COM167", 115200)
mc.power_on()

mc.set_servo_calibration(7)
time.sleep(3)
encoder=mc.get_encoder(7)
time.sleep(3)
print(encoder)
mc.set_encoder(7, 1500)
time.sleep(3)
encoder=mc.get_encoder(7)
time.sleep(3)
print(encoder)
time.sleep(3)
