#encoding: UTF-8
#!/usr/bin/env python3

import rospy
import time
from moving_utils import Movement
from std_srvs.srv import Empty


class Pump(Movement):

    def __init__(self):
        super(Pump, self).__init__()
        rospy.init_node("pump", anonymous=True)

    def run(self):
        self.pub_pump(False)
        time.sleep(1)
        self.pub_pump(True)
        time.sleep(5)
        self.pub_pump(False)

#    def gpiod(self):
#        import RPi.GPIO as GPIO
#        GPIO.setwarnings(False)
#        GPIO.setmode(GPIO.BCM)
#        GPIO.setup(20, GPIO.OUT)
#        GPIO.setup(21, GPIO.OUT)
#        GPIO.output(20, 0)
#        GPIO.output(21, 0)
#        time.sleep(3)
#        print "close"
#        GPIO.output(20, 1)
#        GPIO.output(21, 1)
        
    # Calling the suction gripper service ON
    def gripper_control_on(self):
      rospy.wait_for_service('on')
      try:
        
        #create handler to call the service
        suction_on = rospy.ServiceProxy('on', Empty)
        print ("ServiceProxy success ...")

        #call the service by passing the request to the handler
        #it doesn't require to have the same names as in the .srv
        #But I did it for clarity
        response = suction_on()
        print ("rosservice call success")

        #return the whole response
        return response

      except rospy.ServiceException as e:
        print ("Service call failed: %s")%e
        
    # Calling the suction gripper service OFF
    def gripper_control_off(self):
      rospy.wait_for_service('off')
      try:
        
        #create handler to call the service
        suction_off = rospy.ServiceProxy('off', Empty)
        print ("ServiceProxy success ...")

        #call the service by passing the request to the handler
        #it doesn't require to have the same names as in the .srv
        #But I did it for clarity
        response = suction_off()
        print ("rosservice call success")

        #return the whole response
        return response

      except rospy.ServiceException as e:
        print ("Service call failed: %s")%e

#if __name__ == "__main__":
#    pump = Pump()
#    pump.gpiod()
    
if __name__ == "__main__":
    pump = Pump()
    pump.gripper_control_on()
    time.sleep(3)
    print ("close")
    pump.gripper_control_off()
