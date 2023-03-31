#encoding: UTF-8
#!/usr/bin/env python2
import cv2 as cv
import os
import numpy as np
import time
import rospy
from visualization_msgs.msg import Marker
from moving_utils import Movement
import RPi.GPIO as GPIO

from pymycobot.mycobot import MyCobot
# y轴偏移量
pump_y = -60
# x轴偏移量
pump_x = -45 # era 15

class Detect_marker(Movement):
    def __init__(self):
        super(Detect_marker, self).__init__()
        # set cache of real coord
        self.cache_x = self.cache_y = 0  
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(22,GPIO.OUT)
        GPIO.setup(19,GPIO.OUT)
        GPIO.output(22,1)
        GPIO.output(19,1)

        global mc
        mc = MyCobot("/dev/ttyAMA0", 115200)
        
        # Creating a Camera Object
        cap_num = 0
        self.cap = cv.VideoCapture(cap_num)
        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
        # Get ArUco marker params.
        self.aruco_params = cv.aruco.DetectorParameters_create()
        self.calibrationParams = cv.FileStorage("calibrationFileName.xml", cv.FILE_STORAGE_READ)
        # Get distance coefficient.
        self.dist_coeffs = self.calibrationParams.getNode("distCoeffs").mat()

        height = self.cap.get(4)
        focal_length = width = self.cap.get(3)
        center = [width / 2, height / 2]
        # Calculate the camera matrix.
        self.camera_matrix = np.array(
            [
                [focal_length, 0, center[0]],
                [0, focal_length, center[1]],
                [0, 0, 1],
            ],
            dtype=np.float32,
        )
        # init a node and a publisher
        rospy.init_node("encode_marker", anonymous=True)
        self.pub = rospy.Publisher('/cube', Marker, queue_size=1)

        self.marker = Marker()
        self.marker.header.frame_id = "base"
        self.marker.ns = "cube"
        self.marker.type = self.marker.CUBE
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.04
        self.marker.scale.y = 0.04
        self.marker.scale.z = 0.04
        self.marker.color.a = 1
        self.marker.color.r = 1 # era 0.3
        self.marker.color.g = 1 # era 0.3
        self.marker.color.b = 0.3


        # marker position initial
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0.03
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1.0

    def gpio_status(self, flag):
        if flag:
            GPIO.output(22, 0)
            GPIO.output(19, 0)
        else:
            GPIO.output(22, 1)
            GPIO.output(19, 1)

    # Grasping motion
    def move(self, x, y):

        coords = [
            [145.6, -64.9, 285.2, 179.88, 7.67, -177.06],
            [130.1, -155.6, 243.9, 178.99, 5.38, -179.9]
        ]

        # publish marker
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x = (coords[0][0]-x)/1000.0
        self.marker.pose.position.y = (coords[0][1]-y)/1000.0
        self.pub.publish(self.marker)

        # send coordinates to move mycobot
        self.pub_coords(coords[0], 15, 1)
        time.sleep(2)
        self.pub_coords([coords[0][0]-x, coords[0][1]-y, 165, 60.6, -175, -20.8], 15, 1) # era self.pub_coords([coords[0][0]-x, coords[0][1]-y, 240, 178.99, 5.38, -179.9], 15, 1)
        time.sleep(2)
        self.pub_coords([coords[0][0]-x, coords[0][1]-y, 80, 60.6, -175, -20.8], 15, 1) # self.pub_coords([coords[0][0]-x, coords[0][1]-y, 200, 178.99, 5.38, -179.9], 15, 1)
        time.sleep(2)
        self.pub_coords([coords[0][0]-x, coords[0][1]-y, 90, 178.99, 5.38, -100.9], 15, 1) # self.pub_coords([coords[0][0]-x, coords[0][1]-y, 90, 178.99, 5.38, -179.9], 15, 1)
	print [coords[0][0]-x, coords[0][1]-y, 90, 178.99, 5.38, -179.9]
        time.sleep(2)

        self.gripper_status(True) # era self.gpio_status(True)
        print("estado pinza verdadero/cerrada")

        #self.gpio_status(True)
	time.sleep(1)
        self.pub_coords(coords[0], 15, 1)
        time.sleep(3)
        self.pub_coords(coords[1], 15, 1)
        time.sleep(2)
        
        self.gripper_status(False) # anadido por Stefano
        print("estado pinza falso/abierta")

        #self.gpio_status(False)
        # publish marker
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x = coords[1][0]/1000.0
        self.marker.pose.position.y = coords[1][1]/1000.0
        self.pub.publish(self.marker)

        self.pub_coords(coords[0], 30, 1)
        time.sleep(2)

    def gripper_status(self, flag):
        if flag:
            mc.set_encoder(7, 1450)
        else:
            mc.set_encoder(7, 2040)

    # decide whether grab cube
    def decide_move(self, x, y):

        print(x,y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5: # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
	    print x,y
            self.move(x, y)

    # init mycobot
    def init_mycobot(self):
        self.gpio_status(False)
        for _ in range(5):
            print _
            #self.pub_coords([140.2, -63.2, 297.8, 176.45, 13.98, -178.51], 20, 1)
	    self.pub_coords([145.6, -64.9, 285.2, 179.88, 0, -170.06],15,1) # era self.pub_coords([145.6, -64.9, 285.2, 179.88, 7.67, 179.06],20,1)
            time.sleep(0.5)
        


    def run(self):
        global pump_y, pump_x
	self.init_mycobot()
        num = sum_x = sum_y = 0 
        while cv.waitKey(1) < 0:
            success, img = self.cap.read()
            if not success:
                print("It seems that the image cannot be acquired correctly.")
                break

            # transfrom the img to model of gray
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            # Detect ArUco marker.
            corners, ids, rejectImaPoint = cv.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

            if len(corners) > 0:
                if ids is not None:
                    # get informations of aruco
                    ret = cv.aruco.estimatePoseSingleMarkers(
                        corners, 0.03, self.camera_matrix, self.dist_coeffs
                    )
                    # rvec:rotation offset,tvec:translation deviator
                    (rvec, tvec) = (ret[0], ret[1])
                    (rvec - tvec).any()
                    xyz = tvec[0, 0, :]
                    # calculate the coordinates of the aruco relative to the pump

                    xyz = [round(xyz[0]*1000+pump_y, 2), round(xyz[1]*1000+pump_x, 2), round(xyz[2]*1000, 2)]
		    print '000:',xyz[1],xyz[0]

                    for i in range(rvec.shape[0]):
			# draw the aruco on img
                        cv.aruco.drawDetectedMarkers(img, corners)
                        cv.aruco.drawAxis(
                            img,
                            self.camera_matrix,
                            self.dist_coeffs,
                            rvec[i, :, :],
                            tvec[i, :, :],
                            0.03,
                        )
                        if num < 40 :
                            sum_x += xyz[1]-30
                            sum_y += xyz[0]
                            num += 1
                        elif num ==40 :
			    print sum_x,sum_y
                            self.decide_move(sum_x/40.0, sum_y/40.0)
                            num = sum_x = sum_y = 0

            cv.imshow("encode_image", img)

if __name__ == "__main__":
    detect = Detect_marker()
    detect.run()
