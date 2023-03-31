# encoding:utf-8

from tokenize import Pointfloat
import cv2
import numpy as np
import time
import json
import os
import rospy
from visualization_msgs.msg import Marker
import RPi.GPIO as GPIO
from moving_utils import Movement

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"
# Adaptive seeed


class Object_detect(Movement):

    def __init__(self, camera_x=150, camera_y=-10):
        # inherit the parent class
        super(Object_detect, self).__init__()
        # get path of file
        dir_path = os.path.dirname(__file__)
        # 移动角度
        self.move_angles = [
            [-7.11, -6.94, -55.01, -24.16, 0, -38.84],  # init the point
            [-1.14, -10.63, -87.8, 9.05, -3.07, -37.7],  # point to grab
            [17.4, -10.1, -87.27, 5.8, -2.02, -37.7],  # point to grab
        ]
        # 移动坐标
        self.move_coords = [
            [120.1, -141.6, 240.9, -173.34, -8.15, -83.11],  # above the red bucket
            # above the yello bucket
            [215.2, -127.8, 260.9, -157.51, -17.5, -71.18],
            [209.7, -18.6, 230.4, -168.48, -9.86, -39.38],
            [196.9, -64.7, 232.6, -166.66, -9.44, -52.47],
            [126.6, -118.1, 305.0, -157.57, -13.72, -75.3],
        ]
        # which robot: USB* is m5; ACM* is wio; AMA* is raspi
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(22,GPIO.OUT)
        GPIO.setup(19,GPIO.OUT)
        GPIO.output(22,1)
        GPIO.output(19,1)

        # choose place to set cube
        self.color = 0
        # parameters to calculate camera clipping parameters
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord
        self.cache_x = self.cache_y = 0
        # set color HSV
        self.HSV = {
            "yellow": [np.array([11, 115, 70]), np.array([40, 255, 245])],
            "red": [np.array([0, 43, 46]), np.array([8, 255, 255])],
            "green": [np.array([35, 43, 46]), np.array([77, 255, 255])],
            "blue": [np.array([100, 43, 46]), np.array([124, 255, 255])],
            "cyan": [np.array([78, 43, 46]), np.array([99, 255, 255])],
        }
        # use to calculate coord between cube and mycobot
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # The coordinates of the grab center point relative to the mycobot
        self.camera_x, self.camera_y = camera_x, camera_y
        # The coordinates of the cube relative to the mycobot
        self.c_x, self.c_y = 0, 0
        # The ratio of pixels to actual values
        self.ratio = 0
        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Get ArUco marker params.
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # init a node and a publisher
        rospy.init_node("marker", anonymous=True)
        self.pub = rospy.Publisher('/cube', Marker, queue_size=10)
        # init a Marker
        self.marker = Marker()
        self.marker.header.frame_id = "base"
        self.marker.ns = "cube"
        self.marker.type = self.marker.CUBE
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.04
        self.marker.scale.y = 0.04
        self.marker.scale.z = 0.04
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
        self.marker.color.r = 1.0

        # marker position initial
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0.03
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1.0

    # publish marker
    def pub_marker(self, x, y, z=0.03):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        self.marker.color.g = self.color
        self.pub.publish(self.marker)

    def gpio_status(self, int):
        if int == 1:
            GPIO.output(22, 0)
            GPIO.output(19, 1)
        elif int == 2:
            GPIO.output(22, 1)
            GPIO.output(19, 0)
        elif int == 3:
            GPIO.output(22, 0)
            GPIO.output(19, 0)
        elif int == 4:
            GPIO.output(22, 1)
            GPIO.output(19, 1)

    # Grasping motion
    def move(self):
        # send Angle to move mycobot
        #print color
        self.pub_angles(self.move_angles[0], 20)
        time.sleep(1.5)
        #self.pub_angles(self.move_angles[1], 20)
        #time.sleep(1.5)
        #self.pub_angles(self.move_angles[2], 20)
        #time.sleep(1.5)
        # send coordinates to move mycobot
        #self.pub_coords([x, y, 165,  -178.9, -1.57, -25.95], 20, 1)
        #time.sleep(1.5)
        #self.pub_coords([x, y, 90,  -178.9, -1.57, -25.95], 20, 1)
        #time.sleep(1.5)
        # open pump
        self.gpio_status(3)
        print("bomba vacio encendida")
        time.sleep(1)
        
        # close pump
        self.gpio_status(4)
        print("bomba vacio apagada")
        time.sleep(1)
        self.gpio_status(1)
        print("bomba vacio apagada")
        time.sleep(1)
        self.gpio_status(2)
        print("bomba vacio apagada")
        #if color == 1:
        #    self.pub_marker(
        #        self.move_coords[color][0]/1000.0+0.04, self.move_coords[color][1]/1000.0-0.02)
        #elif color == 0:
        #    self.pub_marker(
        #        self.move_coords[color][0]/1000.0+0.03, self.move_coords[color][1]/1000.0)
        #self.pub_angles(self.move_angles[0], 20)
        #time.sleep(3)

    # decide whether grab cube

    def decide_move(self, x, y, color):

        print(x, y, self.cache_x, self.cache_y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            
            self.move(x+13, y+13, color)

    # init mycobot
    def run(self):

        for _ in range(5):
            self.pub_angles([-7.11, -6.94, -55.01, -24.16, 0, -38.84], 20)
            print(_)
            time.sleep(0.5)
        self.gpio_status(False)
        print("bomba vacio apagada")


    # draw aruco
    def draw_marker(self, img, x, y):
        # draw rectangle on img
        cv2.rectangle(
            img,
            (x - 20, y - 20),
            (x + 20, y + 20),
            (0, 255, 0),
            thickness=2,
            lineType=cv2.FONT_HERSHEY_COMPLEX,
        )
        # add text on rectangle
        cv2.putText(img, "({},{})".format(x, y), (x, y),
                    cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (243, 0, 0), 2,)

    # get points of two aruco
    def get_calculate_params(self, img):
        # Convert the image to a gray image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Detect ArUco marker.
        corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        """
        Two Arucos must be present in the picture and in the same order.
        There are two Arucos in the Corners, and each aruco contains the pixels of its four corners.
        Determine the center of the aruco by the four corners of the aruco.
        """
        if len(corners) > 0:
            if ids is not None:
                if len(corners) <= 1 or ids[0] == 1:
                    return None
                x1 = x2 = y1 = y2 = 0
                point_11, point_21, point_31, point_41 = corners[0][0]
                x1, y1 = int((point_11[0] + point_21[0] + point_31[0] + point_41[0]) / 4.0), int(
                    (point_11[1] + point_21[1] + point_31[1] + point_41[1]) / 4.0)
                point_1, point_2, point_3, point_4 = corners[1][0]
                x2, y2 = int((point_1[0] + point_2[0] + point_3[0] + point_4[0]) / 4.0), int(
                    (point_1[1] + point_2[1] + point_3[1] + point_4[1]) / 4.0)
                return x1, x2, y1, y2
        return None

    # set camera clipping parameters
    def set_cut_params(self, x1, y1, x2, y2):
        self.x1 = int(x1)
        self.y1 = int(y1)
        self.x2 = int(x2)
        self.y2 = int(y2)
        print(self.x1, self.y1, self.x2, self.y2)

    # set parameters to calculate the coords between cube and mycobot
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 220.0/ratio

    # calculate the coords between cube and mycobot
    def get_position(self, x, y):
        return ((y - self.c_y)*self.ratio + self.camera_x), ((x - self.c_x)*self.ratio + self.camera_y)

    """
    Calibrate the camera according to the calibration parameters.
    Enlarge the video pixel by 1.5 times, which means enlarge the video size by 1.5 times.
    If two ARuco values have been calculated, clip the video.
    """

    def transform_frame(self, frame):
        # enlarge the image by 1.5 times
        fx = 1.5
        fy = 1.5
        frame = cv2.resize(frame, (0, 0), fx=fx, fy=fy,
                           interpolation=cv2.INTER_CUBIC)
        if self.x1 != self.x2:
            # the cutting ratio here is adjusted according to the actual situation
            frame = frame[int(self.y2*0.2):int(self.y1*1.15),
                          int(self.x1*0.7):int(self.x2*1.15)]
        return frame

    # detect cube color
    def color_detect(self, img):
        # set the arrangement of color'HSV
        x = y = 0
        for mycolor, item in self.HSV.items():
            redLower = np.array(item[0])
            redUpper = np.array(item[1])
            # transfrom the img to model of gray
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # wipe off all color expect color in range
            mask = cv2.inRange(hsv, item[0], item[1])
            # a etching operation on a picture to remove edge roughness
            erosion = cv2.erode(mask, np.ones((1, 1), np.uint8), iterations=2)
            # the image for expansion operation, its role is to deepen the color depth in the picture
            dilation = cv2.dilate(erosion, np.ones(
                (1, 1), np.uint8), iterations=2)
            # adds pixels to the image
            target = cv2.bitwise_and(img, img, mask=dilation)
            # the filtered image is transformed into a binary image and placed in binary
            ret, binary = cv2.threshold(dilation, 127, 255, cv2.THRESH_BINARY)
            # get the contour coordinates of the image, where contours is the coordinate value, here only the contour is detected
            contours, hierarchy = cv2.findContours(
                dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                # do something about misidentification
                boxes = [
                    box
                    for box in [cv2.boundingRect(c) for c in contours]
                    if min(img.shape[0], img.shape[1]) / 10
                    < min(box[2], box[3])
                    < min(img.shape[0], img.shape[1]) / 1
                ]
                if boxes:
                    for box in boxes:
                        x, y, w, h = box
                    # find the largest object that fits the requirements
                    c = max(contours, key=cv2.contourArea)
                    # get the lower left and upper right points of the positioning object
                    x, y, w, h = cv2.boundingRect(c)
                    # locate the target by drawing rectangle
                    cv2.rectangle(img, (x, y), (x+w, y+h), (153, 153, 0), 2)
                    # calculate the rectangle center
                    x, y = (x*2+w)/2, (y*2+h)/2
                    # calculate the real coordinates of mycobot relative to the target
                    if mycolor == "yellow":
                        self.color = 1
                    elif mycolor == "red":
                        self.color = 0
                    else:
                        self.color = 1

        if abs(x) + abs(y) > 0:
            return x, y
        else:
            return None


if __name__ == "__main__":
    # open the camera
    #cap_num = 0
    #cap = cv2.VideoCapture(cap_num)
    #if not cap.isOpened():
    #    cap.open()
    # init a class of Object_detect
    detect = Object_detect()
    # init mycobot
    detect.run()
    detect.move()