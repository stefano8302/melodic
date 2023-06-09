#! /usr/bin/env python 
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
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
#from pymycobot import MycobotCommandGenerater
#from pymycobot.error import calibration_parameters

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"
# Adaptive seeed


class Object_detect(Movement):

    def __init__(self, camera_x=190, camera_y=-100): # it was (self, camera_x=150, camera_y=-10)
        # inherit the parent class
        super(Object_detect, self).__init__()
        # get path of file
        dir_path = os.path.dirname(__file__)
        # 移动角度
        self.move_angles = [
            [0, 0, 0, 0, 0, 0],  # init the point; era [0, -6.94, -55.01, -24.16, 0, 0]
            [36.4, -60.8, -65.5, -50.6, -90, 166.8],  # point to grab; [1] it was [-1.14, -10.63, -87.8, 9.05, -3.07, -37.7]
            [-10, -8.1, 0.27, 0, -2.02, 0],  # point to grab; [2] it was [17.4, -10.1, -87.27, 5.8, -2.02, -37.7]
        ]
        # 移动坐标
        self.move_coords = [
            [120, -170, 378, -87, 1, -110],  # above the red bucket it was [120.1, -141.6, 240.9, -173.34, -8.15, -83.11]
            [212.2, 125, 360.9, -80, 0, -91.18], # above the yellow bucket. era [215.2, -127.8, 260.9, -157.51, -17.5, -71.18]
            [209.7, -40.6, 230.4, -90.48, -9.86, -39.38], # it was [209.7, -18.6, 230.4, -168.48, -9.86, -39.38]
            [196.9, -64.7, 232.6, -90.66, -9.44, -52.47], # it was [196.9, -64.7, 232.6, -166.66, -9.44, -52.47]
            [80.6, -118.1, 305.0, -157.57, -13.72, -75.3], # era [126.6, -118.1, 305.0, -157.57, -13.72, -75.3]
        ]
        # which robot: USB* is m5; ACM* is wio; AMA* is raspi
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(20,GPIO.OUT)
        GPIO.setup(21,GPIO.OUT)
        GPIO.output(20,1)
        GPIO.output(21,1)

        #calibration_parameters=0
        #MycobotCommandGenerater.set_gripper_ini(self)
        #MycobotCommandGenerater.set_gripper_state(self, 1, 20)
        #MycobotCommandGenerater.set_gripper_state(self, 0, 20)
        global mc
        mc = MyCobot("/dev/ttyAMA0", 115200) # era mc = MyCobot("COM167", 115200)
        #mc.power_on()

        #mc.set_servo_calibration(7)
        #time.sleep(3)
        #encoder=mc.get_encoder(7)
        #time.sleep(3)
        #print(encoder)
        #mc.set_encoder(7, 2040)
        #time.sleep(2)
        #mc.set_encoder(7, 1450)
        #time.sleep(2)

        # choose place to set cube
        self.color = 0
        # parameters to calculate camera clipping parameters
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord
        self.cache_x = self.cache_y = 0
        # set color HSV
        self.HSV = {
            "Amarillo": [np.array([11, 115, 70]), np.array([40, 255, 245])],
            "Rojo": [np.array([165, 43, 46]), np.array([180, 255, 255])],
            #"Verde": [np.array([35, 43, 46]), np.array([70, 255, 255])],
            "Fucsia": [np.array([130, 150, 46]), np.array([170, 255, 255])],
            "Turquesa": [np.array([80, 43, 46]), np.array([90, 255, 255])],
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
        self.pub = rospy.Publisher('/cube', Marker, queue_size=1)
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
        self.marker.pose.position.z = 0.01
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1.0

    # publish marker
    def pub_marker(self, x, y, z=0.01):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        self.marker.color.g = self.color
        self.pub.publish(self.marker)

    def gpio_status(self, flag):
        if flag:
            GPIO.output(20, 0)
            GPIO.output(21, 0)
        else:
            GPIO.output(20, 1)
            GPIO.output(21, 1)

    def gripper_status(self, flag):
        if flag:
            mc.set_encoder(7, 1450)
        else:
            mc.set_encoder(7, 2040)
            
    # Grasping motion
    def move(self, x, y, color):
        # send Angle to move mycobot
        print color
        #self.pub_angles(self.move_angles[0], 15) # estos primeros 3 movimientos son para que el robot mire a donde esta el objeto si tiene montada la camara el el TCP
        #time.sleep(5)
        #self.pub_angles(self.move_angles[1], 15)
        #time.sleep(5)
        #self.pub_angles(self.move_angles[2], 15)
        #time.sleep(10)
        # send coordinates to move mycobot
        self.pub_coords([(x+15), (y+10), 165, 60.6, -175, -20.8], 15, 1) # era ([x, y, 165,  -178.9, -1.57, -25.95], 15, 1)
        time.sleep(5)
        self.pub_coords([(x+15), (y+10), 80, 60.6, -175, -20.8], 15, 1) # era ([x, y, 90,  -178.9, -1.57, -25.95], 15, 1)
        time.sleep(3)
        # open pump
        self.gripper_status(True) # era self.gpio_status(True)
        print("estado pinza verdadero/cerrada")
        #time.sleep(1.5)
        #self.pub_angles(self.move_angles[2], 15)
        #time.sleep(5)
        self.pub_marker(
            self.move_coords[2][0]/1000.0, self.move_coords[2][1]/1000.0, self.move_coords[2][2]/1000.0)

        #self.pub_angles(self.move_angles[1], 15)
        #time.sleep(4.5)
        self.pub_marker(
            self.move_coords[3][0]/1000.0, self.move_coords[3][1]/1000.0, self.move_coords[3][2]/1000.0)
        self.gpio_status(True)
        self.gripper_status(True) # anadido por Stefano
        print("estado pinza verdadero/cerrada")
        time.sleep(1.5)

        #self.pub_angles(self.move_angles[0], 15)
        #time.sleep(5)
        self.pub_marker(
            self.move_coords[4][0]/1000.0, self.move_coords[4][1]/1000.0, self.move_coords[4][2]/1000.0)

        self.pub_coords(self.move_coords[color], 15, 1) # el movimiento hacia el contenedor correspondiente al color detectado
        self.pub_marker(self.move_coords[color][0]/1000.0, self.move_coords[color]
                        [1]/1000.0, self.move_coords[color][2]/1000.0)
        time.sleep(5)
        # close pump
        self.gpio_status(False)
        self.gripper_status(False) # anadido por Stefano
        print("estado pinza falso/abierta")
        time.sleep(2)
        if color == 1: # el color detectado es amarillo
            self.pub_marker(
                self.move_coords[color][0]/1000.0+0.05, self.move_coords[color][1]/1000.0-0.02) #el color puede ser solo 0 o 1 y identifica si escoger el move_coords de la primera linea [0] o de la segunda [1]
        # era self.move_coords[color][0]/1000.0+0.04, self.move_coords[color][1]/1000.0-0.02)
        elif color == 0: # el color detectado es rojo
            self.pub_marker(
                self.move_coords[color][0]/1000.0+0.02, self.move_coords[color][1]/1000.0-0.00)
        # era self.move_coords[color][0]/1000.0+0.03, self.move_coords[color][1]/1000.0
        self.pub_angles(self.move_angles[2], 15) # era 20, manda el robot volver a su posicion inicial
        time.sleep(5)

    # decide whether grab cube

    def decide_move(self, x, y, color):

        print(x, y, self.cache_x, self.cache_y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            
            self.move(x+8, y+8, color) # era self.move(x+13, y+13, color)

    # init mycobot
    def run(self):

        for _ in range(5): # el "_" es la variable contador
            #self.pub_angles([0, 0, 0, 0, 0, 0], 15) # era [0, -6.94, -55.01, -24.16, 0, 0]
            print(_)
            time.sleep(1.2)
        
        self.gpio_status(False)
        self.gripper_status(False) # anadido por Stefano
        print("estado pinza falso/abierta")
        time.sleep(2)
        # mover el robot a la posicion inicial para detectar los arucos
        self.pub_angles(self.move_angles[2], 15)
        time.sleep(4.5)

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
        cv2.putText(img, "{}, {}".format(x, y), (x, y),
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

        Localiza las 4 esquinas de cada aruco y calcula el centro geometrico.
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
        self.ratio = 220.0/ratio # imagino que 220 son la z en mm de donde se supone este la punta del robot 

    # calculate the coords between cube and mycobot
    def get_position(self, x, y):
        return ((y - self.c_y)*self.ratio + self.camera_x), ((x - self.c_x)*self.ratio + self.camera_y) # estas son las x y que salen en la ventana terminal

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
            frame = frame[int(self.y2*0.15):int(self.y1*1.2),
                          int(self.x1*0.65):int(self.x2*1.2)]
        return frame

    # detect cube color
    def color_detect(self, img):

        

        # set the arrangement of color'HSV
        x = y = 0
        for mycolor, item in self.HSV.items():
            redLower = np.array(item[0])
            redUpper = np.array(item[1])
            yellowLower = np.array(item[0])
            yellowUpper = np.array(item[1])
            cyanLower = np.array(item[0])
            cyanUpper = np.array(item[1])
            greenLower = np.array(item[0])
            greenUpper = np.array(item[1])
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
                    if mycolor == "Amarillo":
                        self.color = 1
                        print("Amarillo")
                    elif mycolor == "Rojo":
                        self.color = 0
                        print("Rojo")
                    elif mycolor == "Turquesa":
                        self.color = 0
                        print("Turquesa")
                    elif mycolor == "Verde":
                        self.color = 0
                        print("Verde")
                    else:
                        self.color = 1
                    
                    cv2.rectangle(
                        img,
                        (x-w/2, y-h/2),
                        (x+w/2, y+h/2),
                        (0, 230, 0),
                        thickness=5,
                    )
                    cv2.putText(
                        img,
                        "x={}: y={}, Color={}".format(x,y,mycolor),
                        (20, 40 - 10),
                        cv2.FONT_HERSHEY_COMPLEX_SMALL,
                        1,
                        (243, 0, 0),
                        2,
                    )

        if abs(x) + abs(y) > 0:
            return x, y
        else:
            return None


if __name__ == "__main__":
    # open the camera
    cap_num = 0
    cap = cv2.VideoCapture(cap_num)
    if not cap.isOpened():
        cap.open()
    # init a class of Object_detect
    detect = Object_detect()
    # init mycobot
    detect.run()

    _init_ = 20  #
    init_num = 0
    nparams = 0
    num = 0
    real_sx = real_sy = 0
    while cv2.waitKey(1) < 0:
       # read camera
        _, frame = cap.read()
        # deal img
        frame = detect.transform_frame(frame)

        if _init_ > 0:
            _init_ -= 1
            continue
        # calculate the parameters of camera clipping
        if init_num < 20:
            if detect.get_calculate_params(frame) is None:
                #frame = cv2.rotate(frame, cv2.ROTATE_180)
                cv2.imshow("figure", frame)
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.draw_marker(frame, x1, y1)
                detect.draw_marker(frame, x2, y2)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                init_num += 1
                continue
        elif init_num == 20:
            detect.set_cut_params(
                (detect.sum_x1)/20.0,
                (detect.sum_y1)/20.0,
                (detect.sum_x2)/20.0,
                (detect.sum_y2)/20.0,
            )
            detect.sum_x1 = detect.sum_x2 = detect.sum_y1 = detect.sum_y2 = 0
            init_num += 1
            continue

        # calculate params of the coords between cube and mycobot
        if nparams < 10:
            if detect.get_calculate_params(frame) is None:
                #frame = cv2.rotate(frame, cv2.ROTATE_180)
                cv2.imshow("figure", frame)
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.draw_marker(frame, x1, y1)
                detect.draw_marker(frame, x2, y2)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                nparams += 1
                continue
        elif nparams == 10:
            nparams += 1
            # calculate and set params of calculating real coord between cube and mycobot
            detect.set_params(
                (detect.sum_x1+detect.sum_x2)/20.0,
                (detect.sum_y1+detect.sum_y2)/20.0,
                abs(detect.sum_x1-detect.sum_x2)/10.0 +
                abs(detect.sum_y1-detect.sum_y2)/10.0
            )
            print "ok"
            continue

        # get detect result
        detect_result = detect.color_detect(frame)
        if detect_result is None:
            #frame = cv2.rotate(frame, cv2.ROTATE_180)
            cv2.imshow("figure", frame)
            continue
        else:
            x, y = detect_result
            # calculate real coord between cube and mycobot
            real_x, real_y = detect.get_position(x, y)
            if num == 20:
                detect.pub_marker(real_sx/20.0/1000.0, real_sy/20.0/1000.0)
                detect.decide_move(real_sx/20.0, real_sy/20.0, detect.color)
                num = real_sx = real_sy = 0

            else:
                num += 1
                real_sy += real_y
                real_sx += real_x
        #frame = cv2.rotate(frame, cv2.ROTATE_180)
        cv2.imshow("figure", frame)
mc.set_encoder(7, 2040)
time.sleep(2)
self.pub_angles(self.move_angles[2], 15)
