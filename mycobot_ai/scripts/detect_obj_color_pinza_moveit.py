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

import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
from math import pi
from std_msgs.msg import String
from std_srvs.srv import Empty
from moveit_commander.conversions import pose_to_list
from pymycobot import PI_PORT, PI_BAUD

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"
# Adaptive seeed

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class Object_detect(Movement):

    def __init__(self, camera_x=140, camera_y=-140): # it was (self, camera_x=150, camera_y=-10)
        # inherit the parent class
        super(Object_detect, self).__init__()
        # get path of file
        dir_path = os.path.dirname(__file__)

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm_group"
        group = moveit_commander.MoveGroupCommander(group_name)
 
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
        
        planning_frame = group.get_planning_frame()
        
        eef_link = group.get_end_effector_link()
        print ("============ End effector: %s" % eef_link)

        group_names = robot.get_group_names()
        
        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    
        global mc
        mc = MyCobot("/dev/ttyAMA0", 115200)
        #mc.power_on()
        #mc.set_servo_calibration(7)
        #time.sleep(1)
        #encoder=mc.get_encoder(7)
        #time.sleep(1)
        #print(encoder)


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
        GPIO.setup(19,GPIO.OUT)
        GPIO.setup(22,GPIO.OUT)
        GPIO.output(19,1)
        GPIO.output(22,1)

        # choose place to set cube
        self.color = 0
        # parameters to calculate camera clipping parameters
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord
        self.cache_x = self.cache_y = 0
        # set color HSV
        self.HSV = {
            "Amarillo": [np.array([11, 115, 70]), np.array([40, 255, 245])],
            "Rojo": [np.array([0, 170, 70]), np.array([10, 255, 245])],
            "Rojo": [np.array([160, 170, 90]), np.array([180, 255, 245])],
            "Verde": [np.array([40, 170, 90]), np.array([60, 255, 245])],
            "Fucsia": [np.array([130, 150, 46]), np.array([170, 255, 245])],
            "Turquesa": [np.array([75, 170, 46]), np.array([95, 255, 245])],
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
        #rospy.init_node("marker", anonymous=True)
        self.pub = rospy.Publisher("/cube", Marker, queue_size=1)
        # init a Marker
        self.marker = Marker()
        self.marker.header.frame_id = "base"
        self.marker.ns = "cube"
        self.marker.type = self.marker.CUBE
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.03
        self.marker.scale.y = 0.03
        self.marker.scale.z = 0.03
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
        self.marker.color.r = 1.0
        self.marker.color.b = 1.0

        # marker position initial
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0.03
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1.0

    def gripper_status(self, flag):
        if flag:
            mc.set_encoder(7, 1450)
        else:
            mc.set_encoder(7, 2040)

    def go_to_joint_state(self):
    
        group = self.group

        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -0.121
        joint_goal[2] = -0.96
        joint_goal[3] = -0.421
        joint_goal[4] = 0
        joint_goal[5] = 0

        group.go(joint_goal, wait=True)

        group.stop()

        #mc.power_on()
        #time.sleep(1)
        #self.gripper_status(False)
        #time.sleep(1)
        #print("False")

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.05)

    def go_to_joint_state2(self):
        group = self.group
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 1.2
        joint_goal[2] = 1.3
        joint_goal[3] = -1.2
        joint_goal[4] = 0
        joint_goal[5] = -2.2

        group.go(joint_goal, wait=True)

        group.stop()
    
        #mc.power_on()
        #time.sleep(1)
        #self.gripper_status(False)
        #time.sleep(1)
        #print("False")

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.05)

    def go_to_joint_state3(self):
   
        group = self.group

        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0

        group.go(joint_goal, wait=True)

        group.stop()

        #mc.power_on()
        #time.sleep(1)
        #self.gripper_status(False)
        #time.sleep(1)
        #print("False")

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.05)

    def go_to_pose_goal(self):
    
        group = self.group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0.021
        pose_goal.orientation.x = -0.021
        pose_goal.orientation.y = -0.707
        pose_goal.orientation.z = 0.707
        pose_goal.position.x = -0.053
        pose_goal.position.y = -0.231
        pose_goal.position.z = 0.311
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)

        group.stop()
   
        group.clear_pose_targets()
 
        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.05)
        
        #mc.power_on()
        #time.sleep(1)
        #self.gripper_status(False)
        #time.sleep(1)
        #print("False")

    def go_to_pose_goal2(self):
  
        group = self.group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0.501
        pose_goal.orientation.x = -0.502
        pose_goal.orientation.y = 0.439
        pose_goal.orientation.z = -0.551
        pose_goal.position.x = 0.235
        pose_goal.position.y = -0.179
        pose_goal.position.z = 0.197
        group.set_pose_target(pose_goal)
        
        plan = group.go(wait=True)

        group.stop()

        group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.05)  

    def go_to_pose_goal3(self, x, y):
  
        group = self.group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0.5598
        pose_goal.orientation.x = -0.5122
        pose_goal.orientation.y = 0.4193
        pose_goal.orientation.z = -0.4984
        pose_goal.position.x = (x)/1000.0 # 210
        pose_goal.position.y = (y)/1000.0 # -50
        pose_goal.position.z = 0.105
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)

        group.stop()

        group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.05)

        #mc.power_on()
        #time.sleep(1)
        #self.gripper_status(True)
        #time.sleep(1)
        #print("estado pinza verdadero/cerrada")
        

    def go_to_pose_goal4(self):
    
        group = self.group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0.707
        pose_goal.orientation.x = -0.707
        pose_goal.orientation.y = -0.002
        pose_goal.orientation.z = 0.002
        pose_goal.position.x = -0.042
        pose_goal.position.y = 0.229
        pose_goal.position.z = 0.311
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)

        group.stop()
   
        group.clear_pose_targets()
 
        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.05)
       
        #mc.power_on()
        #time.sleep(1)
        #self.gripper_status(False)
        #time.sleep(2)
        #print("False")

    def go_to_pose_goal5(self):
    
        group = self.group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0.707
        pose_goal.orientation.x = -0.707
        pose_goal.orientation.y = -0.002
        pose_goal.orientation.z = 0.002
        pose_goal.position.x = -0.150
        pose_goal.position.y = 0.228
        pose_goal.position.z = 0.311
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)

        group.stop()
   
        group.clear_pose_targets()
 
        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.05)

    def go_to_pose_goal6(self):
    
        group = self.group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0.013
        pose_goal.orientation.x = -0.707
        pose_goal.orientation.y = 0.707
        pose_goal.orientation.z = -0.013
        pose_goal.position.x = 0.230
        pose_goal.position.y = -0.072
        pose_goal.position.z = 0.306
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)

        group.stop()
   
        group.clear_pose_targets()
 
        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.05)

    def go_to_pose_goal7(self):
    
        group = self.group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = -0.021
        pose_goal.orientation.x = 0.021
        pose_goal.orientation.y = 0.707
        pose_goal.orientation.z = -0.707
        pose_goal.position.x = -0.148
        pose_goal.position.y = -0.226
        pose_goal.position.z = 0.311
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)

        group.stop()
   
        group.clear_pose_targets()
 
        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.05)

    def display_trajectory(self, plan):
    
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

    def execute_plan(self, plan):
 
        group = self.group

        group.execute(plan, wait=True)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
   
        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
          # Test if the box is in attached objects
          attached_objects = scene.get_attached_objects([box_name])
          is_attached = len(attached_objects.keys()) > 0

          # Test if the box is in the scene.
          # Note that attaching the box will remove it from known_objects
          is_known = box_name in scene.get_known_object_names()

          # Test if we are in the expected state
          if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

          # Sleep so that we give other threads time on the processor
          rospy.sleep(0.1)
          seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, x, y, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene at the location of the left finger:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base"
        box_pose.pose.orientation.w = -1.0
        box_pose.pose.position.x = (x/1000) # 0
        box_pose.pose.position.y = (y/1000) # 0.08
        box_pose.pose.position.z = 0.03 # 0.03
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.03, 0.03, 0.03))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name=box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


    def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
        grasping_group = 'arm_group'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
        box_name = self.box_name
        scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
        scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)       
 
    # publish marker
    def pub_marker(self, x, y, z=0.03):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        if self.color == 1:
            self.marker.color.r = 1
            self.marker.color.g = 1
            self.marker.color.b = 0
            self.pub.publish(self.marker)
        elif self.color == 0:
            self.marker.color.r = 1
            self.marker.color.g = 0
            self.marker.color.b = 0
            self.pub.publish(self.marker)
        elif self.color == 2:
            self.marker.color.r = 0
            self.marker.color.g = 1
            self.marker.color.b = 0
            self.pub.publish(self.marker)
        elif self.color == 3:
            self.marker.color.r = 0
            self.marker.color.g = 1
            self.marker.color.b = 1
            self.pub.publish(self.marker)

    def gpio_status(self, flag):
        if flag:
            GPIO.output(19, 0)
            GPIO.output(22, 0)
        else:
            GPIO.output(19, 1)
            GPIO.output(22, 1)
            
    # Grasping motion
    def move(self, x, y, color):
        # send Angle to move mycobot
        print color
        self.add_box(x, y)
        time.sleep(1)
        self.go_to_pose_goal2()
        self.go_to_pose_goal3(x, y)
        
        #mc.power_on()
        time.sleep(1)
        self.gripper_status(True)
        time.sleep(1)
        print("Cerada")
        self.attach_box()
        
        if color == 1: # el color detectado es amarillo
             #cv2.putText(image, text, org, font, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]])
             #cv2.putText(img, "AMARILLO".format(x, y), (x, y),
             #       cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (243, 0, 0), 2,)
             self.go_to_pose_goal()
             time.sleep(1)
             #mc.power_on()
             #time.sleep(1)
             self.gripper_status(False)
             time.sleep(1)
             print("Abierta")
             self.pub_marker(-0.05, -0.25, 0.115)
             self.detach_box()
             self.remove_box()
        
        elif color == 0: # el color detectado es rojo
             
             self.go_to_pose_goal4()
             time.sleep(1)
             #mc.power_on()
             #time.sleep(1)
             #self.gripper_status(False)
             #time.sleep(2)
             #print("False")
             self.pub_marker(-0.05, 0.25, 0.115)
             self.detach_box()
             self.remove_box()

        elif color == 2: # el color detectado es verde
             
             self.go_to_pose_goal5()
             time.sleep(1)
             #mc.power_on()
             #time.sleep(1)
             #self.gripper_status(False)
             #time.sleep(2)
             #print("False")
             self.pub_marker(-0.15, 0.25, 0.115)
             self.detach_box()
             self.remove_box()

        elif color == 3: # el color detectado es turquesa
             
             self.go_to_pose_goal7()
             time.sleep(1)
             #mc.power_on()
             #time.sleep(1)
             #self.gripper_status(False)
             #time.sleep(2)
             #print("False")
             self.pub_marker(-0.15, -0.25, 0.115)
             self.detach_box()
             self.remove_box()
        
        self.go_to_pose_goal6() # era 20, manda el robot volver a su posicion inicial
        time.sleep(3)

    # decide whether grab cube

    def decide_move(self, x, y, color):

        print(x, y, self.cache_x, self.cache_y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            
            self.move(x+60, y-30, color) # era self.move(x+13, y+13, color)

    # init mycobot
    def run(self):

        for _ in range(5): # el "_" es la variable contador
            #self.pub_angles([0, 0, 0, 0, 0, 0], 15) # era [0, -6.94, -55.01, -24.16, 0, 0]
            print(_)
            time.sleep(1.2)
        
        #self.gpio_status(False)
        #self.gripper_status(False) # anadido por Stefano
        #print("estado pinza falso/abierta")
        #time.sleep(1)
        #self.gripper_status(True)
        #time.sleep(1)
        #print("estado pinza verdadero/cerrada")
        # mover el robot a la posicion inicial para detectar los arucos
        #self.pub_angles(self.move_angles[2], 15)
        #self.go_to_joint_state()
        self.go_to_pose_goal6()
        time.sleep(1)

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
                        self.color = 3
                        print("Turquesa")
                    elif mycolor == "Verde":
                        self.color = 2
                        print("Verde")
                    
                    
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
                cv2.moveWindow("figure", 1060, 0)
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
                cv2.moveWindow("figure", 1060, 0)
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
            cv2.moveWindow("figure", 1060, 0)
            cv2.imshow("figure", frame)
            continue
        else:
            x, y = detect_result
            # calculate real coord between cube and mycobot
            real_x, real_y = detect.get_position(x, y)
            if num == 20:
                detect.pub_marker(real_sx/14.5/1000.0, real_sy/16.5/1000.0)
                detect.decide_move(real_sx/20.0, real_sy/20.0, detect.color)
                num = real_sx = real_sy = 0

            else:
                num += 1
                real_sy += real_y
                real_sx += real_x
        #frame = cv2.rotate(frame, cv2.ROTATE_180)
        cv2.moveWindow("figure", 1060, 0)
        cv2.imshow("figure", frame)
        
#mc.set_encoder(7, 2040)
#time.sleep(1)
#print("estado pinza falso/abierta")
