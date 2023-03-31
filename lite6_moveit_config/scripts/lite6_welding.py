#!/usr/bin/env python
# encoding:utf-8

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import math
from math import pi, cos, sin
from std_msgs.msg import String
from std_srvs.srv import Empty
from moveit_commander.conversions import pose_to_list
from nav_msgs.msg import Path

import time
from tokenize import Pointfloat
import json

import rviz_tools_py as rviz_tools
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon # he borrado Pose
import tf.transformations as tr
import numpy as np
#from scipy.spatial.transform import Rotation
## END_SUB_TUTORIAL

#markers = rviz_tools.RvizMarkers('/world', 'visualization_marker')

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


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "lite6"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    #print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print ("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

    
  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 2.26
    joint_goal[1] = 1.95
    joint_goal[2] = -1.117
    joint_goal[3] = 2.199
    joint_goal[4] = -1.012
    joint_goal[5] = -3

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.05)

  def home(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()


    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.05)

  def angulos(self, j1=0, j2=0, j3=0, j4=0, j5=0, j6=0):
    group = self.group

    line = []
    line.append( Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))

    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    j1 = float(input("Elige el angulo de la articulacion 1 en grados sexagesimales: "))
    j1 = j1*pi/180
    j2 = float(input("Elige el angulo de la articulacion 2 en grados sexagesimales: "))
    j2 = j2*pi/180
    j3 = float(input("Elige el angulo de la articulacion 3 en grados sexagesimales: "))
    j3 = j3*pi/180
    j4 = float(input("Elige el angulo de la articulacion 4 en grados sexagesimales: "))
    j4 = j4*pi/180
    j5 = float(input("Elige el angulo de la articulacion 5 en grados sexagesimales: "))
    j5 = j5*pi/180
    j6 = float(input("Elige el angulo de la articulacion 6 en grados sexagesimales: "))
    j6 = j6*pi/180
    joint_goal[0] = j1
    joint_goal[1] = j2
    joint_goal[2] = j3
    joint_goal[3] = j4
    joint_goal[4] = j5
    joint_goal[5] = j6
    pose_goal = group.get_current_pose().pose
    print(pose_goal)
    line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
    width = 0.002
    #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime

    group.go(joint_goal, wait=True)

    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.05)

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    line = []
    line.append( Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))
    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 0
    pose_goal.orientation.x = 1
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0
    pose_goal.position.x = 0.25
    pose_goal.position.y = 0
    pose_goal.position.z = 0.04
    group.set_pose_target(pose_goal)
    line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
    width = 0.002
    #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime
    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    
    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.05)
  
  def coordenadas(self, x=0, y=0, z=0, ox=0, oy=0, oz=0, ow=0):
    
    group = self.group

    line = []
    line.append( Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))
    
    x = float(input("Elige la coordenada x en mm o pon 0 si deseas mantener la actual: "))
    x = (x/1000)
    if x == 0: 
      x = group.get_current_pose().pose.position.x
    y = float(input("Elige la coordenada y en mm o pon 0 si deseas mantener la actual: "))
    y = y/1000
    if y == 0: 
      y = group.get_current_pose().pose.position.y
    z = float(input("Elige la coordenada z en mm o pon 0 si deseas mantener la actual: "))
    z = z/1000
    if z == 0: 
      z = group.get_current_pose().pose.position.z
    ox = float(input("Elige la orientacion ox entre 0.001 y 1 o pon 0 si deseas mantener la actual: "))
    if ox == 0: 
      ox = group.get_current_pose().pose.orientation.x
    oy = float(input("Elige la orientacion oy entre 0.001 y 1 o pon 0 si deseas mantener la actual: "))
    if oy == 0: 
      oy = group.get_current_pose().pose.orientation.y
    oz = float(input("Elige la orientacion oz entre 0.001 y 1 o pon 0 si deseas mantener la actual: "))
    if oz == 0: 
      oz = group.get_current_pose().pose.orientation.z
    ow = float(input("Elige la orientacion ow entre 0.001 y 1 o pon 0 si deseas mantener la actual: "))
    if ow == 0: 
      ow = group.get_current_pose().pose.orientation.w

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = ow
    pose_goal.orientation.x = ox
    pose_goal.orientation.y = oy
    pose_goal.orientation.z = oz
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    group.set_pose_target(pose_goal)
    line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
    width = 0.002
    #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime

    plan = group.go(wait=True)
    
    group.stop()
    
    group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.05)


  def rectangulo(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    #self.go_to_pose_goal()

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    # establecemos la velocidad y la aceleracion que deseamos con los siguientes parametros
    # y luego hacemos referencia a los mismos con la funcion retime_trajectory()
    velocity_scaling_factor = 0.5
    acceleration_scaling_factor = 0.5

    x1 = float(input("Elige la coordenada x1 del rectangulo en mm, pon 0 si deseas la coordenada actual: "))
    x1 = x1/1000
    if x1 == 0: 
      x1 = group.get_current_pose().pose.position.x
    y1 = float(input("Elige la coordenada y1 del rectangulo en mm, pon 0 si deseas la coordenada actual: "))
    y1 = y1/1000
    if y1 == 0: 
      y1 = group.get_current_pose().pose.position.y
    z1 = float(input("Elige la coordenada z1 del rectangulo en mm, pon 0 si deseas la coordenada actual: "))
    z1 = z1/1000
    if z1 == 0: 
      z1 = group.get_current_pose().pose.position.z
    x2 = float(input("Elige la coordenada x2 del rectangulo en mm: "))
    x2 = x2/1000
    if x2 == 0: 
      x2 = group.get_current_pose().pose.position.x
    y2 = float(input("Elige la coordenada y2 del rectangulo en mm: "))
    y2 = y2/1000
    if y2 == 0: 
      y2 = group.get_current_pose().pose.position.y
    z2 = float(input("Elige la coordenada z2 del rectangulo en mm: "))
    z2 = z2/1000
    if z2 == 0: 
      z2 = group.get_current_pose().pose.position.z

    orientation=self.orientacion_tcp()
 
    waypoints = []
    if orientation=="xy-" or "xy+":
      wpose = group.get_current_pose().pose
      wpose.position.x = scale*(x1)
      wpose.position.y = scale*(y1)
      wpose.position.z = scale*(z1)
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = scale*(wpose.position.x+(x2-x1))
      wpose.position.y = scale*(wpose.position.y)
      wpose.position.z = scale*(wpose.position.z)
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = scale*(wpose.position.x)
      wpose.position.y = scale*(wpose.position.y+(y2-y1))
      wpose.position.z = scale*(wpose.position.z)
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = scale*(wpose.position.x-(x2-x1))
      wpose.position.y = scale*(wpose.position.y)
      wpose.position.z = scale*(wpose.position.z)
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = scale*(wpose.position.x)
      wpose.position.y = scale*(wpose.position.y-(y2-y1))
      wpose.position.z = scale*(wpose.position.z)
      waypoints.append(copy.deepcopy(wpose))
    if orientation=="xz-" or "xz+":
      wpose = group.get_current_pose().pose
      wpose.position.x = scale*(x1)
      wpose.position.y = scale*(y1)
      wpose.position.z = scale*(z1)
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = scale*(wpose.position.x+(x2-x1))
      wpose.position.y = scale*(wpose.position.y)
      wpose.position.z = scale*(wpose.position.z)
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = scale*(wpose.position.x)
      wpose.position.y = scale*(wpose.position.y)
      wpose.position.z = scale*(wpose.position.z+(z2-z1))
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = scale*(wpose.position.x-(x2-x1))
      wpose.position.y = scale*(wpose.position.y)
      wpose.position.z = scale*(wpose.position.z)
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = scale*(wpose.position.x)
      wpose.position.y = scale*(wpose.position.y)
      wpose.position.z = scale*(wpose.position.z-(z2-z1))
      waypoints.append(copy.deepcopy(wpose))
    if orientation=="yz-" or "yz+":
      wpose = group.get_current_pose().pose
      wpose.position.x = scale*(x1)
      wpose.position.y = scale*(y1)
      wpose.position.z = scale*(z1)
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = scale*(wpose.position.x)
      wpose.position.y = scale*(wpose.position.y)
      wpose.position.z = scale*(wpose.position.z+(z2-z1))
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = scale*(wpose.position.x)
      wpose.position.y = scale*(wpose.position.y+(y2-y1))
      wpose.position.z = scale*(wpose.position.z)
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = scale*(wpose.position.x)
      wpose.position.y = scale*(wpose.position.y)
      wpose.position.z = scale*(wpose.position.z-(z2-z1))
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = scale*(wpose.position.x)
      wpose.position.y = scale*(wpose.position.y-(y2-y1))
      wpose.position.z = scale*(wpose.position.z)
      waypoints.append(copy.deepcopy(wpose))

    width = 0.002
    #markers.publishPath(waypoints, 'orange', width, 60) # path, color, width, lifetime

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.005,        # eef_step
                                       0.0)         # jump_threshold

    # en la variable plan empleamos la funcion retime_trajectory() para establecer la velocidad y aceleracion definidas arriba
    plan = group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), 
                                       plan, 
                                       velocity_scaling_factor,
                                       acceleration_scaling_factor)

    print('El TCP se mueve a lo largo de trayectorias lineales')
    self.execute_plan(plan)

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL

  def cartesiano_xyz(self, scale=1):
    group = self.group

    velocity_scaling_factor = 0.1
    acceleration_scaling_factor = 0.1

    x = float(input("Elige el desplazamiento a lo largo del eje x en mm: "))
    x = x/1000
    y = float(input("Elige el desplazamiento a lo largo del eje y en mm: "))
    y = y/1000
    z = float(input("Elige el desplazamiento a lo largo del eje z en mm: "))
    z = z/1000
    waypoints = []

    line = []    
    line.append( Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))

    self.orientacion_tcp()

    wpose = group.get_current_pose().pose
    print(x)
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x += scale * x
    #waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += scale * y
    #waypoints.append(copy.deepcopy(wpose))

    wpose.position.z += scale * z
    waypoints.append(copy.deepcopy(wpose))
    
    width = 0.002
    #markers.publishPath(waypoints, 'orange', width, 20) # path, color, width, lifetime

    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.005,        # eef_step
                                       0.0)         # jump_threshold

    plan = group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), 
                                       plan, 
                                       velocity_scaling_factor,
                                       acceleration_scaling_factor)

    print('El TCP se mueve a lo largo de trayectorias lineales')
    self.execute_plan(plan)
    return plan, fraction
  
  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL


  def circular(self, r=0, x_center=0.25, y_center=0, z_center=0.1):

    group = self.group
    velocity_scaling_factor = 0.5
    acceleration_scaling_factor = 0.5
    
    angle_resolution = 0.1256
    # Defining trajectory parameters. 
    angle1 = 0
    angle = 0
    waypoints = []
    line = []
    line.append( Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))

    r = float(input("Elige el radio de la trayectoria circular en mm: "))
    r = r/1000
    x_center = float(input("Elige la coordenada x del centro de la trayectoria circular en mm: "))
    x_center = x_center/1000
    if x_center == 0: 
      x_center = group.get_current_pose().pose.position.x
    y_center = float(input("Elige la coordenada y del centro de la trayectoria circular en mm: "))
    y_center = y_center/1000
    if y_center == 0: 
      y_center = group.get_current_pose().pose.position.y
    z_center = float(input("Elige la coordenada z del centro de la trayectoria circular en mm: "))
    z_center = z_center/1000
    if z_center == 0: 
      z_center = group.get_current_pose().pose.position.z

    print('Elige el plano del circulo (xy-, xy+, xz-, xz+, yz-, yz+):')
    seleccion = input("")

    if seleccion == 'xy-':
      pose_goal = group.get_current_pose().pose
      pose_goal.orientation.w = 0
      pose_goal.orientation.x = 1
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      pose_goal.position.x = x_center
      pose_goal.position.y = y_center
      pose_goal.position.z = z_center
      group.set_pose_target(pose_goal)
      line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
      width = 0.002
      #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime
      plan = group.go(wait=True)
      wpose = group.get_current_pose().pose
      for i in range(0, 51):
        wpose.position.x = x_center+(r*cos(angle))
        wpose.position.y = y_center+(r*sin(angle)) 
        wpose.position.z = z_center
        waypoints.append(copy.deepcopy(wpose))
        # Publish a path using a list of ROS Point Msgs
        width = 0.002
        #markers.publishPath(waypoints, 'orange', width, 20) # path, color, width, lifetime
        angle = angle+angle_resolution
    
    elif seleccion == 'xy+':
      pose_goal = group.get_current_pose().pose
      pose_goal.orientation.w = 0
      pose_goal.orientation.x = 0
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 1
      pose_goal.position.x = x_center
      pose_goal.position.y = y_center
      pose_goal.position.z = z_center
      group.set_pose_target(pose_goal)
      line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
      width = 0.002
      #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime
      plan = group.go(wait=True)
      wpose = group.get_current_pose().pose
      for i in range(0, 51):
        wpose.position.x = x_center+(r*cos(angle))
        wpose.position.y = y_center+(r*sin(angle)) 
        wpose.position.z = z_center
        waypoints.append(copy.deepcopy(wpose))
        # Publish a path using a list of ROS Point Msgs
        width = 0.002
        #markers.publishPath(waypoints, 'orange', width, 20) # path, color, width, lifetime
        angle = angle+angle_resolution

    elif seleccion == 'xz-':
      pose_goal = group.get_current_pose().pose
      pose_goal.orientation.w = 0.707
      pose_goal.orientation.x = 0.707
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      pose_goal.position.x = x_center
      pose_goal.position.y = y_center
      pose_goal.position.z = z_center
      group.set_pose_target(pose_goal)
      line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
      width = 0.002
      #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime
      plan = group.go(wait=True)
      wpose = group.get_current_pose().pose
      for i in range(0, 51):
        wpose.position.x = x_center+(r*cos(angle))
        wpose.position.y = y_center 
        wpose.position.z = z_center+(r*sin(angle))
        waypoints.append(copy.deepcopy(wpose))
        # Publish a path using a list of ROS Point Msgs
        width = 0.002
        #markers.publishPath(waypoints, 'orange', width, 20) # path, color, width, lifetime
        angle = angle+angle_resolution

    elif seleccion == 'xz+':
      pose_goal = group.get_current_pose().pose
      pose_goal.orientation.w = 0.707
      pose_goal.orientation.x = -0.707
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      pose_goal.position.x = x_center
      pose_goal.position.y = y_center
      pose_goal.position.z = z_center
      group.set_pose_target(pose_goal)
      line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
      width = 0.002
      #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime
      plan = group.go(wait=True)
      wpose = group.get_current_pose().pose
      for i in range(0, 51):
        wpose.position.x = x_center+(r*cos(angle))
        wpose.position.y = y_center 
        wpose.position.z = z_center+(r*sin(angle))
        waypoints.append(copy.deepcopy(wpose))
        # Publish a path using a list of ROS Point Msgs
        width = 0.002
        #markers.publishPath(waypoints, 'orange', width, 20) # path, color, width, lifetime
        angle = angle+angle_resolution

    elif seleccion == 'yz-':
      pose_goal = group.get_current_pose().pose
      pose_goal.orientation.w = 0.5
      pose_goal.orientation.x = -0.5
      pose_goal.orientation.y = -0.5
      pose_goal.orientation.z = 0.5
      pose_goal.position.x = x_center
      pose_goal.position.y = y_center
      pose_goal.position.z = z_center
      group.set_pose_target(pose_goal)
      line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
      width = 0.002
      #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime
      plan = group.go(wait=True)
      wpose = group.get_current_pose().pose
      for i in range(0, 51):
        wpose.position.x = x_center
        wpose.position.y = y_center+(r*cos(angle)) 
        wpose.position.z = z_center+(r*sin(angle))
        waypoints.append(copy.deepcopy(wpose))
        # Publish a path using a list of ROS Point Msgs
        width = 0.002
        #markers.publishPath(waypoints, 'orange', width, 20) # path, color, width, lifetime
        angle = angle+angle_resolution

    elif seleccion == 'yz+':
      pose_goal = group.get_current_pose().pose
      pose_goal.orientation.w = 0.5
      pose_goal.orientation.x = 0.5
      pose_goal.orientation.y = 0.5
      pose_goal.orientation.z = 0.5
      pose_goal.position.x = x_center
      pose_goal.position.y = y_center
      pose_goal.position.z = z_center
      group.set_pose_target(pose_goal)
      line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
      width = 0.002
      #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime
      plan = group.go(wait=True)
      wpose = group.get_current_pose().pose
      for i in range(0, 51):
        wpose.position.x = x_center
        wpose.position.y = y_center+(r*cos(angle)) 
        wpose.position.z = z_center+(r*sin(angle))
        waypoints.append(copy.deepcopy(wpose))
        # Publish a path using a list of ROS Point Msgs
        width = 0.002
        #markers.publishPath(waypoints, 'orange', width, 20) # path, color, width, lifetime
        angle = angle+angle_resolution

    (plan, fraction) = group.compute_cartesian_path(
                           waypoints, # waypoints to follow
                           0.005,      #eef step
                           0.0)       # jump theshold
    # en la variable plan empleamos la funcion retime_trajectory() para establecer la velocidad y aceleracion definidas arriba
    plan = group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), 
                                       plan, 
                                       velocity_scaling_factor,
                                       acceleration_scaling_factor)
    print('El TCP se mueve a lo largo de una trayectoria circular')
    self.execute_plan(plan)
    
    return plan, fraction

  def orientacion_tcp(self):
    group = self.group
    line = []
    print('Elige el plano perpendicular a la orientación del TCP (xy-, xy+, xz-, xz+, yz-, yz+):')
    seleccion = input("")

    if seleccion == 'xy-':
      pose_goal = group.get_current_pose().pose
      pose_goal.orientation.w = 0
      pose_goal.orientation.x = 1
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      group.set_pose_target(pose_goal)
      line.append(Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))
      width = 0.002
      #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime
      plan = group.go(wait=True)

    if seleccion == 'xy+':
      pose_goal = group.get_current_pose().pose
      pose_goal.orientation.w = 0
      pose_goal.orientation.x = 0
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 1
      group.set_pose_target(pose_goal)
      line.append(Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))
      width = 0.002
      #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime
      plan = group.go(wait=True)

    if seleccion == 'xz-':
      pose_goal = group.get_current_pose().pose
      pose_goal.orientation.w = 0.707
      pose_goal.orientation.x = 0.707
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      group.set_pose_target(pose_goal)
      line.append(Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))
      width = 0.002
      #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime
      plan = group.go(wait=True)

    if seleccion == 'xz+':
      pose_goal = group.get_current_pose().pose
      pose_goal.orientation.w = 0.707
      pose_goal.orientation.x = -0.707
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      group.set_pose_target(pose_goal)
      line.append(Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))
      width = 0.002
      #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime
      plan = group.go(wait=True)

    if seleccion == 'yz-':
      pose_goal = group.get_current_pose().pose
      pose_goal.orientation.w = 0.5
      pose_goal.orientation.x = -0.5
      pose_goal.orientation.y = -0.5
      pose_goal.orientation.z = 0.5
      group.set_pose_target(pose_goal)
      line.append(Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))
      width = 0.002
      #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime
      plan = group.go(wait=True)

    if seleccion == 'yz+':
      pose_goal = group.get_current_pose().pose
      pose_goal.orientation.w = 0.5
      pose_goal.orientation.x = 0.5
      pose_goal.orientation.y = 0.5
      pose_goal.orientation.z = 0.5
      group.set_pose_target(pose_goal)
      line.append(Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))
      width = 0.002
      #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime
      plan = group.go(wait=True)

  def orientacion_tcp_3_puntos(self):
    group = self.group
    line = []
    line.append( Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))

    #Introducir (x,y,z) del punto a soldar [Punto O]
    xO = float(input("Introducir la coordenada x del punto a soldar [Punto O] en mm, pon 0 si la x es la actual: "))
    xO = xO/1000
    if xO == 0: 
      xO = group.get_current_pose().pose.position.x
    yO = float(input("Introducir la coordenada y del punto a soldar [Punto O] en mm, pon 0 si la y es la actual: "))
    yO = yO/1000
    if yO == 0: 
      yO = group.get_current_pose().pose.position.y
    zO = float(input("Introducir la coordenada z del punto a soldar [Punto O] en mm, pon 0 si la z es la actual: "))
    zO = zO/1000
    if zO == 0: 
      zO = group.get_current_pose().pose.position.z
    PO = [xO, yO, zO]
    
    #Introducir (x,y,z) de un punto ubicado en el plano de soldadura delante del punto a soldar (más cerca del usuario) [Punto A]
    xA = float(input("Introducir la coordenada x de un punto ubicado en el plano de soldadura delante del punto a soldar en mm (más cerca del usuario) [Punto A]: "))
    xA = xA/1000
    yA = float(input("Introducir la coordenada y de un punto ubicado en el plano de soldadura delante del punto a soldar en mm (más cerca del usuario) [Punto A]: "))
    yA = yA/1000
    zA = float(input("Introducir la coordenada z de un punto ubicado en el plano de soldadura delante del punto a soldar en mm (más cerca del usuario) [Punto A]: "))
    zA = zA/1000
    PA=[xA, yA, zA]

    #Introducir (x,y,z) de un punto ubicado en el plano de soldadura a la derecha del punto a soldar [Punto B]
    xB = float(input("Introducir la coordenada x de un punto ubicado en el plano de soldadura a la derecha del punto a soldar en mm (más cerca del usuario) [Punto B]: "))
    xB = xB/1000
    yB = float(input("Introducir la coordenada y de un punto ubicado en el plano de soldadura a la derecha del punto a soldar en mm (más cerca del usuario) [Punto B]: "))
    yB = yB/1000
    zB = float(input("Introducir la coordenada z de un punto ubicado en el plano de soldadura a la derecha del punto a soldar en mm (más cerca del usuario) [Punto B]: "))
    zB = zB/1000    
    PB=[xB, yB, zB]

    #Obtengo el módulo del vector, los convierto en unitarios y defino dos arrays, OA y OB para guardarlos
    ModOA=math.sqrt(((xA-xO)**2)+((yA-yO)**2)+((zA-zO)**2))
    print("ModOA", ModOA)
    ModOB=math.sqrt(((xB-xO)**2)+((yB-yO)**2)+((zB-zO)**2))
    print("ModOB", ModOB)
    OA = [(xA-xO)/ModOA, (yA-yO)/ModOA, (zA-zO)/ModOA]
    print("OA", OA)
    #OA = np.array(OA)
    #print("OA array", OA)
    OB = [(xB-xO)/ModOB, (yB-yO)/ModOB, (zB-zO)/ModOB]
    print("OB", OB)
    #OB = np.array(OB)
    #print("OB array", OB)

    #Multiplico vectorialmente OA x OB y lo guardo en un array nuevo llamado N con NumPy
    N=np.cross(OA, OB)
    print("N", N)
    ModN=math.sqrt((N[0]**2)+(N[1]**2)+(N[2]**2))
    N[0]=N[0]/ModN
    N[1]=N[1]/ModN
    N[2]=N[2]/ModN
    #ModN=round(math.sqrt((N[0]**2)+(N[1]**2)+(N[2]**2)))
    print("N normalizado", N)
    #N es ortogonal respecto a OA y a OB, pero ellos entre sí no tienen por qué serlo. Utilizo N y OA para obtener un vector ortogonal a ambos en el sentido positivo de OB.
    Xp=np.cross(N, OA)
    print("Xp", Xp)
    #Redefino las cadenas con los nombres de los ejes del centro de coordenadas objetivo
    Yp=N
    print("Yp", Yp)
    Zp=OA
    print("Zp", Zp)
    #Transformada que hay que tener en cuenta visto que queremos considerar el nuevo centro de coordenadas referenciandolo al absoluto (x0,y0,z0)
    R=[[Xp[0], Yp[0], Zp[0]],
       [Xp[1], Yp[1], Zp[1]],
       [Xp[2], Yp[2], Zp[2]]]
    print(R)
    #Ahora transformo esta matriz en cuaterniones
    #q = tr.quaternion_from_matrix(R)
    #q = Rotation.from_matrix(R)
    #q = q.as_quat()

    # converting list to array
    m = np.array(R)
    print(m)
    q=self.matriz_a_cuaterniones(R, m)
    print(q)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = (q[3])
    pose_goal.orientation.x = (q[0])
    pose_goal.orientation.y = (q[1])
    pose_goal.orientation.z = (q[2])
    pose_goal.position.x = xO
    pose_goal.position.y = yO
    pose_goal.position.z = zO
    group.set_pose_target(pose_goal)
    line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
    width = 0.002
    #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime

    plan = group.go(wait=True)
    
    group.stop()
    
    group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.05)

  def soldadura_por_puntos(self):
    group = self.group
    line = []
    line.append( Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))

    #Introducir (x,y,z) del punto a soldar [Punto O]
    xO = float(450)
    xO = xO/1000
    yO = float(-210)
    yO = yO/1000
    zO = float(395)
    zO = zO/1000
    PO = [xO, yO, zO]
    
    #Introducir (x,y,z) de un punto ubicado en el plano de soldadura delante del punto a soldar (más cerca del usuario) [Punto A]
    xA = float(500)
    xA = xA/1000
    yA = float(-210)
    yA = yA/1000
    zA = float(395)
    zA = zA/1000
    PA=[xA, yA, zA]

    #Introducir (x,y,z) de un punto ubicado en el plano de soldadura a la derecha del punto a soldar [Punto B]
    xB = float(450)
    xB = xB/1000
    yB = float(-200)
    yB = yB/1000
    zB = float(395)
    zB = zB/1000    
    PB=[xB, yB, zB]

    #Obtengo el módulo del vector, los convierto en unitarios y defino dos arrays, OA y OB para guardarlos
    ModOA=math.sqrt(((xA-xO)**2)+((yA-yO)**2)+((zA-zO)**2))
    print("ModOA", ModOA)
    ModOB=math.sqrt(((xB-xO)**2)+((yB-yO)**2)+((zB-zO)**2))
    print("ModOB", ModOB)
    OA = [(xA-xO)/ModOA, (yA-yO)/ModOA, (zA-zO)/ModOA]
    print("OA", OA)
    #OA = np.array(OA)
    #print("OA array", OA)
    OB = [(xB-xO)/ModOB, (yB-yO)/ModOB, (zB-zO)/ModOB]
    print("OB", OB)
    #OB = np.array(OB)
    #print("OB array", OB)

    #Multiplico vectorialmente OA x OB y lo guardo en un array nuevo llamado N con NumPy
    N=np.cross(OA, OB)
    print("N", N)
    ModN=math.sqrt((N[0]**2)+(N[1]**2)+(N[2]**2))
    N[0]=N[0]/ModN
    N[1]=N[1]/ModN
    N[2]=N[2]/ModN
    #ModN=round(math.sqrt((N[0]**2)+(N[1]**2)+(N[2]**2)))
    print("N normalizado", N)
    #N es ortogonal respecto a OA y a OB, pero ellos entre sí no tienen por qué serlo. Utilizo N y OA para obtener un vector ortogonal a ambos en el sentido positivo de OB.
    Xp=np.cross(N, OA)
    print("Xp", Xp)
    #Redefino las cadenas con los nombres de los ejes del centro de coordenadas objetivo
    Yp=N
    print("Yp", Yp)
    Zp=OA
    print("Zp", Zp)
    #Transformada que hay que tener en cuenta visto que queremos considerar el nuevo centro de coordenadas referenciandolo al absoluto (x0,y0,z0)
    R=[[Xp[0], Yp[0], Zp[0]],
       [Xp[1], Yp[1], Zp[1]],
       [Xp[2], Yp[2], Zp[2]]]
    print(R)
    #Ahora transformo esta matriz en cuaterniones

    # converting list to array
    m = np.array(R)
    print(m)
    q=self.matriz_a_cuaterniones(R, m)
    print(q)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = (q[3])
    pose_goal.orientation.x = (q[0])
    pose_goal.orientation.y = (q[1])
    pose_goal.orientation.z = (q[2])
    pose_goal.position.x = xO
    pose_goal.position.y = yO
    pose_goal.position.z = zO
    group.set_pose_target(pose_goal)
    line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
    width = 0.002
    #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime

    plan = group.go(wait=True)
    
    time.sleep(5)

    group = self.group
    line = []
    line.append( Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))
    #Introducir (x,y,z) del punto a soldar [Punto O]
    xO = float(450)
    xO = xO/1000
    yO = float(0)
    yO = yO/1000
    zO = float(195)
    zO = zO/1000
    PO = [xO, yO, zO]
    
    #Introducir (x,y,z) de un punto ubicado en el plano de soldadura delante del punto a soldar (más cerca del usuario) [Punto A]
    xA = float(500)
    xA = xA/1000
    yA = float(0)
    yA = yA/1000
    zA = float(195)
    zA = zA/1000
    PA=[xA, yA, zA]

    #Introducir (x,y,z) de un punto ubicado en el plano de soldadura a la derecha del punto a soldar [Punto B]
    xB = float(450)
    xB = xB/1000
    yB = float(50)
    yB = yB/1000
    zB = float(195)
    zB = zB/1000    
    PB=[xB, yB, zB]

    #Obtengo el módulo del vector, los convierto en unitarios y defino dos arrays, OA y OB para guardarlos
    ModOA=math.sqrt(((xA-xO)**2)+((yA-yO)**2)+((zA-zO)**2))
    print("ModOA", ModOA)
    ModOB=math.sqrt(((xB-xO)**2)+((yB-yO)**2)+((zB-zO)**2))
    print("ModOB", ModOB)
    OA = [(xA-xO)/ModOA, (yA-yO)/ModOA, (zA-zO)/ModOA]
    print("OA", OA)
    #OA = np.array(OA)
    #print("OA array", OA)
    OB = [(xB-xO)/ModOB, (yB-yO)/ModOB, (zB-zO)/ModOB]
    print("OB", OB)
    #OB = np.array(OB)
    #print("OB array", OB)

    #Multiplico vectorialmente OA x OB y lo guardo en un array nuevo llamado N con NumPy
    N=np.cross(OA, OB)
    print("N", N)
    ModN=math.sqrt((N[0]**2)+(N[1]**2)+(N[2]**2))
    N[0]=N[0]/ModN
    N[1]=N[1]/ModN
    N[2]=N[2]/ModN
    #ModN=round(math.sqrt((N[0]**2)+(N[1]**2)+(N[2]**2)))
    print("N normalizado", N)
    #N es ortogonal respecto a OA y a OB, pero ellos entre sí no tienen por qué serlo. Utilizo N y OA para obtener un vector ortogonal a ambos en el sentido positivo de OB.
    Xp=np.cross(N, OA)
    print("Xp", Xp)
    #Redefino las cadenas con los nombres de los ejes del centro de coordenadas objetivo
    Yp=N
    print("Yp", Yp)
    Zp=OA
    print("Zp", Zp)
    #Transformada que hay que tener en cuenta visto que queremos considerar el nuevo centro de coordenadas referenciandolo al absoluto (x0,y0,z0)
    R=[[Xp[0], Yp[0], Zp[0]],
       [Xp[1], Yp[1], Zp[1]],
       [Xp[2], Yp[2], Zp[2]]]
    print(R)
    #Ahora transformo esta matriz en cuaterniones
    #q = tr.quaternion_from_matrix(R)
    #q = Rotation.from_matrix(R)
    #q = q.as_quat()

    # converting list to array
    m = np.array(R)
    print(m)
    q=self.matriz_a_cuaterniones(R, m)
    print(q)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = (q[3])
    pose_goal.orientation.x = (q[0])
    pose_goal.orientation.y = (q[1])
    pose_goal.orientation.z = (q[2])
    pose_goal.position.x = xO
    pose_goal.position.y = yO
    pose_goal.position.z = zO
    group.set_pose_target(pose_goal)
    line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
    width = 0.002
    #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime

    plan = group.go(wait=True)
    
    time.sleep(5)

    group = self.group
    line = []
    line.append( Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))
    #Introducir (x,y,z) del punto a soldar [Punto O]
    xO = float(450)
    xO = xO/1000
    yO = float(-155)
    yO = yO/1000
    zO = float(300)
    zO = zO/1000
    PO = [xO, yO, zO]
    
    #Introducir (x,y,z) de un punto ubicado en el plano de soldadura delante del punto a soldar (más cerca del usuario) [Punto A]
    xA = float(500)
    xA = xA/1000
    yA = float(-155)
    yA = yA/1000
    zA = float(300)
    zA = zA/1000
    PA=[xA, yA, zA]

    #Introducir (x,y,z) de un punto ubicado en el plano de soldadura a la derecha del punto a soldar [Punto B]
    xB = float(450)
    xB = xB/1000
    yB = float(-105)
    yB = yB/1000
    zB = float(200)
    zB = zB/1000    
    PB=[xB, yB, zB]

    #Obtengo el módulo del vector, los convierto en unitarios y defino dos arrays, OA y OB para guardarlos
    ModOA=math.sqrt(((xA-xO)**2)+((yA-yO)**2)+((zA-zO)**2))
    print("ModOA", ModOA)
    ModOB=math.sqrt(((xB-xO)**2)+((yB-yO)**2)+((zB-zO)**2))
    print("ModOB", ModOB)
    OA = [(xA-xO)/ModOA, (yA-yO)/ModOA, (zA-zO)/ModOA]
    print("OA", OA)
    #OA = np.array(OA)
    #print("OA array", OA)
    OB = [(xB-xO)/ModOB, (yB-yO)/ModOB, (zB-zO)/ModOB]
    print("OB", OB)
    #OB = np.array(OB)
    #print("OB array", OB)

    #Multiplico vectorialmente OA x OB y lo guardo en un array nuevo llamado N con NumPy
    N=np.cross(OA, OB)
    print("N", N)
    ModN=math.sqrt((N[0]**2)+(N[1]**2)+(N[2]**2))
    N[0]=N[0]/ModN
    N[1]=N[1]/ModN
    N[2]=N[2]/ModN
    #ModN=round(math.sqrt((N[0]**2)+(N[1]**2)+(N[2]**2)))
    print("N normalizado", N)
    #N es ortogonal respecto a OA y a OB, pero ellos entre sí no tienen por qué serlo. Utilizo N y OA para obtener un vector ortogonal a ambos en el sentido positivo de OB.
    Xp=np.cross(N, OA)
    print("Xp", Xp)
    #Redefino las cadenas con los nombres de los ejes del centro de coordenadas objetivo
    Yp=N
    print("Yp", Yp)
    Zp=OA
    print("Zp", Zp)
    #Transformada que hay que tener en cuenta visto que queremos considerar el nuevo centro de coordenadas referenciandolo al absoluto (x0,y0,z0)
    R=[[Xp[0], Yp[0], Zp[0]],
       [Xp[1], Yp[1], Zp[1]],
       [Xp[2], Yp[2], Zp[2]]]
    print(R)
    #Ahora transformo esta matriz en cuaterniones

    # converting list to array
    m = np.array(R)
    print(m)
    q=self.matriz_a_cuaterniones(R, m)
    print(q)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = (q[3])
    pose_goal.orientation.x = (q[0])
    pose_goal.orientation.y = (q[1])
    pose_goal.orientation.z = (q[2])
    pose_goal.position.x = xO
    pose_goal.position.y = yO
    pose_goal.position.z = zO
    group.set_pose_target(pose_goal)
    line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
    width = 0.002
    #markers.publishPath(line, 'red', width, 20.0) # path, color, width, lifetime

    plan = group.go(wait=True)
    
    group.stop()
    
    group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.05)

  def matriz_a_cuaterniones(self, R, q):
  #Ahora transformo esta matriz en cuaterniones
    #q = Rotation.from_matrix(R)
    #q = q.as_quat()
    q=[]
    # converting list to array
    m = np.array(R)
    print(m)
    #q0 = qw
    t = np.matrix.trace(m)
    q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    if(t > 0):
        t = np.sqrt(t + 1)
        q[3] = 0.5 * t
        t = 0.5/t
        q[0] = (m[2,1] - m[1,2]) * t
        q[1] = (m[0,2] - m[2,0]) * t
        q[2] = (m[1,0] - m[0,1]) * t

    else:
        i = 0
        if (m[1,1] > m[0,0]):
            i = 1
        if (m[2,2] > m[i,i]):
            i = 2
        j = (i+1)%3
        k = (j+1)%3

        t = np.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
        q[i] = 0.5 * t
        t = 0.5 / t
        q[3] = (m[k,j] - m[j,k]) * t
        q[j] = (m[j,i] + m[i,j]) * t
        q[k] = (m[k,i] + m[i,k]) * t

    return q

  def soldadura_laser(self):
    y0=float(-0.101) #menos 100mm
    z0=float(0.15) # menos 50mm
    x0=float(0.270)
    aux=float(0)
    alpha=float(180)
    X=float(0)
    Y=float(0)
    Z=float(0)
    VX=[1,0,0]
    P=[0,0,0]
    S=[0,0,0]
    PS=[0,0,0]
    N=[0,0,0]
    R=[0,0,0]
    q=[0, 0, 0, 0]
    group = self.group

    line = []
    #line.append( Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))
    waypoints = []
    pose_goal = group.get_current_pose().pose
    pose_goal.orientation.w = -0.5
    pose_goal.orientation.x = 0.5
    pose_goal.orientation.y = 0.5
    pose_goal.orientation.z = 0.5
    pose_goal.position.x = x0
    pose_goal.position.y = y0
    pose_goal.position.z = z0
    group.set_pose_target(pose_goal)
    poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
    waypoints.append(copy.deepcopy(poses))
    line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
    width = 0.002
    #markers.publishPath(line, 'red', width, 80.0) # path, color, width, lifetime
    plan = group.go(wait=True)

    for aux in range(101):
      if aux<=12 and aux>0:
        print("aux", aux)
        Y=y0
        X=1.010+x0-(math.sqrt((1.010**2)-((Y-y0)**2)))
        Z=z0+(0.040/12)*aux
        S=[0,Y,Z]
        PS=[S[0]-P[0], S[1]-P[1], S[2]-P[2]]
        ModPS=math.sqrt((PS[0]**2)+(PS[1]**2)+(PS[2]**2))
        print("ModPS", ModPS)
        PS=(PS[0]/ModPS, PS[1]/ModPS, PS[2]/ModPS)
        #print(f"PS {PS}")
        #print()
        N=np.cross(PS,VX)
        #print(f"N {N}")
        #print()
        P=[0,Y,Z]
        #print(f"P {P}")
        R=[[PS[0], VX[0], N[0]],
           [PS[1], VX[1], N[1]],
           [PS[2], VX[2], N[2]]]

        m = np.array(R)
        print(m)
        q=self.matriz_a_cuaterniones(R, m)
        
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z
        #group.set_pose_target(pose_goal)
        #wpose = group.set_pose_target(pose_goal)
        poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        waypoints.append(copy.deepcopy(poses))
        
        line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
        width = 0.002
        #markers.publishPath(line, 'red', width, 80.0) # path, color, width, lifetime
        plan = group.go(wait=True)        
    

      if aux<=55 and aux>12:
        print("aux", aux)
        alpha=180-(143.11/43)*(aux-12)
        Y=y0+0.060+0.060*math.cos(math.radians(alpha))
        Z=z0+0.040+0.060*math.sin(math.radians(alpha))
        X=x0+1.010-(math.sqrt((1.010**2)-((Y-y0)**2)))
        #print(X)
        S=[0,Y,Z]
        #print(f"S {S}")
        #print()
        PS=[S[0]-P[0], S[1]-P[1], S[2]-P[2]]
        ModPS=math.sqrt((PS[0]**2)+(PS[1]**2)+(PS[2]**2))
        PS=(PS[0]/ModPS, PS[1]/ModPS, PS[2]/ModPS)
        #print(f"PS {PS}")
        #print()
        N=np.cross(PS,VX)
        ModPS=math.sqrt((N[0]**2)+(N[1]**2)+(N[2]**2))
        #print(f"N {N}")
        #print()
        P=[0,Y,Z]
        #print(f"P {P}")
        R=[[PS[0], VX[0], N[0]],
           [PS[1], VX[1], N[1]],
           [PS[2], VX[2], N[2]]]
        print("R", R)
        
        m = np.array(R)
        print(m)
        q=self.matriz_a_cuaterniones(R, m)
        
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z
        group.set_pose_target(pose_goal)
        poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        waypoints.append(copy.deepcopy(poses))
        line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
        width = 0.002
        #markers.publishPath(line, 'red', width, 80.0) # path, color, width, lifetime        
        plan = group.go(wait=True)

      if aux<=64 and aux>55:
        alpha=216.89+(90/9)*(aux-55)
        Y=y0+0.124+0.020*math.cos(math.radians(alpha))
        Z=z0+0.088+0.020*math.sin(math.radians(alpha))
        X=x0+1.010-(math.sqrt((1.010**2)-((Y-y0)**2)))
        #print(X)
        S=[0,Y,Z]
        
        PS=[S[0]-P[0], S[1]-P[1], S[2]-P[2]]
        ModPS=math.sqrt((PS[0]**2)+(PS[1]**2)+(PS[2]**2))
        PS=(PS[0]/ModPS, PS[1]/ModPS, PS[2]/ModPS)
        
        N=np.cross(PS,VX)
        
        P=[0,Y,Z]
        R=[[PS[0], VX[0], N[0]],
           [PS[1], VX[1], N[1]],
           [PS[2], VX[2], N[2]]]
        print("R", R)
        m = np.array(R)
        print(m)
        q=self.matriz_a_cuaterniones(R, m)

        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z
        group.set_pose_target(pose_goal)
        poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        waypoints.append(copy.deepcopy(poses))
        line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
        width = 0.002
        #markers.publishPath(line, 'red', width, 80.0) # path, color, width, lifetime        
        plan = group.go(wait=True)

      if aux<=89 and aux>64:
        alpha=126.89-(126.89/25)*(aux-64)
        Y=y0+0.160+0.040*math.cos(math.radians(alpha))
        Z=z0+0.040+0.040*math.sin(math.radians(alpha))
        X=x0+1.010-(math.sqrt((1.010**2)-((Y-y0)**2)))
        #print(X)
        S=[0,Y,Z]
        #print(f"S {S}")
        #print()
        PS=[S[0]-P[0], S[1]-P[1], S[2]-P[2]]
        ModPS=math.sqrt((PS[0]**2)+(PS[1]**2)+(PS[2]**2))
        PS=(PS[0]/ModPS, PS[1]/ModPS, PS[2]/ModPS)
        #print(f"PS {PS}")
        #print()
        N=np.cross(PS,VX)
        #print(f"N {N}")
        #print()
        P=[0,Y,Z]
        #print(f"P {P}")
        R=[[PS[0], VX[0], N[0]],
           [PS[1], VX[1], N[1]],
           [PS[2], VX[2], N[2]]]
        print("R", R)
        m = np.array(R)
        print(m)
        q=self.matriz_a_cuaterniones(R, m)

        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z
        group.set_pose_target(pose_goal)
        poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        waypoints.append(copy.deepcopy(poses))
        line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
        width = 0.002
        #markers.publishPath(line, 'red', width, 80.0) # path, color, width, lifetime        
        plan = group.go(wait=True)

      if aux<=101 and aux>89:
        Z=z0+0.040-(0.04/12)*(aux-89)
        X=x0+1.010-(math.sqrt((1.010**2)-((Y-y0)**2)))
        #print(X)
        S=[0,Y,Z]
        #print(f"S {S}")
        #print()
        PS=[S[0]-P[0], S[1]-P[1], S[2]-P[2]]
        ModPS=math.sqrt((PS[0]**2)+(PS[1]**2)+(PS[2]**2))
        PS=(PS[0]/ModPS, PS[1]/ModPS, PS[2]/ModPS)
        #print(f"PS {PS}")
        #print()
        N=np.cross(PS,VX)
        #print(f"N {N}")
        #print()
        P=[0,Y,Z]
        #print(f"P {P}")
        R=[[PS[0], VX[0], N[0]],
           [PS[1], VX[1], N[1]],
           [PS[2], VX[2], N[2]]]
        print("R", R)
        m = np.array(R)
        print(m)
        q=self.matriz_a_cuaterniones(R, m)

        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z
        group.set_pose_target(pose_goal)
        poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        waypoints.append(copy.deepcopy(poses))
        line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
        width = 0.002
        #markers.publishPath(line, 'red', width, 40.0) # path, color, width, lifetime        
        plan = group.go(wait=True)
    
    #movimiento cartesiano hacia el punto de partida
    #lastpoint=[]
    #pose_goal = group.get_current_pose().pose
    #pose_goal.orientation.w = 0
    #pose_goal.orientation.x = 0
    #pose_goal.orientation.y = 0
    #pose_goal.orientation.z = 1
    #pose_goal.position.x = X+0.02
    #pose_goal.position.y = Y
    #pose_goal.position.z = Z-0.005
    #group.set_pose_target(pose_goal)
    #line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
    #width = 0.002
    #markers.publishPath(line, 'red', width, 80.0) # path, color, width, lifetime
    #plan = group.go(wait=True)
    #wpose = group.get_current_pose().pose
    #lastpoint.append(copy.deepcopy(wpose))
    #wpose.position.x = x0
    #wpose.position.y = y0 
    #wpose.position.z = z0
    #wpose = group.get_current_pose().pose
    #lastpoint.append(copy.deepcopy(wpose))
    # Publish a path using a list of ROS Point Msgs
    #width = 0.002
    #markers.publishPath(line, 'orange', width, 80) # path, color, width, lifetime
    #(plan, fraction) = group.compute_cartesian_path(
    #                                   lastpoint,   # waypoints to follow
    #                                   0.005,        # eef_step
    #                                   0.0)         # jump_threshold
    #self.execute_plan(plan)
    #print("waypoints", waypoints)
    #plan=[]
    #for i in range(20):
    #  a=i*5
    #  group.set_pose_target(waypoints[a])
    #  plan=group.plan()
    #  group.execute(plan)

  def soldadura_mig(self):
    y0=float(-0.101)
    z0=float(0.15)
    x0=float(0.270)
    aux=float(0)
    aux2=float(1) #Si aux2 es igual a cero, entonces la variable aux es par. Si aux2 es igual a una, la variable aux es impar
    alpha=float(180)
    X=float(0)
    Y=float(0)
    Z=float(0)
    VX=[1,0,0]
    P=[0,0,0]
    S=[0,0,0]
    PS=[0,0,0]
    N=[0,0,0]
    R=[0,0,0]
    q=[0, 0, 0, 0]
    AtkA=math.radians(330)
    #AtkT=math.radians(15)
    qxaux=[math.cos(AtkA/2), 0.0, 0.0, math.sin(AtkA/2)]
    print("qxaux", qxaux)
    group = self.group
    line = []
    line.append( Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))
    waypoints = []
    decision=input("Desea añadir un movimiento oscilante en zig-zag? S/N: ")
    if decision=="S" or decision=="s":
      amp=float(input("Introduzca la amplitud del movimiento en mm:"))
      amp=amp/1000

    for aux in range(101):
      if aux<=12 and aux>0:
        if aux2==1:
          aux2=0
        else:
          aux2=aux2+1
        Y=y0
        X=1.010+x0-(math.sqrt((1.010**2)-((Y-y0)**2)))
        Z=z0+(0.040/12)*aux
        if decision=="S" or decision=="s" and aux2==0:
          X=X+amp/2
        if decision=="S" or decision=="s" and aux2==1:
          X=X-amp/2
        #print(f"pos {aux}  -  {X}")
        S=[0,Y,Z]
        #print(f"S {S}")
        #print()
        PS=[S[0]-P[0], S[1]-P[1], S[2]-P[2]]
        ModPS=math.sqrt((PS[0]**2)+(PS[1]**2)+(PS[2]**2))
        print("ModPS", ModPS)
        PS=(PS[0]/ModPS, PS[1]/ModPS, PS[2]/ModPS)
        #print(f"PS {PS}")
        #print()
        N=np.cross(PS,VX)
        #print(f"N {N}")
        #print()
        P=[0,Y,Z]
        #print(f"P {P}")
        R=[[PS[0], VX[0], N[0]],
           [PS[1], VX[1], N[1]],
           [PS[2], VX[2], N[2]]]

        m = np.array(R)
        print(m)
        q=self.matriz_a_cuaterniones(R, m)

        q=self.multiplica_2_cuaterniones(q, qxaux)
        
        #self.matriz_cuaterniones(R, q)
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z
        group.set_pose_target(pose_goal)
        #wpose = group.set_pose_target(pose_goal)
        line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
        width = 0.002
        #markers.publishPath(line, 'red', width, 80.0) # path, color, width, lifetime
                
        #waypoints.append(copy.deepcopy(wpose))
        plan = group.go(wait=True)

      if aux<=55 and aux>12:
        if aux2==1:
          aux2=0
        else:
          aux2=aux2+1
        print("aux", aux)
        alpha=180-(143.11/43)*(aux-12)
        Y=y0+0.060+0.060*math.cos(math.radians(alpha))
        Z=z0+0.040+0.060*math.sin(math.radians(alpha))
        X=x0+1.010-(math.sqrt((1.010**2)-((Y-y0)**2)))
        if decision=="S" or decision=="s" and aux2==0:
          X=X+amp/2
        if decision=="S" or decision=="s" and aux2==1:
          X=X-amp/2
        #print(X)
        S=[0,Y,Z]
        #print(f"S {S}")
        #print()
        PS=[S[0]-P[0], S[1]-P[1], S[2]-P[2]]
        ModPS=math.sqrt((PS[0]**2)+(PS[1]**2)+(PS[2]**2))
        PS=(PS[0]/ModPS, PS[1]/ModPS, PS[2]/ModPS)
        #print(f"PS {PS}")
        #print()
        N=np.cross(PS,VX)
        ModPS=math.sqrt((N[0]**2)+(N[1]**2)+(N[2]**2))
        #print(f"N {N}")
        #print()
        P=[0,Y,Z]
        #print(f"P {P}")
        R=[[PS[0], VX[0], N[0]],
           [PS[1], VX[1], N[1]],
           [PS[2], VX[2], N[2]]]
        print("R", R)
        
        m = np.array(R)
        print(m)
        q=self.matriz_a_cuaterniones(R, m)
        
        q=self.multiplica_2_cuaterniones(q, qxaux)

        #self.matriz_cuaterniones(R, q)
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z
        group.set_pose_target(pose_goal)
        
        line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
        width = 0.002
        #markers.publishPath(line, 'red', width, 80.0) # path, color, width, lifetime        
        plan = group.go(wait=True)

      if aux<=64 and aux>55:
        if aux2==1:
          aux2=0
        else:
          aux2=aux2+1
        alpha=216.89+(90/9)*(aux-55)
        Y=y0+0.124+0.020*math.cos(math.radians(alpha))
        Z=z0+0.088+0.020*math.sin(math.radians(alpha))
        X=x0+1.010-(math.sqrt((1.010**2)-((Y-y0)**2)))
        if decision=="S" or decision=="s" and aux2==0:
          X=X+amp/2
        if decision=="S" or decision=="s" and aux2==1:
          X=X-amp/2
        #print(X)
        S=[0,Y,Z]
        
        PS=[S[0]-P[0], S[1]-P[1], S[2]-P[2]]
        ModPS=math.sqrt((PS[0]**2)+(PS[1]**2)+(PS[2]**2))
        PS=(PS[0]/ModPS, PS[1]/ModPS, PS[2]/ModPS)
        
        N=np.cross(PS,VX)
        
        P=[0,Y,Z]
        R=[[PS[0], VX[0], N[0]],
           [PS[1], VX[1], N[1]],
           [PS[2], VX[2], N[2]]]
        print("R", R)
        m = np.array(R)
        print(m)
        q=self.matriz_a_cuaterniones(R, m)

        q=self.multiplica_2_cuaterniones(q, qxaux)

        #self.matriz_cuaterniones(R, q)
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z
        group.set_pose_target(pose_goal)
        
        line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
        width = 0.002
        #markers.publishPath(line, 'red', width, 80.0) # path, color, width, lifetime        
        plan = group.go(wait=True)

      if aux<=89 and aux>64:
        if aux2==1:
          aux2=0
        else:
          aux2=aux2+1
        alpha=126.89-(126.89/25)*(aux-64)
        Y=y0+0.160+0.040*math.cos(math.radians(alpha))
        Z=z0+0.040+0.040*math.sin(math.radians(alpha))
        X=x0+1.010-(math.sqrt((1.010**2)-((Y-y0)**2)))
        if decision=="S" or decision=="s" and aux2==0:
          X=X+amp/2
        if decision=="S" or decision=="s" and aux2==1:
          X=X-amp/2
        #print(X)
        S=[0,Y,Z]
        #print(f"S {S}")
        #print()
        PS=[S[0]-P[0], S[1]-P[1], S[2]-P[2]]
        ModPS=math.sqrt((PS[0]**2)+(PS[1]**2)+(PS[2]**2))
        PS=(PS[0]/ModPS, PS[1]/ModPS, PS[2]/ModPS)
        #print(f"PS {PS}")
        #print()
        N=np.cross(PS,VX)
        #print(f"N {N}")
        #print()
        P=[0,Y,Z]
        #print(f"P {P}")
        R=[[PS[0], VX[0], N[0]],
           [PS[1], VX[1], N[1]],
           [PS[2], VX[2], N[2]]]
        print("R", R)
        m = np.array(R)
        print(m)
        q=self.matriz_a_cuaterniones(R, m)

        q=self.multiplica_2_cuaterniones(q, qxaux)

        #self.matriz_cuaterniones(R, q)
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z
        group.set_pose_target(pose_goal)
        line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
        width = 0.002
        #markers.publishPath(line, 'red', width, 80.0) # path, color, width, lifetime        
        plan = group.go(wait=True)

      if aux<=101 and aux>89:
        if aux2==1:
          aux2=0
        else:
          aux2=aux2+1
        Z=z0+0.040-(0.04/12)*(aux-89)
        X=x0+1.010-(math.sqrt((1.010**2)-((Y-y0)**2)))
        if decision=="S" or decision=="s" and aux2==0:
          X=X+amp/2
        if decision=="S" or decision=="s" and aux2==1:
          X=X-amp/2
        #print(X)
        S=[0,Y,Z]
        #print(f"S {S}")
        #print()
        PS=[S[0]-P[0], S[1]-P[1], S[2]-P[2]]
        ModPS=math.sqrt((PS[0]**2)+(PS[1]**2)+(PS[2]**2))
        PS=(PS[0]/ModPS, PS[1]/ModPS, PS[2]/ModPS)
        #print(f"PS {PS}")
        #print()
        N=np.cross(PS,VX)
        #print(f"N {N}")
        #print()
        P=[0,Y,Z]
        #print(f"P {P}")
        R=[[PS[0], VX[0], N[0]],
           [PS[1], VX[1], N[1]],
           [PS[2], VX[2], N[2]]]
        print("R", R)
        m = np.array(R)
        print(m)
        q=self.matriz_a_cuaterniones(R, m)

        q=self.multiplica_2_cuaterniones(q, qxaux)

        #self.matriz_cuaterniones(R, q)
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z
        group.set_pose_target(pose_goal)
        line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
        width = 0.002
        #markers.publishPath(line, 'red', width, 40.0) # path, color, width, lifetime        
        plan = group.go(wait=True)

  def soldadura_tig(self):
    y0=float(-0.101)
    z0=float(0.15)
    x0=float(0.270)
    aux=float(0)
    aux2=float(1) #Si aux2 es igual a cero, entonces la variable aux es par. Si aux2 es igual a una, la variable aux es impar
    alpha=float(180)
    X=float(0)
    Y=float(0)
    Z=float(0)
    VX=[1,0,0]
    P=[0,0,0]
    S=[0,0,0]
    PS=[0,0,0]
    N=[0,0,0]
    R=[0,0,0]
    q=[0, 0, 0, 0]
    #AtkA=math.radians(330)
    AtkT=math.radians(15)
    qxaux=[math.cos(AtkT/2), 0.0, 0.0, math.sin(AtkT/2)]
    group = self.group
    line = []
    line.append( Point(group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z))
    waypoints = []
    decision=input("Desea añadir un movimiento oscilante en zig-zag? S/N: ")
    if decision=="S" or decision=="s":
      amp=float(input("Introduzca la amplitud del movimiento en mm:"))
      amp=amp/1000

    for aux in range(101):
      if aux<=12 and aux>0:
        if aux2==1:
          aux2=0
        else:
          aux2=aux2+1
        Y=y0
        X=1.010+x0-(math.sqrt((1.010**2)-((Y-y0)**2)))
        Z=z0+(0.040/12)*aux
        if decision=="S" or decision=="s" and aux2==0:
          X=X+amp/2
        if decision=="S" or decision=="s" and aux2==1:
          X=X-amp/2
        #print(f"pos {aux}  -  {X}")
        S=[0,Y,Z]
        #print(f"S {S}")
        #print()
        PS=[S[0]-P[0], S[1]-P[1], S[2]-P[2]]
        ModPS=math.sqrt((PS[0]**2)+(PS[1]**2)+(PS[2]**2))
        print("ModPS", ModPS)
        PS=(PS[0]/ModPS, PS[1]/ModPS, PS[2]/ModPS)
        #print(f"PS {PS}")
        #print()
        N=np.cross(PS,VX)
        #print(f"N {N}")
        #print()
        P=[0,Y,Z]
        #print(f"P {P}")
        R=[[PS[0], VX[0], N[0]],
           [PS[1], VX[1], N[1]],
           [PS[2], VX[2], N[2]]]

        m = np.array(R)
        print(m)
        q=self.matriz_a_cuaterniones(R, m)

        q=self.multiplica_2_cuaterniones(q, qxaux)
        
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z
        group.set_pose_target(pose_goal)
        #wpose = group.set_pose_target(pose_goal)
        line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
        width = 0.002
        #markers.publishPath(line, 'red', width, 80.0) # path, color, width, lifetime
                
        #waypoints.append(copy.deepcopy(wpose))
        plan = group.go(wait=True)

      if aux<=55 and aux>12:
        if aux2==1:
          aux2=0
        else:
          aux2=aux2+1
        print("aux", aux)
        alpha=180-(143.11/43)*(aux-12)
        Y=y0+0.060+0.060*math.cos(math.radians(alpha))
        Z=z0+0.040+0.060*math.sin(math.radians(alpha))
        X=x0+1.010-(math.sqrt((1.010**2)-((Y-y0)**2)))
        if decision=="S" or decision=="s" and aux2==0:
          X=X+amp/2
        if decision=="S" or decision=="s" and aux2==1:
          X=X-amp/2
        #print(X)
        S=[0,Y,Z]
        #print(f"S {S}")
        #print()
        PS=[S[0]-P[0], S[1]-P[1], S[2]-P[2]]
        ModPS=math.sqrt((PS[0]**2)+(PS[1]**2)+(PS[2]**2))
        PS=(PS[0]/ModPS, PS[1]/ModPS, PS[2]/ModPS)
        #print(f"PS {PS}")
        #print()
        N=np.cross(PS,VX)
        ModPS=math.sqrt((N[0]**2)+(N[1]**2)+(N[2]**2))
        #print(f"N {N}")
        #print()
        P=[0,Y,Z]
        #print(f"P {P}")
        R=[[PS[0], VX[0], N[0]],
           [PS[1], VX[1], N[1]],
           [PS[2], VX[2], N[2]]]
        print("R", R)
        
        m = np.array(R)
        print(m)
        q=self.matriz_a_cuaterniones(R, m)

        q=self.multiplica_2_cuaterniones(q, qxaux)
        
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z
        group.set_pose_target(pose_goal)
        
        line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
        width = 0.002
        #markers.publishPath(line, 'red', width, 80.0) # path, color, width, lifetime        
        plan = group.go(wait=True)

      if aux<=64 and aux>55:
        if aux2==1:
          aux2=0
        else:
          aux2=aux2+1
        alpha=216.89+(90/9)*(aux-55)
        Y=y0+0.124+0.020*math.cos(math.radians(alpha))
        Z=z0+0.088+0.020*math.sin(math.radians(alpha))
        X=x0+1.010-(math.sqrt((1.010**2)-((Y-y0)**2)))
        if decision=="S" or decision=="s" and aux2==0:
          X=X+amp/2
        if decision=="S" or decision=="s" and aux2==1:
          X=X-amp/2
        #print(X)
        S=[0,Y,Z]
        
        PS=[S[0]-P[0], S[1]-P[1], S[2]-P[2]]
        ModPS=math.sqrt((PS[0]**2)+(PS[1]**2)+(PS[2]**2))
        PS=(PS[0]/ModPS, PS[1]/ModPS, PS[2]/ModPS)
        
        N=np.cross(PS,VX)
        
        P=[0,Y,Z]
        R=[[PS[0], VX[0], N[0]],
           [PS[1], VX[1], N[1]],
           [PS[2], VX[2], N[2]]]
        print("R", R)
        m = np.array(R)
        print(m)
        q=self.matriz_a_cuaterniones(R, m)

        q=self.multiplica_2_cuaterniones(q, qxaux)

        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z
        group.set_pose_target(pose_goal)
        
        line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
        width = 0.002
        #markers.publishPath(line, 'red', width, 80.0) # path, color, width, lifetime        
        plan = group.go(wait=True)

      if aux<=89 and aux>64:
        if aux2==1:
          aux2=0
        else:
          aux2=aux2+1
        alpha=126.89-(126.89/25)*(aux-64)
        Y=y0+0.160+0.040*math.cos(math.radians(alpha))
        Z=z0+0.040+0.040*math.sin(math.radians(alpha))
        X=x0+1.010-(math.sqrt((1.010**2)-((Y-y0)**2)))
        if decision=="S" or decision=="s" and aux2==0:
          X=X+amp/2
        if decision=="S" or decision=="s" and aux2==1:
          X=X-amp/2
        #print(X)
        S=[0,Y,Z]
        #print(f"S {S}")
        #print()
        PS=[S[0]-P[0], S[1]-P[1], S[2]-P[2]]
        ModPS=math.sqrt((PS[0]**2)+(PS[1]**2)+(PS[2]**2))
        PS=(PS[0]/ModPS, PS[1]/ModPS, PS[2]/ModPS)
        #print(f"PS {PS}")
        #print()
        N=np.cross(PS,VX)
        #print(f"N {N}")
        #print()
        P=[0,Y,Z]
        #print(f"P {P}")
        R=[[PS[0], VX[0], N[0]],
           [PS[1], VX[1], N[1]],
           [PS[2], VX[2], N[2]]]
        print("R", R)
        m = np.array(R)
        print(m)
        q=self.matriz_a_cuaterniones(R, m)

        q=self.multiplica_2_cuaterniones(q, qxaux)

        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z
        group.set_pose_target(pose_goal)
        line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
        width = 0.002
        #markers.publishPath(line, 'red', width, 80.0) # path, color, width, lifetime        
        plan = group.go(wait=True)

      if aux<=101 and aux>89:
        if aux2==1:
          aux2=0
        else:
          aux2=aux2+1
        Z=z0+0.040-(0.04/12)*(aux-89)
        X=x0+1.010-(math.sqrt((1.010**2)-((Y-y0)**2)))
        if decision=="S" or decision=="s" and aux2==0:
          X=X+amp/2
        if decision=="S" or decision=="s" and aux2==1:
          X=X-amp/2
        #print(X)
        S=[0,Y,Z]
        #print(f"S {S}")
        #print()
        PS=[S[0]-P[0], S[1]-P[1], S[2]-P[2]]
        ModPS=math.sqrt((PS[0]**2)+(PS[1]**2)+(PS[2]**2))
        PS=(PS[0]/ModPS, PS[1]/ModPS, PS[2]/ModPS)
        #print(f"PS {PS}")
        #print()
        N=np.cross(PS,VX)
        #print(f"N {N}")
        #print()
        P=[0,Y,Z]
        #print(f"P {P}")
        R=[[PS[0], VX[0], N[0]],
           [PS[1], VX[1], N[1]],
           [PS[2], VX[2], N[2]]]
        print("R", R)
        m = np.array(R)
        print(m)
        q=self.matriz_a_cuaterniones(R, m)

        q=self.multiplica_2_cuaterniones(q, qxaux)

        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z
        group.set_pose_target(pose_goal)
        line.append( Point(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z) )
        width = 0.002
        #markers.publishPath(line, 'red', width, 40.0) # path, color, width, lifetime        
        plan = group.go(wait=True)

  def multiplica_2_cuaterniones(self, q, qxaux):
    #q=quaternion_multiply(q, qxaux)
    # No tenemos la funcion para multiplicar 2 cuaterniones, asi que la ponemos entera aqui abajo
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]
     
    # Extract the values from Q1
    qxaux_w = qxaux[0]
    qxaux_x = qxaux[1]
    qxaux_y = qxaux[2]
    qxaux_z = qxaux[3]
     
    # Computer the product of the two quaternions, term by term
    Q0Q1_w = qw * qxaux_w - qx * qxaux_x - qy * qxaux_y - qz * qxaux_z
    Q0Q1_x = qw * qxaux_x + qx * qxaux_w + qy * qxaux_z - qz * qxaux_y
    Q0Q1_y = qw * qxaux_y - qx * qxaux_z + qy * qxaux_w + qz * qxaux_x
    Q0Q1_z = qw * qxaux_z + qx * qxaux_y - qy * qxaux_x + qz * qxaux_w
     
    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([Q0Q1_w, Q0Q1_x, Q0Q1_y, Q0Q1_z])
    print("Final quaternion", final_quaternion)
     
    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    q=final_quaternion
    return q

  def tipo_movimiento(self):
    print("Elige tipo de movimiento que deseas realizar")
    print("0=home,           1=angulos,        2=coordenadas,")
    print("3=cartesiano-xyz, 4=circular,       5=rectangulo,")
    print("6=orientacion3p,  7=Soldadura laser,8=Soldadura por puntos,")
    print("9=Soldadura MIG,  10=Soldadura TIG, q=cerrar):")
    seleccion = input("")
    if seleccion == '0':
        movimiento = self.home()
    if seleccion == '1':
        movimiento = self.angulos()
    if seleccion == '2':
        movimiento = self.coordenadas()
    if seleccion == '3':
        movimiento = self.cartesiano_xyz()
    if seleccion == '4':
        movimiento = self.circular()
    if seleccion == '5':
        movimiento = self.rectangulo()
    if seleccion == '6':
        movimiento = self.orientacion_tcp_3_puntos()
    if seleccion == '7':
        movimiento = self.soldadura_laser()
    if seleccion == '8':
        movimiento = self.soldadura_por_puntos()
    if seleccion == '9':
        movimiento = self.soldadura_mig()
    if seleccion == '10':
        movimiento = self.soldadura_tig()
    if seleccion == 'q':
        print ("============ Tutorial de soldadura terminado!")
        sys.exit()
    return movimiento

def main():
  while True:
    try:
      tutorial = MoveGroupPythonIntefaceTutorial()
      tutorial.tipo_movimiento()
      #tutorial.home()
    
    except rospy.ROSInterruptException:
      return
    except KeyboardInterrupt:
      return

if __name__ == '__main__':
  main()
