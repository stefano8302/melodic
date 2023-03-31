#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm_group")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

group_variable_values = group.get_current_joint_values()

group_variable_values[0] = 3.14 # turns eje1 X amount of radians
group_variable_values[1] = -0.05 # turns eje1 X amount of radians
group_variable_values[2] = 0 # turns eje1 X amount of radians
group_variable_values[3] = 1.57 # turns eje1 X amount of radians
group_variable_values[4] = -1.57 # turns eje1 X amount of radians
group_variable_values[5] = 1.0 # turns eje1 X amount of radians
group.set_joint_value_target(group_variable_values)

plan2 = group.plan()

rospy.sleep(5)

group.go(wait=True)
group.stop()

moveit_commander.roscpp_shutdown()
