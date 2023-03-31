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

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 0
pose_target.orientation.y = 0
pose_target.orientation.z = 1
pose_target.orientation.w = 0
pose_target.position.x = 0.09
pose_target.position.y = 0.04
pose_target.position.z = 0.43
group.set_pose_target(pose_target)

plan1 = group.plan()

rospy.sleep(3)

group.go(wait=True)
group.stop()

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 0
pose_target.orientation.y = 0
pose_target.orientation.z = 1
pose_target.orientation.w = 0
pose_target.position.x = 0.1
pose_target.position.y = 0.1
pose_target.position.z = 0.4
group.set_pose_target(pose_target)

plan1 = group.plan()

rospy.sleep(3)

group.go(wait=True)
group.stop()

moveit_commander.roscpp_shutdown()
