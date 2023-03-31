#!/usr/bin/env python

import os
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospkg
from math import pi
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Bool


class MoveGroupSafeWatcher(object):

    def __init__(self):
        super(MoveGroupSafeWatcher, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)

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
        group_name = "arm_group"
        print("Waiting MAX 5 minutes >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        group = moveit_commander.MoveGroupCommander(group_name, wait_for_servers=300.0)
        print("Waiting MAX 5 minutes DONE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        rospy.loginfo("============ Reference frame: %s" % planning_frame)

        # We can also rospy.loginfo the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        rospy.loginfo("============ End effector: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        rospy.loginfo("============ Robot Groups:"+str(robot.get_group_names()))

        # Sometimes for debugging it is useful to rospy.loginfo the entire state of the
        # robot:
        rospy.loginfo("============ rospy.loginfoing robot state")
        rospy.loginfo(robot.get_current_state())
        rospy.loginfo ("")

        # Misc variables

        self.group = group


    def joint_traj_set_plan(self):

        joint_goal = self.group.get_current_joint_values()
        rospy.loginfo ("Group Vars:")
        
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/3
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        rospy.loginfo (joint_goal)

        self.group.set_joint_value_target(joint_goal)

    def joint_traj_set_error_plan(self):

        joint_goal = self.group.get_current_joint_values()
        rospy.loginfo ("Group Vars:")
        rospy.loginfo("Length="+str(len(joint_goal)))
        joint_goal[0] = -1.76
        joint_goal[1] = 2.00
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0
        rospy.loginfo (joint_goal)

        self.group.set_joint_value_target(joint_goal)

    def plan_trajectory(self):

        self.plan = self.group.plan()
        plan_ok = self.plan[0]
        
        return plan_ok

        
    def execute_trajectory(self):
        self.group.go(wait=True)
        self.group.stop()


    def test_traj(self):
        rospy.loginfo("Planning Valid Joint Config")
        self.joint_traj_set_plan()
        plan_result = self.plan_trajectory()
        rospy.loginfo("Plan Result="+str(plan_result))

        execute = self.execute_trajectory()



def main():
    rospy.init_node('move_group_watcher',
                    anonymous=True)
    try:
        move_group_watcher_obj = MoveGroupSafeWatcher()

        move_group_watcher_obj.test_traj()

        rospy.loginfo("============ Python move_group_watcher_obj demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
