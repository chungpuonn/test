#!/usr/bin/env python2.7

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Header
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint

import tf

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_photoneo', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("photoneo_3d_scanner")
group.set_planner_id("PRMkConfigDefault")
group.set_planning_time(10)
group_name = "photoneo_3d_scanner"
move_group = moveit_commander.MoveGroupCommander(group_name)
eef_link = move_group.get_end_effector_link()
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

group_variable_values = group.get_current_joint_values()
# group_variable_values[2] = -0.59  #Rotate C-CW to -34 degrees
group_variable_values[2] = 0.90  #Rotate CW 52 degrees
group.set_joint_value_target(group_variable_values)
# plan1 = group.plan()
# rospy.sleep(2)
group.go(wait=True)
rospy.sleep(2)

moveit_commander.roscpp_shutdown()