#!/usr/bin/env python
import rospy
import sys
import copy

import actionlib
import control_msgs.msg

import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose

from pprint import pprint
import time

def move_cartesian_pose(arm, wpose):
		waypoints = []
		waypoints.append(copy.deepcopy(wpose))
		(plan, fraction) = arm.compute_cartesian_path(
													waypoints,
													0.01,
													0.0)
		print("\n---------------------------------------------")
		#print(plan)
		arm.execute(plan)


def main():
    robot_description = "/robot_description"
    joint_state_topic = ["joint_states:=/joint_states"]

    mc = moveit_commander
    mc.roscpp_initialize(joint_state_topic)
    robot = mc.RobotCommander(robot_description)

    arm1 = mc.MoveGroupCommander("arm1", robot_description)
    arm1.set_named_target("arm1_ready")
    plan = arm1.plan()
    arm1.execute(plan)

    wpose = arm1.get_current_pose().pose

    pose_goal = Pose()
    pose_goal.orientation = wpose.orientation
    pose_goal.position.x = -0.2
    pose_goal.position.z = 0.3
    arm1.set_pose_target(pose_goal)

    plan = arm1.go(wait=True)
    arm1.stop()
    arm1.clear_pose_targets()


if __name__ == "__main__":	
	try :
		rospy.init_node("move_test")
		main()

	except rospy.ROSInterruptException:
		pass