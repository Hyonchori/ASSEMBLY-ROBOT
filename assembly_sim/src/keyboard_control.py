#!/usr/bin/env python
import rospy
import sys
import copy

import actionlib
import control_msgs.msg

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list


class Robot_Controller:
	def __init__(self, ns="simul_robot1", scale=0.05):
		self.ns = ns
		self.scale = scale
		self.robot_description = "/{}/robot_description".format(self.ns)
		self.joint_state_topic = ["joint_states:=/{}/joint_states".format(self.ns)]

		self.mc = moveit_commander
		self.mc.roscpp_initialize(self.joint_state_topic)
		self.robot = self.mc.RobotCommander(self.robot_description)

		self.group_arm = self.mc.MoveGroupCommander("arm", self.robot_description, self.ns)
		#self.group_hand = self.mc.MoveGroupCommander("hand", self.robot_description, self.ns)
		self.trajectory_pub = rospy.Publisher("move_group/display_planned_path",
												moveit_msgs.msg.DisplayTrajectory,
												queue_size=20)

		self.init_arm_pose()
		self.ready_arm_pose()


	def init_arm_pose(self):
		self.group_arm.set_named_target("init")
		plan = self.group_arm.plan()
		self.group_arm.execute(plan)


	def ready_arm_pose(self):
		self.group_arm.set_named_target("ready")
		plan = self.group_arm.plan()
		self.group_arm.execute(plan)


	def gripper_control(self, grasp=True):
		max_gripper_position = 0.804

		client = actionlib.SimpleActionClient(
					"{}/gripper_controller/gripper_cmd".format(self.ns),
					control_msgs.msg.GripperCommandAction)
		client.wait_for_server()

		goal = control_msgs.msg.GripperCommandGoal()
		if grasp == True:
			goal.command.position = max_gripper_position
		else:
			goal.command.position = 0.0

		client.send_goal(goal)
		client.wait_for_result()
		return client.get_result()


	def move_cartesian_pose(self, wpose):
		waypoints = []
		waypoints.append(copy.deepcopy(wpose))
		(plan, fraction) = self.group_arm.compute_cartesian_path(
													waypoints,
													0.01,
													0.0)
		print("\n---------------------------------------------")
		print(plan)
		self.group_arm.execute(plan)


	def move_cartesian_path(self, wpose_list):
		waypoints = []


	def move_tcp_by_KeyboardInput(self, verbose):
		verbose_list = ["a", "d", "w", "s", "q", "e", "g", "h"]
		wpose = self.group_arm.get_current_pose().pose

		if verbose == 0:
			wpose.position.y -= self.scale
			self.move_cartesian_pose(wpose)
			print(" tcp move to '-y'")

		elif verbose == 1:
			wpose.position.y += self.scale
			self.move_cartesian_pose(wpose)
			print(" tcp move to '+y'")

		elif verbose == 2:
			wpose.position.x -= self.scale
			self.move_cartesian_pose(wpose)
			print(" tcp move to '-x'")

		elif verbose == 3:
			wpose.position.x += self.scale
			self.move_cartesian_pose(wpose)
			print(" tcp move to '+x'")

		elif verbose == 4:
			wpose.position.z -= self.scale
			self.move_cartesian_pose(wpose)
			print(" tcp move to '-z'")

		elif verbose == 5:
			wpose.position.z += self.scale
			self.move_cartesian_pose(wpose)
			print(" tcp move to '+z'")

		elif verbose == 6:
			self.gripper_control(True)
			print(" gripper folding")

		elif verbose == 7:
			self.gripper_control(False)
			print(" gripper unfolding")

		elif verbose == 8:
			wpose.position.z += self.scale/2
			self.move_cartesian_pose(wpose)
			print(" tcp move to '+z/2'")

		elif verbose == 9:
			wpose.position.z -= self.scale/2
			self.move_cartesian_pose(wpose)
			print(" tcp move to '-z/2'")

		else:
			print("Input number ranged (0 ~ 7)")



def main(RC):
	while not rospy.is_shutdown():
		verbose = input("\nPress number (0 ~ 7)\n : ")
		RC.move_tcp_by_KeyboardInput(verbose)



if __name__ == "__main__":	
	try :
		rospy.init_node("move_test")
		RC1 = Robot_Controller(ns="simul_robot1")
		RC1.gripper_control(True)
		RC1.gripper_control(False)

		#RC2 = Robot_Controller(ns="simul_robot2")
		#RC2.gripper_control(True)
		#RC2.gripper_control(False)

		main(RC1)

	except rospy.ROSInterruptException:
		pass