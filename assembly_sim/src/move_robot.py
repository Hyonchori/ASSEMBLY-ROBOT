#!/usr/bin/env python
import rospy
import sys
import copy

import actionlib
import control_msgs.msg

from gazebo_msgs.srv import GetModelState

from geometry_msgs.msg import Pose

from std_msgs.msg import Float64

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

import tf.transformations as tf
import math as m



class Robot_Controller:
	def __init__(self, ns="simul_robot1", scale=0.1):
		self.ns = ns
		self.scale = scale
		self.robot_description = "/{}/robot_description".format(self.ns)
		self.joint_state_topic = ["joint_states:=/{}/joint_states".format(self.ns)]

		self.mc = moveit_commander
		self.mc.roscpp_initialize(self.joint_state_topic)
		self.robot = self.mc.RobotCommander(self.robot_description)
		self.scene = self.mc.PlanningSceneInterface()

		self.group_arm = self.mc.MoveGroupCommander("arm", self.robot_description, self.ns)
		self.group_hand = self.mc.MoveGroupCommander("hand", self.robot_description, self.ns)
		self.trajectory_pub = rospy.Publisher("move_group/display_planned_path",
												moveit_msgs.msg.DisplayTrajectory,
												queue_size=20)


		self.init_arm_pose()
		self.ready_arm_pose()

		rospy.wait_for_service("gazebo/get_model_state")
		self.client_modelstate = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)



		self.effort_pub = rospy.Publisher("{}/gripper_controller/command".format(self.ns), 
							Float64,
							queue_size=10)


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


	def effort_grasp(self, grasp=0.0):
		self.effort_pub.publish(grasp)
		rospy.sleep(1)




	def move_cartesian_path_test(self, vervose=1):
		waypoints = []

		wpose = self.group_arm.get_current_pose().pose

		wpose.position.z -= self.scale
		waypoints.append(copy.deepcopy(wpose))

		wpose.position.y += self.scale
		waypoints.append(copy.deepcopy(wpose))

		wpose.position.x += self.scale
		waypoints.append(copy.deepcopy(wpose))

		(plan, fraction) = self.group_arm.compute_cartesian_path(
													waypoints,
													0.01,
													0.0)
		print(wpose)
		self.group_arm.execute(plan)


	def move_pose(self, xyzRPY=[0, 0, 0, 0, 0, 0]):
		pose_goal = Pose()
		pose_goal.position.x = xyzRPY[0]
		pose_goal.position.y = xyzRPY[1]
		pose_goal.position.z = xyzRPY[2]

		quat = tf.quaternion_from_euler(xyzRPY[3], xyzRPY[4], xyzRPY[5])
		pose_goal.orientation.x = quat[0]
		pose_goal.orientation.y = quat[1]
		pose_goal.orientation.z = quat[2]
		pose_goal.orientation.w = quat[3]

		self.group_arm.set_pose_target(pose_goal)

		plan = self.group_arm.go(wait=True)
		self.group_arm.stop()
		self.group_arm.clear_pose_targets()


		target_pose = self.client_modelstate("box1", "simul_robot1").pose
		target_pt = target_pose.position
		target_ori = target_pose.orientation

		pose_goal = Pose()
		pose_goal.position.x = target_pt.x
		pose_goal.position.y = target_pt.y
		pose_goal.position.z = target_pt.z

		#quat = tf.quaternion_from_euler(xyzRPY[3], xyzRPY[4], xyzRPY[5])
		pose_goal.orientation.x = quat[0]
		pose_goal.orientation.y = quat[1]
		pose_goal.orientation.z = quat[2]
		pose_goal.orientation.w = quat[3]

		self.group_arm.set_pose_target(pose_goal)

		plan = self.group_arm.go(wait=True)
		self.group_arm.stop()
		self.group_arm.clear_pose_targets()




		




def main():
	pass






if __name__ == "__main__":	
	try :
		rospy.init_node("move_test")
		RC1 = Robot_Controller(ns="simul_robot1")
		#RC1.move_cartesian_path_test()
		RC1.move_pose([0.02, 0.23, 0.36, -m.pi, m.pi/2, 0])
		main()



	except rospy.ROSInterruptException:
		pass