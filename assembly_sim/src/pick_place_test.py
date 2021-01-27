#!/usr/bin/env python
import rospy
import copy

import actionlib
import control_msgs.msg

from geometry_msgs.msg import PoseStamped

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
		self.group_hand = self.mc.MoveGroupCommander("hand", self.robot_description, self.ns)

		self.scene = self.mc.PlanningSceneInterface()
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


def main():
	box_pose = PoseStamped()
	box_pose.header.frame_id = "shoulder_link"
	box_pose.pose.orientation.w = 1.0
	box_pose.pose.position.z = 1.0
	box_name = "test_box"
	RC1.scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

	box_pose = PoseStamped()
	box_pose.header.frame_id = "shoulder_link"
	box_pose.pose.orientation.w = 1.0
	box_pose.pose.position.z = 1.0
	box_name = "test_mesh"
	stl_path = '/home/cai/catkin_ws/src/assembly_robot/object_description/chair_meshes/chair part6.SLDPRT.STL'
	RC1.scene.add_mesh(box_name, box_pose, stl_path)
	
	print(RC1.robot.get_link_names(group="hand"))
	print(RC1.robot.get_link_names(group="arm"))


if __name__ == "__main__":	
	try :
		rospy.init_node("move_test")
		RC1 = Robot_Controller(ns="simul_robot1")
		RC1.gripper_control(True)
		RC1.gripper_control(False)

		main()

	except rospy.ROSInterruptException:
		pass