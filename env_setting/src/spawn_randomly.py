#!/usr/bin/env python
import rospy
import rospkg

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetModelState

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

import os
import numpy as np
import math as m
import tf.transformations as tf


OBJECT_LIST = ["chair_part1.SLDPRT.urdf", "chair_part2.SLDPRT.urdf", 
			"chair_part3.SLDPRT.urdf", "chair_part4.SLDPRT.urdf",
			"chair_part5.SLDPRT.urdf", "chair_part6.SLDPRT.urdf"]


ROSPACK = rospkg.RosPack()


class Spawner:
	def __init__(self):
		self.model_names_in_gazebo = []
		self.pkg_path = ROSPACK.get_path("object_description")
		self.urdfs_path = os.path.join(self.pkg_path, "chair_urdf")

		rospy.wait_for_service("gazebo/spawn_urdf_model")
		rospy.wait_for_service("gazebo/delete_model")
		rospy.wait_for_service("gazebo/get_model_state")

		self.client_spawn = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
		self.client_delete = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
		self.client_modelstate = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)


	def clear_workspace(self):
		for i in OBJECT_LIST:
			model_name = i.split(".")[0]
			if self.client_modelstate(model_name, "").success:
				self.client_delete(model_name)
			else:
				pass


	def spawn_one_model_randomPose(self, object_num=1):
		object_index = object_num - 1
		urdf_name = OBJECT_LIST[object_index]
		model_name = urdf_name.split(".")[0]

		if self.client_modelstate(model_name, "").success:
			self.client_delete(model_name)
			print("{} already exists in gazebo, so delete and respawn".format(model_name))
		else:
			print("spawning {}...".format(model_name))

		urdf_path = os.path.join(self.urdfs_path, urdf_name)
		with open(urdf_path, "r") as urdf_file:
			model_xml = urdf_file.read()
			urdf_file.close()

		x = np.random.randint(-10, 10, 1) / 100.0
		y = np.random.randint(-10, 10, 1) / 100.0
		z = 1.1
		model_point = Point(x, y, z)

		rx = 0
		ry = 0
		rz = np.random.randint(-314, 314, 1) / 100.0
		quat = tf.quaternion_from_euler(rx, ry, rz)
		model_quaternion = Quaternion(quat[0], quat[1], quat[2], quat[3])

		model_pose = Pose(model_point, model_quaternion)
		self.client_spawn(model_name, model_xml, "", model_pose, "world")


	def spawn_models(self, object_list):
		self.clear_workspace()

		for i in object_list:
			self.spawn_one_model_randomPose(i)
		
		

if __name__ == "__main__":	
	try :
		rospy.init_node("spawn_test")
		spawner = Spawner()

		object_list = [3, 1, 2, 4]
		spawner.spawn_models(object_list)

	except rospy.ROSInterruptException:
		pass