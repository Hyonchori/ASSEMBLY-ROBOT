#!/usr/bin/env python
import rospy
import rospkg

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from fiducial_msgs.msg import FiducialTransformArray

from std_srvs.srv import Empty

import os


OBJECT_LIST = ["chair_part1.SLDPRT.urdf", "chair_part2.SLDPRT.urdf", 
			"chair_part3.SLDPRT.urdf", "chair_part4.SLDPRT.urdf",
			"chair_part5.SLDPRT.urdf", "chair_part6.SLDPRT.urdf"]

ROSPACK = rospkg.RosPack()
SCALE_FACTOR = 4.24


class Spawner:
	def __init__(self):
		self.object_existences = [False, False, False, False, False, False]
		self.object_transforms = [False, False, False, False, False, False]
		self.model_names_in_gazebo = []
		self.scale_factor = SCALE_FACTOR

		self.reference_frame = rospy.get_param('~reference_frame', 'camera_global')
		self.pkg_path = ROSPACK.get_path("env_setting")
		self.urdfs_path = os.path.join(self.pkg_path, "urdf")

		rospy.wait_for_service("gazebo/spawn_urdf_model")
		rospy.wait_for_service("gazebo/delete_model")
		rospy.wait_for_service("gazebo/pause_physics")
		rospy.wait_for_service("gazebo/unpause_physics")

		self.client_spawn = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
		self.client_delete = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
		self.client_pause = rospy.ServiceProxy("gazebo/pause_physics", Empty)
		self.client_unpause = rospy.ServiceProxy("gazebo/unpause_physics", Empty)

		rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.tf_callback, queue_size=10)
		rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback, queue_size=10)

		rospy.Service("spawn_objects", Empty, self.spawn_all_models)


	def tf_callback(self, data):
		object_existences = [False, False, False, False, False, False]
		object_transforms = [False, False, False, False, False, False]

		transforms = data.transforms

		for tf in transforms:
			object_id = tf.fiducial_id
			object_tf = tf.transform

			object_index = object_id - 1
			object_existences[object_index] = True
			object_transforms[object_index] = object_tf

		self.object_existences = object_existences
		self.object_transforms = object_transforms


	def model_states_callback(self, data):
		self.model_names_in_gazebo = data.name


	def spawn_specific_model(self, object_num):
		self.client_pause()
		index = object_num - 1
		object_name = OBJECT_LIST[index]

		if not self.object_existences[index]:
			print("\n{} dosen't exist in the image from camera".format(object_name))
			
		else:
			print("\nSpawning {} ...".format(object_name))

			urdf_path = os.path.join(self.urdfs_path, object_name)
			with open(urdf_path, "r") as urdf_file:
				model_xml = urdf_file.read()
				urdf_file.close()

			urdf_name = object_name.split(".")[0]

			try:
				self.model_names_in_gazebo.index(urdf_name)
				self.client_delete(urdf_name)
				print("\t{} already exists in gazebo, so delete and respawn".format(urdf_name))

			except ValueError:
				print("\tJust spawn")
				pass

			urdf_point = self.object_transforms[index].translation
			urdf_quaternion = self.object_transforms[index].rotation

			urdf_point.x /= self.scale_factor 
			urdf_point.y /= self.scale_factor 
			urdf_point.z /= self.scale_factor 

			print(urdf_point)
			print(self.object_transforms)

			urdf_pose = Pose(urdf_point, urdf_quaternion)
			self.client_spawn(urdf_name, model_xml, "", urdf_pose, self.reference_frame)


	def spawn_all_models(self, req):
		print("\n-------------------------------")
		self.client_unpause()
		rospy.sleep(1)

		for obj in OBJECT_LIST:
			obj_num = OBJECT_LIST.index(obj) + 1
			self.spawn_specific_model(obj_num)
		return []


if __name__ == "__main__":	
	try :
		rospy.init_node("object_spawner")
		spawner = Spawner()
		rospy.loginfo("Spawner ready")
		rospy.spin()

	except rospy.ROSInterruptException:
		pass