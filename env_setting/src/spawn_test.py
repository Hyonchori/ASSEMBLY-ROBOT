#!/usr/bin/env python
import rospy
import rospkg

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

from tf2_msgs.msg import TFMessage

from std_srvs.srv import Empty

from tf.transformations import quaternion_multiply
from tf.transformations import euler_from_quaternion
import numpy as np
import os

from coord_tf import Pose_Transform


urdf_list = ["chair_part1.SLDPRT.urdf", "chair_part2.SLDPRT.urdf", 
			"chair_part3.SLDPRT.urdf", "chair_part4.SLDPRT.urdf",
			"chair_part5.SLDPRT.urdf", "chair_part6.SLDPRT.urdf"]
rospack = rospkg.RosPack()


class Spawner:
	def __init__(self):
		self.part_existence = [False, False, False, False, False, False]
		self.part_translations = [False, False, False, False, False, False]
		self.part_rotations = [False, False, False, False, False, False]
		self.model_names_in_gazebo = []
		self.scale_factor = 4.24

		self.pkg_path = rospack.get_path("env_setting")
		self.urdfs_path = os.path.join(self.pkg_path, "urdf")

		rospy.wait_for_service("gazebo/spawn_urdf_model")
		rospy.wait_for_service("gazebo/delete_model")
		rospy.wait_for_service("gazebo/get_model_state")
		rospy.wait_for_service("gazebo/pause_physics")
		rospy.wait_for_service("gazebo/unpause_physics")

		self.spawn_client = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
		self.delete_client = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
		self.model_state_client = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
		self.pause_client = rospy.ServiceProxy("gazebo/pause_physics", Empty)
		self.unpause_client = rospy.ServiceProxy("gazebo/unpause_physics", Empty)
		
		rospy.Subscriber("/tf", TFMessage, self.tf_callback, queue_size=10)
		rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback, queue_size=10)

		self.camera_pose = self.get_camera_pose()


	def get_camera_pose(self):
		camera_pose = self.model_state_client("camera_1", "").pose
		print(camera_pose)

		rx = camera_pose.orientation.x
		ry = camera_pose.orientation.y
		rz = camera_pose.orientation.z
		rw = camera_pose.orientation.w

		euler = euler_from_quaternion(np.array([rx, ry, rz, rw]))
		print(euler)

		return camera_pose


	def tf_callback(self, data):
		transforms = data.transforms[0]

		try:
			urdf_num = int(transforms.child_frame_id[-1])
			part_index = urdf_num - 1

			part_translation = transforms.transform.translation
			part_rotation = transforms.transform.rotation

			self.part_existence[part_index] = True
			self.part_translations[part_index] = part_translation
			self.part_rotations[part_index] = part_rotation

		except ValueError:
			#print("pass")
			pass


	def model_states_callback(self, data):
		self.model_names_in_gazebo = data.name


	def frame_compensation(self, pose):
		new_x = self.camera_pose.position.x - pose.position.x
		new_y = self.camera_pose.position.y + pose.position.y
		new_z = self.camera_pose.position.z - pose.position.z

		new_position = Point(new_x, new_y, new_z)

		camera_rx = self.camera_pose.orientation.x
		camera_ry = self.camera_pose.orientation.y
		camera_rz = self.camera_pose.orientation.z
		camera_rw = self.camera_pose.orientation.w
		camera_qua = np.array([camera_rx, camera_ry, camera_rz, camera_rw])

		marker_rx = pose.orientation.x
		marker_ry = pose.orientation.y
		marker_rz = pose.orientation.z
		marker_rw = pose.orientation.w
		marker_qua = np.array([marker_rx, marker_ry, marker_rz, marker_rw])

		new_qua = quaternion_multiply(camera_qua, marker_qua)
		new_orientation = Quaternion(new_qua[0], new_qua[1], new_qua[2], new_qua[3])

		new_pose = Pose(new_position, new_orientation)
		return new_pose


	def spawn_specific_model(self, num=1):
		self.pause_client()
		index = num-1

		if not self.part_existence[index]:
			print("not_exist")
			pass

		else:
			urdf_path = os.path.join(self.urdfs_path, urdf_list[index])
			with open(urdf_path, "r") as urdf_file:
				model_xml = urdf_file.read()
				urdf_file.close()

			urdf_name = "chair_part_{}".format(str(index+1))

			try:
				self.model_names_in_gazebo.index(urdf_name)
				self.delete_client(urdf_name)
				print("delete")
				
			except ValueError:
				print("pass")
				pass

			urdf_quaternion = self.part_rotations[index]
			urdf_point = self.part_translations[index]



			scaled_x = self.part_translations[index].x / self.scale_factor
			scaled_y = self.part_translations[index].y / self.scale_factor
			scaled_z = self.part_translations[index].z / self.scale_factor

			urdf_point = Point(scaled_x, scaled_y, scaled_z)

			urdf_pose = Pose(urdf_point, urdf_quaternion)
			#compensated_pose = self.frame_compensation(urdf_pose)	
			#compensated_pose = Pose_Transform(urdf_pose, self.camera_pose)
			print(urdf_point)
			#print(compensated_pose)	
			self.spawn_client(urdf_name, model_xml, "", urdf_pose, "camera_1")


	def spawn_whole_models(self):
		self.pause_client()

		for urdf in urdf_list:
			index = urdf_list.index(urdf)

			self.spawn_specific_model(index+1)




def main(spn):
	spn.unpause_client()
	rospy.sleep(2)
	#spn.spawn_specific_model(1)
	spn.spawn_whole_models()
	pass



if __name__ == "__main__":	
	try :
		rospy.init_node("spawn_test")
		spawner = Spawner()
		main(spawner)
		rospy.spin()

	except rospy.ROSInterruptException:
		pass