import numpy as np
import math as m
import tf.transformations as tf

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

# This file receives two ros_geometry_Pose data and
# transforms one pose regarding another pose as local coordinate and
# outputs a pose viewed by gloabl coordinate

# pose : The pose viwed by local coordinate
# (position, orientation)

# target_coord : The local coordinate system that we use to transform above pose
# (position, orientation)

def Pose_Transform(pose, target_coord):

	#position of pose
	x = pose.position.x
	y = pose.position.y
	z = pose.position.z
	pose_matrix = np.array([[x], [y], [z], [1]])

	#quaternion of pose
	qx = pose.orientation.x
	qy = pose.orientation.y
	qz = pose.orientation.z
	qw = pose.orientation.w
	quaternion = np.array([qx, qy, qz, qw])
	
	# Calculating homogeneous transformation matrix with shape(4,4)
	x_offset = target_coord.position.x
	y_offset = target_coord.position.y
	z_offset = target_coord.position.z

	offset_quaternion = np.array([target_coord.orientation.x,
								  target_coord.orientation.y,
								  target_coord.orientation.z,
								  target_coord.orientation.w])

	offset_euler = tf.euler_from_quaternion(offset_quaternion)
	rx_offset = offset_euler[0]
	ry_offset = offset_euler[1]
	rz_offset = offset_euler[2]

	tf_matrix_translation = np.array([[1, 0, 0, x_offset],
									  [0, 1, 0, y_offset],
									  [0, 0, 1, z_offset],
									  [0, 0, 0, 1]])

	tf_matrix_rotation_x = np.array([[1, 0, 0, 0],
									 [0, m.cos(rx_offset), -m.sin(rx_offset), 0],
									 [0, m.sin(rx_offset), m.cos(rx_offset), 0],
									 [0, 0, 0, 1]])

	tf_matrix_rotation_y = np.array([[m.cos(ry_offset), 0, m.sin(ry_offset), 0],
									 [0, 1, 0, 0],
									 [-m.sin(ry_offset), 0, m.cos(ry_offset), 0],
									 [0, 0, 0, 1]])

	tf_matrix_rotation_z = np.array([[m.cos(rz_offset), -m.sin(rz_offset), 0, 0],
									 [m.sin(rz_offset), m.cos(rz_offset), 0, 0 ],
									 [0, 0, 1, 0],
									 [0, 0, 0, 1]])

	print(tf_matrix_translation)
	print(tf_matrix_rotation_x)

	tf_t_rx = np.matmul(tf_matrix_translation, tf_matrix_rotation_x)
	tf_t_rx_ry = np.matmul(tf_t_rx, tf_matrix_rotation_y)
	tf_t_rx_ry_rz = np.matmul(tf_t_rx_ry, tf_matrix_rotation_z)

	new_quaternion = tf.quaternion_multiply(offset_quaternion, quaternion)
	new_orientation = Quaternion(new_quaternion[0], 
								 new_quaternion[1],
								 new_quaternion[2], 
								 new_quaternion[3])

	new_pose_matrix = np.matmul(tf_t_rx_ry_rz, pose_matrix)
	new_position = Point(new_pose_matrix[0][0], 
						 new_pose_matrix[1][0], 
						 new_pose_matrix[2][0])

	new_pose = Pose(new_position, new_orientation)
	return new_pose