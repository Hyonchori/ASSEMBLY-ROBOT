#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty

import tf.transformations as tf
import numpy as np
import math as m
import copy



def get_pose_by_xyz_rpy(xyz, rpy):
	point = Point(xyz[0], xyz[1], xyz[2])
	quat = tf.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
	rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])
	pose = Pose(point, rotation)
	return pose

def is_in_gazebo(model_name, model_ref):
	return client_get_model(model_name, model_ref).success

def spawn_model(model_name, model_xml, model_ns, model_pose, model_ref):
	already = is_in_gazebo(model_name, model_ref)
	if not already:
		pass
	else:
		rospy.logwarn("'{}' already exists in gazebo, so delete and respawn this part.".format(model_name))
		client_delete(model_name)
	client_pause()
	return client_spawn(model_name, model_xml, model_ns, model_pose, model_ref).success

param_seq = ['/stefan_101350_14_description', '/stefan_104322_6_description', '/stefan_122620_4_description', '/stefan_122925_4_description', 
'/stefan_part1_1_description', '/stefan_part2_1_description', '/stefan_part3_1_description', 
'/stefan_part4_1_description', '/stefan_part5_1_description', '/stefan_part6_1_description']

def part1_spawn(param):
	part_name = "part1_1"
	part_xml = rospy.get_param(param)
	part_ns = ""
	temp_tr = np.array([0, -0.005, 0.209])
	temp_rpy = np.array([-m.pi/2, 0, 0])
	part_pose = get_pose_by_xyz_rpy(temp_tr, temp_rpy)
	part_ref = "part1_holder"
	spawn_model(part_name, part_xml, part_ns, part_pose, part_ref)

def part2_spawn(param):
	part_name = "part2_1"
	part_xml = rospy.get_param(param)
	part_ns = ""
	temp_tr = np.array([0.2, 0.0045, 0.7861])
	temp_rpy = np.array([0, 0, 0])
	part_pose = get_pose_by_xyz_rpy(temp_tr, temp_rpy)
	part_ref = "table"
	spawn_model(part_name, part_xml, part_ns, part_pose, part_ref)

def part3_spawn(param):
	part_name = "part3_1"
	part_xml = rospy.get_param(param)
	part_ns = ""
	temp_tr = np.array([0.07, 0.30, 0.7861])
	temp_rpy = np.array([0, 0, 0])
	part_pose = get_pose_by_xyz_rpy(temp_tr, temp_rpy)
	part_ref = "table"
	spawn_model(part_name, part_xml, part_ns, part_pose, part_ref)

def part4_spawn(param):
	part_name = "part4_1"
	part_xml = rospy.get_param(param)
	part_ns = ""
	temp_tr = np.array([0.32, 0.16, 0.7861])
	temp_rpy = np.array([0, 0, 0])
	part_pose = get_pose_by_xyz_rpy(temp_tr, temp_rpy)
	part_ref = "table"
	spawn_model(part_name, part_xml, part_ns, part_pose, part_ref)

def part5_spawn(param):
	part_name = "part5_1"
	part_xml = rospy.get_param(param)
	part_ns = ""
	temp_tr = np.array([-0.495, -0.25, 0.7861])
	temp_rpy = np.array([0, 0, 0])
	part_pose = get_pose_by_xyz_rpy(temp_tr, temp_rpy)
	part_ref = "table"
	spawn_model(part_name, part_xml, part_ns, part_pose, part_ref)

def part6_spawn(param):
	part_name = "part6_1"
	part_xml = rospy.get_param(param)
	part_ns = ""
	temp_tr = np.array([0, -0.005, 0.7861])
	temp_rpy = np.array([0, 0, 0])
	part_pose = get_pose_by_xyz_rpy(temp_tr, temp_rpy)
	part_ref = "table"
	spawn_model(part_name, part_xml, part_ns, part_pose, part_ref)

def _101350_spawn(param):
	part_total_num = int(param.split("_")[2])
	part_xml = rospy.get_param(param)
	part_ns = ""
	part_ref = "pin_holder"
	temp_tr1 = np.array([-0.075, 0.05, 0.013])
	temp_tr2 = np.array([-0.075, 0.025, 0.013]) 
	temp_rpy = np.array([0, 0, 0])
	
	for i in range(0, part_total_num):
		part_name = "101350_{}".format(i+1)
		scale = i/2
		if i%2 == 0:
			tr = copy.deepcopy(temp_tr1)
			tr += np.array([0.025, 0, 0]) * scale
		else:
			tr = copy.deepcopy(temp_tr2)
			tr += np.array([0.025, 0, 0]) * scale
		part_pose = get_pose_by_xyz_rpy(tr, temp_rpy)

		spawn_model(part_name, part_xml, part_ns, part_pose, part_ref)

def _104322_spawn(param):
	part_total_num = int(param.split("_")[2])
	part_xml = rospy.get_param(param)
	part_ns = ""
	part_ref = "pin_holder"
	temp_tr = np.array([-0.075, 0.00, 0.073])
	temp_rpy = np.array([m.pi, 0, 0])
	
	for i in range(0, part_total_num):
		part_name = "104322_{}".format(i+1)
		scale = i
		tr = copy.deepcopy(temp_tr)
		tr += np.array([0.025, 0, 0]) * scale
		part_pose = get_pose_by_xyz_rpy(tr, temp_rpy)

		spawn_model(part_name, part_xml, part_ns, part_pose, part_ref)


def _122620_spawn(param):
	part_total_num = int(param.split("_")[2])
	part_xml = rospy.get_param(param)
	part_ns = ""
	part_ref = "pin_holder"
	temp_tr = np.array([-0.075, 0.01+-0.025, 0.033])
	temp_rpy = np.array([m.pi/2, 0, 0])
	
	for i in range(0, part_total_num):
		part_name = "122620_{}".format(i+1)
		scale = i
		tr = copy.deepcopy(temp_tr)
		tr += np.array([0.025, 0, 0]) * scale
		part_pose = get_pose_by_xyz_rpy(tr, temp_rpy)

		spawn_model(part_name, part_xml, part_ns, part_pose, part_ref)


def _122925_spawn(param):
	part_total_num = int(param.split("_")[2])
	part_xml = rospy.get_param(param)
	part_ns = ""
	part_ref = "pin_holder"
	temp_tr = np.array([-0.075, -0.05, 0.0249])
	temp_rpy = np.array([m.pi, 0, 0])
	
	for i in range(0, part_total_num):
		part_name = "122925_{}".format(i+1)
		scale = i
		tr = copy.deepcopy(temp_tr)
		tr += np.array([0.025, 0, 0]) * scale
		part_pose = get_pose_by_xyz_rpy(tr, temp_rpy)

		spawn_model(part_name, part_xml, part_ns, part_pose, part_ref)

		






def main():
	param_names = rospy.get_param_names()
	param_names.sort()
	stefan_params = [param for param in param_names if param.startswith("/stefan")]

	'''
	for param in stefan_params[-6:]:
		param_split = param.split("_")
		xml = rospy.get_param(param)
		name = param_split[1]
		num = int(param_split[2])

		for i in range(1, num+1):
			model_name = "{}_{}".format(name, i)
			spawn_model(model_name, xml, "", part1_pose, "world")
	'''
	
	part1_spawn(stefan_params[4])
	part2_spawn(stefan_params[5])
	part3_spawn(stefan_params[6])
	part4_spawn(stefan_params[7])
	part5_spawn(stefan_params[8])
	part6_spawn(stefan_params[9])
	

	_101350_spawn(stefan_params[0])
	_104322_spawn(stefan_params[1])
	_122620_spawn(stefan_params[2])
	_122925_spawn(stefan_params[3])











if __name__ == "__main__":
	try:
		rospy.init_node("spawn_chair_parts")

		client_spawn 		= rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
		client_get_model 	= rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
		client_delete 		= rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
		client_pause 		= rospy.ServiceProxy("gazebo/pause_physics", Empty)
		client_unpause 		= rospy.ServiceProxy("gazebo/unpause_physics",Empty)

		main()
	
	except rospy.ROSInterruptException:
		pass