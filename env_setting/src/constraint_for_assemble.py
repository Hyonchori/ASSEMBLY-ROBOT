#!/usr/bin/env python
import rospy
import rospkg

from gazebo_msgs.srv import SpawnModel

import os
import xmltodict





OBJECT_LIST = ["chair_part1.SLDPRT.urdf", "chair_part2.SLDPRT.urdf", 
			"chair_part3.SLDPRT.urdf", "chair_part4.SLDPRT.urdf",
			"chair_part5.SLDPRT.urdf", "chair_part6.SLDPRT.urdf"]

ROSPACK = rospkg.RosPack()




class ReadParam_and_SetContraint:
	def __init__(self):
		self.param_names = rospy.get_param_names()
		self.chair_param_names = [param for param in self.param_names \
									if param.startswith("/chair_part")]
		self.chair_param_names.sort()
		self.component_param_names = [param for param in self.param_names\
										if param.startswith("/component")]
		self.component_param_names.sort()
		self.object_param_names = self.chair_param_names + self.component_param_names

		self.object_constraints = self.get_object_constraints()



	def read_param(self, param_name="description"):
		description = rospy.get_param(param_name)
		dict_type = xmltodict.parse(description)
		return dict_type


	def extract_contraint_info(self, dict_type):
		print("\n-----------------------------------------")
		hole = []; shaft = []; surface = []
		joints = dict_type["robot"]["joint"]

		if not type(joints)==list:
			joints = [joints]
		else:
			pass

		for joint in joints:
			link_name = str(joint["parent"]["@link"])
			joint_name = str(joint["@name"])
			link_type = joint_name.split("_")[0]
			link_num = int(joint_name.split("_")[1])
			joint_xyz = joint["origin"]["@xyz"].split(" ")
			joint_xyz = map(float, joint_xyz)
			joint_rpy = joint["origin"]["@rpy"].split(" ")
			joint_rpy = map(float, joint_rpy)
			
			if link_type == "hole":
				hole.append([link_name, link_num, joint_xyz, joint_rpy])

			elif link_type == "shaft":
				shaft.append([link_name, link_num, joint_xyz, joint_rpy])

			elif link_type == "surface":
				surface.append([link_name, link_num, joint_xyz, joint_rpy])

			else:
				pass

		constraint = {"hole":hole, "shaft":shaft, "surface":surface}
		return constraint


	def get_object_constraints(self):
		constraints = []

		for param_name in self.object_param_names:
			print(param_name)
			object_param_dict = self.read_param(param_name)
			object_constraint = self.extract_contraint_info(object_param_dict)
			constraints.append(object_constraint)

		return constraints


	def spawn_specific_model(self, object_description):
		pass






def main():
	asb = ReadParam_and_SetContraint()
	oc = asb.object_constraints


	for i in oc:
		print("\n--------------------")
		#print(i)





if __name__ == "__main__":	
	try :
		rospy.init_node("assemble_test")
		main()

	except rospy.ROSInterruptException:
		pass