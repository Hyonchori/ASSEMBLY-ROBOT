#!/usr/bin/env python
import rospy
import rospkg

import json
import xmltodict
import xml.etree.ElementTree as elemTree



def read_joints_from_description(target="object_name"):
	param_name = "{}_description".format(target)
	if rospy.has_param(param_name):
		pass
	else:
		print("'{}_description' does not exists.".format(target))
		return 

	description = rospy.get_param(param_name)
	dict_type = xmltodict.parse(description)
	joints = dict_type["robot"]["joint"]

	for joint in joints:
			print("\n-------------------------------------------")

			link_name = joint["parent"]["@link"]
			joint_name = joint["@name"]
			joint_xyz = joint["origin"]["@xyz"]
			joint_rpy = joint["origin"]["@rpy"]
			link_type = joint_name.split("_")[0]

			print(link_name)
			print(joint_name)
			print(joint_xyz)
			print(joint_rpy)
			print(link_type)

	return joints



def main():
	joints = read_joints_from_description("chair_part1")
	print(rospy.get_param_names())


if __name__ == "__main__":	
	try :
		rospy.init_node("read_param_test")
		main()

	except rospy.ROSInterruptException:
		pass