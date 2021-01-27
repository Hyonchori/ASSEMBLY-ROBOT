#!/usr/bin/env python
import rospy
import rospkg

import os
import glob
import json
import xmltodict
import xml.etree.ElementTree as elemTree


XACRO_LIST = ['chair_part1_post.urdf.xacro', 'chair_part2_post.urdf.xacro'
			  'chair_part3_post.urdf.xacro', 'chair_part4_post.urdf.xacro'
			  'chair_part5_post.urdf.xacro', 'chair_part6_post.urdf.xacro']

def main():
	ROSPACK = rospkg.RosPack()
	pkg_path =  ROSPACK.get_path("object_description")
	xacros_path = os.path.join(pkg_path, "chair_xacro")
	#xacros = glob.glob(xacros_path+"/*")
	xacros = os.listdir(xacros_path)
	chair_xacros = [file for file in xacros if file.startswith("chair_part")]
	chair_xacros.sort()
	chair_path = [xacros_path+"/"+file for file in chair_xacros]

	print(xacros)
	print(chair_xacros)
	print(chair_path)

	dict_type = xmltodict.parse(chair_path[0])
	print(dict_type)


if __name__ == "__main__":	
	try :
		rospy.init_node("read_xml_test")
		main()

	except rospy.ROSInterruptException:
		pass