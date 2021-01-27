#!/usr/bin/env python
import rospy

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import DeleteModel

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

import numpy as np
import math as m
import tf.transformations as tf



def main():
	client_spawn = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
	client_get_model = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
	client_delete = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

	param_names = rospy.get_param_names()
	object_param = [param for param in param_names if param.startswith("/object")][0]
	object_xml = rospy.get_param(object_param)

	object_xyz = Point(0, 0, 0)
	rpy = np.array([0, 0, 0])
	quat = tf.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
	object_rpy = Quaternion(quat[0], quat[1], quat[2], quat[3])
	object_pose = Pose(object_xyz, object_rpy)

	object_name = "object"
	object_namespace = ""
	reference = "world"

	already = client_get_model(object_name, "world").success
	
	if not already:
		pass
	else:
		print("'{}' already exists in gazebo, so delete and respawn this.".format(object_name))
		client_delete(object_name)

	client_spawn(object_name, object_xml, object_namespace, object_pose, reference)



if __name__ == "__main__":	
	try :
		rospy.init_node("simple_object_spawner")
		main()

	except rospy.ROSInterruptException:
		pass