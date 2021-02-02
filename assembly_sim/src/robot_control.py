#!/usr/bin/env python
import rospy
import sys
import copy

import actionlib
import control_msgs.msg

import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty

from pprint import pprint
import time


__metaclass__ = type
class BasicController():
    def __init__(self, robot_description, joint_state_topic):
        self.client_spawn = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
        self.client_get_model = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
        self.client_set_model = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
        self.client_delete = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.client_get_physics = rospy.ServiceProxy("gazebo/get_physics_properties", \
            GetPhysicsProperties)
        self.client_set_physics = rospy.ServiceProxy("gazebo/set_physics_properties", \
            SetPhysicsProperties)
        self.client_pause = rospy.ServiceProxy("gazebo/pause_physics", Empty)
        self.client_unpause = rospy.ServiceProxy("gazebo/unpause_physics", Empty)
        self.client_reset = rospy.ServiceProxy("gazebo/reset_world", Empty)
        self.wait_for_gazebo_services()
        self.gazebo_setting()
        
        self.robot_description = robot_description
        self.joint_state_topic = joint_state_topic
        
        self.mc = moveit_commander
        self.mc.roscpp_initialize(self.joint_state_topic)
        self.robot = self.mc.RobotCommander(self.robot_description)

        self.arm1 = self.mc.MoveGroupCommander("arm1", self.robot_description)
        self.arm2 = self.mc.MoveGroupCommander("arm2", self.robot_description)
        self.hand1 = self.mc.MoveGroupCommander("hand1", self.robot_description)
        self.hand2 = self.mc.MoveGroupCommander("hand2", self.robot_description)

        
        
        self.ready()
    
    def wait_for_gazebo_services(self):
        print("--- Wait for gazebo service...")
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        rospy.wait_for_service("gazebo/get_model_state")
        rospy.wait_for_service("gazebo/set_model_state")
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/get_physics_properties")
        rospy.wait_for_service("gazebo/set_physics_properties")
        rospy.wait_for_service("gazebo/pause_physics")
        rospy.wait_for_service("gazebo/unpause_physics")
        rospy.wait_for_service("gazebo/reset_world")
        rospy.wait_for_service("init_obj_spawn")
        print("--- All gazebo service servers are ready!")
    
    def ready(self):
        self.arm1.set_named_target("arm1_ready")
        self.arm2.set_named_target("arm2_ready")
        plan1 = self.arm1.plan()
        plan2 = self.arm2.plan()
        self.arm1.execute(plan1)
        self.arm2.execute(plan2)

    def gazebo_setting(self):
        temp_phisics = self.client_get_physics()
        temp_phisics.gravity.z = 0.0
        self.client_set_physics(temp_phisics.time_step,
                                temp_phisics.max_update_rate,
                                temp_phisics.gravity,
                                temp_phisics.ode_config)
        self.client_unpause()

def main():
    robot_description = "/robot_description"
    joint_state_topic = ["joint_states:=/joint_states"]
    controller = BasicController(robot_description, joint_state_topic)

if __name__ == "__main__":	
	try :
		rospy.init_node("move_test")
		main()

	except rospy.ROSInterruptException:
		pass