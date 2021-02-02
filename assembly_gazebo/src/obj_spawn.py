#!/usr/bin/env python
import rospy

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import DeleteModel

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

from std_srvs.srv import Empty

import numpy as np
import math as m
import tf.transformations as tf
import copy
import os
import csv
from pprint import pprint

TARGET_DIR = "/home/cai/share_for_compt/for_yoon/part_coordinate"

class Object_Spawner():
    def __init__(self):
        self.ros_param_names = rospy.get_param_names()
        self.target_param = [param for param in self.ros_param_names if "obj_spawn" in param]

        self.client_spawn 		= rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
        self.client_get_model 	= rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
        self.client_delete 		= rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.client_pause 		= rospy.ServiceProxy("gazebo/pause_physics", Empty)
        self.client_unpause 	= rospy.ServiceProxy("gazebo/unpause_physics",Empty)

        rospy.wait_for_service("gazebo/spawn_urdf_model")
        rospy.wait_for_service("gazebo/get_model_state")
        rospy.wait_for_service("gazebo/delete_model")

        rospy.Service("/init_obj_spawn", Empty, self.server_init_spawn)
    
    def get_pose_by_xyz_rpy(self, xyz, rpy):
        point = Point(xyz[0], xyz[1], xyz[2])
        quat = tf.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        pose = Pose(point, rotation)
        return pose

    def is_in_gazebo(self, model_name, model_ref):
        return self.client_get_model(model_name, model_ref).success

    def spawn_model(self, model_name, model_xml, model_ns, model_pose, model_ref):
        already = self.is_in_gazebo(model_name, model_ref)
        if not already:
            pass
        else:
            rospy.logwarn("'{}' already exists in gazebo, so delete and respawn this part.".format(model_name))
            self.client_delete(model_name)
        self.client_pause()
        return self.client_spawn(model_name, model_xml, model_ns, model_pose, model_ref).success

    def obj_spawn(self, obj_name, obj_xml, obj_ns="", obj_tr=[0, 0, 0.7861], obj_rpy=[0,0,0], obj_ref="table"):
        obj_pose = self.get_pose_by_xyz_rpy(obj_tr, obj_rpy)
        self.spawn_model(obj_name, obj_xml, obj_ns, obj_pose, obj_ref)
    
    def init_spawn(self):
        self.spawning_by_detection()
        print("\nSpawn initializing is finished!")
        print("----------------------------------------")
        
    def server_init_spawn(self, req):
        print("\n=====================================")
        print("Spawn initializing request is recieved!")
        self.init_spawn()
        return []
    
    def spawning_by_detection(self):
        target_dir = TARGET_DIR
        target_files = os.listdir(target_dir)
        target_files.sort()

        for target_file in target_files:
            if target_file.endswith("txt") and \
                (target_file.startswith("p") or target_file.startswith("C")):
                print("\n---------------------")
                print(target_file)
                target_path = os.path.join(target_dir, target_file)
                ref_name, part_name, xyz, rpy = \
                    self.read_txt(target_path)
                xyz[2] += 0.80

                name_split = target_file.split(".")[0]
                param_name = name_split.split("_")[0]
                part_param_name = [param for param in self.target_param if param_name in param][0]
                part_xml = rospy.get_param(part_param_name)
                part_ns = ""

                pose = self.get_pose_by_xyz_rpy(xyz, rpy)
                print(ref_name)
                print(part_name)
                print(pose)

                self.obj_spawn(name_split, part_xml, part_ns, xyz, rpy, ref_name)

            
    
    def read_txt(self, txt_path):
        with open(txt_path) as f:
            reader = csv.reader(f)
            lines = list(reader)
            ref_name = lines[0][0]
            part_name = lines[1][0]
            xyz = np.array([lines[2][0], lines[2][1], lines[2][2]], np.float)
            rpy = np.array([lines[3][0], lines[3][1], lines[3][2]], np.float)
        return ref_name, part_name, xyz, rpy

def main():
    obj_spawner = Object_Spawner()
    obj_spawner.init_spawn()
    

if __name__ == "__main__":	
    try :
        rospy.init_node("init_obj_spawn")
        main()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass