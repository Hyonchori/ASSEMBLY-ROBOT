import rospy
import os
import csv
import copy
import numpy as np
import tf.transformations as tf
from geometry_msgs.msg import PoseStamped

import utils

COORDINATE_PATH = "/home/cai/share_for_compt/for_yoon/part_coordinate"

class InterfaceForDetector():
    def __init__(self):
        self.dir_path = COORDINATE_PATH
        self.check_for_ready_to_start()
        self.initial_obj_pose_dict = self.initial_detect()
    
    def check_for_ready_to_start(self):
        print("--- Wait for_ready...")
        req_file_name = "01.txt"
        req_file_path = os.path.join(self.dir_path, req_file_name)
        while True:
            with open(req_file_path) as f:
                data = f.read()
                if "0" in data:
                    break
                else:
                    continue
        print("--- Ready for assembly task!")
    
    def initial_detect(self):
        obj_pose_dict = {}
        obj_list = os.listdir(self.dir_path)
        for file_name in obj_list:
            obj_name = file_name.split(".")[0]
            if "PART" in obj_name or "C" in obj_name:
                obj_pose, success = self.get_detect_info(obj_name)
                if success:
                    obj_pose_dict[obj_name] = obj_pose
        return obj_pose_dict
    
    def get_detect_info(self, target_name, verbose=0):
        print("\n------------------------")
        print(target_name)
        if verbose == 1:
            self.req_for_refresh()
        else:
            pass
        target_file = "{}.txt".format(target_name)
        target_path = os.path.join(self.dir_path, target_file)
        print(target_path)
        try:
            target_pose = self.get_pose_by_read_file(target_path)
            return target_pose, True
        except IOError:
            return None, False

    def get_pose_by_read_file(self, file_path):
        with open(file_path) as f:
            reader = csv.reader(f)
            lines = list(reader)
            ref_name = lines[0][0]
            part_name = lines[1][0]
            xyz = np.array([lines[2][0], lines[2][1], lines[2][2]], np.float)
            xyz[2] += 0.02
            rpy = np.array([lines[3][0], lines[3][1], lines[3][2]], np.float)
            quat = tf.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
            
            temp_pose = PoseStamped()
            temp_pose.pose.position.x = xyz[0]
            temp_pose.pose.position.y = xyz[1]
            temp_pose.pose.position.z = xyz[2]
            temp_pose.pose.orientation.x = quat[0]
            temp_pose.pose.orientation.y = quat[1]
            temp_pose.pose.orientation.z = quat[2]
            temp_pose.pose.orientation.w = quat[3]
            temp_pose.header.frame_id = ref_name
            temp_pose.header.stamp = rospy.Time.now()
            return temp_pose

    def req_for_refresh(self):
        print("--- Wait for_refresh...")
        req_file_name = "01.txt"
        req_file_path = os.path.join(self.dir_path, req_file_name)
        with open(req_file_path, "w") as f:
            f.write("1")
        
        while True:
            with open(req_file_path) as f:
                data = f.read()
                if "0" in data:
                    break
                else:
                    continue
        print("--- Refresh is completed!")
    
    def get_tfMat_from_pose(self, pose):
        tr = np.array([pose.pose.position.x, 
                    pose.pose.position.y, 
                    pose.pose.position.z])
        quat = np.array([pose.pose.orientation.x,
                        pose.pose.orientation.y,
                        pose.pose.orientation.z,
                        pose.pose.orientation.w])
        tf_mat = utils.get_tf_matrix(tr, quat)
        return tf_mat
        