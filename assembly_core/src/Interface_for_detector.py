import rospy
import os
import csv
import numpy as np
import tf.transformations as tf

from geometry_msgs.msg import PoseStamped

import rospkg

r = rospkg.RosPack()
r_path = r.get_path("object_description")
TEMP_COORDINATE_PATH = os.path.join(r_path, "temp_pose")
INITIAL_COORDINATE_PATH = "/home/cai/share_for_compt/for_yoon/part_coordinate"

class InterfaceForDetector():
    def __init__(self, start_at_specific_point=False):
        self.initial_coordinate_path = INITIAL_COORDINATE_PATH
        self.temp_coordinate_path = TEMP_COORDINATE_PATH
        self.check_for_ready_to_start()

        if not start_at_specific_point:
            self.initial_obj_pose_dict = self.read_initial_pose(self.initial_coordinate_path)
        else:
            self.initial_obj_pose_dict = self.read_initial_pose(self.temp_coordinate_path)

    def check_for_ready_to_start(self):
        print("--- Wait for_ready...")
        req_file_name = "01.txt"
        req_file_path = os.path.join(self.initial_coordinate_path, req_file_name)
        while True:
            with open(req_file_path) as f:
                data = f.read()
                if "0" in data:
                    break
                else:
                    continue
        print("--- Ready for assembly task!")

    def read_initial_pose(self, dir_path):
        obj_pose_dict = {}
        file_list = os.listdir(dir_path)
        for file_name in file_list:
            obj_name = file_name.split(".")[0]
            if "part" in obj_name or "C" in obj_name:
                obj_pose, success = self.read_pose_file(file_name, dir_path)
                if success:
                    obj_pose_dict[obj_name] = obj_pose
        return obj_pose_dict
    
    def read_pose_file(self, file_name, dir_path, verbose=0):
        if verbose == 1:
            self.req_for_refresh()
        else:
            pass
        target_path = os.path.join(dir_path, file_name)
        try:
            target_pose = self.get_pose_from_read_file(target_path)
            return target_pose, True
        except IOError:
            return None, False

    def get_pose_from_read_file(self, file_path):
        with open(file_path) as f:
            reader = csv.reader(f)
            lines = list(reader)
            ref_name = lines[0][0]
            part_name = lines[1][0]
            xyz = np.array([lines[2][0], lines[2][1], lines[2][2]], np.float)
            if len(lines[3]) == 3:
                rpy = np.array([lines[3][0], lines[3][1], lines[3][2]], np.float)
                quat = tf.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
            elif len(lines[4]) == 4:
                quat = np.array([lines[3][0], lines[3][1], lines[3][2], lines[3][3]], np.float)

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
        req_file_path = os.path.join(self.initial_coordinate_path, req_file_name)
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
        tf_mat = self.get_tf_matrix(tr, quat)
        return tf_mat

    def get_tf_matrix(self, xyz, rot):
        tr_mat = tf.translation_matrix(xyz)
        if len(rot) == 3:
            rot_mat = tf.euler_matrix(rot[0], rot[0], rot[0])
        elif len(rot) == 4:
            rot_mat = tf.quaternion_matrix(rot)
        else:
            print("Given rot component's number is wrong!")
            return TypeError
        tf_mat = tf.concatenate_matrices(tr_mat, rot_mat)
        tf_mat[np.abs(tf_mat) < 1e-5] = 0
        return tf_mat