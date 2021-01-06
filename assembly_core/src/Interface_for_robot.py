import rospy
import os
import sys
import copy

import moveit_commander
import tf.transformations as tf
from tf2_ros import StaticTransformBroadcaster

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from assembly_robot_msgs.msg import Asm_test
from assembly_robot_msgs.srv import asm_Srv
from assembly_robot_msgs.srv import check_Srv
from assembly_robot_msgs.srv import findRefHoles_Srv

import rospkg
import grasp_poses

r = rospkg.RosPack()
r_path = r.get_path("object_description")

class InterfaceForRobot():
    def __init__(self, cowork=False):
        self.stl_pkg_dir = r_path
        self.grasping_pose = grasp_poses.grasping_pose
        self.br = StaticTransformBroadcaster()

        if cowork:
            moveit_commander.roscpp_initialize(sys.argv)
            self.scene = moveit_commander.PlanningSceneInterface()
            self.client_for_HoleCheck = rospy.ServiceProxy("/to_HoleCheck", asm_Srv)
            self.client_for_RobotControl = rospy.ServiceProxy("/to_RobotControl", asm_Srv)
            #self.wait_for_services()
        else:
            pass

    def wait_for_services(self):
        rospy.logwarn("\n--- Wait for services ... ---")
        rospy.wait_for_service("/to_HoleCHeck")
        rospy.wait_for_service("/to_RobotControl")
        rospy.logwarn("\nWhole servers are open!\n")

    def publish_init_tf(self, obj_dict):
        obj_name_list = obj_dict.keys()
        obj_name_list.sort()
        for obj_name in obj_name_list:
            obj = obj_dict[obj_name]
            rospy.logerr(obj_name)
            obj_real_pose = obj.real_pose_mat
            obj_posestamped_msg = self.get_posestampedMSG_from_mat(obj_real_pose, "table")

            self.send_tf_from_posestampedMSG(obj_posestamped_msg, obj_name)
            self.send_holepin_tf(obj)
            self.send_grasp_tf(obj)      

            stl_path = self.get_obj_stl_path(obj.obj_type)
            self.scene.add_mesh(obj_name, obj_posestamped_msg, stl_path)   
    
    def get_posestampedMSG_from_mat(self, tf_mat, frame_id):
        tr = tf.translation_from_matrix(tf_mat)
        quat = tf.quaternion_from_matrix(tf_mat)
        posestamped = PoseStamped()
        posestamped.header.stamp = rospy.Time.now()
        posestamped.header.frame_id = frame_id
        posestamped.pose.position.x = tr[0]
        posestamped.pose.position.y = tr[1]
        posestamped.pose.position.z = tr[2]
        posestamped.pose.orientation.x = quat[0]
        posestamped.pose.orientation.y = quat[1]
        posestamped.pose.orientation.z = quat[2]
        posestamped.pose.orientation.w = quat[3]
        return posestamped
    
    def get_obj_stl_path(self, obj_type):
        stl_path = os.path.join(
            self.stl_pkg_dir, 
            "urdfs", 
            obj_type+".SLDPRT", 
            "meshes", 
            obj_type+".SLDPRT.STL"
            )
        return stl_path
    
    def update_just_parent_tf_stl(self, obj):
        obj_real_pose = obj.real_pose_mat
        obj_posestamped_msg = self.get_posestampedMSG_from_mat(obj_real_pose, "table")
        self.send_tf_from_posestampedMSG(obj_posestamped_msg, obj.obj_name)
        stl_path = self.get_obj_stl_path(obj.obj_type)
        self.scene.add_mesh(obj.obj_name, obj_posestamped_msg, stl_path)  
    
    def send_tf_from_posestampedMSG(self, posestamped, target_name):
        t = TransformStamped()
        t.child_frame_id = target_name
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = posestamped.header.frame_id
        t.transform.translation.x = posestamped.pose.position.x
        t.transform.translation.y = posestamped.pose.position.y
        t.transform.translation.z = posestamped.pose.position.z
        t.transform.rotation.x = posestamped.pose.orientation.x
        t.transform.rotation.y = posestamped.pose.orientation.y
        t.transform.rotation.z = posestamped.pose.orientation.z
        t.transform.rotation.w = posestamped.pose.orientation.w
        print(target_name)
        self.br.sendTransform(t)

    def send_holepin_tf(self, obj):
        for holepin_name, holepin_info in obj.assem_HolePins.items():
            holepin_end_mat = holepin_info["end_coordinate"]
            holepin_end_posestamped = self.get_posestampedMSG_from_mat(holepin_end_mat, obj.obj_name)
            self.send_tf_from_posestampedMSG(holepin_end_posestamped, holepin_name+"_end")

            holepin_entry_mat = holepin_info["entry_coordinate"]
            holepin_entry_posestamped = self.get_posestampedMSG_from_mat(holepin_entry_mat, obj.obj_name)
            self.send_tf_from_posestampedMSG(holepin_entry_posestamped, holepin_name+"_entry")

    def send_grasp_tf(self, obj):
        target_obj_type = obj.obj_type
        if target_obj_type in self.grasping_pose.keys():
            target_grasping_pose = self.grasping_pose[target_obj_type]
            idx = 1
            for grasping_pose_dict in target_grasping_pose:
                grasp_tr = grasping_pose_dict["tr"]
                grasp_rot = grasping_pose_dict["rot"]
                grasp_mat = self.get_mat_from_tr_rot(grasp_tr, grasp_rot)
                grasp_posestamped = self.get_posestampedMSG_from_mat(grasp_mat, obj.obj_name)
                grasp_name = "{}-{}-{}".format(obj.obj_name, "GRASP", idx)
                self.send_tf_from_posestampedMSG(grasp_posestamped, grasp_name)
                idx += 1
        
    def get_mat_from_tr_rot(self, xyz, rot):
        tr_mat = tf.translation_matrix(xyz)
        if len(rot) == 3:
            rot_mat = tf.euler_matrix(rot[0], rot[0], rot[0])
        elif len(rot) == 4:
            rot_mat = tf.quaternion_matrix(rot)
        else:
            print("Given rot component's number is wrong!")
            return TypeError
        tf_mat = tf.concatenate_matrices(tr_mat, rot_mat)
        return tf_mat

    def send_msg_for_RobotControl(self, \
        assembly_type, parent_obj_name, parent_tf_names, child_obj_name, child_tf_names):
        parent = Asm_test([parent_obj_name], parent_tf_names)
        child = Asm_test([child_obj_name], child_tf_names)
        rospy.logwarn("\n--- Send msg to RobotControl")
        rospy.logwarn(assembly_type)
        rospy.logwarn(parent)
        rospy.logwarn(child)
        resp = self.client_for_RobotControl(assembly_type, parent, child)
        rospy.logwarn("\n--- Got resp from RobotControl")
        return resp

    def send_msg_for_HoleCheck(self, \
        assembly_type, parent_obj_name, parent_tf_names, child_obj_name, child_tf_names):
        parent = Asm_test([parent_obj_name], parent_tf_names)
        child = Asm_test([child_obj_name], child_tf_names)
        rospy.logwarn("\n--- Send msg to HoleCheck")
        rospy.logwarn(assembly_type)
        rospy.logwarn(parent)
        rospy.logwarn(child)
        resp = self.client_for_HoleCheck(assembly_type, parent, child)
        rospy.logwarn("\n--- Got resp from HoleCheck")
        return resp

    def get_xyz_quat_from_tfpose(self, tfpose):
        x = tfpose.TransStamped.transform.translation.x
        y = tfpose.TransStamped.transform.translation.y
        z = tfpose.TransStamped.transform.translation.z
        rx = tfpose.TransStamped.transform.rotation.x
        ry = tfpose.TransStamped.transform.rotation.y
        rz = tfpose.TransStamped.transform.rotation.z
        rw = tfpose.TransStamped.transform.rotation.w
        tr = [x, y, z]
        quat = [rx, ry, rz, rw]
        return tr, quat

    def send_redetected_holepin_tf(self, obj, holepin_names, criterion):
        for holepin_name, cri in zip(holepin_names, criterion):
            target_holepin = obj.assem_HolePins[holepin_name]
            target_mat = target_holepin[cri + "_coordinate"]
            target_posestamped = self.get_posestampedMSG_from_mat(target_mat, obj.obj_name)
            self.send_tf_from_posestampedMSG(target_posestamped, holepin_name+"_"+cri)

    def send_component_tf(self, parent_obj, sub_obj):
        pre_comp_list = set(parent_obj.components.keys())
        new_comp_list = set(sub_obj.components.keys())
        target_comp_list = list(new_comp_list - pre_comp_list)

        for child_obj_name in target_comp_list:
            target_component = sub_obj.components[child_obj_name]
            target_tr = target_component["tr"]
            target_quat = target_component["quat"]
            target_mat = sub_obj.get_tf_matrix(target_tr, target_quat)
            target_posestamped = self.get_posestampedMSG_from_mat(target_mat, sub_obj.obj_name)
            self.send_tf_from_posestampedMSG(target_posestamped, child_obj_name)

            child_obj_type = child_obj_name.split("_")[0]
            stl_path = self.get_obj_stl_path(child_obj_type)
            self.scene.add_mesh(child_obj_name, target_posestamped, stl_path) 

    def update_component_stl(self, obj):
        target_comp_list = obj.components.keys()

        for child_obj_name in target_comp_list:
            target_component = obj.components[child_obj_name]
            target_tr = target_component["tr"]
            target_quat = target_component["quat"]
            target_mat = obj.get_tf_matrix(target_tr, target_quat)
            target_posestamped = self.get_posestampedMSG_from_mat(target_mat, obj.obj_name)
            self.send_tf_from_posestampedMSG(target_posestamped, child_obj_name)

            child_obj_type = child_obj_name.split("_")[0]
            stl_path = self.get_obj_stl_path(child_obj_type)
            self.scene.add_mesh(child_obj_name, target_posestamped, stl_path) 