import rospy
import rospkg

import os
import sys
import copy
import moveit_commander

from tf2_ros import StaticTransformBroadcaster
import tf.transformations as tf

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

from assembly_robot_msgs.msg import Asm_test
from assembly_robot_msgs.srv import asm_Srv
from assembly_robot_msgs.srv import check_Srv
from assembly_robot_msgs.srv import findRefHoles_Srv

import grasp_poses

from pprint import pprint

r = rospkg.RosPack()

class InterfaceForRobot():
    def __init__(self, cowork=False):
        self.stl_pkg_path = r.get_path("object_description")
        self.grasping_pose = grasp_poses.grasping_pose
        self.br = StaticTransformBroadcaster()
        
        if cowork:
            moveit_commander.roscpp_initialize(sys.argv)
            self.scene = moveit_commander.PlanningSceneInterface()
            self.client_for_robot = rospy.ServiceProxy("/to_HoleCheck", asm_Srv)
            self.client_asm_check = rospy.ServiceProxy("/to_RobotControl", asm_Srv)
            rospy.wait_for_service("/to_HoleCheck")
            rospy.wait_for_service("/to_RobotControl")
        else:
            pass

    def req_msg_for_HoleCheck(self,
        assembly_type,
        ref_obj_name, ref_const_names,
        move_obj_name, move_const_names):
        parent = Asm_test([ref_obj_name], ref_const_names)
        child = Asm_test([move_obj_name], move_const_names)
        print("\n--- HoleCheck")
        print(assembly_type, parent, child)
        resp = self.client_for_robot(assembly_type, parent, child)
        print(resp)
        return resp
        
    def req_msg_for_RobotControl(self,
        assembly_type,
        ref_obj_name, ref_const_names,
        move_obj_name, move_const_names):
        parent = Asm_test([ref_obj_name], ref_const_names)
        child = Asm_test([move_obj_name], move_const_names)
        print("\n--- RobotControl")
        print(assembly_type, parent, child)
        resp = self.client_asm_check(assembly_type, parent, child)
        print(resp)
        return resp
    
    def publish_obj_tf(self, obj, obj_pose):
        target_posestamped = PoseStamped()
        target_posestamped.header.stamp = rospy.Time.now()
        target_posestamped.header.frame_id = "table"
        target_posestamped.pose = obj_pose
        target_name = obj.referencePart
        target_type = target_name.split("_")[0]
        stl_path = os.path.join(self.stl_pkg_path, "urdfs", target_type+".SLDPRT", \
            "meshes", target_type+".SLDPRT.STL")
        self.scene.add_mesh(target_name, target_posestamped, stl_path)
        self.send_tf_by_pose(obj_pose, target_name, "table")
        self.send_const_tf(obj, obj_pose)
        self.send_grasp_tf(obj)

    def publish_just_tf(self, obj_name, obj_pose, reference):
        target_posestamped = PoseStamped()
        target_posestamped.header.stamp = rospy.Time.now()
        target_posestamped.header.frame_id = reference
        target_posestamped.pose = obj_pose
        target_name = obj_name
        target_type = target_name.split("_")[0]
        stl_path = os.path.join(self.stl_pkg_path, "urdfs", target_type+".SLDPRT", \
            "meshes", target_type+".SLDPRT.STL")
        self.scene.add_mesh(target_name, target_posestamped, stl_path)
        self.send_tf_by_pose(obj_pose, target_name, reference)
    
    def send_tf_by_pose(self, pose, target_name, ref_name="world"):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = ref_name
        t.child_frame_id = target_name
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w
        self.br.sendTransform(t)
    
    def send_const_tf(self, obj, obj_pose):
        target_consts = obj.assemConsts
        for const_name, const in target_consts.items():
            const_end_coor = const["endCoordinate"]
            const_end_pose = self.get_pose_from_tf_mat(const_end_coor).pose
            self.send_tf_by_pose(const_end_pose, const_name+"_end", const["parent"])
            
            const_entry_coor = const["entryCoordinate"]
            const_entry_pose = self.get_pose_from_tf_mat(const_entry_coor).pose
            self.send_tf_by_pose(const_entry_pose, const_name+"_entry", const["parent"])
            

    def send_grasp_tf(self, obj):
        target_obj_type = obj.obj_type
        target_obj_name = obj.referencePart
        if target_obj_type in self.grasping_pose.keys():
            target_grasping_pose = self.grasping_pose[target_obj_type]
            idx = 1
            for grasping_pose_dict in target_grasping_pose:
                grasp_tr = grasping_pose_dict["tr"]
                grasp_rot = grasping_pose_dict["rot"]
                grasp_pose = self.get_pose_from_tr_rot(grasp_tr, grasp_rot).pose
                pose_name = "{}-{}-{}".format(target_obj_name, "GRASP", idx)
                #print(pose_name)
                self.send_tf_by_pose(grasp_pose, pose_name, target_obj_name)
                idx += 1
        else:
            pass

    def get_pose_from_tr_quat(self, tr, quat):
        posestamped = PoseStamped()
        posestamped.pose.position.x = tr[0]
        posestamped.pose.position.y = tr[1]
        posestamped.pose.position.z = tr[2]
        posestamped.pose.orientation.x = quat[0]
        posestamped.pose.orientation.y = quat[1]
        posestamped.pose.orientation.z = quat[2]
        posestamped.pose.orientation.w = quat[3]
        return posestamped
    
    def get_pose_from_tr_rot(self, tr, rot):
        quat = tf.quaternion_from_euler(rot[0], rot[1], rot[2])
        posestamped = PoseStamped()
        posestamped.pose.position.x = tr[0]
        posestamped.pose.position.y = tr[1]
        posestamped.pose.position.z = tr[2]
        posestamped.pose.orientation.x = quat[0]
        posestamped.pose.orientation.y = quat[1]
        posestamped.pose.orientation.z = quat[2]
        posestamped.pose.orientation.w = quat[3]
        return posestamped

    def get_pose_from_tf_mat(self, tf_mat):
        quat = tf.quaternion_from_matrix(tf_mat)
        tr = tf.translation_from_matrix(tf_mat)
        posestamped = self.get_pose_from_tr_quat(tr, quat)
        return posestamped
