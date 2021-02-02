#!/usr/bin/env python
import rospy
import copy

import assembly_description
import assembly_tester

from pprint import pprint

rospy.init_node("core")

CONNECTION_PATH = "/home/cai/share_for_compt/for_kitech/holepin_connection/holepair_except_part1.txt"
TARGET_OBJECT = "stefan"

__metaclass__ = type
class Initial_Core():
    def __init__(self):
        self.connection_path = CONNECTION_PATH
        self.holepin_connection = self.read_holepin_connection(self.connection_path)
        
        self.target_object = TARGET_OBJECT
        self.stefan_dict = self.get_obj_dict(self.target_object)
        self.stefan_dict["part1_1"].is_attachPart = True
        
        self.AT = assembly_tester.Assembly_Tester()
        self.final_dict = self.get_final_obj(self.holepin_connection, self.stefan_dict)
    
    def read_holepin_connection(self, target_path):
        holepin_connection = {}
        with open(target_path) as f:
            lines = f.readlines()
        for line in lines:
            line_split = line.split(" : ")
            const1 = line_split[0]
            const2 = line_split[1]
            const2 = const2.replace("\n", "")

            if const1 not in holepin_connection.keys():
                holepin_connection[const1] = [const2]
            else:
                holepin_connection[const1].append(const2)
            
            if const2 not in holepin_connection.keys():
                holepin_connection[const2] = [const1]
            else:
                holepin_connection[const2].append(const1)

        return holepin_connection

    def get_obj_dict(self, target_object):
        stefan_param_names = [param for param in rospy.get_param_names() \
            if ("assembly_core" in param) and (target_object in param)]
        obj_dict = {}
        for param in stefan_param_names:
            param_description = param.split("/")[-1]
            param_split = param_description.split("_")
            obj_xml = rospy.get_param(param)
            obj_type = param_split[1]
            obj_total_num = int(param_split[2])
            for idx in range(1, obj_total_num+1):
                obj = assembly_description.Assembly_Object(obj_type, obj_xml, idx)
                obj_dict[obj.obj_name] = obj
        return obj_dict
    
    def get_final_obj(self, holepin_connection, obj_dict):
        initial_dict = copy.deepcopy(obj_dict)
        pprint(initial_dict.keys())
        print("")

        self.connect_screw(holepin_connection, initial_dict)
        pprint(initial_dict.keys())
        print("")


        self.connect_insert(holepin_connection, initial_dict)
        pprint(initial_dict.keys())
        print("")


        self.connect_part(holepin_connection, initial_dict)
        pprint(initial_dict.keys())
        print("")


        self.connect_attach(holepin_connection, initial_dict)
        pprint(initial_dict.keys())
        print("")
        
        return initial_dict
    
    def connect_screw(self, holepin_connection, obj_dict):
        for obj_name, obj in obj_dict.items():
            if "C" not in obj_name:
                continue
            
            obj_holepin_set = set(obj.assem_HolePins.keys())
            used_holepin_list = list(obj_holepin_set & set(holepin_connection.keys()))

            for used_holepin_name in used_holepin_list:
                used_holepin = obj.assem_HolePins[used_holepin_name]

                if used_holepin["feature"] != "screw":
                    continue

                mate_holepin_name = holepin_connection[used_holepin_name][0]
                mate_obj_name = mate_holepin_name.split("-")[0]
                mate_obj = obj_dict[mate_obj_name]
                parent_holepin_names = [mate_holepin_name]
                child_holepin_names = [used_holepin_name]

                avail_parent_holepin_names = self.AT.get_avail_holepin_of_obj(mate_obj, parent_holepin_names)
                avail_child_holepin_names = self.AT.get_avail_holepin_of_obj(obj, child_holepin_names)


                self.assembly_task(
                    obj_dict,
                    mate_obj, avail_parent_holepin_names,
                    obj, avail_child_holepin_names
                )
                break
            
    
    def connect_insert(self, holepin_connection, obj_dict):
        for obj_name, obj in obj_dict.items():
            if "C" not in obj_name:
                continue

            obj_holepin_set = set(obj.assem_HolePins.keys())
            used_holepin_list = list(obj_holepin_set & set(holepin_connection.keys()))

            for used_holepin_name in used_holepin_list:
                used_holepin = obj.assem_HolePins[used_holepin_name]

                if used_holepin["contact_with"] is not None:
                    continue

                mate_holepin_name = holepin_connection[used_holepin_name][0]
                if "hole" in used_holepin_name and "hole" in mate_holepin_name:
                    continue

                mate_obj_name = mate_holepin_name.split("-")[0]
                mate_obj = obj_dict[mate_obj_name]
                parent_holepin_names = [mate_holepin_name]
                child_holepin_names = [used_holepin_name]

                avail_parent_holepin_names = self.AT.get_avail_holepin_of_obj(mate_obj, parent_holepin_names)
                avail_child_holepin_names = self.AT.get_avail_holepin_of_obj(obj, child_holepin_names)


                self.assembly_task(
                    obj_dict,
                    mate_obj, avail_parent_holepin_names,
                    obj, avail_child_holepin_names
                )
                break

    def connect_part(self, holepin_connection, obj_dict):
        initial_dict = copy.deepcopy(obj_dict)
        for obj_name, obj in initial_dict.items():

            if obj_name not in obj_dict.keys():
                continue

            temp_obj = obj_dict[obj_name]

            if temp_obj.is_attachPart:
                continue
            
            for target_obj_name, target_obj in obj_dict.items():
                
                if target_obj_name != obj_name:
                    used_holepin_names = []
                    for target_holepin_name, target_holepin_info in target_obj.assem_HolePins.items():
                        if target_holepin_name in holepin_connection:
                            mate_holepin_name_list = holepin_connection[target_holepin_name]
                            
                            for mate_holepin_name in mate_holepin_name_list:
                                if "hole" in target_holepin_name and "hole" in mate_holepin_name:
                                    continue

                                for ref_holepin_name in temp_obj.assem_HolePins.keys():
                                    if ref_holepin_name.startswith(mate_holepin_name):
                                        used_holepin_names.append(target_holepin_name)
                                    
                    used_holepin_names = list(set(used_holepin_names))
                    
                    parent_holepin_names = []
                    for holepin_name in used_holepin_names:
                        mate_holepin_list = holepin_connection[holepin_name]
                        for mate_holepin_name in mate_holepin_list:
                            if mate_holepin_name in temp_obj.assem_HolePins.keys():
                                parent_holepin_names.append(mate_holepin_name)

                    avail_parent_holepin_names = self.AT.get_avail_holepin_of_obj(temp_obj, parent_holepin_names)

                    avail_child_holepin_names = self.AT.get_avail_holepin_of_obj(target_obj, used_holepin_names)

                    if avail_child_holepin_names and avail_parent_holepin_names:
                        self.assembly_task(
                            obj_dict,
                            temp_obj, avail_parent_holepin_names,
                            target_obj, avail_child_holepin_names
                        )
                        temp_obj = obj_dict[obj_name]

    def connect_attach(self, holepin_connection, obj_dict):
        for obj_name, obj in obj_dict.items():
            if not obj.is_attachPart:
                continue

            obj_holepin_set = set(obj.assem_HolePins.keys())
            used_holepin_list = list(obj_holepin_set & set(holepin_connection.keys()))

            target_obj_list = []
            target_holepin_list = []
            for used_holepin_name in used_holepin_list:
                mate_holepin_name = holepin_connection[used_holepin_name][0]
                for target_obj_name, target_obj in obj_dict.items():
                    if target_obj_name == obj_name:
                        continue
                    target_holepin_names = target_obj.assem_HolePins.keys()
                    if mate_holepin_name in target_holepin_names:
                        target_obj_list.append(target_obj_name)
                        target_holepin_list.append(mate_holepin_name)
            target_obj_set = list(set(target_obj_list))

            if len(target_obj_set) == 1:
                target_obj = obj_dict[target_obj_set[0]]
                self.attach_task(
                    obj_dict,
                    target_obj, target_holepin_list,
                    obj, used_holepin_list
                )

    def assembly_task(self, obj_dict, parent_obj, parent_holepin_names, child_obj, child_holepin_names):
        tr, quat, ref_axis, criterion, can_assembly = self.AT.get_AsmPose_by_HolePin(
            parent_obj, parent_holepin_names, child_obj, child_holepin_names
        )
        if not can_assembly:
            rospy.logwarn("Given assembly task can't be done!: \nparent_holepin_names: {}\nchild_holepin_names: {}".format(\
                parent_holepin_names, child_holepin_names))
            return TypeError
        else:
            sub_name = parent_obj.assem_name
            sub_obj = self.AT.try_attaching(
                parent_obj, parent_holepin_names,
                child_obj, child_holepin_names,
                tr, quat, criterion, sub_name, verbose=1
            )

            self.add_subObject(
                sub_obj,
                parent_obj.assem_name,
                child_obj.assem_name,
                obj_dict
            )
    
    def attach_task(self, obj_dict, parent_obj, parent_holepin_names, child_obj, child_holepin_names):
        tr, quat, ref_axis, criterion, can_attach = self.AT.get_AttachPose(
            parent_obj, parent_holepin_names, child_obj, child_holepin_names
        )
        if not can_attach:
            rospy.logwarn("Given attach task can't be done!: \nparent_holepin_names: {}\nchild_holepin_names: {}".format(\
                parent_holepin_names, child_holepin_names))
            return TypeError
        else:
            sub_name = parent_obj.assem_name
            sub_obj = self.AT.just_attaching(
                parent_obj, parent_holepin_names,
                child_obj, child_holepin_names,
                tr, quat, criterion, sub_name, verbose=1
            )

            self.add_subObject(
                sub_obj,
                parent_obj.assem_name,
                child_obj.assem_name,
                obj_dict
            )
    
    def add_subObject(self, sub_obj, parent_name, child_name, obj_dict):
        if parent_name not in obj_dict.keys() or child_name not in obj_dict.keys():
            rospy.logwarn("Given parent_name '{}' or child_name '{}' are not in obj_dict!".format(\
                parent_name, child_name))
        else:
            del obj_dict[parent_name]
            del obj_dict[child_name]
            obj_dict[sub_obj.assem_name] = sub_obj


from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
import tf.transformations as tf

class Sim_Core(Initial_Core):
    def __init__(self, is_sim=False):
        super(Sim_Core, self).__init__()
        if is_sim:
            self.client_pause = rospy.ServiceProxy("gazebo/pause_physics", Empty)
            self.client_unpause = rospy.ServiceProxy("gazebo/unpause_physics", Empty)
            self.client_spawn = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
            self.client_delete = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
            self.client_getModel = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
            self.client_reset = rospy.ServiceProxy("gazebo/reset_wordl", Empty)
            self.client_initSpawn = rospy.ServiceProxy("init_obj_spawn", Empty)
            self.wait_for_all_service()

    def wait_for_all_service(self):
        print("--- Wait for gazebo service...")
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        rospy.wait_for_service("gazebo/get_model_state")
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/pause_physics")
        rospy.wait_for_service("gazebo/unpause_physics")
        rospy.wait_for_service("gazebo/reset_world")
        rospy.wait_for_service("init_obj_spawn")
        print("--- All gazebo service servers are ready!")

    def is_in_gazebo(self, model_name, reference="world"):
        return self.client_getModel(model_name, reference).success
    
    def spawn_obj(self, obj, obj_pose, reference="world"):
        obj_name = obj.reference_part
        already = self.is_in_gazebo(obj_name)
        if not already:
            pass
        else:
            rospy.logwarn("'{}' already exists in gazebo, so delete and respawn this".format(obj_name))
            self.client_delete(obj_name)
        #self.client_pause()
        obj_xml = obj.assem_xml
        obj_ns = ""
        self.client_spawn(obj_name, obj_xml, obj_ns, obj_pose, reference)

    def get_pose_by_tr_quat(self, tr, quat):
        point = Point(tr[0], tr[1],tr[2])
        rotation = Quaternion(quat[0], quat[1], quat[2],quat[3])
        pose = Pose(point, rotation)
        return pose

    def spawn_final_obj(self):
        for final_obj_name, final_obj in self.final_dict.items():
            obj_pose = self.get_pose_by_tr_quat([0,0,1], [0,0,0,1])
            self.spawn_obj(final_obj, obj_pose)
    
    def spawn_subObject(self, parent_obj, child_obj, sub_obj):
        parent_pose = self.client_getModel(parent_obj.reference_part, "table").pose
        self.delete_obj_in_sim(child_obj.reference_part)
        self.delete_obj_in_sim(parent_obj.reference_part)
        self.spawn_obj(sub_obj, parent_pose, "table")
    
    def delete_obj_in_sim(self, obj_name):
        already = self.is_in_gazebo(obj_name)
        if not already:
            pass
        else:
            self.client_delete(obj_name)
    
    def spawn_updated_obj(self, obj, reference="table"):
        obj_tf = obj.real_pose_mat
        obj_xyz = tf.translation_from_matrix(obj_tf)
        obj_quat = tf.quaternion_from_matrix(obj_tf)
        obj_pose = self.get_pose_by_tr_quat(obj_xyz, obj_quat)
        self.spawn_obj(obj, obj_pose, reference)



import Interface_for_detector

import numpy as np
import math as m

class Core_for_Seq_Generation(Sim_Core):
    def __init__(self, is_sim=False, cowork=False, start_at_specific_point=False):
        super(Core_for_Seq_Generation, self).__init__(is_sim)
        self.cowork = cowork
        self.start_at_specific_point = start_at_specific_point
        #self.IR = Interface_for_robot.InterfaceForRobot(self.cowork)
        self.ID = Interface_for_detector.InterfaceForDetector(self.start_at_specific_point)
        self.initial_pose_update()
        self.init_compt_factors()

        initial_dict = copy.deepcopy(self.stefan_dict)
        self.assembly_seq, self.re_final_dict = \
            self.generate_seq_from_final_dict(initial_dict, self.final_dict)

    def initial_pose_update(self):
        pose_dict = self.ID.initial_obj_pose_dict
        for obj_name in self.stefan_dict.keys():
            if obj_name in pose_dict.keys():
                obj = self.stefan_dict[obj_name]
                obj_pose = pose_dict[obj.reference_part]
                obj_pose_tf = self.ID.get_tfMat_from_pose(obj_pose)
                obj.real_pose_mat = obj_pose_tf
                obj.update_real_holepin_pose()
            else:
                pass

    def init_compt_factors(self):
        self.attachPart = "part1_1"
        self.megaParent = "part6_1"
        self.megaChild = "part5_1"
        self.upScrew_list = ["C104322_1", "C104322_2", "C104322_3"]
        self.attachScrew_list = ["C122925_1", "C122925_2", "C122925_3", "C122925_4"]
        self.attach_rotate_axis = [0, 1, 0]
    
    def generate_seq_from_final_dict(self, init_dict, final_dict):
        used_part_names = init_dict.keys()
        output_part_names = final_dict.keys()

        path = []
        re_final_dict = {}
        for final_obj_name, final_obj in final_dict.items():
            temp_path = []
            used_obj_dict = {part_name: part for (part_name, part) in init_dict.items() \
                if part_name in final_obj.components.keys()}

            #while True:
            for i in range(20):
                print("\n===========================================")
                pprint(used_obj_dict.keys())
                print([final_obj.reference_part])
                print("")
                if len(used_obj_dict.keys()) == 1:
                    rospy.loginfo("Generation of seq is done!!")
                    break

                used_obj_holepin_dict = {}
                for used_obj_name, used_obj in used_obj_dict.items():
                    used_obj_holepin = used_obj.assem_HolePins
                    used_obj_holepin_dict[used_obj_name] = {}
                    for holepin_name, holepin_info in used_obj_holepin.items():
                        used_holepin_in_fin = final_obj.assem_HolePins[holepin_name]
                        if holepin_info["contact_with"] is None and \
                            used_holepin_in_fin["contact_with"] is not None:
                            target_holepin = copy.deepcopy(holepin_info)
                            target_holepin["contact_with"] = used_holepin_in_fin["contact_with"]
                            used_obj_holepin_dict[used_obj_name][holepin_name] = target_holepin

                temp_path = self.megaParent_setting(used_obj_dict, temp_path)
                
                candidates = self.get_candidates(used_obj_holepin_dict, used_obj_dict)
                print("\nprocess assembly...\n")
                if candidates:
                    child_dict = {}
                    for candi_name, candi_info in candidates.items():
                        temp_child = self.get_child(candi_info, used_obj_dict)
                        child_dict[candi_name] = temp_child
                    
                    for candi_name, child_info in child_dict.items():
                        child_list = list(set(child_info.values()))
                        for child_name in child_list:
                            parent_obj = used_obj_dict[candi_name]
                            parent_holepin_names = [holepin_name for (holepin_name, mate_obj_name) \
                                in child_info.items() if mate_obj_name == child_name]
                            child_obj = used_obj_dict[child_name]
                            child_holepin_names = [candidates[candi_name][holepin_name]["contact_with"] \
                                for holepin_name in parent_holepin_names]

                            temp_seq = {}
                            temp_seq["parent_obj_name"] = parent_obj.reference_part
                            temp_seq["parent_holepin_names"] = parent_holepin_names
                            temp_seq["child_obj_name"] = child_obj.reference_part
                            temp_seq["child_holepin_names"] = child_holepin_names
                            if "104322" in child_obj.obj_type or "122925" in child_obj.obj_type:
                                temp_seq["assembly_type"] = "screw"
                            else:
                                temp_seq["assembly_type"] = "insert"
                            temp_path.append(temp_seq)

                            self.assembly_task(
                                used_obj_dict,
                                parent_obj, parent_holepin_names,
                                child_obj, child_holepin_names
                            )
                            print("")
                            rospy.logwarn("Parent: {}".format(parent_obj.reference_part))
                            rospy.logwarn("Child: {}".format(child_obj.reference_part))

                else:
                    rospy.logwarn("Rotation task is required!")
                    temp_path = self.rotation_setting(used_obj_holepin_dict, used_obj_dict, temp_path)
                    
                    

            for seq in temp_path:
                print("\n")
                pprint(seq)
            
            print("=======")
            pprint(used_obj_dict.keys())
            path.append(temp_path)

            re_final_obj_name = used_obj_dict.keys()[0]
            re_final_dict[re_final_obj_name] = used_obj_dict[re_final_obj_name]
        return path, re_final_dict

    def rotation_setting(self, used_obj_holepin_dict, used_obj_dict, path):
        print(used_obj_dict.keys())
        print(used_obj_holepin_dict.keys())

        for obj_name, obj in used_obj_dict.items():
            if "C" in obj.obj_type:
                continue
            
            used_holepin_dict = used_obj_holepin_dict[obj_name]
            target_holepin_info = None
            for holepin_name, holepin_info in used_holepin_dict.items():
                if holepin_info["contact_with"] is not None:
                    temp_holepin_info = obj.assem_HolePins[holepin_name]
                    if temp_holepin_info["contact_with"] is None:
                        target_holepin_info = temp_holepin_info
            
            if target_holepin_info:
                holepin_type = target_holepin_info["type"]
                if holepin_type=="hole":
                    target_axis = [0, 0, -1]
                else:
                    target_axis = [0, 0, 1]
                
                hole_z = target_holepin_info["real_zAxis"]
                rot_degrees = self.AT.get_angles_between_two_vector(hole_z, target_axis)
                cross_vector = self.AT.get_cross_vector(hole_z, target_axis)
                rot_axis = tf.unit_vector(cross_vector)

                obj_cm = obj.assem_cm
                obj_cm_mat = tf.translation_matrix(obj_cm)

                init_mat = obj.real_pose_mat
                init_tr = tf.translation_from_matrix(init_mat)
                init_quat = tf.quaternion_from_matrix(init_mat)

                pre_cm_mat = tf.concatenate_matrices(init_mat, obj_cm_mat)
                pre_cm = tf.translation_from_matrix(pre_cm_mat)
                real_pre_cm = tf.translation_from_matrix(pre_cm_mat)

                theta = m.radians(rot_degrees)
                rot_quat = tf.quaternion_about_axis(theta, rot_axis)
                rot_mat = tf.quaternion_matrix(rot_quat)
                real_mat = obj.real_pose_mat
                rotated_mat = tf.concatenate_matrices(rot_mat, real_mat)
                obj.real_pose_mat = rotated_mat
                obj.update_real_holepin_pose()

                real_rot_cm = obj.real_cm

                target_tr_vec = np.array(real_pre_cm) - np.array(real_rot_cm)
                target_tr_mat = tf.translation_matrix(target_tr_vec)

                target_real_mat = tf.concatenate_matrices(rotated_mat, target_tr_mat)

                pre_tr = tf.translation_from_matrix(rotated_mat)
                pre_quat = tf.quaternion_from_matrix(rotated_mat)

                target_tr = tf.translation_from_matrix(target_real_mat)
                target_quat = tf.quaternion_from_matrix(target_real_mat)
                
                obj.real_pose_mat = target_real_mat
                obj.update_real_holepin_pose()

                real_rot_cm2 = obj.real_cm

                temp_seq = {}
                temp_seq["assembly_type"] = "rotate"
                temp_seq["pre_tr"] = init_tr
                temp_seq["pre_quat"] = init_quat
                temp_seq["rotated_tr"] = target_tr
                temp_seq["rotated_quat"] = target_quat
                temp_seq["target_obj_name"] = obj.reference_part
                path.append(temp_seq)

        return path


    
    def get_obj_by_holepin(self, obj_dict, target_holepin_name):
        target_obj = None
        for obj_name, obj in obj_dict.items():
            obj_holepin = obj.assem_HolePins
            if target_holepin_name in obj_holepin.keys():
                target_obj = copy.deepcopy(obj)
                break
        return target_obj
    
    def megaParent_setting(self, used_obj_dict, path):
        for obj_name, obj in used_obj_dict.items():
            if (obj_name == self.megaParent) and (not obj.is_setted):
                temp_seq = {}
                temp_seq["assembly_type"] = "lift"
                path.append(temp_seq)
                obj.is_setted = True
                
                obj_real_pose = obj.real_pose_mat
                obj_real_quat = tf.quaternion_from_matrix(obj_real_pose)
                obj_real_rpy = list(tf.euler_from_quaternion(obj_real_quat))
                obj_real_tr = tf.translation_from_matrix(obj_real_pose)

                rot_rpy = obj_real_rpy
                rot_rpy[1] = 0
                rot_quat = tf.quaternion_from_euler(rot_rpy[0], rot_rpy[1], rot_rpy[2])
                
                rotated_real_pose = obj.get_tf_matrix(obj_real_tr, rot_quat)
                rot_real_quat = tf.quaternion_from_matrix(rotated_real_pose)
                rot_real_rpy = list(tf.euler_from_quaternion(rot_real_quat))
                rot_real_tr = tf.translation_from_matrix(rotated_real_pose)

                obj.real_pose_mat = rotated_real_pose
                obj.update_real_holepin_pose()
        return path
            

    def get_candidates(self, used_obj_holepin_dict, used_obj_dict):
        candidates = {}
        for obj_name, obj_holepin_dict in used_obj_holepin_dict.items():
            if obj_name not in used_obj_dict.keys():
                continue
            if used_obj_dict[obj_name].is_attachPart:
                continue
            valid_holepin_dict = {}
            for holepin_name, holepin_info in obj_holepin_dict.items():
                holepin_zAxis = holepin_info["real_zAxis"]
                is_aligned_zAxis = self.is_right_zAxis_for_assembly(obj_name, holepin_name, holepin_zAxis)
                if is_aligned_zAxis:
                    if "104322" in holepin_info["contact_with"] \
                        or "122925" in holepin_info["contact_with"]:
                        used_obj = used_obj_dict[obj_name]
                        obj_whole_holepins = used_obj.assem_HolePins
                        can_screw = self.is_screw_possible(holepin_info, obj_whole_holepins)
                        if can_screw:
                            valid_holepin_dict[holepin_name] = holepin_info
                    else:
                        valid_holepin_dict[holepin_name] = holepin_info
            if valid_holepin_dict:
                candidates[obj_name] = valid_holepin_dict
        
        if len(candidates) != 0:
            return candidates
        else:
            #rospy.logwarn("Rotating task is required for force to zAxis!")
            return candidates

    def is_right_zAxis_for_assembly(self, obj_name, holepin_name, zAxis):
        if "C" in obj_name:
            return False
        else:
            if "hole" in holepin_name:
                ref_z = np.array([0, 0, -1])
            elif "pin" in holepin_name:
                ref_z = np.array([0, 0, 1])
            else:
                rospy.logwarn("Not ready for this type '{}'".format(holepin_name))
                return False
            
            degrees = self.AT.get_angles_between_two_vector(zAxis, ref_z)
            if degrees <= 10:
                is_right = True
            else:
                is_right = False
            return is_right

    def is_screw_possible(self, target_hole_info, holepin_dict):
        can_screw = False
        target_hole_end = target_hole_info["end_coordinate"]
        target_hole_end_tr = tf.translation_from_matrix(target_hole_end)
        target_hole_end_quat = tf.quaternion_from_matrix(target_hole_end)
        for holepin_name, holepin_info in holepin_dict.items():
            target_parent = target_hole_info["name"].split("-")[0]
            holepin_parent = holepin_name.split("_")[0]
            if target_parent != holepin_parent:
                if holepin_info["name"] != target_hole_info["name"] and \
                    "hole" in holepin_name and holepin_info["contact_with"] is None:
                    hole_entry = holepin_info["entry_coordinate"]
                    hole_entry_tr = tf.translation_from_matrix(hole_entry)
                    dist = np.linalg.norm(target_hole_end_tr - hole_entry_tr)
                    if dist >= 0.005:
                        hole_entry_quat = tf.quaternion_from_matrix(hole_entry)
                        hole_entry_z_axis = self.get_twisted_zAxis(hole_entry_quat)
                        target_hole_end_z_axis = self.get_twisted_zAxis(target_hole_end_quat)
                        degree = self.AT.get_angles_between_two_vector(hole_entry_z_axis, target_hole_end_z_axis)
                        if degree <= 10:
                            can_screw = True
                            break
        return can_screw
    
    def get_twisted_zAxis(self, rot):
        initial_zAxis = np.array([0, 0, 1])
        if len(rot) == 3:
            rot_mat = tf.euler_matrix(rot[0], rot[0], rot[0])
        elif len(rot) == 4:
            rot_mat = tf.quaternion_matrix(rot)
        else:
            print("Given rot component's number is wrong!")
            return TypeError
        initial_tf = tf.translation_matrix(initial_zAxis)
        twisted_tf = tf.concatenate_matrices(rot_mat, initial_tf)
        return twisted_tf[:3, 3]

    def get_child(self, parent_info, used_obj_dict):
        child_dict = {}
        for holepin_name, holepin_info in parent_info.items():
            mate_holepin_name = holepin_info["contact_with"]
            for used_obj_name, used_obj in used_obj_dict.items():
                if mate_holepin_name in used_obj.assem_HolePins.keys():
                    child_dict[holepin_name] = used_obj_name
                    break
        return child_dict
    
    def spawn_final_obj(self):
        for final_obj_name, final_obj in self.re_final_dict.items():
            obj_pose = self.get_pose_by_tr_quat([0,0,1], [0,0,0,1])
            self.spawn_obj(final_obj, obj_pose)



import sys
#import Interface_for_robot 
import Control_robot

from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties

import matplotlib.pyplot as plt

from ref_seqs import ref_seq

class Core_Main(Core_for_Seq_Generation):
    def __init__(self, is_sim=False, cowork=False, start_at_specific_point=False):
        super(Core_Main, self).__init__(is_sim, cowork, start_at_specific_point)
        self.client_get_physics = rospy.ServiceProxy("gazebo/get_physics_properties", \
            GetPhysicsProperties)
        self.client_set_physics = rospy.ServiceProxy("gazebo/set_physics_properties", \
            SetPhysicsProperties)

        self.client_initSpawn()
        self.gazebo_setting()

        robot_description = "/robot_description"
        joint_state_topic = ["joint_states:=/joint_states"]

        self.IR = Control_robot.BasicController(robot_description, joint_state_topic)

        self.total_time = 0
        self.time_list = []

        self.ref_seq = [ref_seq]
        

    def gazebo_setting(self):
        temp_phisics = self.client_get_physics()
        temp_phisics.gravity.z = 0.0
        self.client_set_physics(temp_phisics.time_step,
                                temp_phisics.max_update_rate,
                                temp_phisics.gravity,
                                temp_phisics.ode_config)
        self.client_unpause()

    def main(self):
        seq = copy.deepcopy(self.assembly_seq)
        #seq = copy.deepcopy(self.ref_seq)
        obj_dict = copy.deepcopy(self.stefan_dict)


        for assembly_unit in seq:
            print("\n##############################################")
            print("############ Start Assembly Task #############")
            print("##############################################\n")


            for idx in range(len(assembly_unit)):
                print("\n--- {} / {} ---".format(idx+1, len(assembly_unit)))
                current_task = assembly_unit[idx]
                pprint(self.time_list)

                while True:
                    pprint(current_task)
                    obj_dict, success = self.process_current_task_sim(obj_dict, current_task)
                    if success:
                        break
                    else:
                        keyboard_input = None
                        try:
                            keyboard_input = input("\nInput your choice: \n\t1: Exit \n\t2: Re try \n\t3: Next task\n\n\t")
                        except SyntaxError:
                            rospy.logwarn("Don't input empty!")
                        except NameError:
                            rospy.logwarn("Don't input empty!")
                        if keyboard_input == 1:
                            sys.exit()
                        elif keyboard_input == 2:
                            continue
                        elif keyboard_input == 3:
                            break
                        else:
                            sys.exit()
            

    def process_current_task_sim(self, obj_dict, task):
        success = False
        temp_dict = copy.deepcopy(obj_dict)

        assembly_type = assembly_type = task["assembly_type"]
        print(assembly_type)

        if assembly_type=="insert" or assembly_type=="screw" or assembly_type=="upScrew" or assembly_type=="attachScrew":
            parent_obj_name = task["parent_obj_name"]
            parent_obj = obj_dict[parent_obj_name]
            parent_holepin_names = task["parent_holepin_names"]
            child_obj_name = task["child_obj_name"]
            child_obj = obj_dict[child_obj_name]
            child_holepin_names = task["child_holepin_names"]
            
            tr, quat, ref_axis, criterion, can_assembly = \
                self.AT.get_AsmPose_by_HolePin(parent_obj, parent_holepin_names, child_obj, child_holepin_names)
            if can_assembly:

                self.assembly_task(
                    temp_dict,
                    parent_obj, parent_holepin_names,
                    child_obj, child_holepin_names
                )
                success = True
                sub_obj = temp_dict[parent_obj_name]
                #self.spawn_subObject(parent_obj, child_obj, sub_obj)

               

                target_arm_name, target_arm, target_obj_pose = \
                    self.get_target_arm(child_obj_name)
                rospy.logwarn(target_arm_name)
                self.time_list.append(self.IR.approach_to_obj(target_arm, target_obj_pose, 0.2))
                self.time_list.append(self.IR.approach_to_obj(target_arm, target_obj_pose, 0.1))
                #self.IR.grasp_control(target_arm_name, True)
                self.time_list.append(self.IR.approach_to_obj(target_arm, target_obj_pose, 0.2))
                self.delete_obj_in_sim(child_obj_name)


                asm_pose = self.get_pose_by_tr_quat(tr, quat)
                real_asm_pose = self.get_asm_pose_in_gazebo(parent_obj_name, asm_pose, 0.2)
                self.time_list.append(self.IR.movel_just_tr(target_arm, real_asm_pose))
                real_asm_pose = self.get_asm_pose_in_gazebo(parent_obj_name, asm_pose, 0.1)
                self.time_list.append(self.IR.movel_just_tr(target_arm, real_asm_pose))
                #self.IR.grasp_control(target_arm_name, False)

                self.spawn_subObject(parent_obj, child_obj, sub_obj)

                real_asm_pose = self.get_asm_pose_in_gazebo(parent_obj_name, asm_pose, 0.2)
                self.time_list.append(self.IR.movel_just_tr(target_arm, real_asm_pose))

                
                self.time_list.append(self.IR.move_by_target(target_arm, target_arm_name, "ready"))

        

        elif assembly_type=="lift":
            ###################################################
            ############# Interface ###########################
            ###################################################

            obj = temp_dict[self.megaParent]
            obj_real_pose = obj.real_pose_mat
            obj_real_quat = tf.quaternion_from_matrix(obj_real_pose)
            obj_real_rpy = list(tf.euler_from_quaternion(obj_real_quat))
            obj_real_tr = tf.translation_from_matrix(obj_real_pose)

            rot_rpy = obj_real_rpy
            rot_rpy[1] = 0
            rot_quat = tf.quaternion_from_euler(rot_rpy[0], rot_rpy[1], rot_rpy[2])
            
            rotated_real_pose = obj.get_tf_matrix(obj_real_tr, rot_quat)
            rot_real_quat = tf.quaternion_from_matrix(rotated_real_pose)
            rot_real_rpy = list(tf.euler_from_quaternion(rot_real_quat))
            rot_real_tr = tf.translation_from_matrix(rotated_real_pose)

            obj.real_pose_mat = rotated_real_pose
            obj.update_real_holepin_pose()
            success = True

            self.spawn_updated_obj(obj)

        elif assembly_type=="rotate":
            ###################################################
            ############# Interface ###########################
            ###################################################

            if "unvalid" not in task.keys():
                megaParent_obj = temp_dict[task["target_obj_name"]]

                target_real_tr = task["rotated_tr"]
                target_real_quat = task["rotated_quat"]
                target_real_mat = megaParent_obj.get_tf_matrix(target_real_tr, target_real_quat)
                megaParent_obj.real_pose_mat = target_real_mat
                megaParent_obj.update_real_holepin_pose()

                #self.IR.update_just_parent_tf_stl(megaParent_obj)
                #self.IR.update_component_stl(megaParent_obj)
                

                self.spawn_updated_obj(megaParent_obj)
            
            target_obj_name = task["target_obj_name"]
            target_arm_name, target_arm, target_obj_pose = \
                    self.get_target_arm(target_obj_name)
            self.time_list.append(self.IR.approach_to_obj(target_arm, target_obj_pose, 0.2))
            self.time_list.append(self.IR.approach_to_obj(target_arm, target_obj_pose, 0.1))
            #self.IR.grasp_control(target_arm_name, True)
            self.time_list.append(self.IR.approach_to_obj(target_arm, target_obj_pose, 0.2))
            self.time_list.append(10)
            self.time_list.append(self.IR.approach_to_obj(target_arm, target_obj_pose, 0.1))
            self.time_list.append(self.IR.approach_to_obj(target_arm, target_obj_pose, 0.2))
            self.time_list.append(self.IR.move_by_target(target_arm, target_arm_name, "ready"))
            success = True
        return temp_dict, success

    def get_target_arm(self, obj_name):
        arm1_base = self.IR.arm1_base
        arm2_base = self.IR.arm2_base

        pose_to_arm1 = self.client_getModel(obj_name, arm1_base)
        pose_to_arm2 = self.client_getModel(obj_name, arm2_base)
        
        dist_to_arm1 = self.dist_from_pose(pose_to_arm1.pose)
        dist_to_arm2 = self.dist_from_pose(pose_to_arm2.pose)
        rospy.logwarn(dist_to_arm1)
        rospy.logwarn(dist_to_arm2)
         
        if dist_to_arm1 >= dist_to_arm2:
            return "arm2", self.IR.arm2, self.client_getModel(obj_name, "table").pose
        else:
            return "arm1", self.IR.arm1, self.client_getModel(obj_name, "table").pose

    def dist_from_pose(self, pose):
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z 
        vec = np.array([x, y, z])
        return np.linalg.norm(vec)

    def get_asm_pose_in_gazebo(self, obj_name, asm_pose, z_offset):
        obj_pose = self.client_getModel(obj_name, "table").pose
        obj_tf_mat = self.get_tf_mat_from_pose(obj_pose)
        asm_tf_mat = self.get_tf_mat_from_pose(asm_pose)
        target_tf_mat = tf.concatenate_matrices(obj_tf_mat, asm_tf_mat)
        target_tr = list(tf.translation_from_matrix(target_tf_mat))
        target_tr[2] += z_offset
        target_quat = tf.quaternion_from_matrix(target_tf_mat)
        return self.get_pose_by_tr_quat(target_tr, target_quat)

    def get_tf_mat_from_pose(self, pose):
        tr = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        return self.AT.get_tf_matrix(tr, quat)

    def shutdown_hook(self):
        step = np.arange(1, len(self.time_list)+1)
        print("Falure counts: {}".format(len(self.time_list) - np.count_nonzero(self.time_list)))
        print("Total times: {}".format(np.sum(self.time_list)))
        print("Total steps: {}".format(len(self.time_list)))
        plt.bar(step, self.time_list)
        plt.show()


        
                

#core = Initial_Core()

#core = Sim_Core(is_sim=True)
#core.spawn_final_obj()

#core = Core_for_Seq_Generation(cowork=True)


core = Core_Main(is_sim=True, cowork=False)
rospy.on_shutdown(core.shutdown_hook)
#pprint(core.re_final_dict)

core.main()