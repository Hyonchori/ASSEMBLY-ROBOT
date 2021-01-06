#!/usr/bin/env python
import rospy

import os
import copy
import numpy as np
from pprint import pprint

from pddl_processor import PDDL2Seq
from assembly_tester import AssemblyTester
import object_description

PDDL_DIR = "/home/cai/share_for_compt/for_kitech/asm_seq/origin_combined_v1"
#PDDL_DIR = "/home/cai/share_for_compt/for_kitech/asm_seq/origin_sep_v1"
TARGET_OBJECT = "stefan"

__metaclass__ = type
class InitialCore():
    def __init__(self):
        self.P2S = PDDL2Seq()
        self.pddl_dir = PDDL_DIR
        self.total_ref_pddl = self.get_assembly_seq(self.pddl_dir)
        pprint(self.total_ref_pddl)
        
        self.AT = AssemblyTester()
        self.target_object = TARGET_OBJECT
        self.stefan_dict = self.get_obj_dict(self.target_object)
        self.final_product, self.can_assembly = self.check_can_assembly(self.total_ref_pddl, self.stefan_dict)

    def get_assembly_seq(self, pddl_dir):
        pddl_files = os.listdir(pddl_dir)
        pddl_files.sort()
        pddl_path = [os.path.join(pddl_dir, file) for file in pddl_files]
        total_ref_pddl = [self.P2S.get_target_seq(pddl_path) for pddl_path in pddl_path]
        return total_ref_pddl
    
    def get_obj_dict(self, target_object):
        stefan_param_names = [param for param in rospy.get_param_names() 
                    if ("assembly_core" in param) and (target_object in param)]
        obj_dict = {}
        for param in stefan_param_names:
            param_description = param.split("/")[2]
            param_split = param_description.split("_")
            obj_type = param_split[1]
            obj_totalNum = int(param_split[2])
            for idx in range(1, obj_totalNum+1):
                obj = object_description.Assembly_Object(param, idx)
                obj_dict[obj.obj_name] = obj
        return obj_dict
    
    def check_can_assembly(self, total_pddl, obj_dict):
        target_pddl = copy.deepcopy(total_pddl)
        target_dict = copy.deepcopy(obj_dict)

        print("\n-------------------------------------")
        print("Check assembly possiblity start!")
        for pddl_idx in range(len(target_pddl)):
            pddl = total_pddl[pddl_idx]
            print("\n--- {} PAGE".format(pddl_idx+1))
            for seq_idx in range(len(pddl)):
                seq = pddl[seq_idx]
                print("\n - {} NUM".format(seq_idx+1))
                target_dict, success = self.process_current_seq(seq, target_dict)
                if not success:
                    return target_dict, False
        return target_dict, True
    
    def process_current_seq(self, seq, obj_dict):
        pprint(seq)
        target_obj_dict = copy.deepcopy(obj_dict)
        process_success = False

        assembly_type = seq["assembly_type"]
        if assembly_type == "insert" or assembly_type == "screw":
            parent_obj_name = seq["parent_obj_name"]
            child_obj_name = seq["child_obj_name"]

            if self.AT.check_parent_child_in_dict(seq, target_obj_dict):
                rospy.logwarn("Given name '{}' or '{}' are not in obj_dict's key: {}".format(\
                    parent_obj_name, child_obj_name, obj_dict.keys()))
            else:
                parent_obj, parent_const_names, child_obj, child_const_names, const_ready = \
                    self.AT.get_data_for_InsertScrew(seq, target_obj_dict)
                if not const_ready:
                    rospy.logwarn("Parent or child object's consts are not ready!")
                else:
                    tr, quat, ref_axis, criterion, can_assembly = self.AT.get_AsmPose_by_HolePin(
                        parent_obj, parent_const_names, child_obj, child_const_names
                    )
                    if not can_assembly:
                        rospy.logwarn("Assembly pose can't be calculated!")
                    else:
                        sub_name = seq["subassembly_name"]

                        sub_obj = self.AT.try_attaching(
                            parent_obj,
                            parent_const_names,
                            child_obj,
                            child_const_names,
                            tr, quat, criterion, sub_name,
                            verbose=1
                            )

                        self.AT.add_subObject(
                            sub_obj, 
                            parent_obj_name, 
                            child_obj_name,
                            target_obj_dict
                            )

                        print("Parent name : {}".format(parent_obj.referencePart))
                        print("Child name : {}".format(child_obj.referencePart))
                        print("tr: {}".format(tr))
                        print("quat: {}".format(quat))
                        print("ref_Axis: {}".format(ref_axis))
                        process_success = True
        else:
            rospy.logwarn("Not ready for given task '{}'!".format(assembly_type))

        return target_obj_dict, process_success




from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
import tf.transformations as tf
import utils

class SimCore(InitialCore):
    def __init__(self, is_sim=False):
        super(SimCore, self).__init__()
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
        obj_name = obj.referencePart
        already = self.is_in_gazebo(obj_name)
        if not already:
            pass
        else:
            rospy.logwarn("'{}' already exists in gazebo, \
                so delete and respawn this".format(obj_name))
            self.client_delete(obj_name)
        self.client_pause()
        obj_xml = obj.assemXML
        obj_ns = ""
        self.client_spawn(obj_name, obj_xml, obj_ns, obj_pose, reference)

    def check_initial_hole_axis(self):
        init_obj_dict = copy.deepcopy(self.stefan_dict)
        for obj_name, obj in init_obj_dict.items():
            if self.is_in_gazebo(obj_name):
                obj_pose = self.client_getModel(obj_name, "world")
                #print(obj_pose)
                obj_tf = self.get_tfMat_from_pose(obj_pose)
                obj.real_pose_mat = obj_tf
                obj.update_real_const_pose()
                print("\n---")
                print(obj_name)
                print("")
                for const_name, const in obj.assemConsts.items():
                    print("")
                    print(const_name)
                    print(const["real_zAxis"])


    
    def get_pose_by_tr_quat(self, tr, quat):
        point = Point(tr[0], tr[1],tr[2])
        rotation = Quaternion(quat[0], quat[1], quat[2],quat[3])
        pose = Pose(point, rotation)
        return pose
    
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

    def spawn_subObject(self, parent_obj, child_obj, sub_obj):
        parent_pose = self.client_getModel(parent_obj.referencePart, "world").pose
        self.delete_obj_in_sim(parent_obj.referencePart)
        self.delete_obj_in_sim(child_obj.referencePart)
        self.spawn_obj(sub_obj, parent_pose, "world")
    
    def delete_obj_in_sim(self, obj_name):
        already = self.is_in_gazebo(obj_name)
        if not already:
            pass
        else:
            self.client_delete(obj_name)
    
    def get_ref_axis_in_env(self, obj_tf, ref_axis):
        quat = tf.quaternion_from_matrix(obj_tf)
        quat_mat = tf.quaternion_matrix(quat)
        ref_axis_mat = tf.translation_matrix(ref_axis)
        ref_axis_in_env_mat = tf.concatenate_matrices(quat_mat, ref_axis_mat)
        ref_axis_in_env = tf.translation_from_matrix(ref_axis_in_env_mat)
        return ref_axis_in_env

    def check_can_assembly_sim(self, total_pddl, obj_dict):
        target_pddl = copy.deepcopy(total_pddl)
        target_dict = copy.deepcopy(obj_dict)

        print("\n-------------------------------------")
        print("Check assembly possiblity start!")
        for pddl_idx in range(len(target_pddl)):
            pddl = total_pddl[pddl_idx]
            print("\n--- {} / {} PAGE".format(pddl_idx+1, len(target_pddl)))
            for seq_idx in range(len(pddl)):
                seq = pddl[seq_idx]
                print("\n - {} / {} NUM".format(seq_idx+1, len(pddl)))
                target_dict, success = self.process_current_seq_sim(seq, target_dict)
                if not success:
                    return target_dict, False
        return target_dict, True

    def process_current_seq_sim(self, seq, obj_dict):
        pprint(seq)
        target_obj_dict = copy.deepcopy(obj_dict)
        process_success = False

        assembly_type = seq["assembly_type"]
        if assembly_type == "insert" or assembly_type == "screw":
            parent_obj_name = seq["parent_obj_name"]
            child_obj_name = seq["child_obj_name"]

            if self.AT.check_parent_child_in_dict(seq, target_obj_dict):
                rospy.logwarn("Given name '{}' or '{}' are not in obj_dict's key: {}".format(\
                    parent_obj_name, child_obj_name, obj_dict.keys()))
            else:
                parent_obj, parent_const_names, child_obj, child_const_names, const_ready = \
                    self.AT.get_data_for_InsertScrew(seq, target_obj_dict)
                if not const_ready:
                    rospy.logwarn("Parent or child object's consts are not ready!")
                else:
                    tr, quat, ref_axis, criterion, can_assembly = self.AT.get_AsmPose_by_HolePin(
                        parent_obj, parent_const_names, child_obj, child_const_names
                    )
                    if not can_assembly:
                        rospy.logwarn("Assembly pose can't be calculated!")
                    else:
                        sub_name = seq["subassembly_name"]

                        sub_obj = self.AT.try_attaching(
                            parent_obj,
                            parent_const_names,
                            child_obj,
                            child_const_names,
                            tr, quat, criterion, sub_name,
                            verbose=1
                            )

                        self.AT.add_subObject(
                            sub_obj, 
                            parent_obj_name, 
                            child_obj_name,
                            target_obj_dict
                            )

                        obj_tf = parent_obj.real_pose_mat
                        ref_axis_in_env = self.get_ref_axis_in_env(obj_tf, ref_axis)

                        print("Parent name : {}".format(parent_obj.referencePart))
                        print("Child name : {}".format(child_obj.referencePart))
                        print("tr: {}".format(tr))
                        print("quat: {}".format(quat))
                        print("ref_Axis_in_env: {}".format(ref_axis_in_env))
                        self.spawn_subObject(parent_obj, child_obj, sub_obj)
                        process_success = True
        else:
            rospy.logwarn("Not ready for given task '{}'!".format(assembly_type))

        return target_obj_dict, process_success




from Interface_for_detector import InterfaceForDetector
from interface_for_robot import InterfaceForRobot

import math as m
import rospkg
import os
import csv

r = rospkg.RosPack()
r_path = r.get_path("object_description")
TEMP_DIR = os.path.join(r_path, "temp_pose")

class CoreForComptetion(SimCore):
    def __init__(self, is_sim=False, cowork=False, start_at_specific_point=True):
        super(CoreForComptetion, self).__init__(is_sim)
        self.cowork = cowork
        self.ID = InterfaceForDetector()
        self.IR = InterfaceForRobot(self.cowork)
        self.initial_pose_update()
        if self.can_assembly:
            init_dict = copy.deepcopy(self.stefan_dict)
            fin_dict = copy.deepcopy(self.final_product)
            test_fin_dict = self.test(init_dict, fin_dict)
            self.edited_pddl = self.generate_seq_from_finProduct(init_dict, test_fin_dict)

        self.real_obj_dict = copy.deepcopy(self.stefan_dict)
        self.save_dict = copy.deepcopy(self.stefan_dict)

    def initial_pose_update(self):
        pose_dict = self.ID.initial_obj_pose_dict
        for obj_name in self.stefan_dict.keys():
            if obj_name in pose_dict.keys():
                obj = self.stefan_dict[obj_name]
                obj_pose = pose_dict[obj.referencePart]
                obj_pose_tf = self.ID.get_tfMat_from_pose(obj_pose)
                obj.real_pose_mat = obj_pose_tf
                obj.update_real_const_pose()
            else:
                pass
    



    ''''''''''''''''''''''''''''''''''''''''''''''''''
    ''''''''' Attach Task!!! '''''''''''''''
    '''''''''''''''''''''''''''''''''''''''''''''''''



    def test(self, init_dict, fin_dict):
        initial_dict = copy.deepcopy(init_dict)

        fin_obj = copy.deepcopy(fin_dict["stefan12_step8_1"])
        
        mate_dict = {}
        for const_name, const_info in fin_obj.assemConsts.items():
            mate_dict[const_name] = const_info["contactWith"]
        #pprint(mate_dict)
        
        for obj_name, obj in initial_dict.items():
            if "C" not in obj_name:
                continue
            else:
                pass
            print("\n############################")
            print(obj_name)

            for const_name, const in obj.assemConsts.items():
                if const_name not in mate_dict.keys():
                    continue
                else:
                    pass

                if const["feature"] != "screw":
                    continue

                mate_const_name = mate_dict[const_name]
                if mate_const_name is None:
                    continue
                else:
                    pass
                rospy.logwarn(const_name)
                mate_obj_name = mate_const_name.split("-")[0]
                mate_obj = initial_dict[mate_obj_name]

                child_obj = obj
                child_const_names = [const_name]
                parent_obj = mate_obj
                parent_const_names = [mate_const_name]
                self.assembly_task(initial_dict, parent_obj, parent_const_names, child_obj, child_const_names)
                break
        pprint(initial_dict.keys())
        print("")
        
        for obj_name, obj in initial_dict.items():
            if "C" not in obj_name:
                continue
            else:
                pass
            print("\n############################")
            print(obj_name)

            for const_name, const in obj.assemConsts.items():
                if const_name not in mate_dict.keys():
                    continue
                else:
                    pass

                mate_const_name = mate_dict[const_name]
                if mate_const_name is None:
                    continue
                else:
                    pass

                mate_obj_name = mate_const_name.split("-")[0]
                if mate_obj_name not in initial_dict.keys():
                    continue
                else:
                    pass

                if const["contactWith"] is not None:
                    continue
                else:
                    pass

                rospy.logwarn("{} : {}".format(const_name, mate_obj_name))
                mate_obj = initial_dict[mate_obj_name]

                child_obj = obj
                child_const_names = [const_name]
                parent_obj = mate_obj
                parent_const_names = [mate_const_name]
                self.assembly_task(initial_dict, parent_obj, parent_const_names, child_obj, child_const_names)
                break
        pprint(initial_dict.keys())
        print("")

        part_dict = {}
        for obj_name, obj in initial_dict.items():
            if obj_name not in initial_dict.keys():
                continue
            mate_obj_list = {part_name: [] for part_name in initial_dict.keys() if part_name != obj_name}
            for const_name, const_info in obj.assemConsts.items():
                if const["contactWith"] is not None:
                    continue
                else:
                    pass

                mate_const_name = mate_dict[const_name]
                if mate_const_name is None:
                    continue
                else:
                    pass

                
                for target_obj_name, target_obj in initial_dict.items():
                    if obj_name == target_obj_name:
                        continue
                    else:
                        pass

                    target_const_names = target_obj.assemConsts.keys()
                    if mate_const_name in target_const_names:
                        mate_obj_list[target_obj_name].append(const_name)
                        #mate_obj_list.append(target_obj_name)

            part_dict[obj_name] = mate_obj_list
            print("\n==============================================")
            rospy.logerr(obj_name)
            pprint(mate_obj_list)


            for mate_obj_name, used_const_names in mate_obj_list.items():
                if used_const_names:
                    pass
                else:
                    continue

                if mate_obj_name not in initial_dict.keys():
                    continue
                else:
                    pass
                print("")
                rospy.logwarn(mate_obj_name)
                mate_obj = initial_dict[mate_obj_name]
                mate_const_names = [mate_dict[const_name] for const_name in used_const_names]
                print(used_const_names)
                print(mate_const_names)
                
                parent_obj = initial_dict[obj_name]
                parent_const_names = used_const_names
                child_obj = mate_obj
                child_const_names = mate_const_names
                self.assembly_task(initial_dict, parent_obj, parent_const_names, child_obj, child_const_names)
 
        print("\n\n\n\n\n")
        print("#######################################")
        pprint(initial_dict.keys())
        print("")
        fin_obj_test = initial_dict.values()[0]
        #pprint(fin_obj_test.assemConsts.keys())
        #pprint(fin_obj_test.components.keys())
        const_name_list = fin_obj_test.assemConsts.keys()
        const_name_list.sort()

        for const_name in const_name_list:
            target_const = fin_obj_test.assemConsts[const_name]
            print("\n{} : {}".format(const_name, target_const["contactWith"]))

        return initial_dict




    
    def assembly_task(self, obj_dict, parent_obj, parent_const_names, child_obj, child_const_names):
        tr, quat, ref_axis, criterion, can_assembly = self.AT.get_AsmPose_by_HolePin(
            parent_obj, parent_const_names, child_obj, child_const_names
        )
        sub_name = parent_obj.assem_name

        sub_obj = self.AT.try_attaching(
            parent_obj,
            parent_const_names,
            child_obj,
            child_const_names,
            tr, quat, criterion, sub_name,
            verbose=1
            )

        self.AT.add_subObject(
            sub_obj, 
            parent_obj.assem_name, 
            child_obj.assem_name,
            obj_dict
            )

        











    def generate_seq_from_finProduct(self, init_dict, fin_dict):
        '''init_part_names = set(init_dict.keys())
        fin_part_names = set(fin_dict.keys())
        used_part_names = list(init_part_names - fin_part_names)
        output_part_names = list(fin_part_names - init_part_names)'''
        used_part_names = init_dict.keys()
        output_part_names = fin_dict.keys()
        output_ref_names = [fin_dict[part_name].referencePart for part_name in output_part_names]

        pprint(used_part_names)
        pprint(output_part_names)
        print(output_ref_names)
        
        sub_obj_list = [fin_dict[part_name] for part_name in output_part_names]

        path = []
        for sub_obj in sub_obj_list:
            temp_path = []
            used_obj_dict = {init_dict[part_name].referencePart: init_dict[part_name] for part_name in used_part_names \
                if part_name in sub_obj.components.keys()}
            pprint(used_obj_dict.keys())

            
            while True:
                print("\n===========================================")
                pprint(used_obj_dict.keys())
                print(sub_obj.referencePart)
                print("")
                if used_obj_dict.keys() == [sub_obj.referencePart]:
                    rospy.logwarn("???")
                    break

                used_obj_const_dict = {}
                for used_obj in used_obj_dict.values():
                    used_obj_const = used_obj.assemConsts
                    used_obj_const_dict[used_obj.referencePart] = {}
                    for const_name, const in used_obj_const.items():
                        used_const_in_sub = sub_obj.assemConsts[const_name]
                        if const["contactWith"] is None and used_const_in_sub["contactWith"] is not None:
                            target_const = copy.deepcopy(const)
                            target_const["contactWith"] = used_const_in_sub["contactWith"]
                            used_obj_const_dict[used_obj.referencePart][const_name] = target_const
                
                candidates = self.get_candidates(used_obj_const_dict, used_obj_dict)
                pprint(candidates.keys())

                if candidates:
                    child_names = []
                    child_dict = {}
                    for candi_name, candi_info in candidates.items():
                        temp_child = self.get_child(candi_info, used_obj_dict)
                        child_dict[candi_name] = temp_child

                    for candi_name, candi_info in child_dict.items():
                        child_list = list(set(candi_info.values()))
                        for child_name in child_list:
                            parent_obj = used_obj_dict[candi_name]
                            parent_const_names = [const_name for (const_name, const_mate_obj) \
                                in candi_info.items() if const_mate_obj == child_name]
                            child_obj = used_obj_dict[child_name]
                            child_const_names = [candidates[candi_name][const_name]["contactWith"] for const_name \
                                in parent_const_names]
                            
                            temp_seq = {}
                            temp_seq["parent_obj_name"] = parent_obj.referencePart
                            temp_seq["parent_const_names"] = parent_const_names
                            temp_seq["child_obj_name"] = child_obj.referencePart
                            temp_seq["child_const_names"] = child_const_names
                            temp_seq["subassembly_name"] = parent_obj.referencePart
                            temp_seq["assembly_type"] = "insert" 
                            if "C122925" in child_obj.obj_type or "C104322" in child_obj.obj_type:
                                temp_seq["assembly_type"] = "screw"
                            else:
                                temp_seq["assembly_type"] = "insert"

                            temp_path.append(temp_seq)
                            used_obj_dict, _ = self.process_current_seq(temp_seq, used_obj_dict)
                else:

                    heaviest_obj = self.get_heaviest_obj(used_obj_dict)
                    const_names_in_obj = used_obj_const_dict[heaviest_obj.assem_name]
                    pprint(const_names_in_obj.keys())
                    
                    temp_seq = {}
                    temp_seq["target_obj_name"] = heaviest_obj.referencePart
                    temp_seq["assembly_type"] = "rotate"
                    temp_path.append(temp_seq)
                    break
            path.append(temp_path)
        return path

    def get_candidates(self, used_obj_const_dict, used_obj_dict):
        candidates = {}
        for obj_name, obj_const_dict in used_obj_const_dict.items():
            valid_const_dict = {}
            for const_name, const_info in obj_const_dict.items():
                const_zAxis = const_info["real_zAxis"]
                is_right_zAxis = self.is_right_zAxis(obj_name, const_name, const_zAxis)
                if is_right_zAxis:
                    if "C104322" in const_info["contactWith"]:
                        used_obj = used_obj_dict[obj_name]
                        temp_const_dict = used_obj.assemConsts
                        can_screw = self.can_screw(const_info, temp_const_dict)
                        print(obj_name, const_name, can_screw)
                        if can_screw:
                            valid_const_dict[const_name] = const_info
                    else:
                        valid_const_dict[const_name] = const_info
            if valid_const_dict:
                candidates[obj_name] = valid_const_dict

        if len(candidates) != 0:
            return candidates
        else:
            rospy.logwarn("Rotating task is required for force to zAxis!")
            return candidates
    
    def is_right_zAxis(self, ref_name, const_name, z_axis):
        if "C" in ref_name:
            return False
        else:
            if "hole" in const_name:
                ref_z = np.array([0, 0, -1])
            elif "pin" in const_name:
                ref_z = np.array([0, 0, 1])
            else:
                rospy.logwarn("Not ready for this type '{}'".format(const_name))
                return False

            degree = self.axis_diff(z_axis, ref_z)
            if degree <= 10:
                is_right = True
            else:
                is_right = False
            return is_right
        
    def axis_diff(self, axis1, axis2):
        inner_product = np.dot(axis1, axis2)
        theta = m.acos(inner_product)
        degree = m.degrees(theta)
        return degree

    def can_screw(self, target_hole_info, const_dict):
        print("\n")
        can_screw = False
        hole_end = target_hole_info["endCoordinate"]
        hole_end_tr = tf.translation_from_matrix(hole_end)
        hole_end_quat = tf.quaternion_from_matrix(hole_end)
        print(hole_end_tr)
        for const_name, const_info in const_dict.items():
            target_init_parent = target_hole_info["name"].split("-")[0]
            const_init_parent = const_name.split("_")[0]
            if target_init_parent != const_init_parent:
                if const_info["name"] != target_hole_info["name"] and "hole" in const_name and const_info["contactWith"] is None:
                    hole_entry = const_info["entryCoordinate"]
                    hole_entry_tr = tf.translation_from_matrix(hole_entry)
                    dist = np.linalg.norm(hole_end_tr - hole_entry_tr)
                    print(" {}    {}     {}".format(const_name, hole_entry_tr, dist))
                    if dist <= 0.005:
                        hole_entry_quat = tf.quaternion_from_matrix(hole_entry)
                        hole_entry_z_axis = utils.get_transformed_zAxis(hole_entry_quat)
                        hole_end_z_axis = utils.get_transformed_zAxis(hole_end_quat)
                        degree = self.axis_diff(hole_entry_z_axis, hole_end_z_axis)
                        if degree <= 10:
                            can_screw = True
                            break
        return can_screw

    def get_child(self, parent_info, used_obj_dict):
        child_dict = {}
        for const_name, const_info in parent_info.items():
            mate_const_name = const_info["contactWith"]
            for used_obj_name, used_obj in used_obj_dict.items():
                if mate_const_name in used_obj.assemConsts.keys():
                    child_dict[const_name] = used_obj_name
                    break
        return child_dict

    def get_heaviest_obj(self, used_obj_dict):
        obj_name_list = used_obj_dict.keys()
        obj_mass_list = [used_obj_dict[obj_name].assem_mass for obj_name in obj_name_list]
        max_mass_idx = np.argmax(obj_mass_list)
        heaviest_obj = used_obj_dict[obj_name_list[max_mass_idx]]
        return heaviest_obj

    def get_remain_const_dict(self, obj):
        const_dict = {}
        for const_name, const_info in obj.assemConsts.items():
            if const_info["contactWith"] is None:
                const_dict[const_name] = const_info
        return const_dict
    
    def show_edited_pddl(self):
        for seq_idx in range(len(self.edited_pddl)):
            seq = self.edited_pddl[seq_idx]
            print("\n - {} NUM".format(seq_idx+1))
            for task in seq:
                print("")
                pprint(task)


    def check_can_assembly_cowork(self):
        target_pddl = copy.deepcopy(self.edited_pddl)
        target_dict = copy.deepcopy(self.stefan_dict)
        self.publish_init_tf(target_dict)

        print("\n-------------------------------------")
        print("Check assembly possiblity start!")
        for pddl_idx in range(len(target_pddl)):
            pddl = target_pddl[pddl_idx]
            print("\n--- {} / {} PAGE".format(pddl_idx+1, len(target_pddl)))
            for seq_idx in range(len(pddl)):

                self.publish_all_tf(target_dict)
                self.save_dict = target_dict

                seq = pddl[seq_idx]
                print("\n - {} / {} NUM".format(seq_idx+1, len(pddl)))
                target_dict, success = self.process_current_seq_compt(seq, target_dict)
                if not success:
                    return target_dict, False
        return target_dict, True

    def process_current_seq_compt(self, seq, obj_dict):
        pprint(seq)
        target_obj_dict = copy.deepcopy(obj_dict)
        process_success = False

        assembly_type = seq["assembly_type"]
        if assembly_type == "insert" or assembly_type == "screw":
            parent_obj_name = seq["parent_obj_name"]
            child_obj_name = seq["child_obj_name"]

            if self.AT.check_parent_child_in_dict(seq, target_obj_dict):
                rospy.logwarn("Given name '{}' or '{}' are not in obj_dict's key: {}".format(\
                    parent_obj_name, child_obj_name, obj_dict.keys()))
            else:
                parent_obj, parent_const_names, child_obj, child_const_names, const_ready = \
                    self.AT.get_data_for_InsertScrew(seq, target_obj_dict)
                if not const_ready:
                    rospy.logwarn("Parent or child object's consts are not ready!")
                else:
                    resp_from_robot = self.send_msg_for_HoleCheck(
                        assembly_type, parent_obj, parent_const_names, child_obj, child_const_names
                    )
                    if resp_from_robot.result:
                        self.update_holepin_pose(
                            resp_from_robot,
                            parent_obj, parent_const_names, child_obj, child_const_names
                        )

                        tr, quat, ref_axis, criterion, can_assembly = self.AT.get_AsmPose_by_HolePin(
                            parent_obj, parent_const_names, child_obj, child_const_names
                        )
                        if not can_assembly:
                            rospy.logwarn("Assembly pose can't be calculated!")
                        else:
                            self.publish_obj_tf(parent_obj, "table")
                            resp_from_robot = self.send_msg_for_RobotControl(
                                assembly_type, parent_obj, parent_const_names, child_obj, child_const_names
                            )
                            
                            sub_name = seq["subassembly_name"]

                            sub_obj = self.AT.try_attaching(
                                parent_obj,
                                parent_const_names,
                                child_obj,
                                child_const_names,
                                tr, quat, criterion, sub_name,
                                verbose=1
                                )

                            self.AT.add_subObject(
                                sub_obj, 
                                parent_obj_name, 
                                child_obj_name,
                                target_obj_dict
                                )

                            obj_tf = parent_obj.real_pose_mat
                            ref_axis_in_env = self.get_ref_axis_in_env(obj_tf, ref_axis)

                            print("Parent name : {}".format(parent_obj.referencePart))
                            print("Child name : {}".format(child_obj.referencePart))
                            print("tr: {}".format(tr))
                            print("quat: {}".format(quat))
                            print("ref_Axis_in_env: {}".format(ref_axis_in_env))

                            process_success = True
        else:
            rospy.logwarn("Not ready for given task '{}'!".format(assembly_type))

        return target_obj_dict, process_success

    def send_msg_for_HoleCheck(self, 
                        assembly_type,
                        parent_obj, parent_const_names,
                        child_obj, child_const_names):
        parent_const_tf_names, child_const_tf_names = \
                            self.AT.get_cri_tf(
                                parent_obj, parent_const_names,
                                child_obj, child_const_names
                            )
        
        resp = self.IR.req_msg_for_HoleCheck(assembly_type, 
                parent_obj.referencePart, parent_const_tf_names,
                child_obj.referencePart, child_const_tf_names)
        return resp

    def send_msg_for_RobotControl(self, 
                        assembly_type,
                        parent_obj, parent_const_names,
                        child_obj, child_const_names):
        parent_const_tf_names, child_const_tf_names = \
                            self.AT.get_cri_tf(
                                parent_obj, parent_const_names,
                                child_obj, child_const_names
                            )

        resp = self.IR.req_msg_for_RobotControl(assembly_type, 
                parent_obj.referencePart, parent_const_tf_names,
                child_obj.referencePart, child_const_tf_names)
        return resp

    def update_holepin_pose(self, resp, parent_obj, parent_const_names, child_obj, child_const_names):
        parent_const_tf_names, child_const_tf_names = \
            self.AT.get_cri_tf(parent_obj, parent_const_names, child_obj, child_const_names)
        pre_parent_const_pose = []
        for (const_name, tf_name) in zip(parent_const_names, parent_const_tf_names):
            target_const = parent_obj.assemConsts[const_name]
            if "end" in tf_name:
                target_pose = target_const["endCoordinate"]
            else:
                target_pose = target_const["entryCoordinate"]
            pre_parent_const_pose.append(target_pose)
        '''print("---")
        print("Pre parent const pose:")
        pprint(pre_parent_const_pose)
        print("")'''

        re_parent_const_pose = resp.holepin_pose
        '''print("---")
        print("Given parent const pose:")
        print(re_parent_const_pose)
        print("---")'''

        for const_name, tf_name, re_pose in zip(parent_const_names, parent_const_tf_names, re_parent_const_pose):
            xyz, quat = self.get_xyz_quat_from_tfpose(re_pose)
            parent_obj = self.AT.update_const_tf(parent_obj, const_name, tf_name, xyz, quat)

        edited_parent_const_pose = []
        for (const_name, tf_name) in zip(parent_const_names, parent_const_tf_names):
            target_const = parent_obj.assemConsts[const_name]
            if "end" in tf_name:
                target_pose = target_const["endCoordinate"]
            else:
                target_pose = target_const["entryCoordinate"]
            edited_parent_const_pose.append(target_pose)
        '''print("---")
        print("Edited parent const pose:")
        pprint(edited_parent_const_pose)
        print("")'''

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

    def get_ref_axis_in_env(self, obj_tf, ref_axis):
        quat = tf.quaternion_from_matrix(obj_tf)
        quat_mat = tf.quaternion_matrix(quat)
        ref_axis_mat = tf.translation_matrix(ref_axis)
        ref_axis_in_env_mat = tf.concatenate_matrices(quat_mat, ref_axis_mat)
        ref_axis_in_env = tf.translation_from_matrix(ref_axis_in_env_mat)
        return ref_axis_in_env

    def publish_init_tf(self, obj_dict):
        temp_dict = copy.deepcopy(obj_dict)
        for obj_name, obj in temp_dict.items():
            print("\n---")
            obj_name = obj.referencePart
            print(obj_name)
            obj_real_pose_mat = obj.real_pose_mat
            if obj_real_pose_mat is not None:
                obj_real_pose = self.IR.get_pose_from_tf_mat(obj_real_pose_mat).pose
                self.IR.publish_obj_tf(obj, obj_real_pose)

    def publish_all_tf(self, obj_dict):
        temp_dict = copy.deepcopy(obj_dict)
        for obj_name, obj in temp_dict.items():
            print("\n---")
            obj_name = obj.referencePart
            print(obj_name)
            obj_real_pose_mat = obj.real_pose_mat
            if obj_real_pose_mat is not None:
                obj_real_pose = self.IR.get_pose_from_tf_mat(obj_real_pose_mat).pose
                self.IR.publish_just_tf(obj_name, obj_real_pose, "table")
                for comp_name, comp in obj.components.items():
                    if obj_name != comp_name:
                        print("")
                        print(comp_name)
                        comp_tr = comp["tr"]
                        comp_quat = comp["quat"]
                        comp_real_pose = self.IR.get_pose_from_tr_quat(comp_tr, comp_quat).pose
                        #comp_real_pose.header.frame_id = obj_name
                        self.IR.publish_just_tf(comp_name, comp_real_pose, obj_name)

    def publish_obj_tf(self, obj, reference):
        obj_name = obj.referencePart
        obj_real_pose_mat = obj.real_pose_mat
        obj_real_pose = self.IR.get_pose_from_tf_mat(obj_real_pose_mat).pose
        self.IR.publish_just_tf(obj_name, obj_real_pose, reference)

    def shutdown_hook(self):
        rospy.logwarn("Save temp workspace!!")
        for obj_name, obj in self.save_dict.items():
            obj_type = obj.referencePart
            obj_pose = obj.real_pose_mat
            if obj_pose is not None:
                obj_tr = tf.translation_from_matrix(obj_pose)
                obj_quat = tf.quaternion_from_matrix(obj_pose)
                obj_rpy = tf.euler_from_quaternion(obj_quat)

                file_name = "{}.txt".format(obj_type)
                file_path = os.path.join(TEMP_DIR, file_name)
                with open(file_path, "w") as f:
                    wr = csv.writer(f)
                    wr.writerow(["world"])
                    wr.writerow([obj_type])
                    wr.writerow([obj_tr[0], obj_tr[1], obj_tr[2]])
                    wr.writerow([obj_rpy[0], obj_rpy[1], obj_rpy[2]])

                for comp_name, comp in obj.components.items():
                    if comp_name == obj_type:
                        continue
                    comp_tr = comp["tr"]
                    comp_quat = comp["quat"]
                    comp_rpy = tf.euler_from_quaternion(comp_quat)
                    
                    file_name = "{}.txt".format(comp_name)
                    file_path = os.path.join(TEMP_DIR, file_name)
                    with open(file_path, "w") as f:
                        wr = csv.writer(f)
                        wr.writerow([obj_type])
                        wr.writerow([comp_name])
                        wr.writerow([comp_tr[0], comp_tr[1], comp_tr[2]])
                        wr.writerow([comp_rpy[0], comp_rpy[1], comp_rpy[2]])
    
    





def main():
    #core = SimCore()
    #core = InitialCore()
    #core.check_initial_hole_axis()
    core = CoreForComptetion(cowork=False, start_at_specific_point=False, is_sim=False)
    rospy.on_shutdown(core.shutdown_hook)
    
    #pprint(core.final_product["stefan12_step7_1"].components)
    core.show_edited_pddl()
    #core.check_can_assembly_cowork()
    #core.check_can_assembly_sim(core.edited_pddl, core.stefan_dict)



if __name__ == "__main__":	
    try :
        rospy.init_node("assembly_core")
        main()
    except rospy.ROSInterruptException:
        pass