#!/usr/bin/env python
import rospy
import copy

import assembly_description
import assembly_tester

from pprint import pprint

rospy.init_node("core")

CONNECTION_PATH = "/home/cai/share_for_compt/for_kitech/holepin_connection/holepair.txt"
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
        self.client_pause()
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

                temp_path = self.upScrew_setting(used_obj_dict, used_obj_holepin_dict, temp_path)
                temp_path = self.megaParent_setting(used_obj_dict, temp_path)
                temp_path = self.attach_setting(used_obj_holepin_dict, used_obj_dict, temp_path)
                temp_path = self.attachScrew_setting(used_obj_holepin_dict, used_obj_dict, temp_path)
                
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
                            rospy.logwarn(parent_obj.reference_part)
                            rospy.logwarn(child_obj.reference_part)

                else:
                    rospy.logwarn("Rotation task is required!")
                    

            for seq in temp_path:
                print("\n")
                pprint(seq)
            
            print("=======")
            pprint(used_obj_dict.keys())
            path.append(temp_path)

            re_final_obj_name = used_obj_dict.keys()[0]
            re_final_dict[re_final_obj_name] = used_obj_dict[re_final_obj_name]
        return path, re_final_dict

    def upScrew_setting(self, used_obj_dict, used_obj_holepin_dict, path):
        for obj_name, obj in used_obj_dict.items():
            if obj_name not in used_obj_holepin_dict.keys():
                continue
            if obj_name in self.upScrew_list and not obj.is_setted:
                temp_seq = {}
                temp_seq["assembly_type"] = "move"
                temp_seq["target_obj_name"] = obj_name
                path.append(temp_seq)
                obj.is_setted = True
            
            elif obj_name in self.upScrew_list and obj.is_setted:
                screw_name = obj.assem_HolePins.keys()[0]
                mate_hole_name = used_obj_holepin_dict[obj_name][screw_name]["contact_with"]
                mate_obj = self.get_obj_by_holepin(used_obj_dict, mate_hole_name)
                if mate_obj is not None:
                    if self.megaChild in mate_obj.components.keys():
                        can_upScrew = True
                        for remain_obj_name in used_obj_dict.keys():
                            if remain_obj_name == obj_name:
                                continue
                            elif remain_obj_name.startswith(obj.obj_type) and remain_obj_name not in self.upScrew_list:
                                can_upScrew = False
                        if can_upScrew:
                            temp_seq = {}
                            temp_seq["assembly_type"] = "upScrew"
                            temp_seq["parent_obj_name"] = mate_obj.obj_name
                            temp_seq["parent_holepin_names"] = [mate_hole_name]
                            temp_seq["child_obj_name"] = obj_name
                            temp_seq["child_holepin_names"] = [screw_name]
                            path.append(temp_seq)

                            self.assembly_task(
                                used_obj_dict,
                                mate_obj, [mate_hole_name],
                                obj, [screw_name]
                            )
                            print("")
                            rospy.logwarn(mate_obj.reference_part)
                            rospy.logwarn(obj.reference_part)
                            print("")
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

    def attach_setting(self, used_obj_holepin_dict, used_obj_dict, path):
        for obj_name, obj in used_obj_dict.items():
            if obj_name not in used_obj_holepin_dict.keys():
                continue

            if obj.is_attachPart:
                holepin_in_attach_part = used_obj_holepin_dict[obj_name]
                mate_of_attach_part = []
                for holepin_name, holepin_info in holepin_in_attach_part.items():
                    mate_holepin_name = holepin_info["contact_with"]
                    for temp_obj_name, used_holepin_dict in used_obj_holepin_dict.items():
                        if mate_holepin_name in used_holepin_dict.keys():
                            mate_of_attach_part.append(temp_obj_name)
                mate_of_attach_part_set = list(set(mate_of_attach_part))

                if len(mate_of_attach_part_set) == 1:
                    can_attach = True

                    megaParent_obj = used_obj_dict[self.megaParent]
                    if self.megaChild not in megaParent_obj.components.keys():
                        can_attach = False

                    for upScrew_name in self.upScrew_list:
                        if upScrew_name in used_obj_dict.keys():
                            can_attach = False
                            break
                    
                    if can_attach:
                        child_holepin_names = holepin_in_attach_part.keys()
                        parent_holepin_names = []
                        for holepin_name in child_holepin_names:
                            temp_holepin = holepin_in_attach_part[holepin_name]
                            parent_holepin_names.append(temp_holepin["contact_with"])

                        parent_holepins = {holepin_name: megaParent_obj.assem_HolePins[holepin_name] \
                            for holepin_name in parent_holepin_names}
                        is_aligned_zAxis, rot_axis, rot_degrees = \
                            self.is_right_zAxis_for_attach(parent_holepins)
                        
                        if is_aligned_zAxis:
                            temp_seq = {}
                            temp_seq["assembly_type"] = "attach"
                            temp_seq["parent_obj_name"] = megaParent_obj.reference_part
                            temp_seq["parent_holepin_names"] = parent_holepin_names
                            temp_seq["child_obj_name"] = obj_name
                            temp_seq["child_holepin_names"] = child_holepin_names
                            path.append(temp_seq)

                            self.attach_task(
                                used_obj_dict,
                                megaParent_obj, parent_holepin_names,
                                obj, child_holepin_names
                            )
                            print("")
                            rospy.logwarn(megaParent_obj.reference_part)
                            rospy.logwarn(obj.reference_part)
                            print("")

                        else:

                            init_mat = megaParent_obj.real_pose_mat
                            init_tr = tf.translation_from_matrix(init_mat)
                            init_quat = tf.quaternion_from_matrix(init_mat)

                            real_pre_cm = megaParent_obj.real_cm

                            theta = m.radians(rot_degrees)
                            rot_quat = tf.quaternion_about_axis(theta, rot_axis)
                            rot_mat = tf.quaternion_matrix(rot_quat)
                            real_mat = megaParent_obj.real_pose_mat
                            rotated_mat = tf.concatenate_matrices(rot_mat, real_mat)
                            megaParent_obj.real_pose_mat = rotated_mat
                            megaParent_obj.update_real_holepin_pose()

                            real_rot_cm = megaParent_obj.real_cm

                            target_tr_vec = np.array(real_pre_cm) - np.array(real_rot_cm)
                            target_tr_vec[2] = 0
                            target_tr_mat = tf.translation_matrix(target_tr_vec)

                            target_real_mat = tf.concatenate_matrices(target_tr_mat, rotated_mat)
                            pprint(target_real_mat)
                            target_real_mat[2, 3] = 0
                            pprint(target_real_mat)

                            pre_tr = tf.translation_from_matrix(rotated_mat)
                            pre_quat = tf.quaternion_from_matrix(rotated_mat)

                            target_tr = tf.translation_from_matrix(target_real_mat)
                            target_quat = tf.quaternion_from_matrix(target_real_mat)
                            
                            megaParent_obj.real_pose_mat = target_real_mat
                            megaParent_obj.update_real_holepin_pose()

                            real_rot_cm2 = megaParent_obj.real_cm

                            print("!!!!!!!!!!!!!!!!!!!!!!")
                            rospy.logwarn(rot_axis)
                            rospy.logwarn(rot_degrees)

                            rospy.logerr(real_pre_cm)
                            rospy.logerr(real_rot_cm)
                            print("")
                            rospy.logerr(target_tr_vec)
                            rospy.logerr(pre_tr)
                            rospy.logerr(target_tr)
                            print("")
                            rospy.logerr(real_rot_cm2)

                            temp_seq = {}
                            temp_seq["assembly_type"] = "rotate"
                            temp_seq["pre_tr"] = init_tr
                            temp_seq["pre_quat"] = init_quat
                            temp_seq["rotated_tr"] = target_tr
                            temp_seq["rotated_quat"] = target_quat
                            temp_seq["target_obj_name"] = megaParent_obj.reference_part
                            path.append(temp_seq)
                else:
                    pass
        return path
    
    def is_right_zAxis_for_attach(self, parent_holepins):
        is_right = False
        rot_axis = None
        rot_degrees = None

        hole_z = np.array([0., 0., 0.])
        for holepin_name, holepin_info in parent_holepins.items():
            temp_hole_z = holepin_info["real_zAxis"]
            hole_z[0] += temp_hole_z[0]
            hole_z[1] += temp_hole_z[1]
            hole_z[2] += temp_hole_z[2]
        hole_z = hole_z / len(parent_holepins.keys())

        ref_z = np.array([0, 0, 1])

        degrees = self.AT.get_angles_between_two_vector(hole_z, ref_z)
        cross_vector = self.AT.get_cross_vector(hole_z, ref_z)
        print("\n---")
        print(hole_z)
        print(degrees)
        print(cross_vector)

        if degrees <= 5:
            is_right = True
        else:
            rot_axis = cross_vector
            rot_degrees = degrees

        return is_right, rot_axis, rot_degrees

    def attachScrew_setting(self, used_obj_holepin_dict, used_obj_dict, path):
        for obj_name, obj in used_obj_dict.items():
            if obj_name not in used_obj_holepin_dict.keys():
                continue

            if obj_name not in self.attachScrew_list:
                continue
            for holepin_name, holepin_info in obj.assem_HolePins.items():
                mate_holepin_name = used_obj_holepin_dict[obj_name][holepin_name]["contact_with"]
                mate_obj = self.get_obj_by_holepin(used_obj_dict, mate_holepin_name)

                if mate_obj is not None:
                    if self.attachPart in mate_obj.components.keys(): 
                        mate_holepin = mate_obj.assem_HolePins[mate_holepin_name]
                        is_aligned_zAxis = self.is_right_zAxis_for_attachScrew(mate_holepin)
                        if is_aligned_zAxis:
                            temp_seq = {}
                            temp_seq["parent_obj_name"] = mate_obj.reference_part
                            temp_seq["parent_holepin_names"] = [mate_holepin_name]
                            temp_seq["child_obj_name"] = obj.reference_part
                            temp_seq["child_holepin_names"] = [holepin_name]
                            temp_seq["assembly_type"] = "attachScrew"
                            path.append(temp_seq)

                            self.assembly_task(
                                used_obj_dict,
                                mate_obj, [mate_holepin_name],
                                obj, [holepin_name]
                            )
                            print("")
                            rospy.logwarn(mate_obj.reference_part)
                            rospy.logwarn(obj.reference_part)
        return path

    def is_right_zAxis_for_attachScrew(self, hole_info):
        is_right = False
        ref_z = np.array([0, 0, 1])
        hole_z = hole_info["real_zAxis"]
        degrees = self.AT.get_angles_between_two_vector(ref_z, hole_z)
        if degrees <= 5:
            is_right = True
        return is_right
            

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
import Interface_for_robot 

class Core_Main(Core_for_Seq_Generation):
    def __init__(self, is_sim=False, cowork=False, start_at_specific_point=False):
        super(Core_Main, self).__init__(is_sim, cowork, start_at_specific_point)
        self.IR = Interface_for_robot.InterfaceForRobot(cowork)

    def main(self):
        seq = copy.deepcopy(self.assembly_seq)
        obj_dict = copy.deepcopy(self.stefan_dict)

        self.IR.publish_init_tf(obj_dict)

        for assembly_unit in seq:
            print("\n##############################################")
            print("############ Start Assembly Task #############")
            print("##############################################\n")


            for idx in range(len(assembly_unit)):
                print("\n--- {} / {} ---".format(idx+1, len(assembly_unit)))
                current_task = assembly_unit[idx]

                while True:
                    pprint(current_task)
                    obj_dict, success = self.process_current_task(obj_dict, current_task)
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
    
    def process_current_task(self, obj_dict, task):
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
            parent_tf_names, child_tf_names = self.get_tf_names(parent_holepin_names, child_holepin_names, criterion)
            resp_HoleCheck = self.IR.send_msg_for_HoleCheck(assembly_type, parent_obj_name, parent_tf_names, child_obj_name, child_tf_names)

            if resp_HoleCheck.result:
                self.IR.send_redetected_holepin_tf(\
                    parent_obj, parent_holepin_names, criterion)

                tr, quat, ref_axis, criterion, can_assembly = \
                    self.AT.get_AsmPose_by_HolePin(parent_obj, parent_holepin_names, child_obj, child_holepin_names)
                if can_assembly:
                    self.IR.send_msg_for_RobotControl(assembly_type, parent_obj_name, parent_tf_names, child_obj_name, child_tf_names)

                    self.assembly_task(
                        temp_dict,
                        parent_obj, parent_holepin_names,
                        child_obj, child_holepin_names
                    )
                    
                    success = True
                    sub_obj = temp_dict[parent_obj_name]
                    self.IR.send_component_tf(parent_obj, sub_obj)
        
        elif assembly_type=="attach":
            parent_obj_name = task["parent_obj_name"]
            parent_obj = obj_dict[parent_obj_name]
            parent_holepin_names = task["parent_holepin_names"]
            child_obj_name = task["child_obj_name"]
            child_obj = obj_dict[child_obj_name]
            child_holepin_names = task["child_holepin_names"]
            
            tr, quat, ref_axis, criterion, can_assembly = \
                self.AT.get_AttachPose(parent_obj, parent_holepin_names, child_obj, child_holepin_names)
            parent_tf_names, child_tf_names = self.get_attach_tf_names(parent_holepin_names, child_holepin_names)
            resp_HoleCheck = self.IR.send_msg_for_HoleCheck(assembly_type, parent_obj_name, parent_tf_names, child_obj_name, child_tf_names)

            if resp_HoleCheck.result:
                self.IR.send_redetected_holepin_tf(\
                    parent_obj, parent_holepin_names, criterion)

                tr, quat, ref_axis, criterion, can_assembly = \
                    self.AT.get_AttachPose(parent_obj, parent_holepin_names, child_obj, child_holepin_names)
                if can_assembly:
                    self.IR.send_msg_for_RobotControl(assembly_type, parent_obj_name, parent_tf_names, child_obj_name, child_tf_names)

                    self.attach_task(
                        temp_dict,
                        parent_obj, parent_holepin_names,
                        child_obj, child_holepin_names
                    )
                    
                    success = True
                    sub_obj = temp_dict[parent_obj_name]
                    self.IR.send_component_tf(parent_obj, sub_obj)

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
            self.IR.update_just_parent_tf_stl(obj)
            success = True

        elif assembly_type=="rotate":
            ###################################################
            ############# Interface ###########################
            ###################################################

            megaParent_obj = temp_dict[task["target_obj_name"]]

            target_real_tr = task["rotated_tr"]
            target_real_quat = task["rotated_quat"]
            target_real_mat = megaParent_obj.get_tf_matrix(target_real_tr, target_real_quat)
            megaParent_obj.real_pose_mat = target_real_mat
            megaParent_obj.update_real_holepin_pose()

            self.IR.update_just_parent_tf_stl(megaParent_obj)
            self.IR.update_component_stl(megaParent_obj)
            success = True

        return temp_dict, success
    
    def get_tf_names(self, parent_holepin_names, child_holepin_names, criterion):
        parent_tf_names = []
        child_tf_names = []
        for holepin_name1, holepin_name2, cri in zip(parent_holepin_names, child_holepin_names, criterion):
            parent_tf_names.append(holepin_name1 + "_{}".format(cri))
            child_tf_names.append(holepin_name2 + "_{}".format(cri))
        return parent_tf_names, child_tf_names

    def get_attach_tf_names(self, parent_holepin_names, child_holepin_names):
        parent_tf_names = []
        child_tf_names = []
        for holepin_name1, holepin_name2 in zip(parent_holepin_names, child_holepin_names):
            parent_tf_names.append(holepin_name1 + "_end")
            child_tf_names.append(holepin_name2 + "_entry")
        return parent_tf_names, child_tf_names

    def update_redetected_holepins(self, resp, target_obj, target_holepin_names, criterion):
        print(resp)
        for idx in range(len(target_holepin_names)):
            target_holepin_name = target_holepin_names[idx]
            target_holepin = target_obj.assem_HolePins[target_holepin_name]
            entry_or_end = criterion[idx]
            target_holepin_mat = target_holepin[entry_or_end + "_coordinate"]
            redetected_pose = resp.holepin_pose[idx]
            re_tr, _ = self.IR.get_xyz_quat_from_tfpose(redetected_pose)

            rospy.logwarn("\nUpdate redetected holepin '{}'".format(target_holepin_name))
            pprint(tf.translation_from_matrix(target_holepin_mat))
            target_holepin_mat[0, 3] = re_tr[0]
            target_holepin_mat[1, 3] = re_tr[1]
            target_holepin_mat[2, 3] = re_tr[2]
            pprint(tf.translation_from_matrix(target_holepin_mat))




        
                

#core = Initial_Core()

#core = Sim_Core(is_sim=True)
#core.spawn_final_obj()

core = Core_for_Seq_Generation(cowork=True)

#core = Core_Main(cowork=True)
#pprint(core.re_final_dict)

#core.main()