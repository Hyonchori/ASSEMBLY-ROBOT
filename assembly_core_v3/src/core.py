#!/usr/bin/env python
import rospy

import os
import copy
import numpy as np
from pprint import pprint

from assembly_tester import AssemblyTester
import object_description

CONNECTION_PATH = "/home/cai/share_for_compt/for_kitech/holepin_connection/holepair.txt"
TARGET_OBJECT = "stefan"
ATTACH_PART = "part1"

__metaclass__ = type
class InitialCore():
    def __init__(self):
        self.connection_path = CONNECTION_PATH
        self.holepin_connection = self.read_holepin_connection(self.connection_path)

        self.AT = AssemblyTester()
        self.target_furniture = TARGET_OBJECT
        self.stefan_dict = self.get_obj_dict(self.target_furniture)

        self.final_dict, self.final_obj = \
            self.get_final_obj(self.holepin_connection, self.stefan_dict)

    def read_holepin_connection(self, target_path):
        holepin_connection = {}
        with open(target_path) as f:
            lines = f.readlines()
        for line in lines:
            line_split = line.split(" : ")
            const1 = line_split[0]
            const2 = line_split[1]
            const2 = const2.replace("\n", "")
            holepin_connection[const1] = const2
            holepin_connection[const2] = const1
        return holepin_connection

    def get_obj_dict(self, target_furniture):
        stefan_param_names = [param for param in rospy.get_param_names() \
            if ("assembly_core" in param) and (target_furniture in param)]
        obj_dict = {}
        for param in stefan_param_names:
            param_description = param.split("/")[2]
            param_split = param_description.split("_")
            obj_type = param_split[1]
            obj_total_num = int(param_split[2])
            for idx in range(1, obj_total_num+1):
                obj = object_description.Assembly_Object(param, idx)
                obj_dict[obj.obj_name] = obj
        return obj_dict

    def get_final_obj(self, holepin_connection, obj_dict):
        initial_dict = copy.deepcopy(obj_dict)
        pprint(initial_dict.keys())
        for obj_name, obj in initial_dict.items():
            if "C" not in obj_name:
                continue
            else:
                pass
            for const_name, const in obj.assemConsts.items():
                if const_name not in holepin_connection.keys():
                    continue
                else:
                    pass

                if const["feature"] != "screw":
                    continue
                else:
                    pass

                mate_const_name = holepin_connection[const_name]
                if mate_const_name is None:
                    continue
                else:
                    pass

                mate_obj_name = mate_const_name.split("-")[0]
                mate_obj = initial_dict[mate_obj_name]
                child_obj = obj
                child_const_names = [const_name]
                parent_obj = mate_obj
                parent_const_names = [mate_const_name]
                
                avail_parent_const_names = [self.AT.get_avail_const(parent_obj, parent_const_name) \
                    for parent_const_name in parent_const_names]
                
                avail_child_const_names = [self.AT.get_avail_const(child_obj, child_const_name) \
                    for child_const_name in child_const_names]

                self.assembly_task(
                    initial_dict, 
                    parent_obj, avail_parent_const_names,
                    child_obj, avail_child_const_names
                    )
                break
        pprint(initial_dict.keys())
        for obj_name, obj in initial_dict.items():
            if "C" not in obj_name:
                continue
            else:
                pass
            for const_name, const in obj.assemConsts.items():
                if const_name not in holepin_connection.keys():
                    continue
                else:
                    pass

                mate_const_name = holepin_connection[const_name]
                mate_obj_name = mate_const_name.split("-")[0]
                if mate_const_name is None:
                    continue
                else:
                    pass

                if mate_obj_name not in initial_dict.keys():
                    continue
                else:
                    pass

                if "hole" in const_name and "hole" in mate_const_name:
                    continue
                else:
                    pass

                if const["contactWith"] is not None:
                    if const_name+"_spare" in obj.assemConsts.keys() and "101350" in const_name:
                        spare_const = obj.assemConsts[const_name+"_spare"]
                        if spare_const["contactWith"] is not None:
                            continue
                        else:
                            pass
                else:
                    pass

                mate_obj = initial_dict[mate_obj_name]
                child_obj = obj
                child_const_names = [const_name]
                parent_obj = mate_obj
                parent_const_names = [mate_const_name]

                avail_parent_const_names = [self.AT.get_avail_const(parent_obj, parent_const_name) \
                    for parent_const_name in parent_const_names]
                
                avail_child_const_names = [self.AT.get_avail_const(child_obj, child_const_name) \
                    for child_const_name in child_const_names]

                self.assembly_task(
                    initial_dict, 
                    parent_obj, avail_parent_const_names,
                    child_obj, avail_child_const_names
                    )
                break
        pprint(initial_dict.keys())
        for obj_name, obj in initial_dict.items():
            if obj_name not in initial_dict.keys():
                continue
            if ATTACH_PART in obj_name:
                continue
            mate_obj_dict = {part_name: [] for part_name in initial_dict.keys() \
                if part_name != obj_name}
            for const_name, const_info in obj.assemConsts.items():
                if const_name not in holepin_connection.keys():
                    continue
                else:
                    pass

                if const_info["contactWith"] is not None:
                    if const_name+"_spare" in obj.assemConsts.keys() and "101350" in const_name:
                        spare_const = obj.assemConsts[const_name+"_spare"]
                        if spare_const["cantactWith"] is not None:
                            continue
                        else:
                            pass
                else:
                    pass

                mate_const_name = holepin_connection[const_name]
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
                        mate_obj_dict[target_obj_name].append(const_name)

            print("\n==============================================")
            rospy.logerr(obj_name)

            for mate_obj_name, used_const_names in mate_obj_dict.items():
                if used_const_names:
                    pass
                else:
                    continue

                if mate_obj_name not in initial_dict.keys():
                    continue
                else:
                    pass

                mate_obj = initial_dict[mate_obj_name]
                mate_const_names = [holepin_connection[const_name] for const_name \
                    in used_const_names]
                
                print("")
                rospy.logwarn(mate_obj_name)
                print(used_const_names)
                print(mate_const_names)
                
                parent_obj = initial_dict[obj_name]
                parent_const_names = used_const_names
                child_obj = mate_obj
                child_const_names = mate_const_names

                avail_parent_const_names = [self.AT.get_avail_const(parent_obj, parent_const_name) \
                    for parent_const_name in parent_const_names]
                
                avail_child_const_names = [self.AT.get_avail_const(child_obj, child_const_name) \
                    for child_const_name in child_const_names]

                self.assembly_task(
                    initial_dict, 
                    parent_obj, avail_parent_const_names,
                    child_obj, avail_child_const_names
                    )
        pprint(initial_dict.keys())
        attach_part_dict = {part_name: part for (part_name, part) in initial_dict.items() \
            if ATTACH_PART in part.obj_type}
        for attach_part_name, attach_part in attach_part_dict.items():
            target_obj_list = []
            used_const_names = []
            mate_const_names = []
            for const_name, const_info in attach_part.assemConsts.items():
                if const_name not in holepin_connection.keys():
                    continue
                else:
                    pass

                mate_const_name = holepin_connection[const_name]
                
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
                        target_obj_list.append(target_obj_name)
                        used_const_names.append(const_name)
                        mate_const_names.append(mate_const_name)
                        
            target_obj_set = list(set(target_obj_list))
            if len(target_obj_set) == 1:
                child_obj = attach_part
                child_const_names = used_const_names
                parent_obj = initial_dict[target_obj_set[0]]
                parent_const_names = mate_const_names
                
                self.attach_task(
                    initial_dict, 
                    parent_obj, parent_const_names,
                    child_obj, child_const_names
                    )
            else:
                rospy.logwarn("Assembly task is required more!")
        pprint(initial_dict.keys())
        final_dict = initial_dict
        final_obj = final_dict.values()[0]
        return final_dict, final_obj
    
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
    
    def attach_task(self, obj_dict, parent_obj, parent_const_names, child_obj, child_const_names):
        tr, quat, ref_axis, criterion, can_assembly = self.AT.get_AttachPose(
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
    
    def get_pose_by_tr_quat(self, tr, quat):
        point = Point(tr[0], tr[1],tr[2])
        rotation = Quaternion(quat[0], quat[1], quat[2],quat[3])
        pose = Pose(point, rotation)
        return pose
        
    def spawn_final_obj(self):
        final_obj = self.final_obj
        obj_pose = self.get_pose_by_tr_quat([0,0,1], [0,0,0,1])
        self.spawn_obj(final_obj, obj_pose)



from Interface_for_detector import InterfaceForDetector
from interface_for_robot import InterfaceForRobot

import math as m
import rospkg
import csv

r = rospkg.RosPack()
r_path = r.get_path("object_description")
TEMP_DIR = os.path.join(r_path, "temp_pose")

class CoreForCompetition(SimCore):
    def __init__(self, is_sim=False, cowork=False, start_at_specific_point=True):
        super(CoreForCompetition, self).__init__(is_sim)
        self.cowork = cowork
        self.ID = InterfaceForDetector()
        self.IR = InterfaceForRobot(self.cowork)
        self.initial_pose_update()
        self.init_compt_factors()

        initial_dict = copy.deepcopy(self.stefan_dict)
        final_dict = copy.deepcopy(self.final_dict)
        self.assembly_seq = self.generate_seq_from_finalProduct(initial_dict, final_dict)
    
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
    
    def init_compt_factors(self):
        self.crain_tr = np.array([])
        self.stefan_dict["part1_1"].is_attahPart = True
        self.stefan_dict["part6_1"].is_megaParent = True
        self.stefan_dict["C104322_1"].is_upscrew = True
        self.stefan_dict["C104322_2"].is_upscrew = True
        self.stefan_dict["C104322_3"].is_upscrew = True
    
    def generate_seq_from_finalProduct(self, init_dict, fin_dict):
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
            

            for sub_const_name, sub_const in sub_obj.assemConsts.items():
                print("\n====")
                print("{} : {}".format(sub_const_name, sub_const["contactWith"]))

            
            #while True:
            for i in range(3):
                print("\n===========================================")
                pprint(used_obj_dict.keys())
                print([sub_obj.referencePart])
                print("")
                if len(used_obj_dict.keys()) == 1:
                    break

                path = self.upscrew_setting(used_obj_dict, path)
                path = self.megaParent_setting(used_obj_dict, path)
                

                used_obj_const_dict = {}
                for used_obj in used_obj_dict.values():
                    used_obj_const = used_obj.assemConsts
                    used_obj_const_dict[used_obj.referencePart] = {}
                    for const_name, const_info in used_obj_const.items():
                        used_const_in_sub = sub_obj.assemConsts[const_name]
                        if const_info["contactWith"] is None and \
                            used_const_in_sub["contactWith"] is not None:
                            target_const = copy.deepcopy(const_info)
                            target_const["contactWith"] = used_const_in_sub["contactWith"]
                            used_obj_const_dict[used_obj.referencePart][const_name] = target_const
                
                path = self.attach_setting(used_obj_const_dict, used_obj_dict, path)
                
                candidates = self.get_candidates(used_obj_const_dict, used_obj_dict)
                break
        pprint(path)

    def upscrew_setting(self, used_obj_dict, path):
        for obj_name, obj in used_obj_dict.items():
            if obj.is_upscrew and not obj.is_setted:
                temp_seq = {}
                temp_seq["assembly_type"] = "move"
                temp_seq["target_obj_name"] = obj_name
                path.append(temp_seq)
                obj.is_setted = True
        return path
    
    def megaParent_setting(self, used_obj_dict, path):
        for obj_name, obj in used_obj_dict.items():
            if obj.is_megaParent and not obj.is_setted:
                temp_seq = {}
                temp_seq["assembly_type"] = "lift"
                path.append(temp_seq)
                obj.is_setted = True
        return path

    def attach_setting(self, used_obj_const_dict, used_obj_dict, path):
        for obj_name, obj in used_obj_dict.items():
            if obj.is_attachPart:
                rospy.logwarn(obj_name)
                const_in_attach_part = used_obj_const_dict[obj_name]
                print(const_in_attach_part)
                mate_of_attach_part = []
                for const_name, const_info in const_in_attach_part.items():
                    mate_const_name = const_info["contactWith"]
                    for temp_obj_name, used_const_dict in used_obj_const_dict.items():
                        if mate_const_name in used_const_dict.keys():
                            mate_of_attach_part.append(temp_obj_name)
                mate_of_attach_part_set = list(set(mate_of_attach_part))
                print(mate_of_attach_part)

                
        
        


    def get_candidates(self, used_obj_const_dict, used_obj_dict):
        candidates = {}
        for obj_name, obj_const_dict in used_obj_const_dict.items():
            valid_const_dict = {}
            for const_name, const_info in obj_const_dict.items():
                const_zAxis = const_info["real_zAxis"]
        return candidates


    def get_heaviest_obj(self, used_obj_dict):
        obj_name_list = used_obj_dict.keys()
        obj_mass_list = [used_obj_dict[obj_name].assem_mass for obj_name in obj_name_list]
        max_mass_idx = np.argmax(obj_mass_list)
        heaviest_obj = used_obj_dict[obj_name_list[max_mass_idx]]
        return heaviest_obj

    def process_current_seq(self, seq, obj_dict):
        '''''''''''''''''''''''''''
        '''' Il hae ra hot san ''''
        '''''''''''''''''''''''''''

rospy.init_node("core")

#core = InitialCore()

#core = SimCore(is_sim=True)
#core.spawn_final_obj()

core = CoreForCompetition(is_sim=False)