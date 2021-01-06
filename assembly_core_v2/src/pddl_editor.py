import copy
import math as m
import numpy as np
from pprint import pprint

class PDDLEditor():
    def generate_seq_from_finProduct(self, init_dict, fin_dict):
        init_part_names = set(init_dict.keys())
        fin_part_names = set(fin_dict.keys())
        used_part_names = list(init_part_names - fin_part_names)
        output_part_names = list(fin_part_names - init_part_names)

        pprint(used_part_names)
        pprint(output_part_names)
        
        sub_obj_list = [fin_dict[part_name] for part_name in output_part_names]

        for sub_obj in sub_obj_list:
            used_obj_dict = {init_dict[part_name].referencePart: init_dict[part_name] for part_name in used_part_names \
                if part_name in sub_obj.components.keys()}
            pprint(used_obj_dict.keys())

            used_const_mate_dict = {}

            for used_obj in used_obj_dict.values():
                used_obj_const = used_obj.assemConsts
                used_const_mate_dict[used_obj.referencePart] = {}

                for const_name, const in used_obj_const.items():
                    sub_const = sub_obj.assemConsts[const_name]

                    if const["contactWith"] is None and sub_const["contactWith"] is not None:
                        const_mate = sub_const["contactWith"]
                        used_const_mate_dict[used_obj.referencePart][const_name] = const
                        used_const_mate_dict[used_obj.referencePart][const_name]["fufilled"] = False
                        used_const_mate_dict[used_obj.referencePart][const_name]["contactWith"] = sub_const["contactWith"]

            pprint(used_const_mate_dict)

            path = []

            candidates = self.get_candidates(used_const_mate_dict, used_obj_dict)
            print("")
            pprint(candidates)
            print("")

            child_names = []
            child_dict = {}
            for candi_name, candi_info in candidates.items():
                temp_child = self.get_child(candi_info, used_obj_dict)
                child_names += temp_child.values()
                child_dict[candi_name] = temp_child
            
            for candi_name, candi_info in child_dict.items():
                child_list = candi_info.values()
                for child_name in child_list:
                    pass




    
    def get_candidates(self, used_const_mate_dict, used_obj_dict):
        candidates = {}
        for obj_name, obj_const_dict in used_const_mate_dict.items():
            used_obj = used_obj_dict[obj_name]
            temp_const_list = {}
            for const_name, const_info in obj_const_dict.items():
                const_z = const_info["real_zAxis"]
                is_right_z = self.is_right_zAxis(obj_name, const_name, const_z)
                
                if is_right_z:
                    temp_const_list[const_name] = const_info
                    candidates[obj_name] = temp_const_list
        if len(candidates) != 0:
            return list(set(candidates))
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
            is_right = np.allclose(ref_z, z_axis, rtol=0.1, atol=0.1)
            return is_right

    def get_child(self, parent_info, used_obj_dict):
        child_dict = {}
        for const_name, const_info in parent_info.items():
            mate_const_name = const_info["contactWith"]
            for used_obj_name, used_obj in used_obj_dict.items():
                if mate_const_name in used_obj.assemConsts.keys():
                    mate_obj_name = used_obj_name
                    child_dict[const_name] = mate_obj_name
                    break
                
        return child_dict