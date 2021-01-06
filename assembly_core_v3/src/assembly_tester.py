import rospy

import copy
import collections
import math as m
import numpy as np
import tf.transformations as tf
from pprint import pprint

import utils

class AssemblyTester():
    def __init__(self):
        pass

    def check_parent_child_in_dict(self, seq, obj_dict):
        parent_obj_name = seq["parent_obj_name"]
        child_obj_name = seq["child_obj_name"]
        if parent_obj_name not in obj_dict.keys() or child_obj_name not in obj_dict.keys():
            return True
        else:
            return False

    def get_data_for_InsertScrew(self, seq, obj_dict):
        parent_obj_name = seq["parent_obj_name"]
        parent_obj = obj_dict[parent_obj_name]
        parent_const_names = seq["parent_const_names"]
        parent_avail_const_names = [self.get_avail_const(parent_obj, parent_const_name) \
            for parent_const_name in parent_const_names]
        parent_const_ready = self.is_RightConst(parent_obj, parent_const_names)
            
        child_obj_name = seq["child_obj_name"]
        child_obj = obj_dict[child_obj_name]
        child_const_names = seq["child_const_names"]
        child_avail_const_names = [self.get_avail_const(child_obj, child_const_name) \
            for child_const_name in child_const_names]
        child_const_ready = self.is_RightConst(child_obj, child_const_names)

        const_ready = parent_const_ready and child_const_ready
        return parent_obj, parent_avail_const_names, child_obj, child_avail_const_names, const_ready

    def get_avail_const(self, obj, const_name):
        if const_name not in obj.assemConsts.keys():
            rospy.logwarn("Given const '{}' is not belong to given part '{}'!".format(\
                const_name, obj.assem_name))
            return False
        else:
            if "pin" not in const_name:
                return const_name
            else:
                target_const = obj.assemConsts[const_name]
                if target_const["contactWith"] is None:
                    return const_name
                else:
                    if "spare" not in const_name:
                        spare_name = const_name + "_spare"
                    else:
                        spare_name = const_name[:-6]
                    if spare_name in obj.assemConsts.keys():
                        spare_const = obj.assemConsts[spare_name]
                        if spare_const["contactWith"] is None:
                            return spare_name
                        else:
                            pprint(target_const)
                            return False
                    else:
                        return False
    
    def is_RightConst(self, obj, const_names):
        avail_const_names = [self.get_avail_const(obj, const_name) for const_name \
            in const_names]
        const_ready = True if False not in avail_const_names else False
        return const_ready

    def get_AsmPose_by_HolePin(self, ref_obj, ref_const_names, move_obj, move_const_names):
        target_tr = np.array([0, 0, 0])
        target_quat = np.array([0, 0, 0, 1])
        ref_axis = np.array([0, 0, -1])
        criterion = []
        success = False

        ref_consts = [ref_obj.assemConsts[const] for const \
            in ref_const_names]
        move_consts = [move_obj.assemConsts[const] for const \
            in move_const_names]
        criterion = [self.HolePin_criterion(ref, move) for (ref, move) \
            in zip(ref_consts, move_consts)]
        if False in criterion:
            pprint(ref_consts)
            pprint(move_consts)
            rospy.logwarn("Hole and pin's specs are not satispying given citerion")
            pass
        else:
            ref_coors = [const[cri+"Coordinate"] for (const, cri) \
                in zip(ref_consts, criterion)]
            ref_quats = [tf.quaternion_from_matrix(coor) for coor in ref_coors]
            ref_trs = [tf.translation_from_matrix(coor) for coor in ref_coors]
            ref_zs = [utils.get_transformed_zAxis(quat) for quat in ref_quats]
            ref_z_diffs = [utils.zAxis_difference(ref_zs[0], zAxis) for zAxis in ref_zs]
            ref_tr_diffs = [tr - ref_trs[0] for tr in ref_trs]

            move_coors = [const[cri+"Coordinate"] for (const, cri) \
                in zip(move_consts, criterion)]
            move_quats = [tf.quaternion_from_matrix(coor) for coor in move_coors]
            move_trs = [tf.translation_from_matrix(coor) for coor in move_coors]
            move_zs = [utils.get_transformed_zAxis(quat) for quat in move_quats]
            move_z_diffs = [utils.zAxis_difference(move_zs[0], zAxis) for zAxis in move_zs]
            move_tr_diffs = [tr - move_trs[0] for tr in move_trs]

            is_same_z_diffs = [np.allclose(ref_z_diff, move_z_diff, rtol=0.005, atol=0.005) \
                for (ref_z_diff, move_z_diff) in zip(ref_z_diffs, move_z_diffs)]
            if not is_same_z_diffs:
                print(ref_z_diffs)
                print(move_z_diffs)
                print(is_same_z_diffs)
                rospy.logwarn("Target hole and pin's direction is not proper for assembly!")
                pass
            else:
                if len(ref_consts) > 1:

                    ref_axis = ref_zs[0]
                    fixed_tr = ref_trs[0]
                    idx = 1
                    '''for i in range(1, len(ref_trs)):
                        temp_tr = ref_trs[i]
                        temp_dist = temp_tr - fixed_tr
                        inner_product = np.dot(ref_axis, temp_dist)
                        theta = m.acos(inner_product)
                        degree = m.degrees(theta)
                        #rospy.logerr("{}, {}".format(i, degree))
                        if degree > 80 and degree < 100:
                            idx = i
                            break'''

                    move_quat_inv1 = tf.quaternion_inverse(move_quats[0])
                    temp_quat = tf.quaternion_multiply(ref_quats[0], move_quat_inv1)

                    
                    move2_tr_from_ref1_temp = \
                        utils.get_translated_origin([0,0,0], temp_quat, move_tr_diffs[idx])[:3, 3]
                    move2_tr_from_ref1_temp_re = \
                        utils.get_translated_origin([0,0,0], tf.quaternion_inverse(ref_quats[0]), move2_tr_from_ref1_temp)[:3, 3]
                    ref2_tr_from_ref1_temp = \
                        utils.get_translated_origin([0,0,0], tf.quaternion_inverse(ref_quats[0]), ref_tr_diffs[idx])[:3, 3]
                    
                    grad_move2_from_ref1_temp = \
                        np.arctan2(move2_tr_from_ref1_temp_re[1], move2_tr_from_ref1_temp_re[0])
                    grad_ref2_from_ref1_temp = \
                        np.arctan2(ref2_tr_from_ref1_temp[1], ref2_tr_from_ref1_temp[0])
                    
                    temp_grad1 = tf.quaternion_about_axis(grad_move2_from_ref1_temp, ref_axis)
                    temp_grad1_inv = tf.quaternion_inverse(temp_grad1)
                    temp_grad2 = tf.quaternion_about_axis(grad_ref2_from_ref1_temp, ref_axis)

                    target_quat = tf.quaternion_multiply(temp_grad1_inv, temp_quat)
                    target_quat = tf.quaternion_multiply(temp_grad2, target_quat)
                    target_tr = utils.get_translated_origin(ref_trs[0], target_quat, -move_trs[0])[:3, 3]
                else:
                    ref_axis = ref_zs[0]

                    move_quat_inv = tf.quaternion_inverse(move_quats[0])
                    target_quat = tf.quaternion_multiply(ref_quats[0], move_quat_inv)
                    target_tr = utils.get_translated_origin(ref_trs[0], target_quat, -move_trs[0])[:3, 3]
                
                if "pin" in ref_const_names[0]:
                    ref_axis = -ref_zs[0]
                else:
                    ref_axis = ref_zs[0]
                
                edit_quats = [tf.quaternion_multiply(target_quat, move_quat) \
                    for move_quat in move_quats]
                edit_zs = [utils.get_transformed_zAxis(quat) for quat in edit_quats]
                edit_quat_mat = tf.quaternion_matrix(target_quat)
                edit_coors = [tf.concatenate_matrices(edit_quat_mat, coor) \
                    for coor in move_coors]
                edit_trs = [tf.translation_from_matrix(coor) for coor in edit_coors]
                edit_tr_diffs = [tr - edit_trs[0] for tr in edit_trs]

                edit_tr_min = self.get_min_vector_from_ref(edit_zs, edit_trs, edit_tr_diffs)
                ref_tr_min = self.get_min_vector_from_ref(ref_zs, ref_trs, ref_tr_diffs)
                is_same_arange = [np.allclose(ref, move, rtol=0.05, atol=0.05) \
                    for (ref, move) in zip(ref_tr_min, edit_tr_min)]
                arange_success = all(is_same_arange)

                stuck_success = self.check_const_mating(ref_consts, move_consts, target_tr, target_quat)
                success = arange_success and stuck_success
                
        return target_tr, target_quat, ref_axis, criterion, success
    
    def HolePin_criterion(self, const1, const2):
        holeConst = None
        pinConst = None
        for const in (const1, const2):
            if const["type"] == "hole":
                holeConst = copy.deepcopy(const)
            elif const["type"] == "pin":
                pinConst = copy.deepcopy(const)
            else:
                return False
        
        if holeConst and pinConst:
            if holeConst["feature"]=="blocked" and pinConst["feature"]=="insert":
                if holeConst["radius"] >= pinConst["radius"]:
                    if holeConst["length"] >= pinConst["length"]:
                        return "entry"
                    else:
                        return "end"
                else:
                    return False
            
            elif holeConst["feature"]=="blocked" and pinConst["feature"]=="screw":
                if holeConst["length"] >= pinConst["length"]:
                    return "entry"
                else:
                    return False
            
            elif holeConst["feature"]=="opened" and pinConst["feature"]=="insert":
                if holeConst["radius"] == pinConst["radius"]:
                    return "entry"
                else:
                    return False

            elif holeConst["feature"]=="opened" and pinConst["feature"]=="screw":
                return "entry"
            
            else:
                return False
        else:
            return False
    
    def get_min_vector_from_ref(self, zs, trs, tr_diffs):
        projs = [np.dot(zAxis, tr_diff)*zAxis for (zAxis, tr_diff) \
            in zip(zs, tr_diffs)]
        tr_min = [tr - (trs[0]+proj) for (tr, proj) in zip(trs, projs)]
        return tr_min

    def check_const_mating(self, ref_consts, move_consts, tr, quat):
        tf_mat = utils.get_tf_matrix(tr, quat)
        for ref_const, move_const in zip(ref_consts, move_consts):
            ref_const_copy = copy.deepcopy(ref_const)
            move_const_copy = copy.deepcopy(move_const)
            move_entry = move_const_copy["entryCoordinate"]
            move_end = move_const_copy["endCoordinate"]
            move_const_copy["entryCoordinate"] = tf.concatenate_matrices(tf_mat, move_entry)
            move_const_copy["endCoordinate"] = tf.concatenate_matrices(tf_mat, move_end)

            ref_type = ref_const["type"]
            move_type = move_const["type"]
            if ref_type == "hole":
                hole_const = ref_const_copy
                pin_const = move_const_copy
            else:
                hole_const = move_const_copy
                pin_const = ref_const_copy

            hole_entry = hole_const["entryCoordinate"]
            hole_end = hole_const["endCoordinate"]

            pin_entry = pin_const["entryCoordinate"]
            pin_end = pin_const["endCoordinate"]

            hole_entry_tr = tf.translation_from_matrix(hole_entry)
            hole_end_tr = tf.translation_from_matrix(hole_end)
            pin_entry_tr = tf.translation_from_matrix(pin_entry)
            pin_end_tr = tf.translation_from_matrix(pin_end)

            ref_axis = tf.unit_vector((hole_end_tr - hole_entry_tr).round(6))
            
            eps = 1e-03
            ref_axis[np.abs(ref_axis) < eps] = 0

            inter_entry_diff = (hole_entry_tr - pin_entry_tr).round(6)
            #inter_entry_diff[np.abs(inter_entry_diff) < eps] = 0
            inter_end_diff = (hole_end_tr - pin_end_tr).round(6)
            #inter_end_diff[np.abs(inter_end_diff) < eps] = 0
            entry_end_diff = (pin_end_tr - hole_entry_tr).round(6)
            #entry_end_diff[np.abs(entry_end_diff) < eps] = 0

            print("\n---")
            print("parent: {}, child: {}".format(ref_const["name"], move_const["name"]))
            print(inter_entry_diff)
            print(inter_end_diff)
            print(entry_end_diff)

            '''if np.linalg.norm(inter_entry_diff) == 0. or np.array_equal(np.sign(inter_entry_diff), ref_axis):
                pass
            else:
                print("1")
                return False

            if pin_const["feature"] == "screw":
                continue
            else:
                if np.array_equal(np.sign(entry_end_diff), ref_axis):
                    pass
                else:
                    print("2")
                    return False
                
                if np.linalg.norm(inter_end_diff) == 0. or np.array_equal(np.sign(inter_end_diff), ref_axis):
                    pass
                else:
                    print("3")
                    return False'''
        return True

    def get_cri_tf(self, ref_obj, ref_const_names, move_obj, move_const_names):
        ref_consts = [ref_obj.assemConsts[const] for const \
            in ref_const_names]
        move_consts = [move_obj.assemConsts[const] for const \
            in move_const_names]
        criterion = [self.HolePin_criterion(ref, move) for (ref, move) \
            in zip(ref_consts, move_consts)]
        if False in criterion:
            rospy.logwarn("Hole and pin's specs are not satispying given citerion")
            pass
        else:
            ref_const_tfs = [ref_const_name+"_"+cri for (cri, ref_const_name) \
                in zip(criterion, ref_const_names)]
            move_const_tfs = [move_const_name+"_"+cri for (cri, move_const_name) \
                in zip(criterion, move_const_names)]
            return ref_const_tfs, move_const_tfs
    
    def try_attaching(self, ref_obj, ref_const_names, move_obj, move_const_names, \
        rel_tr, rel_quat, criterion, sub_name, verbose=0):
        sub_obj = copy.deepcopy(ref_obj)
        sub_obj.attach_obj(move_obj, rel_tr, rel_quat, sub_name)
        self.const_contacting(sub_obj, ref_const_names, move_const_names, criterion)

        if verbose == 0:
            print("\n--- {} ---".format(sub_obj.assem_name))
            for comp_name, comp in sub_obj.components.items():
                print("")
                print(comp_name)
                pprint(comp)
            print("")
            print("Total mass: {}".format(sub_obj.assem_mass))
        else:
            pass
        return sub_obj
    
    def const_contacting(self, sub_obj, ref_const_names, move_const_names, criterion):
        for const1_name ,const2_name, cri in zip(ref_const_names, move_const_names, criterion):
            const1 = sub_obj.assemConsts[const1_name]
            const2 = sub_obj.assemConsts[const2_name]
            const1["contactWith"] = const2_name
            const2["contactWith"] = const1_name

            ref_entry_tr = tf.translation_from_matrix(const1["entryCoordinate"])
            ref_end_tr = tf.translation_from_matrix(const1["endCoordinate"])
            move_entry_tr = tf.translation_from_matrix(const2["entryCoordinate"])
            move_end_tr = tf.translation_from_matrix(const2["endCoordinate"])
            if "pin" in const2_name and cri=="end":
                spare_name = None
                if const2_name+"_spare" in sub_obj.assemConsts.keys():
                    spare_name = const2_name+"_spare"
                elif "spare" in const2_name:
                    spare_name = const2_name.replace("_spare", "")

                if spare_name is not None:
                    const2_spare = sub_obj.assemConsts[spare_name]
                    move_entry_tr_spare = tf.translation_from_matrix(const2_spare["entryCoordinate"])
                    move_end_tr_spare = tf.translation_from_matrix(const2_spare["endCoordinate"])

                    const2_spare_entry = const2_spare["entryCoordinate"]
                    const2_spare_entry[0, 3] = ref_entry_tr[0]
                    const2_spare_entry[1, 3] = ref_entry_tr[1]
                    const2_spare_entry[2, 3] = ref_entry_tr[2]
                    move_entry_tr_spare = tf.translation_from_matrix(const2_spare["entryCoordinate"])
                    move_end_tr_spare = tf.translation_from_matrix(const2_spare["endCoordinate"])

                    const2_spare_length = np.linalg.norm(move_end_tr_spare - move_entry_tr_spare)
                    const2_spare["length"] = const2_spare_length
    
    def add_subObject(self, sub_obj, ref_obj_name, move_obj_name, obj_dict):
        if ref_obj_name not in obj_dict.keys() and move_obj_name not in obj_dict.keys():
            rospy.logwarn("Given ref_name or move_name are not in obj_dict!")
            pass
        else:
            del obj_dict[ref_obj_name]
            del obj_dict[move_obj_name]
            obj_dict[sub_obj.assem_name] = sub_obj

    def update_const_tf(self, ref_obj, const_name, tf_name, xyz, quat):
        obj = copy.deepcopy(ref_obj)
        target_const = obj.assemConsts[const_name]
        cri = tf_name.split("_")[-1]
        re_coord = utils.get_tf_matrix(xyz, quat)
        target_const[cri+"Coordinate"] = re_coord
        const_length = target_const["length"]
        if cri == "end":
            target_const["entryCoordinate"] = utils.get_translated_origin(xyz, quat, [0, 0, -const_length])
        else:
            target_const["endCoordinate"] = utils.get_translated_origin(xyz, quat, [0, 0, const_length])
        
        if const_name+"_spare" in obj.assemConsts.keys():
            spare_const = obj.assemConsts[const_name+"_spare"]
            spare_xyz = xyz
            rot_pi_x = tf.quaternion_from_euler(m.pi, 0, 0)
            spare_quat = tf.quaternion_multiply(rot_pi_x, quat)
            spare_const["endCoordinate"] = utils.get_translated_origin(spare_xyz, spare_quat, [0, 0, const_length])
            spare_const["entryCoordinate"] = utils.get_translated_origin(spare_xyz, spare_quat, [0, 0, -const_length])
        return obj

    def get_AttachPose(self, ref_obj, ref_const_names, move_obj, move_const_names):
        target_tr = np.array([0, 0, 0])
        target_quat = np.array([0, 0, 0, 1])
        ref_axis = np.array([0, 0, -1])
        criterion = []
        success = False

        ref_consts = [ref_obj.assemConsts[const] for const \
            in ref_const_names]
        move_consts = [move_obj.assemConsts[const] for const \
            in move_const_names]
            
        ref_coors = [const["endCoordinate"] for const \
            in ref_consts]
        ref_quats = [tf.quaternion_from_matrix(coor) for coor in ref_coors]
        ref_trs = [tf.translation_from_matrix(coor) for coor in ref_coors]
        ref_zs = [utils.get_transformed_zAxis(quat) for quat in ref_quats]
        ref_z_diffs = [utils.zAxis_difference(ref_zs[0], zAxis) for zAxis in ref_zs]
        ref_tr_diffs = [tr - ref_trs[0] for tr in ref_trs]

        move_coors = [const["entryCoordinate"] for const \
            in move_consts]
        move_quats = [tf.quaternion_from_matrix(coor) for coor in move_coors]
        move_trs = [tf.translation_from_matrix(coor) for coor in move_coors]
        move_zs = [utils.get_transformed_zAxis(quat) for quat in move_quats]
        move_z_diffs = [utils.zAxis_difference(move_zs[0], zAxis) for zAxis in move_zs]
        move_tr_diffs = [tr - move_trs[0] for tr in move_trs]

        is_same_z_diffs = [np.allclose(ref_z_diff, move_z_diff, rtol=0.005, atol=0.005) \
            for (ref_z_diff, move_z_diff) in zip(ref_z_diffs, move_z_diffs)]
        if not is_same_z_diffs:
            print(ref_z_diffs)
            print(move_z_diffs)
            print(is_same_z_diffs)
            rospy.logwarn("Target hole and pin's direction is not proper for assembly!")
            pass
        else:
            if len(ref_consts) > 1:
                move_quat_inv1 = tf.quaternion_inverse(move_quats[0])
                temp_quat = tf.quaternion_multiply(ref_quats[0], move_quat_inv1)

                ref_axis = ref_zs[0]

                move2_tr_from_ref1_temp = \
                    utils.get_translated_origin([0,0,0], temp_quat, move_tr_diffs[1])[:3, 3]
                move2_tr_from_ref1_temp_re = \
                    utils.get_translated_origin([0,0,0], tf.quaternion_inverse(ref_quats[0]), move2_tr_from_ref1_temp)[:3, 3]
                ref2_tr_from_ref1_temp = \
                    utils.get_translated_origin([0,0,0], tf.quaternion_inverse(ref_quats[0]), ref_tr_diffs[1])[:3, 3]
                
                grad_move2_from_ref1_temp = \
                    np.arctan2(move2_tr_from_ref1_temp_re[1], move2_tr_from_ref1_temp_re[0])
                grad_ref2_from_ref1_temp = \
                    np.arctan2(ref2_tr_from_ref1_temp[1], ref2_tr_from_ref1_temp[0])
                
                temp_grad1 = tf.quaternion_about_axis(grad_move2_from_ref1_temp, ref_axis)
                temp_grad1_inv = tf.quaternion_inverse(temp_grad1)
                temp_grad2 = tf.quaternion_about_axis(grad_ref2_from_ref1_temp, ref_axis)

                target_quat = tf.quaternion_multiply(temp_grad1_inv, temp_quat)
                target_quat = tf.quaternion_multiply(temp_grad2, target_quat)
                target_tr = utils.get_translated_origin(ref_trs[0], target_quat, -move_trs[0])[:3, 3]
            else:
                ref_axis = ref_zs[0]

                move_quat_inv = tf.quaternion_inverse(move_quats[0])
                target_quat = tf.quaternion_multiply(ref_quats[0], move_quat_inv)
                target_tr = utils.get_translated_origin(ref_trs[0], target_quat, -move_trs[0])[:3, 3]
            
            if "pin" in ref_const_names[0]:
                ref_axis = -ref_zs[0]
            else:
                ref_axis = ref_zs[0]
            
            edit_quats = [tf.quaternion_multiply(target_quat, move_quat) \
                for move_quat in move_quats]
            edit_zs = [utils.get_transformed_zAxis(quat) for quat in edit_quats]
            edit_quat_mat = tf.quaternion_matrix(target_quat)
            edit_coors = [tf.concatenate_matrices(edit_quat_mat, coor) \
                for coor in move_coors]
            edit_trs = [tf.translation_from_matrix(coor) for coor in edit_coors]
            edit_tr_diffs = [tr - edit_trs[0] for tr in edit_trs]

            edit_tr_min = self.get_min_vector_from_ref(edit_zs, edit_trs, edit_tr_diffs)
            ref_tr_min = self.get_min_vector_from_ref(ref_zs, ref_trs, ref_tr_diffs)
            is_same_arange = [np.allclose(ref, move, rtol=0.05, atol=0.05) \
                for (ref, move) in zip(ref_tr_min, edit_tr_min)]
            arange_success = all(is_same_arange)

            stuck_success = self.check_const_mating(ref_consts, move_consts, target_tr, target_quat)
            success = arange_success and stuck_success
                
        return target_tr, target_quat, ref_axis, criterion, success