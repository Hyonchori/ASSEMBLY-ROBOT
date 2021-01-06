import rospy
import copy
import math as m
import numpy as np
import tf.transformations as tf

class Assembly_Tester():
    def __init__(self):
        pass

    def get_avail_holepin_of_obj(self, obj, holepin_names):
        avail_holepin_names = []
        for holepin_name in holepin_names:
            if holepin_name not in obj.assem_HolePins.keys():
                rospy.logwarn("Given const '{}' is not belong to given part '{}'!".format(\
                    holepin_name, obj.assem_name))
                return KeyError
            else:
                target_holepin = obj.assem_HolePins[holepin_name]
                if target_holepin["contact_with"] is None:
                    avail_holepin_names.append(holepin_name)
                else:
                    if "101350" not in holepin_name:
                        rospy.logwarn("Given const '{}' is already contacted in '{}'!".format(\
                            holepin_name, target_holepin["contact_with"]))
                        return KeyError
                    else:
                        spare_name = holepin_name + "_spare"
                        spare_holepin = obj.assem_HolePins[spare_name]
                        if spare_holepin["contact_with"] is None:
                            avail_holepin_names.append(spare_name)
                        else:
                            rospy.logwarn("Given pin '{}' is both contacted in '{}'!".format(\
                                holepin_name, obj.assem_name))
                            return KeyError
        return avail_holepin_names

    def get_AsmPose_by_HolePin(self, parent_obj, parent_holepin_names, child_obj, child_holepin_names):
        target_tr = np.array([0, 0, 0])
        target_quat = np.array([0, 0, 0, 1])
        ref_axis = np.array([0, 0, -1])
        criterion = []
        success = False

        parent_holepins = [parent_obj.assem_HolePins[holepin_name] for holepin_name \
            in parent_holepin_names]
        child_holepins = [child_obj.assem_HolePins[holepin_name] for holepin_name \
            in child_holepin_names]
        criterion = [self.HolePin_critetion(parent, child) for (parent, child) \
            in zip(parent_holepins, child_holepins)]

        if False in criterion:
            rospy.logwarn("Hole and pin's specs are not satispying given citerion")
            rospy.logwarn(parent_holepins)
            rospy.logwarn(child_holepins)
        else:
            parent_coors = [holepin[cri+"_coordinate"] for (holepin, cri) \
                in zip(parent_holepins, criterion)]
            parent_quats = [tf.quaternion_from_matrix(coor) for coor in parent_coors]
            parent_trs = [tf.translation_from_matrix(coor) for coor in parent_coors]
            parent_zs = [self.get_twisted_zAxis(quat) for quat in parent_quats]
            parent_z_diffs = [np.dot(parent_zs[0], z) for z in parent_zs]
            parent_tr_diffs = [tr - parent_trs[0] for tr in parent_trs]

            child_coors = [holepin[cri+"_coordinate"] for (holepin, cri) \
                in zip(child_holepins, criterion)]
            child_quats = [tf.quaternion_from_matrix(coor) for coor in child_coors]
            child_trs = [tf.translation_from_matrix(coor) for coor in child_coors]
            child_zs = [self.get_twisted_zAxis(quat) for quat in child_quats]
            child_z_diffs = [np.dot(child_zs[0], z) for z in child_zs]
            child_tr_diffs = [tr - child_trs[0] for tr in child_trs]

            is_same_z_diffs = [np.allclose(parent_z_diff, child_z_diff, rtol=0.005, atol=0.005) \
                for (parent_z_diff, child_z_diff) in zip(parent_z_diffs, child_z_diffs)]
            if False in is_same_z_diffs:
                print(parent_z_diffs)
                print(child_z_diffs)
                print(is_same_z_diffs)
                rospy.logwarn("Target hole and pin's direction is not proper for assembly!")
            else:
                if "pin" in parent_holepin_names[0]:
                    ref_axis = -parent_zs[0]
                else:
                    ref_axis = parent_zs[0]

                if len(parent_holepins) > 1:
                    fixed_tr = parent_trs[0]
                    idx = 1
                    for i in range(1, len(parent_trs)):
                        temp_tr = parent_trs[i]
                        temp_diff = temp_tr - fixed_tr
                        degrees = self.get_angles_between_two_vector(ref_axis, temp_diff)
                        if degrees > 88 and degrees < 92:
                            idx = i
                            break
                    
                    child_quat_inv1 = tf.quaternion_inverse(child_quats[0])
                    temp_quat = tf.quaternion_multiply(parent_quats[0], child_quat_inv1)
                    temp_tf = self.get_translated_origin(parent_trs[0], temp_quat, -child_trs[0])
                    
                    parent_quat_inv1 = tf.quaternion_inverse(parent_quats[0])
                    parent_quat_inv1_mat = tf.quaternion_matrix(parent_quat_inv1)
                    temp_translated_child_holepin1 = tf.concatenate_matrices(temp_tf, child_coors[0])
                    temp_translated_child_holepin2 = tf.concatenate_matrices(temp_tf, child_coors[idx])
                    temp_child_tr1 = tf.translation_from_matrix(temp_translated_child_holepin1)
                    temp_child_tr2 = tf.translation_from_matrix(temp_translated_child_holepin2)

                    vec_from_parent = parent_trs[0] - parent_trs[idx]
                    vec_from_child = temp_child_tr1 - temp_child_tr2
                    angles_for_assembly = self.get_angles_between_two_vector(vec_from_parent, vec_from_child)
                    cross_vector = self.get_cross_vector(vec_from_child, vec_from_parent)
                    rot_direction = self.compare_axis(ref_axis, cross_vector)
                    rot_theta = m.radians(angles_for_assembly)
                    if rot_direction:
                        add_quat = tf.quaternion_about_axis(rot_theta, ref_axis)
                    else:
                        add_quat = tf.quaternion_about_axis(-rot_theta, ref_axis)
                    
                    target_quat = tf.quaternion_multiply(add_quat, temp_quat)
                    target_tr = self.get_translated_origin(parent_trs[0], target_quat, -child_trs[0])[:3, 3]
                    success = True
                    
                else:
                    child_quat_inv = tf.quaternion_inverse(child_quats[0])
                    target_quat = tf.quaternion_multiply(parent_quats[0], child_quat_inv)
                    target_tr = self.get_translated_origin(parent_trs[0], target_quat, -child_trs[0])[:3, 3]
                    success = True
        return target_tr, target_quat, ref_axis, criterion, success
                
        
    def HolePin_critetion(self, holepin1, holepin2):
        hole = None
        pin = None
        for holepin in (holepin1, holepin2):
            if holepin["type"] == "hole":
                hole = holepin
            elif holepin["type"] == "pin":
                pin = holepin
            else:
                return False
        
        if hole and pin:
            if hole["feature"]=="blocked" and pin["feature"]=="insert":
                if hole["radius"] >= pin["radius"]:
                    if hole["length"] >= pin["length"]:
                        return "entry"
                    else:
                        return "end"
                else:
                    return False
            
            elif hole["feature"]=="blocked" and pin["feature"]=="screw":
                if hole["length"] >= pin["length"]:
                    return "entry"
                else:
                    return False
            
            elif hole["feature"]=="opened" and pin["feature"]=="insert":
                if hole["radius"] == pin["radius"]:
                    return "entry"
                else:
                    return False

            elif hole["feature"]=="opened" and pin["feature"]=="screw":
                return "entry"
            
            else:
                return False
        else:
            return False

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
        twisted_tf[np.abs(twisted_tf)<1e-5] = 0

        return twisted_tf[:3, 3]
    
    def get_translated_origin(self, xyz, quat, tr):
        target_tr = tf.translation_matrix(tr)
        origin_tf = self.get_tf_matrix(xyz, quat)
        translated_origin = tf.concatenate_matrices(origin_tf, target_tr)
        return translated_origin
    
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
        return tf_mat
    
    def get_angles_between_two_vector(self, vec1, vec2):
        vec1_unit = tf.unit_vector(vec1)
        vec2_unit = tf.unit_vector(vec2)
        inner_product = np.dot(vec1_unit, vec2_unit)
        theta = m.acos(inner_product)
        degrees = m.degrees(theta)
        return abs(degrees)
    
    def get_cross_vector(self, vec1, vec2):
        vec1_unit = tf.unit_vector(vec1)
        vec2_unit = tf.unit_vector(vec2)
        cross_product = np.cross(vec1_unit, vec2_unit)
        return cross_product
    
    def compare_axis(self, vec1, vec2):
        vec = vec1 * vec2
        vec[np.abs(vec)==0.] = 0
        for value in vec:
            if value < 0:
                return False
        return True
    
    def get_rot_mat_by_principalAxis(self, k, theta):
		ct = m.cos(theta)
		st = m.sin(theta)
		vt = 1 - m.cos(theta)
		rot_matrix = np.array([[k[0]*k[0]*vt + ct, 			k[0]*k[1]*vt - k[2]*st, 		k[0]*k[2]*vt + k[1]*st, 	0],	
							   [k[0]*k[1]*vt + k[2]*st, 	k[1]*k[1]*vt + ct, 				k[1]*k[2]*vt - k[0]*st, 	0],
							   [k[0]*k[2]*vt - k[1]*st, 	k[1]*k[2]*vt + k[0]*st, 		k[2]*k[2]*vt + ct, 			0],
							   [0,							0,								0,							1]])

		return rot_matrix

    def get_AttachPose(self, parent_obj, parent_holepin_names, child_obj, child_holepin_names):
        target_tr = np.array([0, 0, 0])
        target_quat = np.array([0, 0, 0, 1])
        ref_axis = np.array([0, 0, -1])
        criterion = []
        success = False

        parent_holepins = [parent_obj.assem_HolePins[holepin_name] for holepin_name \
            in parent_holepin_names]
        child_holepins = [child_obj.assem_HolePins[holepin_name] for holepin_name \
            in child_holepin_names]

        parent_coors = [holepin["end_coordinate"] for holepin in parent_holepins]
        parent_quats = [tf.quaternion_from_matrix(coor) for coor in parent_coors]
        parent_trs = [tf.translation_from_matrix(coor) for coor in parent_coors]
        parent_zs = [self.get_twisted_zAxis(quat) for quat in parent_quats]
        parent_z_diffs = [np.dot(parent_zs[0], z) for z in parent_zs]
        parent_tr_diffs = [tr - parent_trs[0] for tr in parent_trs]

        child_coors = [holepin["entry_coordinate"] for holepin in child_holepins]
        child_quats = [tf.quaternion_from_matrix(coor) for coor in child_coors]
        child_trs = [tf.translation_from_matrix(coor) for coor in child_coors]
        child_zs = [self.get_twisted_zAxis(quat) for quat in child_quats]
        child_z_diffs = [np.dot(child_zs[0], z) for z in child_zs]
        child_tr_diffs = [tr - child_trs[0] for tr in child_trs]

        is_same_z_diffs = [np.allclose(parent_z_diff, child_z_diff, rtol=0.005, atol=0.005) \
            for (parent_z_diff, child_z_diff) in zip(parent_z_diffs, child_z_diffs)]
        if False in is_same_z_diffs:
            print(parent_z_diffs)
            print(child_z_diffs)
            print(is_same_z_diffs)
            rospy.logwarn("Target hole and pin's direction is not proper for assembly!")
        else:
            if "pin" in parent_holepin_names[0]:
                ref_axis = -parent_zs[0]
            else:
                ref_axis = parent_zs[0]

            if len(parent_holepins) > 1:
                fixed_tr = parent_trs[0]
                idx = 1
                for i in range(1, len(parent_trs)):
                    temp_tr = parent_trs[i]
                    temp_diff = temp_tr - fixed_tr
                    degrees = self.get_angles_between_two_vector(ref_axis, temp_diff)
                    if degrees > 88 and degrees < 92:
                        idx = i
                        break
                
                child_quat_inv1 = tf.quaternion_inverse(child_quats[0])
                temp_quat = tf.quaternion_multiply(parent_quats[0], child_quat_inv1)
                temp_tf = self.get_translated_origin(parent_trs[0], temp_quat, -child_trs[0])
                
                parent_quat_inv1 = tf.quaternion_inverse(parent_quats[0])
                parent_quat_inv1_mat = tf.quaternion_matrix(parent_quat_inv1)
                temp_translated_child_holepin1 = tf.concatenate_matrices(temp_tf, child_coors[0])
                temp_translated_child_holepin2 = tf.concatenate_matrices(temp_tf, child_coors[idx])
                temp_child_tr1 = tf.translation_from_matrix(temp_translated_child_holepin1)
                temp_child_tr2 = tf.translation_from_matrix(temp_translated_child_holepin2)

                vec_from_parent = parent_trs[0] - parent_trs[idx]
                vec_from_child = temp_child_tr1 - temp_child_tr2
                angles_for_assembly = self.get_angles_between_two_vector(vec_from_parent, vec_from_child)
                cross_vector = self.get_cross_vector(vec_from_child, vec_from_parent)
                rot_direction = self.compare_axis(ref_axis, cross_vector)
                rot_theta = m.radians(angles_for_assembly)
                if rot_direction:
                    add_quat = tf.quaternion_about_axis(rot_theta, ref_axis)
                else:
                    add_quat = tf.quaternion_about_axis(-rot_theta, ref_axis)
                
                target_quat = tf.quaternion_multiply(add_quat, temp_quat)
                target_tr = self.get_translated_origin(parent_trs[0], target_quat, -child_trs[0])[:3, 3]
                success = True
                
            else:
                child_quat_inv = tf.quaternion_inverse(child_quats[0])
                target_quat = tf.quaternion_multiply(parent_quats[0], child_quat_inv)
                target_tr = self.get_translated_origin(parent_trs[0], target_quat, -child_trs[0])[:3, 3]
                success = True
        return target_tr, target_quat, ref_axis, criterion, success

    def try_attaching(self, 
        parent_obj, parent_holepin_names, child_obj, child_holepin_names, 
        tr, quat, criterion, sub_name, verbose=0):
        sub_obj = copy.deepcopy(parent_obj)
        sub_obj.attach_obj(child_obj, tr, quat, sub_name, parent_holepin_names, child_holepin_names, criterion)
        
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

    def just_attaching(self, 
        parent_obj, parent_holepin_names, child_obj, child_holepin_names, 
        tr, quat, criterion, sub_name, verbose=0):
        sub_obj = copy.deepcopy(parent_obj)
        sub_obj.attach_obj(child_obj, tr, quat, sub_name, parent_holepin_names, child_holepin_names, criterion)
        
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