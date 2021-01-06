import xmltodict
import copy
import math as m
import numpy as np
import tf.transformations as tf

from pprint import pprint

__metaclass__ = type
class Initial_Description():
    def __init__(self, obj_type, obj_xml):
        self.obj_type = obj_type
        self.obj_xml = obj_xml
        self.dict_type = xmltodict.parse(self.obj_xml)
        self.HolePins, self.cm, self.mass = self.get_HolePins_from_dict(self.dict_type, self.obj_type)

    def get_HolePins_from_dict(self, dict_type, obj_type):
        HolePins = {}
        robot = dict_type["robot"]
        robot_name = robot["@name"]
        robot_links = robot["link"]
        robot_joints = robot["joint"]
        robot_mass = float(robot_links[0]["inertial"]["mass"]["@value"])
        robot_cm = robot_links[0]["inertial"]["origin"]["@xyz"].split(" ")
        robot_cm = np.array(map(float, robot_cm))

        link_list = self.make_it_list(robot_links[1:])
        joint_list = self.make_it_list(robot_joints)

        for link, joint in zip(link_list, joint_list):
            link_name_split = link["@name"].split(" ")
            holepin_type = str(link_name_split[1])
            holepin_feature = str(link_name_split[2])
            holepin_idx = int(link_name_split[3])

            holepin_name = "{}-{}_{}".format(obj_type, holepin_type, holepin_idx)

            origin_xyz, origin_rpy = self.get_xyz_rpy_from_joint(joint)
            origin_quat = tf.quaternion_from_euler(origin_rpy[0], origin_rpy[1], origin_rpy[2])

            if holepin_type == "hole" or holepin_type == "pin":
                holepin = {}
                length = float(link["visual"]["geometry"]["cylinder"]["@length"])
                radius = float(link["visual"]["geometry"]["cylinder"]["@radius"])
                entry_coordinate = self.get_translated_origin(origin_xyz, origin_quat, [0,0,-length/2])
                end_coordinate = self.get_translated_origin(origin_xyz, origin_quat, [0,0,length/2])
                entry_coordinate[np.abs(entry_coordinate)<1e-5] = 0
                end_coordinate[np.abs(end_coordinate)<1e-5] = 0
                holepin["entry_coordinate"] = entry_coordinate
                holepin["end_coordinate"] = end_coordinate
                holepin["name"] = holepin_name
                holepin["feature"] = holepin_feature
                holepin["type"] = holepin_type
                holepin["length"] = length
                holepin["radius"] = radius
                holepin["contact_with"] = None
                HolePins[holepin_name] = holepin
                if "101350" in holepin_name:
                    spare_name = holepin_name + "_spare"
                    spare_holepin = copy.deepcopy(holepin)
                    spare_quat = self.get_twisted_quat(origin_quat, [m.pi, 0, 0])
                    spare_entry_coordinate = self.get_translated_origin(origin_xyz, spare_quat, [0,0,-length/2])
                    spare_end_coordinate = self.get_translated_origin(origin_xyz, spare_quat, [0,0,length/2])
                    spare_entry_coordinate[np.abs(spare_entry_coordinate)<1e-5] = 0
                    spare_end_coordinate[np.abs(spare_end_coordinate)<1e-5] = 0
                    spare_holepin["entry_coordinate"] = spare_entry_coordinate
                    spare_holepin["end_coordinate"] = spare_end_coordinate
                    HolePins[spare_name] = spare_holepin
            else:
                print("Given constType('{}') is not valid".format(constType))
                return TypeError

        return HolePins, robot_cm, robot_mass
    
    def make_it_list(self, target):
        if type(target) is not list:
            return [target]
        else:
            return target
    
    def get_xyz_rpy_from_joint(self, joint):
        xyz = joint["origin"]["@xyz"].split(" ")
        rpy = joint["origin"]["@rpy"].split(" ")
        return np.array(map(float, xyz)), np.array(map(float, rpy))

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

    def get_twisted_quat(self, quat, rot_degrees):
        rot_mat = tf.euler_matrix(rot_degrees[0], rot_degrees[1], rot_degrees[2])
        quat_mat = tf.quaternion_matrix(quat)
        twisted_mat = tf.concatenate_matrices(quat_mat, rot_mat)
        twisted_quat = tf.quaternion_from_matrix(twisted_mat)
        return twisted_quat


class Part_Object(Initial_Description):
    def __init__(self, obj_type, obj_xml, obj_idx):
        super(Part_Object, self).__init__(obj_type, obj_xml)
        self.obj_idx = obj_idx
        self.obj_name = "{}_{}".format(self.obj_type, self.obj_idx)
        self.part_HolePins = self.get_part_HolePins(self.HolePins)
        self.part_dict = self.get_part_dict(self.dict_type)
        self.part_xml = xmltodict.unparse(self.part_dict, pretty=True)

    def get_part_HolePins(self, holepins):
        part_HolePins = {}
        for holepin_name, holepin_info in holepins.items():
            holepin_info["parent"] = self.obj_name
            new_holepin_name = holepin_name.replace(self.obj_type, self.obj_name)
            part_HolePins[new_holepin_name] = holepin_info
        return part_HolePins

    def get_part_dict(self, dict_type):
        part_dict = copy.deepcopy(dict_type)
        robot = part_dict["robot"]
        robot["@name"] = self.obj_name
        robot_links = self.make_it_list(robot["link"])
        robot_joints = self.make_it_list(robot["joint"])
        robot_gazebos = self.make_it_list(robot["gazebo"])

        for link in robot_links:
            link["@name"] = link["@name"].replace(self.obj_type, self.obj_name)

        for joint in robot_joints:
            joint["@name"] = joint["@name"].replace(self.obj_type, self.obj_name)
            joint["parent"]["@link"] = joint["parent"]["@link"].replace(self.obj_type, self.obj_name)
            joint["child"]["@link"] = joint["child"]["@link"].replace(self.obj_type, self.obj_name)

        for gazebo in robot_gazebos:
            gazebo["@reference"] = gazebo["@reference"].replace(self.obj_type, self.obj_name)
    
        return part_dict


class Assembly_Object(Part_Object):
    def __init__(self, obj_type, obj_xml, obj_idx):
        super(Assembly_Object, self).__init__(obj_type, obj_xml, obj_idx)
        self.reference_part = self.obj_name
        self.assem_name = self.obj_name
        self.assem_mass = self.mass
        self.assem_cm = self.cm
        self.components = {self.obj_name: \
            {"tr": np.array([0,0,0]), "quat": np.array([0,0,0,1]), "mass": self.mass}}
        self.assem_HolePins = copy.deepcopy(self.part_HolePins)
        self.assem_dict = copy.deepcopy(self.part_dict)
        self.assem_xml = copy.deepcopy(self.part_xml)

        self.real_pose_mat = None
        self.real_cm = None
        self.real_cm_just_rot = None
        self.is_attachPart = False
        self.is_setted = False

    def attach_obj(self, child_obj, tr, quat, sub_name, parent_holepin_names, child_holepin_names, criterion):
        self.attach_mass(child_obj.assem_cm, child_obj.assem_mass, tr, quat)
        self.attach_holepins(child_obj.assem_HolePins, tr, quat)
        self.holepins_contacting(parent_holepin_names, child_holepin_names, criterion)
        self.attach_dict(child_obj.assem_dict, tr, quat, child_obj.reference_part)
        self.attach_components(child_obj.components, tr, quat, sub_name)

    def attach_components(self, child_components, tr, quat, sub_name):
        rel_tf = self.get_tf_matrix(tr, quat)
        for comp_name, comp_info in child_components.items():
            comp_tr = comp_info["tr"]
            comp_quat = comp_info["quat"]
            comp_mass = comp_info["mass"]
            comp_tf = self.get_tf_matrix(comp_tr, comp_quat)
            new_tf = tf.concatenate_matrices(rel_tf, comp_tf)
            new_tr = tf.translation_from_matrix(new_tf)
            new_quat = tf.quaternion_from_matrix(new_tf)
            self.components[comp_name] = {"tr": new_tr, "quat": new_quat, "mass": comp_mass}

        self.assem_name = sub_name

    def attach_mass(self, child_cm, child_mass, tr, quat):
        child_cm_mat = tf.translation_matrix(child_cm)
        rel_tf = self.get_tf_matrix(tr, quat)
        new_child_cm_mat = tf.concatenate_matrices(rel_tf, child_cm_mat)
        new_move_cm = tf.translation_from_matrix(new_child_cm_mat)
        inter_cm = (new_move_cm-self.cm) / (self.assem_mass+child_mass) * child_mass
        self.assem_cm += inter_cm
        self.assem_mass += child_mass
    
    def attach_holepins(self, child_holepins, tr, quat):
        rel_tf = self.get_tf_matrix(tr, quat)
        target_holepins = copy.deepcopy(child_holepins)
        for holepin_name in target_holepins.keys():
            target_holepins[holepin_name]["parent"] = self.reference_part
            target_holepins[holepin_name]["entry_coordinate"] = \
                tf.concatenate_matrices(rel_tf, target_holepins[holepin_name]["entry_coordinate"])
            target_holepins[holepin_name]["end_coordinate"] = \
                tf.concatenate_matrices(rel_tf, target_holepins[holepin_name]["end_coordinate"])
            
            if "real_zAxis" in target_holepins[holepin_name].keys():
                target_holepins[holepin_name]["real_end_coordinate"] = \
                    tf.concatenate_matrices(self.real_pose_mat,  target_holepins[holepin_name]["end_coordinate"])
                real_holepins_quat = tf.quaternion_from_matrix(target_holepins[holepin_name]["real_end_coordinate"])
                target_holepins[holepin_name]["real_zAxis"] = self.get_twisted_zAxis(real_holepins_quat)
            self.assem_HolePins[holepin_name] = target_holepins[holepin_name]
    
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

    def holepins_contacting(self, parent_holepin_names, child_holepin_names, criterion):
        if criterion:
            for holepin_name1, holepin_name2, cri in zip(parent_holepin_names, child_holepin_names, criterion):
                holepin_1 = self.assem_HolePins[holepin_name1]
                holepin_2 = self.assem_HolePins[holepin_name2]
                holepin_1["contact_with"] = holepin_name2
                holepin_2["contact_with"] = holepin_name1

                parent_entry_tr = tf.translation_from_matrix(holepin_1["entry_coordinate"])
                parent_end_tr = tf.translation_from_matrix(holepin_1["end_coordinate"])
                child_entry_tr = tf.translation_from_matrix(holepin_2["entry_coordinate"])
                child_end_tr = tf.translation_from_matrix(holepin_2["end_coordinate"])
                if "101350" in holepin_name2 and cri=="end":
                    if "spare" in holepin_name2:
                        spare_name = holepin_name2.replace("_spare", "")
                    else:
                        spare_name = holepin_name2 + "_spare"
                    
                    spare_holepin = self.assem_HolePins[spare_name]
                    if spare_holepin["contact_with"] is not None:
                        continue
                    else:
                        spare_entry = spare_holepin["entry_coordinate"]
                        spare_entry[0, 3] = parent_entry_tr[0]
                        spare_entry[0, 3] = parent_entry_tr[0]
                        spare_entry[0, 3] = parent_entry_tr[0]
                        spare_entry_tr = tf.translation_from_matrix(spare_entry)
                        spare_end_tr = tf.translation_from_matrix(spare_holepin["end_coordinate"])

                        spare_length = np.linalg.norm(spare_end_tr - spare_entry_tr)
                        spare_holepin["length"] = spare_length
        else:
            for holepin_name1, holepin_name2 in zip(parent_holepin_names, child_holepin_names):
                holepin_1 = self.assem_HolePins[holepin_name1]
                holepin_2 = self.assem_HolePins[holepin_name2]
                holepin_1["contact_with"] = holepin_name2
                holepin_2["contact_with"] = holepin_name1
        
    def attach_dict(self, child_dict, tr, quat, child_name):
        parent_robot = self.assem_dict["robot"]
        child_robot = child_dict["robot"]

        parent_links = self.make_it_list(parent_robot["link"])
        parent_joints = self.make_it_list(parent_robot["joint"])
        parent_gazebos = self.make_it_list(parent_robot["gazebo"])

        child_links = self.make_it_list(child_robot["link"])
        child_joints = self.make_it_list(child_robot["joint"])
        child_gazebos = self.make_it_list(child_robot["gazebo"])

        parent_links += child_links
        parent_joints += child_joints
        parent_gazebos += child_gazebos

        attach_joint = copy.deepcopy(parent_joints[0])
        rpy = tf.euler_from_quaternion(quat)
        attach_joint["@name"] = "fixed_joint_for {}".format(child_name)
        attach_joint["@type"] = "fixed"
        attach_joint["origin"]["@xyz"] = "{} {} {}".format(tr[0], tr[1], tr[2])
        attach_joint["origin"]["@rpy"] = "{} {} {}".format(rpy[0], rpy[1], rpy[2])
        attach_joint["parent"]["@link"] = parent_links[0]["@name"]
        attach_joint["child"]["@link"] = child_links[0]["@name"]

        parent_joints.append(attach_joint)
        self.assem_xml = xmltodict.unparse(self.assem_dict, pretty=True)
    
    def update_real_holepin_pose(self):
        if self.real_pose_mat is None:
            rospy.logwarn("Real pose of '{}' is not reflected".format(self.referencePart))
            pass
        else:
            for holepin_name in self.assem_HolePins.keys():
                holepin = self.assem_HolePins[holepin_name]
                holepin_end_tf = holepin["end_coordinate"]
                real_holepin_end_tf = tf.concatenate_matrices(self.real_pose_mat, holepin_end_tf)
                holepin["real_end_coordinate"] = real_holepin_end_tf
                real_quat = tf.quaternion_from_matrix(real_holepin_end_tf)
                holepin["real_zAxis"] = self.get_twisted_zAxis(real_quat)

                cm_mat = tf.translation_matrix(self.assem_cm)
                real_cm_mat = tf.concatenate_matrices(self.real_pose_mat, cm_mat)
                real_quat_mat = tf.quaternion_matrix(real_quat)
                real_cm_just_rot_mat = tf.concatenate_matrices(real_quat_mat, cm_mat)
                self.real_cm = tf.translation_from_matrix(real_cm_mat)
                self.real_cm_just_rot = tf.translation_from_matrix(real_cm_just_rot_mat)