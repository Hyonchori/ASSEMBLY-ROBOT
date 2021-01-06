import rospy
import xmltodict
import dicttoxml
import copy

import utils
import numpy as np
import tf.transformations as tf

from pprint import pprint

__metaclass__ = type 
class Object_Description():
    def __init__(self, param_name):
        self.param_name = param_name
        self.param_descirption = self.param_name.split("/")[2]
        self.param_split = self.param_descirption.split("_")
        self.obj_type = self.param_split[1]

        self.xml = rospy.get_param(self.param_name)
        self.dict_type = xmltodict.parse(self.xml)
        self.constraints, self.cm, self.mass = utils.get_constraints_from_dict(self.dict_type, self.obj_type)


class Part_Object(Object_Description):
    def __init__(self, obj_param, obj_idx):
        super(Part_Object, self).__init__(obj_param)
        self.obj_idx = obj_idx
        self.obj_name = "{}_{}".format(self.obj_type, self.obj_idx)
        self.partConsts = self.get_partConsts(self.constraints)
        self.partDict = self.get_partDict(self.dict_type)
        self.partXML = xmltodict.unparse(self.partDict, pretty=True)
    
    def get_partConsts(self, constraints):
        partConsts = {}
        for constName, const in constraints.items():
            const["parent"] = self.obj_name
            newConstName = constName.replace(self.obj_type, self.obj_name)
            '''if "PART" in self.obj_name:
                const_type = const["type"]
                part_num = self.obj_type[-1]
                const_idx = constName[-1]
                newConstName = "{}{}-{}".format(const_type, part_num, const_idx)'''
            partConsts[newConstName] = const
        return partConsts

    def get_partDict(self, dict_type):
        partDict = copy.deepcopy(self.dict_type)
        robot = partDict["robot"]
        robot["@name"] = self.obj_name
        robotLinks = utils.make_it_list(robot["link"])
        robotJoints = utils.make_it_list(robot["joint"])
        robotGazebos = utils.make_it_list(robot["gazebo"])

        for link in robotLinks:
            link["@name"] = link["@name"].replace(self.obj_type, self.obj_name)
        
        for joint in robotJoints:
            joint["@name"] = joint["@name"].replace(self.obj_type, self.obj_name)
            joint["parent"]["@link"] = joint["parent"]["@link"].replace(self.obj_type, self.obj_name)
            joint["child"]["@link"] = joint["child"]["@link"].replace(self.obj_type, self.obj_name)

        for gazebo in robotGazebos:
            gazebo["@reference"] = gazebo["@reference"].replace(self.obj_type, self.obj_name)

        return partDict


class Assembly_Object(Part_Object):
    def __init__(self, obj_param, obj_idx):
        super(Assembly_Object, self).__init__(obj_param, obj_idx)
        self.referencePart = self.obj_name
        self.assem_name = self.obj_name
        self.assem_mass = self.mass
        self.components = {self.obj_name: \
            {"tr": np.array([0,0,0]), "quat": np.array([0,0,0,1]), "mass": self.mass}}
        self.subassem_components = {self.obj_name: \
            {"tr": np.array([0,0,0]), "quat": np.array([0,0,0,1]), "mass": self.mass}}
        self.assemConsts = copy.deepcopy(self.partConsts)
        self.assemDict = copy.deepcopy(self.partDict)
        self.assemXML = copy.deepcopy(self.partXML)

        self.real_pose_mat = None
        self.is_attahPart = False
        self.is_megaParent = False
        self.is_upscrew = False
        self.is_setted = False

    def attach_to(self, referencePart):
        self.referencePart = referencePart
    
    def attach_obj(self, move_obj, rel_tr, rel_quat, subassembly_name):
        self.attach_cm(move_obj.cm, move_obj.assem_mass, rel_tr, rel_quat)
        self.attach_consts(move_obj.assemConsts, rel_tr, rel_quat)
        self.attach_dict(move_obj.assemDict, rel_tr, rel_quat, move_obj.referencePart)
        self.assemXML = xmltodict.unparse(self.assemDict)
        self.assem_mass += move_obj.assem_mass

        rel_tf = utils.get_tf_matrix(rel_tr, rel_quat)
        for comp_name, comp in move_obj.components.items():
            comp_tr = comp["tr"]
            comp_quat = comp["quat"]
            comp_mass = comp["mass"]
            comp_tf = utils.get_tf_matrix(comp_tr, comp_quat)
            new_tf = tf.concatenate_matrices(rel_tf,comp_tf)
            new_tr = tf.translation_from_matrix(new_tf)
            new_quat = tf.quaternion_from_matrix(new_tf)
            self.components[comp_name] = {"tr": new_tr, "quat": new_quat, "mass": comp_mass}
        
        self.subassem_components[self.obj_name]["mass"] = self.assem_mass
        self.subassem_components[move_obj.assem_name] = {"tr": rel_tr, "quat": rel_quat, "mass": move_obj.assem_mass}

        self.assem_name = subassembly_name

    def attach_cm(self, move_cm, move_mass, rel_tr, rel_quat):
        move_cm_mat = tf.translation_matrix(move_cm)
        rel_tf = utils.get_tf_matrix(rel_tr, rel_quat)
        new_move_cm_mat = tf.concatenate_matrices(rel_tf, move_cm_mat)
        new_move_cm = tf.translation_from_matrix(new_move_cm_mat)
        inter_cm = (new_move_cm-self.cm) / (self.assem_mass+move_mass) * move_mass
        new_cm = self.cm + inter_cm
        self.cm = new_cm

    def attach_consts(self, move_const, rel_tr, rel_quat):
        rel_tf = utils.get_tf_matrix(rel_tr, rel_quat)
        const = copy.deepcopy(move_const)
        for key in const.keys():
            const[key]["parent"] = self.referencePart
            coordinate_keys = [ck for ck in const[key].keys() if "Coordinate" in ck]
            for coordinate_key in coordinate_keys:
                const[key][coordinate_key] = tf.concatenate_matrices(rel_tf, const[key][coordinate_key])
            
            if "real_zAxis" in const[key].keys():
                const[key]["real_endCoordinate"] = tf.concatenate_matrices(self.real_pose_mat, const[key]["endCoordinate"])
                const_quat = tf.quaternion_from_matrix(const[key]["real_endCoordinate"])
                const[key]["real_zAxis"] = utils.get_transformed_zAxis(const_quat)
            self.assemConsts[key] = const[key]

    def attach_dict(self, move_dict, rel_tr, rel_quat, move_name):
        ref_robot = self.assemDict["robot"]
        move_robot = move_dict["robot"]

        ref_name = ref_robot["@name"]
        ref_links = utils.make_it_list(ref_robot["link"])
        ref_joints = utils.make_it_list(ref_robot["joint"])
        ref_gazebos = utils.make_it_list(ref_robot["gazebo"])

        move_links = utils.make_it_list(move_robot["link"])
        move_joints = utils.make_it_list(move_robot["joint"])
        move_gazebos = utils.make_it_list(move_robot["gazebo"])

        ref_links += move_links
        ref_joints += move_joints
        ref_gazebos += move_gazebos

        attachJoint = copy.deepcopy(ref_joints[0])
        rel_rpy = tf.euler_from_quaternion(rel_quat)
        attachJoint["@name"] = "fixed_joint_for {}".format(move_name)
        attachJoint["@type"] = "fixed"
        attachJoint["origin"]["@xyz"] = "{} {} {}".format(rel_tr[0], rel_tr[1], rel_tr[2])
        attachJoint["origin"]["@rpy"] = "{} {} {}".format(rel_rpy[0], rel_rpy[1], rel_rpy[2])
        attachJoint["parent"]["@link"] = ref_links[0]["@name"]
        attachJoint["child"]["@link"] = move_links[0]["@name"]

        ref_joints.append(attachJoint)
    
    def update_real_const_pose(self):
        if self.real_pose_mat is None:
            rospy.logwarn("Real pose of '{}' is not detected!".format(self.referencePart))
            pass
        else:
            for const_name in self.assemConsts.keys():
                const = self.assemConsts[const_name]
                const_end_pose = const["endCoordinate"]
                real_const_end_pose = tf.concatenate_matrices(self.real_pose_mat, const_end_pose)
                const["real_endCoordinate"] = real_const_end_pose
                const_quat = tf.quaternion_from_matrix(real_const_end_pose)
                const["real_zAxis"] = utils.get_transformed_zAxis(const_quat)