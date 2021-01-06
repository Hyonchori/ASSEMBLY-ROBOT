#!/usr/bin/env python
import rospy
import csv
import os
import codecs
import math as m
import numpy as np
import tf.transformations as tf

from pprint import pprint
import rospkg
import xmltodict


r = rospkg.RosPack()
r_path = r.get_path("object_description")
urdf_dir = os.path.join(r_path, "urdfs")

class UrdfCreator():
    def __init__(self):
        pass
    
    def read_csv(self, csv_path):
        with codecs.open(csv_path, encoding="utf-8-sig") as f:
            reader = csv.reader(f)
            lines = list(reader)
        return lines

    def find_column_idx(self, lines):
        idx = []
        for line in lines:
            idx.append(lines.index(line))
            for col in line:
                try:
                    float(col)
                    idx.remove(lines.index(line))
                    break
                except:
                    pass  
        return idx

    def split_line(self, lines, idx):
        re_lines = []
        for i in range(len(idx)):
            try:
                target_range = idx[i+1] - 1
            except IndexError:
                target_range = len(lines) - 1
            if target_range - (idx[i]+1) == 0:
                target = [lines[idx[i]+1]]
            else:
                target = lines[idx[i]+1: target_range+1]
            for j in range(len(target)):
                for k in range(len(target[j])):
                    c = target[j][k]
                    if c == "TRUE":
                        target[j][k] = True
                        continue
                    elif c == "FALSE":
                        target[j][k] = False
                        continue
                    else:
                        try:
                            target[j][k] = float(target[j][k])
                        except ValueError:
                            pass
            target.insert(0, lines[idx[i]])
            re_lines.append(target)
        return re_lines

    def cal_rpy(self, z_axis):
        ref_axis = np.array([0, 0, 1])
        target_axis = z_axis
        target_axis_unit = tf.unit_vector(z_axis)
        axis_inner_product = np.dot(ref_axis, target_axis_unit)
        target_theta = m.acos(axis_inner_product)
        if abs(axis_inner_product) != 1:
            cross_axis = np.cross(ref_axis, target_axis_unit)
            cross_axis_unit = tf.unit_vector(cross_axis)
            print(cross_axis_unit)
            rot_matrix = self.get_rot_mat_by_principalAxis(cross_axis_unit, target_theta)
            rpy = np.array(list(tf.euler_from_matrix(rot_matrix)))
            rpy = np.nan_to_num(rpy)
            return rpy
        else:
            return np.array([0, target_theta, 0])

    def get_rot_mat_by_principalAxis(self, k, theta):
		ct = m.cos(theta)
		st = m.sin(theta)
		vt = 1 - m.cos(theta)
		rot_matrix = np.array([[k[0]*k[0]*vt + ct, 			k[0]*k[1]*vt - k[2]*st, 		k[0]*k[2]*vt + k[1]*st, 	0],	
							   [k[0]*k[1]*vt + k[2]*st, 	k[1]*k[1]*vt + ct, 				k[1]*k[2]*vt - k[0]*st, 	0],
							   [k[0]*k[2]*vt - k[1]*st, 	k[1]*k[2]*vt + k[0]*st, 		k[2]*k[2]*vt + ct, 			0],
							   [0,							0,								0,							1]])

		return rot_matrix

    def edit_mesh_path(self, link, target_type):
        del link["visual"]["material"]["texture"]
        visual_mesh_path = link["visual"]["geometry"]["mesh"]["@filename"]
        add_path = "{}/{}/".format("//object_description", "urdfs")
        edited_visual_path = visual_mesh_path.replace("//", add_path)
        link["visual"]["geometry"]["mesh"]["@filename"] = edited_visual_path
        link["collision"]["geometry"]["mesh"]["@filename"] = edited_visual_path
        return link

    def mesh_scaling(self, link, scale):
        visual_mesh_scale = link["visual"]["geometry"]["mesh"]["@scale"] = "{} {} {}".format(scale, scale, scale)
        collision_mesh_scale = link["collision"]["geometry"]["mesh"]["@scale"] = "{} {} {}".format(scale, scale, scale)
        return link
    
    def edit_part_urdf(self, urdf_path, lines, target_type):
        with open(urdf_path) as f:
            data = f.read()
            target_dict = xmltodict.parse(data)
            repathed_link = self.edit_mesh_path(target_dict["robot"]["link"], target_type)
            target_dict["robot"]["link"] = [repathed_link]
            target_dict["robot"]["joint"] = []
            target_dict["robot"]["gazebo"] = []

            ref_name = target_type.split(".")[0]

            for line in lines:
                col = line[0]
                datas = line[1:]
                for data in datas:
                    if "pin" in col[0]:
                        # add this part later if "pin" is in part
                        continue
                    elif "hole" in col[0]:
                        hole_num = int(data[0]) + 1
                        hole_feature = "opened" if data[9]==True else "blocked"
                        part_name = "{} {} {} {}".format(ref_name, "hole", hole_feature, hole_num)
                        part_xyz = np.array([data[1]/1000, data[2]/1000, data[3]/1000])
                        part_zAxis = np.array([data[4], data[5], data[6]])
                        part_rpy = self.cal_rpy(part_zAxis)
                        hole_length = data[8]/1000
                        hole_radius = data[7]/1000
                        part_xyz_re = part_xyz + part_zAxis*hole_length/2

                        target_link = {}
                        target_link["@name"] = part_name
                        target_link["visual"] = {}
                        target_link["visual"]["origin"] = {}
                        target_link["visual"]["origin"]["@xyz"] = "0 0 0"
                        target_link["visual"]["origin"]["@rpy"] = "0 0 0"
                        target_link["visual"]["geometry"] = {}
                        target_link["visual"]["geometry"]["cylinder"] = {}
                        target_link["visual"]["geometry"]["cylinder"]["@length"] = hole_length
                        target_link["visual"]["geometry"]["cylinder"]["@radius"] = hole_radius
                        target_dict["robot"]["link"].append(target_link)

                        target_joint = {}
                        target_joint["@name"] = "{} joint".format(part_name)
                        target_joint["@type"] = "fixed"
                        target_joint["origin"] = {}
                        target_joint["origin"]["@xyz"] = "{} {} {}".format(part_xyz_re[0], part_xyz_re[1], part_xyz_re[2])
                        target_joint["origin"]["@rpy"] = "{} {} {}".format(part_rpy[0], part_rpy[1], part_rpy[2])
                        target_joint["parent"] = {}
                        target_joint["parent"]["@link"] = target_type
                        target_joint["child"] = {}
                        target_joint["child"]["@link"] = part_name
                        target_dict["robot"]["joint"].append(target_joint)

                        target_gazebo = {}
                        target_gazebo["@reference"] = part_name
                        target_gazebo["material"] = "Gazebo/BlueTransparent"
                        target_dict["robot"]["gazebo"].append(target_gazebo)
            temp_xml = xmltodict.unparse(target_dict, pretty=True)
            

            xml_split = temp_xml.split("\n")
            xacro_line1 = '<?xml version="1.0"?>\n'
            xacro_line2 = '<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(target_dict["robot"]["@name"])
            xacro = temp_xml.replace(xml_split[0], xacro_line1)
            xacro = temp_xml.replace(xml_split[1], xacro_line2)

            urdf_name = urdf_path.split("/")[-1]
            new_urdf_path = urdf_path.replace(urdf_name, target_type+"_HolePin.xacro")
            with open(new_urdf_path, "w") as f:
                f.write(xacro)

    def cal_pinLength(self, fromXYZ, toXYZ):
        from_np = np.array([fromXYZ[0], fromXYZ[1], fromXYZ[2]]) / 1000.
        to_np = np.array([toXYZ[0], toXYZ[1], toXYZ[2]]) / 1000.
        dist_vec = to_np - from_np
        dist = np.linalg.norm(dist_vec)
        return dist

    def edit_comp_urdf(self, urdf_path, lines, target_type):
        with open(urdf_path) as f:
            data = f.read()
            target_dict = xmltodict.parse(data)
            repathed_link = self.edit_mesh_path(target_dict["robot"]["link"], target_type)
            if "C101350" in urdf_path:
                print("hi!")
                self.mesh_scaling(repathed_link, 0.9)
            target_dict["robot"]["link"] = [repathed_link]
            target_dict["robot"]["joint"] = []
            target_dict["robot"]["gazebo"] = []

            ref_name = target_type.split(".")[0]

            for line in lines:
                col = line[0]
                datas = line[1:]
                for data in datas:
                    if "pin" in col[0]:
                        pin_num = int(data[0]) + 1
                        print(data[11])
                        pin_feature = "insert" if data[11]=="pin" else "screw"
                        part_name = "{} {} {} {}".format(ref_name, "pin", pin_feature, pin_num)
                        print(part_name)
                        part_xyz = np.array([data[1]/1000, data[2]/1000, data[3]/1000])
                        part_zAxis = np.array([data[7], data[8], data[9]])
                        part_rpy = self.cal_rpy(part_zAxis)
                        pin_length = self.cal_pinLength(data[1:4], data[4:7])
                        pin_radius = data[10]/1000
                        print(part_xyz, part_zAxis, pin_length)
                        part_xyz_re = part_xyz + part_zAxis*pin_length/2

                        target_link = {}
                        target_link["@name"] = part_name
                        target_link["visual"] = {}
                        target_link["visual"]["origin"] = {}
                        target_link["visual"]["origin"]["@xyz"] = "0 0 0"
                        target_link["visual"]["origin"]["@rpy"] = "0 0 0"
                        target_link["visual"]["geometry"] = {}
                        target_link["visual"]["geometry"]["cylinder"] = {}
                        target_link["visual"]["geometry"]["cylinder"]["@length"] = pin_length
                        target_link["visual"]["geometry"]["cylinder"]["@radius"] = pin_radius
                        target_dict["robot"]["link"].append(target_link)

                        target_joint = {}
                        target_joint["@name"] = "{} joint".format(part_name)
                        target_joint["@type"] = "fixed"
                        target_joint["origin"] = {}
                        target_joint["origin"]["@xyz"] = "{} {} {}".format(part_xyz_re[0], part_xyz_re[1], part_xyz_re[2])
                        target_joint["origin"]["@rpy"] = "{} {} {}".format(part_rpy[0], part_rpy[1], part_rpy[2])
                        target_joint["parent"] = {}
                        target_joint["parent"]["@link"] = target_type
                        target_joint["child"] = {}
                        target_joint["child"]["@link"] = part_name
                        target_dict["robot"]["joint"].append(target_joint)

                        target_gazebo = {}
                        target_gazebo["@reference"] = part_name
                        target_gazebo["material"] = "Gazebo/RedTransparent"
                        target_dict["robot"]["gazebo"].append(target_gazebo)

                    elif "hole" in col[0]:
                        hole_num = int(data[0]) + 1
                        hole_feature = "opened" if data[12]==True else "blocked"
                        part_name = "{} {} {} {}".format(ref_name, "hole", hole_feature, hole_num)
                        print(part_name)
                        part_xyz = np.array([data[1]/1000, data[2]/1000, data[3]/1000])
                        part_zAxis = np.array([data[7], data[8], data[9]])
                        part_rpy = self.cal_rpy(part_zAxis)
                        hole_length = data[11]/1000
                        hole_radius = data[10]/1000
                        part_xyz_re = part_xyz + part_zAxis*hole_length/2

                        target_link = {}
                        target_link["@name"] = part_name
                        target_link["visual"] = {}
                        target_link["visual"]["origin"] = {}
                        target_link["visual"]["origin"]["@xyz"] = "0 0 0"
                        target_link["visual"]["origin"]["@rpy"] = "0 0 0"
                        target_link["visual"]["geometry"] = {}
                        target_link["visual"]["geometry"]["cylinder"] = {}
                        target_link["visual"]["geometry"]["cylinder"]["@length"] = hole_length
                        target_link["visual"]["geometry"]["cylinder"]["@radius"] = hole_radius
                        target_dict["robot"]["link"].append(target_link)

                        target_joint = {}
                        target_joint["@name"] = "{} joint".format(part_name)
                        target_joint["@type"] = "fixed"
                        target_joint["origin"] = {}
                        target_joint["origin"]["@xyz"] = "{} {} {}".format(part_xyz_re[0], part_xyz_re[1], part_xyz_re[2])
                        target_joint["origin"]["@rpy"] = "{} {} {}".format(part_rpy[0], part_rpy[1], part_rpy[2])
                        target_joint["parent"] = {}
                        target_joint["parent"]["@link"] = target_type
                        target_joint["child"] = {}
                        target_joint["child"]["@link"] = part_name
                        target_dict["robot"]["joint"].append(target_joint)

                        target_gazebo = {}
                        target_gazebo["@reference"] = part_name
                        target_gazebo["material"] = "Gazebo/BlueTransparent"
                        target_dict["robot"]["gazebo"].append(target_gazebo)
            temp_xml = xmltodict.unparse(target_dict, pretty=True)
            

            xml_split = temp_xml.split("\n")
            xacro_line1 = '<?xml version="1.0"?>\n'
            xacro_line2 = '<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(target_dict["robot"]["@name"])
            xacro = temp_xml.replace(xml_split[0], xacro_line1)
            xacro = temp_xml.replace(xml_split[1], xacro_line2)

            urdf_name = urdf_path.split("/")[-1]
            new_urdf_path = urdf_path.replace(urdf_name, target_type+"_HolePin.xacro")
            with open(new_urdf_path, "w") as f:
                f.write(xacro)


            
                    
    

target_dir = os.path.join(r_path, "hole_pin")
targets = os.listdir(target_dir)

uc = UrdfCreator()

for target_name in targets:
    if "csv" not in target_name or "lock" in target_name:
        pass
    else:
        print("\n---")
        print(target_name)
        target_path = os.path.join(target_dir, target_name)
        print(target_path)

        target_lines = uc.read_csv(target_path)
        column_idx = uc.find_column_idx(target_lines)
        re_target_lines = uc.split_line(target_lines, column_idx)



        target_type = target_name.split('.')[0] + ".SLDPRT"
        urdf_name = "{}.urdf".format(target_type) 
        mate_urdf_path = os.path.join(r_path, "urdfs", target_type, "urdf", urdf_name)
        if "part" in target_name:
            uc.edit_part_urdf(mate_urdf_path, re_target_lines, target_type)
            pass
        else:
            uc.edit_comp_urdf(mate_urdf_path, re_target_lines, target_type)
            pass
