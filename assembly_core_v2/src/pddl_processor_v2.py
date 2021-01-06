import re
import os
import copy
from pprint import pprint

TARGET_FN_LIST = ["assembly_part", "attach_part"]
LEFT_IDENTIFIER = "("
RIGH_IDENTIFIER = ")"
REGEX = re.compile("{}(.*){}".format(\
    re.escape(LEFT_IDENTIFIER), re.escape(RIGH_IDENTIFIER)))


class PDDL2Seq():
    def __init__(self):
        pass

    def get_target_seq(self, target_pddl_path):
        pddl = self.get_pddl_from_path(target_pddl_path)
        target_line = self.find_target_line(pddl, TARGET_FN_LIST)
        divided_target_line = self.devide_line(target_line, REGEX)
        target_part = self.estimate_connection_in_pddl(divided_target_line)

    def get_pddl_from_path(self, pddl_path):
        with open(pddl_path) as f:
            pddl = f.readlines()
        return pddl
    
    def find_target_line(self, pddl, target_fn_list):
        target_line = [line for line in pddl \
            if self.check_target_in_line(target_fn_list, line)]
        return target_line
    
    def check_target_in_line(self, target_fn_list, line):
        for target_fn in target_fn_list:
            if target_fn in line:
                return True
        return False
    
    def devide_line(self, target_line, regex):
        divided_line = []
        for line in target_line:
            line_split = regex.findall(line)[0].split(" ")
            divided_line.append(line_split)
        return divided_line

    
    def estimate_connection_in_pddl(self, divided_target_line):
        connection_in_pddl = {line[3]: [] for line in divided_target_line}
        for line in divided_target_line:
            temp_dict = {}
            parent_name = line[1]
            child_name = line[2]
            sub_name = line[3]
            temp_dict["parent_name"] = parent_name
            temp_dict["child_name"] = child_name
            temp_dict["subassembly_name"] = sub_name
            
            if line[0] == "assembly_part":
                assembly_type = "screw" if ("c104322" in child_name or "c122925" in child_name) else "insert"
                parent_const_name = line[4]
                child_const_name = line[5]
                temp_dict["assembly_type"] = assembly_type
                temp_dict["parent_const_name"] = parent_const_name
                temp_dict["child_const_name"] = child_const_name
                

            elif line[0] == "attach_part":
                assembly_type = "attach"
                temp_dict["assembly_type"] = assembly_type
            
            connection_in_pddl[sub_name].append(temp_dict)
        
        pprint(connection_in_pddl)
                

    

PDDL_DIR = "/home/cai/share_for_compt/for_kitech/asm_seq/kitech_sep_v2"
p2s = PDDL2Seq()

pddl_files = os.listdir(PDDL_DIR)
pddl_files.sort()
for pddl_file in pddl_files:
    pddl_path = os.path.join(PDDL_DIR, pddl_file)
    print("\n--- {}\n".format(pddl_file))
    p2s.get_target_seq(pddl_path)