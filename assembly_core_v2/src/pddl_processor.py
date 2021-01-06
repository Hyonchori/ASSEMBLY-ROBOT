import re
import copy

TARGET_FN_LIST = ["assembly_part", "attach_part"]
LEFT_IDENTIFIER = "("
RIGH_IDENTIFIER = ")"
regex = re.compile("{}(.*){}".format(\
    re.escape(LEFT_IDENTIFIER), re.escape(RIGH_IDENTIFIER)))


class PDDL2Seq():
    def __init__(self):
        pass
    
    def get_target_seq(self, target_pddl_path):
        pddl = self.get_pddl_from_path(target_pddl_path)
        target_line = self.find_target_line(pddl)
        target_part = self.find_part_in_line(target_line)
        target_seq = self.generate_seq(target_part)
        return target_seq

    def get_pddl_from_path(self, pddl_path):
        with open(pddl_path) as f:
            pddl = f.readlines()
        return pddl

    def find_target_line(self, pddl, target_fn_list=TARGET_FN_LIST):
        target_line = [line for line in pddl \
            if self.check_target_in_line(target_fn_list, line)]
        return target_line
    
    def check_target_in_line(self, target_fn_list, line):
        for target_fn in target_fn_list:
            if target_fn in line:
                return True
        return False
    
    def find_part_in_line(self, target_line):
        target_part = []
        for line in target_line:
            line_split = regex.findall(line)[0].split(" ")
            temp_dict = {}
            if TARGET_FN_LIST[0] in line:
                temp_dict["parent_obj_name"] = line_split[1]
                temp_dict["child_obj_name"] = line_split[2]
                temp_dict["subassembly_name"] = line_split[3]
                temp_dict["parent_const_names"] = line_split[4]
                temp_dict["child_const_names"] = line_split[5]
                if "C122925" in line_split[2] or "C104322" in line_split[2]:
                    temp_dict["assembly_type"] = "screw"
                else:
                    temp_dict["assembly_type"] = "insert"
            elif TARGET_FN_LIST[1] in line:
                temp_dict["parent_obj_name"] = line_split[1]
                temp_dict["child_obj_name"] = line_split[2]
                temp_dict["subassembly_name"] = line_split[3]
                temp_dict["assembly_type"] = "attach"
            target_part.append(temp_dict)
        return target_part

    def generate_seq(self, target_part):
        seq = []
        target_part_copy = copy.deepcopy(target_part)

        temp_seq = []
        for i in range(len(target_part_copy)):
            try:
                temp_seq.append(i)
                parent_set = set((target_part_copy[i]["parent_obj_name"], target_part_copy[i]["child_obj_name"]))
                next_set = set((target_part_copy[i+1]["parent_obj_name"], target_part_copy[i+1]["child_obj_name"]))
                
                is_same_task = (parent_set == next_set)
                if not is_same_task:
                    seq.append(temp_seq)
                    temp_seq = []
                    pass
                else:
                    if i+1 == len(target_part_copy)-1:
                        seq.append(temp_seq)
                    pass

            except IndexError:
                if i in seq[-1]:
                    pass
                else:
                    seq.append([i])
                pass

        task_seq = []
        for indices in seq:
            task = copy.deepcopy(target_part_copy[indices[0]])
            if task["assembly_type"] == "insert" or task["assembly_type"] == "screw":
                task["parent_const_names"] = [target_part_copy[idx]["parent_const_names"] for idx in indices]
                task["child_const_names"] = [target_part_copy[idx]["child_const_names"] for idx in indices]
            elif task["assembly_type"] == "attach":
                pass
            task_seq.append(task)
        
        return task_seq