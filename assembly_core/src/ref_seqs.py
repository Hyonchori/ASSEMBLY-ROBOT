

def insert_task(
    parent_obj_name, parent_holepin_names, child_obj_name, child_holepin_names):
    temp_seq = {}
    temp_seq["assembly_type"] = "insert"
    temp_seq["parent_obj_name"] = parent_obj_name
    temp_seq["parent_holepin_names"] = parent_holepin_names
    temp_seq["child_obj_name"] = child_obj_name
    temp_seq["child_holepin_names"] = child_holepin_names
    return temp_seq

def rotate_task(
    pre_tr, pre_quat, rotated_tr, rotated_quat, target_obj_name):
    temp_seq = {}
    temp_seq["assembly_type"] = "rotate"
    temp_seq["pre_tr"] = pre_tr
    temp_seq["pre_quat"] = pre_quat
    temp_seq["rotated_tr"] = rotated_tr
    temp_seq["rotated_quat"] = rotated_quat
    temp_seq["target_obj_name"] = target_obj_name
    temp_seq["unvalid"] = True
    return temp_seq

ref_seq = []
ref_seq.append(insert_task("part2_1", ["part2_1-hole_7"], "C122620_3", ["C122620_3-pin_1"]))
ref_seq.append(insert_task("part2_1", ["part2_1-hole_8"], "C122620_4", ["C122620_4-pin_1"]))
ref_seq.append(insert_task("part3_1", ["part3_1-hole_7"], "C122620_1", ["C122620_1-pin_1"]))
ref_seq.append(insert_task("part3_1", ["part3_1-hole_8"], "C122620_2", ["C122620_2-pin_1"]))

ref_seq.append(rotate_task(None, None, None, None, target_obj_name="part3_1"))
ref_seq.append(insert_task("part3_1", ["part3_1-hole_1"], "C101350_1", ["C101350_1-pin_1"]))
ref_seq.append(insert_task("part3_1", ["part3_1-hole_2"], "C101350_2", ["C101350_2-pin_1"]))
ref_seq.append(rotate_task(None, None, None, None, target_obj_name="part3_1"))
ref_seq.append(insert_task("part3_1", ["part3_1-hole_5"], "C101350_3", ["C101350_3-pin_1"]))
ref_seq.append(insert_task("part3_1", ["part3_1-hole_6"], "C101350_4", ["C101350_4-pin_1"]))

ref_seq.append(rotate_task(None, None, None, None, target_obj_name="part2_1"))
ref_seq.append(insert_task("part2_1", ["part2_1-hole_1"], "C101350_5", ["C101350_5-pin_1"]))
ref_seq.append(insert_task("part2_1", ["part2_1-hole_2"], "C101350_6", ["C101350_6-pin_1"]))
ref_seq.append(rotate_task(None, None, None, None, target_obj_name="part2_1"))
ref_seq.append(insert_task("part2_1", ["part2_1-hole_5"], "C101350_7", ["C101350_7-pin_1"]))
ref_seq.append(insert_task("part2_1", ["part2_1-hole_6"], "C101350_8", ["C101350_8-pin_1"]))

ref_seq.append(insert_task("part6_1", ["part6_1-hole_1", "part6_1-hole_3"], 
"part2_1", ["C101350_8-pin_1_spare", "C101350_6-pin_1_spare"]))

ref_seq.append(insert_task("part6_1", ["part6_1-hole_4", "part6_1-hole_6"], 
"part3_1", ["C101350_3-pin_1_spare", "C101350_1-pin_1_spare"]))

ref_seq.append(rotate_task(None, None, None, None, target_obj_name="part4_1"))
ref_seq.append(insert_task("part4_1", ["part4_1-hole_2"], "C101350_10", ["C101350_10-pin_1"]))
ref_seq.append(insert_task("part4_1", ["part4_1-hole_4"], "C101350_12", ["C101350_12-pin_1"]))
ref_seq.append(insert_task("part4_1", ["part4_1-hole_8"], "C101350_14", ["C101350_14-pin_1"]))
ref_seq.append(rotate_task(None, None, None, None, target_obj_name="part4_1"))
ref_seq.append(insert_task("part4_1", ["part4_1-hole_1"], "C101350_9", ["C101350_9-pin_1"]))
ref_seq.append(insert_task("part4_1", ["part4_1-hole_3"], "C101350_11", ["C101350_11-pin_1"]))
ref_seq.append(insert_task("part4_1", ["part4_1-hole_7"], "C101350_13", ["C101350_13-pin_1"]))

ref_seq.append(insert_task("part6_1", ["part6_1-hole_7", "part6_1-hole_8", "part6_1-hole_10"], 
"part4_1", ["C101350_10-pin_1_spare", "C101350_12-pin_1_spare", "C101350_14-pin_1_spare"]))

ref_seq.append(insert_task("part6_1", ["C101350_4-pin_1_spare", 
"C101350_2-pin_1_spare",
"C101350_7-pin_1_spare",
"C101350_5-pin_1_spare",
"C101350_9-pin_1_spare",
"C101350_11-pin_1_spare",
"C101350_13-pin_1_spare"], 
"part5_1", ["part5_1-hole_4", 
"part5_1-hole_6", 
"part5_1-hole_1", 
"part5_1-hole_3", 
"part5_1-hole_7", 
"part5_1-hole_8", 
"part5_1-hole_10"]))

ref_seq.append(insert_task("part6_1", ["part5_1-hole_2"], "C104322_4", ["C104322_4-pin_1"]))
ref_seq.append(insert_task("part6_1", ["part5_1-hole_5"], "C104322_5", ["C104322_5-pin_1"]))
ref_seq.append(insert_task("part6_1", ["part5_1-hole_9"], "C104322_6", ["C104322_6-pin_1"]))

ref_seq.append(rotate_task(None, None, None, None, target_obj_name="part4_1"))
ref_seq.append(insert_task("part6_1", ["part6_1-hole_2"], "C104322_1", ["C104322_1-pin_1"]))
ref_seq.append(insert_task("part6_1", ["part6_1-hole_5"], "C104322_2", ["C104322_2-pin_1"]))
ref_seq.append(insert_task("part6_1", ["part6_1-hole_9"], "C104322_3", ["C104322_3-pin_1"]))