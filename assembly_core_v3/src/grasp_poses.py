from math import pi

grasping_pose = {}
#grasping_pose = {"C101350": [{"tr": [0, 0, 0.015], "rot": [0, 0, 0]}]}
#grasping_pose = {"C104322": [{"tr": [0, 0, 0.0], "rot": [0, 0, 0]}]}
#grasping_pose = {"C122620": [{"tr": [0, 0.01, 0.0], "rot": [pi/2, 0, 0]}]}
#grasping_pose = {"C122925": [{"tr": [0, 0, 0.0], "rot": [0, 0, 0]}]}

grasping_pose["PART1"] = []

grasping_pose["PART2"] = []
grasping_pose["PART2"].append({"tr": [0.1293, 0, 0.01], "rot": [0, pi, pi/2]})
grasping_pose["PART2"].append({"tr": [-0.1293, 0, 0.01], "rot": [0, pi, pi/2]})
grasping_pose["PART2"].append({"tr": [0, 0, 0.01], "rot": [0, pi, pi/2]})

grasping_pose["PART3"] = []
grasping_pose["PART3"].append({"tr": [0.1117, 0, 0.01], "rot": [0, pi, pi/2]})
grasping_pose["PART3"].append({"tr": [-0.1117, 0, 0.01], "rot": [0, pi, pi/2]})
grasping_pose["PART3"].append({"tr": [0, 0, 0.01], "rot": [0, pi, pi/2]})

grasping_pose["PART4"] = []
grasping_pose["PART4"].append({"tr": [0.094, 0, 0.005], "rot": [0, 0, pi/2]})
grasping_pose["PART4"].append({"tr": [0.028, -0.295, 0.005], "rot": [0, 0, pi/2]})
grasping_pose["PART4"].append({"tr": [-0.094, 0, 0.005], "rot": [0, 0, pi/2]})
grasping_pose["PART4"].append({"tr": [-0.028, -0.295, 0.005], "rot": [0, 0, pi/2]})

grasping_pose["PART5"] = []
grasping_pose["PART5"].append({"tr": [0.383863, 0.26097, 0.04], "rot": [0, 0, 0.11+pi]})
grasping_pose["PART5"].append({"tr": [0.378777, 0.435449, 0.04], "rot": [0, 0, -0.11]})
grasping_pose["PART5"].append({"tr": [0.123621, 0.40500, 0.011], "rot": [0.09, 0, pi/2]})
grasping_pose["PART5"].append({"tr": [0.245913, 0.40000, 0.022], "rot": [0.09, 0, pi/2]})
grasping_pose["PART5"].append({"tr": [0.405777, 0.735449, 0.04], "rot": [0, 0, -0.11]})

grasping_pose["PART6"] = []
grasping_pose["PART6"].append({"tr": [0.383863, 0.18097, -0.04], "rot": [0, 0, 0.11+pi]})
grasping_pose["PART6"].append({"tr": [0.000000, 0.06000, 0.00], "rot": [0, 0, pi]})
grasping_pose["PART6"].append({"tr": [0.123621, 0.40500, -0.04], "rot": [0.09, 0, pi/2]})
grasping_pose["PART6"].append({"tr": [0.378777, 0.45545, -0.011], "rot": [0, 0, -0.11]})
grasping_pose["PART6"].append({"tr": [0.245913, 0.40000, -0.022], "rot": [0.09, 0, pi/2]})
