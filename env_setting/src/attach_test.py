#!/usr/bin/env python
import rospy
from gazebo_ros_link_attacher.srv import Attach
from gazebo_ros_link_attacher.srv import AttachRequest


def attach_link(model1, link1, model2, link2):
	req = AttachRequest()
	req.model_name_1 = model1
	req.link_name_1 = link1
	req.model_name_2 = model2
	req.link_name_2 = link2
	client_attach.call(req)

def detach_link(model1, link1, model2, link2):
	req = AttachRequest()
	req.model_name_1 = model1
	req.link_name_1 = link1
	req.model_name_2 = model2
	req.link_name_2 = link2
	client_detach.call(req)



def main():
	#attach_link("simul_robot1", "wrist_3_link", "part5_1", "chair part5.SLDPRT")	
	#attach_link("part6_1", "chair part6.SLDPRT", "simul_robot1", "gripper_finger2_finger_tip_link")
	#attach_link("simul_robot1", "gripper_finger1_finger_tip_link", "part5_1", "chair part6.SLDPRT")
	#attach_link("simul_robot1", "gripper_finger2_finger_tip_link", "part5_1", "chair part6.SLDPRT")
	#attach_link("part6_1", "chair part6.SLDPRT", "part5_1", "chair part5.SLDPRT")
	#detach_link("part6_1", "chair part6.SLDPRT", "part5_1", "chair part5.SLDPRT")
	#detach_link("simul_robot1", "wrist_3_link", "part5_1", "chair part5.SLDPRT")

	detach_link("simul_robot1", "gripper_finger1_finger_tip_link", "object", "chair part5.SLDPRT")
	#attach_link("simul_robot1", "gripper_finger2_finger_tip_link", "part6_1", "part6_hole_101350-1_2")


if __name__ == "__main__":
	try:
		rospy.init_node("attach_test")

		client_attach = rospy.ServiceProxy('link_attacher_node/attach', Attach)
		client_detach = rospy.ServiceProxy('link_attacher_node/detach', Attach)
		main()
	
	except rospy.ROSInterruptException:
		pass