import rospy
import sys
import copy

import actionlib
import control_msgs.msg

import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose

a = moveit_msgs.msg.CollisionObject

__metaclass__ = type
class BasicController():
    def __init__(self, robot_description, joint_state_topic):
        self.robot_description = robot_description
        self.joint_state_topic = joint_state_topic
        
        self.mc = moveit_commander
        self.scene = self.mc.PlanningSceneInterface()
        self.mc.roscpp_initialize(self.joint_state_topic)
        self.robot = self.mc.RobotCommander(self.robot_description)

        self.arm1 = self.mc.MoveGroupCommander("arm1", self.robot_description)
        self.arm2 = self.mc.MoveGroupCommander("arm2", self.robot_description)
        self.hand1 = self.mc.MoveGroupCommander("hand1", self.robot_description)
        self.hand2 = self.mc.MoveGroupCommander("hand2", self.robot_description)

        self.arm1_base = "robot1_shoulder_link"
        self.arm2_base = "robot2_shoulder_link"

        self.max_grasp_position = 0.7
        self.client_hand1 = actionlib.SimpleActionClient(
            "/hand1_controller/gripper_cmd",
            control_msgs.msg.GripperCommandAction)
        self.client_hand2 = actionlib.SimpleActionClient(
            "/hand2_controller/gripper_cmd",
            control_msgs.msg.GripperCommandAction)

        self.ready()

    def ready(self):
        self.arm1.set_named_target("arm1_ready")
        self.arm2.set_named_target("arm2_ready")
        plan1 = self.arm1.plan()
        plan2 = self.arm2.plan()
        self.arm1.execute(plan1)
        self.arm2.execute(plan2)

    def approach_to_obj(self, arm, target_pose, z_offset):
        target_tr = copy.deepcopy(target_pose.position)
        target_tr.z += z_offset
        wpose = arm.get_current_pose().pose
        #rospy.logwarn(wpose.position)
        wpose.position = target_tr
        #rospy.logwarn(wpose.position)
        return self.movel_by_path(arm, wpose)

    def movel(self, arm, cartesian_pose):
        arm.set_pose_target(cartesian_pose)
        plan = arm.go(wait=True)
        arm.stop()
        arm.clear_pose_targets()

    def movel_just_tr(self, arm, cartesian_pose):
        wpose = arm.get_current_pose().pose
        wpose.position = cartesian_pose.position
        return self.movel_by_path(arm, wpose)

    def move_by_target(self, arm, arm_name, target_class):
        target_name = "{}_{}".format(arm_name, target_class)
        arm.set_named_target(target_name)
        plan = arm.plan()
        arm.execute(plan)

        last_point = plan.joint_trajectory.points[-1]
        time = last_point.time_from_start.secs + \
            last_point.time_from_start.nsecs*0.000000001
        if time != 0.0:
            return time
        else:
            return 0.0

    def movel_by_path(self, arm, target_pose):
        waypoints = []
        waypoints.append(copy.deepcopy(target_pose))
        (plan, fraction) = arm.compute_cartesian_path(waypoints, 0.01, 0.0)
        #rospy.logwarn("\n")
        #rospy.logwarn(plan)
        #arm.execute(plan)

        last_point = plan.joint_trajectory.points[-1]
        time = last_point.time_from_start.secs + \
            last_point.time_from_start.nsecs*0.000000001
        if time != 0.0:
            return time
        else:
            return 0.0

    def grasp_control(self, arm_name, grasping=True):
        goal = control_msgs.msg.GripperCommandGoal()
        if grasping == True:
            goal.command.position = self.max_grasp_position
        else:
            goal.command.position = 0
        return
        if arm_name == "arm1":
            self.client_hand1.send_goal(goal)
            self.client_hand1.wait_for_result()
        else:
            self.client_hand2.send_goal(goal)
            self.client_hand2.wait_for_result()
        
            