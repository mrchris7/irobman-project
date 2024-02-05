#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray
from irobman_project.srv import JointMotionPlanning, CartesianMotionPlanning, PoseMotionPlanning, GripperControl
from pick_and_place_module.eef_control import MoveGroupControl
from pick_and_place_module.grasping import Gripper



class MotionPlannerNode:
    def __init__(self):
        rospy.init_node('motion_planner_node', anonymous=True)
        rospy.Subscriber('/cube_poses', PoseArray, self.cube_callback)
        self.motion_planning_joint_service = rospy.Service('/plan_motion_joint', JointMotionPlanning, self.handle_motion_planning_joint)
        self.motion_planning_pose_service = rospy.Service('/plan_motion_pose', PoseMotionPlanning, self.handle_motion_planning_pose)
        self.gripper_control_service = rospy.Service('/gripper_control', GripperControl, self.handle_gripper_control)
        self.moveit_control = MoveGroupControl()
        self.gripper = Gripper()

        # Gripper Params
        self.cube_size_gripper = 0.045
        self.gripper_speed = 0.1
        self.gripper_force = 10
        self.gripper_ep_inner = 0.005
        self.gripper_ep_outer = 0.008


    def cube_callback(self, cube_poses_msg):
        # Handle incoming cube poses msg
        # update collision information?
        # update target_cube_pose
        return 0


    def handle_motion_planning_joint(self, req):
        j1 = req.joint_state.position[0]
        j2 = req.joint_state.position[1]
        j3 = req.joint_state.position[2]
        j4 = req.joint_state.position[3]
        j5 = req.joint_state.position[4]
        j6 = req.joint_state.position[5]
        j7 = req.joint_state.position[6]
        success = self.moveit_control.go_to_joint_state(j1, j2, j3, j4, j5, j6, j7)
        if success:
            return success, "Destination reached"
        else:
            return success, "Could not go to destination"

    def handle_motion_planning_pose(self, req):
        x = req.pose.position.x
        y = req.pose.position.y
        z = req.pose.position.Z
        qx = req.pose.orientation.x
        qy = req.pose.orientation.y
        qz = req.pose.orientation.z
        qw = req.pose.orientation.w
        success = self.moveit_control.go_to_pose_goal_quaternion(x, y, z, qx, qy, qz, qw)
        return success
    
    def handle_gripper_control(self, req):
        if req.command == "OPEN":
            success = self.gripper.move(0.08,0.1)
        if req.command == "CLOSE":
            success = self.gripper.grasp(self.cube_size_gripper, self.gripper_speed, self.gripper_force, self.gripper_ep_inner, self.gripper_ep_outer )       
        return "True"


if __name__ == '__main__':
    motion_planner_node = MotionPlannerNode()
    rospy.spin()