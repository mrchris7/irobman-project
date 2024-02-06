#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray
from std_srvs.srv import SetBool
from irobman_project.srv import JointMotionPlanning, CartesianMotionPlanning
from pick_and_place_module.eef_control import MoveGroupControl
from pick_and_place_module.grasping import Gripper



class MotionPlannerNode:
    def __init__(self):
        rospy.init_node('motion_planner', anonymous=True)
        rospy.Subscriber('cube_poses', PoseArray, self.handle_cube_poses)
        self.joint_planning_service = rospy.Service('JointMotionPlanning', JointMotionPlanning, self.handle_joint_motion_planning)
        self.cartesian_planning_service = rospy.Service('CartesianMotionPlanning', CartesianMotionPlanning, self.handle_cartesian_motion_planning)
        self.gripper_control_service = rospy.Service('GripperControl', SetBool, self.handle_gripper_control)
        self.moveit_control = MoveGroupControl()
        self.gripper = Gripper()

        # Gripper Params
        self.cube_size_gripper = 0.045
        self.gripper_speed = 0.1
        self.gripper_force = 10
        self.gripper_ep_inner = 0.005
        self.gripper_ep_outer = 0.008


    def handle_cube_poses(self, poses_msg):
        # Handle incoming cube poses msg
        # update collision information?
        # update target_cube_pose
        return 0


    def handle_joint_motion_planning(self, req):
        j1 = req.joint_state.position[0]
        j2 = req.joint_state.position[1]
        j3 = req.joint_state.position[2]
        j4 = req.joint_state.position[3]
        j5 = req.joint_state.position[4]
        j6 = req.joint_state.position[5]
        j7 = req.joint_state.position[6]
        success = self.moveit_control.go_to_joint_state(j1, j2, j3, j4, j5, j6, j7)
        if success:
            return True, "Destination reached"
        else:
            return False, "Could not go to Destination"

    def handle_cartesian_motion_planning(self, req):
        x = req.pose.position.x
        y = req.pose.position.y
        z = req.pose.position.Z
        qx = req.pose.orientation.x
        qy = req.pose.orientation.y
        qz = req.pose.orientation.z
        qw = req.pose.orientation.w
        success = self.moveit_control.go_to_pose_goal_quaternion(x, y, z, qx, qy, qz, qw)
        if success:
            return True, "Destination reached"
        else:
            return False, "Could not go to Destination"
    
    def handle_gripper_control(self, req):
        message = ""
        if req.data == False:  # open gripper
            success = self.gripper.move(0.08,0.1)
            if success:
                message = "Gripper successfully opened"
            else:
                message = "Gripper could not be opened"

        if req.data == True:  # close gripper
            success = self.gripper.grasp(self.cube_size_gripper, self.gripper_speed, self.gripper_force, self.gripper_ep_inner, self.gripper_ep_outer)       
            if success:
                message = "Gripper successfully closed"
            else:
                message = "Gripper could not be closed"
        
        return success, message

if __name__ == '__main__':
    motion_planner_node = MotionPlannerNode()
    rospy.spin()