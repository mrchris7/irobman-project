#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray
from std_srvs.srv import SetBool
from irobman_project.srv import JointMotionPlanning, CartesianMotionPlanning
from pick_and_place_module.eef_control import MoveGroupControl
from pick_and_place_module.grasping import Gripper
from pick_and_place_module.plan_scene import PlanScene
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_msgs.msg import Grasp
from math import pi
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


class MotionPlannerNode:
    def __init__(self):
        rospy.init_node('motion_planner', anonymous=True)
        rospy.Subscriber('cube_poses', PoseArray, self.handle_cube_poses)
        self.joint_planning_service = rospy.Service('JointMotionPlanning', JointMotionPlanning, self.handle_joint_motion_planning)
        self.cartesian_planning_service = rospy.Service('CartesianMotionPlanning', CartesianMotionPlanning, self.handle_cartesian_motion_planning)
        self.gripper_control_service = rospy.Service('GripperControl', SetBool, self.handle_gripper_control)
        self.move_to_cube_service = rospy.Service('MoveToCube', CartesianMotionPlanning, self.handle_move_to_cube)
        self.pick_cube_service = rospy.Service('PickCube', CartesianMotionPlanning, self.handle_pick_cube)
        self.moveit_control = MoveGroupControl()
        self.gripper = Gripper()

        # Gripper Params
        self.cube_size_gripper = 0.045
        self.gripper_speed = 0.1
        self.gripper_force = 10
        self.gripper_ep_inner = 0.005
        self.gripper_ep_outer = 0.008
        self.plan_scene = PlanScene()


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
        z = req.pose.position.z
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

    def handle_move_to_cube(self, req):
        # DEBUG
        req = rospy.wait_for_message("/cube_0_odom", Odometry).pose
        # END DEBUG

        x = req.pose.position.x
        y = req.pose.position.y
        z = req.pose.position.z
        qx = req.pose.orientation.x
        qy = req.pose.orientation.y
        qz = req.pose.orientation.z
        qw = req.pose.orientation.w
        des_x = x
        des_y = y
        des_z = z + 0.4
        row = pi
        pitch = 0
        yaw = -pi/4          
        (row_cube,pitch_cube,yaw_cube)= euler_from_quaternion([qx,qy,qz,qw])

        desired_yaw=(yaw_cube+2*pi)%(pi/2)
        
        
        if desired_yaw>=pi/4:
            desired_yaw=pi/2-desired_yaw
            ori=[row,pitch,yaw+(-desired_yaw)]
        else:
            ori=[row,pitch,yaw+desired_yaw]
       
        pick_orientation = quaternion_from_euler(ori[0],ori[1],ori[2],'sxyz')
        
        
        success = self.moveit_control.go_to_pose_goal_quaternion(des_x, des_y, des_z, pick_orientation[0], pick_orientation[1], pick_orientation[2], pick_orientation[3])
        if success:
            return True, "Destination reached"
        else:
            return False, "Could not go to Destination"
    
    def handle_pick_cube(self, req):
        # DEBUG
        req = rospy.wait_for_message("/cube_0_odom", Odometry).pose
        # END DEBUG
        self.plan_scene.scene.remove_attached_object(self.moveit_control.eef_link, "cube0")
        self.plan_scene.scene.remove_world_object("cube0")
        
        self.plan_scene.set_env_constrains()
        self.plan_scene.set_table()
        self.plan_scene.set_cube_env(req.pose,"cube0")
        grasps = self.gripper.create_grasp(req.pose)

        self.moveit_control.move_group.set_support_surface_name("table1")
        self.moveit_control.move_group.pick("cube0", grasps)
        
        return True, "Yaba"

if __name__ == '__main__':
    motion_planner_node = MotionPlannerNode()
    rospy.spin()