#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray
from std_srvs.srv import SetBool
from irobman_project.srv import JointMotionPlanning, CartesianMotionPlanning, PickCube,PlaceCube
from pick_and_place_module.eef_control import MoveGroupControl
from pick_and_place_module.grasping import Gripper
from pick_and_place_module.plan_scene import PlanScene
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_msgs.msg import Grasp
from math import pi,sin,cos
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


class MotionPlannerNode:
    def __init__(self):
        rospy.init_node('motion_planner', anonymous=True)
        rospy.Subscriber('cube_poses', PoseArray, self.handle_cube_poses)
        self.joint_planning_service = rospy.Service('/motion_planner/JointMotionPlanning', JointMotionPlanning, self.handle_joint_motion_planning)
        self.cartesian_planning_service = rospy.Service('/motion_planner/CartesianMotionPlanning', CartesianMotionPlanning, self.handle_cartesian_motion_planning)
        self.gripper_control_service = rospy.Service('/motion_planner/GripperControl', SetBool, self.handle_gripper_control)
        self.pick_cube_service = rospy.Service('/motion_planner/PickCube', PickCube, self.handle_pick_cube)
        self.place_cube_service = rospy.Service('/motion_planner/PlaceCube', PlaceCube, self.handle_place_cube)
        self.moveit_control = MoveGroupControl()
        self.gripper = Gripper()

        # Gripper Params
        self.cube_size_gripper = 0.045
        self.gripper_speed = 0.1
        self.gripper_force = 10
        self.gripper_ep_inner = 0.005
        self.gripper_ep_outer = 0.008
        self.plan_scene = PlanScene()

    def set_pick_orientation(self,req):
            qx = req.pose.orientation.x
            qy = req.pose.orientation.y
            qz = req.pose.orientation.z
            qw = req.pose.orientation.w
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
            return ori

    def handle_pick_cube(self, req):
        approach = req.approach
        # DEBUG
        #req = rospy.wait_for_message("/cube_0_odom", Odometry).pose
        # END DEBUG
        des_x = req.pose.position.x
        des_y = req.pose.position.y
        des_z = req.pose.position.z + 0.2
        pick_orientation_euler = self.set_pick_orientation(req)
        des_roll = pick_orientation_euler[0]
        des_pitch = pick_orientation_euler[1]
        if approach==0 or approach==2 or approach==3:
            des_yaw = pick_orientation_euler[2]
        else:
            des_yaw = pick_orientation_euler[2]+pi/2
        pick_orientation_quaternion = quaternion_from_euler(des_roll,des_pitch,des_yaw,'sxyz')
        success_go_to_cube = self.moveit_control.go_to_pose_goal_quaternion(des_x, des_y, des_z, pick_orientation_quaternion[0], pick_orientation_quaternion[1], pick_orientation_quaternion[2], pick_orientation_quaternion[3])
        if success_go_to_cube:     
            success = self.gripper.move(0.08,0.1)
            vector = 0.1
            if approach == 0:
                des_x = req.pose.position.x
                des_y = req.pose.position.y
                des_z = req.pose.position.z + 0.2
                pick_orientation_euler = self.set_pick_orientation(req)
                des_roll = pick_orientation_euler[0]
                des_pitch = pick_orientation_euler[1]
                des_yaw = pick_orientation_euler[2]
            if approach == 1:
                des_x = req.pose.position.x
                des_y = req.pose.position.y
                des_z = req.pose.position.z + 0.2
                pick_orientation_euler = self.set_pick_orientation(req)
                des_roll = pick_orientation_euler[0]
                des_pitch = pick_orientation_euler[1]
                des_yaw = pick_orientation_euler[2]+pi/2
            if approach == 2:
                pick_orientation_euler = self.set_pick_orientation(req)
                des_roll = pick_orientation_euler[0]
                des_pitch = pick_orientation_euler[1]
                des_yaw = pick_orientation_euler[2]
                des_z = req.pose.position.z + 0.1                
                des_x = req.pose.position.x+(vector*cos(des_yaw+pi/4))
                des_y = req.pose.position.y+(vector*sin(des_yaw+pi/4))
            if approach == 3:
                pick_orientation_euler = self.set_pick_orientation(req)
                des_roll = pick_orientation_euler[0]
                des_pitch = pick_orientation_euler[1]
                des_yaw = pick_orientation_euler[2]
                des_z = req.pose.position.z + 0.1                
                des_x = req.pose.position.x-(vector*cos(des_yaw+pi/4))
                des_y = req.pose.position.y-(vector*sin(des_yaw+pi/4))
            if approach == 4:
                pick_orientation_euler = self.set_pick_orientation(req)
                des_roll = pick_orientation_euler[0]
                des_pitch = pick_orientation_euler[1]
                des_yaw = pick_orientation_euler[2]+pi/2
                des_z = req.pose.position.z + 0.1                
                des_x = req.pose.position.x+(vector*cos(des_yaw+pi/4))
                des_y = req.pose.position.y+(vector*sin(des_yaw+pi/4))
            if approach == 5:
                pick_orientation_euler = self.set_pick_orientation(req)
                des_roll = pick_orientation_euler[0]
                des_pitch = pick_orientation_euler[1]
                des_yaw = pick_orientation_euler[2]+pi/2
                des_z = req.pose.position.z + 0.1                
                des_x = req.pose.position.x-(vector*cos(des_yaw+pi/4))
                des_y = req.pose.position.y-(vector*sin(des_yaw+pi/4))   
            pick_orientation_quaternion = quaternion_from_euler(des_roll,des_pitch,des_yaw,'sxyz')
            success_pre_approach = self.moveit_control.go_to_pose_goal_quaternion(des_x, des_y, des_z, pick_orientation_quaternion[0], pick_orientation_quaternion[1], pick_orientation_quaternion[2], pick_orientation_quaternion[3])
            if success_pre_approach:
                des_z = req.pose.position.z + 0.1                
                des_x = req.pose.position.x
                des_y = req.pose.position.y
                pick_orientation_quaternion = quaternion_from_euler(des_roll,des_pitch,des_yaw,'sxyz')
                success_approach = self.moveit_control.go_to_pose_goal_quaternion(des_x, des_y, des_z, pick_orientation_quaternion[0], pick_orientation_quaternion[1], pick_orientation_quaternion[2], pick_orientation_quaternion[3])
                if success_approach:
                    success_grasp = self.gripper.grasp(self.cube_size_gripper, self.gripper_speed, self.gripper_force, self.gripper_ep_inner, self.gripper_ep_outer)
                    if success_grasp:
                        des_z = req.pose.position.z + 0.25                
                        des_x = req.pose.position.x
                        des_y = req.pose.position.y
                        pick_orientation_quaternion = quaternion_from_euler(des_roll,des_pitch,des_yaw,'sxyz')
                        success_post_approach = self.moveit_control.go_to_pose_goal_quaternion(des_x, des_y, des_z, pick_orientation_quaternion[0], pick_orientation_quaternion[1], pick_orientation_quaternion[2], pick_orientation_quaternion[3])
                        if success_post_approach:
                            return True, "Cube picked successfully"
                        else: 
                            return False, "Could not go to the post-approach position"
                    else:
                        return False, "Could not grasp cube"
                else:
                    return False, "Could not go to approach position" 
            else:
               return False, "Could not go to pre-approach position" 
        else:
            return False, "Could not go to cube position"
        
    def handle_place_cube(self, req):
        # DEBUG
        #req = rospy.wait_for_message("/cube_0_odom", Odometry).pose
        # END DEBUG
        des_x = req.pose.position.x
        des_y = req.pose.position.y
        des_z = req.pose.position.z + 0.2
        pick_orientation_euler = self.set_pick_orientation(req)
        des_roll = pick_orientation_euler[0]
        des_pitch = pick_orientation_euler[1]
        des_yaw = pick_orientation_euler[2]
        pick_orientation_quaternion = quaternion_from_euler(des_roll,des_pitch,des_yaw,'sxyz')
        success_pre_place_cube = self.moveit_control.go_to_pose_goal_quaternion(des_x, des_y, des_z, pick_orientation_quaternion[0], pick_orientation_quaternion[1], pick_orientation_quaternion[2], pick_orientation_quaternion[3])
        if success_pre_place_cube:
            des_z = req.pose.position.z + 0.1
            success_place_cube = self.moveit_control.go_to_pose_goal_quaternion(des_x, des_y, des_z, pick_orientation_quaternion[0], pick_orientation_quaternion[1], pick_orientation_quaternion[2], pick_orientation_quaternion[3])
            success = self.gripper.move(0.08,0.1)
            success_postplace_cube = self.moveit_control.go_to_pose_goal_quaternion(des_x, des_y, des_z+0.1, pick_orientation_quaternion[0], pick_orientation_quaternion[1], pick_orientation_quaternion[2], pick_orientation_quaternion[3])
            if success_place_cube:
                return True, "Cube placed"
            else:
                return False, "Could not place cube"
        else:
            return False, "Could not go to pre-place position"


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

    
        
        

if __name__ == '__main__':
    motion_planner_node = MotionPlannerNode()
    rospy.spin()