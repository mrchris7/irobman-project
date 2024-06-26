#!/usr/bin/env python3

import rospy
from pick_and_place_module.pick_and_place import PickAndPlace
from pick_and_place_module.plan_scene import PlanScene
from pick_and_place_module.generate_structure import GenerateStructure
from tf.transformations import euler_from_quaternion
from pick_and_place_module.eef_control import MoveGroupControl

def task():

    gen = GenerateStructure()
    gen.simple_pyramid(0.5,0.5,0.9)
    gen.moveTask()
'''
    pick_and_place = PickAndPlace(0.05, 0.5)
    pick_position=[rospy.get_param("cube_0_x"),rospy.get_param("cube_0_y"),rospy.get_param("cube_0_z")+0.05]
    yaw = rospy.get_param("cube_0_orient_w")
    (roll, pitch, yaw) = euler_from_quaternion([rospy.get_param("cube_0_orient_x"),rospy.get_param("cube_0_orient_y"),rospy.get_param("cube_0_orient_z"),rospy.get_param("cube_0_orient_w")])
    print(yaw)
    
    pick_orientation=[yaw,3.14,0]

    place_position=[rospy.get_param("cube_0_x")+0.05,rospy.get_param("cube_0_y")+0.2,rospy.get_param("cube_0_z")+0.1]
    place_orientation=[1.57,3.14,0]
    
    pick_and_place.setPickPose(*pick_position,*pick_orientation)
    pick_and_place.setDropPose(*place_position,*place_orientation)
    pick_and_place.setGripperPose(0.045)
    pick_and_place.setGripperParams(0.1,3,0.005,0.008) # Speed, Force, ep_inner,ep_outer
    pick_and_place.execute_cartesian_pick_and_place()
    #pick_and_place.execute_pick_and_place()
'''
def test():
    plan_scene = PlanScene()
    plan_scene.set_envirorment()
    pick_and_place = PickAndPlace(0.05, 0.5)
    pick_position=[0.5,0.5,0.9]
    pick_orientation=[0,3.14,0]
    place_orientation=[0,3.14,0]
    place_position=[0.5,1,0.9]
    pick_and_place.setPickPose(*pick_position,*pick_orientation)
    pick_and_place.setDropPose(*place_position,*place_orientation)
    pick_and_place.setGripperPose(0.045)
    pick_and_place.setGripperParams(0.1,10,0.005,0.008) # Speed, Force, ep_inner,ep_outer
    pick_and_place.execute_pick_and_place()

def test2():
    plan_scene = PlanScene()
    plan_scene.set_envirorment()
    moveit_control = MoveGroupControl()
    moveit_control.go_to_pose_goal(0.5, 0.5,1,0.785,3.14,0)


if __name__ == "__main__":
    test2()
