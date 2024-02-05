import rospy

import moveit_commander
import geometry_msgs.msg
from pick_and_place_module.pick_and_place import PickAndPlace
from pick_and_place_module.eef_control import MoveGroupControl
from pick_and_place_module.plan_scene import PlanScene

class GenerateStructure:
    def __init__(self):
        self.structure = None
        self.pick_and_place = PickAndPlace(0.05, 0.5)
        self.moveit_control = MoveGroupControl()
        self.plan_scene = PlanScene()
    
   



    def simple_pyramid(self,x,y,z): # Creates an simple 3 cube pyramid array based on the base point. 
        cube_dimension = 0.05
        self.structure = [[x-(0.01+(cube_dimension/2)),y,z+(cube_dimension/2)],[x+(0.01+(cube_dimension/2)),y,z+(cube_dimension/2)],[x,y,z+(3*cube_dimension/2)]]
        #self.structure = [[x,y-(0.01+(cube_dimension/2)),z+(cube_dimension/2)],[x,y+(0.01+(cube_dimension/2)),z+(cube_dimension/2)],[x,y,z+(3*cube_dimension/2)]]
    def observeTask(self):
        move_group = self.moveit_control
        move_group.go_to_pose_goal(0.3, 0,1.2,0.785,3.14,0)
        rospy.sleep(3)
    def moveTask(self):
        self.plan_scene.set_envirorment()
        self.observeTask()
        if self.structure:
            for position_id in range(len(self.structure)):
                pick_position=[rospy.get_param("cube_"+str(position_id)+"_x"),rospy.get_param("cube_"+str(position_id)+"_y"),rospy.get_param("cube_"+str(position_id)+"_z")+0.05]
                yaw = rospy.get_param("cube_"+str(position_id)+"_orient_w")
                pick_orientation=[0,3.14,0]
                place_orientation=[0,3.14,0]
                place_position=self.structure[position_id]
                self.pick_and_place.setPickPose(*pick_position,*pick_orientation)
                self.pick_and_place.setDropPose(*place_position,*place_orientation)
                self.pick_and_place.setGripperPose(0.045)
                self.pick_and_place.setGripperParams(0.1,10,0.005,0.008) # Speed, Force, ep_inner,ep_outer
                self.pick_and_place.execute_pick_and_place()
                #self.pick_and_place.execute_cartesian_pick_and_place()
                self.observeTask()
                

    
