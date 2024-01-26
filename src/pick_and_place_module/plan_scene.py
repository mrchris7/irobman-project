import rospy

import moveit_commander
import geometry_msgs.msg

class PlanScene:
    def __init__(self):
        self.scene = moveit_commander.PlanningSceneInterface()

    def set_table(self,width=0.81,lenght=1.49,height=0.787,x=0.495000,y=0,z=0.393500):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_link0"
        box_pose.pose.orientation.w = 0
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z-height
        self.scene.add_box("table1", box_pose, size=(width, lenght, height))
        

    
    def set_cubes(self):
        for index in range(30):
            #print("cube_"+str(index)+"_x")
            if rospy.has_param("cube_"+str(index)+"_x"):
                box_pose = geometry_msgs.msg.PoseStamped()
                #print(index)
                box_pose.header.frame_id = "panda_link0"
                box_pose.pose.orientation.w = rospy.get_param("cube_"+str(index)+"_orient_w")
                box_pose.pose.orientation.x = rospy.get_param("cube_"+str(index)+"_orient_x")
                box_pose.pose.orientation.y = rospy.get_param("cube_"+str(index)+"_orient_y")
                box_pose.pose.orientation.z = rospy.get_param("cube_"+str(index)+"_orient_z")
                box_pose.pose.position.x = rospy.get_param("cube_"+str(index)+"_x")
                box_pose.pose.position.y = rospy.get_param("cube_"+str(index)+"_y")
                box_pose.pose.position.z = rospy.get_param("cube_"+str(index)+"_z")-0.787
                self.scene.add_box("cube_"+str(index), box_pose, size=(0.045, 0.045, 0.045))
    def set_cube(self,cube_index):
        if rospy.has_param("cube_"+str(cube_index)+"_x"):
                box_pose = geometry_msgs.msg.PoseStamped()
                box_pose.header.frame_id = "panda_link0"
                box_pose.pose.orientation.w = rospy.get_param("cube_"+str(cube_index)+"_orient_w")
                box_pose.pose.orientation.x = rospy.get_param("cube_"+str(cube_index)+"_orient_x")
                box_pose.pose.orientation.y = rospy.get_param("cube_"+str(cube_index)+"_orient_y")
                box_pose.pose.orientation.z = rospy.get_param("cube_"+str(cube_index)+"_orient_z")
                box_pose.pose.position.x = rospy.get_param("cube_"+str(cube_index)+"_x")
                box_pose.pose.position.y = rospy.get_param("cube_"+str(cube_index)+"_y")
                box_pose.pose.position.z = rospy.get_param("cube_"+str(cube_index)+"_z")-0.787
                self.scene.add_box("cube_"+str(cube_index), box_pose, size=(0.045, 0.045, 0.045))


    def set_envirorment(self):
        self.set_table(0.81,1.49,0.787,0.495000,0,0.393500)
        self.set_cubes()
        #print(self.scene.get_known_object_names())




