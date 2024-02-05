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
        
    def set_env_constrains(self,width=0.81,lenght=1.49,height=0.787,x=0.495000,y=0,z=0.39350):
        boxA = geometry_msgs.msg.PoseStamped()
        boxA.header.frame_id = "panda_link0"
        boxA.pose.orientation.w = 0
        boxA.pose.position.x = (width/2)
        boxA.pose.position.y = lenght/2
        boxA.pose.position.z = z
        self.scene.add_box("env1", boxA, size=(lenght, 0.01, 1))
        boxB = geometry_msgs.msg.PoseStamped()
        boxB.header.frame_id = "panda_link0"
        boxB.pose.orientation.w = 0
        boxB.pose.position.x = (width/2)+x
        boxB.pose.position.y = 0
        boxB.pose.position.z = z
        self.scene.add_box("env2", boxB, size=(0.01, lenght, 1))
        boxC = geometry_msgs.msg.PoseStamped()
        boxC.header.frame_id = "panda_link0"
        boxC.pose.orientation.w = 0
        boxC.pose.position.x = (width/2)
        boxC.pose.position.y = -lenght/2
        boxC.pose.position.z = z
        self.scene.add_box("env3", boxC, size=(lenght, 0.01, 1))
        boxD = geometry_msgs.msg.PoseStamped()
        boxD.header.frame_id = "panda_link0"
        boxD.pose.orientation.w = 0
        boxD.pose.position.x = -0.6
        boxD.pose.position.y = 0
        boxD.pose.position.z = z
        self.scene.add_box("env4", boxD, size=(0.01, lenght, 1))
    
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
        self.set_env_constrains()
        #self.set_cubes()
        #print(self.scene.get_known_object_names())




