import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_commander.conversions import pose_to_list
from copy import deepcopy
from math import pi, fabs, sqrt, cos
from sensor_msgs.msg import JointState




class StateMachine:

    def __init__(self):
        rospy.init_node("state_machine", anonymous=True)
        self.tower_state = 0
        self.state = 0

        pub = rospy.Publisher('state', int32, queue_size=10)

    def move_eff_to(Pose pose):
        return
    
    def control_gripper(open):
        return


    def loop():
        # C - Statemachine
        
        while (self.tower_state < 5):

            # 1. go intial pose

            # C - 2. detected cube?

            # E - 3. pick target cube

            # E - 4. holding cube? if not go to initial pose

            # E - 5. place target cube (goal_position is based on tower_state)

            # K - 6. check if correctly placed cube -> if yes, increase tower_state

    def execute_state(State s):

        s.enter()
        
        while(not s.finished):
            s.step()
        
        result = s.exit()


        

    