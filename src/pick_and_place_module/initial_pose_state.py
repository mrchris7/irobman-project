from sensor_msgs.msg import JointState
import rospy

class InitialPose(State):

    def __init__(self, name):
        initial_pose = JointState()
        initial_pose.position = [0.759644, 0.0178955, -0.814945, -1.79299, -0.0223154, 1.358, 0.752333]

    def enter():
        pass

    def step():
        pass


    def exit():
        pass
        


