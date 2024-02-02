from enum import Enum
import rospy
from your_package.srv import PlanMotion, DetectCube, GripperControl
from sensor_msgs.msg import JointState


class State(Enum):
    WAIT = 0
    INITIAL_POSE = 1
    AT_CUBE = 2
    AT_CUBE_PICKED = 3
    AT_TOWER = 4
    AT_TOWER_RELEASED = 5


INITIAL_JOINT_STATE = JointState()
INITIAL_JOINT_STATE.position = [0.759644, 0.0178955, -0.814945, -1.79299, -0.0223154, 1.358, 0.752333]

class StateMachine:
    
    def __init__(self):
        rospy.init_node('state_machine', anonymous=True)
        self.motion_planning_client = rospy.ServiceProxy('/plan_motion', PlanMotion)
        self.gripper_client = rospy.ServiceProxy('/control_gripper', GripperControl)
        self.cube_detection_client = rospy.ServiceProxy('/detect_cube', DetectCube)
        
        self.pose_estimation_enabled = False
        self.state = State.WAIT



    ### 1. INITIAL POSE ###
    def move_to_initial_pose_cb(self, response):
        self.state = State.INITIAL_POSE
        self.detect_cube()

    def move_to_initial_pose(self):
        rospy.wait_for_service('plan_motion')
        self.motion_planning_client(INITIAL_JOINT_STATE, self.move_to_initial_pose_cb)



    ### 2. DETECT CUBE ###
    def detect_cube_cb(self, response):
        if response.message == "success":
            position = extract_target_cube_position(response)
            self.move_to_cube(self, position)
        else:
            self.move_to_initial_pose()

    def detect_cube(self):
        rospy.wait_for_service('detect_cube')
        self.cube_detection_client(self.detect_cube_cb)


    ### 3. MOVE TO CUBE ###
    def move_to_cube_cb(self, response):
        self.state = State.AT_CUBE
        self.pick_cube()


    def move_to_cube(self, position):
        rospy.wait_for_service('plan_motion')
        self.motion_planning_client(position, self.move_to_cube_cb)


    
    ### 4. PICK CUBE ###
    def pick_cube_cb(self, response):
        self.state = State.AT_CUBE_PICKED
        self.pick_cube()


    def pick_cube(self, position):
        rospy.wait_for_service('control_gripper')
        self.gripper_client("CLOSE", self.pick_cube_cb)



    ### 5. PLACE CUBE ###
    # TODO



if __name__ == '__main__':
    state_machine_node = StateMachine()
    rospy.spin()