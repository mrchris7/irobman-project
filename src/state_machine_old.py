import rospy
from enum import Enum
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from srv import CartesianMotionPlanning, JointMotionPlanning, SetPoints, GetPoints, GetPoses
from typing import List


class State(Enum):
    AT_UNDEFINED = 0
    AT_INIT_POSE = 1
    AT_CUBE = 2
    AT_CUBE_PICKED = 3
    AT_TOWER = 4
    AT_TOWER_RELEASED = 5


class StateMachine:
    
    def __init__(self):
        rospy.init_node('state_machine', anonymous=True)
        
        # motion planner
        self.cartesian_planning_client = rospy.ServiceProxy('CartesianMotionPlanning', CartesianMotionPlanning)
        self.joint_planning_client = rospy.ServiceProxy('JointMotionPlanning', JointMotionPlanning)
        self.close_gripper_client = rospy.ServiceProxy('ControlGripper', SetBool)  # TODO: rename to 'CloseGripper' (so it is more clear what the data in SetBool does)
        
        # TODO: create the corresponsing rospy.Publisher inside the motion planning node and publish the eff-pose (alternatively, create a service)
        self.eef_pose_sub = rospy.Subscriber('eff_pose', Pose, self.handle_eff_pose)
        
        # cube detection
        self.detection_client = rospy.ServiceProxy('CubeDetection', GetPoints)
        
        # pose estimation
        self.toggle_tracker_client = rospy.ServiceProxy('ToggleTracker', SetBool)
        self.prepare_tracker_client = rospy.ServiceProxy('PrepareTracker', SetPoints)
        self.retrieve_tracked_poses_client = rospy.ServiceProxy('RetrieveTrackedPoses', GetPoses)

        # setup
        self.init_joints = rospy.get_param('init_joints')
        self.state = State.AT_UNDEFINED
        self.pose_eef = Pose()
        self.tower_height = 0


    # called from the outside
    def start(self):
        rospy.loginfo("Starting the task.")
        
        # TODO: problem: every action is a new function-call which goes "infinitely" deep
        self.move_to_initial_pose()

    
    def handle_eff_pose(self, pose: Pose):
        self.pose_eff = pose


    def choose_target_pose(self, poses: List[Pose]):
        # TODO: choose a target cube out of all detected cube poses
        #       i.e. return the pose of the cube that is nearest to the end-effector (self.pose_eff)
        return poses[0]
    

    def calculate_tower_pose(self):
        # TODO: Calculate the pose where the cube should be placed on the tower based on self.tower_height
        tower_pose = Pose()
        return tower_pose


    ### 1. INITIAL POSE ###
    def move_to_initial_pose(self):
        
        # precondition
        if self.tower_height == self.max_tower_height:
            rospy.loginfo("Tower successfully build.")
            return

        if self.state == State.AT_INIT_POSE:
            self.localise_target_cube()
            return

        # execution
        rospy.wait_for_service('JointMotionPlanning')

        js = JointState()
        js.name = list(self.init_joints.keys())
        js.position = list(self.init_joints.values())

        res = self.joint_planning_client(js)
        
        # transition
        if not res.success:
            self.error_handling(res.msg)
            return
            
        self.state = State.AT_INIT_POSE
        self.localise_target_cube()
        


    ### 2. LOCALIZE TARGET CUBE ###
    def localise_target_cube(self):

        rospy.wait_for_service('CubeDetection')
        res = self.detection_client()

        if not res.success:
            self.move_to_initial_pose()
            return
        
        if len(res.points) == 0:
            rospy.INFO("No cubes detected. The tower can not be finished.")

        rospy.wait_for_service('PrepareTracker')
        res = self.prepare_tracker_client(res.points)

        if not res.success:
            self.move_to_initial_pose()
            return

        rospy.wait_for_service('ToggleTracker')
        res = self.toggle_tracker_client(True)
        
        if not res.success:
            self.move_to_initial_pose()
            return
        
        rospy.sleep(3)  # give tracker some time

        rospy.wait_for_service('RetrieveTrackedPoses')
        res = self.retrieve_tracked_poses_client()
        
        if not res.success:
            self.move_to_initial_pose()
            return

        # TODO: evaluate if poses are good, maybe filter before choosing the target?
        #       e.g. ignore poses that have negative z, or too high z coordinates

        target_pose = self.choose_target_pose(res.poses)
        self.move_to_cube(target_pose)


    ### 3. MOVE TO CUBE ###
    def move_to_cube(self, pose: Pose):
        
        # TODO: check if gripper is already closed -> if so, open it first
        #       add a service "IsGripperClosed" to the motion planner node (create GetBool.srv)

        rospy.wait_for_service('CartesianMotionPlanning')
        res = self.cartesian_planning_client(pose)  # TODO: responsible for a good grapsing-strategy

        if not res.success:
            self.move_to_initial_pose()
            return

        self.state = State.AT_CUBE
        self.pick_cube()

    
    ### 4. PICK CUBE ###
    def pick_cube(self):
                
        rospy.wait_for_service('GripperControl')
        res = self.close_gripper_client(True)
        # TODO: check if cube is gripped inside motion planner node
        #       and if not res.success shold be False

        if not res.success:
            self.move_to_initial_pose()
            return

        self.state = State.AT_CUBE_PICKED
        self.move_to_tower()



    ### 5. MOVE TO TOWER ###
    def move_to_tower(self):

        tower_pose = self.calculate_tower_pose()

        rospy.wait_for_service('CartesianMotionPlanning')
        res = self.cartesian_planning_client(tower_pose)

        if not res.success:
            self.move_to_initial_pose()
            return

        self.state = State.AT_TOWER
        self.place_cube()


    
    ### 6. PLACE CUBE ###
    def place_cube(self):

        rospy.wait_for_service('GripperControl')
        res = self.close_gripper_client(False)

        if not res.success:
            self.move_to_initial_pose()
            return

        # TODO: check if placement of cube was correct
        self.tower_height += 1

        self.state = State.AT_TOWER_RELEASED
        self.move_to_initial_pose()    
    


    ### ERROR HANDLING ###
    def error_handling(self, err_msg):
        rospy.logerr(err_msg)
        rospy.loginfo("Stopped the state machine.")
        self.state = State.AT_UNDEFINED


if __name__ == '__main__':
    state_machine_node = StateMachine()
    state_machine_node.start()

    rospy.spin()