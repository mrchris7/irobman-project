import rospy
from enum import Enum
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from irobman_project.srv import CartesianMotionPlanning, JointMotionPlanning, SetPoints, GetPoints, GetPoses
from typing import List


class State(Enum):
    START = 0
    MOVE_TO_INITIAL_POSE = 1
    LOCALISE_TARGET_CUBE = 2
    MOVE_TO_CUBE = 3
    PICK_CUBE = 4
    MOVE_TO_TOWER = 5
    PLACE_CUBE = 6
    END = 7


class Status(Enum):
    READY = 0
    RUNNING = 1
    FINISHED = 2   
    ERROR = 3


class StateMachine:
    
    def __init__(self):
        rospy.init_node('state_machine', anonymous=True)
        
        # motion planner
        self.cartesian_planning_client = rospy.ServiceProxy('CartesianMotionPlanning', CartesianMotionPlanning)
        self.joint_planning_client = rospy.ServiceProxy('JointMotionPlanning', JointMotionPlanning)
        self.close_gripper_client = rospy.ServiceProxy('GripperControl', SetBool)
        
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
        self.pose_eef = Pose()
        self.tower_height = 0
        self.max_tower_height = 2
        self.state = State.START
        self.status = Status.READY
        self.transition_data = None  # used to transfer data between states


    def reset(self):
        self.pose_eef = Pose()
        self.tower_height = 0
        self.state = State.START
        self.status = Status.READY
        self.transition_data = None


    # called from the outside
    def execute(self):
        rospy.loginfo("Starting the task.")

        while not rospy.is_shutdown():
            
            if (self.status == Status.ERROR):
                rospy.loginfo("Task stopped in state " + self.state.name + " due to an error.")
                break

            elif (self.status == Status.FINISHED):
                rospy.loginfo("Task finished.")
                break

            elif (self.status == Status.READY 
                  or self.status == Status.RUNNING):
                self.transition_to(self.state)
            
            rospy.sleep(0.1)  # not necessary


    # call it only from execute
    def transition_to(self, state):
        rospy.loginfo("Transition to state " + state.name)

        data = self.transition_data
        self.transition_data = None

        if state == State.START:
            self.state = State.MOVE_TO_INITIAL_POSE
            self.status = Status.RUNNING

        elif state == State.MOVE_TO_INITIAL_POSE:
            self.move_to_initial_pose()
        
        elif state == State.LOCALISE_TARGET_CUBE:
            self.localise_target_cube()

        elif state == State.MOVE_TO_CUBE:
            self.move_to_cube(data)
        
        elif state == State.PICK_CUBE:
            self.pick_cube()

        elif state == State.MOVE_TO_TOWER:
            self.move_to_tower()

        elif state == State.PLACE_CUBE:
            self.place_cube()

        elif state == State.END:
            self.status = Status.FINISHED

    
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
        
        rospy.wait_for_service('JointMotionPlanning')

        js = JointState()
        js.name = list(self.init_joints.keys())
        js.position = list(self.init_joints.values())

        res = self.joint_planning_client(js)
        
        if not res.success:
            self.error_handling(res.message)
            return
        

        if self.tower_height == self.max_tower_height:
            self.state = State.END
        else:
            self.state = State.LOCALISE_TARGET_CUBE        
        


    ### 2. LOCALIZE TARGET CUBE ###
    def localise_target_cube(self):

        rospy.wait_for_service('CubeDetection')
        res = self.detection_client()

        if not res.success:
            self.error_handling(res.message)
            return
        
        if len(res.points) == 0:
            self.error_handling("No cubes detected. The tower cannot be build.")

        rospy.wait_for_service('PrepareTracker')
        res = self.prepare_tracker_client(res.points)

        if not res.success:
            self.error_handling(res.message)
            return

        rospy.wait_for_service('ToggleTracker')
        res = self.toggle_tracker_client(True)
        
        if not res.success:
            self.error_handling(res.message)
            return
        
        rospy.sleep(3)  # give tracker some time

        rospy.wait_for_service('RetrieveTrackedPoses')
        res = self.retrieve_tracked_poses_client()
        
        if not res.success:
            self.state = State.MOVE_TO_INITIAL_POSE
            return

        # TODO: evaluate if poses are good, maybe filter before choosing the target?
        #       e.g. ignore poses that have negative z, or too high z coordinates

        target_pose = self.choose_target_pose(res.poses)
        
        # go to next state
        self.transition_data = target_pose
        self.state = State.MOVE_TO_CUBE


    ### 3. MOVE TO CUBE ###
    def move_to_cube(self, pose: Pose):
        
        # TODO: check if gripper is already closed -> if so, open it first
        #       add a service "IsGripperClosed" to the motion planner node (create GetBool.srv)

        rospy.wait_for_service('CartesianMotionPlanning')
        res = self.cartesian_planning_client(pose)  # TODO: responsible for a good grapsing-strategy

        if not res.success:
            self.state = State.MOVE_TO_INITIAL_POSE
            return

        self.state = State.PICK_CUBE

    
    ### 4. PICK CUBE ###
    def pick_cube(self):
                
        rospy.wait_for_service('GripperControl')
        res = self.close_gripper_client(True)
        # TODO: check if cube is gripped inside motion planner node
        #       and if not res.success shold be False

        if not res.success:
            self.state = State.MOVE_TO_INITIAL_POSE
            return

        self.state = State.MOVE_TO_TOWER



    ### 5. MOVE TO TOWER ###
    def move_to_tower(self):

        tower_pose = self.calculate_tower_pose()

        rospy.wait_for_service('CartesianMotionPlanning')
        res = self.cartesian_planning_client(tower_pose)

        if not res.success:
            self.state = State.MOVE_TO_INITIAL_POSE
            return

        self.state = State.PLACE_CUBE


    
    ### 6. PLACE CUBE ###
    def place_cube(self):

        rospy.wait_for_service('GripperControl')
        res = self.close_gripper_client(False)

        if not res.success:
            self.state = State.MOVE_TO_INITIAL_POSE
            return

        # TODO: check if placement of cube was correct
        self.tower_height += 1

        self.state = State.MOVE_TO_INITIAL_POSE
    


    ### ERROR HANDLING ###
    def error_handling(self, err_msg):
        rospy.logerr(err_msg)
        self.status = Status.ERROR


if __name__ == '__main__':
    
    state_machine_node = StateMachine()
    state_machine_node.execute()

    rospy.spin()
