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


class DemoStateMachine:
    
    def __init__(self):
        rospy.init_node('state_machine', anonymous=True)
        
        # motion planner
        self.cartesian_planning_client = rospy.ServiceProxy('CartesianMotionPlanning', CartesianMotionPlanning)
        self.joint_planning_client = rospy.ServiceProxy('JointMotionPlanning', JointMotionPlanning)
        self.close_gripper_client = rospy.ServiceProxy('GripperControl', SetBool)
        
        self.eef_pose_sub = rospy.Subscriber('eff_pose', Pose, self.handle_eff_pose)
        
        # cube detection
        self.detection_client = rospy.ServiceProxy('CubeDetection', GetPoints)
        
        # pose estimation
        self.toggle_tracker_client = rospy.ServiceProxy('ToggleTracker', SetBool)
        self.prepare_tracker_client = rospy.ServiceProxy('PrepareTracker', SetPoints)
        self.retrieve_tracked_poses_client = rospy.ServiceProxy('RetrieveTrackedPoses', GetPoses)

        # setup
        self.init_joints = [0.759644,0.0178955,-0.814945,-1.79299,-0.0223154,1.358,0.752333]
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
        js = JointState()
        if self.tower_height == 0:
            js.position = [-0.330565,0.660273,-0.486617,-2.09717,0.539462,2.63023,-0.435691]
        
        elif self.tower_height == 1:
            js.position = [0.0449299,0.54133,-0.254465,-2.21104,0.168812,2.69281,0.347727]
        return js
    
    def choose_target_pose_pre(self, poses: List[Pose]):
        js = JointState()
        if self.tower_height == 0:
            js.position = [-0.30624,0.505797,-0.542583,-2.13841,0.538941,2.54162,-0.49178]
        
        elif self.tower_height == 1:
            js.position = [0.0677275,0.338532,-0.295676,-2.27223,0.12365,2.61353,0.342853]
        return js
    

    def calculate_tower_pose(self):
        js = JointState()
        if self.tower_height == 0:
            js.position = [1.04654,1.21175,-0.46713,-0.988193,0.455717,2.1506,1.42862]
        
        elif self.tower_height == 1:
            js.position = [1.06966,1.15498,-0.491592,-1.0145,0.428463,2.12603,1.50089]
        return js
    
    def calculate_tower_pose_pre(self):
        js = JointState()
        if self.tower_height == 0:
            js.position = [1.09169,1.10297,-0.535518,-1.0093,0.424635,2.02258,1.42981]
        
        elif self.tower_height == 1:
            js.position = [1.06946,1.05264,-0.515923,-1.01394,0.425934,2.00084,1.40731]
        return js


    ### 1. INITIAL POSE ###
    def move_to_initial_pose(self):
        
        rospy.wait_for_service('JointMotionPlanning')

        js = JointState()
        js.position = self.init_joints

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
        
        # CUBE DETECTION -> skip

        # POSE ESTIMATION -> skip

        self.transition_data = self.choose_target_pose_pre([])
        self.state = State.MOVE_TO_CUBE


    ### 3. MOVE TO CUBE ###
    def move_to_cube(self, pre_pose):

        rospy.wait_for_service('JointMotionPlanning')
        res = self.joint_planning_client(pre_pose)

        if not res.success:
            self.state = State.MOVE_TO_INITIAL_POSE
            return
        

        pose = self.choose_target_pose([])

        rospy.wait_for_service('JointMotionPlanning')
        res = self.joint_planning_client(pose)

        if not res.success:
            self.state = State.MOVE_TO_INITIAL_POSE
            return

        self.state = State.PICK_CUBE

    
    ### 4. PICK CUBE ###
    def pick_cube(self):
                
        rospy.wait_for_service('GripperControl')
        res = self.close_gripper_client(True)

        if not res.success:
            self.state = State.MOVE_TO_INITIAL_POSE
            return

        self.state = State.MOVE_TO_TOWER



    ### 5. MOVE TO TOWER ###
    def move_to_tower(self):

        # pre position
        tower_joints = self.calculate_tower_pose_pre()

        rospy.wait_for_service('JointMotionPlanning')
        res = self.joint_planning_client(tower_joints)

        if not res.success:
            self.state = State.MOVE_TO_INITIAL_POSE
            return
        
        # final position
        tower_joints = self.calculate_tower_pose()

        rospy.wait_for_service('JointMotionPlanning')
        res = self.joint_planning_client(tower_joints)

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

        self.tower_height += 1

        self.state = State.MOVE_TO_INITIAL_POSE
    


    ### ERROR HANDLING ###
    def error_handling(self, err_msg):
        rospy.logerr(err_msg)
        self.status = Status.ERROR


if __name__ == '__main__':
    
    state_machine_node = DemoStateMachine()
    state_machine_node.execute()

    rospy.spin()
