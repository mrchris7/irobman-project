#!/usr/bin/env python3

import rospy
import ros_numpy
import tf
import tf.transformations as tf_trans
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from enum import Enum
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, Point
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from irobman_project.srv import CartesianMotionPlanning, JointMotionPlanning, SetPoints, GetPoints, GetPoses, PickCube, PlaceCube
from pick_and_place_module.plan_scene import PlanScene
from typing import List
SIMULATION = 0

class State(Enum):
    START = 0
    MOVE_TO_INITIAL_POSE = 1
    LOCALISE_TARGET_CUBE = 2
    PICK_CUBE = 3
    PLACE_CUBE = 4
    END = 5


class Status(Enum):
    READY = 0
    RUNNING = 1
    FINISHED = 2   
    ERROR = 3


class StateMachine:
    
    def __init__(self):
        rospy.init_node('state_machine', anonymous=True)
        
        # motion planner
        self.cartesian_planning_client = rospy.ServiceProxy('motion_planner/CartesianMotionPlanning', CartesianMotionPlanning)
        self.joint_planning_client = rospy.ServiceProxy('motion_planner/JointMotionPlanning', JointMotionPlanning)
        self.close_gripper_client = rospy.ServiceProxy('motion_planner/GripperControl', SetBool)
        self.pick_cube_client = rospy.ServiceProxy('/motion_planner/PickCube', PickCube)
        self.place_cube_client = rospy.ServiceProxy('/motion_planner/PlaceCube', PlaceCube)
        
        # TODO: create the corresponsing rospy.Publisher inside the motion planning node and publish the eff-pose (alternatively, create a service)
        self.eef_pose_sub = rospy.Subscriber('/joint_states', JointState, self.handle_eff_pose)
        
        # cube detection
        self.detection_client = rospy.ServiceProxy('CubeDetection', GetPoints)
        
        # pose estimation
        self.toggle_tracker_client = rospy.ServiceProxy('pose_estimation/ToggleTracker', SetBool)
        self.prepare_tracker_client = rospy.ServiceProxy('pose_estimation/PrepareTracker', SetPoints)
        self.retrieve_tracked_poses_client = rospy.ServiceProxy('pose_estimation/RetrieveTrackedPoses', GetPoses)

        # setup
        self.init_joints = rospy.get_param('init_joints').values()
        self.pose_eef = np.array([])
        self.tower_height = 0
        self.max_tower_height = 0.8
        self.state = State.START
        self.status = Status.READY
        self.transition_data = None  # used to transfer data between states
        self.cube_number = 0
        self.cube_dimension = 0.045
        self.approach = 0

        # Planning Scene
        self.planning_scene = PlanScene()
        self.planning_scene.set_table()
        # self.planning_scene.set_env_constrains()


    def reset(self):
        self.pose_eef = np.array([])
        self.tower_height = 0
        self.state = State.START
        self.status = Status.READY
        self.transition_data = None
        self.cube_number = 0
        self.approach = 0
        


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
        
        elif state == State.PICK_CUBE:
            self.pick_cube(data)

        elif state == State.PLACE_CUBE:
            tower_pose = self.calculate_tower_pose()
            self.place_cube(tower_pose)

        elif state == State.END:
            self.status = Status.FINISHED

    
    def handle_eff_pose(self, state: JointState):
        pass # TODO remove subscriber too
        ## Find the index of the "panda_hand" joint
        #try:
        #
        #    panda_hand_index = state.name.index("panda_hand")
        #    panda_hand_position = state.position[panda_hand_index]
        #except ValueError:
        #    rospy.logwarn("Joint 'panda_hand' not found in the joint states message.")
        #
        #self.pose_eef = np.array(panda_hand_position[0:3])


    def choose_target_pose(self, poses: List[Pose]):
        # TODO: choose a target cube out of all detected cube poses
        #       i.e. return the pose of the cube that is nearest to the end-effector (self.pose_eef)
        rospy.logwarn(f"all poses: {poses}")
        best_pose = poses[0]  # store chosen pose
        psts_obj = np.zeros((len(poses), 3))
        for i, pose in enumerate(poses):
            # convert MessageType data to NumPy array 
            pst_obj = ros_numpy.numpify(pose)[0:3, 3] # only center of object
            psts_obj[i] = pst_obj

        # method 1: neaest to end effector
        
        # pst_eff = ros_numpy.numpify(self.pose_eef)[0:4, 3]
        pst_eef = self.get_transformation("panda_hand", "world")
        #pst_eef = self.pose_eef
        # compute mininum distance and corresponding index
        dist = np.linalg.norm(psts_obj - pst_eef, axis=1)
        ind_min = np.argmin(dist)
        best_pose = poses[ind_min]
        rospy.logwarn(f"choose pose with min distance: {best_pose}")
    
        # # method 2: least neighbours
        # threshold = 0.5  # meter
        # num_nb = len(poses) - 1
        # # count number of neighbours
        # for j in range(len(poses)):
        #     dist = np.linalg.norm(psts_obj - psts_obj[j], axis=1)
        #     diff = dist - threshold
        #     crr_nb = np.squeeze(np.nonzero(diff)).shape[0]
        #     if  crr_nb < num_nb:
        #         num_nb = crr_nb
        #         best_pose = poses[j]
        # rospy.logwarn(f"choose pose with min neighbours: {best_pose} with {num_nb} neighbour")

        return best_pose
    
    def get_transformation(self, frame_from, frame_to):
        listener = tf.TransformListener()  # TODO: move to constructor
        rate = rospy.Rate(10.0)
        counter = 0
        trans = None

        while not rospy.is_shutdown() and counter <= 5:
            try:
                (trans, rot) = listener.lookupTransform(frame_from, frame_to, rospy.Time(0))
                rospy.loginfo("Translation: %s", trans)
                rospy.loginfo("Rotation: %s", rot)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("repeat:", e)
            counter += 1
                
            rate.sleep()

        return np.array(trans)


    def calculate_tower_pose(self):
        # TODO: Calculate the pose where the cube should be placed on the tower based on self.tower_height
        if SIMULATION == 1:
            tower_pose_dict = rospy.get_param('tower1_sim')
        else:
            tower_pose_dict = rospy.get_param('tower1')
        tower_pose = Pose()
        tower_pose.position.x = tower_pose_dict['x']
        tower_pose.position.y = tower_pose_dict['y']
        tower_pose.position.z = tower_pose_dict['z'] + self.cube_number*self.cube_dimension
        tower_pose.orientation.x = tower_pose_dict['qx']
        tower_pose.orientation.y = tower_pose_dict['qy']
        tower_pose.orientation.z = tower_pose_dict['qz']
        tower_pose.orientation.w = tower_pose_dict['qw']


        return tower_pose
    
    def transform_poses(self, poses_camera: List[Pose]):
        # define frames for transformation
        frame_from = "zed2_left_camera_frame"
        frame_to = "world"

        listener = tf.TransformListener()  # TODO: move to constructor
        
        poses_world = []
        for pose_camera in poses_camera:

            pose_camera_st = PoseStamped()
            pose_camera_st.header.frame_id = frame_from
            pose_camera_st.header.stamp = rospy.Time(0)
            
            pos_old = ros_numpy.numpify(pose_camera)
            theta = -0.5*np.pi
            Tx =  np.array([[1,0,0,0],
                        [0,np.cos(theta), -np.sin(theta),0],
                        [0,np.sin(theta),np.cos(theta),0],
                        [0,0,0,1]])
            Tzx = np.array([[np.cos(theta), -np.sin(theta),0,0],
                        [np.sin(theta),np.cos(theta),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]) @ Tx  
            pos_new = Tzx @ pos_old
            pose_camera_st.pose = ros_numpy.msgify(Pose, pos_new)
            
            
            rate = rospy.Rate(10.0)
            counter = 0

            while not rospy.is_shutdown() and counter <= 5:
                try:
                    # Transform the point
                    pose_world_st = listener.transformPose(frame_to, pose_camera_st)

                    rospy.loginfo("Pose transformed to " + frame_to + ": (%f, %f, %f)",
                                    pose_world_st.pose.position.x,
                                    pose_world_st.pose.position.y,
                                    pose_world_st.pose.position.z)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    print("repeat:", e)
                counter += 1
                    
                rate.sleep()

            pose_world = Pose()
            pose_world = pose_world_st.pose
            poses_world.append(pose_world)
        
        return poses_world


    ### 1. INITIAL POSE ###
    def move_to_initial_pose(self):
        
        rospy.wait_for_service('motion_planner/JointMotionPlanning')

        js = JointState()
        js.position = list(self.init_joints)

        res = self.joint_planning_client(js)
        
        if not res.success:
            self.react_to_failure(res.message, self.state, Status.ERROR)
            return

        rospy.wait_for_service('motion_planner/GripperControl')

        res = self.close_gripper_client(False)

        if not res.success:
            self.react_to_failure(res.message, self.state, Status.ERROR)
            return

        if self.tower_height >= self.max_tower_height:
            self.state = State.END
        else:
            self.state = State.LOCALISE_TARGET_CUBE        
        


    ### 2. LOCALIZE TARGET CUBE ###
    def localise_target_cube(self):
        if not SIMULATION:

            rospy.wait_for_service('CubeDetection')
            res = self.detection_client()

            if not res.success:
                self.react_to_failure(res.message, State.MOVE_TO_INITIAL_POSE)
                return
            
            if len(res.points) == 0:
                # TODO: move to a another alternative initial pose (perhaps cubes can be detected there)
                self.react_to_failure("No cubes detected. The tower cannot be build.", self.state, Status.Error)
                return

            rospy.wait_for_service('pose_estimation/PrepareTracker')
            res = self.prepare_tracker_client(res.points)

            if not res.success:
                self.react_to_failure(res.message, State.MOVE_TO_INITIAL_POSE)
                return

            rospy.wait_for_service('pose_estimation/ToggleTracker')
            res = self.toggle_tracker_client(True)
            
            if not res.success:
                self.react_to_failure(res.message, State.MOVE_TO_INITIAL_POSE)
                return

            rospy.sleep(4)  # give tracker some time

            rospy.wait_for_service('pose_estimation/RetrieveTrackedPoses')
            res = self.retrieve_tracked_poses_client()
 
            if not res.success:
                self.react_to_failure(res.message, State.MOVE_TO_INITIAL_POSE)
                return

            # retrieved poses are body2camera poses -> transform to body2world poses
            print(">>>>> poses before transformation", res.poses)
            transformed_poses = self.transform_poses(res.poses)
            print(">>>>> poses after transformation", transformed_poses)
            
            
            target_pose = transformed_poses[0] #self.choose_target_pose(transformed_poses)

            # TODO: evaluate if poses are good, maybe filter before choosing the target?
            #       e.g. ignore poses that have negative z, or too high z coordinates

            # Planning Scene
            self.planning_scene.set_cube_env(transformed_poses) 

            # set z to align with table height
            target_pose.position.z = self.cube_dimension/2

        else:
            poses = []
            for i in range(4):
                poses.append(rospy.wait_for_message('/cube_%d_odom' % i,Odometry).pose.pose)
            self.planning_scene.set_cube_env(poses)                
            target_pose = rospy.wait_for_message('/cube_%d_odom' % self.cube_number,Odometry).pose.pose
        # go to next state
        self.transition_data = target_pose
        self.state = State.PICK_CUBE



    
    ### 3. PICK CUBE ###
    def pick_cube(self,pose):   
        rospy.wait_for_service('/motion_planner/PickCube')
        res = self.pick_cube_client(pose,self.approach)
        # TODO: check if cube is gripped inside motion planner node
        #       and if not res.success shold be False

        if not res.success:
            self.approach += 1
            if self.approach >= 6:
                self.react_to_failure(res.message, State.END, Status.ERROR)
            else:
                self.react_to_failure(res.message, State.MOVE_TO_INITIAL_POSE)
            return

        self.state = State.PLACE_CUBE




    
    ### 4. PLACE CUBE ###
    def place_cube(self,pose):
        
        rospy.wait_for_service('/motion_planner/PlaceCube')
        res = self.place_cube_client(pose)

        if not res.success:
            self.react_to_failure(res.message, State.MOVE_TO_INITIAL_POSE)
            return

        # TODO: check if placement of cube was correct
        self.cube_number += 1
        self.tower_height = self.cube_number*self.cube_dimension 

        self.state = State.MOVE_TO_INITIAL_POSE
    

    def react_to_failure(self, msg, state, status=None):
        if status is not None:
            self.status = status
        
        if status == Status.ERROR:
            rospy.logerr(msg)
        else:
            rospy.logwarn(msg)

        self.state = state


if __name__ == '__main__':
    
    state_machine_node = StateMachine()
    state_machine_node.execute()

    rospy.spin()
