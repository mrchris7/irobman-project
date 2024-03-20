#!/usr/bin/env python3

import rospy
import ros_numpy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from enum import Enum
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from irobman_project.srv import CartesianMotionPlanning, JointMotionPlanning, SetPoints, GetPoints, GetPoses, PickCube, PlaceCube
from pick_and_place_module.plan_scene import PlanScene
from pick_and_place_module.eef_control import MoveGroupControl
from typing import List
from tower_builder import TowerBuilder

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
        self.move_group = MoveGroupControl().move_group
        self.eef_pose = self.move_group.get_current_pose().pose
        
        # cube detection
        self.detection_client = rospy.ServiceProxy('CubeDetection', GetPoints)
        
        # pose estimation
        self.toggle_tracker_client = rospy.ServiceProxy('pose_estimation/ToggleTracker', SetBool)
        self.prepare_tracker_client = rospy.ServiceProxy('pose_estimation/PrepareTracker', SetPoints)
        self.retrieve_tracked_poses_client = rospy.ServiceProxy('pose_estimation/RetrieveTrackedPoses', GetPoses)

        # setup
        self.init_joints = rospy.get_param('init_joints').values()
        self.pose_eef = np.array([])
        self.state = State.START
        self.status = Status.READY
        self.transition_data = None  # used to transfer data between states
        self.cube_dimension = 0.045
        self.approach = 0

        # tower builder
        if SIMULATION == 1:
            tower_template = rospy.get_param('tower_sim')
        else:
            tower_template = rospy.get_param('tower')
        self.tower_builder = TowerBuilder(tower_template, self.cube_dimension)

        # Planning Scene
        self.planning_scene = PlanScene()
        self.planning_scene.set_table()
        # self.planning_scene.set_env_constrains()


    def reset(self):
        self.pose_eef = np.array([])
        self.state = State.START
        self.status = Status.READY
        self.transition_data = None
        self.approach = 0
        self.tower_builder.reset()
        


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
            tower_pose = self.tower_builder.get_next_pose()
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
        # choose a target cube out of all detected cube poses
        # return the pose of the cube that is nearest to the end-effector
        rospy.logwarn(f"all poses: {poses}")
        best_pose = poses[0]  # store chosen pose
        psts_obj = np.zeros((len(poses), 3))
        for i, pose in enumerate(poses):
            # convert MessageType data to NumPy array 
            pst_obj = ros_numpy.numpify(pose)[0:3, 3] # only center of object
            psts_obj[i] = pst_obj

        # method 1: neaest to end effector
        self.eef_pose = self.move_group.get_current_pose().pose
        pst_eef = ros_numpy.numpify(self.eef_pose)[0:3, 3]
        print(f"{self.eef_pose = }")
        print(pst_eef.shape)
        # pst_eef = self.get_transformation("panda_hand", "world")
        # compute mininum distance and corresponding index
        dist = np.linalg.norm(psts_obj - pst_eef, axis=1)
        ind_min = np.argmin(dist)
        best_pose = poses[ind_min]
        rospy.logwarn(f"choose pose with min distance: {best_pose}")
    
        # # method 2: least neighbours
        # threshold = 0.15  # meter
        # num_nb = len(poses) - 1
        # # count number of neighbours
        # for j in range(len(poses)):
        #     dist = np.linalg.norm(psts_obj - psts_obj[j], axis=1)
        #     diff = dist < threshold
        #     crr_nb = np.count_nonzero(diff)
        #     if  crr_nb < num_nb:
        #         num_nb = crr_nb
        #         best_pose = poses[j]
        # rospy.logwarn(f"choose pose with min neighbours: {best_pose} with {num_nb} neighbour")

        # # method 3: smallest distance to its closest neighbor
        # max_min_distance = 0
        # for i in range(len(poses)):
        # 
        #     # Compute Euclidean distance between the current pose and all other poses
        #     dist = np.linalg.norm(psts_obj - psts_obj[i], axis=1)
        # 
        #     # Exclude distance to the current pose itself
        #     dist[i] = float('inf')
        # 
        #     # Find the minimum distance for the current pose
        #     current_min_distance = np.min(dist)
        # 
        #     # Update the best pose if the current pose has a larger minimum distance
        #     if current_min_distance > max_min_distance:
        #         max_min_distance = current_min_distance
        #         best_pose = poses[i]

        return best_pose
    
    def get_transformation(self, frame_from, frame_to):
        listener = tf.TransformListener()  # TODO: move to constructor

        listener.waitForTransform(frame_from, frame_to, rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform(frame_from, frame_to, rospy.Time(0))
        #rospy.loginfo("Translation: %s", trans)
        #rospy.loginfo("Rotation: %s", rot)

        return np.array(trans)

    
    def transform_poses(self, poses_camera: List[Pose]):
        # define frames for transformation
        frame_from = "zed2_left_camera_frame"
        frame_to = "world"

        listener = tf.TransformListener()  # TODO: move to constructor

        listener.waitForTransform(frame_from, frame_to, rospy.Time(), rospy.Duration(4.0))
        
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
            

            pose_world_st = listener.transformPose(frame_to, pose_camera_st)

            rospy.loginfo("Pose transformed to " + frame_to + ": (%f, %f, %f)",
                            pose_world_st.pose.position.x,
                            pose_world_st.pose.position.y,
                            pose_world_st.pose.position.z)

            pose_world = Pose()
            pose_world = pose_world_st.pose
            # adjust tiny unstablility on orientation
            orientation_list = [pose_world.orientation.x, pose_world.orientation.y, pose_world.orientation.z, pose_world.orientation.w]
            (x_rot, y_rot, z_rot) = euler_from_quaternion(orientation_list)
            #if np.isclose(x_rot, 0, atol=1e-02):
            x_rot = 0

            #if np.isclose(y_rot, 0, atol=1e-02):
            y_rot = 0
            #if np.isclose(z_rot, 0, atol=1e-02):
            #    z_rot = 0
            (pose_world.orientation.x, pose_world.orientation.y, pose_world.orientation.z, pose_world.orientation.w) = quaternion_from_euler(x_rot, y_rot, z_rot)
            poses_world.append(pose_world)
        
        return poses_world


    def filter_poses(self, poses: List[Pose]):
        # filter after pose estimation
        return [pose for pose in poses if pose.position.y > -0.4 and pose.position.y < 1.0 and
                                          pose.position.x > 0.0 and pose.position.x < 1.0]


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

        if self.tower_builder.is_tower_finished():
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
                self.react_to_failure("No cubes detected. The tower cannot be build.", self.state, Status.ERROR)
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

            rospy.sleep(2)  # give tracker some time

            rospy.wait_for_service('pose_estimation/RetrieveTrackedPoses')
            res = self.retrieve_tracked_poses_client()
 
            if not res.success:
                self.react_to_failure(res.message, State.MOVE_TO_INITIAL_POSE)
                return

            # retrieved poses are body2camera poses -> transform to body2world poses
            #print(">>>>> poses before transformation", res.poses)
            transformed_poses = self.transform_poses(res.poses)
            #print(">>>>> poses after transformation", transformed_poses)

            # filter before choosing the target
            transformed_poses = self.filter_poses(transformed_poses)
            if len(transformed_poses) <= 0:
                self.react_to_failure("No cubes are in range. The tower cannot be build.", self.state, Status.ERROR)
                return

            target_pose = self.choose_target_pose(transformed_poses)
            # Planning Scene
            self.planning_scene.set_cube_env(transformed_poses) 

            # set z to align with table height
            target_pose.position.z = self.cube_dimension/2 + 0.005

        else:
            poses = []
            for i in range(4):
                poses.append(rospy.wait_for_message('/cube_%d_odom' % i,Odometry).pose.pose)
            self.planning_scene.set_cube_env(poses)                
            target_pose = rospy.wait_for_message('/cube_%d_odom' % self.tower_builder.placed_blocks,Odometry).pose.pose
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
        
        pose.position.z += 0.005 

        rospy.wait_for_service('/motion_planner/PlaceCube')
        res = self.place_cube_client(pose)

        if not res.success:
            self.react_to_failure(res.message, State.MOVE_TO_INITIAL_POSE)
            return

        # TODO: check if placement of cube was correct
        self.tower_builder.update_tower_state()
        self.approach = 0

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
