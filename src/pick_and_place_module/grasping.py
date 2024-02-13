import rospy
from std_msgs.msg import Float64, Header
from franka_gripper.msg import MoveActionGoal, GraspActionGoal,GraspActionResult, MoveActionResult
from control_msgs.msg import GripperCommandActionGoal
from moveit_msgs.msg import Grasp, GripperTranslation
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion, Point
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, unit_vector, quaternion_multiply
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes
import copy
from math import pi


class Gripper:
    def __init__(self):
        self.gripper_pub = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=10)
        self.gripper_move_pub = rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal, queue_size=10)
        self.gripper_move_result_pub = rospy.Subscriber('/franka_gripper/move/result', MoveActionResult, self.move_callback)
        self.gripper_grasp_result_pub = rospy.Subscriber('/franka_gripper/grasp/result', GraspActionResult, self.grasp_callback)
        self.gripper_move_result = MoveActionResult()
        self.gripper_grasp_result = GraspActionResult()

    def move_callback(self, data):
        self.gripper_move_result = data

    def grasp_callback(self, data):
        self.gripper_grasp_result = data
    
    def move(self, width, speed):
        
        gripper_data = MoveActionGoal()
        gripper_data.goal.width = width
       
        gripper_data.goal.speed = speed
        self.gripper_move_pub.publish(gripper_data)
        rospy.sleep(3)
        return self.gripper_move_result.result.success
        

    def grasp(self, width, speed, force ,ep_inner,ep_outer,):
        
        gripper_data = GraspActionGoal()
        gripper_data.goal.width = width
        gripper_data.goal.epsilon.inner = ep_inner
        gripper_data.goal.epsilon.outer = ep_outer
        gripper_data.goal.force = force
        gripper_data.goal.speed = speed
        rospy.loginfo("Executing grasp Width:%f, Force:%f", width, force)
        
        self.gripper_pub.publish(gripper_data)
        rospy.sleep(3)
        return self.gripper_grasp_result.result.success
    
    def create_grasp(self, pose):
        """
        :type pose: Pose
            pose of the gripper for the grasp
        :type grasp_id: str
            name for the grasp
        :rtype: Grasp
        """
        grasps = Grasp()

        #defining the grasp pose of the end effector
        grasps.grasp_pose.header.frame_id = "panda_link0"
        grasps.grasp_pose.pose.position.x = pose.position.x
        grasps.grasp_pose.pose.position.y = pose.position.y
        grasps.grasp_pose.pose.position.z = pose.position.z + 0.5
        quaternion = quaternion_from_euler(pi, 0, -pi/4)
        grasps.grasp_pose.pose.orientation.x = quaternion[0]
        grasps.grasp_pose.pose.orientation.y = quaternion[1]
        grasps.grasp_pose.pose.orientation.z = quaternion[2]
        grasps.grasp_pose.pose.orientation.w = quaternion[3]

        #setting the pregrasp approach
        grasps.pre_grasp_approach.direction.header.frame_id = "panda_link0"
        grasps.pre_grasp_approach.direction.vector.z = -1.0
        grasps.pre_grasp_approach.min_distance = 0.04
        grasps.pre_grasp_approach.desired_distance = 0.05
    
        #setting post grasp-retreat
        grasps.post_grasp_retreat.direction.header.frame_id = "panda_link0"
        grasps.post_grasp_retreat.direction.vector.z = 1.0
        grasps.post_grasp_retreat.min_distance = 0.04
        grasps.post_grasp_retreat.desired_distance = 0.05

        #opening the hand
        #add points
        grasps.pre_grasp_posture.joint_names.append("panda_finger_joint1")
        grasps.pre_grasp_posture.joint_names.append("panda_finger_joint2")
        point1=JointTrajectoryPoint()
        point1.positions.append(0.04)
        point1.positions.append(0.04)
        point1.time_from_start = rospy.Duration.from_sec(0.5)
        grasps.pre_grasp_posture.points.append(point1)
     
        #closing the hand
        #add points
        grasps.grasp_posture.joint_names.append("panda_finger_joint1")
        grasps.grasp_posture.joint_names.append("panda_finger_joint2")
        point2=JointTrajectoryPoint()
        point2.positions.append(0.00)
        point2.positions.append(0.00)
        point2.time_from_start = rospy.Duration.from_sec(0.5)
        grasps.grasp_posture.points.append(point2)

        return grasps
        
        

        
        
