import rospy
from your_package.msg import CubePoses
from your_package.srv import PlanMotionJoint, PlanMotionPose, GripperControl

class MotionPlannerNode:
    def __init__(self):
        rospy.init_node('motion_planner_node', anonymous=True)
        rospy.Subscriber('/cube_poses', CubePoses, self.cube_callback)
        self.motion_planning_joint_service = rospy.Service('/plan_motion_joint', PlanMotionJoint, self.handle_motion_planning_joint)
        self.motion_planning_pose_service = rospy.Service('/plan_motion_pose', PlanMotionPose, self.handle_motion_planning_pose)
        self.gripper_control_service = rospy.Service('/gripper_control', GripperControl, self.handle_gripper_control)


    def cube_callback(self, cube_poses_msg):
        # Handle incoming cube poses msg
        # update collision information?
        # update target_cube_pose


    def handle_motion_planning_joint(self, req):
        response = PlanMotionJoint()
        response.result = "SUCCESS"
        return response

    def handle_motion_planning_pose(self, req):
        response = PlanMotionJPose()
        response.result = "SUCCESS"
        return response
    
    def handle_gripper_control(self, req):
        # Open or close the gripper
        # TODO
        response = GripperControl()
        response.result = "SUCCESS"
        return response


if __name__ == '__main__':
    motion_planner_node = MotionPlannerNode()
    rospy.spin()