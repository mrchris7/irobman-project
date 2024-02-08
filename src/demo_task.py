import rospy
from enum import Enum
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from typing import List
from irobman_project.srv import CartesianMotionPlanning, JointMotionPlanning


class DemoTask:
    def __init__(self):
        self.joint_planning_client = rospy.ServiceProxy('JointMotionPlanning', JointMotionPlanning)
        self.close_gripper_client = rospy.ServiceProxy('GripperControl', SetBool)
        self.error = False

    def planning(self, joints):
        if (self.error):
            return

        js = JointState()
        js.position = joints
        rospy.wait_for_service('JointMotionPlanning')
        res = self.joint_planning_client(js)
        if not res.success:
            self.error = True
            rospy.logerr(res.message)

    def gripper(self, open):
        if (self.error):
            return

        rospy.wait_for_service('GripperControl')
        res = self.close_gripper_client(open)
        if not res.success:
            self.error = True
            rospy.logerr(res.message)
        

    def start(self):

        rospy.logwarn("Starting demo.")

        # air
        self.planning([2.0678,0.595755,-2.21163,-2.17795,0.453459,1.78567,0.441341])

        # pre cube 1
        self.planning([-0.30624,0.505797,-0.542583,-2.13841,0.538941,2.54162,-0.49178])

        self.gripper(False)

        # cube 1
        self.planning([-0.330565,0.660273,-0.486617,-2.09717,0.539462,2.63023,-0.435691])

        self.gripper(True)

        # air
        self.planning([2.0678,0.595755,-2.21163,-2.17795,0.453459,1.78567,0.441341])

        # tower low pre
        self.planning([1.09169,1.10297,-0.535518,-1.0093,0.424635,2.02258,1.42981])

        # tower low
        self.planning([1.04654,1.21175,-0.46713,-0.988193,0.455717,2.1506,1.42862])

        self.gripper(False)

        # tower low pre
        self.planning([1.09169,1.10297,-0.535518,-1.0093,0.424635,2.02258,1.42981])

        # air
        self.planning([2.0678,0.595755,-2.21163,-2.17795,0.453459,1.78567,0.441341])

        # cube 2 pre
        self.planning([0.0677275,0.338532,-0.295676,-2.27223,0.12365,2.61353,0.342853])

        # cube 2
        self.planning([0.0449299,0.54133,-0.254465,-2.21104,0.168812,2.69281,0.347727])

        self.gripper(True)

        # air
        self.planning([2.0678,0.595755,-2.21163,-2.17795,0.453459,1.78567,0.441341])

        # tower high pre
        self.planning([1.06946,1.05264,-0.515923,-1.01394,0.425934,2.00084,1.40731])
        
        # tower high
        self.planning([1.06966,1.15498,-0.491592,-1.0145,0.428463,2.12603,1.50089])

        self.gripper(False)

        # tower high pre
        self.planning([1.06946,1.05264,-0.515923,-1.01394,0.425934,2.00084,1.40731])

        # air
        self.planning([2.0678,0.595755,-2.21163,-2.17795,0.453459,1.78567,0.441341])

        rospy.logwarn("Finished demo.")


if __name__ == '__main__':
    
    demo = DemoTask()
    demo.start()