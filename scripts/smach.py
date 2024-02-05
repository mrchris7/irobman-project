#!/usr/bin/env python3

import rospy
import smach


class InitiaPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cube_detection'])

    def execute(self, userdata):
        rospy.loginfo('Execution Initial Pose')
        rospy.sleep(2)
        return 'cube_detection'


class CubeDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cube_picking'])

    def execute(self, userdata):
        rospy.loginfo('Execution Cube Detection')
        rospy.sleep(2)
        return 'cube_picking'
        
class CubePicking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cube_place'])
    def execute(self, userdata):
        rospy.loginfo('Execution Cube Picking')
        rospy.sleep(2)
        return 'cube_place'

class CubePlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cube_checking'])
    def execute(self, userdata):
        rospy.loginfo('Execution Cube Placing')
        rospy.sleep(2)
        return 'cube_checking'

class CubeChecking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initial_pose'])
    def execute(self, userdata):
        rospy.loginfo('Execution Cube Checking')
        rospy.sleep(2)
        return 'initial_pose'

# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('InitiaPose', InitiaPose(), 
                               transitions={'cube_detection':'CubeDetection'})
        smach.StateMachine.add('CubeDetection', CubeDetection(), 
                               transitions={'cube_picking':'CubePicking'})
        smach.StateMachine.add('CubePicking', CubePicking(), 
                               transitions={'cube_place':'CubePlace'})
        smach.StateMachine.add('CubePlace', CubePlace(), 
                               transitions={'cube_checking':'CubeChecking'})
        smach.StateMachine.add('CubeChecking', CubeChecking(), 
                               transitions={'initial_pose':'InitiaPose'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()