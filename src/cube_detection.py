import rospy
from irobman_project.srv import GetPoints, GetPointsResponse
from geometry_msgs.msg import Point


class CubeDetectionNode:
    def __init__(self):
        rospy.init_node('cube_detection', anonymous=True)
        self.detection_service = rospy.Service('CubeDetection', GetPoints, self.handle_cube_detection) 


    def detect_cubes(self):

        # TODO: replace this placeholder with the cube detection algorithm
        p1 = Point()
        p1.x = 1
        p1.y = 2
        p1.z = 3

        p2 = Point()
        p2.x = 4
        p2.y = 5
        p2.z = 6

        p3 = Point()
        p3.x = 7
        p3.y = 8
        p3.z = 9

        return [p1, p2, p3]


    def handle_cube_detection(self, req):
        
        cube_points = self.detect_cubes()
         
        res = GetPointsResponse()
        res.points = cube_points
        return res


if __name__ == '__main__':
    cube_detection_node = CubeDetectionNode()
    rospy.spin()
