import rospy
from irobman_project.srv import GetPoints, GetPointsResponse
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import numpy as np
import cv_bridge
import tf

class CubeDetectionNode:
    def __init__(self):
        rospy.init_node('cube_detection', anonymous=True)
        self.detection_service = rospy.Service('CubeDetection', GetPoints, self.handle_cube_detection) 
        
        self.tf_cam2base_lst = tf.TransformListener()
        rospy.loginfo("Start to detect cube...")
        rospy.Subscriber("/camera/image_raw", Image, self.imageCb)

    def imageCb(self, msg):
        rospy.loginfo("Processing raw image...")
        try:
            img = cv_bridge.CvBridge().imgmsg_to_cv2(msg, "bgr8")
        except cv_bridge.CvBridgeError as e:
            rospy.loginfo("cv_bridge exception: %s" % e)
            return
        obj2cam = np.array
        self.tf_cam2base_lst.waitForTransform('/panda_link0', '/zed2_left_camera_frame', rospy.Duration(4.0))
        try:
            now = rospy.Time.now
            self.tf_cam2base_lst.waitForTransform('/panda_link0', '/zed2_left_camera_frame', now, rospy.Duration(4.0))
            (trans, rot) = self.tf_cam2base_lst.lookupTransform('/zed2_left_camera_frame', '/panda_link0', now)
            cam2base = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logerr("TF Exception: %s" % e)

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
