import rospy
from sensor_msgs.msg import Image
from your_package.srv import TargetCube  # request is empty, respond is a position
from cv_bridge import CvBridge


class CubeDetection:
    def __init__(self):
        rospy.init_node('perception_node', anonymous=True)
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.cube_detection_service = rospy.Service('detect_cube', TargetCube, perception_node.handle_capture_cube_image) 
        # -> maybe make it a publisher that always returns all detected cube positions and the selection of the target cube happens
        # inside the statemachine node (so the pose estimation can always use the detected cubes as prior information for 
        # the pose estimation)
        self.bridge = CvBridge()
        self.last_received_image = None

    def image_callback(self, image_msg):
        # Convert ROS Image message to OpenCV image
        self.last_received_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")


    def detect_cube(self):
        # Implement cube detection logic here
        # use self.last_received_image
        # return 3d position of target cube
        return [1, 4, -2]


    def handle_capture_cube_image(self, req):
        # Capture the current image and return it with the detected cube
        captured_image = self.capture_current_image_with_cube()
        return captured_image

    def capture_current_image_with_cube(self):
        # Use the last received image for processing
        if self.last_received_image is not None:
            target_cube_pos = self.detect_cube()
            return target_cube_pos
        else:
            rospy.logwarn("No image received yet.")
            return None

if __name__ == '__main__':
    perception_node = CubeDetection()
    rospy.spin()
