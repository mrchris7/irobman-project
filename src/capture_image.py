#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from irobman_project.srv import CaptureImage, CaptureImageResponse


class CaptureImageNode:

    def __init__(self):
        rospy.init_node('capture_image', anonymous=True)
        self.capture_image = rospy.Service('CaptureImage', CaptureImage, self.capture_rgbd_image_ros)

    def capture_rgbd_image_ros(self, req):

        # Initialize CvBridge
        bridge = CvBridge()

        # Subscribe to the color image topic

        color_image_msg = rospy.wait_for_message('/zed2/zed_node/left/image_rect_color', Image)

        # Subscribe to the depth image topic
        depth_image_msg = rospy.wait_for_message('/zed2/zed_node/depth/depth_registered', Image)

        # Convert messages to OpenCV image
        #color_image = bridge.imgmsg_to_cv2(color_image_msg, desired_encoding="bgr8")
        #depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")

        response = CaptureImageResponse()
        response.color_image = color_image_msg
        response.depth_image = depth_image_msg
        response.success = True
        response.message = "Color and depth image were captured."
        return response


if __name__ == '__main__':
    capture_node = CaptureImageNode()
    rospy.spin()