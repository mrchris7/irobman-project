import pyzed.sl as sl
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def capture_rgbd_image_ros():
    # Initialize the node
    rospy.init_node('zed_image_capture', anonymous=True)

    # Initialize CvBridge
    bridge = CvBridge()

    # Subscribe to the color image topic

    color_image_msg = rospy.wait_for_message('/zed2/zed_node/left/image_rect_color', Image)

    # Convert color image message to OpenCV image
    color_image = bridge.imgmsg_to_cv2(color_image_msg, desired_encoding="bgr8")

    # Subscribe to the depth image topic
    depth_image_msg = rospy.wait_for_message('/zed2/zed_node/depth/depth_registered', Image)

    # Convert depth image message to OpenCV image
    depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")

    return color_image, depth_image

def capture_image():

    # Create a Camera object
    zed = sl.Camera()

    # Create InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 or HD1200 video mode, depending on camera type.
    init_params.camera_fps = 30  # Set fps at 30
    init_params.depth_mode = sl.DEPTH_MODE.NONE

    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_minimum_distance = 0.15  # => 15cm
    init_params.depth_maximum_distance = 2 # => 2m

    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.enable_fill_mode = False



    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Camera Open: " + repr(err) + ". Exit program.")
        exit()

    # Capture 1 frame
    i = 0
    image = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    while i < 5:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # A new image is available if grab() returns SUCCESS
            zed.retrieve_image(image, sl.VIEW.LEFT)
            timestamp = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)
            print("Image resolution: {0} x {1} || Image timestamp: {2}\n".format(image.get_width(), image.get_height(),
                                                                                timestamp.get_milliseconds()))
            i = i+1

    img = image.get_data()

    # Close the camera
    zed.close()

    return img


def capture_depth_image():
    # Create a Camera object
    zed = sl.Camera()

    # Create InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 or HD1200 video mode, depending on camera type.
    init_params.camera_fps = 30  # Set fps at 30
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # or ULTRA, QUALITY, PERFORMANCE
    
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_minimum_distance = 0.15  # => 15cm
    init_params.depth_maximum_distance = 2 # => 2m

    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.enable_fill_mode = True

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Camera Open: " + repr(err) + ". Exit program.")
        exit()

    # Capture 1 frame
    i = 0
    depth_map = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    while i < 5:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH) # Retrieve depth map

            #depth_for_display = sl.Mat()
            #zed.retrieve_image(image, sl.VIEW.LEFT) # Retrieve depth image

            i = i+1

    #img = depth_map.get_data()

    # Close the camera
    zed.close()

    return depth_map

def capture_rgbd_image():
    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # Set depth mode
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Set camera resolution

    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_minimum_distance = 0.15  # => 15cm
    init_params.depth_maximum_distance = 1 # => 2m

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open ZED camera: {str(err)}")
        exit()

    # Capture data
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.enable_fill_mode = True
    depth = sl.Mat()
    image = sl.Mat()

    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # Retrieve depth map
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)

        # Retrieve RGB image
        zed.retrieve_image(image, sl.VIEW.LEFT)

        # Convert depth map and RGB image to numpy arrays
        #depth_data = depth.get_data()
        rgb_data = image.get_data()
    
    # Close the camera
    zed.close()
    return rgb_data, depth

IMG_PATH = '/opt/ros_ws/src/irobman-project/scripts/cube_detection/imgs/'
def load_image(img_name):
    img = cv2.imread(IMG_PATH + img_name)
    return img