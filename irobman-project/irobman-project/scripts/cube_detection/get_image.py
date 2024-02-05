import pyzed.sl as sl
import cv2


def capture_image():

    # Create a Camera object
    zed = sl.Camera()

    # Create InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.AUTO  # Use HD720 or HD1200 video mode, depending on camera type.
    init_params.camera_fps = 30  # Set fps at 30

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Camera Open: " + repr(err) + ". Exit program.")
        exit()

    # Capture 1 frame
    i = 0
    image = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    while i < 1:
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


IMG_PATH = 'imgs/'
def load_image(img_name):
    img = cv2.imread(IMG_PATH + img_name)
    return img