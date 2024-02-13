import rospy
from irobman_project.srv import GetPoints, GetPointsResponse
from geometry_msgs.msg import Point


class CubeDetectionNode:
    def __init__(self):
        rospy.init_node('cube_detection', anonymous=True)
        self.detection_service = rospy.Service('CubeDetection', GetPoints, self.handle_cube_detection) 


    def detect_cubes(self):

        # TODO: replace this placeholder with the cube detection algorithm

        img = capture_image()


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


# initial values
blur = 8
blur_sigmacol = 0.0
blur_sigmaspace = 0.0
morph_ksize = 5
# th_blocksize = 185
th_blocksize = 213
th_c = 4
canny_aperturesize = 3
canny_L2gradient = True
dil_ksize = 1
area_min = 364
area_max = 2832
cnt_thickness = 1
poly_eps = 0.054


# start
#img = load_image("img2.png")
img = capture_image()
#img_depth = capture_depth_image()
scaling = 1

img = cv2.resize(img, (0,0), fx=scaling, fy=scaling)
window_name = 'image'


def center_of_points(points):
    print("input:", points)
    x1, y1 = points[0][0]
    x2, y2 = points[1][0]
    x3, y3 = points[2][0]
    x4, y4 = points[3][0]

    center_x = (x1 + x2 + x3 + x4) / 4
    center_y = (y1 + y2 + y3 + y4) / 4
    return center_x, center_y


def detect_centerpoints(img_to_show=None):

    # gray conversion
    img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    # blur the image
    #img_blurred = cv2.medianBlur(img_gray, blur)
    img_blurred = cv2.bilateralFilter(img_gray, blur, sigmaColor=blur_sigmacol, sigmaSpace=blur_sigmaspace)

    if img_to_show in ['blur', 'blur_sigmacol', 'blur_sigmaspace']:
        cv2.imshow(window_name, img_blurred)

    rect_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (morph_ksize, morph_ksize))
    morph_img = cv2.morphologyEx(img_blurred, cv2.MORPH_CLOSE, rect_kernel)  # MORPH_CLOSE (MOPRPH_RECT also works)
    if img_to_show in ['morph_ksize']:
        cv2.imshow(window_name, morph_img)

    # adaptive thresholding:
    #thresh_image = cv2.adaptiveThreshold(morph_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, th_blocksize, th_c) # 11,2
    # or threshold binarization:
    _, thresh_image = cv2.threshold(morph_img, th_blocksize, 255, cv2.THRESH_BINARY)

    if img_to_show == 'th_blocksize' or img_to_show == 'th_c':
        cv2.imshow(window_name, thresh_image)


    # optional: edge detection
    canny_image = cv2.Canny(thresh_image, 255, 255, apertureSize=canny_aperturesize, L2gradient=canny_L2gradient)
    if img_to_show == 'canny_aperturesize' or img_to_show == 'canny_L2gradient':
        cv2.imshow(window_name, canny_image)
    canny_image = thresh_image # edge detection is not necessary if binarization-threshold is used


    # bring the lines and areas closer together (merge them)
    kernel = np.ones((dil_ksize, dil_ksize), np.uint8)
    dilated_img = cv2.dilate(canny_image, kernel, iterations=1)

    if img_to_show == 'dil_ksize':
        cv2.imshow(window_name, dilated_img)

    # find contours
    contours, hierarchy = cv2.findContours(dilated_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    filtered_contours = []
    min_rects = []
    box_img = img.copy()

    for cnt in contours:
        approx = cv2.contourArea(cnt)

        if approx > area_min and approx < area_max:
            filtered_contours.append(cnt)

            """
            # find minimum rectangles around contour
            # (-> no reliable method because it can be a contour of any shape...)
            min_rect = cv2.minAreaRect(cnt)
            min_rects.append(min_rect)
            box = cv2.boxPoints(min_rect)
            box = np.intp(box)  # np.intp: Integer used for indexing (same as C size_t; normally either int32 or int64)
            cv2.drawContours(box_img, [box], 0, (0, 255, 0), cnt_thickness)
            """
    """
    # show always box img
    cv2.imshow('Rect around contours', box_img)
    """

    width = img.shape[1::-1][1]
    height = img.shape[1::-1][0]

    # create contour image
    contour_img = np.zeros((width, height, 3), np.uint8)
    cv2.drawContours(contour_img, filtered_contours, -1, (255, 255, 255), cnt_thickness)
    if img_to_show in ['cnt_thickness', 'area_min', 'area_max']:
        cv2.imshow(window_name, contour_img)

    # store center points of rectangles
    center_points = []

    # create shape image containing detected rectangles
    shapes_img = img.copy()
    cube_count = 0
    for cnt in filtered_contours:
        x1, y1 = cnt[0][0]
        approx = cv2.approxPolyDP(cnt, poly_eps * cv2.arcLength(cnt, True), True)
        if len(approx) == 4:
            len1 = np.linalg.norm(approx[0, 0] - approx[1, 0])
            len2 = np.linalg.norm(approx[1, 0] - approx[2, 0])
            print(abs(len1 - len2))
            if abs(len1 - len2) <= 20:
                shapes_img = cv2.drawContours(shapes_img, [cnt], -1, (0, 255, 0), 1)
                shapes_img = cv2.drawContours(shapes_img, [approx], -1, (0, 255, 0), 2)
                center_x, center_y = center_of_points(approx)
                center_points.append(round(center_x), round(center_y))
                print("center: ", center_x, center_y)
                cv2.circle(shapes_img, (round(center_x), round(center_y)), 2, (0, 255, 0), 1)
                cv2.putText(shapes_img, 'Cube', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cube_count += 1

    if img_to_show in ['poly_eps']:
        cv2.imshow(window_name, shapes_img)

    # show always contours img
    cv2.imshow("contours", contour_img)

    # show always result img
    cv2.putText(shapes_img, f'cubes detected: {cube_count}', (10, shapes_img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow("result", shapes_img)





def capture_image(depth):

    # Create a Camera object
    zed = sl.Camera()

    # Create InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 or HD1200 video mode, depending on camera type.
    init_params.camera_fps = 30  # Set fps at 30
    init_params.coordinate_units = sl.UNIT.METER
    runtime_parameters = sl.RuntimeParameters()

    if depth:
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # or ULTRA, QUALITY, PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_minimum_distance = 0.15  # => 15cm
        init_params.depth_maximum_distance = 2 # => 2m
        runtime_parameters.enable_fill_mode = True
    else:
        init_params.depth_mode = sl.DEPTH_MODE.NONE
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
    while i < 4:
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



if __name__ == '__main__':
    cube_detection_node = CubeDetectionNode()
    rospy.spin()



