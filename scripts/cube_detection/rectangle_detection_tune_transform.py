# SOURCE: https://ch.mathworks.com/help/supportpkg/robotmanipulator/ug/configure-object-detection-using-opencv.html

import rospy
import cv2
import numpy as np
from get_image import *
import tf
from geometry_msgs.msg import Point


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
# img = load_image("img2.png")
# img = capture_image()
# img_depth = capture_depth_image()
img, img_depth = capture_rgbd_image()
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


def run(img_to_show=None):

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

    # store all center points
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
                center_depth = img_depth.get_value(center_x, center_y)
                print("center_x:", center_x, "center_y:", center_y, "center_depth:", center_depth)
                center_points.append([center_x, center_y, center_depth])
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

    points_3d = transform(center_points, None)


def change_blur(x):
    global blur
    if x % 2 == 1:
        print("blur:", x)
        blur = x
        run('blur')

def change_th_blocksize(x):
    global th_blocksize
    if x % 2 == 1 and x > 1:
        print("th_blocksize:", x)
        th_blocksize = x
        run('th_blocksize')

def change_th_c(x):
    global th_c
    print("th_c:", x)
    th_c = x
    run('th_c')

def change_canny_aperturesize(x):
    global canny_aperturesize
    if x in [3, 5, 7]:
        print("canny_aperturesize:", x)
        canny_aperturesize = x
        run('canny_aperturesize')

def change_canny_L2gradient(x):
    global canny_L2gradient
    print("canny_L2gradient:", x)
    canny_L2gradient = x != 0
    run('canny_L2gradient')

def change_cnt_thickness(x):
    global cnt_thickness
    cnt_thickness = x
    print("cnt_thickness:", cnt_thickness)
    run('cnt_thickness')

def change_area_min(x):
    global area_min
    print("area_min:", x)
    area_min = x
    run('area_min')

def change_area_max(x):
    global area_max
    print("area_max:", x)
    area_max = x
    run('area_max')

def change_cnt_maxlevel(x):
    global cnt_maxlevel
    print("cnt_maxlevel:", x)
    cnt_maxlevel = x
    run('cnt_maxlevel')

def change_poly_eps(x):
    global poly_eps
    poly_eps = x/1000
    print("poly_eps:", poly_eps)
    run('poly_eps')

def change_blur_sigmacol(x):
    global blur_sigmacol
    blur_sigmacol = x
    print("blur_sigmacol:", blur_sigmacol)
    run('blur_sigmacol')

def change_blur_sigmaspace(x):
    global blur_sigmaspace
    blur_sigmaspace = x
    print("blur_sigmaspace:", blur_sigmaspace)
    run('blur_sigmaspace')

def change_dil_ksize(x):
    global dil_ksize
    dil_ksize = x
    print("dil_ksize:", dil_ksize)
    run('dil_ksize')

def change_morph_ksize(x):
    global morph_ksize
    if x > 0:
        morph_ksize = x
        print("morph_ksize:", morph_ksize)
        run('morph_ksize')
        
def transform(points_2d, current_pose=None):

    points_3d = []
    for point_2d in points_2d:

        cube_x, cube_y, cube_depth = point_2d

        print("X-Co-ordinate in Camera Frame: %f", cube_x)
        print("Y-Co-ordinate in Camera Frame: %f", cube_y)

        f_x = 526.945
        f_y = 526.945	
        pp_x = 648.178
        pp_y = 358.773

        """
        holder_pos_x = -0.097397
        holder_pos_y = -0.06
        holder_pos_z = 0.0274111
        holder_roll = 0
        holder_pitch = -0.824473
        holder_yaw = 3.14159

        # Create translation matrix
        translation_matrix = np.array([[1, 0, 0, holder_pos_x],
                                        [0, 1, 0, holder_pos_y],
                                        [0, 0, 1, holder_pos_z],
                                        [0, 0, 0, 1]])

        # Create rotation matrix
        rotation_matrix = np.array([[np.cos(holder_yaw)*np.cos(holder_pitch), -np.sin(holder_yaw)*np.cos(holder_roll) + np.cos(holder_yaw)*np.sin(holder_pitch)*np.sin(holder_roll), np.sin(holder_yaw)*np.sin(holder_roll) + np.cos(holder_yaw)*np.sin(holder_pitch)*np.cos(holder_roll), 0],
                                    [np.sin(holder_yaw)*np.cos(holder_pitch), np.cos(holder_yaw)*np.cos(holder_roll) + np.sin(holder_yaw)*np.sin(holder_pitch)*np.sin(holder_roll), -np.cos(holder_yaw)*np.sin(holder_roll) + np.sin(holder_yaw)*np.sin(holder_pitch)*np.cos(holder_roll), 0],
                                    [-np.sin(holder_pitch), np.cos(holder_pitch)*np.sin(holder_roll), np.cos(holder_pitch)*np.cos(holder_roll), 0],
                                    [0, 0, 0, 1]])

        # Combine translation and rotation to get the transformation matrix
        camera2lefteye_matrix = np.dot(rotation_matrix, translation_matrix)
        """


        
        z_3d = cube_depth
        y_3d = (cube_y - pp_y)* cube_depth/f_y
        x_3d = (cube_x - pp_x)* cube_depth/f_x

        obj_camera_frame = np.array([x_3d, y_3d, z_3d])

        rospy.init_node("transform_listener")

        listener = tf.TransformListener()

        rate = rospy.Rate(10.0)  # Adjust the rate as needed

        while not rospy.is_shutdown():
            try:
                # Get the transform from "panda_hand" to "zed2_left_camera_frame"
                (trans, rot) = listener.lookupTransform("base_link", "zed2_left_camera_frame", rospy.Time(0))
                
                # Create a message to store the transform
                tf_msg = tf.TransformerROS().fromTranslationRotation(trans, rot)
                
                # Call the callback function with the received transform message
                rospy.loginfo("Transform received:")
                rospy.loginfo("Translation:", tf_msg.translation)
                rospy.loginfo("Rotation:", tf_msg.rotation)

                camera_to_robot_ = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))
                obj_robot_frame = np.dot(camera_to_robot_, obj_camera_frame)
            
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
                rospy.logerr("TF Exception: %s" % e)
        point = Point()
        point.x = obj_robot_frame[0]
        point.y = obj_robot_frame[1]
        point.z = obj_robot_frame[2]
        points_3d.append(point)

        # # translation = np.array([[1, 0, 0, x_3d],
        # #                         [0, 1, 0, y_3d],
        # #                         [0, 0, 1, z_3d],
        # #                         [0, 0, 0, 1]])

        # # # Create a 4x4 identity matrix
        # # rotation = np.eye(4)

        # # # Combine translation and rotation to get the transformation matrix
        # # cube2left_camera_matrix = np.dot(rotation, translation)
        
        
        # # <xacro:arg name="zed_pos_x"     default="-0.087397" /> 
        # # <xacro:arg name="zed_pos_y"     default="0.0523762" />   
        # # <xacro:arg name="zed_pos_z"     default="0.0374111" />
        # # <xacro:arg name="zed_roll"      default="0.000374354" />
        # # <xacro:arg name="zed_pitch"     default="0.746327" />
        # # <xacro:arg name="zed_yaw"       default="-1.57774" />        
        # zed_pos_x = -0.11
        # zed_pos_y = 0.056
        # zed_pos_z = 0.035
        # zed_roll = 0
        # zed_pitch = -1.35
        # zed_yaw = 0

        # # Create translation matrix
        # translation_matrix = np.array([[1, 0, 0, zed_pos_x],
        #                                 [0, 1, 0, zed_pos_y],
        #                                 [0, 0, 1, zed_pos_z],
        #                                 [0, 0, 0, 1]])

        # # Create rotation matrix
        # rotation_matrix = np.array([[np.cos(zed_yaw)*np.cos(zed_pitch), -np.sin(zed_yaw)*np.cos(zed_roll) + np.cos(zed_yaw)*np.sin(zed_pitch)*np.sin(zed_roll), np.sin(zed_yaw)*np.sin(zed_roll) + np.cos(zed_yaw)*np.sin(zed_pitch)*np.cos(zed_roll), 0],
        #                             [np.sin(zed_yaw)*np.cos(zed_pitch), np.cos(zed_yaw)*np.cos(zed_roll) + np.sin(zed_yaw)*np.sin(zed_pitch)*np.sin(zed_roll), -np.cos(zed_yaw)*np.sin(zed_roll) + np.sin(zed_yaw)*np.sin(zed_pitch)*np.cos(zed_roll), 0],
        #                             [-np.sin(zed_pitch), np.cos(zed_pitch)*np.sin(zed_roll), np.cos(zed_pitch)*np.cos(zed_roll), 0],
        #                             [0, 0, 0, 1]])

        # # Combine translation and rotation to get the transformation matrix
        # left_camera2eef_matrix = np.dot(rotation_matrix, translation_matrix)

        # # extracted from initial pose 0_T_eff:
        # if current_pose is not None:
        #     eef2base_matrix = np.array(current_pose).T
        # else:
        #     eef2base_matrix = np.array([[0.900853, -0.0270615, -0.433258, 0],
        #                                 [-0.0202124, -0.999578, 0.0204074, 0],
        #                                 [-0.433635, -0.00962709, -0.901037, 0],
        #                                 [0.466656, -0.0297833, 0.411497, 1]]).T


        # cube2base = eef2base_matrix @ left_camera2eef_matrix @ cube2left_camera_matrix

        # point = Point()
        # point.x = cube2base[0][3]
        # point.y = cube2base[1][3]
        # point.z = cube2base[2][3]
        # points_3d.append(point)

        print("X-Co-ordinate in Robot Frame: %f", point.x)
        print("Y-Co-ordinate in Robot Frame: %f", point.y)
        print("Z-Co-ordinate in Robot Frame: %f", point.z)

    return points_3d


cv2.imshow(window_name, img)

run()

cv2.createTrackbar('blur', window_name, 8, 30, change_blur)
cv2.createTrackbar('blur_sigmacol', window_name, 0, 300, change_blur_sigmacol)
cv2.createTrackbar('blur_sigmaspace', window_name, 0, 300, change_blur_sigmaspace)
cv2.createTrackbar('morph_ksize', window_name, 5, 50, change_morph_ksize)
cv2.createTrackbar('th_blocksize', window_name, 213, 255, change_th_blocksize) # 30
#cv2.createTrackbar('th_c', window_name, 4, 255, change_th_c) # 30
#cv2.createTrackbar('canny_aperturesize', window_name, 3, 7, change_canny_aperturesize)
#cv2.createTrackbar('canny_L2gradient', window_name, 1, 1, change_canny_L2gradient)
# cv2.createTrackbar('dil_ksize', window_name, 1, 100, change_dil_ksize)
# cv2.createTrackbar('cnt_thickness', window_name, 1, 10, change_cnt_thickness)
cv2.createTrackbar('area_min', window_name, 364, 10000, change_area_min)
cv2.createTrackbar('area_max', window_name, 2832, 10000, change_area_max)
cv2.createTrackbar('poly_eps', window_name, 54, 100, change_poly_eps)

cv2.waitKey(0)
cv2.destroyAllWindows()


