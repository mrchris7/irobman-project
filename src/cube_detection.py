#!/usr/bin/env python3

import rospy
from irobman_project.srv import GetPoints, GetPointsResponse
from geometry_msgs.msg import Point
import cv2
import numpy as np
from pose_estimation.srv import CaptureImage
from cv_bridge import CvBridge


class CubeDetectionNode:
    def __init__(self):
        rospy.init_node('cube_detection', anonymous=True)
        self.detection_service = rospy.Service('CubeDetection', GetPoints, self.handle_cube_detection) 
        self.capture_image_client = rospy.ServiceProxy('CaptureImage', CaptureImage)
        # initial values
        self.blur = 8
        self.blur_sigmacol = 0.0
        self.blur_sigmaspace = 0.0
        self.morph_ksize = 5
        self.th_blocksize = 213
        self.th_c = 4
        self.canny_aperturesize = 3
        self.canny_L2gradient = True
        self.dil_ksize = 1
        self.area_min = 364
        self.area_max = 2832
        self.cnt_thickness = 1
        self.poly_eps = 0.054


    def handle_cube_detection(self, req):
        
        img, img_depth = self.capture_image()
        center_points = self.detect_center_points(img, img_depth)
        cube_points_3d = self.transform(center_points)
         
        res = GetPointsResponse()
        res.points = cube_points_3d
        res.message = "Successfully detected points"
        res.success = True
        return res 


    def capture_image(self):
        res = self.capture_image_client()

        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(res.color_image, desired_encoding="bgr8")
        img_depth = bridge.imgmsg_to_cv2(res.depth_image, desired_encoding="passthrough")
        
        depth_image_normalized = cv2.normalize(img_depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

        #cv2.imshow("image", img)    
        #cv2.imshow("image depth", depth_image_normalized)

        scaling = 1
        img = cv2.resize(img, (0,0), fx=scaling, fy=scaling)
        return img, img_depth


    def detect_center_points(self, img, img_depth):
        # gray conversion
        img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        # blur the image
        #img_blurred = cv2.medianBlur(img_gray, blur)
        img_blurred = cv2.bilateralFilter(img_gray, self.blur, sigmaColor=self.blur_sigmacol, sigmaSpace=self.blur_sigmaspace)

        rect_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.morph_ksize, self.morph_ksize))
        morph_img = cv2.morphologyEx(img_blurred, cv2.MORPH_CLOSE, rect_kernel)  # MORPH_CLOSE (MOPRPH_RECT also works)


        # adaptive thresholding:
        #thresh_image = cv2.adaptiveThreshold(morph_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, th_blocksize, th_c) # 11,2
        # or threshold binarization:
        _, thresh_image = cv2.threshold(morph_img, self.th_blocksize, 255, cv2.THRESH_BINARY)


        # optional: edge detection
        canny_image = cv2.Canny(thresh_image, 255, 255, apertureSize=self.canny_aperturesize, L2gradient=self.canny_L2gradient)
        canny_image = thresh_image # edge detection is not necessary if binarization-threshold is used


        # bring the lines and areas closer together (merge them)
        kernel = np.ones((self.dil_ksize, self.dil_ksize), np.uint8)
        dilated_img = cv2.dilate(canny_image, kernel, iterations=1)

        # find contours
        contours, hierarchy = cv2.findContours(dilated_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        filtered_contours = []
        min_rects = []
        box_img = img.copy()

        for cnt in contours:
            approx = cv2.contourArea(cnt)

            if approx > self.area_min and approx < self.area_max:
                filtered_contours.append(cnt)


        width = img.shape[1::-1][1]
        height = img.shape[1::-1][0]

        # create contour image
        contour_img = np.zeros((width, height, 3), np.uint8)
        cv2.drawContours(contour_img, filtered_contours, -1, (255, 255, 255), self.cnt_thickness)


        # store all center points
        center_points = []

        # create shape image containing detected rectangles
        shapes_img = img.copy()
        cube_count = 0
        for cnt in filtered_contours:
            x1, y1 = cnt[0][0]
            approx = cv2.approxPolyDP(cnt, self.poly_eps * cv2.arcLength(cnt, True), True)
            if len(approx) == 4:
                len1 = np.linalg.norm(approx[0, 0] - approx[1, 0])
                len2 = np.linalg.norm(approx[1, 0] - approx[2, 0])
                print(abs(len1 - len2))
                if abs(len1 - len2) <= 20:
                    shapes_img = cv2.drawContours(shapes_img, [cnt], -1, (0, 255, 0), 1)
                    shapes_img = cv2.drawContours(shapes_img, [approx], -1, (0, 255, 0), 2)
                    center_x, center_y = center_of_points(approx)
                                        
                    center_depth = img_depth[round(center_y), round(center_x)] #img_depth.get_value(round(center_x), round(center_y))[1] #
                    print("center_x:", center_x, "center_y:", center_y, "center_depth:", center_depth)
                    center_points.append([center_x, center_y, center_depth])
                    
                    cv2.circle(shapes_img, (round(center_x), round(center_y)), 2, (0, 255, 0), 1)
                    cv2.putText(shapes_img, 'Cube', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cube_count += 1

        # show always contours img
        #cv2.imshow("contours", contour_img)

        # show always result img
        cv2.putText(shapes_img, f'cubes detected: {cube_count}', (10, shapes_img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        #cv2.imshow("result", shapes_img)
        #cv2.waitKey(0)
        ##cv2.destroyAllWindows()
        
        return center_points


    def transform(self, points_2d):

        points_3d = []
        for point_2d in points_2d:

            cube_x, cube_y, cube_depth = point_2d
            
            print("X-Co-ordinate in Camera Frame: %f", cube_x)
            print("Y-Co-ordinate in Camera Frame: %f", cube_y)
            print("D-Co-ordinate in Camera Frame: %f", cube_depth)

            f_x = 264.559 #526.945
            f_y = 264.559 #526.945	
            pp_x = 317.703 #648.178
            pp_y = 181.865 #358.773

            # point from zed2_left_camera_frame
            z_3d = cube_depth
            y_3d = (cube_y - pp_y)* cube_depth/f_y
            x_3d = (cube_x - pp_x)* cube_depth/f_x
            
            point = Point()
            point.x = x_3d
            point.y = y_3d
            point.z = z_3d
            points_3d.append(point)

            print("X-Co-ordinate in Camera Frame: %f", point.x)
            print("Y-Co-ordinate in Camera Frame: %f", point.y)
            print("Z-Co-ordinate in Camera Frame: %f", point.z)

        return points_3d

def center_of_points(points):
    x1, y1 = points[0][0]
    x2, y2 = points[1][0]
    x3, y3 = points[2][0]
    x4, y4 = points[3][0]

    center_x = (x1 + x2 + x3 + x4) / 4
    center_y = (y1 + y2 + y3 + y4) / 4
    return center_x, center_y


if __name__ == '__main__':
    cube_detection_node = CubeDetectionNode()
    rospy.spin()



