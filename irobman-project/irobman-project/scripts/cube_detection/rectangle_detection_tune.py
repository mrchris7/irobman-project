# SOURCE: https://ch.mathworks.com/help/supportpkg/robotmanipulator/ug/configure-object-detection-using-opencv.html

import cv2
import numpy as np
from get_image import capture_image, load_image


# initial values
blur = 7
blur_sigmacol = 0.0
blur_sigmaspace = 0.0
morph_ksize = 7
th_blocksize = 185
th_c = 4
canny_aperturesize = 3
canny_L2gradient = True
dil_ksize = 1
area_min = 500
area_max = 10000
cnt_thickness = 1
poly_eps = 0.026


# start
img = load_image("img1.png")
scaling = 0.5  # scale image for visualization

img = cv2.resize(img, (0,0), fx=scaling, fy=scaling)
window_name = 'image'


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

    # create shape image containing detected rectangles
    shapes_img = img.copy()
    cube_count = 0
    for cnt in filtered_contours:
        x1, y1 = cnt[0][0]
        approx = cv2.approxPolyDP(cnt, poly_eps * cv2.arcLength(cnt, True), True)
        if len(approx) == 4:
            # TODO: filter cubes: ensure that the 4 edges have approximately the same length
            shapes_img = cv2.drawContours(shapes_img, [cnt], -1, (0, 255, 0), 1)
            shapes_img = cv2.drawContours(shapes_img, [approx], -1, (0, 255, 0), 2)
            cv2.putText(shapes_img, 'Cube', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cube_count += 1

    if img_to_show in ['poly_eps']:
        cv2.imshow(window_name, shapes_img)

    # show always contours img
    cv2.imshow("contours", contour_img)

    # show always result img
    cv2.putText(shapes_img, f'cubes detected: {cube_count}', (10, shapes_img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow("result", shapes_img)


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



cv2.imshow(window_name, img)

run()

cv2.createTrackbar('blur', window_name, 7, 30, change_blur)
cv2.createTrackbar('blur_sigmacol', window_name, 0, 300, change_blur_sigmacol)
cv2.createTrackbar('blur_sigmaspace', window_name, 0, 300, change_blur_sigmaspace)
cv2.createTrackbar('morph_ksize', window_name, 7, 50, change_morph_ksize)
cv2.createTrackbar('th_blocksize', window_name, 185, 255, change_th_blocksize) # 30
cv2.createTrackbar('th_c', window_name, 4, 255, change_th_c) # 30
cv2.createTrackbar('canny_aperturesize', window_name, 3, 7, change_canny_aperturesize)
cv2.createTrackbar('canny_L2gradient', window_name, 1, 1, change_canny_L2gradient)
cv2.createTrackbar('dil_ksize', window_name, 1, 100, change_dil_ksize)
cv2.createTrackbar('cnt_thickness', window_name, 1, 10, change_cnt_thickness)
cv2.createTrackbar('area_min', window_name, 500, 10000, change_area_min)
cv2.createTrackbar('area_max', window_name, 10000, 10000, change_area_max)
cv2.createTrackbar('poly_eps', window_name, 26, 100, change_poly_eps)

cv2.waitKey(0)
cv2.destroyAllWindows()


