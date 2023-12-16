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
corner_blocksize = 2
corner_ksize = 3
corner_k = 0.04
corner_thresh = 0.01
dil_ksize = 1
lines_thresh = 100
lines_minlen = 80
lines_maxgap = 10



# start
img = load_image("img1.png")
img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
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

    dst = cv2.cornerHarris(thresh_image, blockSize=corner_blocksize, ksize=corner_ksize, k=corner_k)
    if img_to_show in ['corner_blocksize', 'corner_ksize', 'corner_k']:
        cv2.imshow(window_name, dst)

    kernel = np.ones((2, 2), np.uint8)
    dst = cv2.dilate(dst, kernel)

    threshold = corner_thresh * dst.max()
    corners = np.where(dst > threshold)
    result_img = img.copy()

    # Draw corners on the image
    for pt in zip(*corners[::-1]):
        cv2.circle(result_img, pt, 3, (0, 0, 255), 1)

    # edge detection
    edges = cv2.Canny(img_gray, 50, 150, apertureSize=canny_aperturesize, L2gradient=canny_L2gradient)

    if img_to_show in ['canny_aperturesize', 'canny_L2gradient']:
        cv2.imshow(window_name, edges)

    kernel = np.ones((dil_ksize, dil_ksize), np.uint8)
    dil_edges = cv2.dilate(edges, kernel)

    if img_to_show in ['dil_ksize']:
        cv2.imshow(window_name, dil_edges)

    # Line detection using HoughLines
    lines = cv2.HoughLinesP(dil_edges, 1, np.pi / 180, lines_thresh, None, lines_minlen, lines_maxgap)

    corners = np.column_stack((corners[1], corners[0]))  # Convert to (x, y) format
    # Draw lines on the image
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(result_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # show always result img
    cv2.imshow("result", result_img)



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

def change_corner_blocksize(x):
    global corner_blocksize
    if x > 0:
        corner_blocksize = x
        print("corner_blocksize:", corner_blocksize)
        run('corner_blocksize')

def change_corner_ksize(x):
    global corner_ksize
    if x  % 2 == 1 and x <= 31:
        corner_ksize = x
        print("corner_ksize:", corner_ksize)
        run('corner_ksize')

def change_corner_k(x):
    global corner_k
    corner_k = x/100
    print("corner_k:", corner_k)
    run('corner_k')

def change_lines_thresh(x):
    global lines_thresh
    lines_thresh = x
    print("lines_thresh:", lines_thresh)
    run('lines_thresh')

def change_lines_minlen(x):
    global lines_minlen
    lines_minlen = x
    print("lines_minlen:", lines_minlen)
    run('lines_minlen')

def change_lines_maxgap(x):
    global lines_maxgap
    lines_maxgap = x
    print("lines_maxgap:", lines_maxgap)
    run('lines_maxgap')

def change_corner_thresh(x):
    global corner_thresh
    corner_thresh = x/100
    print("corner_thresh:", corner_thresh)
    run('corner_thresh')



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
cv2.createTrackbar('corner_blocksize', window_name, 4, 30, change_corner_blocksize)
cv2.createTrackbar('corner_ksize', window_name, 7, 31, change_corner_ksize)
cv2.createTrackbar('corner_k', window_name, 9, 100, change_corner_k)
cv2.createTrackbar('corner_thresh', window_name, 13, 100, change_corner_thresh)
cv2.createTrackbar('dil_ksize', window_name, 2, 100, change_dil_ksize)
cv2.createTrackbar('lines_thresh', window_name, 12, 300, change_lines_thresh)
cv2.createTrackbar('lines_minlen', window_name, 23, 200, change_lines_minlen)
cv2.createTrackbar('lines_maxgap', window_name, 3, 300, change_lines_maxgap)

cv2.waitKey(0)
cv2.destroyAllWindows()


