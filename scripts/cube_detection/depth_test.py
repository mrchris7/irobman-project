import cv2
import numpy as np
from get_image import capture_image, load_image, capture_depth_image


# start
img_depth = capture_depth_image()
#img = capture_image()
scaling = 1


cv2.imshow("depth_image", img_depth)



img = capture_image()

cv2.imshow("image", img)

cv2.waitKey(0)
cv2.destroyAllWindows()



