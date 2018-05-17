import cv2
import numpy as np

img = 1 - cv2.imread('bikerack_obstacle.png', 0) / 255.0

print(np.max(img), np.min(img))
kernel = np.ones((3,3), np.uint8) # * 255

img_dilated = 1 - cv2.dilate(img, kernel, iterations=1)
img_dilated *= 255
img_dilated = img_dilated.astype(np.uint8)
# img_dilation[img_dilation > 0] = 255

cv2.imshow('Dilation', img_dilated)
# cv2.imshow('original', img)
cv2.waitKey(0)

cv2.imwrite('./bikerack_obstacle_dilated.png', img_dilated)
