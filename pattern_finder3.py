# import the necessary packages
import numpy as np
import argparse
#import imutils
import cv2

img = cv2.imread("markers.png")
mark = cv2.imread("marker_0.png")
(h, w) = mark.shape[:2]

result = cv2.matchTemplate(img, mark, cv2.TM_CCOEFF)
(_, _, minLoc, maxLoc) = cv2.minMaxLoc(result)

topLeft = maxLoc
botRight = (topLeft[0] + w, topLeft[1] + h)
roi = img[topLeft[1]:botRight[1], topLeft[0]:botRight[0]]

mask = np.zeros(img.shape, dtype = "uint8")
img = cv2.addWeighted(img, 0.25, mask, 0.75, 0)

img[topLeft[1]:botRight[1], topLeft[0]:botRight[0]] = roi

cv2.imshow("Image", img)
cv2.imshow("Mark", mark)
cv2.waitKey(0)