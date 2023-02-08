import cv2
import numpy as np

white = np.uint8([[[255,255,255 ]]])
hsv_white = cv2.cvtColor(white,cv2.COLOR_BGR2HSV)

black = np.uint8([[[185,126,96]]])
hsv_black = cv2.cvtColor(black,cv2.COLOR_BGR2HSV)
white = np.uint8([[[0,0,103]]])
rgb_white = cv2.cvtColor(white,cv2.COLOR_HSV2RGB)

#110 137 188
#print(hsv_white)
#print(hsv_black)
print(rgb_white)