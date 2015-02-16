
# Import defpendencies
import numpy as np
import cv2

# Create a capture instance from the camera
capture = cv2.VideoCapture(0)

# Infinite loop
while(True):

	# Take frame
	_, frame = capture.read()

	# Convert frame colorspace from BGR to HSV
	hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

	# HSV Values
	hsv_lower = np.array([0,0,0],dtype=np.uint8)
	hsv_upper = np.array([0,0,255],dtype=np.uint8)

	# Threshold the HSV image to get only defined colors
	mask = cv2.inRange(hsv,hsv_lower,hsv_upper)

	# Bitwise-AND mask and frame
	result = cv2.bitwise_and(frame,frame,mask = mask)

	# Display Images
	cv2.imshow("frame",frame)
	cv2.imshow("mask",mask)
	cv2.imshow("result",result)

	# If key interrupt break
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

# Destroy windows
cv2.destroyAllWindows()
