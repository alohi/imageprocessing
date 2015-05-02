import numpy as np
import cv2

cv2.namedWindow('result')

def on_mouse(event, x, y, flag, param):
	print str(x) + ',' + str(y)

_redL = np.array([1,150,60])
_redH = np.array([33,255,255])


frame 	= cv2.imread("1.jpg",-1)
frame	= cv2.resize(frame, (640,480))
frame 	= cv2.GaussianBlur(frame, (5,5),0)
hsv 	= cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
mask	= cv2.inRange(hsv,_redL,_redH)

se1 	= cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
se2 	= cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(1,1))

kernel 	= cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
mask 	= cv2.dilate(mask,kernel,iterations = 1)

mask 	= cv2.morphologyEx(mask, cv2.MORPH_CLOSE, se1)
mask 	= cv2.morphologyEx(mask, cv2.MORPH_OPEN, se2)

moments = cv2.moments(mask)
area	= moments['m00']
__x		= moments['m10']
__y		= moments['m01']

X 		= __x / area
Y 		= __y / area

print "X : " + str(X)
print "Y : " + str(Y)
print "A : " + str(area)

# cv2.cv.SetMouseCallback("result",on_mouse)

if area > 2500000:
	cv2.circle(frame,(int(X),int(Y)), int((area / 100000)), (22,68,196), 2)

cv2.imshow("result",frame)

k = cv2.waitKey(0)



