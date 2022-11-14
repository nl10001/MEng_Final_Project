#!/usr/bin/env python

import rospy
import numpy as np
import time
import math
import cv2 as cv
from sensor_msgs.msg import CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from matplotlib import pyplot as plt

class Hands(object):
	def __init__(self):
		rospy.init_node('detect_hands')
		rospy.on_shutdown(self.shutdownhook)
		img_sub = rospy.Subscriber('/cv_camera/image_raw/compressed', CompressedImage, self.detect_hands)
		self.current_img = CompressedImage()
		self.rate = rospy.Rate(10)
		self.ctrl_c = False
		
		

	#def get_image(self, msg):
	#	self.current_img = msg
		#img = self.br.compressed_imgmsg_to_cv2(msg)
		#cv.imshow(img)
		

	def detect_hands(self, msg):
		br = CvBridge()
		img = br.compressed_imgmsg_to_cv2(msg)
		hsvim = cv.cvtColor(img, cv.COLOR_BGR2HSV)
		h, s, v = hsvim[:,:,0], hsvim[:,:,1], hsvim[:,:,2]
		lower = np.array([0, 50, 50], dtype = "uint8")
		upper = np.array([50, 255, 255], dtype = "uint8")
		skinRegionHSV = cv.inRange(hsvim, lower, upper)
		blurred = cv.blur(skinRegionHSV, (2,2))
		ret,thresh = cv.threshold(blurred,0,255,cv.THRESH_BINARY)

		#hist_h = cv.calcHist([h],[0],None,[256],[0,256])
		#hist_s = cv.calcHist([s],[0],None,[256],[0,256])
		#hist_v = cv.calcHist([v],[0],None,[256],[0,256])
		#plt.plot(hist_h, color='r', label="hue")
		#plt.show()

		#cv.imshow("thresh", thresh)
		_, contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
		contours = max(contours, key=lambda x: cv.contourArea(x))
		cv.drawContours(img, [contours], -1, (255,255,0), 2)
		#cv.imshow("contours", img)
		hull = cv.convexHull(contours)
		cv.drawContours(img, [hull], -1, (0, 255, 255), 2)
		#cv.imshow("hull", img)
		hull = cv.convexHull(contours, returnPoints=False)
		defects = cv.convexityDefects(contours, hull)
		if defects is not None:
			cnt = 0
		for i in range(defects.shape[0]):  # calculate the angle
			s, e, f, d = defects[i][0]
			start = tuple(contours[s][0])
			end = tuple(contours[e][0])
			far = tuple(contours[f][0])
			a = np.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
			b = np.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
			c = np.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)
			angle = np.arccos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c)) # cosine theorem
		if angle <= np.pi / 2:  # angle less than 90 degree, treat as fingers
			cnt += 1
			cv.circle(img, far, 4, [0, 0, 255], -1)
		if cnt > 0:
			cnt = cnt+1
		cv.putText(img, str(cnt), (0, 50), cv.FONT_HERSHEY_SIMPLEX,1, (255, 0, 0) , 2, cv.LINE_AA)
		cv.imshow('final_result',img)
		cv.waitKey(1)
		
		

	def shutdownhook(self):
		print("ending...")
		self.ctrl_c = True
		cv.destroyAllWindows()


if __name__ == '__main__':
    x = Hands()
    try:
	rospy.spin()
	
    except rospy.ROSInterruptException:
        pass
