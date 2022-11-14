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
