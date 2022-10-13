#! /usr/bin/env python

import rospy 
import time 
import math
import numpy as np
import argparse
import cv2
import sys
import yaml
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

class Turtlebot3(object):
	def __init__(self):
		rospy.init_node('aruco_detect_sub', anonymous=True)
		rospy.on_shutdown(self.shutdownhook)
		self.image_sub = rospy.Subscriber('/cv_camera/image_raw/compressed', CompressedImage, self.get_image)
		self.dist_pub = rospy.Publisher('/aruco_dist', Float32, queue_size=10)
		self.frame = CompressedImage()
		self.ctrl_c = False
		print("Aruco detect starting now...")
		self.rate = rospy.Rate(10)

	def get_image(self,msg):
		self.frame = msg
		# Output debugging information to the terminal
		#rospy.loginfo("receiving video frame")

	def detect_and_drive(self):
		while not rospy.is_shutdown():
			cameraMatrix = np.float32([[719.935376, 0, 327.90201], [0, 724.8456159999999, 166.533253], [0, 0, 1.0]]).reshape(3,3)
			distCoeffs = np.float32([0.260116, -0.954027, -0.017563, -0.009861, 0]).reshape(1,5)
			
			# Used to convert between ROS and OpenCV images
			br = CvBridge()

			# Convert ROS Image message to OpenCV image
      			current_frame = br.compressed_imgmsg_to_cv2(self.frame)
			arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
			arucoParams = cv2.aruco.DetectorParameters_create()
			(corners, ids, rejected) = cv2.aruco.detectMarkers(current_frame, arucoDict,
				parameters=arucoParams)

			# verify *at least* one ArUco marker was detected
			if len(corners) > 0:
				# flatten the ArUco IDs list
				ids = ids.flatten()
				# loop over the detected ArUCo corners
				for (markerCorner, markerID) in zip(corners, ids):
					# extract the marker corners (which are always returned in
					# top-left, top-right, bottom-right, and bottom-left order)
					corners = markerCorner.reshape((4, 2))
					(topLeft, topRight, bottomRight, bottomLeft) = corners
					# convert each of the (x, y)-coordinate pairs to integers
					topRight = (int(topRight[0]), int(topRight[1]))
					bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
					bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
					topLeft = (int(topLeft[0]), int(topLeft[1]))
					rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.1, cameraMatrix, distCoeffs)
				# draw the bounding box of the ArUCo detection
				cv2.line(current_frame, topLeft, topRight, (0, 255, 0), 2)
				cv2.line(current_frame, topRight, bottomRight, (0, 255, 0), 2)
				cv2.line(current_frame, bottomRight, bottomLeft, (0, 255, 0), 2)
				cv2.line(current_frame, bottomLeft, topLeft, (0, 255, 0), 2)
				# compute and draw the center (x, y)-coordinates of the ArUco
				# marker
				cX = int((topLeft[0] + bottomRight[0]) / 2.0)
				cY = int((topLeft[1] + bottomRight[1]) / 2.0)
				cv2.circle(current_frame, (cX, cY), 4, (0, 0, 255), -1)
				# draw the ArUco marker ID on the image
				cv2.putText(current_frame, str(markerID),
					(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
					0.5, (0, 255, 0), 2)
				#print("[INFO] ArUco marker ID: {}".format(markerID))
				#print("[INFO] ArUco marker Distance: {}".format(tvec))
				self.dist_pub.publish(tvec[0][0][2])
				self.rate.sleep()
				print("[INFO] ArUco marker Distance: {} metres".format(tvec[0][0][2]))
			# show the output image
			cv2.imshow("Image", current_frame)

			cv2.waitKey(1)
			

	def shutdownhook(self):
		print("ending...")
		#self.ctrl_c = True
		#self.vel.angular.z = 0
		#self.vel.linear.x = 0
		#self.twist_pub.publish(self.vel)
		#self.rate.sleep()

if __name__ == '__main__':
	x = Turtlebot3()
	try:
		x.detect_and_drive()
	except rospy.ROSInterruptException:
		pass
