#!/usr/bin/env python

import rospy
import numpy as np
import argparse
import cv2
import sys
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

class ArucoBoard:
	def __init__(self):
		rospy.init_node('aruco_board', anonymous=True)
		rospy.on_shutdown(self.shutdownhook)
		rospy.Subscriber('/cv_camera/image_raw/compressed', CompressedImage, self.detect_board)
		self.dist_pub = rospy.Publisher('/aruco_dist', Float32, queue_size=10)
		self.tb_angle_pub = rospy.Publisher('/tb_angle', Float32, queue_size=10)
		self.aruco_angle_pub = rospy.Publisher('/aruco_angle', Float32, queue_size=10)
		self.rate = rospy.Rate(10)

	def detect_board(self, data):
		# Input intrinsic camera matrix and distortion coefficents for use in ArUco functions
		cameraMatrix = np.float32([[719.935376, 0, 327.90201], [0, 724.8456159999999, 166.533253], [0, 0, 1.0]]).reshape(3,3)
		distCoeffs = np.float32([0.260116, -0.954027, -0.017563, -0.009861, 0]).reshape(1,5)

		# Used to convert between ROS and OpenCV images
		br = CvBridge()

		# Output debugging information to the terminal
		#rospy.loginfo("receiving video frame")

		# Convert ROS Image message to OpenCV image
		current_frame = br.compressed_imgmsg_to_cv2(data)
		# Convert image to grayscale 
		gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

		arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
		# Define board with parameters from real life
		board = cv2.aruco.GridBoard_create(3,2,0.079,0.008,arucoDict)
		arucoParams = cv2.aruco.DetectorParameters_create()
		# Detect the markers
		(corners, ids, rejected) = cv2.aruco.detectMarkers(gray, arucoDict,
			parameters=arucoParams)

		# Help refine the detection, identify ID's that might have been part of the board
		corners, ids, rejected, recoveredIds = cv2.aruco.refineDetectedMarkers(
			gray,
		        board,
		        corners,
		        ids,
		        rejected,
		        cameraMatrix,
		        distCoeffs)
		# Check if a marker has been detected
		if len(corners) > 0:
			current_frame = cv2.aruco.drawDetectedMarkers(current_frame, corners, ids)
			# flatten the ArUco IDs list
			ids = ids.flatten()
		
				# Require 15 markers before drawing axis
			if ids is not None and len(ids) >= 6:
				# Estimate the posture of the gridboard, which is a construction of 3D space based on the 2D video 
				_, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs)

				current_frame = cv2.aruco.drawAxis(current_frame, cameraMatrix, distCoeffs, rvec, tvec, 0.1)
				# Publish the relevant data to be used in other files such as drive_towards.py
				self.dist_pub.publish(tvec[2])
				self.aruco_angle_pub.publish(rvec[2])
				#self.rate.sleep()
			else:	
				# If not enough markers are detected then log the info in the terminal
				rospy.loginfo("Board not fully detected")			
			# Check if marker with ID = 1 has been detected
			if 1 in ids:
				# Find the index of marker 1 in the list of ids
				i = np.where(ids == 1)[0][0]
				# Iterate through the corners of marker 1
				for (markerCorner, markerID) in zip(corners[i], ids):
					# extract the marker corners (which are always returned in
					# top-left, top-right, bottom-right, and bottom-left order)
					corners[i] = markerCorner.reshape((4, 2))
					(topLeft, topRight, bottomRight, bottomLeft) = corners[i]
					# convert each of the (x, y)-coordinate pairs to integers
					topRight = (int(topRight[0]), int(topRight[1]))
					bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
					bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
					topLeft = (int(topLeft[0]), int(topLeft[1]))
				cX = int((topLeft[0] + bottomRight[0]) / 2.0)
				cY = int((topLeft[1] + bottomRight[1]) / 2.0)
				# Draw circle in middle of marker 1 to be used for tb3 angle
				cv2.circle(current_frame, (cX, cY), 4, (0, 0, 255), -1)
				cv2.circle(current_frame, (320, 180), 4, (0, 0, 255), -1)
				# Use the vector dot product to calculate the angle between centre of marker 1 and centre of frame
				inv_camMatrix = np.linalg.inv(cameraMatrix)
				rc = inv_camMatrix.dot([320, 180, 1.0])
				r2 = inv_camMatrix.dot([cX, 180, 1.0])
				cos_angle = rc.dot(r2) / (np.linalg.norm(rc) * np.linalg.norm(r2))
				angle = np.degrees(math.acos(cos_angle))
				# If marker 1 is on the right half of the frame then return negative angle
				if cX > 320:
					angle = -angle
				# Publish the angle 
				self.tb_angle_pub.publish(angle)
					
		# Display our image
		cv2.imshow('current_frame', current_frame)
			
		cv2.waitKey(1)

	# Shutdown hook which executes when ctrl-c is pressed
	def shutdownhook(self):
		# Log in the terminal and destroy all open OpenCV windows
		rospy.loginfo("Shutdown")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	# Create an instance of the ArucoBoard class
	x = ArucoBoard()
	try:	
		# rospy.spin() to stop the node from ending early
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
