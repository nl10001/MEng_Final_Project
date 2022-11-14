#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import math
import basic_commands as bc
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler # Package to convert from quaternions to RPY
from sensor_msgs.msg import CompressedImage # CompressedImage is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

class ActionCards:
	# Constructor to intialise the Class
	def __init__(self):
		rospy.init_node('action_cards')
		rospy.on_shutdown(self.shutdownhook)
		rospy.Subscriber('/cv_camera/image_raw/compressed', CompressedImage, self.detect)
		self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.get_rotation)
		self.odom = Odometry()
		self.vel = Twist()
		self.cmd_tb3 = basic_commands.BasicCommands()
		self.corners = []
		self.ids = 0
		self.yaw = 0
		self.rate = rospy.Rate(10)

	def get_rotation(self, msg):
		self.odom = msg
		global roll, pitch, yaw
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)

	def detect(self,data):
		# Camera matrix and distortion coefficients from camera calibration
		cameraMatrix = np.float32([[719.935376, 0, 327.90201], [0, 724.8456159999999, 166.533253], [0, 0, 1.0]]).reshape(3,3)
		distCoeffs = np.float32([0.260116, -0.954027, -0.017563, -0.009861, 0]).reshape(1,5)
		
		# Used to convert between ROS and OpenCV images
		br = CvBridge()

		# Output debugging information to the terminal
		#rospy.loginfo("receiving video frame")

		# Convert ROS Image message to OpenCV image
		current_frame = br.compressed_imgmsg_to_cv2(data)
		arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
		arucoParams = cv2.aruco.DetectorParameters_create()
		(self.corners, self.ids, rejected) = cv2.aruco.detectMarkers(current_frame, arucoDict,
			parameters=arucoParams)

		# verify *at least* one ArUco marker was detected
		if len(self.corners) > 0:
			# flatten the ArUco IDs list
			self.ids = self.ids.flatten()
			# loop over the detected ArUCo corners
			for (markerCorner, markerID) in zip(self.corners, self.ids):
				# extract the marker corners (which are always returned in
				# top-left, top-right, bottom-right, and bottom-left order)
				self.corners = markerCorner.reshape((4, 2))
				(topLeft, topRight, bottomRight, bottomLeft) = self.corners
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
						
			current_frame = cv2.circle(current_frame, (320,180), radius=2, color=(0, 0, 255), thickness=-1)

		
	# Function to choose which action is executed based on visual input in the form of ArUco marker ID	
	def actions(self):
		try:			
			# Run until shutdown
			while not rospy.is_shutdown():	
				# Make sure there is a marker detected	
				if len(self.corners) > 0:
					if self.ids == 0:
						print("hi")
						rospy.Rate(0.5).sleep()
					elif self.ids == 1:
						self.cmd_tb3.RotateDegrees(0.5, 180, 'left')
					elif self.ids == 2:
						self.cmd_tb3.Move(0.1, 0.1)	
				else:
					# If no markers detected output info to terminal 
					rospy.loginfo("No markers detected")
					rospy.Rate(0.5).sleep()
		except rospy.ROSInterruptException:
			pass

	# Shutdown hook which runs when rospy shuts down (when ctrl+c is hit)
	def shutdownhook(self):
		# Stop the robot, log the shutdown and close any OpenCV windows
		rospy.loginfo("Shutdown")
		cv2.destroyAllWindows()
		self.ctrl_c = True
		self.vel.angular.z = 0
		self.vel.linear.x = 0
		self.twist_pub.publish(self.vel)
		self.rate.sleep()	

if __name__ == '__main__':
	# Create instance of ActionCards
	x = ActionCards()	
	try:
		# Run the method actions() and then close all windows opened by OpenCV
		x.actions()
		cv2.destroyAllWindows()
	except rospy.ROSInterruptException:
		pass

	
