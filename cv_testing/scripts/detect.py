#!/usr/bin/env python

import rospy
import numpy as np
import argparse
import cv2
import sys
import yaml
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

#with open('/home/neil/Documents/calibration_cam/cal.yml', 'r') as file:
#	camera_params = yaml.safe_load(file)

dist_pub = rospy.Publisher('/aruco_dist', Float32, queue_size=10)
tb_angle_pub = rospy.Publisher('/tb_angle', Float32, queue_size=10)
aruco_angle_pub = rospy.Publisher('/aruco_angle', Float32, queue_size=10)

averages = []

def callback(data):
	global averages
	cameraMatrix = np.float32([[719.935376, 0, 327.90201], [0, 724.8456159999999, 166.533253], [0, 0, 1.0]]).reshape(3,3)
	distCoeffs = np.float32([0.260116, -0.954027, -0.017563, -0.009861, 0]).reshape(1,5)
	
	#cameraMatrix = np.array((camera_params['camera_matrix']['data']),dtype=np.flat32)
	#distCoeffs = np.array((camera_params['distortion_coefficients']['data']),dtype=CV_32F)
	# Used to convert between ROS and OpenCV images
	br = CvBridge()

	# Output debugging information to the terminal
	#rospy.loginfo("receiving video frame")

	# Convert ROS Image message to OpenCV image
	current_frame = br.compressed_imgmsg_to_cv2(data)
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
		rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.095, cameraMatrix, distCoeffs)
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
		dist_pub.publish(tvec[0][0][2])
		#print("[INFO] ArUco marker Distance: {} metres".format(tvec[0][0][2]))
		#print(np.degrees(rvec[0][0][2]))
		inv_camMatrix = np.linalg.inv(cameraMatrix)
		rc = inv_camMatrix.dot([320, 180, 1.0])
		
		r2 = inv_camMatrix.dot([cX, 180, 1.0])
		cos_angle = rc.dot(r2) / (np.linalg.norm(rc) * np.linalg.norm(r2))
		angle = np.degrees(math.acos(cos_angle))
		if cX > 320:
			angle = -angle
		
		averages.append(rvec[0][0][2])
		if len(averages) == 5:
			aruco_angle = sum(averages)/(len(averages))
			aruco_angle_pub.publish(np.degrees(aruco_angle))
			del averages[:]
		
		tb_angle_pub.publish(angle)
		

		current_frame = cv2.circle(current_frame, (320,180), radius=2, color=(0, 0, 255), thickness=-1)

		#print("[INFO] ArUco marker Orientation: {} degrees".format(angle))
	# show the output image
	cv2.imshow("Image", current_frame)

	cv2.waitKey(1)

def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('aruco_sub_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('/cv_camera/image_raw/compressed', CompressedImage, callback)

  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()
