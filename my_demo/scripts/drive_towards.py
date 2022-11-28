#!/usr/bin/env python

import rospy
import numpy as np
import time
import math
import roslaunch
import basic_commands as bc
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class TurtleBot3(object):
	def __init__(self):
		rospy.init_node('Drive_to_Aruco')
		rospy.on_shutdown(self.shutdownhook)
		self.dist_sub = rospy.Subscriber("/aruco_dist", Float32, self.get_dist)
		self.tb_angle_sub = rospy.Subscriber('/tb_angle', Float32, self.get_tb_angle)
		self.aruco_angle_sub = rospy.Subscriber('/aruco_angle', Float32, self.get_aruco_angle)
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.get_odom)
		self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.get_lasers)
		self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.lasers = 0
		self.vel = Twist()
		self.cmd_tb3 = bc.BasicCommands()
		self.aruco_dist = 0
		self.aruco_angle = 0
		self.tb_angle = 0
		self.odom = Odometry()
		self.yaw = 0
		self.finished = 0
		rospy.loginfo("Starting...")
		self.rate = rospy.Rate(10)

	def get_dist(self, msg):
		self.aruco_dist = msg.data

	def get_tb_angle(self, msg):
		self.tb_angle = msg.data

	def get_aruco_angle(self, msg):
		self.aruco_angle = msg.data

	def get_odom(self, msg):
		self.odom = msg
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)

	def get_lasers(self, msg):
		self.lasers = msg.ranges
	
	def drive_towards(self, aruco_stop_dist, reverse_stop_dist):
		# Execute lineup if the approach angle is greater than 23 degrees
		if (self.aruco_angle > 0.4) or (self.aruco_angle <-0.4):
			self.lineupBasic()
		else:
			pass
		while not rospy.is_shutdown():
			# Find the turn direction towards the marker
			turnDirection = self.findTurnDirection()
			# If finished is at 0 then enter
			if self.finished == 0:
				# Check if current distance to marker is greater than user specified stopping distance
				if self.aruco_dist > aruco_stop_dist:
					# Turn towards marker if needed and drive forwards towards marker
					if turnDirection is not "straight":
						self.cmd_tb3.RotateDegrees(0.05, np.abs(self.tb_angle), turnDirection)
					else:
						self.vel.linear.x = 0.1
						self.twist_pub.publish(self.vel)
						self.rate.sleep()
				# Otherwise stop, turn towards centre of marker and do 180 degree turn
				else:
					self.vel.linear.x = 0
					self.twist_pub.publish(self.vel)
					self.rate.sleep()
					self.cmd_tb3.RotateDegrees(0.1, np.abs(self.tb_angle), turnDirection)
					self.cmd_tb3.RotateDegrees(0.1, 180, "right")
					# Increment finished
					self.finished = self.finished + 1
			# If previous loop entered and completed
			elif self.finished == 1:
				# Reverse to set distance from wall (0.2 minimum)
				self.reverse(0.05, reverse_stop_dist)
				# Increment finished again
				self.finished = self.finished + 1
			# If finished is not 1 or 0 then end
			else:
				print("Finished")
				break

				

	def findTurnDirection(self):
		if self.tb_angle > 1:
			# turn left
			return "left"
		elif self.tb_angle < -1:
			# turn right
			return "right"
		else:
			return "straight"

	#function which reverses at a set speed to a set distance using lidar proximity	taking
	#taking into account the length of the robot
	def reverse(self, linear_vel, distance):
		while self.lasers[175] > (distance + 0.1):
			rospy.loginfo("Reversing...")
			self.vel.linear.x = -linear_vel
			self.twist_pub.publish(self.vel)
			self.rate.sleep()
		self.vel.linear.x = 0
		self.twist_pub.publish(self.vel)
		self.rate.sleep()

	def lineupBasic(self):
		self.cmd_tb3.RotateDegrees(0.05, np.abs(self.tb_angle), self.findTurnDirection())
		ar_angle = self.aruco_angle
		ar_dist = self.aruco_dist
		self.cmd_tb3.RotateDegrees(0.05, 45, "right")
		
		angle = np.radians(135) - np.abs(ar_angle)
		if ar_dist < 1.5:
			dist = math.sin(np.abs(ar_angle))*(ar_dist/math.sin(angle)) - 0.25
		else:
			dist = math.sin(np.abs(ar_angle))*(ar_dist/math.sin(angle)) - 0.65
		print(dist)
		self.cmd_tb3.odom_move(0.05, dist)
		self.vel.linear.x = 0
		self.twist_pub.publish(self.vel)
		self.rate.sleep()
		self.cmd_tb3.RotateDegrees(0.05, 60, "left")
		self.cmd_tb3.RotateDegrees(0.05, np.abs(self.tb_angle), self.findTurnDirection())

	def lineupAdaptive(self):
		self.cmd_tb3.RotateDegrees(0.1, np.abs(self.tb_angle), self.findTurnDirection())
		ar_angle = self.aruco_angle
		ar_dist = self.aruco_dist
		if ar_angle > 0:
			self.cmd_tb3.RotateDegrees(0.1, 45, "left")
		
			angle = np.radians(135 - np.abs(ar_angle))
			dist = math.sin(np.abs(ar_angle))*(ar_dist/math.sin(angle))
			print(dist)
			self.cmd_tb3.odom_move(0.1, dist)
			self.vel.linear.x = 0
			self.twist_pub.publish(self.vel)
			self.rate.sleep()
			self.cmd_tb3.RotateDegrees(0.1, 60, "right")
			self.cmd_tb3.RotateDegrees(0.1, np.abs(self.tb_angle), self.findTurnDirection())
		else:
			self.cmd_tb3.RotateDegrees(0.1, 45, "right")
		
			angle = np.radians(135 - np.abs(ar_angle))
			dist = math.sin(np.abs(ar_angle))*(ar_dist/math.sin(angle))
			print(dist)
			self.cmd_tb3.odom_move(0.1, dist)
			self.vel.linear.x = 0
			self.twist_pub.publish(self.vel)
			self.rate.sleep()
			self.cmd_tb3.RotateDegrees(0.1, 60, "left")
			self.cmd_tb3.RotateDegrees(0.1, np.abs(self.tb_angle), self.findTurnDirection())
	
	def shutdownhook(self):
		rospy.loginfo("Shutdown")
		self.vel.angular.z = 0
		self.vel.linear.x = 0
		self.twist_pub.publish(self.vel)
		self.rate.sleep()		

if __name__ == '__main__':
	# Create instance of turtlebot3
	x = TurtleBot3()
	
	# Add path to arm launch file
	path = "/home/neil/catkin_ws/src/my_demo/launch/my_arm_launch.launch"
	# Create instance of basic commands for moving and turning
	cmd = bc.BasicCommands()
	
	# Configuring roslaunch	
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)
	launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
	
	try:
		# User input for stopping distances
		aruco_stop_dist = input("Enter an distance to stop from the marker in metres:")
		reverse_stop_dist = input("Enter an distance to stop from the wall in metres:")
		
		# Testing #
		#x.lineupBasic()
		#cmd.RotateDegrees(0.1, 90, "right")
		#cmd.RotateDegrees(0.1, 180, "left")
		#cmd.test_turn(0.1, 270, "left")
		#cmd.test_turn(0.1, 90, "right")
		#cmd.odom_move(0.1,0.3)

		# Run main drive towards function
		x.drive_towards(aruco_stop_dist, reverse_stop_dist)
		# Once finished log in terminal that arm is starting and roslaunch launch file for arm
		rospy.loginfo("started arm")
		launch.start()
		rospy.spin()

	except rospy.ROSInterruptException:
		# Once ctrl-c pressed then shutdown arm launch 
		launch.shutdown()
		pass
