#!/usr/bin/env python

import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler


# Class which can be used to issue basic commands to tb3 such as move or turn
class BasicCommands:
	# Initialising the class variables which will be accesible throughout
	def __init__(self):
		self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
		self.reset_odom = rospy.Publisher('/reset', Empty, queue_size=10)
		rospy.Subscriber('/odom', Odometry, self.get_odom)
		self.vel = Twist()
		self.odom = Odometry()
		self.yaw = 0
		self.rate = rospy.Rate(10)

	# Getting Odometry from odom topic and converting orientation from quaternion
	# to euler angles (roll, pitch and yaw)
	def get_odom(self, msg):
		self.odom = msg
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)

	# Linear move function based on d = vt
	def Move(self, linear_vel, distance):
			current_dist = 0
			t0 = rospy.Time.now().to_sec()
			while current_dist < distance:
				self.vel.linear.x = linear_vel
				self.twist_pub.publish(self.vel)
				t1= rospy.Time.now().to_sec()
				current_dist = np.abs(linear_vel) * (t1 - t0)
				# For Debugging
				#rospy.loginfo("Current distance {}".format(current_dist))
				self.rate.sleep()
			# Force stop tb3
			self.vel.linear.x = 0
			self.twist_pub.publish(self.vel)
			self.rate.sleep()	

	# Angular rotate function similarly based on d = vt
	def RotateDegrees(self, angular_vel, degrees, direction):
			# Setting the angle to turn by specified degrees
			angle = np.radians(degrees)
			# Setting the angular velocity to the specified velocity
			relative_angle = angle
			if direction == "left":
				self.vel.angular.z = angular_vel
			elif direction == "right":
				self.vel.angular.z = -angular_vel
				# Setting the current time for distance calculus
			elif direction is None:
				pass
			else:
				rospy.loginfo("Default left turn used, none specified or spelling error")
				self.vel.angular.z = angular_vel
	
			t0 = rospy.Time.now().to_sec()
			current_angle = 0
			while current_angle < relative_angle:
				self.twist_pub.publish(self.vel)
				t1 = rospy.Time.now().to_sec()
				current_angle = np.abs(angular_vel) * (t1 - t0)
				self.rate.sleep()

			# Forcing Turtlebot to stop
			self.vel.angular.z = 0
			self.twist_pub.publish(self.vel)
			self.rate.sleep()

	# Linear move function based on Odometry
	def odom_move(self, linear_vel, distance):
		curr_x = self.odom.pose.pose.position.x
		curr_y = self.odom.pose.pose.position.y
		current_pos = 0
		while current_pos < distance:
			# Calculate euclidean distance of X and Y displacement
			current_pos = np.sqrt(pow(self.odom.pose.pose.position.x - curr_x, 2) + pow(self.odom.pose.pose.position.y - curr_y, 2))
			self.vel.linear.x = linear_vel
			self.twist_pub.publish(self.vel)
			self.rate.sleep()
		# Force tb3 to stop
		self.vel.linear.x = 0
		self.twist_pub.publish(self.vel)
		self.rate.sleep()

	# Angular move function based on Odometry (NOT WORKING)
	def test_turn(self, angular_vel, degrees, direction):
		
		#self.reset_odom.publish(Empty())
		start_yaw = self.yaw
		angle = np.radians(degrees)
		current_angle = 0
		if direction == "left":
			self.vel.angular.z = angular_vel
		elif direction == "right":
			self.vel.angular.z = -angular_vel
		else:
			rospy.loginfo("Default direction used, none specified or spelling error")
			self.vel.angular.z = angular_vel
		while current_angle < angle:
		
			self.twist_pub.publish(self.vel)
			current_angle = self.yaw - start_yaw
			self.rate.sleep()
					
		self.vel.angular.z = 0
		self.twist_pub.publish(self.vel)
		self.rate.sleep()


		

