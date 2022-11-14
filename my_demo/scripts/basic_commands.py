#!/usr/bin/env python

import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# Class which can be used to issue basic commands to tb3 such as move or turn
class BasicCommands:
	def __init__(self):
		self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.vel = Twist()
		self.rate = rospy.Rate(10)

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
				
			self.vel.linear.x = 0
			self.twist_pub.publish(self.vel)
			self.rate.sleep()	

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
			else:
				rospy.loginfo("Default direction used, none specified or spelling error")
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

