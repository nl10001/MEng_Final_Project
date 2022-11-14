#!/usr/bin/env python

import rospy
import numpy as np
import time
import math
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Joy

class Arm(object):
	def __init__(self):
		rospy.init_node('arm')
		rospy.on_shutdown(self.shutdownhook)
		rospy.loginfo("Please wait to enter the mode of operation.")
		self.arm_pub = rospy.Publisher('/joint_array', UInt8MultiArray, queue_size=10)
		self.joy_sub = rospy.Subscriber('/joy', Joy, self.get_joy)
		self.joy = Joy()
		self.ctrl_c = False
		self.arm_array = UInt8MultiArray()
		self.rate = rospy.Rate(10)

	def get_joy(self,msg):
		# Getting controller states
		self.joy = msg

	# Function for controlling the arm manually with the controller
	# Primarily used for testing
	def move_manual(self):
		self.arm_array.data = [90,90,90,90,80,73]
		current_pos = self.arm_array.data
		while not rospy.is_shutdown():
			# Checking if the /joy/buttons topic has been published to
			if len(self.joy.buttons) > 0:
				# Assigning the axes and buttons values to corresponding buttons
				dpad_LR = self.joy.axes[6]
				dpad_UD = self.joy.axes[7]
				right_trigger = self.joy.axes[2]
				left_trigger = self.joy.axes[5]
				a = self.joy.buttons[0]
				b = self.joy.buttons[1]
				x = self.joy.buttons[2]
				y = self.joy.buttons[3]
				xb = self.joy.buttons[8]
				#dpad
				if dpad_LR == -1:
					if current_pos[0] < 180: 
						self.arm_array.data[0] = current_pos[0] + 5
						self.arm_pub.publish(self.arm_array)
						current_pos = self.arm_array.data
						self.rate.sleep() 
					else:
						rospy.loginfo("Max reached. Turn Left")
						rospy.Rate(1).sleep()
				elif dpad_LR == 1:
					if current_pos[0] > 0: 
						self.arm_array.data[0] = current_pos[0] - 5
						self.arm_pub.publish(self.arm_array)
						current_pos = self.arm_array.data
						self.rate.sleep() 
					else:
						rospy.loginfo("Max reached. Turn right")
						rospy.Rate(1).sleep()
				if dpad_UD == -1:
					if current_pos[1] < 165: 
						self.arm_array.data[1] = current_pos[1] + 5
						self.arm_pub.publish(self.arm_array)
						current_pos = self.arm_array.data
						self.rate.sleep() 
					else:
						rospy.loginfo("Max reached. Turn Down")
						rospy.Rate(1).sleep()
				elif dpad_UD == 1:
					if current_pos[1] > 0: 
						self.arm_array.data[1] = current_pos[1] - 5
						self.arm_pub.publish(self.arm_array)
						current_pos = self.arm_array.data
						self.rate.sleep() 
					else:
						rospy.loginfo("Max reached. Turn Up")
						rospy.Rate(1).sleep()
				#triggers
				elif right_trigger == -1:
					self.arm_array.data[5] = 10
					self.arm_pub.publish(self.arm_array)
					current_pos = self.arm_array.data
					self.rate.sleep()
				elif left_trigger == -1:
					self.arm_array.data[5] = 73
					self.arm_pub.publish(self.arm_array)
					current_pos = self.arm_array.data
					self.rate.sleep()
				#buttons
				elif a == 1:
					self.arm_array.data = [current_pos[0],90,45,70,100,73]
					self.arm_pub.publish(self.arm_array)
					current_pos = self.arm_array.data
					self.rate.sleep()
				elif b == 1:
					self.arm_array.data = [current_pos[0],60,35,90,100,73]
					self.arm_pub.publish(self.arm_array)
					current_pos = self.arm_array.data
					self.rate.sleep()
				elif x == 1:
					self.arm_array.data = [current_pos[0],70,65,40,100,10]
					self.arm_pub.publish(self.arm_array)
					current_pos = self.arm_array.data
					self.rate.sleep()
				elif y == 1:
					self.arm_array.data = [current_pos[0],110,0,70,100,73]
					self.arm_pub.publish(self.arm_array)
					current_pos = self.arm_array.data
					self.rate.sleep()
				elif xb == 1:
					self.arm_array.data = [90,90,90,90,100,73]
					self.arm_pub.publish(self.arm_array)
					current_pos = self.arm_array.data
					self.rate.sleep()

	# Function for moving the arm automatically
	def move_automatic(self):
		#Right
		self.arm_array.data = [180,90,90,90,110,73]
		self.arm_pub.publish(self.arm_array)
		self.rate.sleep()
		rospy.sleep(3)
		#A + LT
		self.arm_array.data = [180,90,45,70,110,10]
		self.arm_pub.publish(self.arm_array)
		self.rate.sleep()
		rospy.sleep(3)
		#B + LT
		self.arm_array.data = [180,60,35,90,110,10]
		self.arm_pub.publish(self.arm_array)
		self.rate.sleep()
		rospy.sleep(3)
		#B
		self.arm_array.data = [180,60,35,90,110,73]
		self.arm_pub.publish(self.arm_array)
		self.rate.sleep()
		rospy.sleep(3)
		#A
		self.arm_array.data = [180,90,45,70,110,73]
		self.arm_pub.publish(self.arm_array)
		self.rate.sleep()
		rospy.sleep(3)
		#A + Left
		self.arm_array.data = [90,90,45,70,110,73]
		self.arm_pub.publish(self.arm_array)
		self.rate.sleep()
		rospy.sleep(3)
		#Left
		#self.arm_array.data = [90,90,45,70,110,73]
		#self.arm_pub.publish(self.arm_array)
		#self.rate.sleep()
		#rospy.Rate(0.1).sleep()
		#Y
		self.arm_array.data = [90,110,0,70,110,73]
		self.arm_pub.publish(self.arm_array)
		self.rate.sleep()
		rospy.sleep(3)
		#New
		self.arm_array.data = [90,15,0,170,110,73]
		self.arm_pub.publish(self.arm_array)
		self.rate.sleep()
		rospy.sleep(3)
		#LT
		self.arm_array.data = [90,15,0,170,110,10]
		self.arm_pub.publish(self.arm_array)
		self.rate.sleep()
		rospy.sleep(3)

	# Test function for getting the right arm poses 	
	def try_me(self):
		self.arm_array.data = [90,90,90,90,110,73]
		self.arm_pub.publish(self.arm_array)
		self.rate.sleep()	
		
	# Shutdown hook which runs when ctrl+c is pressed
	def shutdownhook(self):
		# Log shutdown in terminal and move arm back to default pose
		rospy.loginfo("Shutdown")
		self.arm_array.data = [90,90,90,90,110,73]
		self.arm_pub.publish(self.arm_array)
		self.rate.sleep()
		self.ctrl_c = True

if __name__ == '__main__':
	# Create instance of Arm
	x = Arm()
	rospy.sleep(5)
	try:
		# Take user input to determine mode of operation

		#config = raw_input("Please enter a mode of operation, automatic or manual:")
		config="automatic"	
		if config == "manual":
			x.move_manual()
		if config == "automatic":
			x.move_automatic()
			rospy.loginfo("Finished")
			x.shutdownhook()
		else:
			rospy.loginfo("Mode not recognised, please check spelling.")
			#x.try_m()	
		#rospy.spin()

	except rospy.ROSInterruptException:
		pass
