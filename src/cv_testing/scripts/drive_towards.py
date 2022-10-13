#!/usr/bin/env python

import rospy
import numpy as np
import time
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class TurtleBot3(object):
    def __init__(self):
        rospy.init_node('Drive_to_Aruco')
        rospy.on_shutdown(self.shutdownhook)
        self.dist_sub = rospy.Subscriber("/aruco_dist", Float32, self.get_dist)
        self.angle_sub = rospy.Subscriber('/aruco_angle', Float32, self.get_angle)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.get_rotation)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel = Twist()
        self.aruco_dist = 0
        self.aruco_angle = 0
        self.odom = Odometry()
        self.yaw = 0
        self.ctrl_c = False
        print("Drive towards aruco starting now...")
        self.rate = rospy.Rate(10)

    def get_dist(self, msg):
        self.aruco_dist = msg.data

    def get_angle(self, msg):
        self.aruco_angle = msg.data

    def get_rotation(self, msg):
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def drive_towards(self, stop_dist):
        while not rospy.is_shutdown():
            # print(self.aruco_dist)

            turnDirection = self.findTurnDirection()
            if self.aruco_dist > stop_dist:

                self.rotateDegrees(0.1, np.abs(self.aruco_angle), turnDirection)
                self.vel.linear.x = 0.1
                self.twist_pub.publish(self.vel)
                self.rate.sleep()
            else:
                self.vel.linear.x = 0
                self.twist_pub.publish(self.vel)
                self.rate.sleep()
                self.rotateDegrees(0.1, np.abs(self.aruco_angle), turnDirection)
                finished = True
            # print("Waiting for marker to detect")

    def findTurnDirection(self):
        if self.aruco_angle > 1.5:
            # turn left
            return "left"
        elif self.aruco_angle < -1.5:
            # turn right
            return "right"

    def rotateDegrees(self, angular_vel, degrees, direction):
        # Setting the angle to turn by specified degrees
        angle = np.radians(degrees)
        # Setting the angular velocity to the specified velocity
        relative_angle = angle
        if direction == "left":
            self.vel.angular.z = angular_vel
        elif direction == "right":
            self.vel.angular.z = -angular_vel
        # Setting the current time for distance calculus
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
        time.sleep(0.5)

    def shutdownhook(self):
        print("ending...")
        self.ctrl_c = True
        self.vel.angular.z = 0
        self.vel.linear.x = 0
        self.twist_pub.publish(self.vel)
        self.rate.sleep()


if __name__ == '__main__':
    x = TurtleBot3()
    try:
        stop_dist = input("Enter an distance to stop from the marker in metres:")
        x.drive_towards(stop_dist)
    except rospy.ROSInterruptException:
        pass
