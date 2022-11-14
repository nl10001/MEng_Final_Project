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


arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

grid_board = cv2.aruco.GridBoard_create(3,
                                        2,
                                        0.08,
                                        0.01,
                                        arucoDict)

img = grid_board.draw((1920, 1080), 10, 1)
cv2.imwrite('board.png', img)
cv2.imshow("ArUCo Tag", img)
cv2.waitKey(0)




