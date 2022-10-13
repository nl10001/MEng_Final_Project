#!/usr/bin/env python2

import rospy
import numpy as np
import argparse
import cv2
import cv2.aruco as aruco
import sys

ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", required=True,
	help="path to output image containing ArUCo tag")
ap.add_argument("-i", "--id", type=int, required=True,
	help="ID of ArUCo tag to generate")
args = vars(ap.parse_args())

arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_50)

print("[INFO] generating ArUCo tag type '{}' with ID '{}'".format("DICT_4x4_50", args["id"]))

tag = np.zeros((300, 300, 1), dtype="uint8")
cv2.aruco.drawMarker(arucoDict, args["id"], 300, tag, 1)

cv2.imwrite(args["output"], tag)
cv2.imshow("ArUCo Tag", tag)
cv2.waitKey(0)

