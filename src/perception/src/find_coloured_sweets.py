#!/usr/bin/env python

# data for blue Sweets
# 1,48,139,37,255,255,255,6

# data for green sweets
# 1,46,61,0,106,198,218,5
#The structure of this file was taken from the baxter_stocking_stuffer project by students in Northwestern's MSR program - Josh Marino, Sabeen Admani, Andrew Turchina and Chu-Chu Igbokwe
# Message published - opencv/center_of_object - contains x,y,z coordinates as a Point message

import rospy
import numpy as np
import cv2
import baxter_interface

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
import baxter_interface
#from moveit_commander import conversions
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import std_srvs.srv
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

#Create publisher to publish center of object detected
pub = rospy.Publisher('opencv/center_of_object', Point, queue_size = 1)

imageNumber = 1
sweetArea = 0
backgroundImage = 0

#Thresholds image and stores position of object in (x,y) coordinates of the camera's frame, with origin at center.
def callback(message):
    #Capturing image of web camera
    br = CvBridge()	# Create a black image, a window

    #img = br.imgmsg_to_cv2(message, desired_encoding="passthrough")
    cv_image = br.imgmsg_to_cv2(message, "bgr8")
    # Take each frame
    simg = cv2.GaussianBlur(cv_image,(5,5),0)
    # find sweet rectangular area for sweets
    sweetarea = find_background(simg)
    # Convert BGR to HSV
    hsv = cv2.cvtColor(sweetarea, cv2.COLOR_BGR2HSV)

    bluemask, bluecnts = find_sweets(hsv, "blue",48, 139, 37, 255, 255, 255, 100, 6)
    # 1,66,54,0,106,182,113,1
    greenmask, greencnts = find_sweets(hsv, "green", 66, 54, 0, 106, 182, 113, 50, 6)
    # 1,142,0,70,200,160,255,7
    pinkmask, pinkcnts = find_sweets(hsv, "pink", 134, 61, 46, 199, 255, 178, 10, 12)

    cv2.drawContours(cv_image, bluecnts, -1, (255,0,0), 3)
    cv2.drawContours(cv_image, greencnts, -1, (0,255,0), 3)
    cv2.drawContours(cv_image, pinkcnts, -1, (255,204,255), 3)

    cv2.imshow("Sweets found", cv_image)
    #cv2.imshow("Sweets", cv_copy)

    cv2.waitKey(3)

def find_background(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #gray = cv2.bilateralFilter(hsv, 11, 17, 17)
    edges = cv2.Canny(hsv,30,200,apertureSize = 3)
    # DILATE EDGE LINES
    kernel = np.ones((5,5),np.uint8)
    dilation = cv2.dilate(edges,kernel,iterations = 1)
    # FIND 10 LARGEST CONTOURS IN IMAGE
    (cnts, _) = cv2.findContours(dilation, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cns = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
    mask = np.zeros(img.shape[:2], dtype="uint8") * 25

    # FIND RECTANGLE IN IMAGE AND SEGMENT - WHAT IF MULTIPLE RECTANGLES?
    for cont in cns:
        peri = cv2.arcLength(cont, True)
        approx = cv2.approxPolyDP(cont, 0.02 * peri, True)
        if len(approx) == 4:
            print("square is found")
            cv2.drawContours(mask, [cont], -1, 1, -1)
            break

    img = cv2.bitwise_and(img, img, mask=mask)
    cv2.imshow("Rectangular area", dilation)
    cv2.imshow("Edges", img)

    return img

def find_sweets(hsv, colour, r1, g1, b1, r2, g2, b2, area, size):
    lower = np.array([r1,g1,b1])
    upper = np.array([r2,g2,b2])

    mask = cv2.inRange(hsv, lower, upper)

    # filter and fill the mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(size, size))
    mask2 = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)

    contour,hier = cv2.findContours(mask2,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    sweetCount = 0
    sweetcontours = []
    for cnt in contour:
        if (cv2.contourArea(cnt) > area):
            cv2.drawContours(mask2, [cnt], 0, 255, 3)
            sweetcontours.append(cnt)
            sweetCount = sweetCount + 1

    print "There are "+str(sweetCount)+" "+colour+" sweets"

    return mask2, sweetcontours


#Subscribes to left hand camera image feed
def main():

    rospy.init_node('view_sweet_cam', anonymous = True)
    image_topic = rospy.resolve_name("/cameras/right_hand_camera/image")
    #depth_topic = rospy.resolve_name("/camera/depth_registered/image_raw")

    rospy.Subscriber(image_topic, Image, callback)


    #Keep from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
     main()
