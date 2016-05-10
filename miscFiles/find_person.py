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
import math

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from manipulation.srv import *
from manipulation.srv import LookForSweets
from collections import OrderedDict

from cv_bridge import CvBridge, CvBridgeError

import tf

from sensor_msgs.msg import Image
import baxter_interface
#from moveit_commander import conversions
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped
from std_msgs.msg import Header
import std_srvs.srv
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

global receivedimage
receivedimage = 0
global backgroundimage

global resetbackground
resetbackground = False

global personFrame
personFrame = 0
global nopersonFrame
nopersonFrame = 0
global personExists
personExists = False
global personStayed
personStayed = False

#Thresholds image and stores position of object in (x,y) coordinates of the camera's frame, with origin at center.
def callback(message):
    global receivedimage
    #Capturing image of web camera
    br = CvBridge()	# Create a black image, a window

    cv_image = br.imgmsg_to_cv2(message, "bgr8")
    cv_image = cv2.GaussianBlur(cv_image,(5,5),0)

    global nopersonFrame
    global resetbackground
    if receivedimage == 0 or resetbackground == True or nopersonFrame == 100:
        global backgroundimage
        backgroundimage = cv_image
        resetbackground = False
        nopersonFrame = 0

    subtracted = cv2.absdiff(backgroundimage, cv_image)

    gray_image = cv2.cvtColor(subtracted, cv2.COLOR_BGR2GRAY)

    ret,thresh1 = cv2.threshold(gray_image,20,255,cv2.THRESH_BINARY)
    thresh1 = cv2.GaussianBlur(thresh1,(5,5),0)

    kernel = np.ones((5,5),np.uint8)
    thresh1 = cv2.dilate(thresh1,kernel,iterations = 4)
    thresh1 = cv2.erode(thresh1, kernel, iterations=4)

    (cnts, _) = cv2.findContours(thresh1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cns = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]

    global personExists
    global personStayed

    for cont in cns:
        if 10000 < cv2.contourArea(cont) < 40000:
            #print cv2.contourArea(cont)
            cv2.drawContours(cv_image, [cont], -1, (0,255,0), 3)
            personExists = True
            break
    if personExists == True:
        if (any(10000 < cv2.contourArea(cont) < 40000 for cont in cns)) == False:
            print "PERSON HAS EXITED FRAME AND DOES NOT WANT SWEETS"
            personExists = False
            personStayed = False



    global personFrame
    if personFrame == 50:
        print "PERSON ENTERED FRAME AND WANTS SWEETS"
        personStayed = True

    cv2.imshow('image', cv_image)
    cv2.imshow('thresh1', thresh1)

    if personExists == True:
        personFrame = personFrame + 1
    if personExists == False:
        personFrame = 0
        nopersonFrame = nopersonFrame + 1

    receivedimage = receivedimage + 1

    k = cv2.waitKey(1)

def handle_person(req):
    global resetbackground
    resetbackground = True
    global personStayed
    while personStayed == False:
        rospy.sleep(1)
    personStayed = False
    return RequestPersonResponse("OK")

def main():

    rospy.init_node('view_head_cam', anonymous = True)

    image_topic = rospy.resolve_name("/cameras/head_camera/image")
    # rospy.Subscriber(image_topic, Image, callback, queue_size = 1, buff_size=int(1024000))
    rospy.Subscriber(image_topic, Image, callback)

    t = rospy.Service('look_for_person', RequestPerson, handle_person)

    #Keep from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
     main()
