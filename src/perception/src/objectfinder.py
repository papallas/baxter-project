#! /usr/bin/env python2.7

# this node is used to save the colors of the different objects so that we can track them later
import roslib
#roslib.load_manifest('graphs')
import sys
import inspect, os
from optparse import OptionParser
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv
import cv2
import numpy as np
#import scipy.ndimage.morphology as morphology


#--------------------------------------------------------------------------------------#
global x,y,z,rgb_flag,f,obj_count,r1,b1,g1,r2,b2,g2,size
x = 0
y = 0
z = 0
rgb_flag = 0
r1=0
b1=0
g1=0
r2=0
b2=0
g2=0
size=1
directory = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
f = open(directory+'/objects.txt', 'w')
obj_count = 1


#--------------------------------------------------------------------------------------#
if __name__ == '__main__':

#--------------------------------------------------------------------------------------#
    def nothing(x):
	    pass

#--------------------------------------------------------------------------------------#
    def obj(x):
	global f,obj_count,r1,b1,g1,r2,b2,g2,size
	if cv2.getTrackbarPos('object','toolbar')==1:
		f.write(str(obj_count)+','+str(r1)+','+str(b1)+','+str(g1)+','+str(r2)+','+str(b2)+','+str(g2)+','+str(size)+'\n')
		print 'object'+str(obj_count)+' is saved'
		obj_count = obj_count + 1
	cv2.setTrackbarPos('object','toolbar',0)


#--------------------------------------------------------------------------------------#
    def finish(x):
	global f
	f.write('END')
	f.close()
	cv2.destroyAllWindows()

    #pkgdir = roslib.packages.get_pkg_dir("opencv2")
    #haarfile = os.path.join(pkgdir, "opencv/share/opencv/haarcascades/haarcascade_frontalface_alt.xml")
    br = CvBridge()	# Create a black image, a window

#--------------------------------------------------------------------------------------#
    def detect_and_draw(imgmsg):
	global x,y,rgb_flag,r1,b1,g1,r2,b2,g2,size

        img = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")

    	# Take each frame
    	simg = cv2.GaussianBlur(img,(5,5),0)

    	# Convert BGR to HSV
    	hsv = cv2.cvtColor(simg, cv2.COLOR_BGR2HSV)
	#1
    	#lower_blue = np.array([8,185,114])
    	#upper_blue = np.array([13,249,169])

    	# get current positions of four trackbars
    	r1 = cv2.getTrackbarPos('R1','toolbar')
    	g1 = cv2.getTrackbarPos('G1','toolbar')
    	b1 = cv2.getTrackbarPos('B1','toolbar')
    	r2 = cv2.getTrackbarPos('R2','toolbar')
    	g2 = cv2.getTrackbarPos('G2','toolbar')
    	b2 = cv2.getTrackbarPos('B2','toolbar')
    	size = cv2.getTrackbarPos('Size','toolbar')
	if size<1:
		size = 1

    	lower_blue = np.array([r1,b1,g1])
    	upper_blue = np.array([r2,b2,g2])


    	# Threshold the HSV image to get only orange colors
    	mask = cv2.inRange(hsv, lower_blue, upper_blue)

	# filter and fill the mask
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(size,size))
	mask2 = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)

	moments = cv2.moments(mask2)
	area = moments['m00']
	#there can be noise in the video so ignore objects with small areas
	if(area > 10000):
	    #determine the x and y coordinates of the center of the object
	    #we are tracking by dividing the 1, 0 and 0, 1 moments by the area
	    x = moments['m10'] / area
	    y = moments['m01'] / area
	    x = int(x)
	    y = int(y)
	    cv2.line(hsv,(x,0),(x,480),(255,50,100),3)
	    cv2.line(hsv,(0,y),(640,y),(255,50,100),3)
	    rgb_flag = 1

    	cv2.imshow('RGB',hsv)
    	cv2.imshow('Mask',mask2)

        k = cv2.waitKey(5) & 0xFF

#--------------------------------------------------------------------------------------#
    cv2.namedWindow('toolbar', flags=0)
    cv2.moveWindow('toolbar', 1000, 0)
    # create trackbars for color change
    cv2.createTrackbar('R1','toolbar',0,255,nothing)
    cv2.createTrackbar('G1','toolbar',0,255,nothing)
    cv2.createTrackbar('B1','toolbar',0,255,nothing)
    cv2.createTrackbar('R2','toolbar',0,255,nothing)
    cv2.createTrackbar('G2','toolbar',0,255,nothing)
    cv2.createTrackbar('B2','toolbar',0,255,nothing)
    cv2.createTrackbar('Size','toolbar',1,30,nothing)
    cv2.createTrackbar('object','toolbar',0,1,obj)
    cv2.createTrackbar('finish','toolbar',0,1,finish)

    rospy.init_node('rosColordetect')
    image_topic = rospy.resolve_name("/cameras/right_hand_camera/image")
    #depth_topic = rospy.resolve_name("/camera/depth_registered/image_raw")

    rospy.Subscriber(image_topic, sensor_msgs.msg.Image, detect_and_draw)
    #rospy.Subscriber(depth_topic, sensor_msgs.msg.Image, depth_calculation)
    #talker()

    rospy.spin()
