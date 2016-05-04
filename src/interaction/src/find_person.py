#!/usr/bin/env python
import rospy
import numpy as np
import cv2

from manipulation.srv import *

from cv_bridge import CvBridge, CvBridgeError

import tf

from sensor_msgs.msg import Image

class PersonFinder:
    def __init__(self):
        # Initialise variables used to keep track of whether a person is in
        # front of the camera or not
        self.receivedImage = 0
        self.resetBackground = False
        self.personFrame = 0
        self.nopersonFrame = 0
        self.personExists = False
        self.personStayed = False

        # Subscribe to the head camera image
        image_topic = rospy.resolve_name("/cameras/head_camera/image")

        # Create an image subscriber using the personFinder's search_for_person
        # technique
        rospy.Subscriber(image_topic, Image, self.search_for_person)


    def search_for_person(self, message):
        # Convert the received message from the camera to a bgr image
        br = CvBridge()
        cv_image = br.imgmsg_to_cv2(message, "bgr8")
        cv_image = cv2.GaussianBlur(cv_image,(5,5),0)

        # If this is the initial setup or a new background needs to be taken
        # for differencing, then reset the background image and variables
        if self.receivedImage == 0 or self.resetbackground == True or \
           self.nopersonFrame == 100:
            self.backgroundimage = cv_image
            self.resetbackground = False
            self.nopersonFrame = 0

        # Create an inverse grey image of the background and perform absolute
        # differencing between the background and the current frame
        subtracted = cv2.absdiff(self.backgroundimage, cv_image)
        gray_image = cv2.cvtColor(subtracted, cv2.COLOR_BGR2GRAY)

        # Threshold and process the image to dilate and erode to reduce noise
        ret,thresh1 = cv2.threshold(gray_image,20,255,cv2.THRESH_BINARY)
        thresh1 = cv2.GaussianBlur(thresh1,(5,5),0)
        kernel = np.ones((5,5),np.uint8)
        thresh1 = cv2.dilate(thresh1,kernel,iterations = 4)
        thresh1 = cv2.erode(thresh1, kernel, iterations=4)
        dilateanderode = cv2.erode(thresh1, kernel, iterations=4)

        # Find contours of the differenced image
        (cnts, _) = cv2.findContours(dilateanderode, cv2.RETR_EXTERNAL, \
                                                cv2.CHAIN_APPROX_SIMPLE)
        cns = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]

        # Search through the contours found in the image and find a
        # person-sized contour (eliminating small/large ones)
        for cont in cns:
            if 10000 < cv2.contourArea(cont) < 40000:
                cv2.drawContours(cv_image, [cont], -1, (0,255,0), 3)
                self.personExists = True
                break
        # Check if person exists and then if they have exited the frame recently
        if self.personExists == True:
            # Checks if the person still exists
            if (any(15000 < cv2.contourArea(cont) < 40000 for cont in cns)) == False:
                #print "PERSON HAS EXITED FRAME AND DOES NOT WANT SWEETS"
                self.personExists = False
                self.personStayed = False

        # If person has been there for 50 frames then they want some sweets
        if self.personFrame == 50:
            #print "PERSON ENTERED FRAME AND WANTS SWEETS"
            self.personStayed = True

        # If the person exists, add another frame to the number of frames
        # they have existed for
        if self.personExists == True:
            self.personFrame = self.personFrame + 1

        # If they don't exist, reset the person exists frame number and count
        if self.personExists == False:
            self.personFrame = 0
            self.nopersonFrame = self.nopersonFrame + 1

        # Keep track of the receivedImage number
        self.receivedImage = self.receivedImage + 1

        # Uncomment lines to show the current recognised person contour
        # cv2.imshow('image', cv_image)
        # cv2.imshow('thresh1', thresh1)
        # k = cv2.waitKey(1)

    # A callback function used when the main shopkeeper node requests to wait
    # for a customer
    def person_request(self, req):
        # Reset background differencing on request
        self.resetbackground = True
        # Look for person to enter for 50 frames and wait until they do
        while self.personStayed == False:
            rospy.sleep(1)
        self.personStayed = False
        # Send a response back saying they have stayed in the view
        return RequestPersonResponse("OK")

# Main function
if __name__ == '__main__':
     # Create a person finder object
     personFinder = PersonFinder()

     # Initialise the node to find the person
     rospy.init_node('view_head_cam', anonymous = True)

     # Create a service to wait for a request from the shopkeeper to look for a
     # person
     t = rospy.Service('look_for_person', RequestPerson, personFinder.person_request)

     # Keep from exiting until this node is stopped
     rospy.spin()
