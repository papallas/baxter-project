#!/usr/bin/env python

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
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped
from std_msgs.msg import Header
import std_srvs.srv
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

sweetArea = 0
backgroundImage = 0

global totalSweets
global centrelist
global PAGECENTREPOINT
PAGECENTREPOINT = 0

# RED, GREEN, BLUE
global colourAmounts

global sweetarea
global foundsquare

global enableAnalyse
enableAnalyse = True

global pointlist
global tl

global anglelist

global pageCentre

global sweetMask
sweetMask = "nil"

#Thresholds image and stores position of object in (x,y) coordinates of the camera's frame, with origin at center.
def callback(message):
    # Weights retrieved from neural network for RGB recognition
    greenWeights = np.array([366.1219, -463.8519, 63.0028, 8.5000])
    blueWeights = np.array([-42.2808, 25.3629, 30.6613, 6.7000])
    redWeights = np.array([3.4025, 11.9616, -17.8766, 6.9000])

    #Capturing image of web camera
    br = CvBridge()	# Create a black image, a window

    #img = br.imgmsg_to_cv2(message, desired_encoding="passthrough")
    cv_image = br.imgmsg_to_cv2(message, "bgr8")
    # Take each frame
    simg = cv2.GaussianBlur(cv_image,(5,5),0)

    # Initialise/retrieve stored global variables to keep track of vision items
    global PAGECENTREPOINT, sweetarea, foundsquare, pageCentre, tl, sweetMask, colourAmounts

    # If the vision system has been told to analyse the sweets on the table
    if enableAnalyse == True:
        # Retrieve the transformation between the hand and torso, for use when
        # transforming the sweet locations later
        tl = tf.TransformListener()
        try:
            tl.waitForTransform("right_hand_camera", "torso", rospy.Time(0), \
                                                            rospy.Duration(10))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        # Find the rectangular area for the sweets if the system hasn't already
        # found it
        foundsquare = False
        if (sweetMask == "nil"):
            sweetMask, areafound = find_background(simg)

        # Create a mask, masking off all on the image outside the sweet area
        sweetarea = cv2.bitwise_and(simg, simg, mask=sweetMask)

        # On that image, convert to gray and do some edge detection/dilation
        # to get clear edges for the sweets/area
        circleimg = cv2.cvtColor(sweetarea, cv2.COLOR_BGR2GRAY);
        edges = cv2.Canny(circleimg,50,200,apertureSize = 3)
        kernel = np.ones((5,5),np.uint8)
        dilation = cv2.dilate(edges,kernel,iterations = 1)

        # Find contours in the image
        contours,hier = cv2.findContours(dilation,cv2.RETR_LIST,\
                                                        cv2.CHAIN_APPROX_SIMPLE)

        # Sweets near the edge of the page/the edge of the page produces
        # unwanted contours
        mask = np.zeros(circleimg.shape,dtype="uint8")
        centrepoints = []
        # Dilate the page border and create a mask to only look 2cm in from the
        # page border
        for cont in contours:
            area = cv2.contourArea(cont)
            # cut out large border of page contours
            if area < 50000:
                cv2.drawContours(mask, [cont], -1, 255, -1)
        mask = cv2.erode(mask, None, iterations=1)

        # On this mask, now find the actual sweet contours
        sweetcontours,newhier = cv2.findContours(mask,cv2.RETR_EXTERNAL,\
                                                        cv2.CHAIN_APPROX_SIMPLE)

        # Initialise global variable to store number of R,G and B colours found
        colourAmounts = [0,0,0]

        # Initialise lists to store RGB centres and angles
        redcentres = []
        redangles = []
        greencentres = []
        greenangles = []
        bluecentres = []
        blueangles = []

        # Initialise list used to store singulated sweets from a large pile
        splitcontours = []

        # Cycling through the contours on the page
        for cont in sweetcontours:
            # Get the area of the contour
            area = cv2.contourArea(cont)

            # Only get the larger contours with more than one sweet stuck
            # together
            if 1500 < area:
                #############################################################
                #                PROCESSING COLLISION PROCESS               #
                #############################################################

                # Get the bounding rectangle for the sweet pile
                x,y,w,h = cv2.boundingRect(cont)

                # Size of the squares for analysis
                size = 12
                # Convert the bounding box into a grid of columns and rows
                colnum = math.ceil(w/size)+1
                rownum = math.ceil(h/size)+1
                # Use this matrix to store the detected colour of each square
                COLORMATRIX = np.zeros((rownum, colnum))

                # Indexes used to keep track of row/column number in the loop
                a = 0
                b = 0

                # Cycle over the bounding box grid
                for i in range(x, x+w, size):
                    for j in range(y, y+h, size):
                        # Mask the current square from the rest
                        rectmask = np.zeros(circleimg.shape,dtype="uint8")
                        cv2.rectangle(rectmask,(i,j),(i+size,j+size),(255,255,255),-1)
                        colourimage = cv2.bitwise_and(cv_image, cv_image, \
                                                                  mask=rectmask)

                        # Do some binary thresholding and find a contour within
                        # that square
                        ret,thresh5 = cv2.threshold(colourimage,127,255,\
                                                          cv2.THRESH_TOZERO_INV)
                        edges = cv2.Canny(thresh5,50,200,apertureSize = 3)
                        dilation = cv2.dilate(edges,kernel,iterations = 1)
                        contours,hier = cv2.findContours(dilation,cv2.RETR_EXTERNAL,\
                                                        cv2.CHAIN_APPROX_SIMPLE)

                        # For the contour in that square
                        for k in range(0,len(contours)):
                            # Create a mask of it that contour
                            contmask = np.zeros(circleimg.shape,dtype="uint8")
                            cv2.drawContours(contmask, [contours[k]], -1, 1, -1)
                            # Get the mean RGB value of that contour
                            mean = cv2.mean(cv_image, mask=contmask)[:3]
                            meanarray = np.array([mean[0], mean[1], mean[2], 1])

                            # Use the three perceptrons to decide whether the
                            # dominant colour is red, green or blue
                            if np.dot(greenWeights,meanarray) < 0:
                                COLORMATRIX[a][b] = 2
                            elif np.dot(blueWeights,meanarray) < 0:
                                COLORMATRIX[a][b] = 3
                            elif np.dot(redWeights,meanarray) < 0:
                                COLORMATRIX[a][b] = 1

                        a = a + 1
                    a = 0
                    b = b + 1

                # Now we have a grid matrix showing the colours in the contour
                # For example: a 1. 2 and 3 sweet with overlaps
                #  | 0 0 0 0 0 0 2 0 0 |
                #  | 1 1 0 0 2 2 2 2 3 |
                #  | 0 1 1 2 2 0 0 3 3 |
                #  | 0 0 1 1 0 0 0 1 3 |

                # A custom region growing algorithm is done over this matrix,
                # cutting out any colour anomalies by looking at the neighbouring
                # colour values
                REGIONMATRIX = np.zeros((rownum, colnum))
                # Loop over the matrix and check the neighbours in vertical/
                # horizontal and diagonal directions (hard coded below)
                for j in range(0, int(colnum)):
                    for i in range(0, int(rownum)):
                        currItem = COLORMATRIX[i][j]
                        sameNeighbours = 0
                        if currItem != 0:
                            if (i - 1) >= 0:
                                if COLORMATRIX[i-1][j] == currItem:
                                    sameNeighbours = sameNeighbours + 1
                                if (j - 1) >= 0:
                                    if COLORMATRIX[i-1][j-1] == currItem:
                                        sameNeighbours = sameNeighbours + 1
                                if (j + 1) <= (colnum-1):
                                    if COLORMATRIX[i-1][j+1] == currItem:
                                        sameNeighbours = sameNeighbours + 1
                            if (i + 1) <= (rownum-1):
                                if COLORMATRIX[i-1][j] == currItem:
                                    sameNeighbours = sameNeighbours + 1
                                if (j - 1) >= 0:
                                    if COLORMATRIX[i+1][j] == currItem:
                                        sameNeighbours = sameNeighbours + 1
                                if (j + 1) <= (colnum-1):
                                    if COLORMATRIX[i+1][j+1] == currItem:
                                        sameNeighbours = sameNeighbours + 1
                            if (j - 1) >= 0:
                                if COLORMATRIX[i][j-1] == currItem:
                                    sameNeighbours = sameNeighbours + 1
                            if (j + 1) <= (colnum-1):
                                if COLORMATRIX[i][j+1] == currItem:
                                    sameNeighbours = sameNeighbours + 1
                        # If there are more than two neighbours of the same
                        # colour, it is not an anomaly
                        if sameNeighbours >=2:
                            REGIONMATRIX[i][j] = currItem

                # After the regions have been grown, create masks from the
                # blue, red and green values in the region matrix
                bluemask = np.zeros(circleimg.shape,dtype="uint8")
                greenmask = np.zeros(circleimg.shape,dtype="uint8")
                redmask = np.zeros(circleimg.shape,dtype="uint8")

                for j in range(0, int(colnum)):
                    for i in range(0, int(rownum)):
                        if (REGIONMATRIX[i][j]) == 3:
                            cv2.rectangle(bluemask,(x + (j*size),y + (i*size)),\
                                    (x + (j*size)+size,y + (i*size)+size),1,-1)
                        if (REGIONMATRIX[i][j]) == 2:
                            cv2.rectangle(greenmask,(x + (j*size),y + (i*size)),\
                            (x + (j*size)+size,y + (i*size)+size),(255,255,255),-1)
                        if (REGIONMATRIX[i][j]) == 1:
                            cv2.rectangle(redmask,(x + (j*size),y + (i*size)),\
                            (x + (j*size)+size,y + (i*size)+size),(255,255,255),-1)

                # Make an image using the detected blue areas
                blueImage = cv2.bitwise_and(cv_image, cv_image, mask=bluemask)
                # Do pre processing on that area to detect sweets
                ret,thresh5 = cv2.threshold(blueImage,127,255,cv2.THRESH_TOZERO_INV)
                edges = cv2.Canny(thresh5,50,200,apertureSize = 3)
                dilation = cv2.dilate(edges,kernel,iterations = 1)
                dilation = cv2.erode(dilation,kernel,iterations = 1)
                # Find the blue sweets within that mask
                contours,hier = cv2.findContours(dilation,cv2.RETR_EXTERNAL,\
                                                        cv2.CHAIN_APPROX_SIMPLE)
                # For each contour attempting to find blue sweets
                for cont in contours:
                    if cv2.contourArea(cont) > 400:
                        # Create a convex hull of the contour
                        hull = cv2.convexHull(cont)
                        hullmask = np.zeros(circleimg.shape,dtype="uint8")
                        cv2.drawContours(hullmask, [hull], -1, 1, -1)
                        hullimage = cv2.bitwise_and(cv_image, cv_image, mask=hullmask)

                        ret,thresh5 = cv2.threshold(hullimage,127,255,cv2.THRESH_TOZERO_INV)
                        edges = cv2.Canny(thresh5,50,200,apertureSize = 3)
                        dilation = cv2.dilate(edges,kernel,iterations = 1)
                        dilation = cv2.erode(dilation,kernel,iterations = 1)
                        # Find the outer contours to find the best, largest
                        # sweet contour possible
                        hullcontours,hier = cv2.findContours(dilation,\
                                      cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                        # Draw to the screen and append the info as a blue sweet
                        for cont in hullcontours:
                            if cv2.contourArea(cont) > 500:
                                cv2.drawContours(cv_image, [cont], -1, (255, 0, 0), 3)
                                splitcontours.append([cont, "blue"])

                ##################################################
                # DO THE SAME FUNCTIONS FOR GREEN SWEETS         #
                ##################################################
                greenImage = cv2.bitwise_and(cv_image, cv_image, mask=greenmask)

                ret,thresh5 = cv2.threshold(greenImage,127,255,cv2.THRESH_TOZERO_INV)
                edges = cv2.Canny(thresh5,50,200,apertureSize = 3)
                dilation = cv2.dilate(edges,kernel,iterations = 1)
                dilation = cv2.erode(dilation,kernel,iterations = 1)

                contours,hier = cv2.findContours(dilation,cv2.RETR_EXTERNAL,\
                                                        cv2.CHAIN_APPROX_SIMPLE)
                cns = sorted(contours, key = cv2.contourArea, reverse = True)[:10]
                for cont in contours:
                    if cv2.contourArea(cont) > 400:
                        hull = cv2.convexHull(cont)
                        hullmask = np.zeros(circleimg.shape,dtype="uint8")
                        cv2.drawContours(hullmask, [hull], -1, 1, -1)
                        hullimage = cv2.bitwise_and(cv_image, cv_image, mask=hullmask)

                        ret,thresh5 = cv2.threshold(hullimage,127,255,\
                                                        cv2.THRESH_TOZERO_INV)
                        edges = cv2.Canny(thresh5,50,200,apertureSize = 3)
                        dilation = cv2.dilate(edges,kernel,iterations = 1)
                        dilation = cv2.erode(dilation,kernel,iterations = 1)

                        hullcontours,hier = cv2.findContours(dilation,\
                                    cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

                        for cont in hullcontours:
                            if cv2.contourArea(cont) > 500:
                                cv2.drawContours(cv_image, [cont], -1, (0, 255, 0), 3)
                                splitcontours.append([cont, "green"])

                ##################################################
                # DO THE SAME FUNCTIONS FOR RED SWEETS           #
                ##################################################
                redImage = cv2.bitwise_and(cv_image, cv_image, mask=redmask)
                ret,thresh5 = cv2.threshold(redImage,127,255,cv2.THRESH_TOZERO_INV)
                edges = cv2.Canny(thresh5,50,200,apertureSize = 3)
                dilation = cv2.dilate(edges,kernel,iterations = 1)
                dilation = cv2.erode(dilation,kernel,iterations = 1)

                cv2.imshow("dilated image", dilation)

                contours,hier = cv2.findContours(dilation,cv2.RETR_EXTERNAL,\
                                                        cv2.CHAIN_APPROX_SIMPLE)

                for cont in contours:
                    if cv2.contourArea(cont) > 400:
                        hull = cv2.convexHull(cont)
                        hullmask = np.zeros(circleimg.shape,dtype="uint8")
                        cv2.drawContours(hullmask, [hull], -1, 1, -1)
                        hullimage = cv2.bitwise_and(cv_image, cv_image, mask=hullmask)

                        ret,thresh5 = cv2.threshold(hullimage,127,255,\
                                                        cv2.THRESH_TOZERO_INV)
                        edges = cv2.Canny(thresh5,50,200,apertureSize = 3)
                        dilation = cv2.dilate(edges,kernel,iterations = 1)
                        dilation = cv2.erode(dilation,kernel,iterations = 1)

                        hullcontours,hier = cv2.findContours(dilation,\
                                    cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

                        for cont in hullcontours:
                            if cv2.contourArea(cont) > 500:
                                cv2.drawContours(cv_image, [cont], -1, \
                                                                (0, 0, 255), 3)
                                splitcontours.append([cont, "red"])

        # TRAL METHOD FOR TESTING GRIPPER APPROACH - NOT IN FINAL VERSION
        #
        # for split in splitcontours:
        #     rows,cols = cv_image.shape[:2]
        #     [vx,vy,x,y] = cv2.fitLine(split, cv2.cv.CV_DIST_L2,0,0.01,0.01)
        #     lefty = int((-x*vy/vx) + y)
        #     righty = int(((cols-x)*vy/vx)+y)
        #     #cv2.line(cv_image,(cols-1,righty),(0,lefty),(0,0,0),2)
        #     M = cv2.moments(split)
        #     cx = int(M['m10']/M['m00'])
        #     cy = int(M['m01']/M['m00'])
        #
        #     angle = math.atan2(righty - lefty, cols-1 - 0)
        #     approach = angle
        #
        #     dist = 45
        #     approach = approach * (180/math.pi)
        #     print "approach angle: ", approach, "approach/90: ",approach%90
        #
        #     if (approach < 0):
        #         newapproach = approach + 90
        #     elif (approach >= 0):
        #         newapproach = approach
        #
        #     height = dist * math.sin(newapproach)
        #     width = dist * math.cos(newapproach)
        #
        #     print (cx + width)
        #     if (approach > 0):
        #         cv2.circle(cv_image, (int(cx + width), int(cy + height)), 12, (0,0,0), -1)
        #         cv2.circle(cv_image, (int(cx - width), int(cy - height)), 12, (0,0,0), -1)
        #     if (approach < 0):
        #         cv2.circle(cv_image, (int(cx + width), int(cy - height)), 12, (0,0,0), -1)
        #         cv2.circle(cv_image, (int(cx - width), int(cy + height)), 12, (0,0,0), -1)


        # Loop through all the sweets in piles
        for [cont, colour] in splitcontours:
            # Perform PCA to calculate the approach angles and calculate
            # their centres for Baxter to pick up
            rows,cols = cv_image.shape[:2]
            [vx,vy,x,y] = cv2.fitLine(cont, cv2.cv.CV_DIST_L2,0,0.01,0.01)
            lefty = int((-x*vy/vx) + y)
            righty = int(((cols-x)*vy/vx)+y)
            M = cv2.moments(cont)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            # Update the colour totals, centres and angles appropriately
            if colour == "red":
                colourAmounts[0] = colourAmounts[0] + 1
                redangles.append(math.atan2(righty - lefty, cols-1 - 0))
                redcentres.append((cx, cy))
            if colour == "green":
                colourAmounts[1] = colourAmounts[1] + 1
                greenangles.append(math.atan2(righty - lefty, cols-1 - 0))
                greencentres.append((cx, cy))
            if colour == "blue":
                colourAmounts[2] = colourAmounts[2] + 1
                blueangles.append(math.atan2(righty - lefty, cols-1 - 0))
                bluecentres.append((cx, cy))

        #####################################################################
        # METHOD FOR DETECTING SWEETS SEPARATED FROM EACH OTHER             #
        #####################################################################

        for cont in sweetcontours:
            area = cv2.contourArea(cont)
            # Looping through all the sweet contours of an appropriate size
            if 500 < area < 1500:
                # Create a new mask of the sweet
                newmask = np.zeros(circleimg.shape,dtype="uint8")
                cv2.drawContours(newmask, [cont], -1, 255, -1)
                # Calculate the mean colour values of that sweet
                newmask = cv2.erode(newmask, None, iterations=1)
                mean = cv2.mean(cv_image, mask=newmask)[:3]
                meanarray = np.array([mean[0], mean[1], mean[2], 1])
                # Do PCA and moment calculations on each sweet to get the
                # centres and angles of approaches
                rows,cols = cv_image.shape[:2]
                [vx,vy,x,y] = cv2.fitLine(cont, cv2.cv.CV_DIST_L2,0,0.01,0.01)
                lefty = int((-x*vy/vx) + y)
                righty = int(((cols-x)*vy/vx)+y)
                M = cv2.moments(cont)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                # Using the neural network weights, classify each by the
                # colour given by the result, then update the main image
                # and totals/centres/angles lists as appropriate
                if np.dot(redWeights,meanarray) < 0:
                    cv2.drawContours(cv_image, [cont], -1, (0,0,255), 3)
                    colourAmounts[0] = colourAmounts[0] + 1
                    redangles.append(math.atan2(righty - lefty, cols-1 - 0))
                    redcentres.append((cx, cy))
                elif np.dot(greenWeights,meanarray) < 0:
                    cv2.drawContours(cv_image, [cont], -1, (0,255,0), 3)
                    colourAmounts[1] = colourAmounts[1] + 1
                    greenangles.append(math.atan2(righty - lefty, cols-1 - 0))
                    greencentres.append((cx, cy))
                elif np.dot(blueWeights,meanarray) < 0:
                    cv2.drawContours(cv_image, [cont], -1, (255,0,0), 3)
                    colourAmounts[2] = colourAmounts[2] + 1
                    blueangles.append(math.atan2(righty - lefty, cols-1 - 0))
                    bluecentres.append((cx, cy))

        # After the vision processes, create an overall list of centres/angles
        # from the red, green and blue sweets to pass onto the main node
        centres = []
        angles = []
        for i in range(0,len(redcentres)):
            centres.append(redcentres[i])
            angles.append(redangles[i])
        for i in range(0,len(greencentres)):
            centres.append(greencentres[i])
            angles.append(greenangles[i])
        for i in range(0,len(bluecentres)):
            centres.append(bluecentres[i])
            angles.append(blueangles[i])


        # Convert all of the sweet centres to Baxter's coordinate system
        overallcentres = centres
        centrepoints = []
        for centre in overallcentres:
            [x, y] = pixel_to_baxter(centre[0], centre[1])
            point = PointStamped()
            point.header.frame_id = "right_hand_camera"
            point.point.x = x
            point.point.y = y
            point.point.z = 0.45
            centrepoints.append(point)

        # Create the list of centres and angles to send to Baxter
        global pointlist
        pointlist = []
        global anglelist
        anglelist = []
        for i in range(0,len(centrepoints)):
            newpoint = tl.transformPoint("torso", centrepoints[i])
            pointlist.append(newpoint)
            anglelist.append(angles[i])

        # Sweets have been analysed, so send back to main node
        global enableAnalyse
        enableAnalyse = False

        # Show the detected sweets on screen
        cv2.imshow("IMAGE",cv_image)


    k = cv2.waitKey(1)
    global enableAnalyse
    enableAnalyse = False

# Function used to find the sweet area rectangle
def find_background(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(hsv,30,200,apertureSize = 3)
    # DILATE EDGE LINES
    kernel = np.ones((5,5),np.uint8)
    dilation = cv2.dilate(edges,kernel,iterations = 1)
    # FIND 10 LARGEST CONTOURS IN IMAGE
    (cnts, _) = cv2.findContours(dilation, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cns = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
    mask = np.zeros(img.shape[:2], dtype="uint8") * 25

    global foundsquare, pageCentre, tl, PAGECENTREPOINT
    foundsquare = False
    # FIND RECTANGLE IN IMAGE AND SEGMENT - WHAT IF MULTIPLE RECTANGLES?
    for cont in cns:
        peri = cv2.arcLength(cont, True)
        approx = cv2.approxPolyDP(cont, 0.02 * peri, True)
        # If a rectangular shape and around the right size, then accept
        # the rectangle as the main sweet area
        if len(approx) == 4:
            if cv2.contourArea(cont) > 70000:
                foundsquare = True
                cv2.drawContours(mask, [cont], -1, 1, -1)
                # Create a mask and do moment analysis to find the centre,
                # converting the centre to Baxter's coordinate system
                M = cv2.moments(cont)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                [x, y] = pixel_to_baxter(cx, cy)
                point = PointStamped()
                point.header.frame_id = "right_hand_camera"
                point.point.x = x
                point.point.y = y
                point.point.z = 0.45
                PAGECENTREPOINT = tl.transformPoint("torso", point)
                pageCentre = [PAGECENTREPOINT.point.x, \
                              PAGECENTREPOINT.point.y, PAGECENTREPOINT.point.z]
                print cv2.contourArea(cont)

    return mask, foundsquare

# Convert image pixel to Baxter point
def pixel_to_baxter(imagex, imagey):
    # FROM CAMERA_INFO ROSTOPIC
    # INTRINSIC CAMERA MATRIX (fx, 0, cx)
                            # (0, fy, cy)
                            # (0, 0, 1)
    # K: [407.366831078, 0.0, 642.166170111, 0.0, 407.366831078, 387.185928717, 0.0, 0.0, 1.0]

    fx = 407.366831078
    fy = 407.366831078
    cx = 642.166170111
    cy = 387.185928717

    # USING DISTANCE S FROM TABLE AND THE FACT THE 3D CAMERA COORD IS s*((u-cx)/fx, (v-cy)/fy, 1)
    u = imagex
    v = imagey
    xl = (u - cx)/fx
    yl = (v - cy)/fy

    s = 0.45
    x = xl * s
    y = yl * s

    return (x, y)

# Function that gets all the detected sweet information and sends it as a
# response to the main node
def publish_sweet_handle(req):
    global centrelist, colourAmounts, pointlist, pageCentre
    global anglelist
    centrelist = []
    for i in range(0, len(pointlist)):
        centrelist.append(pointlist[i].point.x)
        centrelist.append(pointlist[i].point.y)
        centrelist.append(pointlist[i].point.z)
    return RequestSweetInfoResponse(colourAmounts, centrelist, anglelist, pageCentre)

# Function that resets the analysis so that the program will analyse the sweets
# before being able to return a result - helps delay the system
def handle_reset_sweets(req):
    print "Returning ",req.reset
    global enableAnalyse
    # rospy.sleep(4)
    rospy.sleep(2)
    enableAnalyse = True
    while enableAnalyse == True:
        rospy.sleep(1)
    return LookForSweetsResponse("OK")

#Subscribes to left hand camera image feed
def main():
    rospy.sleep(10)
    rospy.init_node('view_sweet_cam', anonymous = True)

    # Only retrieve the image from baxters hand if the system needs it - helps
    # speed up analysis
    if (enableAnalyse == True):
        image_topic = rospy.resolve_name("/cameras/right_hand_camera/image")
        rospy.Subscriber(image_topic, Image, callback)

    # Subscribe to the appropriate services to exchange information
    s = rospy.Service('publish_sweet_info', RequestSweetInfo, publish_sweet_handle)
    t = rospy.Service('reset_sweets', LookForSweets, handle_reset_sweets)

    #Keep from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
     main()
