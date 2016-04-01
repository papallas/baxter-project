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

sweetArea = 0
backgroundImage = 0

global totalSweets
global centrelist

# RED, GREEN, BLUE
global colourAmounts

global sweetarea
global foundsquare

global enableAnalyse
enableAnalyse = True

global pointlist
# global collisionslist
# global totalCollisions
#
# global collisions
# collisions = "False"
# global collisionslist

global anglelist

#Thresholds image and stores position of object in (x,y) coordinates of the camera's frame, with origin at center.
def callback(message):
    #Capturing image of web camera
    br = CvBridge()	# Create a black image, a window

    #img = br.imgmsg_to_cv2(message, desired_encoding="passthrough")
    cv_image = br.imgmsg_to_cv2(message, "bgr8")
    # Take each frame
    simg = cv2.GaussianBlur(cv_image,(5,5),0)

    # initialize the colors dictionary, containing the color
    # name as the key and the RGB tuple as the value
    colors = OrderedDict({
    	"green": (130.68058107082402, 126.18073194402548, 91.24113616607778),
    	"blue": (167.21600854233967, 113.0324141733287, 81.47253859404914),
        "red": (121.55769231718531, 102.44336928332095, 116.87984092454667)})

    # allocate memory for the L*a*b* image, then initialize
    # the color names list
    lab = np.zeros((len(colors), 1, 3), dtype="uint8")
    colorNames = []

    # loop over the colors dictionary
    for (i, (name, rgb)) in enumerate(colors.items()):
    	# update the L*a*b* array and the color names list
    	lab[i] = rgb
    	colorNames.append(name)

    # convert the L*a*b* array from the RGB color space
    # to L*a*b*
    # lab = cv2.cvtColor(lab, cv2.COLOR_RGB2LAB)

    # lab = []
    # lab.append([[0, 128, 128]])
    # lab.append([[0, -128, 128]])
    # lab.append([[0, 128, -128]])

    if enableAnalyse == True:
        # print lab

        # find sweet rectangular area for sweets
        global sweetarea
        global foundsquare
        foundsquare = False
        sweetarea, areafound = find_background(simg)
        print "Finding sweet area"

        #with open('colours.data', 'a') as file:

        if foundsquare == True:
            # Convert BGR to HS
            circleimg = cv2.cvtColor(sweetarea, cv2.COLOR_BGR2GRAY);

            cvimagelab = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB);

            edges = cv2.Canny(circleimg,50,200,apertureSize = 3)

            kernel = np.ones((5,5),np.uint8)
            dilation = cv2.dilate(edges,kernel,iterations = 1)

            contours,hier = cv2.findContours(dilation,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
            #cv2.drawContours(cv_image, contours, -1, (255,0,0), 3)

            mask = np.zeros(circleimg.shape,dtype="uint8")
            centrepoints = []
            for cont in contours:
                area = cv2.contourArea(cont)
                # cut out large border of page contours
                if area < 50000:
                    cv2.drawContours(mask, [cont], -1, 255, -1)

                    # cv2.drawContours(mask, [cont], -1, (255,255,255), 3)

            newcontours,newhier = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            # cv2.drawContours(cv_image, newcontours, -1, (255,0,0), 3)

            global colourAmounts
            colourAmounts = [0,0,0]

            redcentres = []
            greencentres = []
            bluecentres = []
            redangles = []
            greenangles = []
            blueangles = []

            greenWeights = np.array([366.1219, -463.8519, 63.0028, 8.5000])

            blueWeights = np.array([-42.2808, 25.3629, 30.6613, 6.7000])

            redWeights = np.array([3.4025, 11.9616, -17.8766, 6.9000])

            for cont in newcontours:
                area = cv2.contourArea(cont)
                # cut out large border of page contours
                # print area
                if area < 1900:
                    newmask = np.zeros(circleimg.shape,dtype="uint8")
                    cv2.drawContours(newmask, [cont], -1, 255, -1)

                    newmask = cv2.erode(newmask, None, iterations=1)
                    mean = cv2.mean(cv_image, mask=newmask)[:3]

                    meanarray = np.array([mean[0], mean[1], mean[2], 1])

                    # string = str(mean[0])+", "+str(mean[1])+", "+str(mean[2])+", "+"pink\n"
                    # file.write(string)

                    #initialize the minimum distance found thus far
                    minDist = (np.inf, None)

                    # loop over the known L*a*b* color values
                    for (i, row) in enumerate(lab):
                        # compute the distance between the current L*a*b*
                        # color value and the mean of the image
                        # d = dist.euclidean(row[0], mean)
                        d = np.linalg.norm(mean-row[0])

                        # if the distance is smaller than the current distance,
                        # then update the bookkeeping variable
                        if d < minDist[0]:
                        	minDist = (d, i)

                    rows,cols = cv_image.shape[:2]
                    [vx,vy,x,y] = cv2.fitLine(cont, cv2.cv.CV_DIST_L2,0,0.01,0.01)
                    lefty = int((-x*vy/vx) + y)
                    righty = int(((cols-x)*vy/vx)+y)
                    M = cv2.moments(cont)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    if np.dot(redWeights,meanarray) < 0:
                        #print "This is red"
                        cv2.drawContours(cv_image, [cont], -1, (0,0,255), 3)
                        colourAmounts[0] = colourAmounts[0] + 1
                        redangles.append(math.atan2(righty - lefty, cols-1 - 0))
                        redcentres.append((cx, cy))

                    elif np.dot(greenWeights,meanarray) < 0:
                        #print "This is green"
                        cv2.drawContours(cv_image, [cont], -1, (0,255,0), 3)
                        colourAmounts[1] = colourAmounts[1] + 1
                        greenangles.append(math.atan2(righty - lefty, cols-1 - 0))
                        greencentres.append((cx, cy))

                    elif np.dot(blueWeights,meanarray) < 0:
                        #print "This is blue"
                        cv2.drawContours(cv_image, [cont], -1, (255,0,0), 3)
                        colourAmounts[2] = colourAmounts[2] + 1
                        blueangles.append(math.atan2(righty - lefty, cols-1 - 0))
                        bluecentres.append((cx, cy))

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

            cv2.imshow("IMAGE",cv_image)

            global enableAnalyse
            enableAnalyse = False

            overallcentres = centres
            centrepoints = []
            for centre in overallcentres:
                [x, y] = pixel_to_baxter(centre[0], centre[1])
                #newcoords.append((x,y))
                point = PointStamped()
                point.header.frame_id = "right_hand_camera"
                point.point.x = x
                point.point.y = y
                point.point.z = 0.45
                centrepoints.append(point)

            tl = tf.TransformListener()

            try:
                tl.waitForTransform("right_hand_camera", "torso", rospy.Time(0), rospy.Duration(10))
                # (trans,rot) = tl.lookupTransform('right_hand_camera', 'torso', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            global pointlist
            pointlist = []
            # global collisionslist
            # collisionslist = []
            global anglelist
            anglelist = []


            for i in range(0,len(centrepoints)):
                newpoint = tl.transformPoint("torso", centrepoints[i])
                pointlist.append(newpoint)
                anglelist.append(angles[i])

            print anglelist

    k = cv2.waitKey(1)
    global enableAnalyse
    enableAnalyse = False



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

    global foundsquare
    foundsquare = False
    # FIND RECTANGLE IN IMAGE AND SEGMENT - WHAT IF MULTIPLE RECTANGLES?
    for cont in cns:
        peri = cv2.arcLength(cont, True)
        approx = cv2.approxPolyDP(cont, 0.02 * peri, True)
        if len(approx) == 4:
            foundsquare = True
            cv2.drawContours(mask, [cont], -1, 1, -1)
            # DRAW CONTOURS TO IMAGE
            # cv2.drawContours(img, cont, -1, (0, 255, 0), 3)
            break

    img = cv2.bitwise_and(img, img, mask=mask)

    if (foundsquare == False):
        print "sweet area is not found"

    return img, foundsquare

def find_sweets(hsv, colour, r1, g1, b1, r2, g2, b2, area, size):
    lower = np.array([r1,g1,b1])
    upper = np.array([r2,g2,b2])

    #cv2.imshow("HSV", hsv)

    mask = cv2.inRange(hsv, lower, upper)

    # filter and fill the mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(size, size))
    mask2 = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)

    contour,hier = cv2.findContours(mask2,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    sweetCount = 0
    # collisionCount = 0
    sweetcontours = []
    sweetcentres = []
    # collisioncontours = []
    # collisionCentres = []

    global collisions
    collisions = "False"

    # for cnt in contour:
    #     if (cv2.contourArea(cnt) > area):
    #         if cv2.contourArea(cnt) > 1500:
    #             global collisions
    #             collisions = "True"
    #
    # for cnt in contour:
    #     if (cv2.contourArea(cnt) > area):
    #         if cv2.contourArea(cnt) > 1500:
    #             cv2.drawContours(mask2, [cnt], 0, 255, 3)
    #             collisioncontours.append(cnt)
    #             collisionCount = collisionCount + 1
    #             M = cv2.moments(cnt)
    #             cx = int(M['m10']/M['m00'])
    #             cy = int(M['m01']/M['m00'])
    #             collisionCentres.append((cx, cy))

    sweetangles = []

    for cnt in contour:
        if (cv2.contourArea(cnt) > area):
            if cv2.contourArea(cnt) < 1500:
                rows,cols = hsv.shape[:2]
                [vx,vy,x,y] = cv2.fitLine(cnt, cv2.cv.CV_DIST_L2,0,0.01,0.01)
                lefty = int((-x*vy/vx) + y)
                righty = int(((cols-x)*vy/vx)+y)
                sweetangles.append(math.atan2(righty - lefty, cols-1 - 0))
                # cv2.line(hsv,(cols-1,righty),(0,lefty),(0,255,0),2)
                # print "Angle is ",rect[2]

                cv2.drawContours(mask2, [cnt], 0, 255, 3)
                sweetcontours.append(cnt)
                sweetCount = sweetCount + 1
                M = cv2.moments(cnt)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                sweetcentres.append((cx, cy))

    print "There are "+str(sweetCount)+" "+colour+" sweets"


    #cv2.imshow("bluemask", mask2)

    # return mask2, sweetcontours, sweetCount, sweetcentres, collisioncontours, collisionCount, collisionCentres, sweetangles

    return mask2, sweetcontours, sweetCount, sweetcentres, sweetangles


# convert image pixel to Baxter point
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

def publish_sweet_handle(req):
    print "request has string "+req.A
    global centrelist
    global colourAmounts
    global pointlist
    # global collisions
    # global collisionslist
    # global totalCollisions
    global anglelist
    centrelist = []
    # collisionCentre = []
    for i in range(0, len(pointlist)):
        centrelist.append(pointlist[i].point.x)
        centrelist.append(pointlist[i].point.y)
        centrelist.append(pointlist[i].point.z)
    print colourAmounts
    return RequestSweetInfoResponse(colourAmounts, centrelist, anglelist)

def handle_reset_sweets(req):
    print "Returning ",req.reset
    global enableAnalyse
    # rospy.sleep(4)
    rospy.sleep(1)
    enableAnalyse = True
    while enableAnalyse == True:
        rospy.sleep(1)
    return LookForSweetsResponse("OK")

global subscribeCounter
subscribeCounter = 0
#Subscribes to left hand camera image feed
def main():
    rospy.init_node('view_sweet_cam', anonymous = True)

    # cv2.namedWindow('IMAGE', flags=0)
    # cv2.namedWindow('HSV', flags=0)


    # buffer_size = rospy.get_param("/cameras/right_hand_camera/buffer_size")
    # rospy.Subscriber(image_topic, Image, callback,  queue_size = 1, buff_size=2**24)
    global subscribeCounter
    subscribeCounter = subscribeCounter + 1

    if (enableAnalyse == True):
        image_topic = rospy.resolve_name("/cameras/right_hand_camera/image")
        # rospy.Subscriber(image_topic, Image, callback, queue_size = 1, buff_size=int(1024000))
        rospy.Subscriber(image_topic, Image, callback)

    s = rospy.Service('publish_sweet_info', RequestSweetInfo, publish_sweet_handle)

    t = rospy.Service('reset_sweets', LookForSweets, handle_reset_sweets)


    #Keep from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
     main()
