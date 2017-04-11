#!/usr/bin/env python

import argparse
import sys, os, inspect
import struct
import cv2

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


from mary_tts.srv import *

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from std_msgs.msg import String

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from manipulation.srv import *
# from manipulation.msg import *

import baxter_interface

from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import EndEffectorState

import tf

import math

from std_msgs.msg import UInt16

from sound_play.libsoundplay import SoundClient
import roslib
roslib.load_manifest("baxter_cashier_manipulation")
from baxter_cashier_manipulation import *
from cashier import Cashier

# initialise ros node
rospy.init_node("shopkeeper", anonymous = True)

# file directory
directory = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))


# The shopkeeper class contains all of the manipulation tasks that Baxter
# needs to run the shop, from grabbing the bowl to placing sweets in the bag
class ShopKeeper:
    # Initialise all the needed movement settings for Baxter
    def __init__(self):
        # Firstly, set up the interfaces with Baxters arms
        self.right_interface = baxter_interface.Limb("right")
        self.left_interface = baxter_interface.Limb("left")
        # Set a low movement speed for safety
        self.right_interface.set_joint_position_speed(0.1)
        self.left_interface.set_joint_position_speed(0.1)

        # Store the initial required left arm position in memory
        self.init_left_x = 0.5793              # x     = front back
        self.init_left_y = 0.1883              # y     = left right
        self.init_left_z = 0.15                # z     = up down
        self.left_roll   = 3.1024              # roll  = horizontal
        self.left_pitch  = 0.0446              # pitch = vertical
        self.left_yaw    = 2.8645              # yaw   = rotation

        self.left_pose = [self.init_left_x, self.init_left_y, self.init_left_z,\
                     self.left_roll, self.left_pitch, self.left_yaw]

        # Store the initial required right arm position in memory
        self.init_right_x = 0.5797             # x     = front back
        self.init_right_y = -0.1910            # y     = left right
        self.init_right_z = 0.1013             # z     = up down
        self.right_roll   = -3.0999            # roll  = horizontal
        self.right_pitch  = 0.0441             # pitch = vertical
        self.right_yaw    = -2.8676            # yaw   = rotation

        self.right_pose = [self.init_right_x, self.init_right_y, \
           self.init_right_z, self.right_roll, self.right_pitch, self.right_yaw]

        # Connect to and calibrate Baxter's left and right grippers
        self._left_grip = baxter_interface.Gripper('left', CHECK_VERSION)
        self._right_grip = baxter_interface.Gripper('right', CHECK_VERSION)
        self._left_grip.calibrate()
        self._right_grip.calibrate()

        # Initialise the head interface
        self.head = baxter_interface.Head()

        # Set various initial publishing values
        self._pub_rate          = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._rate              = 500.0  # Hz
        self._max_speed         = .05
        self._arm               = 'none'
        # Set joint state publishing to 500Hz
        self._pub_rate.publish(self._rate)

        # Check the robot is enabled and in the correct state
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # NON-MANIPULATION RELATED VARIABLES

        # MINZ VARIABLE IS USED AS THE TABLE PLANE'S Z VALUE - TO BE DEFINED
        # IN ROSLAUNCH PARAMS IN CASE OF DIFFERING TABLE HEIGHTS!
        self.MINZ = -0.215
        # These variables keep track of the number of coloured sweets given to
        # the customer
        self.redGiven = 0
        self.blueGiven = 0
        self.greenGiven = 0

    # A key function in which you can pass a limb, xyz coordinate and a rpy
    # pose and Baxter will move/rotate the arm to that position
    def move_and_rotate(self, limb, x, y, z, r, p, ya):
        pose = [x, y, z, r, p, ya]
        if limb == "right":
            self.baxter_ik_move("right", pose)
            self.right_pose = [x, y, z, r, p, ya]
        if limb == "left":
            self.baxter_ik_move("left", pose)
            self.left_pose = [x, y, z, r, p, ya]

    # Background function with Inverse Kinematic Solvers to work out where
    # Baxter wants to move
    def baxter_ik_move(self, limb, rpy_pose):
        # Make sure Baxter doesn't try to go below table level to avoid
        # collisions
        if (rpy_pose[2] <  self.MINZ):
            rpy_pose[2] =  self.MINZ
        # Get the actual stamped pose
        quaternion_pose = self.list_to_pose_stamped(rpy_pose, "base")

        # Place a request to the IK solver using the required pose
        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")

        ik_request.pose_stamp.append(quaternion_pose)
        # Wait for an solution/response
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            # Catch invalid movement requests
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")
        # If there is a valid solution for the movement:
        if ik_response.isValid[0]:
            # Convert the response to a zipped dictionary of joint locations
            limb_joints = dict(zip(ik_response.joints[0].name, \
                                                ik_response.joints[0].position))
            # move limb
            if limb == "left":
                self.left_interface.move_to_joint_positions(limb_joints)
            elif limb == "right":
                self.right_interface.move_to_joint_positions(limb_joints)
        else:
            # Else exit the program if an invalid move - may be changed later
            print "requested move =", rpy_pose
            sys.exit("ERROR - baxter_ik_move - \
                                            No valid joint configuration found")

        if limb == "left":               # if working arm
            quaternion_pose = self.left_interface.endpoint_pose()
            position        = quaternion_pose['position']

            # if working arm remember actual (x,y) position achieved
            self.left_pose = [position[0], position[1],                               \
                         self.left_pose[2], self.left_pose[3], self.left_pose[4], self.left_pose[5]]
        if limb == "right":               # if working arm
            quaternion_pose = self.right_interface.endpoint_pose()
            position        = quaternion_pose['position']

            # if working arm remember actual (x,y) position achieved
            self.right_pose = [position[0], position[1],                                \
                         self.right_pose[2], self.right_pose[3], self.right_pose[4], self.right_pose[5]]

    # This method converts a pose list to a stamped list in the correct frame
    def list_to_pose_stamped(self, pose_list, target_frame):
        pose_msg = PoseStamped()
        pose_msg.pose = self.list_to_pose(pose_list)
        pose_msg.header.frame_id = target_frame
        pose_msg.header.stamp = rospy.Time.now()
        return pose_msg

    # Convert the pose list to the appropriate list using conversions where
    # appropriate
    def list_to_pose(self,pose_list):
       pose_msg = Pose()
       if len(pose_list) == 7:
           pose_msg.position.x = pose_list[0]
           pose_msg.position.y = pose_list[1]
           pose_msg.position.z = pose_list[2]
           pose_msg.orientation.y = pose_list[4]
           pose_msg.orientation.z = pose_list[5]
           pose_msg.orientation.w = pose_list[6]
       elif len(pose_list) == 6:
           pose_msg.position.x = pose_list[0]
           pose_msg.position.y = pose_list[1]
           pose_msg.position.z = pose_list[2]
           q = tf.transformations.quaternion_from_euler(pose_list[3], \
                                                    pose_list[4], pose_list[5])
           pose_msg.orientation.x = q[0]
           pose_msg.orientation.y = q[1]
           pose_msg.orientation.z = q[2]
           pose_msg.orientation.w = q[3]
       else:
           raise MoveItCommanderException("Expected either 6 or 7 elements in list: (x,y,z,r,p,y) or (x,y,z,qx,qy,qz,qw)")
       return pose_msg

    # Function to update the current left pose with the actual pose values of
    # the left arm
    def update_left(self):
        quaternion_pose = self.left_interface.endpoint_pose()
        position        = quaternion_pose['position']
        quaternion      = quaternion_pose['orientation']
        euler           = tf.transformations.euler_from_quaternion(quaternion)

        self.left_pose = [position[0], position[1], position[2], euler[0], euler[1], euler[2]]

    # Function for Baxter to move to grab the bowl - requires an x, y and z
    # for the current bowl coordinates
    def move_to_grab_bowl(self, x, y, z):
        self.left_interface.set_joint_position_speed(0.3)
        # Move to the left side of the bowl
        self.move_and_rotate("left", x, y+0.05, -0.15, self.left_pose[3], \
                                          self.left_pose[4], self.left_pose[5])
        # Tilt the arm and move down diagonally to get in position
        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], self.left_pose[2],
        self.left_pose[3]+0.3,self.left_pose[4],self.left_pose[5])

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1]-0.03, -0.2,
        self.left_pose[3],self.left_pose[4],self.left_pose[5])

    # Function to rotate Baxter's gripper by a specified angle
    def rotate_gripper_angle(self, angle):
        # Get the current joint names and angles, change the gripper angle
        # by the specified angle and move to those joint positions
        names = self.left_interface.joint_names()
        angles = self.left_interface.joint_angles()
        main = dict(angles)
        leftpos = main["left_w2"]
        leftpos = leftpos + angle
        main["left_w2"] = leftpos
        self.left_interface.move_to_joint_positions(main)

    # Tip sweets out of the bowl onto the table - requires a bowl x,y and z,
    # the sweet page x,y centre, a tracking variable on how many tilts have
    # occurred and a bowl shake height
    def tip_bowl(self, x, y, z, pagex, pagey, initialtilt, height):
        # Grab the bowl once it is in position
        self._left_grip.close()
        self.left_interface.set_joint_position_speed(0.3)
        # Rotate the arm to a more horizontal position (only if first time)
        if (initialtilt == True):
            self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], -0.1,
            self.left_pose[3]+0.5,self.left_pose[4],self.left_pose[5])
        elif (initialtilt == False):
            self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], -0.1,
            self.left_pose[3],self.left_pose[4],self.left_pose[5])


        self.left_interface.set_joint_position_speed(0.2)
        # Move to the right hand side of the page
        self.move_and_rotate("left", pagex-0.21, pagey-0.1, self.left_pose[2]+0.05,
        self.left_pose[3],self.left_pose[4],self.left_pose[5])
        # Rotate the bowl horizontally using the gripper
        self.rotate_gripper_angle(-1*(1.1+(height*0.2)))
        self.update_left()
        # Move the bowl down to the page
        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1],
            self.left_pose[2]-0.04, self.left_pose[3], self.left_pose[4],self.left_pose[5])
        # Set to a high speed and shake the bowl in an upwards diagonal direction
        self.left_interface.set_joint_position_speed(0.9)

        self.move_and_rotate("left", self.left_pose[0]+0.05, self.left_pose[1],
        self.left_pose[2]+0.2, self.left_pose[3],self.left_pose[4],self.left_pose[5]-0.2)

        self.move_and_rotate("left", self.left_pose[0]-0.05, self.left_pose[1],
        self.left_pose[2]-0.2, self.left_pose[3],self.left_pose[4],self.left_pose[5]+0.2)

        self.left_interface.set_joint_position_speed(0.2)
        # Move to the left hand side of the page
        self.move_and_rotate("left", pagex-0.21, pagey+0.08, self.left_pose[2],
        self.left_pose[3],self.left_pose[4],self.left_pose[5])
        # Rotate slightly further than the last tip
        self.rotate_gripper_angle(-0.1)
        self.update_left()

        self.left_interface.set_joint_position_speed(0.9)
        # Shake the bowl diagonally again
        self.move_and_rotate("left", self.left_pose[0]+0.05, self.left_pose[1],
        self.left_pose[2]+0.2, self.left_pose[3],self.left_pose[4],self.left_pose[5]-0.2)

        self.move_and_rotate("left", self.left_pose[0]-0.05, self.left_pose[1],
        self.left_pose[2]-0.2, self.left_pose[3],self.left_pose[4],self.left_pose[5]+0.2)
        # Rotate the bowl back to horizontal
        self.rotate_gripper_angle(1.2+(height*0.2))
        self.update_left()

        self.left_interface.set_joint_position_speed(0.2)
        # Move the bowl back to the left of the page and lower - keep
        # hold of the bowl for further tilts
        self.move_and_rotate("left", x, y-+0.01, self.left_pose[2],
        self.left_pose[3],self.left_pose[4],self.left_pose[5])

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], -0.18,
        self.left_pose[3],self.left_pose[4],self.left_pose[5])

    # Move the right arm forward so Baxter can see the sweet area fully within
    # the camera image
    def moveToSweets(self):
        self.move_and_rotate("right", 0.8, -0.25, 0.15, -3.0999, 0.0441, -2.8676)

    # A callback function to help Baxter know whether something is in his
    # gripper or not
    def gripperholding(self, state):
        if state.force > 1:
            self.gripped = True
        if state.force <= 1:
            self.gripped = False

    # Pick up a sweet on the page, given it's required colour, PCA angle, xyz
    # centre and the page centre
    def move_and_pick_up_sweet(self, colour, angle, x, y, z, pagex, pagey):
        self.right_interface.set_joint_position_speed(0.5)
        # Standard gripper rotation in radians
        defaultrotation = -2.8676
        # Rotate by 90 degrees to go perpendicular then subtract the angle
        overall = -2.8676 + ((90/(180/math.pi)) - angle)
        # Offset in x direction - added after grabbing too low on occasion
        x = x + 0.008

        send_image('default')

        # Move above the sweet and then move slowly downwards and grab it, then
        # lift the sweet off the page
        self.move_and_rotate("right", x, y, 0,
        self.right_pose[3],self.right_pose[4],overall)

        self.right_interface.set_joint_position_speed(0.1)
        self.move_and_rotate("right", x, y, self.MINZ,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        self._right_grip.close()

        self.right_interface.set_joint_position_speed(0.5)

        self.move_and_rotate("right", self.right_pose[0], self.right_pose[1], 0,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        # If Baxter managed to grip the current sweet:
        if self.gripped == True:
            # Move to the right of the page and drop the sweet in the bag
            self.move_and_rotate("right", self.bag[0], self.bag[1], self.bag[2],
            self.bag[3], self.bag[4], self.bag[5])

            self._right_grip.open()
            # Update the appropriate colour value
            if colour == 0:
                self.redGiven = self.redGiven + 1
            if colour == 1:
                self.greenGiven = self.greenGiven + 1
            if colour == 2:
                self.blueGiven = self.blueGiven + 1
        # If Baxter missed the sweet, then ignore the next step and move on
        if self.gripped == False:
            self._right_grip.open()
            send_image('anger')
            self.speak('Oops, sorry about that')

    # print all 6 arm coordinates (only required for programme development)
    def print_arm_pose(self):
        pi = math.pi

        quaternion_pose = self.right_interface.endpoint_pose()
        position        = quaternion_pose['position']
        quaternion      = quaternion_pose['orientation']
        euler           = tf.transformations.euler_from_quaternion(quaternion)

        print "\nRIGHT POSITION\n"
        print 'front back = %5.4f ' % position[0]
        print 'left right = %5.4f ' % position[1]
        print 'up down    = %5.4f ' % position[2]
        print 'roll       = %5.4f radians %5.4f degrees' % (euler[0], 180.0 * \
                                                               euler[0]/math.pi)
        print 'pitch      = %5.4f radians %5.4f degrees' % (euler[1], 180.0 * \
                                                              euler[1] /math.pi)
        print 'yaw        = %5.4f radians %5.4f degrees' % (euler[2], 180.0 * \
                                                              euler[2] /math.pi)

        quaternion_pose2 = self.left_interface.endpoint_pose()
        position2        = quaternion_pose2['position']
        quaternion2      = quaternion_pose2['orientation']
        euler2           = tf.transformations.euler_from_quaternion(quaternion2)

        print "\nLEFT POSITION\n"
        print 'front back = %5.4f ' % position2[0]
        print 'left right = %5.4f ' % position2[1]
        print 'up down    = %5.4f ' % position2[2]
        print 'roll       = %5.4f radians %5.4f degrees' % (euler2[0], 180.0 * \
                                                              euler2[0]/math.pi)
        print 'pitch      = %5.4f radians %5.4f degrees' % (euler2[1], 180.0 * \
                                                             euler2[1] /math.pi)
        print 'yaw        = %5.4f radians %5.4f degrees' % (euler2[2], 180.0 * \
                                                             euler2[2] /math.pi)

    def get_pose(self, limb):
        if limb == "right":
            quaternion_pose = self.right_interface.endpoint_pose()
        elif limb == "left":
            quaternion_pose = self.left_interface.endpoint_pose()
        position        = quaternion_pose['position']
        quaternion      = quaternion_pose['orientation']
        euler           = tf.transformations.euler_from_quaternion(quaternion)
        return [position[0], position[1], position[2], euler[0], euler[1], euler[2]]

    def speak(self,x):
        rospy.wait_for_service('ros_mary')
        try:
            add_two_ints = rospy.ServiceProxy('ros_mary',ros_mary)
            resp1 = add_two_ints(x)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

# This class contains communcation methods to send and receive custom messages
# to the different app, vision and manipulation nodes
# ALL OF THESE FUNCTIONS USE A SERVICE TO SEND AND WAIT FOR A MESSAGE FROM
# AN EXTERNAL ROS NODE. THIS APPROACH IS ALMOST IDENTICAL IN EACH CASE
class Communications:
    def __init__(self, shopkeeper):
        self.baxter = shopkeeper

    # Get the current bowl position with a request, retrieving the xy and z
    # coordinates from the bowl_pos_transform node
    def get_pos_client(self):
        print "\nRequesting bowl position in Baxter coordinates"
        rospy.wait_for_service('bowl_pos_req')
        try:
            bowl_pos_req = rospy.ServiceProxy('bowl_pos_req', RequestBowlPos)
            resp = bowl_pos_req("")
            return [resp.x, resp.y, resp.z]
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Reset the cumulative averaging on the bowl detection - used when Baxter
    # has moved the bowl - resets the second_average node
    def reset_bowl(self):
        print "Detecting Bowl.."
        rospy.wait_for_service('reset_bowl')
        try:
            reset_bowl_req = rospy.ServiceProxy('reset_bowl', LookForBowl)
            resp = reset_bowl_req("Reset")
            return resp.ok
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Reset the vision system looking at the sweets if the sweets on the table
    # have changed - targets the find_colours node
    def reset_sweets(self):
        rospy.wait_for_service('reset_sweets')
        try:
            reset_sweets_req = rospy.ServiceProxy('reset_sweets', LookForSweets)
            resp = reset_sweets_req("Reset")
            return resp.ok
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Get the current visualised sweet information from the find_colours node
    def get_sweet_client(self):
        rospy.wait_for_service('publish_sweet_info')
        try:
            sweet_number_req = rospy.ServiceProxy('publish_sweet_info', RequestSweetInfo)
            resp = sweet_number_req("")
            return resp.n, resp.pos, resp.angleList, resp.page

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Wait for a customer's order to be entered into the sweet selection app
    def get_app_command(self):
        print "Waiting for voice command"
        rospy.wait_for_service('get_command')
        try:
            voice_req = rospy.ServiceProxy('get_command', RequestCommand)
            resp = voice_req("")
            return resp.n

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Get in position and wait for someone to wait in front of Baxter
    def wait_for_person(self, request):
        # Move the right arm out of the way of the head's camera using preset
        # joint values
        positions = {'right_e0': 0.5250049246537829, 'right_e1': 1.9450876390387046, \
                     'right_s0': -0.3804272353955826, 'right_s1': -1.1692768555656565, \
                     'right_w0': -0.02569417819708068, 'right_w1': 0.8421554525490922, \
                     'right_w2': 0.30104372962251247}

        self.baxter.right_interface.set_joint_position_speed(0.5)
        self.baxter.right_interface.move_to_joint_positions(positions)
        # Move the head to the right to look for customers
        self.baxter.head.set_pan(-0.5)
        # Wait until the find_person node says a person exists in front of the
        # camera
        rospy.sleep(2)
        rospy.wait_for_service('look_for_person')
        try:
            person_req = rospy.ServiceProxy('look_for_person', RequestPerson)
            resp = person_req(request)
            return resp.ok
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

def send_image(img_name):
    """
    Send the image located at the specified path to the head
    display on Baxter.
    @param path: path to the image file to load and send
    """
    path = directory+'/images/'+img_name+'.png'
    img = cv2.imread(path)
    msg = CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    rospy.sleep(1)


# Main function - the basic system logic for Baxter to run the shop - uses
# loops and managing variables where appropriate
if __name__ == '__main__':
    # Initialise a baxter object, of shopkeeper class
    baxter = ShopKeeper()

    # Note that Cashier's constructor will ask for some commands from the
    # operator like to calibrate the hand above the first banknote etc.
    cashier = Cashier()

    # Set up the communications with Baxter and other nodes
    comms = Communications(baxter)

    # Subscribe to the right gripper's state to detect torque changes
    rospy.Subscriber("/robot/end_effector/right_gripper/state", EndEffectorState,
                                                          baxter.gripperholding)

    # Use default face to start
    send_image('bored')

    #############################################################################
    # SYSTEM SETUP TASKS - TERMINAL HELPS USER SETUP THE PROGRAM BEFORE RUNNING #
    #############################################################################

    # Move to the sweet area before the sweet vision system can be initialised
    # and reset retrieve the initial page location etc
    baxter.moveToSweets()
    comms.reset_sweets()

    # A hard coded bowl value is used without the Kinect
    [bowlX,bowlY,bowlZ] = [0.6,0.25, -0.2]
    # Make baxter move where the bowl should be to help in setup
    baxter.move_to_grab_bowl(bowlX , bowlY, bowlZ)

    # Print setup command prompts, with user inputting enter to wait for setup
    # tasks being complete
    print "Make sure the app is open and the IP address on the terminal is \
entered on the android device\n\n"
    print "Place the bowl of sweets so the left rim of the bowl is between \
Baxter's gripper\n\n"
    raw_input("Press Enter when this step is complete...\n\n")

    print "Place the left gripper so it is touching the table and the right \
gripper so it is above the sweet bag/container\n\n"
    raw_input("Press Enter when this step is complete...\n\n")

    # After the user moves the arms, get the current z height of the left to be
    # used as the program's table height value
    leftpose = baxter.get_pose("left")
    baxter.MINZ = leftpose[2]
    # Get the right arm's position as the one used to drop sweets into the bag
    baxter.bag = baxter.get_pose("right")

    print "The setup is now complete"

    # Move the left arm back to normal untucked position
    baxter.left_interface.set_joint_position_speed(0.5)
    baxter.move_and_rotate("left", 0.5815, 0.1831, 0.1008, 3.1209, 0.0492, 2.8580)

    # PRINT INTRO TO TERMINAL
    print "###################################################################"
    print "#                 THE ROBOT SHOPKEEPER PROJECT                    #"
    print "###################################################################\n"
    print "Hello there and welcome to Baxter's sweet shop\n"

    # Whilst there, get the page centre (should be no sweets initially)
    num, centres, anglelist, page = comms.get_sweet_client()

    # THIS IS THE MAIN SHOP LOOP THAT WILL BE RUN CONSTANTLY AFTER LAUNCHING
    while True:
        send_image('bored')
        # First of all, wait for a customer approaches, specifying enter to
        # check for customer entry
        print "Waiting for a customer to approach...\n"
        comms.wait_for_person("enter")
        # Once the person has entered, change to happy face and speak to
        # acknowledge their appearance
        send_image('default')
        baxter.speak('Hello, welcome to my shop.')
        print "Hello customer, welcome to my shop, what sweets would you like \
today?"

        # Inform the user to provide a voice/touch request on the application
        baxter.speak('What sweets would you like today')
        baxter.speak('Speak after the beep and press the place order button to confirm')
        rospy.sleep(9)

        # Get the application user command and assign the requested amounts
        # to variables
        command = comms.get_app_command()
        blueRequest = command[0]
        greenRequest = command[2]
        redRequest = command[1]
        print"You want ",blueRequest," blue sweets, ",greenRequest," green sweets \
and ",redRequest," red sweets"

        # Move the right arm to view the sweets on the table to make sure
        # some aren't already left from last time
        baxter.moveToSweets()
        comms.reset_sweets()
        num, centres, anglelist, centrepage = comms.get_sweet_client()

        # Count variable - on the initial run the arm needs to move to grab the
        # bowl, otherwise it can just grab the bowl
        count = 0
        # This loop runs until there are enough sweets on the page for the request
        while (num[2] < blueRequest or num[1] < greenRequest or \
            num[0] < redRequest):
            # Move to and then grab and tilt the bowl twice on the table
            if count == 0:
                baxter.move_to_grab_bowl(bowlX , bowlY, bowlZ)
                baxter.tip_bowl(bowlX , bowlY, bowlZ, page[0], page[1], True, count)
            else:
                baxter.tip_bowl(bowlX , bowlY, bowlZ, page[0], page[1], False, count)
            count = count + 1

            baxter._right_grip.open()
            baxter.moveToSweets()
            comms.reset_sweets()
            num, centres, anglelist, centrepage = comms.get_sweet_client()

            if (num[0] != 0):
                print str(num[0])," red sweets found on the table"
            if (num[1] != 0):
                print str(num[1]), " green sweets found on the table"
            if (num[2] != 0):
                print str(num[2]), " blue sweets found on the table"

            if (num[2] >= blueRequest and num[1] >= greenRequest \
                and num[0] >= redRequest):
                print "Total number of sweets is greater than request"
                break

            baxter._right_grip.open()

        # If all the requested sweets are found, drop the bowl and move the arm
        # away
        baxter._left_grip.open()
        baxter.left_interface.set_joint_position_speed(0.5)
        baxter.move_and_rotate("left", 0.5815, 0.1831, 0.1008, 3.1209, 0.0492, 2.8580)

        # If baxter hasn't given the customer enough red sweets
        while baxter.redGiven < redRequest:
            # Convert the given sweet positions list to a better format
            points = []
            for i in range(0, ((len(centres)+1)/3)):
                points.append((centres[(i*3)+0], centres[(i*3)+1], centres[(i*3)+2]))
            # Get the red sweets angles and points given the red amount
            redpoints = points[0:num[0]]
            redangles = anglelist[0:num[0]]
            # Cycle through all red sweets
            for i in range(0, len(redpoints)):
                # Using the first sweet in the list
                currpoint = redpoints[i]
                # Pick up the sweet, specifying the pickup angle and the xyz of
                # the sweet
                baxter.move_and_pick_up_sweet(0, redangles[i], currpoint[0], \
                                   currpoint[1], currpoint[2], page[0], page[1])
                # Break from the loop at any point when customer has been given
                # all sweets - check performed in and at the end of the loop
                if baxter.redGiven == redRequest:
                    print "All ",str(redRequest)," red sweets have been given to the customer"
                    break
            if baxter.redGiven == redRequest:
                print "All ",str(redRequest)," red sweets have been given to the customer"
                break
            # If some have been missed/not all given then get the new positions
            # of the red sweets and try again
            baxter.moveToSweets()
            comms.reset_sweets()
            num, centres, anglelist, page = comms.get_sweet_client()
            print str(baxter.redGiven)," red sweets have been given to the customer"

        # After the sweets have been given, state that Baxter is done
        if (redRequest != 0):
            baxter.speak('Here are your red sweets.')

        # Do the same for green sweets
        while baxter.greenGiven < greenRequest:
            points = []
            # Convert the given sweet positions list to a better format
            for i in range(0, ((len(centres)+1)/3)):
                points.append((centres[(i*3)+0], centres[(i*3)+1], centres[(i*3)+2]))
            # Get the green sweets angles and points given the green amount
            greenpoints = points[num[0]:num[0]+num[1]]
            greenangles = anglelist[num[0]:num[0]+num[1]]
            # Cycle through all green sweets
            for i in range(0, len(greenpoints)):
                currpoint = greenpoints[i]
                # Pick up the sweet, specifying the pickup angle and the xyz of
                # the sweet
                baxter.move_and_pick_up_sweet(1, greenangles[i], currpoint[0], \
                                   currpoint[1], currpoint[2], page[0], page[1])
                # Break from the loop at any point when customer has been given
                # all sweets - check performed in and at the end of the loop
                if baxter.greenGiven == greenRequest:
                    print "All ",str(greenRequest)," green sweets have been given to the customer"
                    break
            if baxter.greenGiven == greenRequest:
                print "All ",str(greenRequest)," green sweets have been given to the customer"
                break
            # If some have been missed/not all given then get the new positions
            # of the red sweets and try again
            baxter.moveToSweets()
            comms.reset_sweets()
            num, centres, anglelist, page = comms.get_sweet_client()
            print str(baxter.greenGiven)," green sweets have been given to the customer"

        # After the sweets have been given, state that Baxter is done
        if (greenRequest != 0):
            baxter.speak('Here are your green sweets.')

        while baxter.blueGiven < blueRequest:
            # Convert the given sweet positions list to a better format
            points = []
            for i in range(0, ((len(centres)+1)/3)):
                points.append((centres[(i*3)+0], centres[(i*3)+1], centres[(i*3)+2]))
            # Get the blue sweets angles and points given the blue amount
            bluepoints = points[num[0]+num[1]:len(points)]
            blueangles = anglelist[num[0]+num[1]:len(points)]
            # Cycle through all blue sweets
            for i in range(0, len(bluepoints)):
                currpoint = bluepoints[i]
                # Pick up the sweet, specifying the pickup angle and the xyz of
                # the sweet
                baxter.move_and_pick_up_sweet(2, blueangles[i], currpoint[0], \
                                   currpoint[1], currpoint[2], page[0], page[1])
                # Break from the loop at any point when customer has been given
                # all sweets - check performed in and at the end of the loop
                if baxter.blueGiven == blueRequest:
                    print "All ",str(blueRequest)," blue sweets have been given to the customer"
                    break
            if baxter.blueGiven == blueRequest:
                print "All ",str(blueRequest)," blue sweets have been given to the customer"
                break
            baxter.moveToSweets()
            comms.reset_sweets()

            num, centres, anglelist, page = comms.get_sweet_client()
            print str(baxter.blueGiven)," blue sweets have been given to the customer"

        # After the sweets have been given, state that Baxter is done
        if (blueRequest != 0):
            baxter.speak('Here are your blue sweets.')

        # ====================================================================
        #                 Begining of Baxter Cashier Integration
        # ====================================================================

        grand_total = blueRequest + greenRequest + redRequest
        cashier.amount_due = grand_total
        cashier.interact_with_customer()

        # ====================================================================
        #                 End of Baxter Cashier Integration
        # ====================================================================


        # Baxter is happy at the end of the process - thanks customer etc
        send_image('happy')
        baxter.speak('Here you go. Thankyou for visiting my shop')
        print"Success! Customer has all their sweets!"

        # Reset given values for next customer
        baxter.redGiven = 0
        baxter.blueGiven = 0
        baxter.greenGiven = 0

        # Wait for customer to exit before looping to the beginning and Waiting
        # for a new customer
        comms.wait_for_person("exit")
