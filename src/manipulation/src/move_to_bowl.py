#!/usr/bin/env python

import argparse
import sys
import struct
import threading

from copy import copy
import numpy as np

import rospy
import actionlib

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
from manipulation.msg import *

import baxter_interface

from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import EndEffectorState

import tf

import math

from std_msgs.msg import UInt16


# move a limb
# update pose in x and y direction
# initialise ros node
rospy.init_node("pick_up_sweets", anonymous = True)

global gripped

global redGiven
global greenGiven
global blueGiven

redGiven = 0
greenGiven = 0
blueGiven = 0

class locateAndMove():

    def __init__(self):
        self.right_interface = baxter_interface.Limb("right")
        self.left_interface = baxter_interface.Limb("left")

        self.right_interface.set_joint_position_speed(0.1)
        self.left_interface.set_joint_position_speed(0.1)


        # START ARMS IN INITIAL UNTUCKED POSITION
        self.init_left_x = 0.5793                      # x     = front back
        self.init_left_y = 0.1883                      # y     = left right
        self.init_left_z = 0.15                        # z     = up down
        self.left_roll        = 3.1024              # roll  = horizontal
        self.left_pitch       = 0.0446               # pitch = vertical
        self.left_yaw         = 2.8645               # yaw   = rotation

        self.left_pose = [self.init_left_x, self.init_left_y, self.init_left_z,     \
                     self.left_roll, self.left_pitch, self.left_yaw]

        # START ARMS IN INITIAL UNTUCKED POSITION
        self.init_right_x = 0.5797                      # x     = front back
        self.init_right_y = -0.1910                     # y     = left right
        self.init_right_z = 0.1013                        # z     = up down
        self.right_roll   = -3.0999              # roll  = horizontal
        self.right_pitch  = 0.0441               # pitch = vertical
        self.right_yaw    = -2.8676               # yaw   = rotation

        self.right_pose = [self.init_right_x, self.init_right_y, self.init_right_z,     \
                  self.right_roll, self.right_pitch, self.right_yaw]

        self._left_grip = baxter_interface.Gripper('left', CHECK_VERSION)
        self._right_grip = baxter_interface.Gripper('right', CHECK_VERSION)
        # calibrate
        #self._right_grip.calibrate()
        # self._left_grip_state   = 'open'
        self._right_grip_state  = 'open'

        self._left_grip.calibrate()
        self._right_grip.calibrate()

        # control parameters
        self._pub_rate          = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._rate              = 500.0  # Hz
        self._max_speed         = .05
        self._arm               = 'none'
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        # set joint state publishing to 500Hz
        self._pub_rate.publish(self._rate)

        self.twist = np.array([[.0],[.0],[.0],[.0],[.0],[.0]])

        self.MINZ = -0.215

        self.head = baxter_interface.Head()


    def move_to_point(self, limb, x, y, z):
        if limb == "both":
            #UPDATE RIGHT ARM FIRST IF POSITION ON RIGHT
            if y < self.right_pose[1]:
                poseRight = [x, y-0.10, self.right_pose[2], self.right_pose[3],
                self.right_pose[4], self.right_pose[5]]
                self.baxter_ik_move("right", poseRight)
                # UPDATE LEFT ARM
                poseLeft = [x, y+0.10, self.left_pose[2], self.left_pose[3], self.left_pose[4], self.left_pose[5]]
                self.baxter_ik_move("left", poseLeft)

            #UPDATE LEFT ARM FIRST IF POSITION ON LEFT
            else:
                # UPDATE LEFT ARM
                poseLeft = [x, y+0.10, self.left_pose[2], self.left_pose[3], self.left_pose[4], self.left_pose[5]]
                self.baxter_ik_move("left", poseLeft)
                poseRight = [x, y-0.10, self.right_pose[2], self.right_pose[3], self.right_pose[4], self.right_pose[5]]
                self.baxter_ik_move("right", poseRight)
        if limb == "right":
            #if y > self.left_pose[1] - 0.3:
                # UPDATE LEFT ARM
            #poseLeft = [x-0.15, y+0.3, z + 0.2, self.left_pose[3], self.left_pose[4], self.left_pose[5]]
            #poseLeft = [x, y+0.10, self.left_pose[2], self.left_pose[3], self.left_pose[4], self.left_pose[5]]
            #left_thread = threading.Thread(self.baxter_ik_move("left", poseLeft))
            poseRight = [x, y, z, self.right_pose[3], self.right_pose[4], self.right_pose[5]]
            #poseRight = [x, y-0.10, self.right_pose[2], self.right_pose[3], self.right_pose[4], self.right_pose[5]]
            self.baxter_ik_move("right", poseRight)
            self.right_pose = [x, y, z, self.right_pose[3], self.right_pose[4], self.right_pose[5]]

            #else:
                #pass
                #poseRight = [x, y, z + 0.02, self.right_pose[3], self.right_pose[4], self.right_pose[5]]
                #self.baxter_ik_move("right", poseRight)

    def rotate_to_pos(self, limb, r, p, y):
        if limb == "right":
            poseRight = [self.right_pose[0], self.right_pose[1], self.right_pose[2], r, p, y]
            self.baxter_ik_move("right", poseRight)

    def move_and_rotate(self, limb, x, y, z, r, p, ya):
        if limb == "right":
            poseRight = [x, y, z, r, p, ya]
            self.baxter_ik_move("right", poseRight)
            self.right_pose = [x, y, z, r, p, ya]
        if limb == "left":
            poseLeft = [x, y, z, r, p, ya]
            self.baxter_ik_move("left", poseLeft)
            self.left_pose = [x, y, z, r, p, ya]

    def list_to_pose_stamped(self, pose_list, target_frame):
        pose_msg = PoseStamped()
        pose_msg.pose = self.list_to_pose(pose_list)
        pose_msg.header.frame_id = target_frame
        pose_msg.header.stamp = rospy.Time.now()
        return pose_msg

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
           q = tf.transformations.quaternion_from_euler(pose_list[3], pose_list[4], pose_list[5])
           pose_msg.orientation.x = q[0]
           pose_msg.orientation.y = q[1]
           pose_msg.orientation.z = q[2]
           pose_msg.orientation.w = q[3]
       else:
           raise MoveItCommanderException("Expected either 6 or 7 elements in list: (x,y,z,r,p,y) or (x,y,z,qx,qy,qz,qw)")
       return pose_msg


    def baxter_ik_move(self, limb, rpy_pose):
        if (rpy_pose[2] <  self.MINZ):
            rpy_pose[2] =  self.MINZ
        quaternion_pose = self.list_to_pose_stamped(rpy_pose, "base")

        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")

        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")

        if ik_response.isValid[0]:
            # print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            if limb == "left":
                self.left_interface.move_to_joint_positions(limb_joints)
            elif limb == "right":
                self.right_interface.move_to_joint_positions(limb_joints)
        else:
            # display invalid move message on head display
            #self.splash_screen("Invalid", "move")
            # little point in continuing so exit with error message
            print "requested move =", rpy_pose
            sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")

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
        print 'roll       = %5.4f radians %5.4f degrees' % (euler[0], 180.0 * euler[0]/math.pi)
        print 'pitch      = %5.4f radians %5.4f degrees' % (euler[1], 180.0 * euler[1] /math.pi)
        print 'yaw        = %5.4f radians %5.4f degrees' % (euler[2], 180.0 * euler[2] /math.pi)

        quaternion_pose2 = self.left_interface.endpoint_pose()
        position2        = quaternion_pose2['position']
        quaternion2      = quaternion_pose2['orientation']
        euler2           = tf.transformations.euler_from_quaternion(quaternion2)

        print "\nLEFT POSITION\n"
        print 'front back = %5.4f ' % position2[0]
        print 'left right = %5.4f ' % position2[1]
        print 'up down    = %5.4f ' % position2[2]
        print 'roll       = %5.4f radians %5.4f degrees' % (euler2[0], 180.0 * euler2[0]/math.pi)
        print 'pitch      = %5.4f radians %5.4f degrees' % (euler2[1], 180.0 * euler2[1] /math.pi)# UPDATE LEFT ARM
        print 'yaw        = %5.4f radians %5.4f degrees' % (euler2[2], 180.0 * euler2[2] /math.pi)

    def get_pos_client(self):
        print "\nRequesting bowl position in Baxter coordinates"
        rospy.wait_for_service('bowl_pos_req')
        try:
            bowl_pos_req = rospy.ServiceProxy('bowl_pos_req', RequestBowlPos)
            resp = bowl_pos_req("")
            return [resp.x, resp.y, resp.z]
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # FIRST BASIC SCOOP TRIAL
    def grab_sweets_from_above(self, x, y, z):
        # MOVE TO SEE SWEETS WITH CAMERA
        #self.move_to_point("right",x+0.03,y,z+0.05)
        # MOVE TO START SCOOP POSITION

        #self.move_to_point("right",x,y-0.07,z+0.04)
        self.print_arm_pose()
        #self.rotate_to_pos("right", self.right_pose[3], -0.8, 1.3)
        self._right_grip.open()

        self.move_and_rotate("right", x, y, -0.16,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        self.move_and_rotate("right", x, y, z-0.029,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        self._right_grip.close()

        self.move_and_rotate("right", x, y, z+0.1,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        self.moveSweetToArea()

    # FIRST BASIC SCOOP TRIAL
    def shake_in_bowl(self, x, y, z):
        # MOVE TO SEE SWEETS WITH CAMERA
        #self.move_to_point("right",x+0.03,y,z+0.05)
        # MOVE TO START SCOOP POSITION

        #self.move_to_point("right",x,y-0.07,z+0.04)
        self.print_arm_pose()
        #self.rotate_to_pos("right", self.right_pose[3], -0.8, 1.3)
        self._right_grip.open()

        self.move_and_rotate("right", x, y, z+0.06,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        self.move_and_rotate("right", x, y, self.MINZ,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        self.right_interface.set_joint_position_speed(0.3)

        self.move_and_rotate("right", x, y-0.01, self.right_pose[2]+0.002,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        self.move_and_rotate("right", x, y+0.01, self.right_pose[2],
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        self.move_and_rotate("right", x, y-0.01, self.right_pose[2]-0.002,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        self.right_interface.set_joint_position_speed(0.1)

        self._right_grip.close()
        self.move_and_rotate("right", self.right_pose[0], self.right_pose[1], self.right_pose[2]+0.09,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        self.move_and_rotate("right", self.right_pose[0], self.right_pose[1]-0.3, self.right_pose[2],
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        self._right_grip.open()

    # FIRST BASIC SCOOP TRIAL
    def scoop_from_side(self, x, y, z):
        self.print_arm_pose()

        self._right_grip.close()

        self.move_and_rotate("right", x, y-0.1, z+0.02, self.right_pose[3], -0.8, 1.3)

        self.move_and_rotate("right", self.right_pose[0], self.right_pose[1]+0.03, self.right_pose[2]-0.02,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])
        #
        self.move_and_rotate("right", self.right_pose[0], self.right_pose[1]+0.05, self.right_pose[2]-0.03,
        self.right_pose[3],self.right_pose[4]-0.15,self.right_pose[5])

        self._right_grip.calibrate()

        self.move_and_rotate("right", self.right_pose[0], self.right_pose[1]+0.02, self.right_pose[2],
        self.right_pose[3],self.right_pose[4],self.right_pose[5])


    def grab_bowl_and_tilt(self, x, y, z):

        self._left_grip.calibrate()
        self._right_grip.calibrate()
        #self._left_grip.open()
        self.move_and_rotate("left", x, y+0.069, z+0.05, self.left_pose[3], self.left_pose[4], self.left_pose[5])

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], self.left_pose[2],
        self.left_pose[3]+0.3,self.left_pose[4],self.left_pose[5])

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1]-0.03, -0.19,
        self.left_pose[3],self.left_pose[4],self.left_pose[5])

        self._left_grip.close()

        self.left_interface.set_joint_position_speed(0.5)

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], self.left_pose[2]+0.05,
        self.left_pose[3]+0.3,self.left_pose[4],self.left_pose[5])

        move.scoop_from_side(self.left_pose[0], self.left_pose[1], self.left_pose[2])

        self._right_grip.close()
        self.move_and_rotate("right", self.right_pose[0], self.right_pose[1], self.right_pose[2]+0.2,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        self._left_grip.open()

        self.left_interface.set_joint_position_speed(0.1)

        self.moveSweetToArea()

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], self.left_pose[2]+0.35,
        self.left_pose[3]+0.3,self.left_pose[4],self.left_pose[5])

    def tilt_shake_grab(self, x, y, z):

        self._left_grip.calibrate()
        self._right_grip.calibrate()
        #self._left_grip.open()
        self.move_and_rotate("left", x, y+0.069, z+0.05, self.left_pose[3], self.left_pose[4], self.left_pose[5])

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], self.left_pose[2],
        self.left_pose[3]+0.3,self.left_pose[4],self.left_pose[5])

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1]-0.03, -0.19,
        self.left_pose[3],self.left_pose[4],self.left_pose[5])

        self._left_grip.close()

        self.left_interface.set_joint_position_speed(0.5)

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], self.left_pose[2]+0.05,
        self.left_pose[3]+0.3,self.left_pose[4],self.left_pose[5])

        for i in range(0,5):
            # self._pub_rate.publish(500)
            #
            # move = {'left_w0': 5, 'left_w1': 5, 'left_w2': 5, 'left_e0': 5, 'left_e1': 5, 'left_s0': 5, 'left_s1': 5}
            # move2 = {'left_w0': -1, 'left_w1': -1, 'left_w2': -1, 'left_e0': -1, 'left_e1': -1, 'left_s0': -1, 'left_s1': -1}
            # move3 = {'left_w0': 0, 'left_w1': 0, 'left_w2': 0, 'left_e0': 0, 'left_e1':  0, 'left_s0': 0, 'left_s1': 0}
            #
            # self.left_interface.set_joint_velocities(move)
            # self.left_interface.set_joint_velocities(move)
            # self.left_interface.set_joint_velocities(move)
            # self.left_interface.set_joint_velocities(move)
            # self.left_interface.set_joint_velocities(move3)
            # self.left_interface.set_joint_velocities(move2)
            # self.left_interface.set_joint_velocities(move3)

            self.move_and_rotate("left", self.left_pose[0]-0.04, self.left_pose[1], self.left_pose[2]+0.03,
            self.left_pose[3],self.left_pose[4],self.left_pose[5])

            self.move_and_rotate("left", self.left_pose[0]+0.04, self.left_pose[1], self.left_pose[2]-0.03,
            self.left_pose[3],self.left_pose[4],self.left_pose[5])

            move.scoop_from_side(self.left_pose[0], self.left_pose[1], self.left_pose[2])

            self._right_grip.close()
            self.move_and_rotate("right", self.right_pose[0], self.right_pose[1], self.right_pose[2]+0.2,
            self.right_pose[3],self.right_pose[4],self.right_pose[5])

            self.moveSweetToArea()

    def move_to_grab_bowl(self, x, y, z):
        self.move_and_rotate("left", x, y+0.05, -0.15, self.left_pose[3], self.left_pose[4], self.left_pose[5])

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], self.left_pose[2],
        self.left_pose[3]+0.3,self.left_pose[4],self.left_pose[5])

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1]-0.03, -0.197,
        self.left_pose[3],self.left_pose[4],self.left_pose[5])

    def bowl_tip_trial(self, x, y, z):
        self._left_grip.close()

        self.left_interface.set_joint_position_speed(0.2)

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], self.left_pose[2]+0.05,
        self.left_pose[3],self.left_pose[4],self.left_pose[5])

        self.left_interface.set_joint_position_speed(0.1)
        # -0.17
        self.move_and_rotate("left", 0.59, -0.25, self.left_pose[2]+0.02,
        self.left_pose[3],self.left_pose[4],self.left_pose[5])

        self.left_interface.set_joint_position_speed(0.5)

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], self.left_pose[2]+0.02,
        self.left_pose[3]-0.3,self.left_pose[4],self.left_pose[5])

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], self.left_pose[2]-0.02,
        self.left_pose[3]+0.3,self.left_pose[4],self.left_pose[5])

        self.move_and_rotate("left", 0.59, -0.15, self.left_pose[2],
        self.left_pose[3],self.left_pose[4],self.left_pose[5])

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], self.left_pose[2]+0.02,
        self.left_pose[3]-0.4,self.left_pose[4],self.left_pose[5])

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], self.left_pose[2]-0.02,
        self.left_pose[3]+0.4,self.left_pose[4],self.left_pose[5])

        self.left_interface.set_joint_position_speed(0.1)

        self.move_and_rotate("left", 0.59, self.left_pose[1]+0.24, self.left_pose[2],
        self.left_pose[3],self.left_pose[4],self.left_pose[5])

        self.move_and_rotate("left", self.left_pose[0], self.left_pose[1], -0.15,
        self.left_pose[3],self.left_pose[4],self.left_pose[5])

    def moveSweetToArea(self):
        self.move_and_rotate("right", 0.5, -0.25, -0.1, self.right_pose[3]
            , self.right_pose[4], self.right_pose[5])
        self._right_grip.open()

    def moveToSweets(self):
        self.move_and_rotate("right", 0.8, -0.25, 0.15, self.right_pose[3]
            , self.right_pose[4], self.right_pose[5])

    def move_and_pick_up_sweet(self, colour, angle, x, y, z):
        self.right_interface.set_joint_position_speed(0.3)

        defaultrotation = -2.8676
        sweetOrientation = 90 / (180/math.pi)
        # overall = defaultrotation + sweetOrientation

        # overall = -2.8676 + (90/(180/math.pi)) + angle
        overall = -2.8676 + ((90/(180/math.pi)) - angle)

        self.move_and_rotate("right", x, y, -0.15,
        self.right_pose[3],self.right_pose[4],overall)

        self.right_interface.set_joint_position_speed(0.1)

        self.move_and_rotate("right", x, y, -0.215,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        self._right_grip.close()

        self.move_and_rotate("right", self.right_pose[0], self.right_pose[1], -0.1,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        global gripped

        if gripped == True:
            self.right_interface.set_joint_position_speed(0.3)

            self.move_and_rotate("right", 0.7, 0, 0,
            self.right_pose[3],self.right_pose[4],-2.8676)

            self._right_grip.open()

            if colour == 0:
                global redGiven
                redGiven = redGiven + 1
            if colour == 1:
                global greenGiven
                greenGiven = greenGiven + 1
            if colour == 2:
                global blueGiven
                blueGiven = blueGiven + 1

        if gripped == False:
            self._right_grip.open()

        # self.move_and_rotate("right", 0.75, -0.25, 0, self.right_pose[3]
        #     , self.right_pose[4], self.right_pose[5])

    def separate_collision(self, direction, x, y, z):
        self.right_interface.set_joint_position_speed(0.3)

        self._right_grip.close()

        self.move_and_rotate("right", x, y-0.1, 0,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])


        if direction == 1:
            self.move_and_rotate("right", x+0.05, y, -0.205,
            self.right_pose[3],self.right_pose[4],self.right_pose[5])

            self.move_and_rotate("right", x-0.05, y-0.03, -0.205,
            self.right_pose[3],self.right_pose[4],self.right_pose[5])

        if direction == 0:
            self.move_and_rotate("right", x-0.05, y, -0.205,
            self.right_pose[3],self.right_pose[4],self.right_pose[5])

            self.move_and_rotate("right", x+0.05, y, -0.205,
            self.right_pose[3],self.right_pose[4],self.right_pose[5])

        self.move_and_rotate("right", self.right_pose[0], self.right_pose[1], -0.1,
        self.right_pose[3],self.right_pose[4],self.right_pose[5])

        self.right_interface.set_joint_position_speed(0.1)


    def reset_bowl(self):
        print "Detecting Bowl.."
        rospy.wait_for_service('reset_bowl')
        try:
            reset_bowl_req = rospy.ServiceProxy('reset_bowl', LookForBowl)
            resp = reset_bowl_req("Reset")
            return resp.ok
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def reset_sweets(self):
        print "\nLooking at sweets on table"
        rospy.wait_for_service('reset_sweets')
        try:
            reset_sweets_req = rospy.ServiceProxy('reset_sweets', LookForSweets)
            resp = reset_sweets_req("Reset")
            return resp.ok
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def get_sweet_client(self):
        print "Requesting sweet number on table"
        rospy.wait_for_service('publish_sweet_info')
        try:
            sweet_number_req = rospy.ServiceProxy('publish_sweet_info', RequestSweetInfo)
            resp = sweet_number_req("")
            # return resp.n, resp.pos, resp.collisions, resp.collisionNum, resp.collisionPos, resp.angleList
            return resp.n, resp.pos, resp.angleList

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def get_voice_command(self):
        print "Waiting for voice command"
        rospy.wait_for_service('get_voice')
        try:
            voice_req = rospy.ServiceProxy('get_voice', VoiceRequest)
            resp = voice_req("")
            # return resp.n, resp.pos, resp.collisions, resp.collisionNum, resp.collisionPos, resp.angleList
            return resp.sweetNumber

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def wait_for_person(self):
        self.move_and_rotate("right", 0.4404, -0.5915, 0.0420,
        -2.9815,-0.0122,-0.7517)
        self.head.set_pan(-0.5)

        print "Waiting for voice command"
        rospy.wait_for_service('look_for_person')
        try:
            person_req = rospy.ServiceProxy('look_for_person', PersonRequest)
            resp = person_req("")
            return resp.ok
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

def gripperholding(state):
    if state.force > 1:
        global gripped
        gripped = True
    if state.force <= 1:
        global gripped
        gripped = False


if __name__ == '__main__':
    rospy.Subscriber("/robot/end_effector/right_gripper/state", EndEffectorState, gripperholding)

    move = locateAndMove()
    move.print_arm_pose()
    move.wait_for_person()



#     while True:
# `       `
#
#         global overallGiven
#         overallGiven = 0
#         global redGiven
#         redGiven = 0
#         global blueGiven
#         blueGiven = 0
#         global greenGiven
#         greenGiven = 0
#         # sweetRequests = move.get_voice_command();
#         # blueRequest = sweetRequests[2]
#         # greenRequest = sweetRequests[1]
#         # redRequest = sweetRequests[0]
#         blueRequest = 1
#         greenRequest = 1
#         redRequest = 1
#         print"Customer wants ",blueRequest," blue sweets, ",greenRequest," green sweets and ",redRequest," red sweets"
#
#
#         # ok = move.reset_bowl()
#         # rospy.sleep(6)
#         #
#         # [x, y, z] = move.get_pos_client()
#         # print "Current bowl position = " + str(x) + ", " + str(y) + ", " + str(z)
#         #
#         # move.move_to_grab_bowl(x, y, z)
#         num = [0,0,0]
#
#         while (num[2] < blueRequest or num[1] < greenRequest or num[0] < redRequest):
#             # move.bowl_tip_trial(x, y, z)
#             move._right_grip.open()
#             move.moveToSweets()
#             string = move.reset_sweets()
#
#             # GET NUMBER OF SWEETS AND THEIR CENTRES FROM FIND_COLOURED_SWEETS.PY NODE
#             # num, centres, collisions, collisionNum, collisionPos, anglelist = move.get_sweet_client()
#             num, centres, anglelist = move.get_sweet_client()
#
#             if (num[0] != 0):
#                 print str(num[0])," red sweets found on the table"
#             if (num[1] != 0):
#                 print str(num[1]), " green sweets found on the table"
#             if (num[2] != 0):
#                 print str(num[2]), " blue sweets found on the table"
#             # if (collisionNum != 0):
#             #     print str(collisionNum),"collisions of sweets"
#
#             if (num[2] >= blueRequest and num[1] >= greenRequest and num[0] >= redRequest):
#                 print "Total number of sweets is greater than request"
#                 break
#
#             move._right_grip.open()
#
#         numGiven = 0
#
#         global redGiven
#         while redGiven < redRequest:
#             points = []
#             # print (len(centres)+1)/3
#             for i in range(0, ((len(centres)+1)/3)):
#                 points.append((centres[(i*3)+0], centres[(i*3)+1], centres[(i*3)+2]))
#
#             redpoints = points[0:num[0]]
#             redangles = anglelist[0:num[0]]
#             for i in range(0, len(redpoints)):
#                 currpoint = redpoints[i]
#                 move.move_and_pick_up_sweet(0, redangles[i], currpoint[0], currpoint[1], currpoint[2])
#                 if redGiven == redRequest:
#                     print "All ",str(redRequest)," red sweets have been given to the customer"
#                     break
#             if redGiven == redRequest:
#                 print "All ",str(redRequest)," red sweets have been given to the customer"
#                 break
#             move.moveToSweets()
#             string = move.reset_sweets()
#
#             num, centres, anglelist = move.get_sweet_client()
#             print str(redGiven)," red sweets have been given to the customer"
#
#         global greenGiven
#         while greenGiven < greenRequest:
#             points = []
#             # print (len(centres)+1)/3
#             for i in range(0, ((len(centres)+1)/3)):
#                 points.append((centres[(i*3)+0], centres[(i*3)+1], centres[(i*3)+2]))
#
#             greenpoints = points[num[0]:num[0]+num[1]]
#             greenangles = anglelist[num[0]:num[0]+num[1]]
#             for i in range(0, len(greenpoints)):
#                 currpoint = greenpoints[i]
#                 move.move_and_pick_up_sweet(1, greenangles[i], currpoint[0], currpoint[1], currpoint[2])
#                 if greenGiven == greenRequest:
#                     print "All ",str(greenRequest)," green sweets have been given to the customer"
#                     break
#             if greenGiven == greenRequest:
#                 print "All ",str(greenRequest)," green sweets have been given to the customer"
#                 break
#             move.moveToSweets()
#             string = move.reset_sweets()
#
#             num, centres, anglelist = move.get_sweet_client()
#             print str(greenGiven)," green sweets have been given to the customer"
#
#         global blueGiven
#         while blueGiven < blueRequest:
#             points = []
#             # print (len(centres)+1)/3
#             for i in range(0, ((len(centres)+1)/3)):
#                 points.append((centres[(i*3)+0], centres[(i*3)+1], centres[(i*3)+2]))
#
#             bluepoints = points[num[0]+num[1]:len(points)]
#             blueangles = anglelist[num[0]+num[1]:len(points)]
#             for i in range(0, len(bluepoints)):
#                 currpoint = bluepoints[i]
#                 move.move_and_pick_up_sweet(2, blueangles[i], currpoint[0], currpoint[1], currpoint[2])
#                 if blueGiven == blueRequest:
#                     print "All ",str(blueRequest)," blue sweets have been given to the customer"
#                     break
#             if blueGiven == blueRequest:
#                 print "All ",str(blueRequest)," blue sweets have been given to the customer"
#                 break
#             move.moveToSweets()
#             string = move.reset_sweets()
#
#             num, centres, anglelist = move.get_sweet_client()
#             print str(blueGiven)," blue sweets have been given to the customer"
#
#         print"Success! Customer has all their sweets!"
