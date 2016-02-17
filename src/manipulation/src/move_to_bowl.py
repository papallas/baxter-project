#!/usr/bin/env python

import argparse
import sys
import struct

from copy import copy

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

import baxter_interface

from baxter_interface import CHECK_VERSION

import tf

import math

# move a limb
# update pose in x and y direction
# initialise ros node
rospy.init_node("pick_up_sweets", anonymous = True)

class locateAndMove():

    def __init__(self):
        self.right_interface = baxter_interface.Limb("right")
        self.left_interface = baxter_interface.Limb("left")

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

    def move_both_arms_to_point(self, x, y):
        #UPDATE RIGHT ARM FIRST IF POSITION ON RIGHT
        if y < self.right_pose[1]:
            poseRight = [x, y-0.10, self.right_pose[2], self.right_pose[3], self.right_pose[4], self.right_pose[5]]
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
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            if limb == "left":
                self.left_interface.move_to_joint_positions(limb_joints)
            elif limb == "right":
                self.right_interface.move_to_joint_positions(limb_joints)
        else:
            # display invalid move message on head display
            self.splash_screen("Invalid", "move")
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

    def grasp_gripper(self):
        return

if __name__ == '__main__':
    "Setting up Baxter movement"
    move = locateAndMove()

    move.print_arm_pose()

    [x, y, z] = move.get_pos_client()
    print "Found from client : " + str(x) + ", " + str(y) + ", " + str(z)

    #FOR EXAMPLE IF KINECT FRAME NOT WORKING
    x = 0.640673882798
    y = -0.05758041438
    z = -0.196769654272

    print "Cuurent bowl position = " + str(x) + ", " + str(y) + ", " + str(z)

    #move.move_both_arms_to_point(0.45, 0.2)

    #move.move_both_arms_to_point(x, y)

    #move.prepare_to_scoop()

    #move.grasp_gripper()
