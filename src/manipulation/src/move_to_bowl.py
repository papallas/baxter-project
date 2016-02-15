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

# def pointReceive(point):
#     # Convert specified arm position to floats
#     x = float(point.point[0]])
#     y = float(point.point[1])
#     z = 0.0
#
#     # Cap at baxter's coordinate limits so as not to hit anything
#
#     ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
#     iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
#     ikreq = SolvePositionIKRequest()
#     hdr = Header(stamp=rospy.Time.now(), frame_id='torso')
#     poses = {
#         'left': PoseStamped(
#             header=hdr,
#             pose=Pose(
#                 position=Point(
#                     x=x, #x=0.656982770038,
#                     y=y-0.05, #y=-0.852598021641,
#                     z=z, #z=0.0388609422173,
#                 ),
#                 orientation=Quaternion(
#                     x=0, #x=0.367048116303,
#                     y=1, #y=0.885911751787,
#                     z=0, #z=-0.108908281936,
#                     w=0, #w=0.261868353356,
#                 ),
#             ),
#         )
#         'right': PoseStamped(
#             header=hdr,
#             pose=Pose(
#                 position=Point(
#                     x=x, #x=0.656982770038,
#                     y=y+0.05, #y=-0.852598021641,
#                     z=z, #z=0.0388609422173,
#                 ),
#                 orientation=Quaternion(
#                     x=0, #x=0.367048116303,
#                     y=1, #y=0.885911751787,
#                     z=0, #z=-0.108908281936,
#                     w=0, #w=0.261868353356,
#                 ),
#             ),
#         ),
#     }
#
#     ikreq.pose_stamp.append(poses["left"])
#     try:
#         rospy.wait_for_service(ns, 5.0)
#         resp = iksvc(ikreq)
#     except (rospy.ServiceException, rospy.ROSException), e:
#         rospy.logerr("Service call failed: %s" % (e,))
#         return 1
#
#     # Check if result valid, and type of seed ultimately used to get solution
#     # convert rospy's string representation of uint8[]'s to int's
#     resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
#                                resp.result_type)
#     if (resp_seeds[0] != resp.RESULT_INVALID):
#         seed_str = {
#                     ikreq.SEED_USER: 'User Provided Seed',
#                     ikreq.SEED_CURRENT: 'Current Joint Angles',
#                     ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
#                    }.get(resp_seeds[0], 'None')
#         print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
#               (seed_str,))
#         # Format solution into Limb API-compatible dictionary
#         limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
#         print "\nIK Joint Solution:\n", limb_joints
#         print "------------------"
#         print "Response Message:\n", resp
#         print "X = "+str(x)+", Y = "+str(y)+", Z = "+str(z)
#
#         arm = baxter_interface.Limb("left")
#         while not rospy.is_shutdown():
#             arm.set_joint_positions(limb_joints)
#             rospy.sleep(0.1)
#
#
#     else:
#         print("INVALID POSE - No Valid Joint Solution Found.")
#
#     return 0
#
# def listener():
#     rospy.init_node("sdaadsads");
#
#     pub = rospy.Subscriber('/scooppos', PointStamped, pointReceive)
#
#     rospy.spin()
#
# if __name__ == '__main__':
#     listener()
def main():
    print "Moving Baxter to scoop sweets"
