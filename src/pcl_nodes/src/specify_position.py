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
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

from baxter_interface import CHECK_VERSION

import tf

def ik_test(limb, positions):
    # Convert specified arm position to floats
    x = float(positions[0])
    y = float(positions[1])
    z = float(positions[2])
    # Cap at baxter's coordinate limits so as not to hit anything
    if x > 0.80:
        x = 0.80
    if x < 0.50:
         x = 0.50
    if y > -0.10:
        y = -0.10
    if y < -0.40:
       y = -0.40
    if z > 0.2:
        z = 0.2
    if z < 0.0:
        z = 0.0

    rospy.init_node("rsdk_ik_move")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.657579481614,
                    y=0.851981417433,
                    z=0.0388352386502,
                ),
                orientation=Quaternion(
                    x=-0.366894936773,
                    y=0.885980397775,
                    z=0.108155782462,
                    w=0.262162481772,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=x, #x=0.656982770038,
                    y=y, #y=-0.852598021641,
                    z=z, #z=0.0388609422173,
                ),
                orientation=Quaternion(
                    x=0, #x=0.367048116303,
                    y=1, #y=0.885911751787,
                    z=0, #z=-0.108908281936,
                    w=0, #w=0.261868353356,
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp
        print "X = "+str(x)+", Y = "+str(y)+", Z = "+str(z)

        arm = baxter_interface.Limb(limb)
        while not rospy.is_shutdown():
            arm.set_joint_positions(limb_joints)
            rospy.sleep(0.1)


    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return 0


def main():
    """RSDK Inverse Kinematics Example

    A simple example of using the Rethink Inverse Kinematics
    Service which returns the joint angles and validity for
    a requested Cartesian Pose.

    Run this example, passing the *limb* to test, and the
    example will call the Service with a sample Cartesian
    Pose, pre-defined in the example code, printing the
    response of whether a valid joint solution was found,
    and if so, the corresponding joint angles.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )
    parser.add_argument(
        '-p', '--position', nargs = 3, required=True,
        help="please add an x, y and z coordinate"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    return ik_test(args.limb, [args.position[0], args.position[1], args.position[2]])

if __name__ == '__main__':
    sys.exit(main())
