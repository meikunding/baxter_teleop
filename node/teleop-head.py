#!/usr/bin/env python

"""
Baxter tele-control with kinect
"""
import argparse
import sys

import rospy
import tf2_ros
import math
import numpy as np

import baxter_interface
from baxter_interface import CHECK_VERSION
import FaceImage
BASE_FRAME = 'camera_depth_frame'
FRAMES = [
        'torso',
        'left_shoulder',
        'left_elbow',
        'left_hand',
        'right_shoulder',
        'right_elbow',
        'right_hand'
        ]

TEST_JOINT_ANGLES = dict()
TEST_JOINT_ANGLES['left'] = dict()
TEST_JOINT_ANGLES['right'] = dict()
TEST_JOINT_ANGLES['head'] = dict()
TEST_JOINT_ANGLES['left']['left_s0'] = 0.32366
TEST_JOINT_ANGLES['left']['left_s1'] = 0.86056
TEST_JOINT_ANGLES['left']['left_e0'] = -0.0456
TEST_JOINT_ANGLES['left']['left_e1'] = 0.59326
TEST_JOINT_ANGLES['left']['left_w0'] = 1.02930
TEST_JOINT_ANGLES['left']['left_w1'] = 0.0272
TEST_JOINT_ANGLES['left']['left_w2'] = -0.0276
TEST_JOINT_ANGLES['right']['right_s0'] = -0.4183
TEST_JOINT_ANGLES['right']['right_s1'] = 0.8862
TEST_JOINT_ANGLES['right']['right_e0'] = 0.2078
TEST_JOINT_ANGLES['right']['right_e1'] = 0.5821
TEST_JOINT_ANGLES['right']['right_w0'] = -1.35182
TEST_JOINT_ANGLES['right']['right_w1'] = 0.15684
TEST_JOINT_ANGLES['right']['right_w2'] = 0.29222
TEST_JOINT_ANGLES['head']['pitch'] = 0
TEST_JOINT_ANGLES['head']['yaw'] = 0
TEST_JOINT_ANGLES['head']['roll'] = 0

def get_joint_angles(tfBuffer, test, mirrored):
    """

    @param line: the line described in a list to process
    @param names: joint name keys
    """
    joint_angles = dict()
    joint_angles['left'] = dict()
    joint_angles['right'] = dict()
    joint_angles['head'] = dict()
    frames_all = []
    for i in range(5):
        frames = get_frame_positions(i+1, tfBuffer)
        if frames is None:
            continue
        frames_all.append(frames)
    if not frames_all:
        # if there's a problem with tracking, don't move
        joint_angles = TEST_JOINT_ANGLES
        return joint_angles
    depth_all = []
    for frames in frames_all:
        depth_all.append(-frames['torso'][2])
    closest_user = depth_all.index(min(depth_all))
    frames = frames_all[closest_user]
    head_quat = get_frame_head(closest_user+1, tfBuffer)
    print "The number of user is:%d, The closest user is:%d\n" % (len(frames_all), closest_user)
    if test:
        # use the hardcoded angles for debugging
        joint_angles = TEST_JOINT_ANGLES
    else:
        # do the math to find joint angles!
        reh = frames['right_hand'] - frames['right_elbow']
        res = frames['right_shoulder'] - frames['right_elbow']
        leh = frames['left_hand'] - frames['left_elbow']
        les = frames['left_shoulder'] - frames['left_elbow']
        rse = np.negative(res)
        lse = np.negative(les)
        qx = head_quat[0]
        qy = head_quat[1]
        qz = head_quat[2]
        qw = head_quat[3]

        # find down vector and normals
        #print "pitch: %f,  yaw:  %f, roll:  %f"% (pitch, yaw, roll)
        nt = np.cross(frames['right_shoulder'] - frames['torso'], frames['left_shoulder'] - frames['torso'])
        d = np.cross(nt, frames['right_shoulder'] - frames['left_shoulder'])
        rns = np.cross(d, rse)
        lns = np.cross(d, lse)
        lne = np.cross(leh, les)
        rne = np.cross(reh, res)

        # normalize vectors
        reh = reh / np.linalg.norm(reh)
        res = res / np.linalg.norm(res)
        leh = leh / np.linalg.norm(leh)
        les = les / np.linalg.norm(les)
        rse = rse / np.linalg.norm(rse)
        lse = lse / np.linalg.norm(lse)
        nt = nt / np.linalg.norm(nt)
        d = d / np.linalg.norm(d)
        rns = rns / np.linalg.norm(rns)
        lns = lns / np.linalg.norm(lns)
        lne = lne / np.linalg.norm(lne)
        rne = rne / np.linalg.norm(rne)

        # do the math to find joint angles
        joint_angles['left']['left_s0'] = np.arccos(np.dot(nt,lns)) - math.pi/5.0 # was + 0.0
        joint_angles['left']['left_s1'] = np.arccos(np.dot(d, lse)) - math.pi/2.0
        joint_angles['left']['left_e0'] = np.arccos(np.dot(nt, lne))
        joint_angles['left']['left_e1'] = math.pi - np.arccos(np.dot(leh, les))
        joint_angles['left']['left_w0'] = 0.0
        joint_angles['left']['left_w1'] = 0.0
        joint_angles['left']['left_w2'] = 0.0
        joint_angles['right']['right_s0'] = np.arccos(np.dot(nt,rns)) - math.pi*7.0/8.0 # was - pi
        joint_angles['right']['right_s1'] = np.arccos(np.dot(d, rse)) - math.pi/2.0
        joint_angles['right']['right_e0'] = np.arccos(np.dot(nt, rne)) - math.pi
        joint_angles['right']['right_e1'] = math.pi - np.arccos(np.dot(reh, res))
        joint_angles['right']['right_w0'] = 0.0
        joint_angles['right']['right_w1'] = 0.0
        joint_angles['right']['right_w2'] = 0.0
        joint_angles['head']['pitch'] = math.atan2(2*(qy*qz+qw*qx),qw*qw-qx*qx-qy*qy+qz*qz)
        joint_angles['head']['yaw'] = math.asin(-2*(qx*qz-qw*qy))
        joint_angles['head']['roll'] = math.atan2(2*(qx*qy+qw*qz),qw*qw+qx*qx-qy*qy-qz*qz)

    if mirrored:
        return joint_angles
    else:
        # this is the default option, where the teleoperator's
        # left/right arm corresponds to the robot's left/right arm
        unmirrored = dict()
        unmirrored['left'] = dict()
        unmirrored['right'] = dict()
        unmirrored['head'] = dict()
        unmirrored['left']['left_s0'] = joint_angles['right']['right_s0']
        unmirrored['left']['left_s1'] = joint_angles['right']['right_s1']
        unmirrored['left']['left_e0'] = joint_angles['right']['right_e0']
        unmirrored['left']['left_e1'] = joint_angles['right']['right_e1']
        unmirrored['left']['left_w0'] = 0.0
        unmirrored['left']['left_w1'] = 0.0
        unmirrored['left']['left_w2'] = 0.0
        unmirrored['right']['right_s0'] = joint_angles['left']['left_s0']
        unmirrored['right']['right_s1'] = joint_angles['left']['left_s1']
        unmirrored['right']['right_e0'] = joint_angles['left']['left_e0']
        unmirrored['right']['right_e1'] = joint_angles['left']['left_e1']
        unmirrored['right']['right_w0'] = 0.0
        unmirrored['right']['right_w1'] = 0.0
        unmirrored['right']['right_w2'] = 0.0
        unmirrored['head']['pitch'] = - math.atan2(2*(qy*qz+qw*qx),qw*qw-qx*qx-qy*qy+qz*qz)
        unmirrored['head']['yaw'] = - math.asin(-2*(qx*qz-qw*qy))
        unmirrored['head']['roll'] = - math.atan2(2*(qx*qy+qw*qz),qw*qw+qx*qx-qy*qy-qz*qz)
        return unmirrored



def get_frame_positions(user, tfBuffer):
    frame_positions = dict()
    try:
        for frame in FRAMES:
            transformation= tfBuffer.lookup_transform(BASE_FRAME, "%s_%d" % (frame, user), rospy.Time())
            translation = transformation.transform.translation
           # rotation = transformation.rotation.rotation
            pos = np.array([translation.x, translation.y, translation.z])
            frame_positions[frame] = pos
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print "Problem with kinect tracking."
        print e
        return None
    return frame_positions

def get_frame_head(user, tfBuffer):
    transformation= tfBuffer.lookup_transform(BASE_FRAME, "%s_%d" % ('head', user), rospy.Time())
    rotation = transformation.transform.rotation
    quat = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
    return quat

def teleoperate(rate, test, mirrored):
    """
    Teleoperates the robot based on tf2 frames.

    @param rate: rate at which to sample joint positions in ms

    """
    rate = rospy.Rate(rate)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    head = baxter_interface.Head()
    count = 0
    while not rospy.is_shutdown():
        rate.sleep()
        joint_angles = get_joint_angles(tfBuffer, test, mirrored)
        print joint_angles
        if joint_angles is not None:
            left.set_joint_positions(joint_angles['left'])
            right.set_joint_positions(joint_angles['right'])
            count = count + 1
            if count == 10:
                head_ang = (joint_angles['head']['yaw'])*10
            	if (head_ang > 0.8):
            		head_ang = 0.8
            	elif (head_ang < -0.8):
            		head_ang = -0.8
            	head.set_pan(head_ang)
               	count = 0

            """
            if joint_angles['head']['yaw'] > 0.02 :
            	print "joint_angles['head']['yaw']= %f", (joint_angles['head']['yaw'])
            	head.set_pan(0.7)
            if joint_angles['head']['yaw'] < -0.02 :
            	head.set_pan(-0.7)
            if joint_angles['head']['yaw'] > -0.02 and joint_angles['head']['yaw'] > 0.02 :
            	head.set_pan(0)
            if joint_angles['head']['pitch'] > 1.7:
            """

            #	head.command_nod()
            #if joint_angles['head']['pitch'] < 1.4:
            #	head.command_nod()

            #head.set_pan(angles['head_pan'])
        #if 'head_nod' in self.angles and self.angles['head_nod'] > 0.1:
            #head.command_nod()
            print "updated positions"
    print "Rospy shutdown, exiting loop."
    return True


def main():
    """
    Note: This version of simply drives the joints towards the next position at
    each time stamp. Because it uses Position Control it will not attempt to
    adjust the movement speed to hit set points "on time".
    """

    print("Initializing node... ")
    rospy.init_node("teleop_head")
    mirrored = rospy.get_param("~mirrored", False)
    rate = rospy.get_param("~rate", 6)
    test = rospy.get_param("~test", False)
    #user = rospy.get_param("~user", 1)
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    face = FaceImage.FaceImage()
    #head_init = baxter_interface.Head()
    #head_init.set_pan(0)
    def clean_shutdown():
        print("\nExiting...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)
    print("Enabling robot... ")
    rs.enable()
    face.set_image('waiting')
    rospy.sleep(5)
    face.set_image('intro')
    rospy.sleep(5)
    face.set_image('eyes')

    teleoperate(rate, test, mirrored)
if __name__ == '__main__':
    main()
