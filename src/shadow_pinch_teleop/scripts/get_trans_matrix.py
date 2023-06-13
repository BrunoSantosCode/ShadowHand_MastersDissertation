#!/usr/bin/env python3

#* * * * * * * * * * * get_trans_matrix.py * * * * * * * * * *#
#*  Get transformation matrix from rh_fftip to ra_tool0      *#
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *#

import tf
import rospy
import numpy as np
from termcolor import colored
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from sr_robot_commander.sr_robot_commander import SrRobotCommander

if __name__ == "__main__":
    rospy.init_node('ur5_tests')

    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()

    # Shadow Arm commander
    robot_commander = SrRobotCommander(name='right_arm_and_hand')

    print('\n' + colored('"ur5_tests" ROS node is ready!', 'green') + '\n')

    # Get Shadow Hand Forefinger tip Pose
    try:
        tf_listener.waitForTransform('ra_base_link', 'rh_fftip', rospy.Time(), rospy.Duration(1.0))
        (fftip_translation, fftip_rotation) = tf_listener.lookupTransform('ra_base_link', 'rh_fftip', rospy.Time())
        fftip_pose = tf.TransformerROS().fromTranslationRotation(fftip_translation, fftip_rotation)
        print('FFTip Position:')
        print(fftip_translation)
        print('FFTip Orientation:')
        print(fftip_rotation)
        print()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr('Failed to get FFTip pose!')

    # Get UR5 End-Effector Pose
    try:
        tf_listener.waitForTransform('ra_base_link', 'ra_tool0', rospy.Time(), rospy.Duration(1.0))
        (arm_translation, arm_rotation) = tf_listener.lookupTransform('ra_base_link', 'ra_tool0', rospy.Time())
        arm_pose = tf.TransformerROS().fromTranslationRotation(arm_translation, arm_rotation)
        print('UR5 Position:')
        print(arm_translation)
        print('UR5 Orientation:')
        print(arm_rotation)
        print()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr('Failed to get UR5 end-effector pose!')

    # FFTip in ra_tool0 frame
    try:
        tf_listener.waitForTransform('rh_fftip', 'ra_tool0', rospy.Time(), rospy.Duration(1.0))
        (fftip_arm_translation, fftip_arm_rotation) = tf_listener.lookupTransform('rh_fftip', 'ra_tool0', rospy.Time())
        fftip_arm_pose = tf.TransformerROS().fromTranslationRotation(fftip_arm_translation, fftip_arm_rotation)
        print('UR5 in FFTip frame Position:')
        print(fftip_arm_translation)
        print('UR5 in FFTip frame Orientation:')
        print(fftip_arm_rotation)
        print()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr('Failed to get UR5 pose in FFTip frame!')

    # UR5 in FFTip frame Position:
    trans = np.array([-0.03511002047890843, 0.010341343736855607, -0.43800045027971124])
    # UR5 in FFTip frame Orientation:
    rot = np.array([0.005110887124480915, -0.0007661677627845923, 0.9999866458110008, -4.407373799832884e-06])

    # Create fictitious FFTip frame
    tf_broadcaster.sendTransform((fftip_translation[0], fftip_translation[1], fftip_translation[2]),
                                 (fftip_rotation[0], fftip_rotation[1], fftip_rotation[2], fftip_rotation[3]),
                                 rospy.Time(0),
                                 'fictitious_fftip_frame',
                                 'ra_base_link')

    # Calculate UR5 position from FFTip position
    pose_in_fftip = PoseStamped()
    pose_in_fftip.header.stamp = rospy.Time(0)
    pose_in_fftip.header.frame_id = 'fictitious_fftip_frame'
    pose_in_fftip.pose.position.x = trans[0]
    pose_in_fftip.pose.position.y = trans[1]
    pose_in_fftip.pose.position.z = trans[2]
    pose_in_fftip.pose.orientation.x = rot[0]
    pose_in_fftip.pose.orientation.y = rot[1]
    pose_in_fftip.pose.orientation.z = rot[2]
    pose_in_fftip.pose.orientation.w = rot[3]
    try:
        tf_listener.waitForTransform('ra_base_link', pose_in_fftip.header.frame_id, rospy.Time(0), rospy.Duration(10.0))
        transformed_pose = tf_listener.transformPose('ra_base_link', pose_in_fftip)
        print('Calculated UR5 Position:')
        print(transformed_pose.pose.position)
        print('Calculated UR5 Orientation:')
        print(transformed_pose.pose.orientation)
        print()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr('Failed to calculate UR5 position in base link frame!')
