#!/usr/bin/env python3

#* * * * * * * * * * * ur5_tests.py  * * * * * * * * * * *#
#*  Receives a position in 'ra_base_link'                *#
#*  Moves the robot to a position x.xx m above (z-axis)  *#
#*  Moves the robot to the original position             *#
#*  Loop                                                 *#
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * *#

from hashlib import new
import tf
import rospy
import numpy as np
from termcolor import colored
from geometry_msgs.msg import PoseStamped
from sr_robot_commander.sr_arm_commander import SrArmCommander

# Init ROS
rospy.init_node('ur5_tests')

# Init TF Global Vars
tf_listener = tf.TransformListener()
tf_broadcaster = tf.TransformBroadcaster()

# UR5 Poses

# Default
pose_default = PoseStamped()
pose_default.pose.position.x = 0.2019239147313784
pose_default.pose.position.y = 0.36016053406944526
pose_default.pose.position.z = 0.5647490714802889
pose_default.pose.orientation.x = 0.615540147210404
pose_default.pose.orientation.y = -0.3569674764038792
pose_default.pose.orientation.z = -0.6151713797376471
pose_default.pose.orientation.w = -0.3394830209505622

# Arm up:
pose_up = PoseStamped()
pose_up.pose.position.x = -0.0164443639556814
pose_up.pose.position.y = 0.19123849668391044
pose_up.pose.position.z = 1.00
pose_up.pose.orientation.x = 0.7018344846564541
pose_up.pose.orientation.y = 0.7122998537692892
pose_up.pose.orientation.z = 0.005386603930283156
pose_up.pose.orientation.w = 0.0053159162293153995

# Pose01:
pose_01 = PoseStamped()
pose_01.pose.position.x = 0.3320153763417186
pose_01.pose.position.y = 0.20802650428714126
pose_01.pose.position.z = 0.6431376949278869
pose_01.pose.orientation.x = -0.005796355932569323
pose_01.pose.orientation.y = 0.8000808997663621
pose_01.pose.orientation.z = 0.5947827268721892
pose_01.pose.orientation.w = -0.07791318182075829

# Pose02:
pose_02 = PoseStamped()
pose_02.pose.position.x = 0.16396263951712656
pose_02.pose.position.y = 0.19097405811385304
pose_02.pose.position.z = 0.6593009918338222
pose_02.pose.orientation.x = -0.0073194590015851005
pose_02.pose.orientation.y = 0.7998363082094895
pose_02.pose.orientation.z = 0.5953732593201085
pose_02.pose.orientation.w = -0.07575610652805236

# PoseFFTip (default)
pose_FFTip = PoseStamped()
pose_FFTip.pose.position.x = 0.21288835980376392
pose_FFTip.pose.position.y = 0.38965303238597004
pose_FFTip.pose.position.z = 0.18223539124202698
pose_FFTip.pose.orientation.x = 0.9026200941501608
pose_FFTip.pose.orientation.y = 0.08244953102591612
pose_FFTip.pose.orientation.z = -0.19145898018253993
pose_FFTip.pose.orientation.w = -0.376593281110302

# PoseFFTip02 (default)
pose_FFTip_02 = PoseStamped()
pose_FFTip_02.pose.position.x = 0.8534402081891853
pose_FFTip_02.pose.position.y = 0.41858155564459504
pose_FFTip_02.pose.position.z = 0.41914410905796307
pose_FFTip_02.pose.orientation.x = -0.5380279755738201
pose_FFTip_02.pose.orientation.y = -0.467008903941789
pose_FFTip_02.pose.orientation.z = -0.25186964677075774
pose_FFTip_02.pose.orientation.w = 0.654973482039235


# Function to get initial clipping pose based on the clip hole pose
def get_start_pose(final_pose: PoseStamped, dist):
    """
        Computes the initial clipping pose for a given clip hole pose. It returns a PoseStamped.
        @param final_pose - A given pose of type PoseStamped.
        @param dist - distance in meters to pre-position the UR5
    """
    global tf_listener, tf_broadcaster

    # UR5 in FFTip frame Pose
    tf_listener.waitForTransform('rh_fftip', 'ra_flange', rospy.Time(), rospy.Duration(1.0))
    (fftip_arm_translation, fftip_arm_rotation) = tf_listener.lookupTransform('rh_fftip', 'ra_flange', rospy.Time())

    # Create fictitious FFTip frame
    tf_broadcaster.sendTransform((final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z),
                                 (final_pose.pose.orientation.x, final_pose.pose.orientation.y, final_pose.pose.orientation.z, final_pose.pose.orientation.w),
                                 rospy.Time.now(),
                                 'fictitious_fftip_frame',
                                 'ra_base_link')
    
    # Calculate UR5 position from FFTip position
    pose_in_fftip = PoseStamped()
    #pose_in_fftip.header.stamp = rospy.Time.now()
    pose_in_fftip.header.frame_id = 'fictitious_fftip_frame'
    pose_in_fftip.pose.position.x = fftip_arm_translation[0]
    pose_in_fftip.pose.position.y = fftip_arm_translation[1]
    pose_in_fftip.pose.position.z = fftip_arm_translation[2] - dist
    pose_in_fftip.pose.orientation.x = fftip_arm_rotation[0]
    pose_in_fftip.pose.orientation.y = fftip_arm_rotation[1]
    pose_in_fftip.pose.orientation.z = fftip_arm_rotation[2]
    pose_in_fftip.pose.orientation.w = fftip_arm_rotation[3]

    # Get position in 'ra_base_link' frame
    tf_listener.waitForTransform('ra_base_link', pose_in_fftip.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
    new_pose = tf_listener.transformPose('ra_base_link', pose_in_fftip)

    tf_broadcaster.sendTransform((new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z),
                                 (new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z, new_pose.pose.orientation.w),
                                 rospy.Time.now(),
                                 'fictitious2_fftip_frame',
                                 'ra_base_link')

    return new_pose


if __name__ == "__main__":
    # Shadow Arm commander
    arm_commander = SrArmCommander(name='right_arm')

    # Set control velocity and acceleration
    arm_commander.set_max_velocity_scaling_factor(0.1)
    arm_commander.set_max_acceleration_scaling_factor(0.1)

    print('\n' + colored('"ur5_tests" ROS node is ready!', 'green') + '\n')  

    while not rospy.is_shutdown():
        # Calculate Inverse Kinematics
        new_pose = get_start_pose(pose_FFTip_02, 0.00)
        print(new_pose.pose.orientation)
        target_joints = arm_commander.get_ik(new_pose, avoid_collisions=True)

        # Move to target joints
        arm_commander.move_to_joint_value_target(target_joints)

        print('Pose 1 DONE: Sleeping for 2 sec...')
        rospy.sleep(2.0)

        # Calculate Inverse Kinematics
        new_pose2 = get_start_pose(pose_FFTip_02, 0.10)
        print(new_pose2.pose.orientation)
        target_joints2 = arm_commander.get_ik(new_pose2, avoid_collisions=True)
    
        # Move to target joints
        arm_commander.move_to_joint_value_target(target_joints2)

        print('Pose 2 DONE: Sleeping for 2 sec...')
        rospy.sleep(2.0)

    rospy.spin()