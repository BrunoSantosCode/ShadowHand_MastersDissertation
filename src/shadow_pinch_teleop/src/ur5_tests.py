#!/usr/bin/env python3

#* * * * * * * * * * * ur5_tests.py  * * * * * * * * * * *#
#*  Performs Wiring Fitting task                         *#
#*  Grabs the clip from stand                            *#
#*  Inserts the clip in the respective hole              *#
#*  Note: clip stand and hole poses in 'ra_base_link'    *#
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * *#

import tf
import rospy
import numpy as np
from termcolor import colored
from geometry_msgs.msg import PoseStamped
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander


# CLIP Poses

# Grab clip
clip_stand_01 = PoseStamped()
clip_stand_01.pose.position.x = 0.6283771394157185
clip_stand_01.pose.position.y = 0.2069146791188278
clip_stand_01.pose.position.z = 0.09206483652527714
clip_stand_01.pose.orientation.x = 0.9890882607771632
clip_stand_01.pose.orientation.y = 0.09368992622008868
clip_stand_01.pose.orientation.z = -0.06156698328073961
clip_stand_01.pose.orientation.w = -0.09558303556274649

# Insert clip
clip_hole_01 = PoseStamped()
clip_hole_01.pose.position.x = 0.8870281551963635
clip_hole_01.pose.position.y = 0.13932891868554642
clip_hole_01.pose.position.z = 0.3079324033128754
clip_hole_01.pose.orientation.x = 0.7393360881337623
clip_hole_01.pose.orientation.y = 0.0212294594171611
clip_hole_01.pose.orientation.z = 0.6320528796610841
clip_hole_01.pose.orientation.w = -0.23117226509261704

# HAND Poses

# Preparing to pinch clip
start_pinch_pose = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.35415, 
                    'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0, 
                    'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0, 
                    'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0, 
                    'rh_THJ1': -0.40, 'rh_THJ2': -0.10, 'rh_THJ3': 0.0, 'rh_THJ4': 1.21407, 'rh_THJ5': 0.44347, 
                    'rh_WRJ1': -0.698, 'rh_WRJ2': 0.0}

# Pinch clip
pinch_pose = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.5, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.35415, 
              'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0, 
              'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0, 
              'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0, 
              'rh_THJ1': -0.32294, 'rh_THJ2': -0.14608, 'rh_THJ3': 0.15715, 'rh_THJ4': 1.10, 'rh_THJ5': 0.65, 
              'rh_WRJ1': -0.698, 'rh_WRJ2': 0.0}


def get_start_pose(final_pose: PoseStamped, dist):
    """
        Computes the initial clipping pose for a given clip hole pose. It returns a PoseStamped.
        @param final_pose - A given pose of type PoseStamped.
        @param dist - distance in meters to pre-position the UR5
    """
    global tf_listener, tf_broadcaster

    # UR5 in FFTip frame Pose
    #tf_listener.waitForTransform('rh_fftip', 'ra_flange', rospy.Time(), rospy.Duration(1.0))
    #(fftip_arm_translation, fftip_arm_rotation) = tf_listener.lookupTransform('rh_fftip', 'ra_flange', rospy.Time())

    # Create fictitious Clip frame
    tf_broadcaster.sendTransform((final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z),
                                 (final_pose.pose.orientation.x, final_pose.pose.orientation.y, final_pose.pose.orientation.z, final_pose.pose.orientation.w),
                                 rospy.Time.now(),
                                 'fictitious_clip_frame',
                                 'ra_base_link')
    
    # Calculate UR5 position from Clip position
    pose_in_clip_frame = PoseStamped()
    #pose_in_clip_frame.header.stamp = rospy.Time.now()
    pose_in_clip_frame.header.frame_id = 'fictitious_clip_frame'
    pose_in_clip_frame.pose.position.x = -0.14212348114334017
    pose_in_clip_frame.pose.position.y = -0.24415732893174702
    pose_in_clip_frame.pose.position.z = -0.2613158276162346 - dist
    pose_in_clip_frame.pose.orientation.x = 0.20611992445119792
    pose_in_clip_frame.pose.orientation.y = -0.2702029129684741
    pose_in_clip_frame.pose.orientation.z = 0.5082802656970709
    pose_in_clip_frame.pose.orientation.w = 0.7913002805954882

    # TO DO: Adjust the clip adapter position in relation to the Clip
    # TO DO: Adjust the clip hole position in relation to the Clip

    # Get position in 'ra_base_link' frame
    tf_listener.waitForTransform('ra_base_link', pose_in_clip_frame.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
    new_pose = tf_listener.transformPose('ra_base_link', pose_in_clip_frame)

    tf_broadcaster.sendTransform((new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z),
                                 (new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z, new_pose.pose.orientation.w),
                                 rospy.Time.now(),
                                 'fictitious_ur5_frame',
                                 'ra_base_link')

    return new_pose


def set_hand_pose(pose: str):
    """
        Sets a pre-defined pose for Shadow Hand
        @param pose - A given pose 'grab' or 'release'.
    """
    if pose == 'grab':
        hand_commander.move_to_joint_value_target_unsafe(joint_states=pinch_pose, time=1.0, wait=True, angle_degrees=False)
    elif pose == 'release':
        hand_commander.move_to_joint_value_target_unsafe(joint_states=start_pinch_pose, time=1.0, wait=True, angle_degrees=False)
    else:
        print('\n' + colored('ERROR: "' + pose + '" hand pose is not defined!', 'red') + '\n') 


def move_arm_to(final_pose: PoseStamped, dist):
    """
        Moves UR5 to the specified pose
        @param final_pose - A given pose of type PoseStamped.
        @param dist - distance in meters to pre-position the UR5
    """
    global arm_commander

    arm_go_to = get_start_pose(final_pose, dist)
    #print(arm_go_to.pose.position)
    #print(arm_go_to.pose.orientation)
    method = 3
    if method == 1:
        target_joints = arm_commander.get_ik(arm_go_to, avoid_collisions=True)
        if target_joints == None:
            exit(-1)
        arm_commander.move_to_joint_value_target(target_joints)
    elif method == 2:
        arm_commander.plan_to_pose_target(arm_go_to, 'ra_flange')
        arm_commander.execute()
    elif method == 3:
        arm_commander.plan_to_pose_target(arm_go_to, 'ra_flange', alternative_method=True)
        arm_commander.execute()
    elif method == 4:
        arm_commander.move_to_pose_target(arm_go_to)
    

if __name__ == "__main__":
    global tf_listener, tf_broadcaster
    global arm_commander

    # Init ROS
    rospy.init_node('ur5_tests')

    # Init TF Global Vars
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()

    # Shadow Arm/Hand commander
    arm_commander = SrArmCommander(name='right_arm')
    hand_commander = SrHandCommander(name='right_hand')

    # Set control velocity and acceleration
    hand_commander.set_max_velocity_scaling_factor(0.5)
    hand_commander.set_max_acceleration_scaling_factor(0.5)
    arm_commander.set_max_velocity_scaling_factor(0.5)
    arm_commander.set_max_acceleration_scaling_factor(0.5)

    print('\n' + colored('"ur5_tests" ROS node is ready!', 'green') + '\n') 

    hand_commander.move_to_joint_value_target_unsafe(joint_states=start_pinch_pose, time=1.0, wait=True, angle_degrees=False) 

    while not rospy.is_shutdown():

        #1 GET THE CLIP

        move_arm_to(clip_stand_01, 0.05)
        move_arm_to(clip_stand_01, 0.05)
        print('Preparing to grab the clip')
        if rospy.is_shutdown(): break
        rospy.sleep(1.0)

        move_arm_to(clip_stand_01, 0.0)
        move_arm_to(clip_stand_01, 0.0)
        set_hand_pose('grab')
        print('Clip grabed')
        if rospy.is_shutdown(): break
        rospy.sleep(1.0)

        move_arm_to(clip_stand_01, 0.05)
        move_arm_to(clip_stand_01, 0.05)
        print('Moved back')
        if rospy.is_shutdown(): break
        rospy.sleep(1.0)

        #2 CLIPPING TASK

        move_arm_to(clip_hole_01, 0.07)
        move_arm_to(clip_hole_01, 0.07)
        print('Preparing to clip')
        if rospy.is_shutdown(): break
        rospy.sleep(1.0)

        move_arm_to(clip_hole_01, 0.0)
        move_arm_to(clip_hole_01, 0.0)
        print('Clip in place')
        if rospy.is_shutdown(): break
        rospy.sleep(1.0)

        set_hand_pose('release')
        move_arm_to(clip_hole_01, 0.07)
        move_arm_to(clip_hole_01, 0.07)
        print('Iteration completed!')
        if rospy.is_shutdown(): break
        rospy.sleep(1.0)

    rospy.spin()