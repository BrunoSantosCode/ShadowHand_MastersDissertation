#!/usr/bin/env python3

#* * * * * * * * * * hand_commander_DexPilot_v6  * * * * * * * * * *#
#*  Receives HandKeypoints.msg from "hand_kp" topic                *#
#*  Uses DexPilot to calculate inverse kinematics (thread1)        *#
#*  Send joint angles to Shadow Hand (thread2)                     *#
#*  Adaptable median filter for keypoint positions                 *#
#*  Hand_embodiment solve only if different angles keypoints       *#
#*  Execute only if different angles                               *#
#*  Change hand referential to Shadow Hand referential             *#
#*  + Fix joint 1 to 0                                             *#
#*  + Parameters adjustments                                       *#
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *#

import tf
import rospy
import numpy as np
from math import pi as PI
from termcolor import colored
from threading import Thread, Lock
from dexPilot_v6 import dexPilot_joints
from hand_embodiment_pkg.msg import HandKeypoints
from sr_robot_commander.sr_hand_commander import SrHandCommander


# Keypoints 3D position median filter
AVERAGE_N = 4
keypoints = np.zeros((AVERAGE_N, 21*3))
prev_keypoints = np.zeros(21*3)

# Gobal var to save calculated joints
joints = np.zeros(24)
prev_joints = np.zeros(24)

# Mutex
mutex_kp = Lock()
mutex_joints = Lock()

# Define joints bounds
joints_min = [0.0, 0.0, -15.0, -20.0,
              0.0, 0.0, -15.0, -20.0, 0.0,
              0.0, 0.0, -15.0, -20.0,
              0.0, 0.0, -15.0, -20.0,
              -15.0, -30.0, -12.0, 0.0, -60.0,
              -40.0, -28.0] 
joints_max = [90.0-90.0, 90.0+90.0, 90.0, 20.0,
              90.0-90.0, 90.0+90.0, 90.0, 20.0, 45.0,
              90.0-90.0, 90.0+90.0, 90.0, 20.0,
              90.0-90.0, 90.0+90.0, 90.0, 20.0,
              90.0, 30.0, 12.0, 70.0, 60.0,
              28.0, 8.0] 
joints_min_rad = np.radians(joints_min)
joints_max_rad = np.radians(joints_max)

# File to save keypoints positions
file1_path = '/home/user/projects/shadow_robot/base/src/openpose_mujoco/src/human_hand_kp_dist_dex_pilot.txt'
file2_path = '/home/user/projects/shadow_robot/base/src/openpose_mujoco/src/shadow_hand_kp_dist_dex_pilot.txt'
file_human = open(file1_path, 'w')
file_shadow = open(file2_path, 'w')

# Thread that send commands to Shadow Hand
def send_shadow_commands():
    while not rospy.is_shutdown():
        global mutex_joints, joints, prev_joints

        # Check if new keypoints
        mutex_joints.acquire(blocking=True)
        if np.array_equal(joints, prev_joints):
            mutex_joints.release()
            rospy.sleep(0.05)
            continue
        else:
            prev_joints = joints
            mutex_joints.release()

        # DEBUG
        if False:
            print('Joints command:')
            print(prev_joints)

        # Joints for Shadow Hand
        hand_pos = hand_commander.get_joints_position()

        hand_pos.update({'rh_FFJ1': prev_joints[0], 'rh_FFJ2': prev_joints[1], 'rh_FFJ3': prev_joints[2], 'rh_FFJ4': prev_joints[3]})
        hand_pos.update({'rh_LFJ1': prev_joints[4], 'rh_LFJ2': prev_joints[5], 'rh_LFJ3': prev_joints[6], 'rh_LFJ4': prev_joints[7], 'rh_LFJ5': prev_joints[8]})
        hand_pos.update({'rh_MFJ1': prev_joints[9], 'rh_MFJ2': prev_joints[10], 'rh_MFJ3': prev_joints[11], 'rh_MFJ4': prev_joints[12]})
        hand_pos.update({'rh_RFJ1': prev_joints[13], 'rh_RFJ2': prev_joints[14], 'rh_RFJ3': prev_joints[15], 'rh_RFJ4': prev_joints[16]})
        hand_pos.update({'rh_THJ1': prev_joints[17], 'rh_THJ2': prev_joints[18], 'rh_THJ3': prev_joints[19], 'rh_THJ4': prev_joints[20], 'rh_THJ5': prev_joints[21]})
        hand_pos.update({'rh_WRJ1': prev_joints[22], 'rh_WRJ2': prev_joints[23]})

        hand_commander.move_to_joint_value_target_unsafe(joint_states=hand_pos, time=0.3, wait=True, angle_degrees=False)

        print('\n' + colored('Command sent to Shadow Hand!', 'green') + '\n') 

# Thread to compute inverse kinematics [Hand Embodiment]
def dex_pilot_solver():
    while not rospy.is_shutdown():
        global mutex_kp, keypoints, prev_keypoints, joints

        # Get median keypoints position [Median Filter]
        median_keypoints = np.zeros(21*3)
        mutex_kp.acquire(blocking=True)
        for i in range(0, len(median_keypoints)):
            median_keypoints[i] = np.median(keypoints[:,i])
        mutex_kp.release()

        # Write keypopints to .txt file
        timestamp_sec = rospy.get_rostime().to_sec()
        human_dist = np.sqrt( (median_keypoints[4*3+0] - median_keypoints[8*3+0])**2 + 
                              (median_keypoints[4*3+1] - median_keypoints[8*3+1])**2 + 
                              (median_keypoints[4*3+2] - median_keypoints[8*3+2])**2)
        file_human.write(str(timestamp_sec) + " " + str(human_dist) + '\n')
        listener.waitForTransform('/rh_thtip', '/rh_fftip', rospy.Time(), rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform('/rh_thtip', '/rh_fftip', rospy.Time(0))
        shadow_dist = tf.transformations.vector_norm(trans)
        file_shadow.write(str(timestamp_sec) + " " + str(shadow_dist) + '\n')

        # Check if new keypoints
        if np.array_equal(median_keypoints, prev_keypoints):
            rospy.sleep(0.05)
            continue
        else:
            prev_keypoints = median_keypoints

        # DEBUG
        if False:
            print('Median:')
            print(median_keypoints[0], median_keypoints[30], median_keypoints[-1])
                                 
         # Convert to Shadow Hand (DexPilot)
        this_joints, _ = dexPilot_joints(median_keypoints)

        # DEBUG
        if False:
            # Print the joint angles
            print("Joint angles:")
            print(joint_angles)

        # Dealing with Shadow Joints coupling
        this_joints[0] = max(0, this_joints[1]-PI/2)
        this_joints[1] = min(PI/2, this_joints[1])
        this_joints[4] = max(0, this_joints[5]-PI/2)
        this_joints[5] = min(PI/2, this_joints[5])
        this_joints[9] = max(0, this_joints[10]-PI/2)
        this_joints[10] = min(PI/2, this_joints[10])
        this_joints[13] = max(0, this_joints[14]-PI/2)
        this_joints[14] = min(PI/2, this_joints[14])

        # Check joints limits
        for i in range(0,len(this_joints)):
            if this_joints[i] > joints_max_rad[i]:
                this_joints[i] = joints_max_rad[i]
            elif this_joints[i] < joints_min_rad[i]:
                this_joints[i] = joints_min_rad[i]

        # Set global joints
        mutex_joints.acquire(blocking=True)
        joints = this_joints
        mutex_joints.release()

        print('\n' + colored('Joint angles calculated!', 'cyan') + '\n') 



# OpenPose callback
def openPose_CB(msg):
    print('Data', msg.header.seq, 'received...')

    global keypoints

    # Extract keypoint from msg
    this_keypoints = np.zeros(21*3)
    for i in range(0, len(msg.keypoints)):
        this_keypoints[i*3+0] = msg.keypoints[i].x
        this_keypoints[i*3+1] = msg.keypoints[i].y
        this_keypoints[i*3+2] = msg.keypoints[i].z

    # Add keypoints to median filter
    mutex_kp.acquire(blocking=True)
    keypoints = np.roll(keypoints, 1, axis=0)
    keypoints[0] = this_keypoints
    mutex_kp.release()

    # DEBUG
    if False:
        for i in range(0, len(keypoints)):
            print(keypoints[i][0], keypoints[i][30], keypoints[i][-1])
    


# Subscribes to hand joint angles from OpenPose
def openPose_sub():
    rospy.Subscriber('hand_kp', HandKeypoints, openPose_CB, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('hand_commander_DexPilot_vF')

    listener = tf.TransformListener()

    # Shadow Hand commander
    hand_commander = SrHandCommander(name='right_hand')

    # Set control velocity and acceleration
    hand_commander.set_max_velocity_scaling_factor(0.01)
    hand_commander.set_max_acceleration_scaling_factor(0.5)

    print('\n' + colored('"hand_commander_DexPilot_v6" ROS node is ready!', 'green') + '\n')  

    thread1 = Thread(target=dex_pilot_solver)
    thread1.start()

    thread2 = Thread(target=send_shadow_commands)
    thread2.start()

    openPose_sub()