#!/usr/bin/env python3

#* * * * * * * * * * hand_commander_Joints_vF  * * * * * * * * * * *#
#*  Receives HandKeypoints.msg from "hand_kp" topic                *#
#*  Calculate angles from finger segments and send to SH (thread1) *#
#*  + Adaptable median filter for keypoint positions               *#
#*  + Execute only if new keypoints positions                      *#
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *#

import rospy
import numpy as np
from termcolor import colored
from threading import Thread, Lock
from geometry_msgs.msg import Point
from hand_embodiment_pkg.msg import HandKeypoints
from sr_robot_commander.sr_hand_commander import SrHandCommander

# Keypoints 3D position median filter
AVERAGE_N = 5
keypoints = np.zeros((AVERAGE_N, 21*3))
prev_keypoints = np.zeros(21*3)

# Mutex
mutex_kp = Lock()

#Define joints bounds
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

# Funtion to calculate joint angles from 3 3D points
def get_angle(a, b, c):
    ab = [b.x - a.x, b.y - a.y, b.z - a.z]
    bc = [c.x - b.x, c.y - b.y, c.z - b.z]
    dot = ab[0]*bc[0] + ab[1]*bc[1] + ab[2]*bc[2]
    mag_ab = np.sqrt(ab[0]**2 + ab[1]**2 + ab[2]**2)
    mag_bc = np.sqrt(bc[0]**2 + bc[1]**2 + bc[2]**2)
    if (mag_ab==0) or (mag_bc==0):
        return 0
    angle_rad = np.arccos(dot/(mag_ab*mag_bc))
    return angle_rad

def get_angle_knuckles(a, b, c, d):
    ab = [b.x - a.x, b.y - a.y, b.z - a.z]
    cd = [d.x - c.x, d.y - c.y, d.z - c.z]
    dot = ab[0]*cd[0] + ab[1]*cd[1] + ab[2]*cd[2]
    mag_ab = np.sqrt(ab[0]**2 + ab[1]**2 + ab[2]**2)
    mag_cd = np.sqrt(cd[0]**2 + cd[1]**2 + cd[2]**2)
    if (mag_ab==0) or (mag_cd==0):
        return 0
    angle_rad = np.arccos(dot/(mag_ab*mag_cd))
    return angle_rad

# Thread that send commands to Shadow Hand
def send_shadow_commands():
    while not rospy.is_shutdown():
        global mutex_kp, keypoints, prev_keypoints

        # Get median keypoints position [Median Filter]
        median_keypoints = np.zeros(21*3)
        mutex_kp.acquire(blocking=True)
        for i in range(0, len(median_keypoints)):
            median_keypoints[i] = np.median(keypoints[:,i])
        mutex_kp.release()

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

        # Convert keypoints to geometry_msgs.Point
        kp_point = []
        for i in range(21):
            kp_point.append(Point(median_keypoints[i*3+0], median_keypoints[i*3+1], median_keypoints[i*3+2]))
        
        # Convert to Shadow Hand
        this_joints = np.zeros(24)
        # Fore Finger (index finger)
        this_joints[0] = get_angle(kp_point[6], kp_point[7], kp_point[8])
        this_joints[1] = get_angle(kp_point[5], kp_point[6], kp_point[7])
        this_joints[2] = get_angle_knuckles(kp_point[0], kp_point[9], kp_point[5], kp_point[6])
        # Little Finger
        this_joints[4] = get_angle(kp_point[18], kp_point[19], kp_point[20])
        this_joints[5] = get_angle(kp_point[17], kp_point[18], kp_point[19])
        this_joints[6] = get_angle_knuckles(kp_point[0], kp_point[9], kp_point[17], kp_point[18])
        # Middle Finger
        this_joints[9] = get_angle(kp_point[10], kp_point[11], kp_point[12])
        this_joints[10] = get_angle(kp_point[9], kp_point[10], kp_point[11])
        this_joints[11] = get_angle_knuckles(kp_point[0],  kp_point[9], kp_point[9], kp_point[10])
        # Ring Finger
        this_joints[13] = get_angle(kp_point[14], kp_point[15], kp_point[16])
        this_joints[14] = get_angle(kp_point[13], kp_point[14], kp_point[15])
        this_joints[15] = get_angle_knuckles(kp_point[0],  kp_point[9], kp_point[13], kp_point[14])  
        # Thumb
        this_joints[17] = get_angle(kp_point[2], kp_point[3], kp_point[4])
        this_joints[19] = get_angle(kp_point[1], kp_point[2], kp_point[3])  
        this_joints[20] = get_angle(kp_point[0], kp_point[1], kp_point[2])  
        
        # DEBUG
        if False:
            # Print the joint angles
            print("Joint angles:")
            print(this_joints)
        
        # Check joints limits
        for i in range(0,len(this_joints)):
            if this_joints[i] > joints_max_rad[i]:
                this_joints[i] = joints_max_rad[i]
            elif this_joints[i] < joints_min_rad[i]:
                this_joints[i] = joints_min_rad[i]

        # DEBUG
        if False:
            print('Joints command:')
            print(prev_joints)

        # Joints for Shadow Hand
        hand_pos = hand_commander.get_joints_position()

        hand_pos.update({'rh_FFJ1': this_joints[0], 'rh_FFJ2': this_joints[1], 'rh_FFJ3': this_joints[2], 'rh_FFJ4': this_joints[3]})
        hand_pos.update({'rh_LFJ1': this_joints[4], 'rh_LFJ2': this_joints[5], 'rh_LFJ3': this_joints[6], 'rh_LFJ4': this_joints[7], 'rh_LFJ5': this_joints[8]})
        hand_pos.update({'rh_MFJ1': this_joints[9], 'rh_MFJ2': this_joints[10], 'rh_MFJ3': this_joints[11], 'rh_MFJ4': this_joints[12]})
        hand_pos.update({'rh_RFJ1': this_joints[13], 'rh_RFJ2': this_joints[14], 'rh_RFJ3': this_joints[15], 'rh_RFJ4': this_joints[16]})
        hand_pos.update({'rh_THJ1': this_joints[17], 'rh_THJ2': this_joints[18], 'rh_THJ3': this_joints[19], 'rh_THJ4': this_joints[20], 'rh_THJ5': this_joints[21]})
        hand_pos.update({'rh_WRJ1': this_joints[22], 'rh_WRJ2': this_joints[23]})

        hand_commander.move_to_joint_value_target_unsafe(joint_states=hand_pos, time=0.3, wait=True, angle_degrees=False)

        print('\n' + colored('Command sent to Shadow Hand!', 'green') + '\n') 


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
    rospy.init_node('hand_commander_Joints_vF')

    # Shadow Hand commander
    hand_commander = SrHandCommander(name='right_hand')

    # Set control velocity and acceleration
    hand_commander.set_max_velocity_scaling_factor(0.01)
    hand_commander.set_max_acceleration_scaling_factor(0.5)

    print('\n' + colored('"hand_commander_Joints_vF" ROS node is ready!', 'green') + '\n')  

    thread1 = Thread(target=send_shadow_commands)
    thread1.start()

    openPose_sub()