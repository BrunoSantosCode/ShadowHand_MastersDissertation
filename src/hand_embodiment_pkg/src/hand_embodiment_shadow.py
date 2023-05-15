#!/usr/bin/env python3

import sys
sys.path.append('/home/user/projects/shadow_robot/base/src/hand_embodiment_pkg')

#* * * * * * * * * * hand_embodiment_shadow.py * * * * * * * * * * *#
#*  Receives HandKeypoints.msg from "hand_kp" topic                *#
#*  Uses hand_ambodiment to calculate inverse kinematics (thread1) *#
#*  Send joint angles to Shadow Hand (thread2)                     *#
#*  + Adaptable median filter for keypoint positions               *#
#*  + hand_embodiment solve only if different angles keypoints     *#
#*  + Execute only if different angles                             *#
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *#

from math import pi as PI
import numpy as np
import rospy
import rospkg
from threading import Thread, Lock
from termcolor import colored
from sr_robot_commander.sr_hand_commander import SrHandCommander
from hand_embodiment_pkg.msg import HandKeypoints
from hand_embodiment.pipelines import MoCapToRobot

PACKAGE_PATH = rospkg.RosPack().get_path('hand_embodiment_pkg')
MANO_CONFIG_PATH = PACKAGE_PATH + "/examples/config/mano/20210610_april.yaml"
FINGERS = ["thumb", "index", "middle", "ring", "little"]

# Keypoints 3D position median filter
AVERAGE_N = 5
keypoints = np.zeros((AVERAGE_N, 21*3))
prev_keypoints = np.zeros(21*3)

# Gobal var to save calculated joints
joints = np.zeros(24)
prev_joints = np.zeros(24)

# Mutex
mutex_kp = Lock()
mutex_joints = Lock()

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
def hand_embodiment_solver():
    while not rospy.is_shutdown():
        global mutex_kp, keypoints, prev_keypoints, joints

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

        # Get relevant human hand positions
        aux0 = np.array([median_keypoints[0*3+0], median_keypoints[0*3+1], median_keypoints[0*3+2]])
        aux9 = np.array([median_keypoints[9*3+0], median_keypoints[9*3+1], median_keypoints[9*3+2]])
        aux13 = np.array([median_keypoints[13*3+0], median_keypoints[13*3+1], median_keypoints[13*3+2]])
        # Rotation matrix
        v09 = np.subtract(aux9, aux0)
        v013 = np.subtract(aux13, aux0)
        z = (v09+v013)/2
        z = z / np.linalg.norm(z)
        y = np.cross(v09, v013)
        y = y / np.linalg.norm(y)
        x = np.cross(y, z)
        x = x / np.linalg.norm(x)

        aux1 = aux0 - 0.02*x
        aux2 = aux0 + 0.02*z

        # Define the positions of the hand markers (in millimeters)
        hand_markers = [np.array(aux0),   # Hand top marker
                        np.array(aux1),   # Hand left marker
                        np.array(aux2),   # Hand right marker
        ]

        # Define the positions of the finger markers (in millimeters)
        finger_markers = {
                          "thumb" : np.array(median_keypoints[12:15]),   # Keypoint 4  * 3 = 12
                          "index" : np.array(median_keypoints[24:27]),   # Keypoint 8  * 3 = 24  
                          "middle": np.array(median_keypoints[36:39]),   # Keypoint 12 * 3 = 36
                          "ring"  : np.array(median_keypoints[48:51]),   # Keypoint 16 * 3 = 48
                          "little": np.array(median_keypoints[60:63])}   # Keypoint 20 * 3 = 60
                         
        # Estimate the joint angles of the Shadow hand from the marker positions
        _, joint_angles = shadow_hand.estimate(hand_markers, finger_markers)

        # DEBUG
        if False:
            # Print the joint angles
            print("Joint angles:")
            print(joint_angles)

        # Get joints from hand_embodiment
        this_joints = np.zeros(24)
        th = joint_angles['thumb']  
        ff = joint_angles['index']  
        mf = joint_angles['middle'] 
        rf = joint_angles['ring']    
        lf = joint_angles['little']           
        this_joints[0:4] = [max(0, ff[2]-PI/2), min(PI/2, ff[2]), ff[1], ff[0]]   
        this_joints[4:9] = [max(0, lf[2]-PI/2), min(PI/2, lf[3]), lf[2], lf[1], lf[0]]           
        this_joints[9:13] = [max(0, mf[2]-PI/2), min(PI/2, mf[2]), mf[1], mf[0]]
        this_joints[13:18] = [th[4], th[3], th[2], th[1], th[0]]

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
    rospy.init_node('hand_embodiment_shadow')

    # Shadow Hand commander
    hand_commander = SrHandCommander(name='right_hand')

    # Set control velocity and acceleration
    hand_commander.set_max_velocity_scaling_factor(0.01)
    hand_commander.set_max_acceleration_scaling_factor(0.5)

    # Create a MoCapToRobot object for the Shadow hand
    shadow_hand = MoCapToRobot(
        hand = "shadow",
        mano_config = MANO_CONFIG_PATH,
        use_fingers = FINGERS,
        measure_time = True
    )

    print('\n' + colored('"hand_embodiment_shadow" ROS node is ready!', 'green') + '\n')  

    thread1 = Thread(target=hand_embodiment_solver)
    thread1.start()

    thread2 = Thread(target=send_shadow_commands)
    thread2.start()

    openPose_sub()