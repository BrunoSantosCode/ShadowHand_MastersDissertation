#!/usr/bin/env python3

#* * * * * * * * * * hand_commander_DexPilot_v10  * * * * * * * * * *#
#*  Receives HandKeypoints.msg from "hand_kp" topic                 *#
#*  Uses DexPilot to calculate inverse kinematics (thread1)         *#
#*  Send joint angles to Shadow Hand (thread2)                      *#
#*  Adaptable median filter for keypoint positions                  *#
#*  DexPilot solve only if different angles keypoints               *#
#*  Execute only if different angles                                *#
#*  Change hand referential to Shadow Hand referential              *#
#*  Fix joint 1 to 0                                                *#
#*  Parameters adjustments                                          *#
#*  Shadow Hand keypoints mapping                                   *#
#*  Plot keypoints to RVIZ                                          *#
#*  Print solver duration [ms]                                      *#
#*  - Dealing with joint cooupling                                  *#
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  *#

from statistics import median
import tf
import rospy
import numpy as np
from math import pi as PI
from termcolor import colored
from threading import Thread, Lock
from dexPilot_v10 import dexPilot_joints
from timeit import default_timer as timer
from openpose_mujoco.msg import HandKeypoints
from sr_robot_commander.sr_hand_commander import SrHandCommander

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

marker_publisher_hand = rospy.Publisher('rviz_hand_keypoints_markers', MarkerArray, queue_size=1)
marker_publisher_shadow = rospy.Publisher('rviz_shadow_keypoints_markers', MarkerArray, queue_size=1)


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
joints_max = [90.0, 90.0, 90.0, 20.0,
              90.0, 90.0, 90.0, 20.0, 45.0,
              90.0, 90.0, 90.0, 20.0,
              90.0, 90.0, 90.0, 20.0,
              90.0, 30.0, 12.0, 70.0, 60.0,
              28.0, 8.0] 
joints_min_rad = np.radians(joints_min)
joints_max_rad = np.radians(joints_max)

# File to save keypoints positions
file1_path = '/home/user/projects/shadow_robot/base/src/openpose_mujoco/src/human_hand_kp_dist_dex_pilot.txt'
file2_path = '/home/user/projects/shadow_robot/base/src/openpose_mujoco/src/shadow_hand_kp_dist_dex_pilot.txt'
file_human = open(file1_path, 'w')
file_shadow = open(file2_path, 'w')

# Auxiliary method for mapping human hand keypoints into Shadow Hand
def aux_mapping(kp1, kp2, distance):
    norm = np.sqrt( np.power(kp2[0]-kp1[0], 2) + np.power(kp2[1]-kp1[1], 2) + np.power(kp2[2]-kp1[2], 2) )
    if norm == 0:
        return kp1
    norm_x = (kp2[0]-kp1[0]) / norm
    norm_y = (kp2[1]-kp1[1]) / norm
    norm_z = (kp2[2]-kp1[2]) / norm
    x = kp1[0] + norm_x*distance
    y = kp1[1] + norm_y*distance
    z = kp1[2] + norm_z*distance
    return [x, y, z]

# Maps human hand keypoints into Shadow Hand
def map_shadow_hand(human_kp):

    shadow_kp = human_kp.copy()
    
    # Thumb
    i = 3
    metacarpal = np.sqrt(np.power(29, 2) + np.power(34, 2))
    shadow_kp[i:i+3] = aux_mapping(shadow_kp[0:3], shadow_kp[i:i+3], metacarpal / 1000)
    shadow_kp[i+3:i+6] = aux_mapping(shadow_kp[i:i+3], shadow_kp[i+3:i+6], 38 / 1000)
    shadow_kp[i+6:i+9] = aux_mapping(shadow_kp[i+3:i+6], shadow_kp[i+6:i+9], 32 / 1000)
    shadow_kp[i+9:i+12] = aux_mapping(shadow_kp[i+6:i+9], shadow_kp[i+9:i+12], 27 / 1000)
    
    # Forefinger
    i = 15
    metacarpal = np.sqrt(np.power(95, 2) + np.power(33, 2))
    shadow_kp[i:i+3] = aux_mapping(shadow_kp[0:3], shadow_kp[i:i+3], metacarpal / 1000)
    shadow_kp[i+3:i+6] = aux_mapping(shadow_kp[i:i+3], shadow_kp[i+3:i+6], 45 / 1000)
    shadow_kp[i+6:i+9] = aux_mapping(shadow_kp[i+3:i+6], shadow_kp[i+6:i+9], 25 / 1000)
    shadow_kp[i+9:i+12] = aux_mapping(shadow_kp[i+6:i+9], shadow_kp[i+9:i+12], 26 / 1000)
    
    # Middlefinger
    i = 27
    metacarpal = np.sqrt(np.power(99, 2) + np.power(11, 2))
    shadow_kp[i:i+3] = aux_mapping(shadow_kp[0:3], shadow_kp[i:i+3], metacarpal / 1000)
    shadow_kp[i+3:i+6] = aux_mapping(shadow_kp[i:i+3], shadow_kp[i+3:i+6], 45 / 1000)
    shadow_kp[i+6:i+9] = aux_mapping(shadow_kp[i+3:i+6], shadow_kp[i+6:i+9], 25 / 1000)
    shadow_kp[i+9:i+12] = aux_mapping(shadow_kp[i+6:i+9], shadow_kp[i+9:i+12], 26 / 1000)
    
    # Ringfinger
    i = 39
    metacarpal = np.sqrt(np.power(95, 2) + np.power(11, 2))
    shadow_kp[i:i+3] = aux_mapping(shadow_kp[0:3], shadow_kp[i:i+3], metacarpal / 1000)
    shadow_kp[i+3:i+6] = aux_mapping(shadow_kp[i:i+3], shadow_kp[i+3:i+6], 45 / 1000)
    shadow_kp[i+6:i+9] = aux_mapping(shadow_kp[i+3:i+6], shadow_kp[i+6:i+9], 25 / 1000)
    shadow_kp[i+9:i+12] = aux_mapping(shadow_kp[i+6:i+9], shadow_kp[i+9:i+12], 26 / 1000)
    
    # Littlefinger
    i = 51
    metacarpal = np.sqrt(np.power(86.6, 2) + np.power(33, 2))
    shadow_kp[i:i+3] = aux_mapping(shadow_kp[0:3], shadow_kp[i:i+3], metacarpal / 1000)
    shadow_kp[i+3:i+6] = aux_mapping(shadow_kp[i:i+3], shadow_kp[i+3:i+6], 45 / 1000)
    shadow_kp[i+6:i+9] = aux_mapping(shadow_kp[i+3:i+6], shadow_kp[i+6:i+9], 25 / 1000)
    shadow_kp[i+9:i+12] = aux_mapping(shadow_kp[i+6:i+9], shadow_kp[i+9:i+12], 26 / 1000)

    return shadow_kp


# Plots hand keypoints in RVIZ
def plot_hand_keypoints(keypoints, shadow):

    marker_array = MarkerArray()

    new_keypoints = np.resize(keypoints, (23,3))

    # Create a marker for each keypoint
    for i, keypoint in enumerate(new_keypoints):
        marker = Marker()
        marker.header.frame_id = "rh_wrist"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "hand_keypoints"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        if shadow:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        marker.pose.position.x = keypoint[0]
        marker.pose.position.y = keypoint[1]
        marker.pose.position.z = keypoint[2]

        marker_array.markers.append(marker)

    # Connect the groups of points
    connections = [[0, 1, 2, 3, 4], [0, 5, 6, 7, 8], [0, 9, 10, 11, 12], [0, 13, 14, 15, 16], [0, 17, 18, 19, 20]]

    # Create markers for the connections
    for connection_idx, connection in enumerate(connections):
        line_marker = Marker()
        line_marker.header.frame_id = "rh_wrist"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "hand_connections"
        line_marker.id = connection_idx
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.004
        line_marker.color.a = 1.0
        if shadow:
            line_marker.color.r = 0.1
            line_marker.color.g = 0.1
            line_marker.color.b = 0.1
        else:
            line_marker.color.r = 0.0
            line_marker.color.g = 0.0
            line_marker.color.b = 1.0

        for point_idx in connection:
            point = Point()
            point.x = new_keypoints[point_idx][0]
            point.y = new_keypoints[point_idx][1]
            point.z = new_keypoints[point_idx][2]
            line_marker.points.append(point)

        marker_array.markers.append(line_marker)

    if shadow:
        marker_publisher_shadow.publish(marker_array)
    else:
        marker_publisher_hand.publish(marker_array)



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

        #plot_hand_keypoints(median_keypoints, False)
        
        #shadow_kp = map_shadow_hand(median_keypoints)
        shadow_kp = median_keypoints

        #plot_hand_keypoints(shadow_kp, True)
                                 
        # Convert to Shadow Hand (DexPilot)
        start = timer() 
        this_joints, _ = dexPilot_joints(shadow_kp)
        end = timer()
        print('DexPilot solver duration: ', round((end-start)*1000, 2), ' ms!')

        # Write keypopints to .txt file
        timestamp_sec = rospy.get_rostime().to_sec()
        human_dist = np.sqrt( (shadow_kp[4*3+0] - shadow_kp[8*3+0])**2 + 
                              (shadow_kp[4*3+1] - shadow_kp[8*3+1])**2 + 
                              (shadow_kp[4*3+2] - shadow_kp[8*3+2])**2)
        file_human.write(str(timestamp_sec) + " " + str(human_dist) + '\n')
        listener.waitForTransform('/rh_thtip', '/rh_fftip', rospy.Time(), rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform('/rh_thtip', '/rh_fftip', rospy.Time(0))
        shadow_dist = tf.transformations.vector_norm(trans)
        file_shadow.write(str(timestamp_sec) + " " + str(shadow_dist) + '\n')

        # DEBUG
        if False:
            # Print the joint angles
            print("Joint angles:")
            print(joint_angles)

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
    rospy.init_node('hand_commander_DexPilot_v10')

    listener = tf.TransformListener()

    # Shadow Hand commander
    hand_commander = SrHandCommander(name='right_hand')

    # Set control velocity and acceleration
    hand_commander.set_max_velocity_scaling_factor(0.01)
    hand_commander.set_max_acceleration_scaling_factor(0.5)

    print('\n' + colored('"hand_commander_DexPilot_v10" ROS node is ready!', 'green') + '\n')  

    thread1 = Thread(target=dex_pilot_solver)
    thread1.start()

    thread2 = Thread(target=send_shadow_commands)
    thread2.start()

    openPose_sub()
