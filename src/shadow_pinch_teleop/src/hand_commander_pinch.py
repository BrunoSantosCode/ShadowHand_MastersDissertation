#!/usr/bin/env python3

#* * * * * * * * * * * hand_commander_pinch.py * * * * * * * * * * *#
#*  Receives HandKeypoints.msg from "hand_kp" topic                *#
#*  Adaptable median filter for keypoint positions                 *#
#*  Calculte distance between thumb and forefinger tips            *#
#*  Update state machine and send Shadow Hand command              *#
#*  Execute only if new keypoints positions                        *#
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *#

import rospy
import numpy as np
import tkinter as tk
from termcolor import colored
from threading import Thread, Lock
from hand_embodiment_pkg.msg import HandKeypoints
from sr_robot_commander.sr_hand_commander import SrHandCommander

# State Machine VARS
state = 0   # STATES: [0] Open Hand (initial state)
            #         [1] Start Pinch
            #         [2] Pinch
dist = 10   # Distance between thumb and forefinger tips [cm]
TH1 = 5.5   # Threshold for start pinch pose [cm]
TH2 = 3.5   # Threshold for pinch pose [cm]
HYST = 0.5  # Hysteresis [cm]       

# GUI 
window = tk.Tk()
window.title('Shadow Hand Commander (pinch)')
window.geometry("500x400")
state_lbl = tk.Label(window, text='STATE', font=("Arial", 27, 'bold'))
state_lbl.pack()
state_txt = tk.StringVar()
state_txt.set('[0] Open Hand\n')
state_label = tk.Label(window, text=str(state_txt.get()), font=("Arial", 25))
state_label.pack()
dist_lbl = tk.Label(window, text='DISTANCE', font=("Arial", 27, 'bold'))
dist_lbl.pack()
dist_txt = tk.StringVar()
dist_txt.set(round(dist,2))
dist_label = tk.Label(window, text=str(dist_txt.get()) + 'cm\n', font=("Arial", 25))
dist_label.pack()
info_label = tk.Label(window, text='TH1: '+str(TH1)+'     TH2: '+str(TH2)+'     HYST: '+str(HYST), font=("Arial", 15, 'bold'))
info_label.pack()

state_str = '[0] Open Hand\n'

# Hand Poses
open_pose = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.0,
             'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
             'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
             'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
             'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0, 'rh_THJ4': 0.0, 'rh_THJ5': 0.0,
             'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}

start_pinch_pose = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.35415, 
                    'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0, 
                    'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0, 
                    'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0, 
                    'rh_THJ1': -0.24833, 'rh_THJ2': 0.05104, 'rh_THJ3': 0.0, 'rh_THJ4': 1.21407, 'rh_THJ5': 0.44347, 
                    'rh_WRJ1': -0.698, 'rh_WRJ2': 0.0}

pinch_pose = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.5, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.35415, 
              'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0, 
              'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0, 
              'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0, 
              'rh_THJ1': -0.32294, 'rh_THJ2': -0.14608, 'rh_THJ3': 0.15715, 'rh_THJ4': 1.10, 'rh_THJ5': 0.65, 
              'rh_WRJ1': -0.698, 'rh_WRJ2': 0.0}


# Pinch Pose 01
# start_pinch_pose = {'rh_FFJ1': 0.02488300632883622, 'rh_FFJ2': 0.0, 'rh_FFJ3': 1.566404705503622, 'rh_FFJ4': 0.3513813328956592, 
#                     'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0, 
#                     'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0, 
#                     'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0, 
#                     'rh_THJ1': 0.5866214473810422, 'rh_THJ2': 0.706871554453986, 'rh_THJ3': 0.2114043697106358, 'rh_THJ4': 1.082787266966136, 'rh_THJ5': -0.26724209920157144, 
#                     'rh_WRJ1': -0.698, 'rh_WRJ2': 0.0}
# pinch_pose = {'rh_FFJ1': 0.0, 'rh_FFJ2': 1.5707, 'rh_FFJ3': 1.566404705503622, 'rh_FFJ4': 0.3513813328956592, 
#               'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0, 
#               'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0, 
#               'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0, 
#               'rh_THJ1': 0.5866214473810422, 'rh_THJ2': 0.706871554453986, 'rh_THJ3': 0.2114043697106358, 'rh_THJ4': 1.082787266966136, 'rh_THJ5': -0.26724209920157144, 
#               'rh_WRJ1': -0.698, 'rh_WRJ2': 0.0}


# Keypoints 3D position median filter
AVERAGE_N = 5
keypoints = np.zeros((AVERAGE_N, 2*3))  # save only thumb and forefinger tips
prev_keypoints = np.zeros(2*3)

# Mutex
mutex_kp = Lock()

def update_gui():
    global state_str, dist
    global state_txt, dist_txt
    global state_label, dist_label
    state_txt.set(state_str)
    dist_txt.set(round(dist,2))
    state_label.config(text=str(state_txt.get()))
    dist_label.config(text=str(dist_txt.get()) + 'cm\n')
    window.after(100, update_gui)

# Thread that send commands to Shadow Hand
def send_shadow_commands():
    while not rospy.is_shutdown():
        global keypoints, prev_keypoints, mutex_kp
        global state, dist, state_str
        global open_pose, start_pinch_pose, pinch_pose

        # Get median keypoints position [Median Filter]
        median_keypoints = np.zeros(2*3)
        mutex_kp.acquire(blocking=True)
        th_x = np.median(keypoints[:,0])
        th_y = np.median(keypoints[:,1])
        th_z = np.median(keypoints[:,2])
        ff_x = np.median(keypoints[:,3])
        ff_y = np.median(keypoints[:,4])
        ff_z = np.median(keypoints[:,5])
        mutex_kp.release()

        # Check if new keypoints
        if np.array_equal([th_x, th_y, th_z, ff_x, ff_y, ff_z], prev_keypoints):
            rospy.sleep(0.05)
            continue
        else:
            prev_keypoints = [th_x, th_y, th_z, ff_x, ff_y, ff_z]

        # DEBUG
        if False:
            print('Median:')
            print('Thumb tip:', median_keypoints[0], median_keypoints[1], median_keypoints[2])
            print('Forefinger tip:', median_keypoints[3], median_keypoints[4], median_keypoints[5])

        # Calculate distance between thumb and forefinger tips
        dist = np.sqrt( (th_x-ff_x)**2 + (th_y-ff_y)**2 +(th_z-ff_z)**2 ) * 100  # [m -> cm]
        
        # Check invalid detection of keypoints
        if (dist == 0) or (dist > 20):
            continue

        # DEBUG
        if False:
            print('Distance:', round(dist, 2), 'cm')
        
        # Update state
        if (state == 0) and (dist < (TH1-HYST)):
            state = 1
            state_str = '[1] Start Pinch\n'
            hand_commander.move_to_joint_value_target_unsafe(joint_states=start_pinch_pose, time=0.3, wait=False, angle_degrees=False)
            print('\n' + colored('STATE=1', 'yellow') + '\n')
        elif (state == 1) and (dist > (TH1+HYST)):
            state = 0
            state_str = '[0] Open Hand\n'
            hand_commander.move_to_joint_value_target_unsafe(joint_states=open_pose, time=0.3, wait=False, angle_degrees=False)
            print('\n' + colored('STATE=0', 'green') + '\n')
        elif (state == 1) and (dist < (TH2-HYST)):
            state = 2
            state_str = '[2] Pinch\n'
            hand_commander.move_to_joint_value_target_unsafe(joint_states=pinch_pose, time=0.3, wait=False, angle_degrees=False)
            print('\n' + colored('STATE=2', 'blue') + '\n')
        elif (state == 2) and (dist > (TH2+HYST)):
            state = 1
            state_str = '[1] Start Pinch\n'
            hand_commander.move_to_joint_value_target_unsafe(joint_states=start_pinch_pose, time=0.3, wait=False, angle_degrees=False)
            print('\n' + colored('STATE=1', 'yellow') + '\n')
        
        # Update GUI
        update_gui()

# OpenPose callback
def openPose_CB(msg):
    # DEBUG
    if False:
        print('Data', msg.header.seq, 'received...')

    global keypoints

    # Extract keypoint from msg (Thumb tip + Forefinger tip)
    this_keypoints = np.zeros(2*3)
    this_keypoints[0] = msg.keypoints[4].x
    this_keypoints[1] = msg.keypoints[4].y
    this_keypoints[2] = msg.keypoints[4].z
    this_keypoints[3] = msg.keypoints[8].x
    this_keypoints[4] = msg.keypoints[8].y
    this_keypoints[5] = msg.keypoints[8].z

    # Add keypoints to median filter
    mutex_kp.acquire(blocking=True)
    keypoints = np.roll(keypoints, 1, axis=0)
    keypoints[0] = this_keypoints
    mutex_kp.release()

    # DEBUG
    if False:
        for i in range(0, len(keypoints)):
            print(keypoints[i][0], keypoints[i][30], keypoints[i][-1])
    
    

if __name__ == "__main__":
    rospy.init_node('hand_commander_pinch')

    # Create OpenPose subscriber
    rospy.Subscriber('hand_kp', HandKeypoints, openPose_CB, queue_size=1)

    # Shadow Hand commander
    hand_commander = SrHandCommander(name='right_hand')

    # Set control velocity and acceleration
    hand_commander.set_max_velocity_scaling_factor(0.01)
    hand_commander.set_max_acceleration_scaling_factor(0.5)

    hand_commander.move_to_joint_value_target_unsafe(joint_states=open_pose, time=1.0, wait=True, angle_degrees=False)
    
    # Start GUI
    window.mainloop()

    # Start thread to send Shadow commands
    shadow_thread = Thread(target=send_shadow_commands)
    shadow_thread.start()

    print('\n' + colored('"hand_commander_pinch" ROS node is ready!', 'green') + '\n')  

    rospy.spin()