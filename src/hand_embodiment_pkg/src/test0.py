#!/usr/bin/env python3

#* * * * * * test0.py * * * * * *#
#*   Tests hand_embodiment      *#
#* * * ** * * * * * * * * * * * *#


import rospy
import rospkg
import numpy as np
from math import pi
from timeit import default_timer as timer
from sr_robot_commander.sr_hand_commander import SrHandCommander


if __name__ == "__main__":
    rospy.init_node('test0')

    hand_commander = SrHandCommander(name='right_hand')

    print("Starting the movement...")
    
    # Return from MoCapToRobot.estimate()
    # "thumb": ["rh_THJ5", "rh_THJ4", "rh_THJ3", "rh_THJ2", "rh_THJ1"],
    # "index": ["rh_FFJ4", "rh_FFJ3", "rh_FFJ0"],
    # "middle": ["rh_MFJ4", "rh_MFJ3", "rh_MFJ0"],
    # "ring": ["rh_RFJ4", "rh_RFJ3", "rh_RFJ0"],
    # "little": ["rh_LFJ5", "rh_LFJ4", "rh_LFJ3", "rh_LFJ0"],
    
    from_hand_emb = {'thumb': np.array([-0.64906265,  0.84425225, -0.20943951,  0.05461055,  0.44852955]), 
                     'index': np.array([-0.34906585, -0.26179939,  0.34882821]), 
                     'middle': np.array([-0.18522597, -0.26179939,  0.42122631]), 
                     'ring': np.array([-0.02449233, -0.26179939,  0.66487361]), 
                     'little': np.array([ 0.        , -0.19292931, -0.26179939,  1.22359992])}

    th = from_hand_emb['thumb']  
    ff = from_hand_emb['index']  
    mf = from_hand_emb['middle'] 
    rf = from_hand_emb['ring']    
    lf = from_hand_emb['little']                        

    joints_hand_emb = {'rh_FFJ1': max(0, ff[2]-pi/2), 'rh_FFJ2': min(pi/2, ff[2]), 'rh_FFJ3': ff[1], 'rh_FFJ4': ff[0], 
                       'rh_LFJ1': max(0, lf[2]-pi/2), 'rh_LFJ2': min(pi/2, lf[3]), 'rh_LFJ3': lf[2], 'rh_LFJ4': lf[1], 'rh_LFJ5': lf[0], 
                       'rh_MFJ1': max(0, mf[2]-pi/2), 'rh_MFJ2': min(pi/2, mf[2]), 'rh_MFJ3': mf[1], 'rh_MFJ4': mf[0], 
                       'rh_RFJ1': max(0, rf[2]-pi/2), 'rh_RFJ2': min(pi/2, rf[2]), 'rh_RFJ3': rf[1], 'rh_RFJ4': rf[0], 
                       'rh_THJ1': th[4], 'rh_THJ2': th[3], 'rh_THJ3': th[2], 'rh_THJ4': th[1], 'rh_THJ5': th[0], 
                       'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}
    
    start = timer()

    hand_commander.move_to_joint_value_target_unsafe(joint_states=joints_hand_emb, wait=True, angle_degrees=False)

    stop = timer()

    print('Elapsed time:', round((stop-start)*1000,3), 'ms')
        
    rospy.loginfo("Done. Exiting.")