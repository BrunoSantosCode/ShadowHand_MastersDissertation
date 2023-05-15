#!/usr/bin/env python3
#
# Copyright 2020, 2022 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import rospy
import numpy as np
import matplotlib.pyplot as plt
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from moveit_msgs.msg import RobotTrajectory
from moveit_commander import MoveGroupCommander
from openpose_mujoco.msg import Frame  # 'openpose_mujoco' is the package name
                                       # Frame is inside 'msg' folder
                                       # Path: openpose_mujoco/msg/Frame.msg

AVERAGE_N = 5
# Fore Finger (index finger)
FFJ1 = np.zeros(AVERAGE_N)
FFJ2 = np.zeros(AVERAGE_N)
FFJ3 = np.zeros(AVERAGE_N)
# Little Finger
LFJ1 = np.zeros(AVERAGE_N)
LFJ2 = np.zeros(AVERAGE_N)
LFJ3 = np.zeros(AVERAGE_N)
# Middle Finger
MFJ1 = np.zeros(AVERAGE_N)
MFJ2 = np.zeros(AVERAGE_N)
MFJ3 = np.zeros(AVERAGE_N)
# Ring Finger
RFJ1 = np.zeros(AVERAGE_N)
RFJ2 = np.zeros(AVERAGE_N)
RFJ3 = np.zeros(AVERAGE_N)
# Thumb
THJ1 = np.zeros(AVERAGE_N)
THJ3 = np.zeros(AVERAGE_N)  
THJ4 = np.zeros(AVERAGE_N)

# Function to shift vector positions
def shift_joints():
    global FFJ1, FFJ2, FFJ3, LFJ1, LFJ2, LFJ3, MFJ1, MFJ2, MFJ3, RFJ1, RFJ2, RFJ3, THJ1, THJ3, THJ4
    FFJ1 = np.roll(FFJ1, -1)
    FFJ2 = np.roll(FFJ2, -1)
    FFJ3 = np.roll(FFJ3, -1)
    LFJ1 = np.roll(LFJ1, -1)
    LFJ2 = np.roll(LFJ2, -1)
    LFJ3 = np.roll(LFJ3, -1)
    MFJ1 = np.roll(MFJ1, -1)
    MFJ2 = np.roll(MFJ2, -1)
    MFJ3 = np.roll(MFJ3, -1)
    RFJ1 = np.roll(RFJ1, -1)
    RFJ2 = np.roll(RFJ2, -1)
    RFJ3 = np.roll(RFJ3, -1)
    THJ1 = np.roll(THJ1, -1)
    THJ3 = np.roll(THJ3, -1)
    THJ4 = np.roll(THJ4, -1)

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

# Function to move Shadow Hand
def execute_trajectory_hand(commander, plan, move_group, pos_sh, time):
    coded_trajectory = [{'joint_angles': {'rh_FFJ1': pos_sh[0], 'rh_FFJ2': pos_sh[1], 'rh_FFJ3': pos_sh[2], 'rh_FFJ4': pos_sh[3], 
                                          'rh_LFJ1': pos_sh[4], 'rh_LFJ2': pos_sh[5], 'rh_LFJ3': pos_sh[6], 'rh_LFJ4': pos_sh[7], 'rh_LFJ5': pos_sh[8], 
                                          'rh_MFJ1': pos_sh[9], 'rh_MFJ2': pos_sh[10], 'rh_MFJ3': pos_sh[11], 'rh_MFJ4': pos_sh[12], 
                                          'rh_RFJ1': pos_sh[13], 'rh_RFJ2': pos_sh[14], 'rh_RFJ3': pos_sh[15], 'rh_RFJ4': pos_sh[16], 
                                          'rh_THJ1': pos_sh[17], 'rh_THJ2': pos_sh[18], 'rh_THJ3': pos_sh[19], 'rh_THJ4': pos_sh[20], 'rh_THJ5': pos_sh[21], 
                                          'rh_WRJ1': pos_sh[22], 'rh_WRJ2': pos_sh[23]},  
                                          'interpolate_time': time}]
    joint_trajectory = commander.make_named_trajectory(coded_trajectory)
    plan.joint_trajectory = joint_trajectory
    move_group.execute(plan)

#Define joints bounds
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



# OpenPose callback
def openPose_CB(msg):
    print('Data', msg.header.frame_id, 'received...')

    global joints_min_rad, joints_max_rad
    
    # Get hand information from OpenPose
    person = msg.persons[0]
    hand_kp = person.rightHandParts

    #Joints order ['FFJ1', 'FFJ2', 'FFJ3', 'FFJ4', 
    #              'LFJ1', 'LFJ2', 'LFJ3', 'LFJ4', 'LFJ5', 
    #              'MFJ1', 'MFJ2', 'MFJ3', 'MFJ4', 
    #              'RFJ1', 'RFJ2', 'RFJ3', 'RFJ4', 
    #              'THJ1', 'THJ2', 'THJ3', 'THJ4', 'THJ5', 
    #              'WRJ1', 'WRJ2']

    # DEBUG
    if False:
        print('Wrist position:')
        print('x:', hand_kp[0].point.x, ' y:', hand_kp[0].point.y, ' z:', hand_kp[0].point.z)
        print('Little fingertip position:')
        print('x:', hand_kp[20].point.x, ' y:', hand_kp[20].point.y, ' z:', hand_kp[20].point.z)

    shift_joints()
    # Fore Finger (index finger)
    FFJ1[-1] = get_angle(hand_kp[6].point, hand_kp[7].point, hand_kp[8].point)
    FFJ2[-1] = get_angle(hand_kp[5].point, hand_kp[6].point, hand_kp[7].point)
    FFJ3[-1] = get_angle_knuckles(hand_kp[0].point, hand_kp[9].point, hand_kp[5].point, hand_kp[6].point)
    # Little Finger
    LFJ1[-1] = get_angle(hand_kp[18].point, hand_kp[19].point, hand_kp[20].point)
    LFJ2[-1] = get_angle(hand_kp[17].point, hand_kp[18].point, hand_kp[19].point)
    LFJ3[-1] = get_angle_knuckles(hand_kp[0].point, hand_kp[9].point, hand_kp[17].point, hand_kp[18].point)
    # Middle Finger
    MFJ1[-1] = get_angle(hand_kp[10].point, hand_kp[11].point, hand_kp[12].point)
    MFJ2[-1] = get_angle(hand_kp[9].point, hand_kp[10].point, hand_kp[11].point)
    MFJ3[-1] = get_angle_knuckles(hand_kp[0].point,  hand_kp[9].point, hand_kp[9].point, hand_kp[10].point)
    # Ring Finger
    RFJ1[-1] = get_angle(hand_kp[14].point, hand_kp[15].point, hand_kp[16].point)
    RFJ2[-1] = get_angle(hand_kp[13].point, hand_kp[14].point, hand_kp[15].point)
    RFJ3[-1] = get_angle_knuckles(hand_kp[0].point,  hand_kp[9].point, hand_kp[13].point, hand_kp[14].point)  
    # Thumb
    THJ1[-1] = get_angle(hand_kp[2].point, hand_kp[3].point, hand_kp[4].point)
    THJ3[-1] = get_angle(hand_kp[5].point, hand_kp[2].point, hand_kp[3].point)  
    THJ4[-1] = get_angle(hand_kp[0].point, hand_kp[1].point, hand_kp[2].point)  

    # DEBUG
    if False:
        print('Index finger joints:')
        print('1:', np.degrees(FFJ1), ' 2:', np.degrees(FFJ2), ' 3:', np.degrees(FFJ3), ' [degrees]')
    

    # Run trajectory [Shadow Hand]
    hand_joints  = [np.mean(FFJ1), np.mean(FFJ2), np.mean(FFJ3), 0.0, 
                    np.mean(LFJ1), np.mean(LFJ2), np.mean(LFJ3), 0.0, 0.0, 
                    np.mean(MFJ1), np.mean(MFJ2), np.mean(MFJ3), 0.0, 
                    np.mean(RFJ1), np.mean(RFJ2), np.mean(RFJ3), 0.0, 
                    np.mean(THJ1), 0.0, np.mean(THJ3), np.mean(THJ4), 0.0,
                    0.0, 0.0]
    # Check joints limits
    for i in range(0,len(hand_joints)):
        if hand_joints[i] > joints_max_rad[i]:
            hand_joints[i] = joints_max_rad[i]
        elif hand_joints[i] < joints_min_rad[i]:
            hand_joints[i] = joints_min_rad[i]

    go_to_time = 0.1

    # Send trajectory to Shadow Hand
    execute_trajectory_hand(robot_commander, plan, move_group_commander, hand_joints, go_to_time)
    

# Subscribes to hand joint angles from OpenPose
def openPose_sub():
    rospy.Subscriber('frame_topic', Frame, openPose_CB, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node('hand_commander')
    
    # Execute trajectories [Shadow Hand]
    robot_commander = SrRobotCommander(name='right_hand')
    plan = RobotTrajectory()
    move_group_commander = MoveGroupCommander('right_hand')
    print("Ready to execute...")  

    openPose_sub()