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

import imp
import numpy as np
import rospy
import threading
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from moveit_msgs.msg import RobotTrajectory
from moveit_commander import MoveGroupCommander
from openpose_mujoco.msg import HandKeypoints
from sensor_msgs.msg import JointState

AVERAGE_N = 5
joints = np.zeros((24, AVERAGE_N))
send_joints_prev = []

# Function that move the robot to a specified joints positions
def execute_trajectory_hand_arm(commander: SrRobotCommander, plan: RobotTrajectory, move_group, pos_ur5, pos_sh, time):
    coded_trajectory = [{'joint_angles': {'rh_FFJ1': pos_sh[0], 'rh_FFJ2': pos_sh[1], 'rh_FFJ3': pos_sh[2], 'rh_FFJ4': pos_sh[3], 
                                          'rh_LFJ1': pos_sh[4], 'rh_LFJ2': pos_sh[5], 'rh_LFJ3': pos_sh[6], 'rh_LFJ4': pos_sh[7], 'rh_LFJ5': pos_sh[8], 
                                          'rh_MFJ1': pos_sh[9], 'rh_MFJ2': pos_sh[10], 'rh_MFJ3': pos_sh[11], 'rh_MFJ4': pos_sh[12], 
                                          'rh_RFJ1': pos_sh[13], 'rh_RFJ2': pos_sh[14], 'rh_RFJ3': pos_sh[15], 'rh_RFJ4': pos_sh[16], 
                                          'rh_THJ1': pos_sh[17], 'rh_THJ2': pos_sh[18], 'rh_THJ3': pos_sh[19], 'rh_THJ4': pos_sh[20], 'rh_THJ5': pos_sh[21], 
                                          'rh_WRJ1': pos_sh[22], 'rh_WRJ2': pos_sh[23], 
                                          'ra_elbow_joint': pos_ur5[0], 'ra_shoulder_lift_joint': pos_ur5[1], 'ra_shoulder_pan_joint': pos_ur5[2], 
                                          'ra_wrist_1_joint': pos_ur5[3], 'ra_wrist_2_joint': pos_ur5[4], 'ra_wrist_3_joint': pos_ur5[5]}, 
                                          'interpolate_time': time}]
    joint_trajectory = commander.make_named_trajectory(coded_trajectory)
    plan.joint_trajectory = joint_trajectory
    commander.execute_plan(plan)
    #move_group.execute(plan)

def execute_trajectory_hand(commander, plan, move_group: MoveGroupCommander, pos_sh, time):
    coded_trajectory = [{'joint_angles': {'rh_FFJ1': pos_sh[0], 'rh_FFJ2': pos_sh[1], 'rh_FFJ3': pos_sh[2], 'rh_FFJ4': pos_sh[3], 
                                          'rh_LFJ1': pos_sh[4], 'rh_LFJ2': pos_sh[5], 'rh_LFJ3': pos_sh[6], 'rh_LFJ4': pos_sh[7], 'rh_LFJ5': pos_sh[8], 
                                          'rh_MFJ1': pos_sh[9], 'rh_MFJ2': pos_sh[10], 'rh_MFJ3': pos_sh[11], 'rh_MFJ4': pos_sh[12], 
                                          'rh_RFJ1': pos_sh[13], 'rh_RFJ2': pos_sh[14], 'rh_RFJ3': pos_sh[15], 'rh_RFJ4': pos_sh[16], 
                                          'rh_THJ1': pos_sh[17], 'rh_THJ2': pos_sh[18], 'rh_THJ3': pos_sh[19], 'rh_THJ4': pos_sh[20], 'rh_THJ5': pos_sh[21], 
                                          'rh_WRJ1': pos_sh[22], 'rh_WRJ2': pos_sh[23]},  
                                          'interpolate_time': time}]
    #joint_trajectory = commander.make_named_trajectory(coded_trajectory)
    #plan.joint_trajectory = joint_trajectory
    ##commander.execute_plan(plan)
    #move_group.execute(plan)

    states = JointState()
    states.name = ['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 
                   'rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5', 
                   'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4', 
                   'rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4', 
                   'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5', 
                   'rh_WRJ1', 'rh_WRJ2']
    states.position = pos_sh
    move_group.set_joint_value_target(states)
    move_group.go()


def execute_trajectory_hand_unsafe(hand_commander: SrRobotCommander, pos, time):
    hand_pos = hand_commander.get_joints_position()

    hand_pos.update({'rh_FFJ1': pos[0], 'rh_FFJ2': pos[1], 'rh_FFJ3': pos[2], 'rh_FFJ4': pos[3]})
    hand_pos.update({'rh_LFJ1': pos[4], 'rh_LFJ2': pos[5], 'rh_LFJ3': pos[6], 'rh_LFJ4': pos[7], 'rh_LFJ5': pos[8]})
    hand_pos.update({'rh_MFJ1': pos[9], 'rh_MFJ2': pos[10], 'rh_MFJ3': pos[11], 'rh_MFJ4': pos[12]})
    hand_pos.update({'rh_RFJ1': pos[13], 'rh_RFJ2': pos[14], 'rh_RFJ3': pos[15], 'rh_RFJ4': pos[16]})
    hand_pos.update({'rh_THJ1': pos[17], 'rh_THJ2': pos[18], 'rh_THJ3': pos[19], 'rh_THJ4': pos[20], 'rh_THJ5': pos[21]})
    hand_pos.update({'rh_WRJ1': pos[22], 'rh_WRJ2': pos[23]})

    hand_commander.move_to_joint_value_target_unsafe(joint_states=hand_pos, time=time, wait=True, angle_degrees=False)


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
def openPose_CB(msg: HandKeypoints):
    print('Data', msg.header.seq, 'received...')

    global joints, joints_min_rad, joints_max_rad

    move_group_commander.clear_pose_targets()
    rh_thtip = [msg.keypoints[4].x, msg.keypoints[4].y, msg.keypoints[4].z]
    move_group_commander.set_position_target(rh_thtip, 'rh_thtip')
    rh_fftip = [msg.keypoints[8].x, msg.keypoints[8].y, msg.keypoints[8].z]
    move_group_commander.set_position_target(rh_fftip, 'rh_fftip')
    rh_mftip = [msg.keypoints[12].x, msg.keypoints[12].y, msg.keypoints[12].z]
    move_group_commander.set_position_target(rh_mftip, 'rh_mftip')
    rh_rftip = [msg.keypoints[16].x, msg.keypoints[16].y, msg.keypoints[16].z]
    move_group_commander.set_position_target(rh_rftip, 'rh_rftip')
    rh_lftip = [msg.keypoints[20].x, msg.keypoints[20].y, msg.keypoints[20].z]
    move_group_commander.set_position_target(rh_lftip, 'rh_lftip')

    move_group_commander.go()
    
    # Convert to Shadow Hand (DexPilot)
    #hand_joints, cost = dexPilot_joints(msg.keypoints)

    # Check joints limits
    #for i in range(0,len(hand_joints)):
    #    if hand_joints[i] > joints_max_rad[i]:
    #        hand_joints[i] = joints_max_rad[i]
    #    elif hand_joints[i] < joints_min_rad[i]:
    #        hand_joints[i] = joints_min_rad[i]
#
    #for i in range(0, len(joints)):
    #    joints[i] = np.roll(joints[i], -1)
    #    joints[i][-1] = hand_joints[i]

    #go_to_time = 0.1

    # Convert to degrees and print results
    if False:
        hand_joints_degrees = np.degrees(np.array(hand_joints))
        np.set_printoptions(suppress=True)
        print('Joints after DexPilot [degrees]:\n', hand_joints_degrees)
        print('Cost:', cost)
    
    # Run trajectory [Shadow Hand]
    #send_joints = []
    #for i in range(0, len(joints)):
    #    send_joints.append(np.mean(joints[i]))
    #if False:
    #    print('Joints:', send_joints)
    #execute_trajectory_hand(robot_commander, plan, move_group_commander, send_joints, go_to_time)

    ## Default UR5 pose
    #arm_up2  = [0.00, -1.58, -0.02, -3.14, -1.57, 0.00]        
    ## Run trajectory [UR5+SH]
    #execute_trajectory_hand_arm(robot_commander, plan, move_group_commander, arm_up2, hand_joints, go_to_time) 
    


# Subscribes to hand joint angles from OpenPose
def openPose_sub():
    rospy.Subscriber('hand_kp', HandKeypoints, openPose_CB, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('run_hand_poses')

    # Execute trajectories [UR5]
    #robot_commander = SrRobotCommander(name='right_hand')
    #plan = RobotTrajectory()
    move_group_commander = MoveGroupCommander('right_hand')

    print("Ready to execute...")


    ## Execute trajectories [UR5+SH]
    #robot_commander = SrRobotCommander(name='right_arm_and_hand')
    #plan = RobotTrajectory()
    #move_group_commander = MoveGroupCommander('right_arm_and_hand')
    #print("Ready to execute...")   

    #plot_thread = threading.Thread(target=send_commands_threads)
    #plot_thread.start()

    openPose_sub()