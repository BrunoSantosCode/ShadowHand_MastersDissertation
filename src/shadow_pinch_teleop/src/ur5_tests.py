#!/usr/bin/env python3

#* * * * * * * * * * * ur5_tests.py  * * * * * * * * * *#
#*  Script to test ur5                                 *#
#* * * * * * * * * * * * * * * * * * * * * * * * * * * *#

import tf
import rospy
import numpy as np
from termcolor import colored
from geometry_msgs.msg import PoseStamped
from sr_robot_commander.sr_robot_commander import SrRobotCommander

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

# Pose01: position: 
pose_01 = PoseStamped()
pose_01.pose.position.x = 0.3320153763417186
pose_01.pose.position.y = 0.20802650428714126
pose_01.pose.position.z = 0.6431376949278869
pose_01.pose.orientation.x = -0.005796355932569323
pose_01.pose.orientation.y = 0.8000808997663621
pose_01.pose.orientation.z = 0.5947827268721892
pose_01.pose.orientation.w = -0.07791318182075829

# Pose02: position: 
pose_02 = PoseStamped()
pose_02.pose.position.x = 0.16396263951712656
pose_02.pose.position.y = 0.19097405811385304
pose_02.pose.position.z = 0.6593009918338222
pose_02.pose.orientation.x = -0.0073194590015851005
pose_02.pose.orientation.y = 0.7998363082094895
pose_02.pose.orientation.z = 0.5953732593201085
pose_02.pose.orientation.w = -0.07575610652805236

def pose_stamped_to_matrix(pose_stamped):
    translation = [pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z]
    rotation = [pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w]
    matrix = tf.transformations.compose_matrix(translation, rotation)
    return matrix

def calculate_transformation_matrix(forefinger_pose, arm_end_effector_pose):
    # Convert PoseStamped messages to transformation matrices
    forefinger_matrix = pose_stamped_to_matrix(forefinger_pose)
    arm_end_effector_matrix = pose_stamped_to_matrix(arm_end_effector_pose)

    # Calculate the transformation matrix from forefinger to arm end-effector
    forefinger_to_arm_matrix = np.linalg.inv(forefinger_matrix) @ arm_end_effector_matrix

    return forefinger_to_arm_matrix

# Function to get initial clipping pose based on the clip hole pose
def get_start_pose(final_pose: PoseStamped, dist):
    """
        Computes the initial clipping pose for a given clip hole pose. It returns a PoseStamped.
        @param final_pose - A given pose of type PoseStamped.
        @param dist - distance in meters to pre-position the UR5
    """
    # Get final position (x,y,z)
    x = final_pose.pose.position.x
    y = final_pose.pose.position.y
    z = final_pose.pose.position.z
    # Get orientation quaternion components (qx,qy,qz, qw)
    qx = final_pose.pose.orientation.x
    qy = final_pose.pose.orientation.y
    qz = final_pose.pose.orientation.z
    qw = final_pose.pose.orientation.w
    # Get and Normalize orientation vector
    ox = 1 - 2 * (qy*qy + qz*qz)
    oy = 2 * (qx*qy + qw*qz)
    oz =  2 * (qx*qz - qw*qy)
    orientation = np.array([ox, oy, oz])
    orientation /= np.linalg.norm(orientation)
    # Calculate new pose
    new_position = np.array([x, y, z]) - (dist * orientation)
    new_pose = PoseStamped()
    new_pose.pose.position.x = new_position[0]
    new_pose.pose.position.y = new_position[1]
    new_pose.pose.position.z = new_position[2]
    new_pose.pose.orientation = final_pose.pose.orientation

    return new_pose


if __name__ == "__main__":
    rospy.init_node('ur5_tests')

    # Shadow Arm commander
    arm_commander = SrRobotCommander(name='right_arm_and_hand')

    # Set control velocity and acceleration
    arm_commander.set_max_velocity_scaling_factor(1.0)
    arm_commander.set_max_acceleration_scaling_factor(1.0)

    print('\n' + colored('"ur5_tests" ROS node is ready!', 'green') + '\n')  

    # Get arm current pose (position + orientation)
    # arm_pose = arm_commander.get_current_pose("ra_base_link")
    # print(arm_pose)

    # while not rospy.is_shutdown():
    #     # Calculate Inverse Kinematics
    #     pose_01.pose.position = pose_default.pose.position
    #     target_joints = arm_commander.get_ik(pose_01, avoid_collisions=True)
    
    #     # Move to target joints
    #     arm_commander.move_to_joint_value_target(target_joints)

    #     rospy.sleep(2.0)

    #     # Calculate Inverse Kinematics
    #     new_pose = get_start_pose(pose_01, 0.10)
    #     target_joints = arm_commander.get_ik(new_pose, avoid_collisions=True)
    
    #     # Move to target joints
    #     arm_commander.move_to_joint_value_target(target_joints)

    #     rospy.sleep(2.0)

    rospy.spin()