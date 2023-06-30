#!/usr/bin/env python3

#* * * * * * * * * * * ur5_tests_v2.py * * * * * * * * * * * *#
#*  Performs Wiring Fitting task                             *#
#*  Grabs the clip from stand                                *#
#*  Inserts the clip in the respective hole                  *#
#*  Note: clip stand and hole poses in 'world'               *#
#*  + Adapted to receive clips' positions by ROS msgs        *#
#*  + Receives clip poses from 'clips_pose_topic'            *#
#*  + Receives clip goal poses from 'clips_goal_pose_topic'  *#
#* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *#

import tf
import rospy
from threading import Thread
from termcolor import colored
from geometry_msgs.msg import PoseStamped
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander

# GLOBAL VARS

# CLIP Poses
clip_01 = PoseStamped()
clip_02 = PoseStamped()
clip_03 = PoseStamped()

# CLIP Insert Poses
clip_01_goal = PoseStamped()
clip_02_goal = PoseStamped()
clip_03_goal = PoseStamped()

# Conditions
have_clip_poses = False
have_clip_goal_poses = False

# Grab clip
clip_stand_01 = PoseStamped()
clip_stand_01.pose.position.x = 0.8090350904948178
clip_stand_01.pose.position.y = -0.05905678621987004
clip_stand_01.pose.position.z = 0.000595339747434958
clip_stand_01.pose.orientation.x = -0.7419143595894909
clip_stand_01.pose.orientation.y = -0.6699794054594519
clip_stand_01.pose.orientation.z = 0.025714768389053667
clip_stand_01.pose.orientation.w = -0.005424940719662695

# Insert clip
clip_hole_01 = PoseStamped()
clip_hole_01.pose.position.x = 0.8090350904948178
clip_hole_01.pose.position.y = -0.05905678621987004
clip_hole_01.pose.position.z = 0.000595339747434958
clip_hole_01.pose.orientation.x = -0.7419143595894909
clip_hole_01.pose.orientation.y = -0.6699794054594519
clip_hole_01.pose.orientation.z = 0.025714768389053667
clip_hole_01.pose.orientation.w = -0.005424940719662695


# HAND Poses

# Preparing to pinch clip
start_pinch_pose = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.35415, 
                    'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0, 
                    'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0, 
                    'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0, 
                    'rh_THJ1': -0.40, 'rh_THJ2': -0.15, 'rh_THJ3': 0.0, 'rh_THJ4': 1.21407, 'rh_THJ5': 0.44347, 
                    'rh_WRJ1': -0.698, 'rh_WRJ2': 0.0}

# Pinch clip
pinch_pose = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.5, 'rh_FFJ3': 1.5707, 'rh_FFJ4': 0.35415, 
              'rh_LFJ1': 1.5707, 'rh_LFJ2': 1.5707, 'rh_LFJ3': 1.5707, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0, 
              'rh_MFJ1': 1.5707, 'rh_MFJ2': 1.5707, 'rh_MFJ3': 1.5707, 'rh_MFJ4': 0.0, 
              'rh_RFJ1': 1.5707, 'rh_RFJ2': 1.5707, 'rh_RFJ3': 1.5707, 'rh_RFJ4': 0.0, 
              'rh_THJ1': -0.32294, 'rh_THJ2': -0.14608, 'rh_THJ3': 0.15715, 'rh_THJ4': 1.10, 'rh_THJ5': 0.65, 
              'rh_WRJ1': -0.698, 'rh_WRJ2': 0.0}

# Clip to UR5 poses

# Picking up
clip_ur5_pick = PoseStamped()
clip_ur5_pick.pose.position.x = -0.12850999838071914
clip_ur5_pick.pose.position.y = -0.25302097135223445 + 0.012
clip_ur5_pick.pose.position.z = -0.259793210890643 - 0.013
clip_ur5_pick.pose.orientation.x = 0.19610455102080304
clip_ur5_pick.pose.orientation.y = -0.26948830312491456
clip_ur5_pick.pose.orientation.z = 0.5375557907686208
clip_ur5_pick.pose.orientation.w = 0.7745662214161588

# Clipping
clip_ur5_clip = PoseStamped()
clip_ur5_clip.pose.position.x = -0.18684568
clip_ur5_clip.pose.position.y = -0.2417559
clip_ur5_clip.pose.position.z = -0.23364545
clip_ur5_clip.pose.orientation.x = 0.11152533
clip_ur5_clip.pose.orientation.y = -0.2564281
clip_ur5_clip.pose.orientation.z = 0.46722685
clip_ur5_clip.pose.orientation.w = 0.83875252

# # # Referential Frame # # #
#                           #
#     _       ^ x           #
#    | |      |             #
#    | |      |             #
#    |_|      .-------> y   #
#                           #
# # # # # # # # # # # # # # #

def clips_pose_callback(msg):
    """
        Callback function to handle incoming PoseStamped messages of clips' position
    """

    global clip_01, clip_02, clip_03
    global have_clip_poses

    if have_clip_poses == True:
        return

    # Clip1
    clip_01.header.frame_id = 'world'
    clip_01.pose.position.x = msg.pose.position.x + 0.1015
    clip_01.pose.position.y = msg.pose.position.y - 0.048 
    clip_01.pose.position.z = msg.pose.position.z + 0.012
    clip_01.pose.orientation.x = msg.pose.orientation.x
    clip_01.pose.orientation.y = msg.pose.orientation.y
    clip_01.pose.orientation.z = msg.pose.orientation.z
    clip_01.pose.orientation.w = msg.pose.orientation.w

    # Clip1
    clip_02.header.frame_id = 'world'
    clip_02.pose.position.x = msg.pose.position.x + 0.1015
    clip_02.pose.position.y = msg.pose.position.y 
    clip_02.pose.position.z = msg.pose.position.z + 0.012
    clip_02.pose.orientation.x = msg.pose.orientation.x
    clip_02.pose.orientation.y = msg.pose.orientation.y
    clip_02.pose.orientation.z = msg.pose.orientation.z
    clip_02.pose.orientation.w = msg.pose.orientation.w

    # Clip3
    clip_03.header.frame_id = 'world'
    clip_03.pose.position.x = msg.pose.position.x + 0.1015
    clip_03.pose.position.y = msg.pose.position.y + 0.048 
    clip_03.pose.position.z = msg.pose.position.z + 0.012
    clip_03.pose.orientation.x = msg.pose.orientation.x
    clip_03.pose.orientation.y = msg.pose.orientation.y
    clip_03.pose.orientation.z = msg.pose.orientation.z
    clip_03.pose.orientation.w = msg.pose.orientation.w
    
    print('\n' + colored('Clip poses received!', 'green') + '\n') 
    have_clip_poses = True

def clips_goal_pose_callback(msg):
    """
        Callback function to handle incoming PoseStamped messages of clips' position
    """

    global clip_01, clip_02, clip_03
    global have_clip_goal_poses

    if have_clip_goal_poses == True:
        return

    # ClipGoal1
    clip_01_goal.header.frame_id = 'world'
    clip_01_goal.pose.position.x = msg.pose.position.x
    clip_01_goal.pose.position.y = msg.pose.position.y
    clip_01_goal.pose.position.z = msg.pose.position.z
    clip_01_goal.pose.orientation.x = msg.pose.orientation.x
    clip_01_goal.pose.orientation.y = msg.pose.orientation.y
    clip_01_goal.pose.orientation.z = msg.pose.orientation.z
    clip_01_goal.pose.orientation.w = msg.pose.orientation.w

    # ClipGoal1
    clip_02_goal.header.frame_id = 'world'
    clip_02_goal.pose.position.x = msg.pose.position.x
    clip_02_goal.pose.position.y = msg.pose.position.y 
    clip_02_goal.pose.position.z = msg.pose.position.z
    clip_02_goal.pose.orientation.x = msg.pose.orientation.x
    clip_02_goal.pose.orientation.y = msg.pose.orientation.y
    clip_02_goal.pose.orientation.z = msg.pose.orientation.z
    clip_02_goal.pose.orientation.w = msg.pose.orientation.w

    # ClipGoal3
    clip_03_goal.header.frame_id = 'world'
    clip_03_goal.pose.position.x = msg.pose.position.x
    clip_03_goal.pose.position.y = msg.pose.position.y
    clip_03_goal.pose.position.z = msg.pose.position.z
    clip_03_goal.pose.orientation.x = msg.pose.orientation.x
    clip_03_goal.pose.orientation.y = msg.pose.orientation.y
    clip_03_goal.pose.orientation.z = msg.pose.orientation.z
    clip_03_goal.pose.orientation.w = msg.pose.orientation.w

    print('\n' + colored('Clip goal poses received!', 'green') + '\n') 
    have_clip_goal_poses = True
    


def get_start_pose(final_pose: PoseStamped, dist, mode: str):
    """
        Computes the initial clipping pose for a given clip hole pose. It returns a PoseStamped.
        @param final_pose - A given pose of type PoseStamped.
        @param dist - distance in meters to pre-position the UR5
        @param mode - set mode: 'pick' or 'clip'
    """
    global tf_listener, tf_broadcaster

    # Create fictitious Clip frame
    tf_broadcaster.sendTransform((final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z),
                                 (final_pose.pose.orientation.x, final_pose.pose.orientation.y, final_pose.pose.orientation.z, final_pose.pose.orientation.w),
                                 rospy.Time.now(),
                                 'fictitious_clip_frame',
                                 'world')
    
    # Calculate UR5 position from Clip position
    pose_in_clip_frame = PoseStamped()
    #pose_in_clip_frame.header.stamp = rospy.Time.now()
    pose_in_clip_frame.header.frame_id = 'fictitious_clip_frame'
    if mode == 'pick':
        pose_in_clip_frame.pose.position.x = clip_ur5_pick.pose.position.x
        pose_in_clip_frame.pose.position.y = clip_ur5_pick.pose.position.y
        pose_in_clip_frame.pose.position.z = clip_ur5_pick.pose.position.z
        pose_in_clip_frame.pose.orientation.x = clip_ur5_pick.pose.orientation.x
        pose_in_clip_frame.pose.orientation.y = clip_ur5_pick.pose.orientation.y
        pose_in_clip_frame.pose.orientation.z = clip_ur5_pick.pose.orientation.z
        pose_in_clip_frame.pose.orientation.w = clip_ur5_pick.pose.orientation.w
    elif mode == 'clip':
        pose_in_clip_frame.pose.position.x = clip_ur5_clip.pose.position.x
        pose_in_clip_frame.pose.position.y = clip_ur5_clip.pose.position.y
        pose_in_clip_frame.pose.position.z = clip_ur5_clip.pose.position.z
        pose_in_clip_frame.pose.orientation.x = clip_ur5_clip.pose.orientation.x
        pose_in_clip_frame.pose.orientation.y = clip_ur5_clip.pose.orientation.y
        pose_in_clip_frame.pose.orientation.z = clip_ur5_clip.pose.orientation.z
        pose_in_clip_frame.pose.orientation.w = clip_ur5_clip.pose.orientation.w
    else:
        print('\n' + colored('ERROR: "' + mode + '" mode not defined in "get_start_pose" method!', 'red') + '\n') 
    
    pose_in_clip_frame.pose.position.z -= dist

    # TO DO: Adjust the clip adapter position in relation to the Clip
    # TO DO: Adjust the clip hole position in relation to the Clip

    # Get position in 'world' frame
    tf_listener.waitForTransform('world', pose_in_clip_frame.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
    new_pose = tf_listener.transformPose('world', pose_in_clip_frame)

    # tf_broadcaster.sendTransform((new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z),
    #                              (new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z, new_pose.pose.orientation.w),
    #                              rospy.Time.now(),
    #                              'fictitious_ur5_frame',
    #                              'world')

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



def move_arm_to(final_pose: PoseStamped, dist, mode: str):
    """
        Moves UR5 to the specified pose
        @param final_pose - A given pose of type PoseStamped.
        @param dist - distance in meters to pre-position the UR5
        @param mode - set mode: 'pick' or 'clip'
    """
    global arm_commander

    arm_go_to = get_start_pose(final_pose, dist, mode)
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


def clipping_iteration(clip_pose: PoseStamped, clip_goal_pose: PoseStamped):
    """
        Executes one iteration of the clipping task
        @param clip_pose - Clip pose of type PoseStamped.
        @param clip_goal_pose - Clip goal a given pose of type PoseStamped.
    """
    #1 GET THE CLIP

    move_arm_to(clip_pose, 0.05, 'pick')
    move_arm_to(clip_pose, 0.05, 'pick')
    set_hand_pose('release')
    print(colored('Preparing to grab the clip', 'green'))
    if rospy.is_shutdown(): return
    rospy.sleep(1.0)
    
    move_arm_to(clip_pose, 0.00, 'pick')
    move_arm_to(clip_pose, 0.00, 'pick')
    set_hand_pose('grab')
    print(colored('Clip grabed', 'green'))
    if rospy.is_shutdown(): return
    rospy.sleep(1.0)

    move_arm_to(clip_pose, 0.05, 'pick')
    move_arm_to(clip_pose, 0.05, 'pick')
    print(colored('Moved back', 'green'))
    if rospy.is_shutdown(): return
    rospy.sleep(1.0)
    
    #2 CLIPPING TASK

    move_arm_to(clip_goal_pose, 0.05, 'clip')
    move_arm_to(clip_goal_pose, 0.05, 'clip')
    print(colored('Preparing to clip', 'green'))
    if rospy.is_shutdown(): return
    rospy.sleep(1.0)
    

    move_arm_to(clip_goal_pose, 0.00, 'clip')
    move_arm_to(clip_goal_pose, 0.00, 'clip')
    print(colored('Clip in place', 'green'))
    if rospy.is_shutdown(): return
    rospy.sleep(1.0)

    set_hand_pose('release')
    move_arm_to(clip_goal_pose, 0.05, 'clip')
    move_arm_to(clip_goal_pose, 0.05, 'clip')
    print(colored('Iteration completed!', 'green'))
    if rospy.is_shutdown(): return
    rospy.sleep(1.0)



def main_task():
    """
        Clipping task execution
    """
    while not rospy.is_shutdown():

        if (have_clip_poses==False) or (have_clip_goal_poses==False) :
            rospy.sleep(0.5)
            continue

        print('Starting clipping iteration 1')
        clipping_iteration(clip_01, clip_01_goal)
        print('End of clipping iteration 1')
        if rospy.is_shutdown(): return

        print('Starting clipping iteration 2')
        clipping_iteration(clip_02, clip_02_goal)
        print('End of clipping iteration 2')
        if rospy.is_shutdown(): return

        print('Starting clipping iteration 3')
        clipping_iteration(clip_03, clip_03_goal)
        print('End of clipping iteration 3')
        if rospy.is_shutdown(): return

        exit(1)



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
    speed = 1.0
    hand_commander.set_max_velocity_scaling_factor(speed)
    hand_commander.set_max_acceleration_scaling_factor(speed)
    arm_commander.set_max_velocity_scaling_factor(speed)
    arm_commander.set_max_acceleration_scaling_factor(speed)

    # Main Thread
    thread1 = Thread(target=main_task)
    thread1.start()

    print('\n' + colored('"ur5_tests" ROS node is ready!', 'green') + '\n') 

    # ROS Subscribers
    rospy.Subscriber('clips_pose_topic', PoseStamped, clips_pose_callback)
    rospy.Subscriber('clips_goal_pose_topic', PoseStamped, clips_goal_pose_callback)

    print('Waiting for clip positions...')

    rospy.spin()
