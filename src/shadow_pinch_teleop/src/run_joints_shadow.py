#!/usr/bin/env python3

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from std_msgs.msg import Float64MultiArray

def main():
    rospy.init_node('run_joints_shadow')
    hand_commander = SrHandCommander(name="right_hand")

    hand_commander.set_max_acceleration_scaling_factor(1)
    hand_commander.set_max_velocity_scaling_factor(1)

    while not rospy.is_shutdown():
        joints_msg = rospy.wait_for_message("/shadow_joints", Float64MultiArray)
        goal = joints_msg.data

        # bug:get_joints_position() return radian joints
        hand_pos = hand_commander.get_joints_position()

        # Joints order
        # rh_WRJ2  rh_WRJ1
        # rh_FFJ4  rh_FFJ3  rh_FFJ2  rh_FFJ1 
        # rh_LFJ5  rh_LFJ4  rh_LFJ3  rh_LFJ1
        # rh_MFJ4  rh_MFJ3  rh_MFJ2  rh_MFJ1
        # rh_RFJ4  rh_RFJ3  rh_RFJ2  rh_RFJ1
        # rh_THJ5  rh_THJ4  rh_THJ3  rh_THJ2  rh_THJ1

        # wrist
        hand_pos.update({"rh_WRJ2": goal[0]})
        hand_pos.update({"rh_WRJ1": goal[1]})

        # first finger
        hand_pos.update({"rh_FFJ4": goal[2]})
        hand_pos.update({"rh_FFJ3": goal[3]})
        hand_pos.update({"rh_FFJ2": goal[4]})
        hand_pos.update({"rh_FFJ1": goal[5]})
        
        # little finger
        hand_pos.update({"rh_LFJ5": goal[6]})
        hand_pos.update({"rh_LFJ4": goal[7]})
        hand_pos.update({"rh_LFJ3": goal[8]})
        hand_pos.update({"rh_LFJ2": goal[9]})
        hand_pos.update({"rh_LFJ1": goal[10]})

        # middle finger
        hand_pos.update({"rh_MFJ4": goal[11]})
        hand_pos.update({"rh_MFJ3": goal[12]})
        hand_pos.update({"rh_MFJ2": goal[13]})
        hand_pos.update({"rh_MFJ1": goal[14]})

        # ring finger
        hand_pos.update({"rh_RFJ4": goal[15]})
        hand_pos.update({"rh_RFJ3": goal[16]})
        hand_pos.update({"rh_RFJ2": goal[17]})
        hand_pos.update({"rh_RFJ1": goal[18]})

        # thumb
        hand_pos.update({"rh_THJ5": goal[19]})
        hand_pos.update({"rh_THJ4": goal[20]})
        hand_pos.update({"rh_THJ3": goal[21]})
        hand_pos.update({"rh_THJ2": goal[22]})
        hand_pos.update({"rh_THJ1": goal[23]})

        hand_commander.move_to_joint_value_target(joint_states=hand_pos, wait=False, angle_degrees=False)

        rospy.loginfo("Next one please ---->")


if __name__ == "__main__":
    main()