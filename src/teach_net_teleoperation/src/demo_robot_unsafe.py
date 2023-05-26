#!/usr/bin/env python3

import rospy
import moveit_commander
from math import degrees
from sr_robot_commander.sr_hand_commander import SrHandCommander
from std_msgs.msg import Float64MultiArray

def main():
    rospy.init_node('shadow_unsafe_mode_teleop')
    hand_commander = SrHandCommander(name="right_hand")

    while not rospy.is_shutdown():
        joints_msg = rospy.wait_for_message("/teleop_outputs_joints", Float64MultiArray)
        goal = joints_msg.data

        # bug:get_joints_position() return radian joints
        hand_pos = hand_commander.get_joints_position()

        if False:
            # wrist
            hand_pos.update({"rh_WRJ2": goal[0]})
            hand_pos.update({"rh_WRJ1": goal[1]})

            # first finger
            hand_pos.update({"rh_FFJ3": goal[3]})
            hand_pos.update({"rh_FFJ2": goal[4]})
            hand_pos.update({"rh_FFJ4": goal[2]})

            # middle finger
            hand_pos.update({"rh_MFJ3": goal[12]})
            hand_pos.update({"rh_MFJ2": goal[13]})

            # ring finger
            hand_pos.update({"rh_RFJ3": goal[16]})
            hand_pos.update({"rh_RFJ2": goal[17]})

            # little finger
            hand_pos.update({"rh_LFJ3": goal[8]})
            hand_pos.update({"rh_LFJ2": goal[9]})

            # thumb
            hand_pos.update({"rh_THJ5": goal[19]})
            hand_pos.update({"rh_THJ4": goal[20]})
            hand_pos.update({"rh_THJ3": goal[21]})
            hand_pos.update({"rh_THJ2": goal[22]})
        else:
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

        hand_commander.move_to_joint_value_target_unsafe(hand_pos, 0.3, False, angle_degrees=False)

        for key, value in hand_pos.items():
            hand_pos[key] = '{:.2f}'.format(degrees(value))
        print(hand_pos)

        rospy.loginfo("Next one please ---->")


if __name__ == "__main__":
    main()