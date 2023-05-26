#!/usr/bin/env python3

import rospy
import sys, select, termios, tty
from sr_robot_commander.sr_hand_commander import SrHandCommander

def main():
    rospy.init_node('get_shadow_joints')

    hand_commander = SrHandCommander(name="right_hand")

    print('\n')
    print("Press 'r' to print the current joint positions, 'q' to quit.")

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            if key == 'r':
                print('\n')
                print(hand_commander.get_joints_position())
            elif key == 'q':
                break

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    main()
