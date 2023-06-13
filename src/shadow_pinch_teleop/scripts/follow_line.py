#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
from sr_robot_commander.sr_arm_commander import SrArmCommander

# Constants
START_POINT = Point(x=0.2019239147313784, y=0.36016053406944526, z=0.5647490714802889)  # Start point of the line
END_POINT = Point(x=0.2019239147313784, y=0.36016053406944526, z=0.6147490714802889)    # End point of the line
WAYPOINT_DISTANCE = 0.01                  # Distance between each waypoint

def calculate_waypoints(start_point, end_point, waypoint_distance):
    waypoints = []
    direction = Point(
        x=end_point.x - start_point.x,
        y=end_point.y - start_point.y,
        z=end_point.z - start_point.z
    )
    distance = (direction.x ** 2 + direction.y ** 2 + direction.z ** 2) ** 0.5
    num_waypoints = int(distance / waypoint_distance)

    if num_waypoints == 0:
        rospy.logerr("Invalid waypoint distance. No waypoints generated.")
        return waypoints

    delta_x = direction.x * waypoint_distance / distance
    delta_y = direction.y * waypoint_distance / distance
    delta_z = direction.z * waypoint_distance / distance

    for i in range(num_waypoints + 1):
        waypoint = Point(
            x=start_point.x + i * delta_x,
            y=start_point.y + i * delta_y,
            z=start_point.z + i * delta_z
        )
        waypoints.append(waypoint)

    return waypoints

def follow_line():
    rospy.init_node('follow_line_node', anonymous=True)

    # Initialize the SrArmCommander
    arm_commander = SrArmCommander()

    # Calculate the waypoints along the line
    waypoints = calculate_waypoints(START_POINT, END_POINT, WAYPOINT_DISTANCE)

    # Move the robot along the line
    for waypoint in waypoints:
        # Set the goal pose
        goal_pose = Pose()
        goal_pose.position = waypoint
        goal_pose.orientation.x = 0.615540147210404
        goal_pose.orientation.y = -0.3569674764038792
        goal_pose.orientation.z = -0.6151713797376471
        goal_pose.orientation.w = -0.3394830209505622
        #goal_pose.orientation = Quaternion(w=1.0)

        # Move the robot to the goal pose
        print('Here')
        arm_commander.move_to_pose_target(goal_pose)

        # Wait for the robot to reach the goal pose
        rospy.sleep(2.0)

if __name__ == '__main__':
    try:
        follow_line()
    except rospy.ROSInterruptException:
        pass
