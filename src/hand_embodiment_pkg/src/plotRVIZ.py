#!/usr/bin/env python3

#* * * * * * plotRVIZ.py * * * * * * *#
#*   Plots 3D hand points in RVIZ    *#
#*   Points: - hand_top              *#
#*           - hand_left             *#
#*           - hand_right            *#
#*           - thumb_tip             *#
#*           - thumb_middle          *#
#*           - little_tip            *#
#*           - little_middle         *#
#*           - ring_tip              *#
#*           - ring_middle           *#
#*           - middle_middle         *#
#*           - middle_tip            *#
#*           - index_tip             *#
#*           - index_middle          *#
#* * * * * * * * * * * * * * * * * * *#

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# Init ROS node
rospy.init_node('keypoint_publisher_node')

# Create publisher
pub = rospy.Publisher('keypoints_marker_array', MarkerArray, queue_size=10)

# Create msg
marker_array = MarkerArray()

# Test with frame 2000 of "test/data/recording.tsv"			
line2000 = [[197.090, 186.785, 355.568],  # hand_top          [0:3]
            [148.571, 212.874, 351.756],  # hand_left         [3:6]
            [152.791, 181.826, 360.058],  # hand_right        [6:6]
            [187.023, 258.098, 238.769],  # thumb_tip         [9:12]
            [173.292, 248.519, 266.282],  # thumb_middle      [12:15]
            [286.472, 143.090, 312.203],  # little_tip        [15:18]
            [251.295, 140.943, 328.034],  # little_middle     [18:21] 
            [318.117, 167.697, 311.492],  # ring_tip          [21:24]
            [264.293, 164.880, 337.638],  # ring_middle       [24:27]
            [266.009, 204.709, 340.339],  # middle_middle     [27:30]
            [327.375, 195.134, 316.995],  # middle_tip        [30:33]
            [312.663, 236.605, 322.433],  # index_tip         [33:36]
	        [255.355, 231.975, 342.499]]  # index_middle      [36:39] 
           

# Fill msg
for i, kp in enumerate(line2000):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.id = i
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = kp[0]/100
    marker.pose.position.y = kp[1]/100
    marker.pose.position.z = kp[2]/100
    marker.pose.orientation.w = 1
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker_array.markers.append(marker)

# Add links between joints
links = [ [0, 1, 2, 0],  # Hand
          [3, 4],        # Thumb
          [5, 6],        # Littlefinger
          [7, 8],        # Ringfinger
          [9, 10],       # Middlefinger
          [11, 12] ]     # Indexfinger

for link in links:
    line_marker = Marker()
    line_marker.header.frame_id = "world"
    line_marker.id = len(marker_array.markers)
    line_marker.type = Marker.LINE_STRIP
    line_marker.action = Marker.ADD
    line_marker.pose.orientation.w = 1
    line_marker.scale.x = 0.05
    line_marker.color.a = 1.0
    line_marker.color.r = 0.0
    line_marker.color.g = 1.0
    line_marker.color.b = 0.0

    # set points of the line
    for i in link:
        point = Point()
        point.x = line2000[i][0]/100
        point.y = line2000[i][1]/100
        point.z = line2000[i][2]/100
        line_marker.points.append(point)

    marker_array.markers.append(line_marker)

# Publish the MarkerArray message at a rate of 1Hz
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    pub.publish(marker_array)
    rate.sleep()