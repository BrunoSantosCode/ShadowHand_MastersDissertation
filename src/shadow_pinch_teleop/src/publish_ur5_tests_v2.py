#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    # Creating and setting values for clip_stand_01 pose
    clip_stand_01 = PoseStamped()
    clip_stand_01.pose.position.x = 0.8090350904948178
    clip_stand_01.pose.position.y = -0.05905678621987004
    clip_stand_01.pose.position.z = 0.000595339747434958
    clip_stand_01.pose.orientation.x = -0.7419143595894909
    clip_stand_01.pose.orientation.y = -0.6699794054594519
    clip_stand_01.pose.orientation.z = 0.025714768389053667
    clip_stand_01.pose.orientation.w = -0.005424940719662695

    # Creating and setting values for clip_hole_01 pose
    clip_hole_01 = PoseStamped()
    clip_hole_01.pose.position.x = 0.8090350904948178
    clip_hole_01.pose.position.y = -0.05905678621987004
    clip_hole_01.pose.position.z = 0.000595339747434958
    clip_hole_01.pose.orientation.x = -0.7419143595894909
    clip_hole_01.pose.orientation.y = -0.6699794054594519
    clip_hole_01.pose.orientation.z = 0.025714768389053667
    clip_hole_01.pose.orientation.w = -0.005424940719662695

    rospy.init_node('publish_ur5_tests_v2')
    rate = rospy.Rate(2)

    pub1 = rospy.Publisher('clips_pose_topic', PoseStamped, queue_size=1)
    pub2 = rospy.Publisher('clips_goal_pose_topic', PoseStamped, queue_size=1)

    while not rospy.is_shutdown():
        # Publishing clip_stand_01 pose to 'clips_pose_topic'
        pub1.publish(clip_stand_01)
        
        rospy.sleep(0.1)

        # Publishing clip_hole_01 pose to 'clips_goal_pose_topic'
        pub2.publish(clip_hole_01)

        rate.sleep()
