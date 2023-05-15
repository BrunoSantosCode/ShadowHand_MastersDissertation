#!/usr/bin/env python3

import rospy
from openpose_mujoco.msg import Frame, HandKeypoints  
from geometry_msgs.msg import Point

# OpenPose callback
def openPose_CB(msg):

    # Check if person is detected
    if len(msg.persons) > 0:
        
        # Get hand information from OpenPose
        person = msg.persons[0]
        hand_kp = person.rightHandParts

        # Message to be published
        pub_msg = HandKeypoints()

        # Fill header
        pub_msg.header.seq = msg.header.seq
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.header.frame_id = 'hand_frame'

        # Fill keypoints
        for i in range(0, 21):
            pub_msg.keypoints.append(Point(hand_kp[i].point.x, hand_kp[i].point.y, hand_kp[i].point.z))

        # Publish message
        hand_kp_pub.publish(pub_msg)    
        print('Message',pub_msg.header.seq,'published!')



if __name__ == "__main__":
    rospy.init_node('openpose2handkp')  
    hand_kp_pub = rospy.Publisher('hand_kp', HandKeypoints, queue_size=1)
    rospy.Subscriber('frame_topic', Frame, openPose_CB, queue_size=1)
    rospy.spin()
