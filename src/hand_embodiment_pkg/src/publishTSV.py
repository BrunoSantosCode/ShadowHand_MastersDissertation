#!/usr/bin/env python3

import csv
import rospy
from rospkg import RosPack
from termcolor import colored
from geometry_msgs.msg import Point
from hand_embodiment_pkg.msg import HandKeypoints

rospack = RosPack()
path = rospack.get_path('hand_embodiment_pkg') + '/test/data/recording.tsv'

if __name__ == '__main__':
    rospy.init_node('publishTSV')

    # Create Publisher
    pub = rospy.Publisher('hand_kp', HandKeypoints, queue_size=1)

    # Publish rate
    rate = rospy.Rate(100)  # Publish every 0.01 sec [100 Hz]

    print('\n' + colored('"hand_embodiment_shadow" ROS node is ready!', 'green') + '\n')  

    while not rospy.is_shutdown():
        # Open file
        with open(path, 'r') as f:
            reader = csv.reader(f, delimiter='\t')

            # Skip the header
            for i in range(11):
                next(reader)

            # Get the column indices for the hand keypoints
            header = next(reader)
            hand_top_x_index = header.index('hand_top X')
            hand_top_y_index = header.index('hand_top Y')
            hand_top_z_index = header.index('hand_top Z')
            hand_left_x_index = header.index('hand_left X')
            hand_left_y_index = header.index('hand_left Y')
            hand_left_z_index = header.index('hand_left Z')
            hand_right_x_index = header.index('hand_right X')
            hand_right_y_index = header.index('hand_right Y')
            hand_right_z_index = header.index('hand_right Z')
            thumb_tip_x_index = header.index('thumb_tip X') 
            thumb_tip_y_index = header.index('thumb_tip Y') 	
            thumb_tip_z_index = header.index('thumb_tip Z') 	
            thumb_middle_x_index = header.index('thumb_middle X')	
            thumb_middle_y_index = header.index('thumb_middle Y')		
            thumb_middle_z_index = header.index('thumb_middle Z')		 
            little_tip_x_index = header.index('little_tip X')	
            little_tip_y_index = header.index('little_tip Y')	
            little_tip_z_index = header.index('little_tip Z')		
            little_middle_x_index = header.index('little_middle X')	
            little_middle_y_index = header.index('little_middle Y')		
            little_middle_z_index = header.index('little_middle Z')		
            ring_tip_x_index = header.index('ring_tip X')	
            ring_tip_y_index = header.index('ring_tip Y')		
            ring_tip_z_index = header.index('ring_tip Z')		
            ring_middle_x_index = header.index('ring_middle X')	
            ring_middle_y_index = header.index('ring_middle Y')		
            ring_middle_z_index = header.index('ring_middle Z')		
            middle_middle_x_index = header.index('middle_middle X')	
            middle_middle_y_index = header.index('middle_middle Y')		
            middle_middle_z_index = header.index('middle_middle Z')		
            middle_tip_x_index = header.index('middle_tip X')
            middle_tip_y_index = header.index('middle_tip Y')
            middle_tip_z_index = header.index('middle_tip Z')
            index_tip_x_index = header.index('index_tip X')
            index_tip_y_index = header.index('index_tip Y')
            index_tip_z_index = header.index('index_tip Z')
            index_middle_x_index = header.index('index_middle X')
            index_middle_y_index = header.index('index_middle Y')	
            index_middle_z_index = header.index('index_middle Z')	

            # Iteratively read .tsv file
            for row in reader:
                if rospy.is_shutdown():
                    exit(-1)
                    
                # Message to be published
                pub_msg = HandKeypoints()

                # Fill header
                pub_msg.header.stamp = rospy.Time.now()
                pub_msg.header.frame_id = 'hand_frame'

                # Fill keypoints
                for i in range(0, 21):
                    pub_msg.keypoints.append(Point())

                pub_msg.keypoints[0] = Point(float(row[hand_top_x_index]), float(row[hand_top_y_index]), float(row[hand_top_z_index]))
                pub_msg.keypoints[1] = Point(float(row[hand_left_x_index]), float(row[hand_left_y_index]), float(row[hand_left_z_index]))
                pub_msg.keypoints[2] = Point(float(row[hand_right_x_index]), float(row[hand_right_y_index]), float(row[hand_right_z_index]))

                pub_msg.keypoints[4] = Point(float(row[thumb_tip_x_index]), float(row[thumb_tip_y_index]), float(row[thumb_tip_z_index]))
                pub_msg.keypoints[8] = Point(float(row[index_tip_x_index]), float(row[index_tip_y_index]), float(row[index_tip_z_index]))
                pub_msg.keypoints[12] = Point(float(row[middle_tip_x_index]), float(row[middle_tip_y_index]), float(row[middle_tip_z_index]))
                pub_msg.keypoints[16] = Point(float(row[ring_tip_x_index]), float(row[ring_tip_y_index]), float(row[ring_tip_z_index]))
                pub_msg.keypoints[20] = Point(float(row[little_tip_x_index]), float(row[little_tip_y_index]), float(row[little_tip_z_index]))

                # DEBUG
                if False:
                    print('Hand Top:')
                    print(pub_msg.keypoints[0])

                print('Time: {:.2f} s'.format(float(row[1])))

                # Publish msg
                pub.publish(pub_msg)

                rate.sleep()
