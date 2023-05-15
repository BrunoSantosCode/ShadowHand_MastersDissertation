#include <ros/ros.h>
#include <human_robot_map/HandKeypoints.h>
#include <geometry_msgs/Point.h>

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "hand_keypoints_publisher");
    std::cout << "\"hand_keypoints_publisher\" ROS node started!" << std::endl;
    ros::NodeHandle nh;

    // Create Publisher
    ros::Publisher hand_keypoints_pub = nh.advertise<human_robot_map::HandKeypoints>("hand_kp", 1);

    // Create msg
    human_robot_map::HandKeypoints msg;
    geometry_msgs::Point keypoint;
    keypoint.x = 3.0, keypoint.y = 1.0, keypoint.z = 1.0;  // wrist0
    msg.keypoints.push_back(keypoint);
    keypoint.x = 4.0, keypoint.y = 2.0, keypoint.z = 1.0;  // thumb1
    msg.keypoints.push_back(keypoint);
    keypoint.x = 5.0, keypoint.y = 3.0, keypoint.z = 1.0;  // thumb2
    msg.keypoints.push_back(keypoint);
    keypoint.x = 5.0, keypoint.y = 4.0, keypoint.z = 1.0;  // thumb3
    msg.keypoints.push_back(keypoint);
    keypoint.x = 5.0, keypoint.y = 5.0, keypoint.z = 1.0;  // thumb4
    msg.keypoints.push_back(keypoint);
    keypoint.x = 4.0, keypoint.y = 4.0, keypoint.z = 1.0;  // index5
    msg.keypoints.push_back(keypoint);
    keypoint.x = 4.0, keypoint.y = 5.0, keypoint.z = 1.0;  // index6
    msg.keypoints.push_back(keypoint);
    keypoint.x = 4.0, keypoint.y = 6.0, keypoint.z = 1.0;  // index7
    msg.keypoints.push_back(keypoint);
    keypoint.x = 4.0, keypoint.y = 7.0, keypoint.z = 1.0;  // index8
    msg.keypoints.push_back(keypoint);
    keypoint.x = 3.0, keypoint.y = 4.0, keypoint.z = 1.0;  // middle9
    msg.keypoints.push_back(keypoint);
    keypoint.x = 3.0, keypoint.y = 5.0, keypoint.z = 1.0;  // middle10
    msg.keypoints.push_back(keypoint);
    keypoint.x = 3.0, keypoint.y = 6.0, keypoint.z = 1.0;  // middle11
    msg.keypoints.push_back(keypoint);
    keypoint.x = 3.0, keypoint.y = 7.0, keypoint.z = 1.0;  // middle12
    msg.keypoints.push_back(keypoint);
    keypoint.x = 2.0, keypoint.y = 4.0, keypoint.z = 1.0;  // ring13
    msg.keypoints.push_back(keypoint);
    keypoint.x = 2.0, keypoint.y = 5.0, keypoint.z = 1.0;  // ring14
    msg.keypoints.push_back(keypoint);
    keypoint.x = 2.0, keypoint.y = 6.0, keypoint.z = 1.0;  // ring15
    msg.keypoints.push_back(keypoint);
    keypoint.x = 2.0, keypoint.y = 7.0, keypoint.z = 1.0;  // ring16
    msg.keypoints.push_back(keypoint);
    keypoint.x = 1.0, keypoint.y = 4.0, keypoint.z = 1.0;  // little17
    msg.keypoints.push_back(keypoint);
    keypoint.x = 1.0, keypoint.y = 5.0, keypoint.z = 1.0;  // little18
    msg.keypoints.push_back(keypoint);
    keypoint.x = 1.0, keypoint.y = 6.0, keypoint.z = 1.0;  // little19
    msg.keypoints.push_back(keypoint);
    keypoint.x = 1.0, keypoint.y = 7.0, keypoint.z = 1.0;  // little20
    msg.keypoints.push_back(keypoint);

    std::cout << "Wrist = (" << msg.keypoints[0].x << "," << msg.keypoints[0].y << "," << msg.keypoints[0].z << ")" << std::endl;

    // Publish msg
    ros::Rate rate(1); // rate in Hz
    while (ros::ok())
    {
        hand_keypoints_pub.publish(msg);
        std::cout << "Msg published!" << std::endl;
        rate.sleep();
    }

  return 0;
}