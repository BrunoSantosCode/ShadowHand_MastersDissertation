#include <ros/ros.h>

#include <human_robot_map/HandKeypoints.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// ROS Publisher
ros::Publisher marker_pub, marker_pub_shadow;

// Converts one or two geometry_msgs::Point into Eigen::Vector3d
Eigen::Vector3d point2vector(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2 = geometry_msgs::Point())
{
    Eigen::Vector3d vec1(point1.x, point1.y, point1.z);
    Eigen::Vector3d vec2(point2.x, point2.y, point2.z);

    return vec1 - vec2;
}

// Maps human hand keypoints into Shadow Hand
std::vector<Eigen::Vector3d> mapShadowHand(std::vector<Eigen::Vector3d> human_kp)
{
    std::vector<Eigen::Vector3d> shadow_kp = human_kp;
    Eigen::Vector3d aux;
    // Thumb
    float metacarpal = std::sqrt(std::pow(29, 2) + std::pow(34, 2));
    shadow_kp[1] = (human_kp[1]-human_kp[0]).normalized() * metacarpal/1000;
    shadow_kp[2] = shadow_kp[1] + (human_kp[2]-human_kp[1]).normalized() * 38/1000;
    shadow_kp[3] = shadow_kp[2] + (human_kp[3]-human_kp[2]).normalized() * 32/1000;
    shadow_kp[4] = shadow_kp[3] + (human_kp[4]-human_kp[3]).normalized() * 27.5/1000;
    // Forefinger
    metacarpal = std::sqrt(std::pow(95, 2) + std::pow(33, 2));
    shadow_kp[5] = (human_kp[5]-human_kp[0]).normalized() * metacarpal/1000;
    shadow_kp[6] = shadow_kp[5] + (human_kp[6]-human_kp[5]).normalized() * 45/1000;
    shadow_kp[7] = shadow_kp[6] + (human_kp[7]-human_kp[6]).normalized() * 25/1000;
    shadow_kp[8] = shadow_kp[7] + (human_kp[8]-human_kp[7]).normalized() * 26/1000;
    // Middlefinger
    metacarpal = std::sqrt(std::pow(99, 2) + std::pow(11, 2));
    shadow_kp[9] = (human_kp[9]-human_kp[0]).normalized() * metacarpal/1000;
    shadow_kp[10] = shadow_kp[9] + (human_kp[10]-human_kp[9]).normalized() * 45/1000;
    shadow_kp[11] = shadow_kp[10] + (human_kp[11]-human_kp[10]).normalized() * 25/1000;
    shadow_kp[12] = shadow_kp[11] + (human_kp[12]-human_kp[12]).normalized() * 26/1000;
    // Ringfinger
    metacarpal = std::sqrt(std::pow(95, 2) + std::pow(11, 2));
    shadow_kp[13] = (human_kp[13]-human_kp[0]).normalized() * metacarpal/1000;
    shadow_kp[14] = shadow_kp[13] + (human_kp[14]-human_kp[13]).normalized() * 45/1000;
    shadow_kp[15] = shadow_kp[14] + (human_kp[15]-human_kp[14]).normalized() * 25/1000;
    shadow_kp[16] = shadow_kp[15] + (human_kp[16]-human_kp[15]).normalized() * 26/1000;
    // Littlefinger
    metacarpal = std::sqrt(std::pow(86.6, 2) + std::pow(33, 2));
    shadow_kp[17] = (human_kp[17]-human_kp[0]).normalized() * metacarpal/1000;
    shadow_kp[18] = shadow_kp[17] + (human_kp[18]-human_kp[17]).normalized() * 45/1000;
    shadow_kp[19] = shadow_kp[18] + (human_kp[19]-human_kp[18]).normalized() * 25/1000;
    shadow_kp[20] = shadow_kp[19] + (human_kp[20]-human_kp[19]).normalized() * 26/1000;

    return shadow_kp;
}



// Plots to RVIZ all the 21 hand keypoints and respective lines
void plotKeypointsRVIZ(std::vector<Eigen::Vector3d> kp, bool shadow)
{
    // Create a MarkerArray msg
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(26);

    // Init dot markers [keypoints]
    for (int i = 0; i < 21; i++)
    {
        marker_array.markers[i].header.frame_id = "rh_palm";
        marker_array.markers[i].header.stamp = ros::Time::now();
        marker_array.markers[i].ns = "hand_keypoints";
        marker_array.markers[i].id = i;
        marker_array.markers[i].type = visualization_msgs::Marker::SPHERE;
        marker_array.markers[i].action = visualization_msgs::Marker::ADD;
        marker_array.markers[i].scale.x = 0.01;
        marker_array.markers[i].scale.y = 0.01;
        marker_array.markers[i].scale.z = 0.01;
        marker_array.markers[i].color.a = 1.0;
        if (shadow){
            marker_array.markers[i].color.r = 1.0;
            marker_array.markers[i].color.g = 0.0;
            marker_array.markers[i].color.b = 0.0;
        }else{
            marker_array.markers[i].color.r = 0.0;
            marker_array.markers[i].color.g = 0.0;
            marker_array.markers[i].color.b = 1.0;
        }

        // Set markers positions
        marker_array.markers[i].pose.position.x = kp[i].x();
        marker_array.markers[i].pose.position.y = kp[i].y();
        marker_array.markers[i].pose.position.z = kp[i].z();
    }

    // Init line markers [lines]
    for (int i = 21; i < 26; i++)
    {
        marker_array.markers[i].header.frame_id = "rh_palm";
        marker_array.markers[i].header.stamp = ros::Time::now();
        marker_array.markers[i].ns = "hand_keypoints";
        marker_array.markers[i].id = i;
        marker_array.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
        marker_array.markers[i].action = visualization_msgs::Marker::ADD;
        marker_array.markers[i].scale.x = 0.004;
        marker_array.markers[i].color.a = 1.0;
        if (shadow){
            marker_array.markers[i].color.r = 0.0;
            marker_array.markers[i].color.g = 1.0;
            marker_array.markers[i].color.b = 0.0;
        }else{
            marker_array.markers[i].color.r = 0.0;
            marker_array.markers[i].color.g = 1.0;
            marker_array.markers[i].color.b = 1.0;
        } 
    }

    // Thumb
    marker_array.markers[21].points.resize(5);
    marker_array.markers[21].points[0].x = kp[0].x();
    marker_array.markers[21].points[0].y = kp[0].y();
    marker_array.markers[21].points[0].z = kp[0].z();
    marker_array.markers[21].points[1].x = kp[1].x();
    marker_array.markers[21].points[1].y = kp[1].y();
    marker_array.markers[21].points[1].z = kp[1].z();
    marker_array.markers[21].points[2].x = kp[2].x();
    marker_array.markers[21].points[2].y = kp[2].y();
    marker_array.markers[21].points[2].z = kp[2].z();
    marker_array.markers[21].points[3].x = kp[3].x();
    marker_array.markers[21].points[3].y = kp[3].y();
    marker_array.markers[21].points[3].z = kp[3].z();
    marker_array.markers[21].points[4].x = kp[4].x();
    marker_array.markers[21].points[4].y = kp[4].y();
    marker_array.markers[21].points[4].z = kp[4].z();

    // Forefinger
    marker_array.markers[22].points.resize(5);
    marker_array.markers[22].points[0].x = kp[0].x();
    marker_array.markers[22].points[0].y = kp[0].y();
    marker_array.markers[22].points[0].z = kp[0].z();
    marker_array.markers[22].points[1].x = kp[5].x();
    marker_array.markers[22].points[1].y = kp[5].y();
    marker_array.markers[22].points[1].z = kp[5].z();
    marker_array.markers[22].points[2].x = kp[6].x();
    marker_array.markers[22].points[2].y = kp[6].y();
    marker_array.markers[22].points[2].z = kp[6].z();
    marker_array.markers[22].points[3].x = kp[7].x();
    marker_array.markers[22].points[3].y = kp[7].y();
    marker_array.markers[22].points[3].z = kp[7].z();
    marker_array.markers[22].points[4].x = kp[8].x();
    marker_array.markers[22].points[4].y = kp[8].y();
    marker_array.markers[22].points[4].z = kp[8].z();

    // Middlefinger
    marker_array.markers[23].points.resize(5);
    marker_array.markers[23].points[0].x = kp[0].x();
    marker_array.markers[23].points[0].y = kp[0].y();
    marker_array.markers[23].points[0].z = kp[0].z();
    marker_array.markers[23].points[1].x = kp[9].x();
    marker_array.markers[23].points[1].y = kp[9].y();
    marker_array.markers[23].points[1].z = kp[9].z();
    marker_array.markers[23].points[2].x = kp[10].x();
    marker_array.markers[23].points[2].y = kp[10].y();
    marker_array.markers[23].points[2].z = kp[10].z();
    marker_array.markers[23].points[3].x = kp[11].x();
    marker_array.markers[23].points[3].y = kp[11].y();
    marker_array.markers[23].points[3].z = kp[11].z();
    marker_array.markers[23].points[4].x = kp[12].x();
    marker_array.markers[23].points[4].y = kp[12].y();
    marker_array.markers[23].points[4].z = kp[12].z();

    // Ringfinger
    marker_array.markers[24].points.resize(5);
    marker_array.markers[24].points[0].x = kp[0].x();
    marker_array.markers[24].points[0].y = kp[0].y();
    marker_array.markers[24].points[0].z = kp[0].z();
    marker_array.markers[24].points[1].x = kp[13].x();
    marker_array.markers[24].points[1].y = kp[13].y();
    marker_array.markers[24].points[1].z = kp[13].z();
    marker_array.markers[24].points[2].x = kp[14].x();
    marker_array.markers[24].points[2].y = kp[14].y();
    marker_array.markers[24].points[2].z = kp[14].z();
    marker_array.markers[24].points[3].x = kp[15].x();
    marker_array.markers[24].points[3].y = kp[15].y();
    marker_array.markers[24].points[3].z = kp[15].z();
    marker_array.markers[24].points[4].x = kp[16].x();
    marker_array.markers[24].points[4].y = kp[16].y();
    marker_array.markers[24].points[4].z = kp[16].z();

    // Littlefinger
    marker_array.markers[25].points.resize(5);
    marker_array.markers[25].points[0].x = kp[0].x();
    marker_array.markers[25].points[0].y = kp[0].y();
    marker_array.markers[25].points[0].z = kp[0].z();
    marker_array.markers[25].points[1].x = kp[17].x();
    marker_array.markers[25].points[1].y = kp[17].y();
    marker_array.markers[25].points[1].z = kp[17].z();
    marker_array.markers[25].points[2].x = kp[18].x();
    marker_array.markers[25].points[2].y = kp[18].y();
    marker_array.markers[25].points[2].z = kp[18].z();
    marker_array.markers[25].points[3].x = kp[19].x();
    marker_array.markers[25].points[3].y = kp[19].y();
    marker_array.markers[25].points[3].z = kp[19].z();
    marker_array.markers[25].points[4].x = kp[20].x();
    marker_array.markers[25].points[4].y = kp[20].y();
    marker_array.markers[25].points[4].z = kp[20].z();

    // Publish the MarkerArray message to rviz
    if (shadow)
        marker_pub_shadow.publish(marker_array);
    else
        marker_pub.publish(marker_array);
}

// Receives the keypoints and apply a Kalman Filter
void handKeypointsCB(const human_robot_map::HandKeypoints::ConstPtr& msg)
{
    // Received message info
    ROS_INFO("Received a Hand Keypoints message with %zu keypoints", msg->keypoints.size());

    // Get vector from WRIST to MIDDLE_FINGER
    Eigen::Vector3d wrist_mf_mcp;
    wrist_mf_mcp = point2vector(msg->keypoints[9], msg->keypoints[0]);
    // Get vector from WRIST to RING_FINGER
    Eigen::Vector3d wrist_rf_mcp;
    wrist_rf_mcp = point2vector(msg->keypoints[13], msg->keypoints[0]);
    // New referential axis
    Eigen::Vector3d wrist_X, wrist_Y, wrist_Z;
    wrist_Z = (wrist_mf_mcp + wrist_rf_mcp) / 2.0;
    wrist_Z = wrist_Z.normalized();
    wrist_Y = wrist_rf_mcp.cross(wrist_mf_mcp); 
    wrist_Y = wrist_Y.normalized();
    wrist_X = wrist_Y.cross(wrist_Z);
    wrist_X = wrist_X.normalized();

    // Rotation Matrix
    Eigen::Matrix3d rot;
    rot << wrist_X, wrist_Y, wrist_Z;
    rot.transposeInPlace();

    // Create vector with all keypoint positions
    std::vector<Eigen::Vector3d> keypoints;
    for (int i=0; i<21; i++){
        keypoints.push_back(rot * point2vector(msg->keypoints[i], msg->keypoints[0]));
    }

    // Plot human hand keypoints [RVIZ]
    plotKeypointsRVIZ(keypoints, false);

    // Map from human hand to Shadow Hand
    std::vector<Eigen::Vector3d> shadow_keypoints = mapShadowHand(keypoints);     

    // Plot Shadow Hand keypoints [RVIZ]
    plotKeypointsRVIZ(shadow_keypoints, true);
}

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "plot_keypoints");
    std::cout << "\"plot_keypoints\" ROS node started!" << std::endl;
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(5);
    spinner.start();

    // Create Subscriber
    ros::Subscriber hand_keypoints_sub = nh.subscribe("hand_kp", 1, handKeypointsCB);

    // Create Publisher
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("hand_keypoints_marker", 1);
    marker_pub_shadow = nh.advertise<visualization_msgs::MarkerArray>("shadow_keypoints_marker", 1);

    // Ready 
    std::cout << "\n\033[1;32m\"plot_keypoints\" ROS node is ready!\033[0m\n" << std::endl;

    ros::waitForShutdown(); // because of ros::AsyncSpinner
    //ros::spin();
    return 0;
}