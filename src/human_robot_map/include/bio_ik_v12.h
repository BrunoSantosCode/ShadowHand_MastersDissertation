/* * * * * * * * * * * * bio_ik_v12.h * * * * * * * * * * */
/*  Receives HandKeypoints.msg from "hand_kp" topic       */
/*  Uses BioIK to calculate inverse kinematics (thread1)  */
/*  Send joint angles to Shadow Hand (thread2)            */
/*  Mutex to access joint_angles                          */
/*  Adaptable median filter for keypoint positions        */
/*  Structural reorganization                             */
/*    [kp_pos -> BioIK -> angles -> Shadow Hand]          */
/*  Execute only if different angles                      */
/*  BioIK solve only if different angles keypoints        */
/*  Map human hand into Shadow Hand                       */
/*  Adjustments in hand referential                       */
/*  Sends Shadow Hand commands by SrHandCommander         */
/*  MapShadowHand after median                            */
/*  Redefinition of Goals                                 */
/*  Coupled control of UR5 and Shadow Hand                */
/*  + Collisions avoidance                                */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * *  */

#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/collision_detection/collision_matrix.h>

#include <collision_free_goal.h>
#include <bio_ik/bio_ik.h>

#include <human_robot_map/HandKeypoints.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <algorithm>
#include <vector>
#include <Eigen/Geometry>

#include <thread>
#include <mutex>
#include <chrono>

#include <iostream>
#include <fstream>

#define PI 3.14159265359
#define N_FILTER 5

// GLOBAL VARS
tf2_ros::Buffer tfBuffer, tfBuffer2;
std::string base_frame;
moveit::planning_interface::MoveGroupInterface* mgi_pointer;
const moveit::core::JointModelGroup* joint_model_group;
moveit::core::RobotState* robot_state_pointer;
planning_scene::PlanningScene* planning_scene_pointer;

collision_detection::CollisionRequest collision_request;
collision_detection::CollisionResult collision_result;
std::vector<std::string> collision_pairs;

std::vector<std::vector<Eigen::Vector3d>> kp_positions;
std::vector<Eigen::Vector3d> prev_kp;
std::mutex mutex_kp;

bool exec_thread_started = false;
bool bio_ik_thread_started = false;

// ROS RVIZ Publisher
ros::Publisher marker_pub, marker_pub_shadow, joints_shadow;

// Converts one or two geometry_msgs::Point into Eigen::Vector3d
Eigen::Vector3d point2vector(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2 = geometry_msgs::Point())
{
    Eigen::Vector3d vec1(point1.x, point1.y, point1.z);
    Eigen::Vector3d vec2(point2.x, point2.y, point2.z);

    return vec1 - vec2;
}


// Maps human hand keypoints into Shadow Hand [rh_wrist referential]
std::vector<Eigen::Vector3d> mapShadowHand(std::vector<Eigen::Vector3d> human_kp)
{
    std::vector<Eigen::Vector3d> shadow_kp = human_kp;
    Eigen::Vector3d aux;
    // Thumb
    float metacarpal = std::sqrt(std::pow(29, 2) + std::pow(34, 2));
    shadow_kp[1] = shadow_kp[0] + (human_kp[1]-human_kp[0]).normalized() * metacarpal/1000;
    shadow_kp[2] = shadow_kp[1] + (human_kp[2]-human_kp[1]).normalized() * 38/1000;
    shadow_kp[3] = shadow_kp[2] + (human_kp[3]-human_kp[2]).normalized() * 32/1000;
    shadow_kp[4] = shadow_kp[3] + (human_kp[4]-human_kp[3]).normalized() * 27.5/1000;
    // Forefinger
    metacarpal = std::sqrt(std::pow(95, 2) + std::pow(33, 2));
    shadow_kp[5] = shadow_kp[0] + (human_kp[5]-human_kp[0]).normalized() * metacarpal/1000;
    shadow_kp[6] = shadow_kp[5] + (human_kp[6]-human_kp[5]).normalized() * 45/1000;
    shadow_kp[7] = shadow_kp[6] + (human_kp[7]-human_kp[6]).normalized() * 25/1000;
    shadow_kp[8] = shadow_kp[7] + (human_kp[8]-human_kp[7]).normalized() * 26/1000;
    // Middlefinger
    metacarpal = std::sqrt(std::pow(99, 2) + std::pow(11, 2));
    shadow_kp[9] = shadow_kp[0] + (human_kp[9]-human_kp[0]).normalized() * metacarpal/1000;
    shadow_kp[10] = shadow_kp[9] + (human_kp[10]-human_kp[9]).normalized() * 45/1000;
    shadow_kp[11] = shadow_kp[10] + (human_kp[11]-human_kp[10]).normalized() * 25/1000;
    shadow_kp[12] = shadow_kp[11] + (human_kp[12]-human_kp[11]).normalized() * 26/1000;
    // Ringfinger
    metacarpal = std::sqrt(std::pow(95, 2) + std::pow(11, 2));
    shadow_kp[13] = shadow_kp[0] + (human_kp[13]-human_kp[0]).normalized() * metacarpal/1000;
    shadow_kp[14] = shadow_kp[13] + (human_kp[14]-human_kp[13]).normalized() * 45/1000;
    shadow_kp[15] = shadow_kp[14] + (human_kp[15]-human_kp[14]).normalized() * 25/1000;
    shadow_kp[16] = shadow_kp[15] + (human_kp[16]-human_kp[15]).normalized() * 26/1000;
    // Littlefinger
    metacarpal = std::sqrt(std::pow(86.6, 2) + std::pow(33, 2));
    shadow_kp[17] = shadow_kp[0] + (human_kp[17]-human_kp[0]).normalized() * metacarpal/1000;
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
    marker_array.markers.resize(27);

    // Init dot markers [keypoints]
    for (int i = 0; i < 21; i++)
    {
        marker_array.markers[i].header.frame_id = "world";

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
            marker_array.markers[i].color.g = 1.0;
            marker_array.markers[i].color.b = 1.0;
        }else{
            marker_array.markers[i].color.r = 0.0;
            marker_array.markers[i].color.g = 0.0;
            marker_array.markers[i].color.b = 1.0;
        }

        // Set markers positions
        marker_array.markers[i].pose.position.x = kp[i].x();
        marker_array.markers[i].pose.position.y = kp[i].y();
        marker_array.markers[i].pose.position.z = kp[i].z();
        marker_array.markers[i].pose.orientation.x = 0.0;
        marker_array.markers[i].pose.orientation.y = 0.0;
        marker_array.markers[i].pose.orientation.z = 0.0;
        marker_array.markers[i].pose.orientation.w = 1.0;        
    }

    // Init line markers [lines]
    for (int i = 21; i < 27; i++)
    {
        marker_array.markers[i].header.frame_id = "world";

        marker_array.markers[i].header.stamp = ros::Time::now();
        marker_array.markers[i].ns = "hand_keypoints";
        marker_array.markers[i].id = i;
        marker_array.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
        marker_array.markers[i].action = visualization_msgs::Marker::ADD;
        marker_array.markers[i].scale.x = 0.004;
        marker_array.markers[i].color.a = 1.0;
        if (shadow){
            marker_array.markers[i].color.r = 0.1;
            marker_array.markers[i].color.g = 0.1;
            marker_array.markers[i].color.b = 0.1;
        }else{
            marker_array.markers[i].color.r = 0.0;
            marker_array.markers[i].color.g = 1.0;
            marker_array.markers[i].color.b = 1.0;
        } 
        marker_array.markers[i].pose.orientation.x = 0.0;
        marker_array.markers[i].pose.orientation.y = 0.0;
        marker_array.markers[i].pose.orientation.z = 0.0;
        marker_array.markers[i].pose.orientation.w = 1.0;        
    }

    // Wrist
    marker_array.markers[21].points.resize(2);
    marker_array.markers[21].points[0].x = kp[0].x();
    marker_array.markers[21].points[0].y = kp[0].y();
    marker_array.markers[21].points[0].z = kp[0].z();
    marker_array.markers[21].points[1].x = kp[21].x();
    marker_array.markers[21].points[1].y = kp[21].y();
    marker_array.markers[21].points[1].z = kp[21].z();

    // Thumb
    marker_array.markers[22].points.resize(5);
    marker_array.markers[22].points[0].x = kp[0].x();
    marker_array.markers[22].points[0].y = kp[0].y();
    marker_array.markers[22].points[0].z = kp[0].z();
    marker_array.markers[22].points[1].x = kp[1].x();
    marker_array.markers[22].points[1].y = kp[1].y();
    marker_array.markers[22].points[1].z = kp[1].z();
    marker_array.markers[22].points[2].x = kp[2].x();
    marker_array.markers[22].points[2].y = kp[2].y();
    marker_array.markers[22].points[2].z = kp[2].z();
    marker_array.markers[22].points[3].x = kp[3].x();
    marker_array.markers[22].points[3].y = kp[3].y();
    marker_array.markers[22].points[3].z = kp[3].z();
    marker_array.markers[22].points[4].x = kp[4].x();
    marker_array.markers[22].points[4].y = kp[4].y();
    marker_array.markers[22].points[4].z = kp[4].z();

    // Forefinger
    marker_array.markers[23].points.resize(5);
    marker_array.markers[23].points[0].x = kp[0].x();
    marker_array.markers[23].points[0].y = kp[0].y();
    marker_array.markers[23].points[0].z = kp[0].z();
    marker_array.markers[23].points[1].x = kp[5].x();
    marker_array.markers[23].points[1].y = kp[5].y();
    marker_array.markers[23].points[1].z = kp[5].z();
    marker_array.markers[23].points[2].x = kp[6].x();
    marker_array.markers[23].points[2].y = kp[6].y();
    marker_array.markers[23].points[2].z = kp[6].z();
    marker_array.markers[23].points[3].x = kp[7].x();
    marker_array.markers[23].points[3].y = kp[7].y();
    marker_array.markers[23].points[3].z = kp[7].z();
    marker_array.markers[23].points[4].x = kp[8].x();
    marker_array.markers[23].points[4].y = kp[8].y();
    marker_array.markers[23].points[4].z = kp[8].z();

    // Middlefinger
    marker_array.markers[24].points.resize(5);
    marker_array.markers[24].points[0].x = kp[0].x();
    marker_array.markers[24].points[0].y = kp[0].y();
    marker_array.markers[24].points[0].z = kp[0].z();
    marker_array.markers[24].points[1].x = kp[9].x();
    marker_array.markers[24].points[1].y = kp[9].y();
    marker_array.markers[24].points[1].z = kp[9].z();
    marker_array.markers[24].points[2].x = kp[10].x();
    marker_array.markers[24].points[2].y = kp[10].y();
    marker_array.markers[24].points[2].z = kp[10].z();
    marker_array.markers[24].points[3].x = kp[11].x();
    marker_array.markers[24].points[3].y = kp[11].y();
    marker_array.markers[24].points[3].z = kp[11].z();
    marker_array.markers[24].points[4].x = kp[12].x();
    marker_array.markers[24].points[4].y = kp[12].y();
    marker_array.markers[24].points[4].z = kp[12].z();

    // Ringfinger
    marker_array.markers[25].points.resize(5);
    marker_array.markers[25].points[0].x = kp[0].x();
    marker_array.markers[25].points[0].y = kp[0].y();
    marker_array.markers[25].points[0].z = kp[0].z();
    marker_array.markers[25].points[1].x = kp[13].x();
    marker_array.markers[25].points[1].y = kp[13].y();
    marker_array.markers[25].points[1].z = kp[13].z();
    marker_array.markers[25].points[2].x = kp[14].x();
    marker_array.markers[25].points[2].y = kp[14].y();
    marker_array.markers[25].points[2].z = kp[14].z();
    marker_array.markers[25].points[3].x = kp[15].x();
    marker_array.markers[25].points[3].y = kp[15].y();
    marker_array.markers[25].points[3].z = kp[15].z();
    marker_array.markers[25].points[4].x = kp[16].x();
    marker_array.markers[25].points[4].y = kp[16].y();
    marker_array.markers[25].points[4].z = kp[16].z();

    // Littlefinger
    marker_array.markers[26].points.resize(5);
    marker_array.markers[26].points[0].x = kp[0].x();
    marker_array.markers[26].points[0].y = kp[0].y();
    marker_array.markers[26].points[0].z = kp[0].z();
    marker_array.markers[26].points[1].x = kp[17].x();
    marker_array.markers[26].points[1].y = kp[17].y();
    marker_array.markers[26].points[1].z = kp[17].z();
    marker_array.markers[26].points[2].x = kp[18].x();
    marker_array.markers[26].points[2].y = kp[18].y();
    marker_array.markers[26].points[2].z = kp[18].z();
    marker_array.markers[26].points[3].x = kp[19].x();
    marker_array.markers[26].points[3].y = kp[19].y();
    marker_array.markers[26].points[3].z = kp[19].z();
    marker_array.markers[26].points[4].x = kp[20].x();
    marker_array.markers[26].points[4].y = kp[20].y();
    marker_array.markers[26].points[4].z = kp[20].z();


    // Publish the MarkerArray message to rviz
    if (shadow)
        marker_pub_shadow.publish(marker_array);
    else
        marker_pub.publish(marker_array);
}