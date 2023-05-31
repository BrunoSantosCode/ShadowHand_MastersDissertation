/* * * * * * * * * * * bio_ik_v10.cpp * * * * * * * * * * */
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
/*  + Coupled control of UR5 and Shadow Hand              */
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
planning_scene::PlanningScene* planning_scene_pointer;

std::vector<std::vector<Eigen::Vector3d>> kp_positions;
std::vector<Eigen::Vector3d> prev_kp;
std::mutex mutex_kp;

bool exec_thread_started = false;
bool bio_ik_thread_started = false;

// ROS RVIZ Publisher
ros::Publisher joints_shadow;


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
    shadow_kp[12] = shadow_kp[11] + (human_kp[12]-human_kp[11]).normalized() * 26/1000;
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

    for (int i=0; i<shadow_kp.size(); i++)
        shadow_kp[i].z() = shadow_kp[i].z() + 34.0/1000; // rh_palm -> wr_wrist

    return shadow_kp;
}