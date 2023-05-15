/* * * * * * * * * * * bio_ik_v2 * * * * * * * * * * */
/*  Receives HandKeypoints.msg from "hand_kp" topic  */
/*  Uses BioIK to calculate inverse kinematics       */
/*  Send joint angles to Shadow Hand (thread)        */
/*  Mutex to access joint_values                     */
/*  Adaptable average filter [N_AVERAGE]             */
/*  + Link orientation constraints only              */
/*    Note: it doesn't work properly...              */
/* * * * * * * * * * * * * * * * * * * * * * * * * * */

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
#include <geometry_msgs/Point.h>
#include <vector>
#include <Eigen/Geometry>

#include <thread>
#include <mutex>

#define PI 3.14159265359
#define N_AVERAGE 5

// GLOBAL VARS
tf2_ros::Buffer tfBuffer;
std::string base_frame;
moveit::planning_interface::MoveGroupInterface* mgi_pointer;
const moveit::core::JointModelGroup* joint_model_group;
planning_scene::PlanningScene* planning_scene_pointer;

std::vector<std::vector<double>> joint_values;
std::mutex mutex_joints;

bool can_execute = false;



// Converts one or two geometry_msgs::Point into Eigen::Vector3d
Eigen::Vector3d point2vector(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2 = geometry_msgs::Point())
{
    Eigen::Vector3d vec1(point1.x, point1.y, point1.z);
    Eigen::Vector3d vec2(point2.x, point2.y, point2.z);

    return vec1 - vec2;
}

// Thread to execute Shadow Hand movements
void execute_ShadowHand()
{   
    while(true){
        // Start timer
        ros::Time startTime, endTime;
        ros::Duration time;
        startTime = ros::Time::now();

        // Get joints mean [Average Filter]
        std::vector<double> joint_mean;
        mutex_joints.lock();
            for (int i = 0; i < joint_values[0].size(); i++) {
                double sum = 0;
                for (int j = 0; j < joint_values.size(); j++) {
                    sum += joint_values[j][i];
                }
                double mean = sum / joint_values.size();
                joint_mean.push_back(mean);
            }
        mutex_joints.unlock();

        // Set wrist joints to 0
        joint_mean[0] = 0;
        joint_mean[1] = 0;
        (*mgi_pointer).setJointValueTarget(joint_mean);

        // Execute movement
        (*mgi_pointer).move();

        // Calculate execution duration
        endTime = ros::Time::now();
        time = endTime - startTime;
        std::cout << "Execution time: " << time.toSec() << " seconds!" << std::endl;
    }
}

// Receives the keypoints and translate them into Shadow Hand joint values
void handKeypointsCB(const human_robot_map::HandKeypoints::ConstPtr& msg)
{
    // Received message info
    ROS_INFO("Received a Hand Keypoints message with %zu keypoints", msg->keypoints.size());

    // Start timer
    ros::Time startTime, endTime;
    ros::Duration time;
    startTime = ros::Time::now();

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

    // DEBUG
    if (false) {
        std::cout << "X axis = (" << wrist_X.x() << "," << wrist_X.y() << "," << wrist_X.z() << ")" << std::endl;
        std::cout << "Y axis = (" << wrist_Y.x() << "," << wrist_Y.y() << "," << wrist_Y.z() << ")" << std::endl;
        std::cout << "Z axis = (" << wrist_Z.x() << "," << wrist_Z.y() << "," << wrist_Z.z() << ")" << std::endl;
        std::cout << "Rotation matrix:" << std::endl;
        std::cout << rot << std::endl;
    }
    
    /* * * * [EXAMPLE] * Convert to new referential * [EXAMPLE] * * * */
    // Eigen::Vector3d th_tip;                                        //
    // th_tip = point2vector(msg->keypoints[4], msg->keypoints[0]);   //
    // Eigen::Vector3d new_th_tip = rot * th_tip;                     //
    ////////////////////////////////////////////////////////////////////

    // Get target joints orientation
    std::vector<Eigen::Vector3d> linkDir(20);
    linkDir[0]  = rot * point2vector(msg->keypoints[1],  msg->keypoints[0]);   // th_knuckle
    linkDir[1]  = rot * point2vector(msg->keypoints[5],  msg->keypoints[0]);   // ff_knuckle
    linkDir[2]  = rot * point2vector(msg->keypoints[9],  msg->keypoints[0]);   // mf_knuckle
    linkDir[3]  = rot * point2vector(msg->keypoints[13], msg->keypoints[0]);   // rf_knuckle 
    linkDir[4]  = rot * point2vector(msg->keypoints[17], msg->keypoints[0]);   // lf_knuckle
    linkDir[5]  = rot * point2vector(msg->keypoints[2],  msg->keypoints[1]);   // th_proximal
    linkDir[6]  = rot * point2vector(msg->keypoints[6],  msg->keypoints[5]);   // ff_proximal
    linkDir[7]  = rot * point2vector(msg->keypoints[10], msg->keypoints[9]);   // mf_proximal
    linkDir[8]  = rot * point2vector(msg->keypoints[14], msg->keypoints[13]);  // rf_proximal 
    linkDir[9]  = rot * point2vector(msg->keypoints[18], msg->keypoints[17]);  // lf_proximal
    linkDir[10] = rot * point2vector(msg->keypoints[3],  msg->keypoints[2]);   // th_middle
    linkDir[11] = rot * point2vector(msg->keypoints[7],  msg->keypoints[6]);   // ff_middle
    linkDir[12] = rot * point2vector(msg->keypoints[11], msg->keypoints[10]);  // mf_middle
    linkDir[13] = rot * point2vector(msg->keypoints[15], msg->keypoints[14]);  // rf_middle 
    linkDir[14] = rot * point2vector(msg->keypoints[19], msg->keypoints[18]);  // lf_middle
    linkDir[15] = rot * point2vector(msg->keypoints[4],  msg->keypoints[3]);   // th_distal
    linkDir[16] = rot * point2vector(msg->keypoints[8],  msg->keypoints[7]);   // ff_distal
    linkDir[17] = rot * point2vector(msg->keypoints[12], msg->keypoints[11]);  // mf_distal
    linkDir[18] = rot * point2vector(msg->keypoints[16], msg->keypoints[15]);  // rf_distal 
    linkDir[19] = rot * point2vector(msg->keypoints[20], msg->keypoints[19]);  // lf_distal

    // DEBUG
    if (false) {
        std::cout << "TH_KNUCKLE  = (" << linkDir[0].x() << "," << linkDir[0].y() << "," << linkDir[0].z() << ")" << std::endl;
        std::cout << "FF_KNUCKLE  = (" << linkDir[1].x() << "," << linkDir[1].y() << "," << linkDir[1].z() << ")" << std::endl;
        std::cout << "MF_KNUCKLE  = (" << linkDir[2].x() << "," << linkDir[2].y() << "," << linkDir[2].z() << ")" << std::endl;
        std::cout << "RF_KNUCKLE  = (" << linkDir[3].x() << "," << linkDir[3].y() << "," << linkDir[3].z() << ")" << std::endl;
        std::cout << "LF_KNUCKLE  = (" << linkDir[4].x() << "," << linkDir[4].y() << "," << linkDir[4].z() << ")" << std::endl;
        std::cout << "TH_PROXIMAL = (" << linkDir[0].x() << "," << linkDir[0].y() << "," << linkDir[0].z() << ")" << std::endl;
        std::cout << "FF_PROXIMAL = (" << linkDir[1].x() << "," << linkDir[1].y() << "," << linkDir[1].z() << ")" << std::endl;
        std::cout << "MF_PROXIMAL = (" << linkDir[2].x() << "," << linkDir[2].y() << "," << linkDir[2].z() << ")" << std::endl;
        std::cout << "RF_PROXIMAL = (" << linkDir[3].x() << "," << linkDir[3].y() << "," << linkDir[3].z() << ")" << std::endl;
        std::cout << "LF_PROXIMAL = (" << linkDir[4].x() << "," << linkDir[4].y() << "," << linkDir[4].z() << ")" << std::endl;
        std::cout << "TH_MIDDLE   = (" << linkDir[0].x() << "," << linkDir[0].y() << "," << linkDir[0].z() << ")" << std::endl;
        std::cout << "FF_MIDDLE   = (" << linkDir[1].x() << "," << linkDir[1].y() << "," << linkDir[1].z() << ")" << std::endl;
        std::cout << "MF_MIDDLE   = (" << linkDir[2].x() << "," << linkDir[2].y() << "," << linkDir[2].z() << ")" << std::endl;
        std::cout << "RF_MIDDLE   = (" << linkDir[3].x() << "," << linkDir[3].y() << "," << linkDir[3].z() << ")" << std::endl;
        std::cout << "LF_MIDDLE   = (" << linkDir[4].x() << "," << linkDir[4].y() << "," << linkDir[4].z() << ")" << std::endl;
        std::cout << "TH_DISTAL   = (" << linkDir[0].x() << "," << linkDir[0].y() << "," << linkDir[0].z() << ")" << std::endl;
        std::cout << "FF_DISTAL   = (" << linkDir[1].x() << "," << linkDir[1].y() << "," << linkDir[1].z() << ")" << std::endl;
        std::cout << "MF_DISTAL   = (" << linkDir[2].x() << "," << linkDir[2].y() << "," << linkDir[2].z() << ")" << std::endl;
        std::cout << "RF_DISTAL   = (" << linkDir[3].x() << "," << linkDir[3].y() << "," << linkDir[3].z() << ")" << std::endl;
        std::cout << "LF_DISTAL   = (" << linkDir[4].x() << "," << linkDir[4].y() << "," << linkDir[4].z() << ")" << std::endl;
    }

    // BioIK conditions set
    double timeout = 0.2;
    std::vector<std::string> MapDirectionlinks {"rh_thbase",     "rh_ffknuckle",  "rh_mfknuckle",  "rh_rfknuckle",  "rh_lfknuckle",
                                                "rh_thproximal", "rh_ffproximal", "rh_mfproximal", "rh_rfproximal", "rh_lfproximal",
                                                "rh_thmiddle",   "rh_ffmiddle",   "rh_mfmiddle",   "rh_rfmiddle",   "rh_lfmiddle",
                                                "rh_thdistal",   "rh_ffdistal",   "rh_mfdistal",   "rh_rfdistal",   "rh_lfdistal"};

    // BioIK condition weights
    std::vector <float> MapDirectionweights{0.5, 0.5, 0.5, 0.5, 0.5,
                                            1.0, 1.0, 1.0, 1.0, 1.0,
                                            1.0, 1.0, 1.0, 1.0, 1.0,
                                            1.0, 1.0, 1.0, 1.0, 1.0};

    // BioIK goals
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.replace = true;
    ik_options.return_approximate_solution = true;

    // Set orientation constraints
    for (int i=0; i<MapDirectionlinks.size(); i++)
    {
        // Transform orientation from rh_wrist into base_frame
        geometry_msgs::PointStamped stamped_in;
        stamped_in.header.frame_id = "rh_wrist";
        stamped_in.point.x = linkDir[i].x();
        stamped_in.point.y = linkDir[i].y();
        stamped_in.point.z = linkDir[i].z();

        geometry_msgs::PointStamped stamped_out;
        tfBuffer.transform(stamped_in, stamped_out, base_frame);
        tf2::Vector3 Mapdirection (stamped_out.point.x, stamped_out.point.y, stamped_out.point.z);

        ik_options.goals.emplace_back(new bio_ik::DirectionGoal(MapDirectionlinks[i], tf2::Vector3(0,0,1), Mapdirection.normalized(), MapDirectionweights[i]));
    }
    // Set non-linear Shadow Hand joint coupling constraints
    std::vector<std::string> ff_coupled_joints, mf_coupled_joints, rf_coupled_joints, lf_coupled_joints;
    float couple_constr_weight = 1.0;
    ff_coupled_joints.push_back("rh_FFJ1");
    ff_coupled_joints.push_back("rh_FFJ2");
    mf_coupled_joints.push_back("rh_MFJ1");
    mf_coupled_joints.push_back("rh_MFJ2");
    rf_coupled_joints.push_back("rh_RFJ1");
    rf_coupled_joints.push_back("rh_RFJ2");
    lf_coupled_joints.push_back("rh_LFJ1");
    lf_coupled_joints.push_back("rh_LFJ2");
    auto* ff_goal = new bio_ik::JointFunctionGoal(
                        ff_coupled_joints,
                        [=] (std::vector<double>& vv) {
                            vv[0] = fmax(0, vv[1]-PI);  // if J2<90º => J1=0
                            vv[1] = fmin(PI/2, vv[1]);  // max(J2) = 90º       
                        },  couple_constr_weight 
                    );
    auto* mf_goal = new bio_ik::JointFunctionGoal(
                        mf_coupled_joints,
                        [=] (std::vector<double>& vv) {
                            vv[0] = fmax(0, vv[1]-PI);  // if J2<90º => J1=0
                            vv[1] = fmin(PI/2, vv[1]);  // max(J2) = 90º       
                        },  couple_constr_weight 
                    );
    auto* rf_goal = new bio_ik::JointFunctionGoal(
                        rf_coupled_joints,
                        [=] (std::vector<double>& vv) {
                            vv[0] = fmax(0, vv[1]-PI);  // if J2<90º => J1=0
                            vv[1] = fmin(PI/2, vv[1]);  // max(J2) = 90º       
                        },  couple_constr_weight  
                    );
    auto* lf_goal = new bio_ik::JointFunctionGoal(
                        lf_coupled_joints,
                        [=] (std::vector<double>& vv) {
                            vv[0] = fmax(0, vv[1]-PI);  // if J2<90º => J1=0
                            vv[1] = fmin(PI/2, vv[1]);  // max(J2) = 90º       
                        },  couple_constr_weight  
                    );
    ik_options.goals.emplace_back(ff_goal);
    ik_options.goals.emplace_back(mf_goal);
    ik_options.goals.emplace_back(rf_goal);
    ik_options.goals.emplace_back(lf_goal);

    // Get current robot state
    robot_state::RobotState& current_state = (*planning_scene_pointer).getCurrentStateNonConst();   

    // Set BioIK solver
    bool found_ik = current_state.setFromIK(
                        joint_model_group,             // Shadow Hand joints
                        EigenSTL::vector_Isometry3d(), // no explicit poses here
                        std::vector<std::string>(),
                        timeout,
                        moveit::core::GroupStateValidityCallbackFn(),
                        ik_options
                    );

    // DEBUG
    if (false) {
        std::cout << "BioIK found solutions: " << found_ik << std::endl;
    }
    
    // Get Shadow Hand calculated joints
    if (found_ik){
        std::vector<double> joints_aux;
        current_state.copyJointGroupPositions(joint_model_group, joints_aux);
        mutex_joints.lock();
            joint_values.insert(joint_values.begin(), joints_aux);
            joint_values.pop_back();
        mutex_joints.unlock();
        // Start execution thread
        if (can_execute == false){
            std::thread thread1(execute_ShadowHand);
            thread1.detach();
            can_execute = true;
        }
    }
    else
        std::cout << "Did not find IK solution" << std::endl;

    // Reset planning goals
    for (int j = 0; j <ik_options.goals.size();j++)
        ik_options.goals[j].reset();

    // Calculate BioIK solver duration
    endTime = ros::Time::now();
    time = endTime - startTime;
    std::cout << "BioIK solver duration: " << time.toSec() << " seconds!" << std::endl;

}

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "bio_ik_v2");
    std::cout << "\"bio_ik_v2\" ROS node started!" << std::endl;
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(5);
    spinner.start();

    // ROS Transform
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Moveit
    std::string group_name = "right_hand";
    moveit::planning_interface::MoveGroupInterface mgi(group_name);
    base_frame = mgi.getPoseReferenceFrame();

    // Set MoveGroupInterface parameters
    mgi.setGoalTolerance(0.01);
    mgi.setPlanningTime(1);
    mgi.setMaxVelocityScalingFactor(0.5);
    mgi.setMaxAccelerationScalingFactor(0.5);
    mgi.setPlannerId("RRTConnectkConfigDefault");
    auto robot_model = mgi.getCurrentState()->getRobotModel();
    joint_model_group = robot_model->getJointModelGroup(group_name);
    moveit::core::RobotState robot_state(robot_model);
    planning_scene::PlanningScene planning_scene(robot_model);

    // Define pointers to access vars in callback
    mgi_pointer = &mgi;
    planning_scene_pointer = &planning_scene;

    // Joints first state
    std::vector<double> empty_joints{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    for (int i=0; i<N_AVERAGE; i++)
        joint_values.push_back(empty_joints);

    // Create Subscriber
    ros::Subscriber hand_keypoints_sub = nh.subscribe("hand_kp", 1, handKeypointsCB);

    // Ready 
    std::cout << "\n\033[1;32m\"bio_ik_v2\" ROS node is ready!\033[0m\n" << std::endl;

    ros::waitForShutdown(); // because of ros::AsyncSpinner
    //ros::spin();
    return 0;
}