/* * * * * * * * * * bio_ik_v4.cpp * * * * * * * * * * * */
/*  Receives HandKeypoints.msg from "hand_kp" topic      */
/*  Uses BioIK to calculate inverse kinematics (thread1) */
/*  Send joint angles to Shadow Hand (thread2)           */
/*  Mutex to access joint_angles                         */
/*  Adaptable median filter for keypoint positions       */
/*  Structural reorganization                            */
/*    [kp_pos -> BioIK -> angles -> Shadow Hand]         */
/*  Execute only if different angles                     */
/*  BioIK solve only if different angles keypoints       */
/*  + Map human hand into Shadow Hand                    */
/*  + Adjustments in hand referential                    */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <bio_ik_v4.h>


// Thread to execute Shadow Hand movements
void execute_ShadowHand()
{   
    while(ros::ok()){
        // Start timer
        ros::Time startTime, endTime;
        ros::Duration time;
        startTime = ros::Time::now();

        mutex_joints.lock();
            // Set wrist joints to 0
            joint_angles[0] = 0;
            joint_angles[1] = 0;
            // If same angles ignore
            if (prev_joints == joint_angles){
                mutex_joints.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            (*mgi_pointer).setJointValueTarget(joint_angles);
            prev_joints = joint_angles;
        mutex_joints.unlock();

        // Execute movement
        (*mgi_pointer).move();
        
        std::cout << "\n\033[1;32m\"Command sent to Shadow Hand!\033[0m\n" << std::endl;

        // DEBUG
        if (false){
            std::cout << "\n\033[1;32m\"Command: \033[0m\n" << std::endl;
            for (auto angle : joint_angles){
                std::cout << angle << " ";
            }
        }

        // Calculate execution duration
        endTime = ros::Time::now();
        time = endTime - startTime;
        std::cout << "Execution time: " << time.toSec() << " seconds!" << std::endl;
    }
}


// Thread to compute inverse kinematics [BioIK]
void bio_ik_solver()
{   
    while(ros::ok()){
        // Start timer
        ros::Time startTime, endTime;
        ros::Duration time;
        startTime = ros::Time::now();

        // Get median keypoints position [Median Filter]
        std::vector<Eigen::Vector3d> mean_kp;
        mutex_kp.lock();
            for (int i=0; i<kp_positions[0].size(); i++){
                std::vector<double> x_aux, y_aux, z_aux;
                for (int j=0; j<kp_positions.size(); j++){
                    x_aux.push_back(kp_positions[j][i].x());
                    y_aux.push_back(kp_positions[j][i].y());
                    z_aux.push_back(kp_positions[j][i].z());
                } 
                std::vector<double>::iterator middle_x = x_aux.begin() + x_aux.size()/2;
                std::vector<double>::iterator middle_y = y_aux.begin() + y_aux.size()/2;
                std::vector<double>::iterator middle_z = z_aux.begin() + z_aux.size()/2;
                std::nth_element(x_aux.begin(), middle_x, x_aux.end());
                std::nth_element(y_aux.begin(), middle_y, y_aux.end());
                std::nth_element(z_aux.begin(), middle_z, z_aux.end());
                Eigen::Vector3d result;
                result.x() = *middle_x;
                result.y() = *middle_y;
                result.z() = *middle_z;
                mean_kp.push_back(result);              
            }
        mutex_kp.unlock();

        // Plot Shadow Hand keypoints [RVIZ]
        plotKeypointsRVIZ(mean_kp, true);

        // DEBUG
        if (false){
            std::cout << "Full vector:" << std::endl;
            for (const auto& row : kp_positions) {
                for (const auto& point : row) {
                    std::cout << point.transpose() << " ";
                }
                std::cout << std::endl;
            }
            std::cout << "Mean vector:" << std::endl;
            for (const auto& point : mean_kp) {
                std::cout << point.transpose() << " ";
            }
        }

        // Ignore if same keypoints position
        if (prev_kp == mean_kp)
            continue;

        prev_kp = mean_kp;

        // Get target joints position/orientation
        std::vector<Eigen::Vector3d> linkPos(10);
        std::vector<Eigen::Vector3d> linkDir(6);
        mutex_kp.lock();
            linkPos[0] = mean_kp[4];   // th_tip
            linkPos[1] = mean_kp[8];   // ff_tip
            linkPos[2] = mean_kp[12];  // mf_tip 
            linkPos[3] = mean_kp[16];  // rf_tip
            linkPos[4] = mean_kp[20];  // lf_tip
            linkPos[5] = mean_kp[2];   // th_middle 
            linkPos[6] = mean_kp[6];   // ff_middle
            linkPos[7] = mean_kp[10];  // mf_middle
            linkPos[8] = mean_kp[14];  // rf_middle
            linkPos[9] = mean_kp[18];  // lf_middle
            linkDir[0] = mean_kp[2]  - mean_kp[1];   // th_proximal
            linkDir[1] = mean_kp[6]  - mean_kp[5];   // ff_proximal
            linkDir[2] = mean_kp[10] - mean_kp[9];   // mf_proximal
            linkDir[3] = mean_kp[14] - mean_kp[13];  // rf_proximal 
            linkDir[4] = mean_kp[18] - mean_kp[17];  // lf_proximal
            linkDir[5] = mean_kp[3]  - mean_kp[2];   // th_middle
        mutex_kp.unlock();

        // DEBUG
        if (false) {
            std::cout << "TH_TIP = ("    << linkPos[0].x() << "," << linkPos[0].y() << "," << linkPos[0].z() << ")" << std::endl;
            std::cout << "FF_TIP = ("    << linkPos[1].x() << "," << linkPos[1].y() << "," << linkPos[1].z() << ")" << std::endl;
            std::cout << "MF_TIP = ("    << linkPos[2].x() << "," << linkPos[2].y() << "," << linkPos[2].z() << ")" << std::endl;
            std::cout << "RF_TIP = ("    << linkPos[3].x() << "," << linkPos[3].y() << "," << linkPos[3].z() << ")" << std::endl;
            std::cout << "LF_TIP = ("    << linkPos[4].x() << "," << linkPos[4].y() << "," << linkPos[4].z() << ")" << std::endl;
            std::cout << "TH_MIDDLE = (" << linkPos[5].x() << "," << linkPos[5].y() << "," << linkPos[5].z() << ")" << std::endl;
            std::cout << "FF_MIDDLE = (" << linkPos[6].x() << "," << linkPos[6].y() << "," << linkPos[6].z() << ")" << std::endl;
            std::cout << "MF_MIDDLE = (" << linkPos[7].x() << "," << linkPos[7].y() << "," << linkPos[7].z() << ")" << std::endl;
            std::cout << "RF_MIDDLE = (" << linkPos[8].x() << "," << linkPos[8].y() << "," << linkPos[8].z() << ")" << std::endl;
            std::cout << "LF_MIDDLE = (" << linkPos[9].x() << "," << linkPos[9].y() << "," << linkPos[9].z() << ")" << std::endl;
        }
        if (false) {
            std::cout << "TH_PROXIMAL = (" << linkDir[0].x() << "," << linkDir[0].y() << "," << linkDir[0].z() << ")" << std::endl;
            std::cout << "FF_PROXIMAL = (" << linkDir[1].x() << "," << linkDir[1].y() << "," << linkDir[1].z() << ")" << std::endl;
            std::cout << "MF_PROXIMAL = (" << linkDir[2].x() << "," << linkDir[2].y() << "," << linkDir[2].z() << ")" << std::endl;
            std::cout << "RF_PROXIMAL = (" << linkDir[3].x() << "," << linkDir[3].y() << "," << linkDir[3].z() << ")" << std::endl;
            std::cout << "LF_PROXIMAL = (" << linkDir[4].x() << "," << linkDir[4].y() << "," << linkDir[4].z() << ")" << std::endl;
            std::cout << "TH_MIDDLE = ("   << linkDir[5].x() << "," << linkDir[5].y() << "," << linkDir[5].z() << ")" << std::endl;
        }

        // BioIK conditions set
        double timeout = 0.2;
        std::vector<std::string> MapPositionlinks {
        "rh_thtip",
        "rh_fftip",
        "rh_mftip",
        "rh_rftip",
        "rh_lftip",
        "rh_thmiddle",
        "rh_ffmiddle",
        "rh_mfmiddle",
        "rh_rfmiddle",
        "rh_lfmiddle"
        };
        std::vector<std::string> MapDirectionlinks {
        "rh_thproximal",
        "rh_ffproximal",
        "rh_mfproximal",
        "rh_rfproximal",
        "rh_lfproximal",
        "rh_thmiddle"
        };


        // BioIK condition weights
        std::vector <float> MapPositionweights {1,1,1,1,1,0.5,0.5,0.5,0.5,0.5};
        std::vector <float> MapDirectionweights{0.5,0.5,0.5,0.5,0.5,0.5};

        // BioIK goals
        bio_ik::BioIKKinematicsQueryOptions ik_options;
        ik_options.replace = true;
        ik_options.return_approximate_solution = true;

        // Set position constraints
        for (int i=0; i<MapPositionlinks.size(); i++)
        {
            // Transform position from rh_wrist into base_frame
            geometry_msgs::PointStamped stamped_in;
            stamped_in.header.frame_id = "rh_palm";
            stamped_in.point.x = linkPos[i].x();
            stamped_in.point.y = linkPos[i].y();
            stamped_in.point.z = linkPos[i].z();
            
            geometry_msgs::PointStamped stamped_out ;
            tfBuffer.transform(stamped_in, stamped_out, base_frame);
            tf2::Vector3 Mapposition (stamped_out.point.x, stamped_out.point.y, stamped_out.point.z);

            ik_options.goals.emplace_back(new bio_ik::PositionGoal(MapPositionlinks[i], Mapposition, MapPositionweights[i]));
        }
        // Set orientation constraints
        for (int i=0; i<MapDirectionlinks.size(); i++)
        {
            // Transform orientation from rh_wrist into base_frame
            geometry_msgs::PointStamped stamped_in;
            stamped_in.header.frame_id = "rh_palm";
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
            mutex_joints.lock();
                current_state.copyJointGroupPositions(joint_model_group, joint_angles);
            mutex_joints.unlock();
            // Start execution thread
            if (exec_thread_started == false){
                std::thread thread2(execute_ShadowHand);
                thread2.detach();
                exec_thread_started = true;
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
}


// Receives the keypoints and convert them to hand referential
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
    
    /* * * * * * EXAMPLE: Convert points to hand referential * * * * * */
    /* Eigen::Vector3d th_tip;                                         */
    /* th_tip = point2vector(msg->keypoints[4], msg->keypoints[0]);    */
    /* Eigen::Vector3d new_th_tip = rot * th_tip;                      */
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    // Create vector with all keypoint positions
    std::vector<Eigen::Vector3d> keypoints;
    for (int i=0; i<21; i++){
        keypoints.push_back(rot * point2vector(msg->keypoints[i], msg->keypoints[0]));
    }

    // Plot human hand keypoints [RVIZ]
    plotKeypointsRVIZ(keypoints, false);

    // Map from human hand to Shadow Hand
    std::vector<Eigen::Vector3d> shadow_keypoints = mapShadowHand(keypoints);     

    mutex_kp.lock();
        kp_positions.insert(kp_positions.begin(), shadow_keypoints);
        kp_positions.pop_back();
    mutex_kp.unlock();

    // DEBUG
    if (false){
        std::cout << "Keypoints upgraded!" << std::endl;
    }

    if (bio_ik_thread_started == false){
        std::thread thread1(bio_ik_solver);
        thread1.detach();
        bio_ik_thread_started = true;
    }
    
}

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "bio_ik_v4");
    std::cout << "\"bio_ik_v4\" ROS node started!" << std::endl;
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
    mgi.setPlannerId("RRTConnectkConfigDefault");
    auto robot_model = mgi.getCurrentState()->getRobotModel();
    joint_model_group = robot_model->getJointModelGroup(group_name);
    moveit::core::RobotState robot_state(robot_model);
    planning_scene::PlanningScene planning_scene(robot_model);

    // Set control velocity and acceleration
    mgi.setMaxAccelerationScalingFactor(1.0);
    mgi.setMaxVelocityScalingFactor(1.0);

    // Define pointers to access vars in callback
    mgi_pointer = &mgi;
    planning_scene_pointer = &planning_scene;

    // Init prev_kp and kp_positions
    Eigen::Vector3d empty_pos(0,0,0);
    std::vector<Eigen::Vector3d> empty_hand;
    for (int i=0; i<21; i++)
        empty_hand.push_back(empty_pos);
    for (int i=0; i<N_FILTER; i++)
        kp_positions.push_back(empty_hand);
    for (int i=0; i<21; i++)
        prev_kp.push_back(empty_pos);
    

    // Create Subscriber
    ros::Subscriber hand_keypoints_sub = nh.subscribe("hand_kp", 1, handKeypointsCB);

    // Create Publisher
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("hand_keypoints_marker", 1);
    marker_pub_shadow = nh.advertise<visualization_msgs::MarkerArray>("shadow_keypoints_marker", 1);

    // Ready 
    std::cout << "\n\033[1;32m\"bio_ik_v4\" ROS node is ready!\033[0m\n" << std::endl;

    ros::waitForShutdown(); // because of ros::AsyncSpinner
    //ros::spin();
    return 0;
}