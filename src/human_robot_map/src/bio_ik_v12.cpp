/* * * * * * * * * * * bio_ik_v12.cpp * * * * * * * * * * */
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

#include <bio_ik_v12.h>

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



        // Plot keypoints from median filter [RVIZ]
        //plotKeypointsRVIZ(mean_kp, false);

        // Map OpenPose keypoints to Shadow Hand
        mean_kp = mapShadowHand(mean_kp); 
        
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
        if (prev_kp == mean_kp){
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
            

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

            linkDir[0] = mean_kp[6]  - mean_kp[5];   // ff_proximal
            linkDir[1] = mean_kp[10] - mean_kp[9];   // mf_proximal
            linkDir[2] = mean_kp[14] - mean_kp[13];  // rf_proximal 
            linkDir[3] = mean_kp[18] - mean_kp[17];  // lf_proximal
            linkDir[4] = mean_kp[3]  - mean_kp[2];   // th_middle
            linkDir[5] = mean_kp[4] - mean_kp[3];    // th_distal
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
            std::cout << "FF_PROXIMAL = (" << linkDir[0].x() << "," << linkDir[1].y() << "," << linkDir[1].z() << ")" << std::endl;
            std::cout << "MF_PROXIMAL = (" << linkDir[1].x() << "," << linkDir[2].y() << "," << linkDir[2].z() << ")" << std::endl;
            std::cout << "RF_PROXIMAL = (" << linkDir[2].x() << "," << linkDir[3].y() << "," << linkDir[3].z() << ")" << std::endl;
            std::cout << "LF_PROXIMAL = (" << linkDir[3].x() << "," << linkDir[4].y() << "," << linkDir[4].z() << ")" << std::endl;
            std::cout << "TH_MIDDLE = ("   << linkDir[4].x() << "," << linkDir[0].y() << "," << linkDir[0].z() << ")" << std::endl;
            std::cout << "LF_DISTAL = ("   << linkDir[5].x() << "," << linkDir[5].y() << "," << linkDir[5].z() << ")" << std::endl;
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
            "rh_ffproximal",
            "rh_mfproximal",
            "rh_rfproximal",
            "rh_lfproximal",
            "rh_thmiddle",
            "rh_thdistal"
        };


        // BioIK condition weights
        std::vector <float> MapPositionWeights {1.0,1.0,1.0,1.0,1.0,0.2,0.2,0.2,0.2,0.2};
        std::vector <float> MapDirectionWeights{0.1,0.1,0.1,0.1,0.1,0.1};
        float CoupleJointsWeight = 1.0;
        float CenterJointsWeight = 0.15;
        // BioIK goals
        bio_ik::BioIKKinematicsQueryOptions ik_options;
        ik_options.replace = true;
        ik_options.return_approximate_solution = true;

        // Set position constraints
        for (int i=0; i<MapPositionlinks.size(); i++)
        {
            // Transform position from world into base_frame
            geometry_msgs::PointStamped stamped_in;
            stamped_in.header.frame_id = "world";
            stamped_in.point.x = linkPos[i].x();
            stamped_in.point.y = linkPos[i].y();
            stamped_in.point.z = linkPos[i].z();
            
            geometry_msgs::PointStamped stamped_out ;
            tfBuffer.transform(stamped_in, stamped_out, base_frame);
            tf2::Vector3 Mapposition (stamped_out.point.x, stamped_out.point.y, stamped_out.point.z);

            ik_options.goals.emplace_back(new bio_ik::PositionGoal(MapPositionlinks[i], Mapposition, MapPositionWeights[i]));
        }
        // Set orientation constraints
        for (int i=0; i<MapDirectionlinks.size(); i++)
        {
            // Transform orientation from world into base_frame
            geometry_msgs::PointStamped stamped_in;
            stamped_in.header.frame_id = "world";
            stamped_in.point.x = linkDir[i].x();
            stamped_in.point.y = linkDir[i].y();
            stamped_in.point.z = linkDir[i].z();

            geometry_msgs::PointStamped stamped_out;
            tfBuffer.transform(stamped_in, stamped_out, base_frame);
            tf2::Vector3 Mapdirection (stamped_out.point.x, stamped_out.point.y, stamped_out.point.z);

            ik_options.goals.emplace_back(new bio_ik::DirectionGoal(MapDirectionlinks[i], tf2::Vector3(0,0,1), Mapdirection.normalized(), MapDirectionWeights[i]));
        }
        // Set non-linear Shadow Hand joint coupling constraints
        std::vector<std::string> ff_coupled_joints, mf_coupled_joints, rf_coupled_joints, lf_coupled_joints, wr_forced_joints;
        ff_coupled_joints.push_back("rh_FFJ1");
        ff_coupled_joints.push_back("rh_FFJ2");
        mf_coupled_joints.push_back("rh_MFJ1");
        mf_coupled_joints.push_back("rh_MFJ2");
        rf_coupled_joints.push_back("rh_RFJ1");
        rf_coupled_joints.push_back("rh_RFJ2");
        lf_coupled_joints.push_back("rh_LFJ1");
        lf_coupled_joints.push_back("rh_LFJ2");
        wr_forced_joints.push_back("rh_WRJ1");
        auto* ff_goal = new bio_ik::JointFunctionGoal(
                            ff_coupled_joints,
                            [=] (std::vector<double>& vv) {
                                vv[0] = fmax(0, vv[1]-PI);  // if J2<90º => J1=0
                                vv[1] = fmin(PI/2, vv[1]);  // max(J2) = 90º       
                            },  CoupleJointsWeight 
                        );
        auto* mf_goal = new bio_ik::JointFunctionGoal(
                            mf_coupled_joints,
                            [=] (std::vector<double>& vv) {
                                vv[0] = fmax(0, vv[1]-PI);  // if J2<90º => J1=0
                                vv[1] = fmin(PI/2, vv[1]);  // max(J2) = 90º       
                            },  CoupleJointsWeight 
                        );
        auto* rf_goal = new bio_ik::JointFunctionGoal(
                            rf_coupled_joints,
                            [=] (std::vector<double>& vv) {
                                vv[0] = fmax(0, vv[1]-PI);  // if J2<90º => J1=0
                                vv[1] = fmin(PI/2, vv[1]);  // max(J2) = 90º       
                            },  CoupleJointsWeight  
                        );
        auto* lf_goal = new bio_ik::JointFunctionGoal(
                            lf_coupled_joints,
                            [=] (std::vector<double>& vv) {
                                vv[0] = fmax(0, vv[1]-PI);  // if J2<90º => J1=0
                                vv[1] = fmin(PI/2, vv[1]);  // max(J2) = 90º       
                            },  CoupleJointsWeight  
                        );
        auto* wr_goal = new bio_ik::JointFunctionGoal(
                            wr_forced_joints,
                            [=] (std::vector<double>& vv) {
                                vv[0] = -0.523;  // if J2<90º => J1=0
                                    
                            },  0.5
                        );
        ik_options.goals.emplace_back(ff_goal);
        ik_options.goals.emplace_back(mf_goal);
        ik_options.goals.emplace_back(rf_goal);
        ik_options.goals.emplace_back(lf_goal);
        ik_options.goals.emplace_back(wr_goal);
        // Set Center Joints Goal
        ik_options.goals.emplace_back(new bio_ik::CenterJointsGoal(CenterJointsWeight));

        // Get current robot state
        robot_state::RobotState& current_state = (*planning_scene_pointer).getCurrentStateNonConst(); 
        (*robot_state_pointer) = current_state;  

        // Set BioIK solver
        bool found_ik = (*robot_state_pointer).setFromIK(
                            joint_model_group,             // UR5 + Shadow Hand joints
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
        std::vector<double> joint_angles;
        std_msgs::Float64MultiArray joint_angles_msg;
        if (found_ik){
            (*robot_state_pointer).copyJointGroupPositions(joint_model_group, joint_angles);
            (*mgi_pointer).setJointValueTarget(joint_angles);

            // Get collisions
            collision_request.contacts = true;
            collision_request.max_contacts = 1000;
            collision_result.clear();

            current_state = (*mgi_pointer).getJointValueTarget();
            (*planning_scene_pointer).checkSelfCollision(collision_request, collision_result);
            collision_detection::CollisionResult::ContactMap::const_iterator it;
            collision_pairs.clear();

            for(it = collision_result.contacts.begin();	it != collision_result.contacts.end(); ++it){
                collision_pairs.push_back(it->first.first.c_str());
                collision_pairs.push_back(it->first.second.c_str());
                ROS_WARN("Collision between: %s and %s, need to reIK", it->first.first.c_str(), it->first.second.c_str());
            }

            if (collision_pairs.size() > 0 ){
                // self_collision_free goal
                double collision_weight = 1;
                ik_options.goals.emplace_back(new Collision_freeGoal(collision_pairs, collision_weight));

                // set ik solver again
                bool refound_ik = (*robot_state_pointer).setFromIK(
                                    joint_model_group,             // UR5 + Shadow Hand joints
                                    EigenSTL::vector_Isometry3d(), // no explicit poses here
                                    std::vector<std::string>(),
                                    timeout+0.1,
                                    moveit::core::GroupStateValidityCallbackFn(),
                                    ik_options
                                );

                if (refound_ik){
                    (*robot_state_pointer).copyJointGroupPositions(joint_model_group, joint_angles);
                    (*mgi_pointer).setJointValueTarget(joint_angles);

                    // get collision pairs then use collision free ik
                    collision_request.contacts = true;
                    collision_request.max_contacts = 1000;
                    collision_result.clear();

                    current_state = (*mgi_pointer).getJointValueTarget();
                    (*planning_scene_pointer).checkSelfCollision(collision_request, collision_result);
                    if (collision_result.contacts.size() > 0){
                        ROS_ERROR("Failed to get collision_free result, skip to next one");
                        continue;
                    }
                }
                else{
                    std::cout << "Did not find reIK solution" << std::endl;
                    continue;
                }
            }
            joint_angles_msg.data = joint_angles;
        }
        else
            std::cout << "Did not find IK solution" << std::endl;

        if (joint_angles.size() < 10){
            // Reset planning goals
            for (int j = 0; j <ik_options.goals.size();j++)
                ik_options.goals[j].reset();
            continue;
        }

        // DEUBG
        if (false){
            std::cout << "Sent joint angles:" << std::endl;
            for (int i=0; i<joint_angles.size(); i++)
                std::cout << " " << joint_angles[i] << " ";
            std::cout << std::endl;
        }

        // Publish Shadow Hand command
        joints_shadow.publish(joint_angles_msg);
        std::cout << "\n\033[36mJoint angles calculated!\033[0m\n" << std::endl;

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

    // Create vector with all keypoint positions
    std::vector<Eigen::Vector3d> keypoints;
    for (int i=0; i<21; i++){
        Eigen::Vector3d aux_point(msg->keypoints[i].x, msg->keypoints[i].y, msg->keypoints[i].z);
        keypoints.push_back(aux_point);
    }  

    //std::cout << "WRIST: " << keypoints[0] << std::endl;

    plotKeypointsRVIZ(keypoints, false);

    mutex_kp.lock();
        kp_positions.insert(kp_positions.begin(), keypoints);
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
    ros::init(argc, argv, "bio_ik_v10");
    std::cout << "\"bio_ik_v10\" ROS node started!" << std::endl;
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(5);
    spinner.start();

    // ROS Transform
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformListener tfListener2(tfBuffer2);

    // Moveit
    std::string group_name = "right_arm_and_hand";
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

    // DEBUG
    if (true){
        // Print joint names
        std::vector<std::string> joint_names = joint_model_group->getJointModelNames();
        for (const auto& name : joint_names) {
            std::cout << name << " ";
            std::cout << std::endl;
        }
    }
    
    // Set control velocity and acceleration
    mgi.setMaxAccelerationScalingFactor(0.2);
    mgi.setMaxVelocityScalingFactor(0.2);

    // Add collision object    
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "box1";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    // Define the size of the box in meters
    primitive.dimensions[primitive.BOX_X] = 2;
    primitive.dimensions[primitive.BOX_Y] = 2;
    primitive.dimensions[primitive.BOX_Z] = 0.1;
    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0;
    box_pose.position.y = 0;
    box_pose.position.z = -0.05;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    // Add the collision object to the scene
    planning_scene.applyCollisionObject(collision_object);
    
    // Define pointers to access vars in callback
    mgi_pointer = &mgi;
    robot_state_pointer = &robot_state;
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
    joints_shadow = nh.advertise<std_msgs::Float64MultiArray>("ur5_shadow_joints", 1);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("hand_keypoints_marker", 1);
    marker_pub_shadow = nh.advertise<visualization_msgs::MarkerArray>("shadow_keypoints_marker", 1);

    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    planning_scene_diff_publisher.publish(planning_scene);

    // Ready 
    std::cout << "\n\033[1;32m\"bio_ik_v10\" ROS node is ready!\033[0m\n" << std::endl;

    ros::waitForShutdown(); // because of ros::AsyncSpinner
    //ros::spin();
    return 0;
}
