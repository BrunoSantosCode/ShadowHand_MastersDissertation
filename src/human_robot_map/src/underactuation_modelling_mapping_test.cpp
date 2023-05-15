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

#include <human_robot_map/collision_free_goal.h>
#include <bio_ik/bio_ik.h>


std::string item;


int main(int argc, char** argv){
    ros::init(argc, argv, "bio_ik_human_robot_mapping", 1);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::AsyncSpinner spinner(5);
    spinner.start();
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    std::string mapfile_;
    std::string jointsfile_;
    std::string final_state_file_;
    std::string objectives_file_;
    pnh.getParam("mapfile", mapfile_);
    pnh.getParam("jointsfile", jointsfile_);
    pnh.getParam("statefile", final_state_file_);
    pnh.getParam("objectives", objectives_file_ );
    std::ofstream objectives_file;

    //std::string group_name = "sr_hand";
    std::string group_name = "right_hand";
    moveit::planning_interface::MoveGroupInterface mgi(group_name);
    std::string base_frame = mgi.getPoseReferenceFrame();

    //std::vector <std::string> joint_names = mgi.getJointNames();
    //for(int i = 0; i<joint_names.size(); i++)
    //    std::cout << joint_names[i] <<  std::endl;

    //return 0;

    mgi.setGoalTolerance(0.01);
    mgi.setPlanningTime(25);
    mgi.setPlannerId("RRTConnectkConfigDefault");
    auto robot_model = mgi.getCurrentState()->getRobotModel();
    auto joint_model_group = robot_model->getJointModelGroup(group_name);
    moveit::core::RobotState robot_state(robot_model);
    planning_scene::PlanningScene planning_scene(robot_model);
    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    std::vector<std::string> collision_pairs;


    double timeout = 0.2;
    std::vector<std::string> MapPositionlinks {
      "rh_thtip",
      "rh_fftip",
      "rh_mftip",
      "rh_rftip",
      "rh_lftip"
    };

    std::vector <float> MapPositionweights {1,1,1,1,1};

    // eixo z tá com algum erro no scaling possivelmente
    std::ifstream mapfile(mapfile_);
    std::string line, items;
    while(std::getline(mapfile, line)){
        ros::Time begin = ros::Time::now();
        // track goals using bio ik
        bio_ik::BioIKKinematicsQueryOptions ik_options;
        ik_options.replace = true;
        ik_options.return_approximate_solution = true;
        std::vector <double> cartesian_objectives;
        std::istringstream myline(line);
        std::vector<double> csvItem;
        while(std::getline(myline, items, ','))
        {   
            //std::cout<<items[0]<<std::endl;
            if (items[0]=='S')
            {
                item = items;
                std::cout<< item <<std::endl;
                continue;
            }
            csvItem.push_back(std::stof(items));
        }

        for (int j = 0; j< MapPositionlinks.size(); j++)
        {
            int t = j * 3;

            // transform position from current rh_wrist into base_frame
            geometry_msgs::PointStamped stamped_in;
            stamped_in.header.frame_id = "rh_wrist";
            stamped_in.point.x = csvItem[t];
            stamped_in.point.y = csvItem[t+1];
            stamped_in.point.z = csvItem[t+2]+29/1000; //somar 29mm uma vez que pontos estao representados num referencial 29mm acima do pulso da mão

            geometry_msgs::PointStamped stamped_out ;
            tfBuffer.transform(stamped_in, stamped_out, base_frame);
            tf2::Vector3 Mapposition (stamped_out.point.x, stamped_out.point.y, stamped_out.point.z);
            cartesian_objectives.push_back(stamped_in.point.x);
            cartesian_objectives.push_back(stamped_in.point.y);
            cartesian_objectives.push_back(stamped_in.point.z);
            ik_options.goals.emplace_back(new bio_ik::PositionGoal(MapPositionlinks[j], Mapposition, MapPositionweights[j]));
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

        double ffj1_coef [3] = { 0.29224892 ,-0.48143318,  0.16112862};
        double ffj2_coef [3] = {-0.29224892 , 1.48143318, -0.16112862};
        double mfj1_coef [3] = { 0.29700455 ,-0.48762374,  0.14290133};
        double mfj2_coef [3] = {-0.29700455 , 1.48762374, -0.14290133};
        double rfj1_coef [3] = { 0.29923148 ,-0.48556757,  0.15734697};
        double rfj2_coef [3] = {-0.29923148 , 1.48556757, -0.15734697};
        double lfj1_coef [3] = { 0.28503535 ,-0.44252717,  0.144264  };
        double lfj2_coef [3] = {-0.28503535 , 1.44252717, -0.144264  };
         

        auto* ff_underactuation_goal = new bio_ik::JointFunctionGoal(ff_coupled_joints,
                                                                [=] (std::vector<double>& vv){
                                                                    vv[0] = ffj1_coef[0] + ffj1_coef[1] * (vv[0]+vv[1]) + ffj1_coef[2] * pow((vv[0]+vv[1]),2);
                                                                    vv[1] = ffj2_coef[0] + ffj2_coef[1] * (vv[0]+vv[1]) + ffj2_coef[2] * pow((vv[0]+vv[1]),2);
                                                                }, couple_constr_weight, false
                                                                );

        auto* mf_underactuation_goal = new bio_ik::JointFunctionGoal(mf_coupled_joints,
                                                                [=] (std::vector<double>& vv){
                                                                    vv[0] = mfj1_coef[0] + mfj1_coef[1] * (vv[0]+vv[1]) + mfj1_coef[2] * pow((vv[0]+vv[1]),2);
                                                                    vv[1] = mfj2_coef[0] + mfj2_coef[1] * (vv[0]+vv[1]) + mfj2_coef[2] * pow((vv[0]+vv[1]),2);
                                                                }, couple_constr_weight, false
                                                                );

        auto* rf_underactuation_goal = new bio_ik::JointFunctionGoal(rf_coupled_joints,
                                                                [=] (std::vector<double>& vv){
                                                                    vv[0] = rfj1_coef[0] + rfj1_coef[1] * (vv[0]+vv[1]) + rfj1_coef[2] * pow((vv[0]+vv[1]),2);
                                                                    vv[1] = rfj2_coef[0] + rfj2_coef[1] * (vv[0]+vv[1]) + rfj2_coef[2] * pow((vv[0]+vv[1]),2);
                                                                }, couple_constr_weight, false
                                                                );

        auto* lf_underactuation_goal = new bio_ik::JointFunctionGoal(lf_coupled_joints,
                                                                [=] (std::vector<double>& vv){
                                                                    vv[0] = lfj1_coef[0] + lfj1_coef[1] * (vv[0]+vv[1]) + lfj1_coef[2] * pow((vv[0]+vv[1]),2);
                                                                    vv[1] = lfj2_coef[0] + lfj2_coef[1] * (vv[0]+vv[1]) + lfj2_coef[2] * pow((vv[0]+vv[1]),2);
                                                                }, couple_constr_weight, false
                                                                );                                                                                                                                

        //ik_options.goals.emplace_back(ff_underactuation_goal);
        //ik_options.goals.emplace_back(mf_underactuation_goal);
        //ik_options.goals.emplace_back(rf_underactuation_goal);
        //ik_options.goals.emplace_back(lf_underactuation_goal);


        robot_state = current_state;
        // set ik solver
        bool found_ik = robot_state.setFromIK(
                          joint_model_group,             // active Shadow joints
                          EigenSTL::vector_Isometry3d(), // no explicit poses here
                          std::vector<std::string>(),
                          timeout,
                          moveit::core::GroupStateValidityCallbackFn(),
                          ik_options
                        );
        // move to the solution position
        std::vector<double> joint_values;
        moveit::planning_interface::MoveGroupInterface::Plan shadow_plan;
        if (found_ik)
        {
            robot_state.copyJointGroupPositions(joint_model_group, joint_values);
            // set the angle of two wrist joint zero
            //joint_values[0] = 0;
            //joint_values[1] = 0;
            mgi.setJointValueTarget(joint_values);

            // get collision pairs then use collision free ik
            collision_request.contacts = true;
            collision_request.max_contacts = 1000;
            collision_result.clear();

            current_state = mgi.getJointValueTarget();
            planning_scene.checkSelfCollision(collision_request, collision_result);
            collision_detection::CollisionResult::ContactMap::const_iterator it;
            collision_pairs.clear();
            for(it = collision_result.contacts.begin();	it != collision_result.contacts.end(); ++it)
            {
                collision_pairs.push_back(it->first.first.c_str());
                collision_pairs.push_back(it->first.second.c_str());
                ROS_WARN("Collision between: %s and %s, need to reIK", it->first.first.c_str(), it->first.second.c_str());
            }

            if (collision_pairs.size() > 0 )
            {
                 // self_collision_free goal
                 double collision_weight = 1;
                 ik_options.goals.emplace_back(new Collision_freeGoal(collision_pairs, collision_weight));

                 // set ik solver again
                 bool refound_ik = robot_state.setFromIK(
                                   joint_model_group,           // active Shadow joints
                                   EigenSTL::vector_Isometry3d(), // no explicit poses here
                                   std::vector<std::string>(),
                                   timeout+0.1,
                                   moveit::core::GroupStateValidityCallbackFn(),
                                   ik_options
                                  );

                 if (refound_ik)
                 {
                    robot_state.copyJointGroupPositions(joint_model_group, joint_values);
                    // set the angle of two wrist joint zero
                    //joint_values[0] = 0;
                    //joint_values[1] = 0;
                    mgi.setJointValueTarget(joint_values);

                    // get collision pairs then use collision free ik
                    collision_request.contacts = true;
                    collision_request.max_contacts = 1000;
                    collision_result.clear();

                    current_state = mgi.getJointValueTarget();
                    planning_scene.checkSelfCollision(collision_request, collision_result);
                    if (collision_result.contacts.size() > 0)
                    {
                        ROS_ERROR("Failed to get collision_free result, skip to next one");
                        continue;
                    }
                }
                else
                {
                    std::cout << "Did not find reIK solution" << std::endl;
                    continue;
                }
            }

            if (!(static_cast<bool>(mgi.plan(shadow_plan))))
            {
                std::cout<< "Failed to plan pose " << item << std::endl;
                continue;
            }

            if(!(static_cast<bool>(mgi.execute(shadow_plan))))
            {
                std::cout << "Failed to execute pose " << item<< std::endl;
                continue;
            }

            //save objectives that were attempted
            objectives_file.open(objectives_file_,std::ios::app);
            objectives_file << item << ',' << std::to_string( cartesian_objectives[0]) << ',' << std::to_string( cartesian_objectives[1]) <<','
                << std::to_string( cartesian_objectives[2]) <<',' << std::to_string( cartesian_objectives[3]) <<',' << std::to_string( cartesian_objectives[4]) <<','
                << std::to_string( cartesian_objectives[5]) <<',' << std::to_string( cartesian_objectives[6]) <<',' << std::to_string( cartesian_objectives[7]) <<','
                << std::to_string( cartesian_objectives[8]) <<',' << std::to_string( cartesian_objectives[9]) <<',' << std::to_string( cartesian_objectives[10]) <<','
                << std::to_string( cartesian_objectives[11]) <<',' << std::to_string( cartesian_objectives[12]) <<',' << std::to_string( cartesian_objectives[13]) <<','
                << std::to_string( cartesian_objectives[14]) << std::endl;
            objectives_file.close();  

            cartesian_objectives.clear();
            

            //save executed joint angles
            std::vector<double> joint_states;
            joint_states = mgi.getCurrentJointValues();
            std::ofstream final_state_file;
            final_state_file.open(final_state_file_,std::ios::app);
            final_state_file << item << ',' << std::to_string( joint_states[0]) << ',' << std::to_string( joint_states[1]) <<','
            << std::to_string( joint_states[2]) <<',' << std::to_string( joint_states[3]) <<',' << std::to_string( joint_states[4]) <<','
            << std::to_string( joint_states[5]) <<',' << std::to_string( joint_states[6]) <<',' << std::to_string( joint_states[7]) <<','
            << std::to_string( joint_states[8]) <<',' << std::to_string( joint_states[9]) <<',' << std::to_string( joint_states[10]) <<','
            << std::to_string( joint_states[11]) <<',' << std::to_string( joint_states[12]) <<',' << std::to_string( joint_states[13]) <<','
            << std::to_string( joint_states[14]) <<',' << std::to_string( joint_states[15]) <<',' << std::to_string( joint_states[16]) <<','
            << std::to_string( joint_states[17]) <<',' << std::to_string( joint_states[18]) <<',' << std::to_string( joint_states[19]) <<','
            << std::to_string( joint_states[20]) <<',' << std::to_string( joint_states[21]) <<',' << std::to_string( joint_states[22]) <<','
            << std::to_string( joint_states[23]) << std::endl;
            final_state_file.close();


            // save planned joint angles
            std::ofstream joints_file;
            joints_file.open(jointsfile_,std::ios::app);
            joints_file << item << ',' << std::to_string( joint_values[0]) << ',' << std::to_string( joint_values[1]) <<','
            << std::to_string( joint_values[2]) <<',' << std::to_string( joint_values[3]) <<',' << std::to_string( joint_values[4]) <<','
            << std::to_string( joint_values[5]) <<',' << std::to_string( joint_values[6]) <<',' << std::to_string( joint_values[7]) <<','
            << std::to_string( joint_values[8]) <<',' << std::to_string( joint_values[9]) <<',' << std::to_string( joint_values[10]) <<','
            << std::to_string( joint_values[11]) <<',' << std::to_string( joint_values[12]) <<',' << std::to_string( joint_values[13]) <<','
            << std::to_string( joint_values[14]) <<',' << std::to_string( joint_values[15]) <<',' << std::to_string( joint_values[16]) <<','
            << std::to_string( joint_values[17]) <<',' << std::to_string( joint_values[18]) <<',' << std::to_string( joint_values[19]) <<','
            << std::to_string( joint_values[20]) <<',' << std::to_string( joint_values[21]) <<',' << std::to_string( joint_values[22]) <<','
            << std::to_string( joint_values[23]) << std::endl;
            joints_file.close();

            ros::Duration dur = ros::Time::now() - begin;
            std::cout << "running time is "  << dur <<  std::endl;
        }
        else
        {
            std::cout << "Did not find IK solution" << std::endl;
        }

        for (int j = 0; j <ik_options.goals.size();j++)
            ik_options.goals[j].reset();
    }

    ros::shutdown();
    return 0;
}