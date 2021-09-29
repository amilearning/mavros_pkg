/**
 * @file hmcl_fsm.cpp
 * @brief Main PLanning stack for Drone competition
 */
#include "hmcl_fsm.h"



hmclFSM::~hmclFSM() {}

hmclFSM::hmclFSM(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,const ros::NodeHandle& cmd_nh, const ros::NodeHandle& fsm_nh, const ros::NodeHandle& lidar_nh, const ros::NodeHandle& odom_nh)
: nh_(nh), nh_private_(nh_private), cmd_nh_(cmd_nh), fsm_nh_(fsm_nh), lidar_nh_(lidar_nh), odom_nh_(odom_nh){     
    
    // Dynamic Configure parameters
    f = boost::bind(&hmclFSM::dyn_callback, this,_1, _2);
    server.setCallback(f);

    load_FSM_Params("fsm"); 
    ROS_INFO("init_takeoff_ = %f",  init_takeoff_);
    ROS_INFO("lidar_avoidance_distance_ = %f", lidar_avoidance_distance_);
    ROS_INFO("global_pose_x_min = %f",global_pose_x_min); 
    ROS_INFO("global_pose_y_min = %f",global_pose_y_min); 
    ROS_INFO("global_pose_z_min = %f",global_pose_z_min); 
    ROS_INFO("global_pose_x_max = %f",global_pose_x_max); 
    ROS_INFO("global_pose_y_max = %f",global_pose_y_max); 
    ROS_INFO("global_pose_z_max = %f",global_pose_z_max); 
         
    
    ROS_INFO_STREAM("d0 is set to be = " << d0);
    ROS_INFO_STREAM("k0 is set to be = " << k0);
    
    waypoints_itr = 0;
    odom_received = false;
    att_clb_first_callback = false;
    
    multiDOFJointSub = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/m100/command/trajectory",10,&hmclFSM::multiDOFJointCallback,this);            
    bbx_sub   = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("/trt_yolo_ros/bounding_boxes", 10, &hmclFSM::bbxCallback,this);
    pos_cmd_sub = nh_.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10, &hmclFSM::poseCmdCallback,this);    

    wallfollower_r_sub = nh_.subscribe<mavros_msgs::PositionTarget>("/mapfollow/target/pose", 10, &hmclFSM::wallFollowCmdCallback_r,this);    
    wallfollower_l_sub = nh_.subscribe<mavros_msgs::PositionTarget>("/mapfollow/target/pose_l", 10, &hmclFSM::wallFollowCmdCallback_l,this);    
    


    lidar_sub = lidar_nh_.subscribe<sensor_msgs::LaserScan>("/scan",1,&hmclFSM::lidarCallback,this);    
    
    odom_sub = odom_nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",10, &hmclFSM::odom_cb,this);  
    state_sub = odom_nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &hmclFSM::state_cb,this);
    vision_odom_sub = odom_nh_.subscribe<geometry_msgs::PoseStamped>("/vins/px4/pose",10, &hmclFSM::visCallback,this);    
    

    cmdloop_timer_ = cmd_nh_.createTimer(ros::Duration(0.02), &hmclFSM::cmdloopCallback,this); // Critical -> allocate another thread 
    if(FSM_mode == 0){
        // activate exploartion fsm
        fsm_timer_ = fsm_nh_.createTimer(ros::Duration(0.2), &hmclFSM::mainFSMCallback,this); 
        waypoint_iter_timer_ = nh_.createTimer(ros::Duration(0.0), &hmclFSM::waypointTimerCallback,this,false,true); 
    }else if(FSM_mode ==1 ) {
        // activate local traj fsm
        fsm_timer_ = fsm_nh_.createTimer(ros::Duration(0.1), &hmclFSM::localFSMCallback,this); 
        local_path_trigger_sub = nh_.subscribe<std_msgs::Empty>("/planning/new",1,&hmclFSM::localTrajTrigCallback,this);
    }else{
        ROS_WARN("FSM mode is not valid");        
    }
    
    lidar_timer_ = lidar_nh_.createTimer(ros::Duration(0.01), &hmclFSM::lidarTimeCallback,this); //Critical -> allocate another thread 
    
    
    
    // ros::Subscriber att_thrust_sub = nh.subscribe<mav_msgs::RollPitchYawrateThrust>
    //         ("/lmpc/roll_pitch_yawrate_thrust", 10, rpyt_cb);
    manual_trj_pub =  nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/command/trajectory",5);
    mpc_cmd_pub =  nh_.advertise<mav_msgs::RollPitchYawrateThrust>("/mavros/setpoint_raw/roll_pitch_yawrate_thrust",5);
    rpyt_pub = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);
    position_target_pub = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);    
    vis_pos_pub  =  nh_.advertise<geometry_msgs::TransformStamped>("/vins/posetransform", 10);    
    local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    camera_points_pub = nh_.advertise<sensor_msgs::PointCloud2>("camera_points", 2);     
    vins_odom_pub = nh_.advertise<nav_msgs::Odometry>("vins_odom", 10);
    local_goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
    
    

    
    // Define service Clients 
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    explore_client_start = nh_.serviceClient<std_srvs::Trigger>("/planner_control_interface/std_srvs/automatic_planning");
    explore_client_stop =nh_.serviceClient<std_srvs::Trigger>("/planner_control_interface/std_srvs/stop");
    explore_client_home = nh_.serviceClient<std_srvs::Trigger>("/planner_control_interface/std_srvs/homing_trigger");
    explore_client_global_planner = nh_.serviceClient<planner_msgs::pci_global>("pci_global");
    
  
    
    // initialize control method
    pose_cmd_enable = false;
    mpc_cmd_enable = false;
    send_waypoint = false;
    avoidance_enable = false;
    local_trj_enable = false;

    
    //the setpoint publishing rate MUST be faster than 2Hz
    // ros::Rate rate(2); // 10 HZ
    // while(ros::ok() && !current_state.connected){
    //     ROS_INFO("lets try to connect");
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    
    // // ROS_INFO("PX4 connected");    

    // // // Wait for other topics 
    // while(!odom_received){
    //     ROS_INFO("wait for odom to be available");
    //     ros::spinOnce();
    //     rate.sleep();
    // }


    offb_set_mode.request.custom_mode = "OFFBOARD";    
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    ROS_INFO("Try OFFBOARD and Arming");
    armed = false;
    offboarded = false;
    ROS_INFO("now we are in hand-hold mode");
    armed = true;
    offboarded = false;
    
    // while(ros::ok()){            
    //     if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
    //             {
    //                 // if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){ROS_INFO("Offboard enabled");}
    //                 if( set_mode_client.call(offb_set_mode) ){
    //                     ROS_INFO("Offboard enabled");
    //                     offboarded = true;}
    //                 last_request = ros::Time::now();
    //             } else {          
    //                 if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0))){
    //                     // if( arming_client.call(arm_cmd) && arm_cmd.response.success){ROS_INFO("Vehicle armed");}
    //                     if( arming_client.call(arm_cmd)){
    //                         ROS_INFO("Vehicle armed");
    //                         armed = true;                            
    //                         }else{
    //                         ROS_INFO("arm failed");
    //                         }
    //                     last_request = ros::Time::now();
    //                 }
    //             }
    //     if(offboarded && armed){ break;}                
    // }    
    ROS_INFO("Initializing FSM");    
    mainFSM_mode = mainFSMmode::Init;
    Explore_Mode = ExploreMode::NA;
    Detect_Mode  = DetectMode::NA;
    Land_Mode =  LandMode::NA;
    localsearch_count = 0;

}

void hmclFSM::wallFollowCmdCallback_l(const mavros_msgs::PositionTargetConstPtr &msg){
    if(wall_l_follow){
        pose_target_ = *msg;        
    }
    return ;
}

void hmclFSM::wallFollowCmdCallback_r(const mavros_msgs::PositionTargetConstPtr &msg){
    if(wall_r_follow){
        pose_target_ = *msg;  
    }
    return ;
}

void hmclFSM::localTrajTrigCallback(const std_msgs::EmptyConstPtr &msg){
    // ROS_INFO("trjectory received");
    local_path_received = true;
}


void hmclFSM::load_FSM_Params(std::string group){
    nh_private_.getParam("FSM_mode",FSM_mode);
    nh_private_.getParam("verbos",verbos);
    nh_private_.getParam("d0",d0);
    nh_private_.getParam("k0",k0);
    nh_private_.getParam("thrust_scale",thrust_scale);
    nh_private_.getParam("manual_trj_switch",manual_trj_switch_);
    nh_private_.getParam(group+"/init_takeoff",init_takeoff_);
    nh_private_.getParam(group+"/lidar_avoidance_distance",lidar_avoidance_distance_);
    nh_private_.getParam(group+"/global_pose_x_min",global_pose_x_min);
    nh_private_.getParam(group+"/global_pose_y_min",global_pose_y_min);
    nh_private_.getParam(group+"/global_pose_z_min",global_pose_z_min);
    nh_private_.getParam(group+"/global_pose_x_max",global_pose_x_max);
    nh_private_.getParam(group+"/global_pose_y_max",global_pose_y_max);
    nh_private_.getParam(group+"/global_pose_z_max",global_pose_z_max);
}



void hmclFSM::init_takeoff(){
    if(!odom_received){
        ROS_INFO("odom is not availble");
        return;
    }
    //  if( current_state.mode != "OFFBOARD"){  // if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){ROS_INFO("Offboard enabled");}
    //         set_mode_client.call(offb_set_mode);                
    //     } else {          
    //         if( !current_state.armed ){            
    //             arming_client.call(arm_cmd);
    //         }
    //     }
    pose_target_.header.stamp = ros::Time::now();
    pose_target_.header.frame_id ='map';        
    pose_target_.coordinate_frame = 1; // mavros_msgs::PositionTarget::FRAME_LOCAL_NED;            
    pose_target_.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | 
                                mavros_msgs::PositionTarget::IGNORE_VX  | 
                                mavros_msgs::PositionTarget::IGNORE_VY  | 
                                mavros_msgs::PositionTarget::IGNORE_VZ;
                                
    pose_target_.header.stamp = ros::Time::now();
    
    pose_target_.position.x = 0.0;
    pose_target_.position.y = 0.0;
    pose_target_.position.z = init_takeoff_;     
    pose_target_.yaw = 0.0;
    // position_target_pub.publish(pose_target_);  
}

void hmclFSM::localFSMCallback(const ros::TimerEvent &event){   
    // if(!armed && !offboarded) return;     
    
    if(!check_if_drone_outside_global_box()){
        if(mainFSM_mode == mainFSMmode::Exploration){
            // Explore_Mode = ExploreMode::GlobalSearching;    
             pose_target_.position.z = init_takeoff_;  
            ROS_INFO("Drone is outside of global box --> Global Searching activated ");         
        }else if(mainFSM_mode == mainFSMmode::Avoidance){
            pose_target_.position.z = init_takeoff_;
                ROS_INFO("Avoidance::out side of global box area");
        }else if(mainFSM_mode == mainFSMmode::Detection){
            pose_target_.position.z = init_takeoff_;
                ROS_INFO("Detection::out side of global box area");
        }else if(mainFSM_mode == mainFSMmode::Landing){            
                ROS_INFO("Landing::out side of global box area");
        }
    }
    
    std_srvs::Trigger localsearch_srv;
    planner_msgs::pci_global plan_srv;       
    std_srvs::Trigger stop_srv;  
    std_srvs::Trigger homing_srv;
    double tmp_dist = 0.0;
    
    
    switch (mainFSM_mode)
    {
        case mainFSMmode::NA:
        ROS_INFO("Something is wrong...mainFSM is in NA mode");
        printFSMstate();
        mainFSM_mode = mainFSMmode::Init;        
        // Move_to_target_pose(target_pose);            
        break;

        case mainFSMmode::Init:       
            previous_mode = mainFSM_mode;                            
            ROS_INFO("Initializing ");
            ROS_INFO("Main = %s",stateToString(mainFSM_mode));
            init_takeoff();
                if (fabs(current_pose.position.z - init_takeoff_) < 0.1){
                    ROS_INFO("Successful Take off");
                    mainFSM_mode = mainFSMmode::Exploration;     
                    Explore_Mode = ExploreMode::GlobalSearching;   
                        ros::Duration(2.0).sleep();                    
                }
        break;
        
        case mainFSMmode::Exploration:
            ROS_INFO("Main = %s, Sub = %s",stateToString(mainFSM_mode),stateToString(Explore_Mode));
            previous_mode = mainFSM_mode;                              
            switch (Explore_Mode){
                case ExploreMode::NA:                
                ROS_INFO("Explore NA");                    
                break;  
                
                case ExploreMode::LocalSearching:                           
                        // Send local target goal to local planner 
                    ROS_INFO("initiate local planner");
                    
                    if(local_path_received){
                        local_trj_enable = true;
                        // local_trj_switch_ = true;
                            
                        ROS_INFO("local planner actiavte with cmd");                    
                        if(get_distance(pose_at_request,current_pose) < 0.05){                        
                           goal_request_count++;
                            if(goal_request_count > 10){
                                Explore_Mode = ExploreMode::GlobalSearching; 
                            }
                        }
                       
                    }                     
                    if(get_distance(global_planner_target_pose,current_pose) < 0.2){                                                
                        Explore_Mode = ExploreMode::GlobalSearching; // Return to local exploration planner                                  
                    }
                     

                break;  
                
                case ExploreMode::GlobalSearching:     
                    if(true){ 
                             goal_request_count = 0; 
                            // block control command from local planner
                            pose_at_request = current_pose;

                        local_trj_enable = false;                                   
                        // wait for the new  path 
                        local_path_received = false;                                          

                        geometry_msgs::PoseStamped local_goal_tmp; 
                        local_goal_tmp.header.stamp = ros::Time::now();
                        local_goal_tmp.header.frame_id = "world";
                        global_planner_target_pose.position.x = local_target_x_;
                        global_planner_target_pose.position.y = local_target_y_;
                        global_planner_target_pose.position.z = init_takeoff_;
                        local_goal_tmp.pose = global_planner_target_pose;                            
                        local_goal_pub.publish(local_goal_tmp);
                        // local_target_send_ = false;        
                        ROS_INFO("Local traject send");
                        Explore_Mode = ExploreMode::LocalSearching;
                        }else{
                            ROS_INFO("Pls activate local target switch to initate local planner");
                            
                        }   
                break;  

                case ExploreMode::Homing:
                    ROS_INFO("invalid Homing mode, shouldn't be here");                  
                break;

                default:
                    ROS_INFO("ExploreMode default flag on..something wrong");
                break;     
            }                            
        break;

        case mainFSMmode::Avoidance:
            ROS_INFO("Main = %s",stateToString(mainFSM_mode)); 
            ROS_INFO("mainFSMmode::Avoidance = reset our goal");
            mainFSM_mode = mainFSMmode::Exploration;
            Explore_Mode = ExploreMode::GlobalSearching;
                                 
        break;

        case mainFSMmode::Detection:
            ROS_INFO("invalid Detection mode, shouldn't be here");   
        break;
        
        case mainFSMmode::Landing:
            ROS_INFO("invalid Landing mode, shouldn't be here");                                     
        break;

        default:
             ROS_INFO("invalid default mode, shouldn't be here");
        break;            
    }
    
    
}

void hmclFSM::mainFSMCallback(const ros::TimerEvent &event){   
    // if(!armed && !offboarded) return;     
    
    if(!check_if_drone_outside_global_box()){
        if(mainFSM_mode == mainFSMmode::Exploration){
            // Explore_Mode = ExploreMode::GlobalSearching;    
             pose_target_.position.z = init_takeoff_;  
            ROS_INFO("Drone is outside of global box --> Global Searching activated ");         
        }else if(mainFSM_mode == mainFSMmode::Avoidance){
            pose_target_.position.z = init_takeoff_;
                ROS_INFO("Avoidance::out side of global box area");
        }else if(mainFSM_mode == mainFSMmode::Detection){
            pose_target_.position.z = init_takeoff_;
                ROS_INFO("Detection::out side of global box area");
        }else if(mainFSM_mode == mainFSMmode::Landing){            
                ROS_INFO("Landing::out side of global box area");
        }
    }
    
    std_srvs::Trigger localsearch_srv;
    planner_msgs::pci_global plan_srv;       
    std_srvs::Trigger stop_srv;  
    std_srvs::Trigger homing_srv;
    double tmp_dist = 0.0;
    
    
    switch (mainFSM_mode)
    {
        case mainFSMmode::NA:
        ROS_INFO("Something is wrong...mainFSM is in NA mode");
        printFSMstate();
        mainFSM_mode = mainFSMmode::Init;        
        // Move_to_target_pose(target_pose);            
        break;

        case mainFSMmode::Init:       
            previous_mode = mainFSM_mode;                
            ROS_INFO("Initializing ");
            ROS_INFO("Main = %s",stateToString(mainFSM_mode));
            init_takeoff();
                if (fabs(current_pose.position.z - init_takeoff_) < 0.1){
                    ROS_INFO("Successful Take off");
                    mainFSM_mode = mainFSMmode::Exploration;     
                    Explore_Mode = ExploreMode::LocalSearching;                            
                }
        break;
        
        case mainFSMmode::Exploration:
            ROS_INFO("Main = %s, Sub = %s",stateToString(mainFSM_mode),stateToString(Explore_Mode));
            previous_mode = mainFSM_mode;                              
            switch (Explore_Mode){
                case ExploreMode::NA:                
                ROS_INFO("Explore NA");                    
                break;  
                
                case ExploreMode::LocalSearching:                                                          
                    if (!explore_client_start.call(localsearch_srv)) {
                        // If we cannot get any path from local exploration planner,  let's first stop  
                        ROS_ERROR("[MBPLANNER-UI] Service call failed: %s",explore_client_start.getService().c_str());                                                                    
                        if (!explore_client_stop.call(stop_srv)) {
                            ROS_ERROR("[MBPLANNER-UI] MBplanner Stop Service call failed: %s",explore_client_stop.getService().c_str());
                            mainFSM_mode=mainFSMmode::NA; // If stop is not woking, we are moving into emergency phase.
                            }else{ // And Change state to global searching
                                Explore_Mode =ExploreMode::GlobalSearching;}
                    }
                    // tmp_dist = get_distance(previous_pose,current_pose);
                    // if (tmp_dist < 0.2 && localsearch_count > 3){
                    //     Explore_Mode =ExploreMode::GlobalSearching;
                    //     if (!explore_client_stop.call(stop_srv)) {
                    //         ROS_ERROR("[MBPLANNER-UI] MBplanner Stop Service call failed: %s",explore_client_stop.getService().c_str());
                    //         mainFSM_mode=mainFSMmode::NA; // If stop is not woking, we are moving into emergency phase.
                    //     }
                    //     localsearch_count = 0;    
                    // }
                    // if(tmp_dist > 0.2){
                    //     localsearch_count = 0;
                    // }
                    previous_pose = current_pose;
                    localsearch_count+=1;
                break;  
                
                case ExploreMode::GlobalSearching:     
                    
                    plan_srv.request.id = 0; // Find the best frontier otherwise go to home 
                    if (!explore_client_global_planner.call(plan_srv)){
                        ROS_ERROR("[MBPLANNER-UI] Service call failed: %s",explore_client_global_planner.getService().c_str());
                        Explore_Mode = ExploreMode::Homing;
                        return;
                    }else{
                        if(plan_srv.response.path_empty){
                            Explore_Mode = ExploreMode::Homing;
                        }else{
                        global_planner_target_pose.position.x = plan_srv.response.target_pose[0];
                        global_planner_target_pose.position.y = plan_srv.response.target_pose[1];
                        global_planner_target_pose.position.z = plan_srv.response.target_pose[2];
                        }
                    }
                     
                    if(get_distance(global_planner_target_pose,current_pose) < 0.2){                        
                        ROS_INFO("global planner waypoints reached");
                        Explore_Mode = ExploreMode::LocalSearching; // Return to local exploration planner          
                        localsearch_count = 0;
                    }                    
                    
                break;  

                case ExploreMode::Homing:
                    if (!explore_client_home.call(homing_srv)) {
                            ROS_ERROR("[MBPLANNER-UI] Service call failed: %s",explore_client_home.getService().c_str());
                            }                    
                break;

                default:
                ROS_INFO("ExploreMode default flag on..something wrong");
                break;     
            }                            
        break;

        case mainFSMmode::Avoidance:
            ROS_INFO("Main = %s",stateToString(mainFSM_mode));    
            ROS_INFO("previous mode= %s", stateToString(previous_mode)); 
            // Stop the manuaver depends on the previous mode 
            // e.g. if the previous mode is exploration --> send stop signal to planner first 
            if(previous_mode == mainFSMmode::Exploration){
                if (!explore_client_stop.call(stop_srv)) {
                            ROS_ERROR("[MBPLANNER-UI] MBplanner Stop Service call failed: %s",explore_client_stop.getService().c_str());
                            mainFSM_mode=mainFSMmode::NA; // If stop is not woking, we are moving into emergency phase.
                            }
            }
            
            if(previous_mode == mainFSMmode::Avoidance){
                mainFSM_mode=mainFSMmode::Init; 
            }
            else{
                mainFSM_mode = previous_mode;                         
                avoidance_enable = false;
            }
            
            
            // tmp_dist = get_distance(target_pose,current_pose);
            
            // if(tmp_dist > lidar_avoidance_distance_-0.1){                
            //     ROS_INFO("Avoid obstacle~!");
            // }else{
               
            // }                        
        break;

        case mainFSMmode::Detection:
            previous_mode = mainFSM_mode;
            ROS_INFO("Main = %s, Sub = %s",stateToString(mainFSM_mode),stateToString(Detect_Mode));                
            switch (Detect_Mode){
                case DetectMode::NA:                
                ROS_INFO("DetectMode NA");                    
                break;  
                
                case DetectMode::Filter:                                    
                Detect_Mode = DetectMode::Register;
                break;  
                
                case DetectMode::Register:                                    
                mainFSM_mode = mainFSMmode::Landing;           
                Land_Mode = LandMode::LookingHelipad;
                break;  

                default:
                ROS_INFO("Detection default flag on..something wrong");
                break;     
            }             
        break;
        
        case mainFSMmode::Landing:
            previous_mode = mainFSM_mode;
            ROS_INFO("Main = %s, Sub = %s",stateToString(mainFSM_mode),stateToString(Land_Mode));                
            switch (Land_Mode){
                case LandMode::NA: 
                ROS_INFO("LandMode NA");                                                 
                break;  
                
                case LandMode::LookingHelipad:                                    
                Land_Mode = LandMode::HomeApproach;
                break;  

                case LandMode::HomeApproach:                                    
                Land_Mode = LandMode::TargetApproach;
                break; 

                case LandMode::TargetApproach:                                    
                Land_Mode = LandMode::Landing;
                break;                     

                case LandMode::Landing:                                    
                Land_Mode = LandMode::Takeoff;
                break; 

                case LandMode::Takeoff:                                    
                mainFSM_mode = mainFSMmode::NA;         
                Explore_Mode = ExploreMode::NA;
                Detect_Mode = DetectMode::NA;
                Land_Mode = LandMode::NA;
                printFSMstate();
                break;  

                default:
                ROS_INFO("Detection default flag on..something wrong");
                break;     
            }                         
        break;

        default:
        ROS_INFO("default");
        break;            
    }
    
    
}


void hmclFSM::printFSMstate(){
    ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    ROS_INFO("Print out all state mode");
    ROS_INFO("mainFSM_mode = %s", stateToString(mainFSM_mode));    
    ROS_INFO("mainFSM_mode = %s", stateToString(Explore_Mode));    
    ROS_INFO("mainFSM_mode = %s", stateToString(Detect_Mode));    
    ROS_INFO("mainFSM_mode = %s", stateToString(Land_Mode));  
    ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
}

double hmclFSM::get_distance(geometry_msgs::Pose &p1,geometry_msgs::Pose &p2){
    double dist = hypot(hypot(p1.position.x-p2.position.x,p1.position.y-p2.position.y),p1.position.z-p2.position.z);
        return dist;
}

bool hmclFSM::check_if_drone_outside_global_box(){
     if(!odom_received){return true;}
    if(current_pose.position.x < global_pose_x_min || current_pose.position.x > global_pose_x_max){
        return false;
    }
     if(current_pose.position.y < global_pose_y_min || current_pose.position.y > global_pose_y_max){
        return false;
    }
     if(current_pose.position.z < global_pose_z_min || current_pose.position.z > global_pose_z_max){
        return false;
    }
    return true;
}

void hmclFSM::poseCmdCallback(const quadrotor_msgs::PositionCommandConstPtr &msg){
    
    tmp_target_.yaw = msg->yaw;
    if (msg->yaw_dot > yaw_rate_max){
        tmp_target_.yaw_rate = yaw_rate_max;        
    }else{
        tmp_target_.yaw_rate = msg->yaw_dot;
    }

    if (msg->yaw_dot < -1*yaw_rate_max){
        tmp_target_.yaw_rate = -1*yaw_rate_max;        
    }else{
        tmp_target_.yaw_rate = msg->yaw_dot;
    }
    // ROS_INFO("pose cmd recied");
    if(manual_trj_switch_ || !local_trj_enable)
    {   
        ROS_INFO("local planner command ignored");
        return;
    }
    
    pose_target_.header.stamp = ros::Time::now();
    pose_target_.header.frame_id ='c';        
    pose_target_.coordinate_frame = 1; // mavros_msgs::PositionTarget::FRAME_LOCAL_NED;         
    
    pose_target_.type_mask = 0;
    pose_target_.position.x = msg->position.x; 
    pose_target_.position.y = msg->position.y; 
    pose_target_.position.z = msg->position.z; 

    


    if( msg->position.z > global_pose_z_max){
         pose_target_.position.z = global_pose_z_max;
    }    
    
    pose_target_.velocity.x = msg->velocity.x; 
    pose_target_.velocity.y = msg->velocity.y; 
    pose_target_.velocity.z = msg->velocity.z; 

    pose_target_.acceleration_or_force.x = msg->acceleration.x;
    pose_target_.acceleration_or_force.y = msg->acceleration.y;
    pose_target_.acceleration_or_force.z = msg->acceleration.z; 

    pose_target_.yaw = msg->yaw;
    if (msg->yaw_dot > yaw_rate_max){
        pose_target_.yaw_rate = yaw_rate_max;        
    }
    if (msg->yaw_dot < -1*yaw_rate_max){
        pose_target_.yaw_rate = -1*yaw_rate_max;        
    }

}

void hmclFSM::bbxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg){
    if(msg->bounding_boxes.size() > 0){        
        detected_bbx = *msg;
        // for(int i=0; i < detected_bbx.bounding_boxes.size();i++){           
        //     detected_bbx.bounding_boxes[i].z
        // }
    }
 }

void hmclFSM::dyn_callback(const hmcl_fsm::dyn_paramsConfig &config, uint32_t level) {  
            d0 = config.d0;
            k0 = config.k0;
            thrust_scale = config.thrust_scale;
            lidar_min_threshold = config.lidar_min_threshold;
            lidar_avoidance_distance_ = config.lidar_avoidance_distance;
            lidar_avoidance_move_distance_ = config.lidar_avoidance_move_distance;
            manual_trj_switch_ = config.manual_trj_switch;
            target_x=config.target_x;
            target_y=config.target_y;
            target_z=config.target_z;
            target_yaw = config.target_yaw;
            waypoint_switch_ = config.waypoint_switch;
            local_avoidance_switch_ = config.local_avoidance_switch;
            // local_trj_switch_ = config.local_trj_switch;
            // local_target_send_ = config.local_target_send;
            local_target_x_ = config.local_target_x;
            local_target_y_ = config.local_target_y;
            landing_switch_ = config.landing_switch;
            wall_r_follow = config.wall_r_follow;
            wall_l_follow = config.wall_l_follow;
            
            
            // ROS_INFO("d0 = %f, k= %f, thrust_scale = %f", d0,k0,thrust_scale);            
}


void hmclFSM::local_avoidance(double min_distance){
    ROS_INFO("local avoidance activated");
    //TODO - using 2d lidar data, send avoidance manuever cmd to drone     
	float avoidance_vector_x = 0; 
	float avoidance_vector_y = 0;
	bool avoid = false;	
    
    for (double angle_tmp=-3.14195; angle_tmp < 3.14195;  angle_tmp +=lidar_data.angle_increment){
        if(angle_tmp < lidar_data.angle_min || angle_tmp > lidar_data.angle_max){
            float x = cos(angle_tmp);
			float y = sin(angle_tmp);
			float U = -0.5*k0*pow(((1/(min_distance*1.4)) - (1/d0)), 2);	
			avoidance_vector_x = avoidance_vector_x + x*U;
			avoidance_vector_y = avoidance_vector_y + y*U;
        }            
    } 
    
    for(int i=0; i<lidar_data.ranges.size(); i++)
	{    
		if( lidar_data.ranges[i] > lidar_min_threshold)
		{   double tmp_dist = lidar_data.ranges[i];
            if (tmp_dist > d0){
                tmp_dist = d0;
            }
			avoid = true;
			float x = cos(lidar_data.angle_increment*i+lidar_data.angle_min);
			float y = sin(lidar_data.angle_increment*i+lidar_data.angle_min);
			float U = -0.5*k0*pow(((1/tmp_dist) - (1/d0)), 2);	

			avoidance_vector_x = avoidance_vector_x + x*U;
			avoidance_vector_y = avoidance_vector_y + y*U;
		}        
	}	
    
	avoidance_vector_x = avoidance_vector_x*cos(current_yaw) - avoidance_vector_y*sin(current_yaw);
	avoidance_vector_y = avoidance_vector_x*sin(current_yaw) + avoidance_vector_y*cos(current_yaw);
    
   
    if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > lidar_avoidance_move_distance_)
		{
			avoidance_vector_x = lidar_avoidance_move_distance_ * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
            avoidance_vector_y = lidar_avoidance_move_distance_ * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));    
		}

	if(avoid && local_avoidance_switch_)
	{   		

        pose_target_.header.stamp = ros::Time::now();
        pose_target_.header.frame_id ='c';    
        pose_target_.coordinate_frame = 1;       
        if((avoidance_vector_x + current_pose.position.x)  < global_pose_x_min || (avoidance_vector_x + current_pose.position.x)  > global_pose_x_max){
        avoidance_vector_x = 0.0;
        }
         if((avoidance_vector_y + current_pose.position.y)  < global_pose_y_min || (avoidance_vector_y + current_pose.position.y)  > global_pose_y_max){
        avoidance_vector_y = 0.0;
        }
        pose_target_.position.x =  avoidance_vector_x + current_pose.position.x;        
        pose_target_.position.y =  avoidance_vector_y + current_pose.position.y;
        if(current_pose.position.z < init_takeoff_ -0.5|| current_pose.position.z > global_pose_z_max-0.5){
            pose_target_.position.z = init_takeoff_;    
        }else{
            pose_target_.position.z =  current_pose.position.z;    
        }        
        // pose_target_.yaw = current_yaw;
        // pose_target_.yaw_rate = 0.0;
        pose_target_.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | 
                                     mavros_msgs::PositionTarget::IGNORE_VX  | 
                                     mavros_msgs::PositionTarget::IGNORE_VY  | 
                                     mavros_msgs::PositionTarget::IGNORE_VZ;
              
        avoidance_enable = true;
        target_pose.position.x = pose_target_.position.x;
        target_pose.position.y = pose_target_.position.y;
        target_pose.position.z = pose_target_.position.z;
        
        //
        pose_target_.yaw =  tmp_target_.yaw;
        pose_target_.yaw_rate = tmp_target_.yaw_rate;
       

        // Disable local trajectory follower if enabled
        // if(local_trj_switch_){
        //     local_trj_switch_ = false;     
            
        // }
        local_trj_enable = false;   
        
        
        if(waypoints.points.size() > 0 ){
        refine_path_via_lidarData();        
        }          
	}
    
    // ROS_INFO("Avoidance vector x = %f, y = %f", avoidance_vector_x, avoidance_vector_y);  
       
    // and refine trajectory to follow if there is any     
    
    
}


void hmclFSM::refine_path_via_lidarData(){
    // delete below as this is for lidar testing
    waypoints.points.clear();
    command_waiting_times_.clear();
    waypoints_itr = 0;
    //TODO - refine path using the 2d lidar data && current pose     
    ROS_INFO("trajectory has been refined");
}

void hmclFSM::check_drone_status(){
    //TODO - update Mode status to identify the required manuever of drone
    // ROS_INFO("check mode")    
    send_waypoint = false;
    // follow the local avoidance pose control     
    // if(avoidance_enable){        
    //     mpc_cmd_enable = false;
    // }else{        
    //     mpc_cmd_enable = true;
    // }    
    // if(manual_trj_switch_){
    //     sendManualTrajectory();                
    // }
    return;
}

void hmclFSM::lidarTimeCallback(const ros::TimerEvent& e){
    // if (mainFSM_mode == mainFSMmode::Init){
    //     return;
    // }
    if (lidar_data.ranges.size() < 1){
        return;
    }
    //If the minimum distance is less than threshold -> activate local_avoidance    
    int size = lidar_data.ranges.size();
    int minIndex = 0;
    double minval = 999;
    for(int i = 0; i < lidar_data.ranges.size(); i++){        
        if ( lidar_data.ranges[i] <= minval && lidar_data.ranges[i] > lidar_min_threshold){
            minval = lidar_data.ranges[i];
            minIndex = i;
        }                
    }
    //Average the neighboring signals to reject any noisy signal around the minimum point
    // int avg_half_width = 5;    
    // double dist_tmp = -0.01;
    // int qq = 0;
    // for(int k = minIndex-avg_half_width; k < minIndex + avg_half_width ; k++)
    // {   
    //     if (k > lidar_data.ranges.size())
    //          qq = k -lidar_data.ranges.size();
    //     else if(k < 0)
    //         qq = k+lidar_data.ranges.size();
    //     else
    //         qq = k;
        
    //     if (lidar_data.ranges[qq] < 100){
    //             dist_tmp = dist_tmp + lidar_data.ranges[qq];           
    //     }else{
    //         dist_tmp = dist_tmp + 100;           
    //     }
    // }
    //  ROS_INFO("minval = %f",minval);  
    //  ROS_INFO("dist_tmp  = %f",dist_tmp);       
    // if (dist_tmp < 0){ dist_tmp = 100*avg_half_width;}
    // dist_tmp = dist_tmp / (2*avg_half_width);   
    
    // ROS_INFO("pose_dist_tmp  = %f",dist_tmp);  
    // ROS_INFO_STREAM("minimum distance (2d lidar) = " << dist_tmp);
    // ROS_INFO_STREAM(" = " << lidar_avoidance_distance_);
    if(minval < lidar_avoidance_distance_){
        ROS_INFO("Warning!! --> Obstacles approaching");        
        mainFSM_mode = mainFSMmode::Avoidance;
        local_avoidance(minval);                
    }else{
        avoidance_enable = false;
        
    }
}

void hmclFSM::lidarCallback(const sensor_msgs::LaserScanConstPtr &msg){
    lidar_data = *msg;       
}



void hmclFSM::cmdloopCallback(const ros::TimerEvent &event) {   
    if(landing_switch_){
        pose_target_.position.x = current_pose.position.x; 
        pose_target_.position.y = current_pose.position.y; 
        pose_target_.position.z = current_pose.position.z-0.3;   
        position_target_pub.publish(pose_target_);   
        ROS_INFO("emergency landing actiavted");
        return;
    }

    if(manual_trj_switch_){
        sendManualTrajectory();                
    }   
    position_target_pub.publish(pose_target_);   
//    tf_broadcaster_.sendTransform(transformStamped_tmp); 
  
    // tf_broadcaster.sendTransform(tf::StampedTransform(cam_to_world_tf, ros::Time::now(), "world", "camera_link2"));
    // try {      
    //   tf_listener_.waitForTransform("/world", "/camera",ros::Time::now() ,ros::Duration(1.0));
    //   tf_listener_.lookupTransform("/world", "/camera", ros::Time::now() ,cam_to_world);
    //   cam_to_world_tf.setBasis(cam_to_world.getBasis());
    //   cam_to_world_tf.setOrigin(cam_to_world.getOrigin());
      
    // } catch (tf::TransformException& ex) {
    //         ROS_INFO("no such transform from camera to world");   
            
    // }  
    
    
    
    // tf_broadcaster.sendTransform(tf::StampedTransform(cam_to_world_tf, ros::Time::now(), "world", "camera_link"));
    // ROS_INFO("~");
    // if (avoidance_enable){
    //     // Lidar based local avoidance activated
    //      position_target_pub.publish(pose_target_);         
    //      return; 
    // }


    
    
    // if (mpc_cmd_enable){                
    //     // mpc_cmd_pub.publish(mpc_cmd);
    //     rpyt_pub.publish(att);
        
    //     return;
    // }
    
    // position_target_pub.publish(pose_target_);  
    // ROS_INFO("position target pub");  
     return;
}


void hmclFSM::sendManualTrajectory(){
    // waypoints.points.clear();    
    // waypoints_itr = 0;
    pose_target_.header.stamp = ros::Time::now();
    pose_target_.header.frame_id ='map';        
    pose_target_.coordinate_frame = 1; // mavros_msgs::PositionTarget::FRAME_LOCAL_NED;            
    pose_target_.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | 
                                mavros_msgs::PositionTarget::IGNORE_VX  | 
                                mavros_msgs::PositionTarget::IGNORE_VY  | 
                                mavros_msgs::PositionTarget::IGNORE_VZ;
    // mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;            
    pose_target_.position.x = target_x;
    pose_target_.position.y = target_y;
    pose_target_.position.z = target_z;                         
    pose_target_.yaw = target_yaw;            
    pose_target_.yaw_rate = 0.1;
    // position_target_pub.publish(pose_target_);   
    // waypoints.points.clear();    
}



void hmclFSM::waypointTimerCallback(const ros::TimerEvent &event){   
    // ROS_INFO("command_waiting_times_ = %lf" , command_waiting_times_.front().toSec());
    // ROS_INFO("waypoints_itr = %d", waypoints_itr);
    
    if(!waypoint_switch_ || manual_trj_switch_){
        return;
    }
    
    if (waypoints.points.size() > 0){  
        
        if (waypoints_itr >= 0 && waypoints_itr < waypoints.points.size()){            
        
            pose_target_.header.stamp = ros::Time::now();
            pose_target_.header.frame_id ='map';        
            pose_target_.coordinate_frame = 1; // mavros_msgs::PositionTarget::FRAME_LOCAL_NED;            
            pose_target_.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | 
                                     mavros_msgs::PositionTarget::IGNORE_VX  | 
                                     mavros_msgs::PositionTarget::IGNORE_VY  | 
                                     mavros_msgs::PositionTarget::IGNORE_VZ;
            // mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;            
            pose_target_.position.x = waypoints.points[waypoints_itr].transforms[0].translation.x;
            pose_target_.position.y = waypoints.points[waypoints_itr].transforms[0].translation.y;
            pose_target_.position.z = waypoints.points[waypoints_itr].transforms[0].translation.z;               
            tf::Quaternion q(waypoints.points[waypoints_itr].transforms[0].rotation.x,waypoints.points[waypoints_itr].transforms[0].rotation.y,waypoints.points[waypoints_itr].transforms[0].rotation.z,waypoints.points[waypoints_itr].transforms[0].rotation.w);
            tf::Matrix3x3 m(q);            
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);        
            pose_target_.yaw = yaw;            
            pose_target_.yaw_rate = 0.1;
            position_target_pub.publish(pose_target_);   
            send_waypoint = true;            
        }       
    } 
        waypoint_iter_timer_.stop();       
    if(!command_waiting_times_.empty()){           
        waypoint_iter_timer_.setPeriod(command_waiting_times_.front());
        command_waiting_times_.pop_front();
        waypoint_iter_timer_.start();
        waypoints_itr++;
    }

}

void hmclFSM::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg) {  
    
    if (msg->points.size() > 1){  
            int lookahead_idx = 3;
            float min_dist = 0;
            float min_idx = 1;    
        // if size is greather than lookhaed idx, Find the closest point from current position and propogate only the points foreahead.  
            if (msg->points.size() > lookahead_idx){        
                for (int i=1; i < msg->points.size();i++){                    
                    float tmp_dist = hypot(hypot(msg->points[i].transforms[0].translation.x-current_pose.position.x,
                                                msg->points[i].transforms[0].translation.y-current_pose.position.y),
                                        msg->points[i].transforms[0].translation.z-current_pose.position.z);
                        if(tmp_dist <= min_dist){
                            min_dist = tmp_dist;
                            min_idx = i;
                        }                 
                }     
                if (min_idx +lookahead_idx > msg->points.size()){
                    min_idx = 1;
                }else{
                    min_idx += lookahead_idx;
                }
            }    
            waypoint_iter_timer_.stop();
            waypoints.points.clear();
            command_waiting_times_.clear();
            //Replace trajectory starting from lookaheaded points
            for (int i=min_idx-1;i < msg->points.size(); i++){
                    waypoints.points.push_back(msg->points[i]);
            }
                
            //   waypoints.points = msg->points;    
                // ROS_INFO("Trjectory length = %d",msg->points.size());
                waypoints_itr = 0; 

                // ROS_INFO("trejactory size = %d", msg->points.size());
                // ROS_INFO("the closest point + lookahead at trajectory index %d", min_idx);
                for (int i=min_idx-1 ; i < msg->points.size(); i++){      
                    command_waiting_times_.push_back(msg->points[i].time_from_start  - msg->points[i-1].time_from_start);
                    // ROS_INFO("push time = %lf",double((msg->points[i].time_from_start  - msg->points[i-1].time_from_start).toSec()));
                }
                waypoint_iter_timer_.setPeriod(command_waiting_times_.front());
                command_waiting_times_.pop_front();
                waypoint_iter_timer_.start();
                ROS_WARN_STREAM("Got MultiDOFJointTrajectory message");
        }
        else{
            ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
        }
    
}




void hmclFSM::odom_cb(const nav_msgs::OdometryConstPtr& msg){
    
    odom_state.pose = msg->pose;
    current_pose = msg->pose.pose;
    tf::Quaternion q(current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w);
    tf::Matrix3x3 m(q);            
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);        
    current_yaw = yaw;
    geometry_msgs::TransformStamped transformStamped;
    static tf2_ros::TransformBroadcaster br;  
    // transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.stamp = msg->header.stamp;
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;    
    transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;
    br.sendTransform(transformStamped);   
    if(!odom_received){
        previous_pose = current_pose;
        odom_received = true;
        } 
}


void hmclFSM::visCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    
    vis_pose.header = msg->header;
    vis_pose.header.frame_id = "world";
    vis_pose.child_frame_id = "base_link";
    vis_pose.transform.translation.x = msg->pose.position.x;
    vis_pose.transform.translation.y = msg->pose.position.y;
    vis_pose.transform.translation.z = msg->pose.position.z;
    vis_pose.transform.rotation.x = msg->pose.orientation.x;
    vis_pose.transform.rotation.y = msg->pose.orientation.y;
    vis_pose.transform.rotation.z = msg->pose.orientation.z;
    vis_pose.transform.rotation.w = msg->pose.orientation.w;
    vis_pos_pub.publish(vis_pose);
    
    vins_odom_state.header = msg->header;
    vins_odom_state.header.frame_id = "world";
    vins_odom_state.child_frame_id = "base_link";
    vins_odom_state.pose.pose.position.x =  msg->pose.position.x;
    vins_odom_state.pose.pose.position.y = msg->pose.position.y;
    vins_odom_state.pose.pose.position.z =  msg->pose.position.z;
    vins_odom_state.pose.pose.orientation.x = msg->pose.orientation.x;
    vins_odom_state.pose.pose.orientation.y = msg->pose.orientation.y;
    vins_odom_state.pose.pose.orientation.z = msg->pose.orientation.z;
    vins_odom_state.pose.pose.orientation.w = msg->pose.orientation.w;
    vins_odom_pub.publish(vins_odom_state);
      
}

void hmclFSM::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}



int main(int argc, char** argv){
    ros::init(argc, argv, "hmcl_fsm");
    ros::NodeHandle nh;
    ros::NodeHandle cmd_nh_;
    ros::NodeHandle fsm_nh_;
    ros::NodeHandle lidar_nh_;
    ros::NodeHandle odom_nh_;
    ros::NodeHandle nh_private("~");   
      

    ros::CallbackQueue callback_queue_cmd;
    cmd_nh_.setCallbackQueue(&callback_queue_cmd);
    
    ros::CallbackQueue callback_queue_fsm;
    fsm_nh_.setCallbackQueue(&callback_queue_fsm);

    ros::CallbackQueue callback_queue_lidar;
    lidar_nh_.setCallbackQueue(&callback_queue_lidar);

    ros::CallbackQueue callback_queue_odom;
    odom_nh_.setCallbackQueue(&callback_queue_odom);
    
    hmclFSM hmcl_FSM(nh,nh_private,cmd_nh_,fsm_nh_,lidar_nh_,odom_nh_);        


    std::thread spinner_thread_cmd([&callback_queue_cmd]() {
    ros::SingleThreadedSpinner spinner_cmd;
    spinner_cmd.spin(&callback_queue_cmd);
    });

    std::thread spinner_thread_fsm([&callback_queue_fsm]() {
    ros::SingleThreadedSpinner spinner_fsm;
    spinner_fsm.spin(&callback_queue_fsm);
    });

    std::thread spinner_thread_lidar([&callback_queue_lidar]() {
    ros::SingleThreadedSpinner spinner_lidar;
    spinner_lidar.spin(&callback_queue_lidar);
    });

    std::thread spinner_thread_odom([&callback_queue_odom]() {
    ros::SingleThreadedSpinner spinner_odom;
    spinner_odom.spin(&callback_queue_odom);
    });


    ros::spin();
    spinner_thread_cmd.join();
    spinner_thread_fsm.join();
    spinner_thread_lidar.join();
    spinner_thread_odom.join();
    
    return 0;
}