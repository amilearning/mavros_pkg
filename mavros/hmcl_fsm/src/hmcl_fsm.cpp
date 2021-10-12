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
    
    cali_done = false;
    rtb = false;
    rtb_once = false;
    landing = false;
    init_count =0;
    wall_l_follow = false;
    emergency_landing_count = 0;
    global_goal_dir=0.0;
    prev_global_goal_dir=0.0;

    // d0 = 3.0;
    // k0 = 0.004;
    odom_received = false;  // check odom is available 
    ndt_fused_pose_listen = false; // check ndt is working 
    wall_r_follow = false;
    
    lidar_sub = lidar_nh_.subscribe<sensor_msgs::LaserScan>("/scan",5,&hmclFSM::lidarCallback,this);    

    odom_sub = odom_nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",10, &hmclFSM::odom_cb,this);  
    ndt_fused_pose_sub = odom_nh_.subscribe<geometry_msgs::TransformStamped>("/ndt_fused_pose",10, &hmclFSM::ndt_fused_cb,this);  
    battery_state_sub = odom_nh_.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 10, &hmclFSM::battery_state_cb,this);
   
    vision_odom_sub = odom_nh_.subscribe<geometry_msgs::PoseStamped>("/vins/px4/pose",10, &hmclFSM::visCallback,this);    
   
    cmdloop_timer_ = cmd_nh_.createTimer(ros::Duration(0.02), &hmclFSM::cmdloopCallback,this); // Critical -> allocate another thread 
    
    if(FSM_mode == 0){
        // activate exploartion fsm
        fsm_timer_ = fsm_nh_.createTimer(ros::Duration(0.02), &hmclFSM::mainFSMCallback,this); 
        wallfollower_r_sub = nh_.subscribe<mavros_msgs::PositionTarget>("/mapfollow/target/pose", 10, &hmclFSM::wallFollowCmdCallback_r,this);    
        rtb_pub = fsm_nh_.advertise<std_msgs::Bool>("/rtb",1);
        // wallfollower_l_sub = nh_.subscribe<mavros_msgs::PositionTarget>("/mapfollow/target/pose_l", 10, &hmclFSM::wallFollowCmdCallback_l,this);                
    }else if(FSM_mode ==1 ) {
        // activate local traj fsm
        fsm_timer_ = fsm_nh_.createTimer(ros::Duration(0.02), &hmclFSM::localFSMCallback,this); 
        local_path_trigger_sub = fsm_nh_.subscribe<std_msgs::Empty>("/planning/new",1,&hmclFSM::localTrajTrigCallback,this);
        local_avoidance_switch_pub = fsm_nh_.advertise<std_msgs::Bool>("/local_avoidance_switch",1);
        global_direction_sub = nh_.subscribe<std_msgs::Float32>("goal_direction",1,&hmclFSM::goaldirectionCallback,this);
        pos_cmd_sub = nh_.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10, &hmclFSM::poseCmdCallback,this);    
    }
    
    position_target_pub = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);    
    vis_pos_pub  =  nh_.advertise<geometry_msgs::TransformStamped>("/vins/posetransform", 10);    
    local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);    
    vins_odom_pub = nh_.advertise<nav_msgs::Odometry>("vins_odom", 10);
    local_goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
    avoidance_vector_vis_pub = nh_.advertise<geometry_msgs::PoseStamped>("pose_target", 1);
    state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &hmclFSM::state_cb,this);
    // Define service Clients 
    ekf_reinit_client = nh_.serviceClient<sensor_fusion_comm::InitScale>("/msf_viconpos_sensor/pose_sensor/initialize_msf_scale");     
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    
    px4_cmd_client = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  
    // initialize control method
    pose_cmd_enable = false;
    mpc_cmd_enable = false;
    send_waypoint = false;
    avoidance_enable = false;
    local_trj_enable = false;
    odom_received = false;

    ekf_init_param.request.scale = 1.0; 
    px4_reboot_cmd.request.broadcast = false;
    px4_reboot_cmd.request.command = 246;
    px4_reboot_cmd.request.confirmation = 1;
    px4_reboot_cmd.request.param1 = 1.0;

    px4_kill_cmd.request.command = 400;
    px4_kill_cmd.request.broadcast = false;
    px4_kill_cmd.request.confirmation = 1;
    px4_kill_cmd.request.param1 = 0.0;
    px4_kill_cmd.request.param2 = 21196.0;

    armed = false;
    offboarded = false;


    int check_count = 0;
     re_init = true;   

    offb_set_mode.request.custom_mode = "OFFBOARD";    
    arm_cmd.request.value = true;
    
    
    
    mainFSM_mode = mainFSMmode::Init;
    Explore_Mode = ExploreMode::LocalSearching;    
    Land_Mode =  LandMode::NearHome;
    RTB_mode = RTBmode::Init;
   

}

void hmclFSM::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

bool hmclFSM::offboard_and_arm(){      
        // if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
        //         {                    
        //             if( set_mode_client.call(offb_set_mode) ){
        //                 ROS_INFO("Offboard enabled");
        //                 offboarded = true;
        //                 }
        //             last_request = ros::Time::now();
        //         }   
                // } else {          
                    // if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0))){
                        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0))){
                        // if( arming_client.call(arm_cmd) && arm_cmd.response.success){ROS_INFO("Vehicle armed");}
                        if( arming_client.call(arm_cmd)){
                            if(verbos){ROS_INFO("Vehicle armed");}
                            armed = true;                            
                            }
                        last_request = ros::Time::now();
                    }
                // }
        // if(offboarded && armed)
        if( armed)
        // if(offboarded)
        { return true; }
        else{
            return false;
        }                
   
}

bool hmclFSM::init_cali(){
    if(!ndt_fused_pose_listen){
          return false;    
    }      
    if(re_init){            
            for(int i=0;i<3;i++){
                px4_cmd_client.call(px4_reboot_cmd);
            }            
            if( px4_cmd_client.call(px4_reboot_cmd)){                
                re_init = false;
            }else{                
                  return false;    }

            ros::Duration(2.0).sleep();           
                if( !ekf_reinit_client.call(ekf_init_param)){                        
                          return false;    }                    
                ros::Duration(2.0).sleep();    
            
    }
    //check distance between ndt_fused_pose and mavros local position
    double distance_between = sqrt( pow(current_pose.position.x-ndt_fused_pose.transform.translation.x,2)
                                     +pow(current_pose.position.y-ndt_fused_pose.transform.translation.y,2)
                                     +pow(current_pose.position.z-ndt_fused_pose.transform.translation.z,2)
                                        );
    if(distance_between<1e-1 && fabs(current_yaw) < 1e-1){
        pose_init_cali_succ_count++;        
        }else{
            pose_init_cali_succ_count = 0;            
            }  

    if(pose_init_cali_succ_count > 10){        
        return true;
        }            
        check_count++;        
            ros::Duration(0.5).sleep();        
        if (check_count > 60){            
            re_init = true;
            check_count = 0;            
        }
    return false;
}

void hmclFSM::ndt_fused_cb(const geometry_msgs::TransformStampedConstPtr& msg){
    if(!ndt_fused_pose_listen){ndt_fused_pose_listen=true;}
    ndt_fused_pose = *msg;
}

// void hmclFSM::wallFollowCmdCallback_l(const mavros_msgs::PositionTargetConstPtr &msg){  
//     if(avoidance_enable){
//         return;
//     }    
//     if(wall_l_follow){
//         pose_target_ = *msg;        
//     }
//     return;
// }

void hmclFSM::wallFollowCmdCallback_r(const mavros_msgs::PositionTargetConstPtr &msg){
    if(avoidance_enable){
        return;
    }
    if(rtb){
        return;
    }
    if(wall_r_follow){
        pose_target_ = *msg;  
    }
    
    return ;
}

bool hmclFSM::check_if_stuck(){
    
    
    double dist_to_check =  sqrt(
        pow(current_pose.position.x- prev_pose_for_check.position.x,2)
        +pow(current_pose.position.y-prev_pose_for_check.position.y ,2));       
           stuck_count++;
        if(stuck_count > 100){
            stuck_count = 0;
            return true;
        }
        if( dist_to_check > 0.3){            
            stuck_count = 0;
            prev_pose_for_check = current_pose;
        }
                
        return false;
}

void hmclFSM::localTrajTrigCallback(const std_msgs::EmptyConstPtr &msg){
    // ROS_INFO("trjectory received");
    local_path_received = true;    
}

void hmclFSM::load_FSM_Params(std::string group){
    nh_private_.getParam("FSM_mode",FSM_mode);
    nh_private_.getParam("verbos",verbos);
    nh_private_.getParam("total_run_distance",total_run_distance);   
    nh_private_.getParam("safe_distance",safe_distance);   
    nh_private_.getParam("D_sp",D_sp); 
    nh_private_.getParam("battery_thres",battery_thres); 
    
     
    // nh_private_.getParam("d0",d0);
    // nh_private_.getParam("k0",k0);    
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


void hmclFSM::goaldirectionCallback(const std_msgs::Float32ConstPtr &msg){
    global_goal_dir =  msg->data;
}


void hmclFSM::init_takeoff(){
    if(!odom_received){
        if(verbos){ROS_INFO("odom is not availble");}
        return;
    }    
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

void hmclFSM::check_emergency_landing(){
    
    double emergency_dist_check =  sqrt(
        pow(ndt_fused_pose.transform.translation.x-current_pose.position.x ,2)
        +pow(ndt_fused_pose.transform.translation.y-current_pose.position.y ,2));
    
    if(emergency_dist_check > 5.0){
        emergency_landing_count++;
        if(emergency_landing_count > 20){
            if(verbos){ROS_INFO("Emergency LANDING ACTIVATED");
            ROS_INFO("Emergency LANDING ACTIVATED");
            ROS_INFO("Emergency LANDING ACTIVATED");}
            landing = true;
        }        
        px4_cmd_client.call(px4_kill_cmd);
    }else{
        emergency_landing_count = 0;
    }
    
}


void hmclFSM::localFSMCallback(const ros::TimerEvent &event){            
    if(!cali_done){
        cali_done = init_cali();        
        return;
    } 
    if(!offboard_and_arm()){
        return;
    }

    switch (mainFSM_mode)
    {
        case mainFSMmode::Init:                   
            init_takeoff();
                if(init_count > 50){                    
                    mainFSM_mode = mainFSMmode::Exploration;     
                    LocalPlan_Mode = LocalPlanMode::WaypointRequest;
                     
                }
                if (fabs(current_pose.position.z - init_takeoff_) < 0.1){     
                    init_count ++;                      
                }else{
                  mainFSM_mode = mainFSMmode::Init;  
                }
               
                
        break;
        
        case mainFSMmode::Exploration:  
            if(verbos){ROS_INFO("Main = %s, Sub = %s",stateToString(mainFSM_mode),stateToString(LocalPlan_Mode));}                      
            switch (LocalPlan_Mode){                               
                case LocalPlanMode::Planning:{                           
                        // Send local target goal to local planner                                    
                    if(local_path_received){
                        std_msgs::Bool tmp_data_;
                        tmp_data_.data = false;
                        local_avoidance_switch_pub.publish(tmp_data_);
                        local_trj_enable = true;                        
                        local_path_received = false;                   
                    }  
                    if(global_goal_dir != prev_global_goal_dir){
                        prev_global_goal_dir =global_goal_dir; 
                        LocalPlan_Mode = LocalPlanMode::YawMatching; 
                        }                        
                break;  
                }
                
                case LocalPlanMode::YawMatching:{                      
                        local_trj_enable = false;    
                        pose_target_.yaw =   global_goal_dir; 
                        double tmp_target_yaw =global_goal_dir;      
                        angle_wrap(tmp_target_yaw);
                        if( fabs(current_yaw -tmp_target_yaw) < 0.1){
                        LocalPlan_Mode = LocalPlanMode::WaypointRequest;
                        }                                      
                break;  
                }

                case LocalPlanMode::WaypointRequest:{                      
                        goal_request_count = 0;                             
                        pose_at_request = current_pose;
                        local_trj_enable = false;                                   
                        // wait for the new  path 
                        local_path_received = false;                 
                        geometry_msgs::PoseStamped local_goal_tmp; 
                        local_goal_tmp.header.stamp = ros::Time::now();
                        local_goal_tmp.header.frame_id = "world";                        
                        global_planner_target_pose.position.z = init_takeoff_;
                        local_goal_tmp.pose = global_planner_target_pose;                            
                        local_goal_pub.publish(local_goal_tmp);                                                       
                        Explore_Mode = ExploreMode::LocalSearching;                         
                break;  
                }

                default:{
                 if(verbos){   ROS_INFO("LocalPlanMode default flag on..something wrong");}
                break; 
                }
            }               

        break;

        case mainFSMmode::Avoidance:                        
            mainFSM_mode = mainFSMmode::Exploration;                
            LocalPlan_Mode = LocalPlanMode::WaypointRequest;                                 
        break;
        
        case mainFSMmode::Landing:
            mainFSM_mode = mainFSMmode::Exploration;            
        break;
       
        case mainFSMmode::RTB:
            mainFSM_mode = mainFSMmode::Exploration;           
        break;

        default:{
             if(verbos){ROS_INFO("invalid default mode, shouldn't be here");}
        break;   
        }         
    }
    
    
}

void hmclFSM::mainFSMCallback(const ros::TimerEvent &event){   
    // if(!armed && !offboarded) return;     
    
     if(!cali_done){
        cali_done = init_cali();
        if(verbos){
            ROS_INFO("cali ongoing");
        }        
        return;
    } 

    if(!offboard_and_arm()){
        return;
    }
    

    if(rtb && !rtb_once && odom_received){
        if(wp_x.size() >= 0){            
        mainFSM_mode = mainFSMmode::RTB; 
        RTB_mode = RTBmode::Init;        
        wp_x.push_back(current_x);
        wp_y.push_back(current_y);
       rtb_once = true;       
        }                
    }
    if(rtb)
    {
        std_msgs::Bool rtb_tmp_data_;
        rtb_tmp_data_.data = true;
        rtb_pub.publish(rtb_tmp_data_);
    }

    if(!rtb && odom_received){
        STORE_WP();
    }  

    check_rtb_init();

    switch (mainFSM_mode)
    {
        case mainFSMmode::Init:                                             
            init_takeoff();
                if(init_count > 50){                    
                    mainFSM_mode = mainFSMmode::Exploration;     
                    Explore_Mode = ExploreMode::LocalSearching;   
                }
                if (fabs(current_pose.position.z - init_takeoff_) < 0.05){     
                    init_count ++;                                       
                }else{
                  mainFSM_mode = mainFSMmode::Init;  
                }
        break;
        

        
        case mainFSMmode::Exploration:
            if(verbos){ROS_INFO("Main = %s, Sub = %s",stateToString(mainFSM_mode),stateToString(Explore_Mode));}                                        
            switch (Explore_Mode){
                case ExploreMode::LocalSearching:                                                          
                    wall_r_follow = true;
                break;  
                
                case ExploreMode::GlobalSearching:     
                     wall_r_follow = true;                               
                break;  
                
                default:
                    Explore_Mode = ExploreMode::LocalSearching;
                break;     
            }                            
        break;

        case mainFSMmode::Avoidance:
            mainFSM_mode = mainFSMmode::Exploration;
            if(rtb){
                  mainFSM_mode = mainFSMmode::RTB; 
                    if(RTB_mode == RTBmode::Init){                      
                        RTB_mode = RTBmode::Init;                                               
                    }
                    else if(RTB_mode == RTBmode::Search){                                        
                        RTB_mode = RTBmode::Search; 
                    
                    }
                    else if(RTB_mode == RTBmode::YawAglign){                                       
                        RTB_mode = RTBmode::Search; 
                    }
                    else if(RTB_mode ==RTBmode::MoveNear){
                        RTB_mode = RTBmode::MoveNear;
                    }
                    else if(RTB_mode == RTBmode::Move){ 
                        // if(current_min_idx == prev_min_idx){
                        //     avoid_perpendicualar_factor++;
                        // }                                       
                        RTB_mode = RTBmode::Search; 
                    }
                    else if(RTB_mode == RTBmode::Init_yaw){                        
                        RTB_mode = RTBmode::Init_yaw; 
                    }else{                        
                        RTB_mode = RTBmode::Init; 
                    }
            }
            if(landing){
                mainFSM_mode = mainFSMmode::Landing;
                Land_Mode =  LandMode::MovetoHome;
            }                
        break;

               
        case mainFSMmode::Landing:    
            switch (Land_Mode){           
                case LandMode::NearHome:                                                        
                    pose_target_.yaw =   M_PI;      
                     if( fabs(current_yaw -M_PI) < 0.2){
                        Land_Mode = LandMode::MovetoHome;
                        }                   
                break; 

                case LandMode::MovetoHome:                                    
                    Land_Mode = LandMode::Landing;                    
                break;                     

                case LandMode::Landing:                                    
                   landing = true;
                break; 

                default:
                    mainFSM_mode = mainFSMmode::Landing;
                    Land_Mode =  LandMode::NearHome;                    
                break;     
            }                         
        break;

         case mainFSMmode::RTB:                        
            if(verbos){ROS_INFO("Main = %s, Sub = %s",stateToString(mainFSM_mode),stateToString(RTB_mode));}            
             switch (RTB_mode){
                case RTBmode::Init:{                                    
                    if(wp_x.size() > 0 && lidar_data.ranges.size() > 0){                         
                        current_min_idx = get_closest_wp_idx();                             
                        prev_min_idx =   current_min_idx;             
                        target_yaw = current_yaw - 3.14195;
                        angle_wrap(target_yaw);
                        RTB_mode = RTBmode::Init_yaw;                    
                    }               
                break;  
                }
               
                
                case RTBmode::Search:                
                     current_min_idx = get_safe_wp_idx();                     
                    if( current_min_idx != prev_min_idx){
                        RTB_mode = RTBmode::YawAglign;                       
                    }else{
                        if(  sqrt(pow(current_x-wp_x[current_min_idx],2)+ pow(current_y-wp_y[current_min_idx],2))< 0.2){    
                        RTB_mode = RTBmode::MoveNear;
                        }else{
                        RTB_mode = RTBmode::Move;    
                        }
                    }
                    prev_min_idx = current_min_idx;
                break;

                case RTBmode::YawAglign:                 
                        x_check = wp_x[current_min_idx];  y_check = wp_y[current_min_idx];                               
                        x_dc = x_check - current_x;   y_dc = y_check - current_y;                            
                        target_yaw = atan2(y_dc, x_dc);                                                                   
                        angle_wrap(target_yaw);                    
                        pose_target_.yaw =   target_yaw;      
                     if( fabs(current_yaw -target_yaw) < 0.3){
                        RTB_mode = RTBmode::Search;
                        }                        
                break;  
                
                case RTBmode::Move:   
                        pose_target_.position.x =   wp_x[current_min_idx];                             
                        pose_target_.position.y =   wp_y[current_min_idx];     
                        if(  sqrt(pow(current_x-wp_x[current_min_idx],2)+ pow(current_y-wp_y[current_min_idx],2))< 0.2){    
                            if(current_min_idx < 1){
                                    current_min_idx =1;
                                    mainFSM_mode = mainFSMmode::Landing;  
                                    Land_Mode = LandMode::NearHome;  
                                }
                                // x_check = wp_x[current_min_idx-1];  y_check = wp_y[current_min_idx-1];                               
                                // x_dc = x_check - current_x;   y_dc = y_check - current_y;    
                                // target_yaw = atan2(y_dc, x_dc);   
                                // angle_wrap(target_yaw);
                                // pose_target_.yaw = target_yaw;
                                // avoid_perpendicualar_factor = 0;    
                                RTB_mode = RTBmode::Search;     
                                                           
                        }else{                                  
                                x_check = wp_x[current_min_idx];  y_check = wp_y[current_min_idx];                               
                                x_dc = x_check - current_x;   y_dc = y_check - current_y;    
                                target_yaw = atan2(y_dc, x_dc);   
                                angle_wrap(target_yaw);
                                pose_target_.yaw = target_yaw;
                        }                                  

                break;   

                        
                case RTBmode::Init_yaw:                  
                    pose_target_.yaw = target_yaw;                    
                    if( fabs(current_yaw -target_yaw) < 0.1){
                        RTB_mode = RTBmode::Search;
                    }
                break;   

                
                case RTBmode::MoveNear:{
                        // if(  sqrt(pow(current_x-wp_x[current_min_idx],2)+ pow(current_y-wp_y[current_min_idx],2))< 0.2){    
                            if(current_min_idx < 1){
                                    current_min_idx =1;
                                    mainFSM_mode = mainFSMmode::Landing;  
                                    Land_Mode = LandMode::NearHome;
                                }                                
                                x_check = wp_x[current_min_idx-1];  y_check = wp_y[current_min_idx-1];                               
                                x_dc = x_check - current_x;   y_dc = y_check - current_y;    
                                target_yaw = atan2(y_dc, x_dc);   
                                angle_wrap(target_yaw);
                                pose_target_.yaw = target_yaw;
                               
                                if( fabs(current_yaw- target_yaw) < 0.1){
                                    RTB_mode = RTBmode::Search;     
                                }                                
                                                           
                break;
                }         

                default:{
                    if(verbos){ROS_INFO("RTB default flag on..something wrong");}
                break;   }  
            }                            
        break;

        default:{
        if(verbos){ROS_INFO("default");}
        break;            
        }
    }
    
    
}

int hmclFSM::get_safe_wp_idx(){
    if(lidar_data.ranges.size() < 1){
           return -1;
    }
    int rtb_idx = get_closest_wp_idx();       
    for( int i=rtb_idx; i >=0 ;i--){
        x_check = wp_x[i];  y_check = wp_y[i];      
        // target_yaw = atan2(y_dc, x_dc);  
        x_dc = x_check - current_x;   y_dc = y_check - current_y;    
        double angle_between = atan2(y_dc, x_dc) - current_yaw;
        angle_wrap(angle_between);
        if( angle_between > lidar_data.angle_min+M_PI_4|| angle_between < lidar_data.angle_max-M_PI_4)
        {               
            double expected = sqrt( pow(current_x-x_check,2) + pow(current_y-y_check,2) );    
            double lidar_idx = (int)((lidar_data.ranges.size()/2)+(angle_between/lidar_data.angle_increment));    
            double measured = lidar_data.ranges[lidar_idx];  
                if (measured > expected-1 && CHECK_CLEARANCE_AVOIDANCE(lidar_idx,expected))                                
                {    // found safe points                  
                    if(current_min_idx > i){
                    current_min_idx = i;
                    }
                }
            
        }
    }    
    return current_min_idx;
}

void hmclFSM::printFSMstate(){
    ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    ROS_INFO("Print out all state mode");
    ROS_INFO("mainFSM_mode = %s", stateToString(mainFSM_mode));    
    ROS_INFO("mainFSM_mode = %s", stateToString(Explore_Mode));            
    ROS_INFO("mainFSM_mode = %s", stateToString(Land_Mode));  
    ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
}

double hmclFSM::get_distance(geometry_msgs::Pose &p1,geometry_msgs::Pose &p2){
    double dist = hypot(hypot(p1.position.x-p2.position.x,p1.position.y-p2.position.y),p1.position.z-p2.position.z);
        return dist;
}

bool hmclFSM::check_if_drone_outside_global_box(){
     if(!odom_received){return true;}
   

    if(pose_target_.position.z < global_pose_z_min){
        pose_target_.position.z = global_pose_z_min+0.1;
        return false;
    }
    if(pose_target_.position.z > global_pose_z_max){
        pose_target_.position.z = global_pose_z_max-0.1;
        return false;
    }
    
    
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
    if(avoidance_enable){
        return;
    }
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
    if( msg->position.z < global_pose_z_min){
         pose_target_.position.z = global_pose_z_min;
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



void hmclFSM::dyn_callback(const hmcl_fsm::dyn_paramsConfig &config, uint32_t level) {              
            
            d0 = config.d0;
            k0 = config.k0;
            lidar_min_threshold = config.lidar_min_threshold;
            lidar_avoidance_distance_ = config.lidar_avoidance_distance;
            lidar_avoidance_move_distance_ = config.lidar_avoidance_move_distance;
            lidar_final_avoidance_distance = config.lidar_final_avoidance_distance;
            manual_trj_switch_ = config.manual_trj_switch;
            target_x=config.target_x;
            target_y=config.target_y;
            target_z=config.target_z;
            target_yaw = config.target_yaw;            
            vector_avoidance_scale = config.vector_avoidance_scale;
                    
}


void hmclFSM::local_avoidance(double min_distance){
    if(verbos){
        ROS_INFO("local avoidance activated");
        ROS_INFO("local activation distance =  %f",min_distance);
    }    
    
    std_msgs::Bool tmp_data_;
    tmp_data_.data = true;  
    local_avoidance_switch_pub.publish(tmp_data_);  
	float avoidance_vector_x = 0; 
	float avoidance_vector_y = 0;
	bool avoid = true;	
    bool final_avoidance_activate = false;
    for (double angle_tmp=-3.14195; angle_tmp < 3.14195;  angle_tmp +=lidar_data.angle_increment){
        if(angle_tmp < lidar_data.angle_min || angle_tmp > lidar_data.angle_max){
            float x = cos(angle_tmp);
			float y = sin(angle_tmp);
			float U = -0.5*k0*pow(((1/(min_distance*1.2)) - (1/d0)), 2);	
			avoidance_vector_x = avoidance_vector_x + x*U;
			avoidance_vector_y = avoidance_vector_y + y*U;
        }            
    } 
    
    for(int i=0; i<lidar_data.ranges.size(); i++)
	{    
		if( lidar_data.ranges[i] > lidar_min_threshold)
		{   
            
               double tmp_dist = lidar_data.ranges[i];
            if (tmp_dist > d0){
                tmp_dist = d0;
            }			
			float x = cos(lidar_data.angle_increment*i+lidar_data.angle_min);
			float y = sin(lidar_data.angle_increment*i+lidar_data.angle_min);
			float U = -0.5*k0*pow(((1/tmp_dist) - (1/d0)), 2);	
            
            if(lidar_data.ranges[i] <= lidar_final_avoidance_distance){
                    final_avoidance_activate = true;
            }
            //////////////// ADD more VECTOR corresponds to velocity 
            double projected_vector = mav_vel_x*x + mav_vel_y*y / 1.0;
            projected_vector = std::max(vector_avoidance_scale,projected_vector);
            if(projected_vector < 0){
                projected_vector = 0.0;
            }
            ////////////////////////////          
			
            if(i < 20 || i > 1020){
                avoidance_vector_x = avoidance_vector_x + 1.1*x*U-1*x*projected_vector*vector_avoidance_scale;
			    avoidance_vector_y = avoidance_vector_y + 1.1*y*U-1*y*projected_vector*vector_avoidance_scale;
            }else{
                avoidance_vector_x = avoidance_vector_x + x*U-1*x*projected_vector*vector_avoidance_scale;
			    avoidance_vector_y = avoidance_vector_y + y*U-1*y*projected_vector*vector_avoidance_scale;  
            }
            
		}        
	}	

//////////////////////////////////////////////////////////////////////////////
/////////////////// if final distance actiaated -> re calculate avoidance vector 
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
if(final_avoidance_activate){
            avoidance_vector_x = 0.0;
            avoidance_vector_y = 0.0;

        for (double angle_tmp=-3.14195; angle_tmp < 3.14195;  angle_tmp +=lidar_data.angle_increment){
                if(angle_tmp < lidar_data.angle_min || angle_tmp > lidar_data.angle_max){
                    float x = cos(angle_tmp);
                    float y = sin(angle_tmp);
                    float U = -0.5*k0*pow(((1/(min_distance*1.2)) - (1/d0)), 2);	
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
                float x = cos(lidar_data.angle_increment*i+lidar_data.angle_min);
                float y = sin(lidar_data.angle_increment*i+lidar_data.angle_min);
                float U = -0.5*k0*pow(((1/tmp_dist) - (1/d0)), 2);	
                
                if(lidar_data.ranges[i] <= lidar_final_avoidance_distance*sqrt(2)+0.05 ){
                        U = 5*U;
                }
                //////////////// ADD more VECTOR corresponds to velocity 
                double projected_vector = mav_vel_x*x + mav_vel_y*y / 1.0;
                projected_vector = std::max(vector_avoidance_scale,projected_vector);
                if(projected_vector < 0){
                    projected_vector = 0.0;
                }
                ////////////////////////////          
                avoidance_vector_x = avoidance_vector_x + x*U-1*x*projected_vector*vector_avoidance_scale;
                avoidance_vector_y = avoidance_vector_y + y*U-1*y*projected_vector*vector_avoidance_scale;
            }        
        }	

}
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
    
    float avoidance_vector_x_bf = avoidance_vector_x;
    float avoidace_vector_y_bf = avoidance_vector_y;

    // if(rtb){
    //    avoidace_vector_y_bf = ( 1 + 0.1*avoid_perpendicualar_factor) * avoidace_vector_y_bf ;
    // }
        
    // Transform from Body frame to Local frame  
	avoidance_vector_x = avoidance_vector_x_bf*cos(current_yaw) - avoidace_vector_y_bf*sin(current_yaw);
	avoidance_vector_y = avoidance_vector_x_bf*sin(current_yaw) + avoidace_vector_y_bf*cos(current_yaw);   
   
    if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > lidar_avoidance_move_distance_)
		{
			avoidance_vector_x = lidar_avoidance_move_distance_ * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
            avoidance_vector_y = lidar_avoidance_move_distance_ * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));    
		}

	if(avoid)
	{   		

        pose_target_.header.stamp = ros::Time::now();
        pose_target_.header.frame_id ='c';    
        pose_target_.coordinate_frame = 1;       
        // if((avoidance_vector_x + current_pose.position.x)  < global_pose_x_min || (avoidance_vector_x + current_pose.position.x)  > global_pose_x_max){
        // avoidance_vector_x = 0.0;
        // }
        //  if((avoidance_vector_y + current_pose.position.y)  < global_pose_y_min || (avoidance_vector_y + current_pose.position.y)  > global_pose_y_max){
        // avoidance_vector_y = 0.0;
        // }
        pose_target_.position.x =  avoidance_vector_x + current_pose.position.x;        
        pose_target_.position.y =  avoidance_vector_y + current_pose.position.y;
        if(FSM_mode ==0){
                pose_target_.position.z = init_takeoff_;    
        }
        // }else{
        //         if(current_pose.position.z < init_takeoff_ -0.2 || current_pose.position.z > global_pose_z_max-0.2){
        //              pose_target_.position.z = init_takeoff_;    
        //         }else{
        //              pose_target_.position.z =  current_pose.position.z;    
        //         }        
        // }
       
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
        
        double angle_to_avoidance = atan2(avoidance_vector_y,avoidance_vector_x);
        angle_wrap(angle_to_avoidance);
            
        
        if(FSM_mode ==1){          

            pose_target_.yaw =  tmp_target_.yaw;
            pose_target_.yaw_rate = tmp_target_.yaw_rate;    
        }
        // else{
            // if(!rtb){
            //     double angle_increment_tmp_ =0.005;
            //     if(final_avoidance_activate){
            //         angle_increment_tmp_ +=0.01;
            //     }
            //     if( angle_to_avoidance < lidar_data.angle_min){
            //         pose_target_.yaw = current_yaw+angle_increment_tmp_;                       
            //     }else if(angle_to_avoidance > lidar_data.angle_max){
            //         pose_target_.yaw = current_yaw-angle_increment_tmp_;                    
            //     }else{
            //         pose_target_.yaw =  current_yaw;
            //          if(if_stuck){
            //             pose_target_.yaw =pose_target_.yaw+0.005;
            //             }
            //     }
            //     pose_target_.yaw_rate = 0.01;
            // }    
            // pose_target_.yaw =  current_yaw;                
        // }
        
        local_trj_enable = false;   
	}   
    
}




void hmclFSM::lidarCallback(const sensor_msgs::LaserScanConstPtr &msg){
    
    lidar_data = *msg;  
     if (lidar_data.ranges.size() < 1){
        return;
    }    
    int size = lidar_data.ranges.size();
    int minIndex = 0;
    double minval = 999;
    for(int i = 0; i < lidar_data.ranges.size(); i++){        
        if ( lidar_data.ranges[i] <= minval && lidar_data.ranges[i] > lidar_min_threshold){
            minval = lidar_data.ranges[i];
            minIndex = i;
        }                
    }
    
    double avoidance_adhoc_vel = sqrt(pow(mav_vel_x,2)+pow(mav_vel_y,2));    
    avoidance_adhoc_vel = std::max(avoidance_adhoc_vel,1.0);        
    avoidance_adhoc_vel = avoidance_adhoc_vel*avoidance_adhoc_vel;
    
    avoidance_adhoc_vel = 0.0;

    if(minval < lidar_avoidance_distance_+avoidance_adhoc_vel*0.25){        
        if(mainFSM_mode == mainFSMmode::Init){
                mainFSM_mode = mainFSMmode::Init;
        }else{
            mainFSM_mode = mainFSMmode::Avoidance;
        }
        if_stuck = check_if_stuck();
        local_avoidance(minval+avoidance_adhoc_vel*0.25);      
        avoidance_enable = true;          
    }else{
        avoidance_enable = false;
        
    }   
    
}



void hmclFSM::cmdloopCallback(const ros::TimerEvent &event) {  
    
    if(!cali_done){        
        return;        
    }

    if( pose_target_.position.z > global_pose_z_max){
         pose_target_.position.z = global_pose_z_max;
    }    
    if( pose_target_.position.z < global_pose_z_min){
         pose_target_.position.z = global_pose_z_min;
    }

    if(landing){
        pose_target_.position.x = current_pose.position.x; 
        pose_target_.position.y = current_pose.position.y; 
        if(current_pose.position.z > 0.3){
                pose_target_.position.z = current_pose.position.z-0.5;             
        }else{  
            arm_cmd.request.value = false;                     
            if(current_state.armed)
            {   if( arming_client.call(arm_cmd))
                {
                    if(verbos){ROS_INFO("Vehicle disarmed");}
                    armed = false;                            
                }                      
                    
            }
            pose_target_.position.z = current_pose.position.z-2.0; 
        }
                
        position_target_pub.publish(pose_target_);        
        return;
    }

    if(manual_trj_switch_){
        sendManualTrajectory();                
    }   
    
    position_target_pub.publish(pose_target_);   
    if(verbos){
            avoidance_vector_display.header = pose_target_.header;
            avoidance_vector_display.header.frame_id = "world";
            avoidance_vector_display.pose.position.x = pose_target_.position.x;
            avoidance_vector_display.pose.position.y = pose_target_.position.y;
            avoidance_vector_display.pose.position.z = pose_target_.position.z;
            tf2::Quaternion tmp_quat_;
            tmp_quat_.setRPY( 0, 0, pose_target_.yaw);  // Create this quaternion from roll/pitch/yaw (in radian
            tmp_quat_.normalize();
            tf2::convert(tmp_quat_, avoidance_vector_display.pose.orientation);
            avoidance_vector_vis_pub.publish(avoidance_vector_display);
    }
   

    return;
}


void hmclFSM::sendManualTrajectory(){  
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




void hmclFSM::odom_cb(const nav_msgs::OdometryConstPtr& msg){    
    odom_state.pose = msg->pose;
    current_pose = msg->pose.pose;
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
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

void hmclFSM::battery_state_cb(const sensor_msgs::BatteryStateConstPtr& msg){
    current_battery = msg->percentage;    
}

////////////////////////////// RTB ////////////////////////
////////////////////////////// RTB ////////////////////////
////////////////////////////// RTB ////////////////////////

void hmclFSM::check_rtb_init(){
    ////////// If total flight path length is greater than certain amount
    double total_run_distance_tmp = 0.0;
    if(wp_x.size()>2){        
        for(int i=1;i < wp_x.size()-2;i++){
            total_run_distance_tmp = total_run_distance_tmp+ sqrt(pow(wp_x[i]-wp_x[i-1],2) + pow(wp_y[i]-wp_y[i-1],2));
        }        
        
        if(total_run_distance_tmp > total_run_distance){
            rtb = true;
        }   
         ROS_INFO("total_run_distance_ = %f",total_run_distance_tmp);     
    }
   
     ////////////  If bettery is not enough 
    if(current_battery < battery_thres){  
            rtb = true;
    }
   

}

void hmclFSM::STORE_WP(){
    double store_x, store_y;
    
    if(avoidance_enable){
        store_x = pose_target_.position.x;
        store_y = pose_target_.position.y;         
    }else{
        store_x = current_x;
        store_y = current_y;         
    }    
    if (wp_x.size() <1){
        wp_x.push_back(store_x);
        wp_y.push_back(store_y);
        prev_wp_yaw = current_yaw;    

    }else{
        if ( abs(current_yaw-prev_wp_yaw) > A_sw ||
            sqrt(pow(wp_x.back()-store_x,2) + pow(wp_y.back()-store_y,2)) > D_sw){          
                wp_x.push_back(store_x);
                wp_y.push_back(store_y);
                prev_wp_yaw = current_yaw;
        }
    }

    
}


bool hmclFSM::CHECK_CLEARANCE_AVOIDANCE(int lidar_idx_c, double expected)
{   

    double measured = lidar_data.ranges[lidar_idx_c];
    double theta = atan2(safe_distance,expected);    
    angle_wrap(theta);

    int lidar_idx_l = (int)(lidar_idx_c + (theta/lidar_data.angle_increment+ 1 ));     
    int lidar_idx_r = (int)(lidar_idx_c - (theta/lidar_data.angle_increment+ 1 ));     

    // int lidar_idx_bound_l = lidar_idx_l+ (int)(lidar_idx_c + M_PI_4/lidar_data.angle_increment);     
    // int lidar_idx_bound_r = (int)(lidar_idx_c - M_PI_4/lidar_data.angle_increment);    

    int lidar_idx_bound_l = lidar_idx_c + (int)(M_PI_4/lidar_data.angle_increment); 
    int lidar_idx_bound_r = lidar_idx_c - (int)(M_PI_4/lidar_data.angle_increment);  
    
      
    if(lidar_idx_l > lidar_data.ranges.size()){
        lidar_idx_l= lidar_data.ranges.size()-1;
    }
    if(lidar_idx_r < 0){
        lidar_idx_r = 0;
    }

    if(lidar_idx_bound_l > lidar_data.ranges.size()){
        lidar_idx_bound_l= lidar_data.ranges.size()-1;
    }
    if(lidar_idx_bound_r < 0){
        lidar_idx_bound_r = 0;
    }
    
    for(int i =lidar_idx_r ;  i < lidar_idx_l; i++){
        double tmp_theta = (i-lidar_idx_c)*lidar_data.angle_increment;
        if( fabs(expected/cos(tmp_theta)) > lidar_data.ranges[i]){            
            return false;
        }
    } 
    for(int i =lidar_idx_l ;  i < lidar_idx_bound_l; i++){
            double tmp_theta = (i-lidar_idx_c)*lidar_data.angle_increment;
            if(fabs(lidar_data.ranges[i]*sin(tmp_theta)) < safe_distance){                
                return false;
            }
    }
    for(int i =lidar_idx_bound_r ;  i < lidar_idx_r; i++){
        double tmp_theta = (i-lidar_idx_c)*lidar_data.angle_increment;
        if(fabs(lidar_data.ranges[i]*sin(tmp_theta)) < safe_distance){          
            return false;
        }
    }

    return true;
}


int hmclFSM::get_closest_wp_idx(){
    
    int min_idx = wp_x.size()-1;     
    int min_idx_tmp = wp_x.size()-1;    
    double min_value = sqrt(pow(current_x-wp_x[min_idx],2)+ pow(current_y-wp_y[min_idx],2));        
    for(int i=min_idx_tmp; i >= 0 ; i--){        
        
        double ttmp_ = sqrt(pow(current_x-wp_x[i],2)+ pow(current_y-wp_y[i],2));         
        if(min_value > ttmp_ && ttmp_ > D_sp+0.2){
            min_value = ttmp_;
            min_idx = i;
        }
    }
    
    // min_idx = min_idx-1;
    if(min_idx < 0){
        min_idx = 0;
    }

    return min_idx;
}

void hmclFSM::angle_wrap(double &angle){
    while (angle < -M_PI) {
        angle += 2 * M_PI;
        }
        while (angle > M_PI) {
        angle -= 2 * M_PI;
        }
}


////////////////////////////// RTB END////////////////////////
////////////////////////////// RTB END////////////////////////
////////////////////////////// RTB END////////////////////////









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