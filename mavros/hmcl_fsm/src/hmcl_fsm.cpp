/**
 * @file hmcl_fsm.cpp
 * @brief Main PLanning stack for Drone competition
 */
#include "hmcl_fsm.h"



hmclFSM::~hmclFSM() {}

hmclFSM::hmclFSM(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,const ros::NodeHandle& cmd_nh, const ros::NodeHandle& fsm_nh, const ros::NodeHandle& lidar_nh)
: nh_(nh), nh_private_(nh_private), cmd_nh_(cmd_nh), fsm_nh_(fsm_nh), lidar_nh_(lidar_nh){     
    waypoints_itr = 0;
    odom_received = false;
    att_clb_first_callback = false;
    mpcCommand_sub = nh_.subscribe<mav_msgs::RollPitchYawrateThrust>("/m100/setpoint_raw/roll_pitch_yawrate_thrust",1,&hmclFSM::mpcCommandCallback,this);        
    multiDOFJointSub = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/m100/command/trajectory",10,&hmclFSM::multiDOFJointCallback,this);            
    odom_sub = nh_.subscribe<nav_msgs::Odometry>("/ground_truth_pose",10, &hmclFSM::odom_cb,this);  
    lidar_sub = nh_.subscribe<sensor_msgs::LaserScan>("/laser/scan",1,&hmclFSM::lidarCallback,this);    
    state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &hmclFSM::state_cb,this);
    bbx_sub   = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("/trt_yolo_ros/bounding_boxes", 10, &hmclFSM::bbxCallback,this);
    // vision_odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_node/odometry",10, &hmclFSM::visCallback,this);    
    pos_cmd_sub = nh_.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10, &hmclFSM::poseCmdCallback,this);    


    cmdloop_timer_ = cmd_nh_.createTimer(ros::Duration(0.1), &hmclFSM::cmdloopCallback,this); // Critical -> allocate another thread 
    fsm_timer_ = fsm_nh_.createTimer(ros::Duration(1), &hmclFSM::mainFSMCallback,this); // Critical -> allocate another thread 
    lidar_timer_ = lidar_nh_.createTimer(ros::Duration(0.1), &hmclFSM::lidarTimeCallback,this); //Critical -> allocate another thread 
    
    
    waypoint_iter_timer_ = nh_.createTimer(ros::Duration(0.0), &hmclFSM::waypointTimerCallback,this,false,true); 
    // ros::Subscriber att_thrust_sub = nh.subscribe<mav_msgs::RollPitchYawrateThrust>
    //         ("/lmpc/roll_pitch_yawrate_thrust", 10, rpyt_cb);
    manual_trj_pub =  nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/command/trajectory",5);
    mpc_cmd_pub =  nh_.advertise<mav_msgs::RollPitchYawrateThrust>("/mavros/setpoint_raw/roll_pitch_yawrate_thrust",5);
    rpyt_pub = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);
    position_target_pub = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);    
    vis_pos_pub  =  nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);    
    local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    camera_points_pub = nh_.advertise<sensor_msgs::PointCloud2>("camera_points", 2);     

    // Define service Clients 
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    explore_client_start = nh_.serviceClient<std_srvs::Trigger>("/planner_control_interface/std_srvs/automatic_planning");
    explore_client_stop =nh_.serviceClient<std_srvs::Trigger>("/planner_control_interface/std_srvs/stop");
    explore_client_home = nh_.serviceClient<std_srvs::Trigger>("/planner_control_interface/std_srvs/homing_trigger");
    explore_client_global_planner = nh_.serviceClient<planner_msgs::pci_global>("pci_global");
    
    
    f = boost::bind(&hmclFSM::dyn_callback, this,_1, _2);
    server.setCallback(f);

    // ros::param::set("d0",2.5);
    // ros::param::set("k0",0.5);
    // ros::param::set("thrust_scale",0.035);
    // ros::param::set("manual_trj_switch",false);    

    load_FSM_Params("fsm");        
    print_FSM_Params();
    
         
    
    ROS_INFO_STREAM("d0 is set to be = " << d0);
    ROS_INFO_STREAM("k0 is set to be = " << k0);
    
    // initialize control method
    pose_cmd_enable = false;
    mpc_cmd_enable = false;
    send_waypoint = false;
    avoidance_enable = false;

    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(2); // 10 HZ
    while(ros::ok() && !current_state.connected){
        ROS_INFO("lets try to connect");
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("PX4 connected");    

    // Wait for other topics 
    while(!odom_received){
        ROS_INFO("wait for odom to be available");
        ros::spinOnce();
        rate.sleep();
    }


    offb_set_mode.request.custom_mode = "OFFBOARD";    
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();


    ROS_INFO("Try OFFBOARD and Arming");
    armed = false;
    offboarded = false;
    
    while(ros::ok()){            
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
                {
                    // if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){ROS_INFO("Offboard enabled");}
                    if( set_mode_client.call(offb_set_mode) ){
                        ROS_INFO("Offboard enabled");
                        offboarded = true;}
                    last_request = ros::Time::now();
                } else {          
                    if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0))){
                        // if( arming_client.call(arm_cmd) && arm_cmd.response.success){ROS_INFO("Vehicle armed");}
                        if( arming_client.call(arm_cmd)){
                            ROS_INFO("Vehicle armed");
                            armed = true;                            
                            }else{
                            ROS_INFO("arm failed");
                            }
                        last_request = ros::Time::now();
                    }
                }
        if(offboarded && armed){ break;}                
    }    
    ROS_INFO("Initializing FSM");    
    mainFSM_mode = mainFSMmode::NA;
    Explore_Mode = ExploreMode::NA;
    Detect_Mode  = DetectMode::NA;
    Land_Mode =  LandMode::NA;
    localsearch_count = 0;

}

void hmclFSM::load_FSM_Params(std::string group){
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

void hmclFSM::print_FSM_Params(){    
    ROS_INFO("init_takeoff_ = %f",  init_takeoff_);
    ROS_INFO("lidar_avoidance_distance_ = %f", lidar_avoidance_distance_);
    ROS_INFO("global_pose_x_min = %f",global_pose_x_min); 
    ROS_INFO("global_pose_y_min = %f",global_pose_y_min); 
    ROS_INFO("global_pose_z_min = %f",global_pose_z_min); 
    ROS_INFO("global_pose_x_max = %f",global_pose_x_max); 
    ROS_INFO("global_pose_y_max = %f",global_pose_y_max); 
    ROS_INFO("global_pose_z_max = %f",global_pose_z_max); 
}

void hmclFSM::init_takeoff(){
    if(!odom_received){
        ROS_INFO("odom is not availble");
        return;
    }
     if( current_state.mode != "OFFBOARD"){  // if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){ROS_INFO("Offboard enabled");}
            set_mode_client.call(offb_set_mode);                
        } else {          
            if( !current_state.armed ){            
                arming_client.call(arm_cmd);
            }
        }
    pose_target_.header.stamp = ros::Time::now();
    pose_target_.header.frame_id ='c';        
    pose_target_.coordinate_frame = 1; // mavros_msgs::PositionTarget::FRAME_LOCAL_NED;         
    pose_target_.position.x = current_pose.position.x; 
    pose_target_.position.y = current_pose.position.y; 
    pose_target_.position.z = init_takeoff_;     
    pose_target_.yaw = current_yaw;
    position_target_pub.publish(pose_target_);  
}

void hmclFSM::mainFSMCallback(const ros::TimerEvent &event){   
    if(!armed && !offboarded) return;     
    
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
            }else if(previous_mode == mainFSMmode::Avoidance){
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


void hmclFSM::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg){
    geometry_msgs::TransformStamped transformStamped;
     tf2_ros::TransformListener tfListener(tfBuffer);   
        try {
            transformStamped = tfBuffer.lookupTransform("base_link", "camera_link",
                    msg->header.stamp,ros::Duration(0.3));            
            tf2::doTransform(*msg, cloud_out, transformStamped);
            cloud_out.header.frame_id = "cam_points";
            cloud_out.header.stamp = msg->header.stamp;
            camera_points_pub.publish(cloud_out);            
//          pcl_ros::transformPointCloud("robotarm", cloud_in, cloud_out, tfListener);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());                       
        }

}

void hmclFSM::poseCmdCallback(const quadrotor_msgs::PositionCommandConstPtr &msg){
    // ROS_INFO("pose cmd recied");
    pose_target_.header.stamp = ros::Time::now();
    pose_target_.header.frame_id ='c';        
    pose_target_.coordinate_frame = 1; // mavros_msgs::PositionTarget::FRAME_LOCAL_NED;         
    
    pose_target_.position.x = msg->position.x; 
    pose_target_.position.y = msg->position.y; 
    pose_target_.position.z = msg->position.z; 
    
    pose_target_.velocity.x = msg->velocity.x; 
    pose_target_.velocity.y = msg->velocity.y; 
    pose_target_.velocity.z = msg->velocity.z; 

    pose_target_.acceleration_or_force.x = msg->acceleration.x;
    pose_target_.acceleration_or_force.y = msg->acceleration.y;
    pose_target_.acceleration_or_force.z = msg->acceleration.z; 

    pose_target_.yaw = msg->yaw;
    pose_target_.yaw_rate = msg->yaw_dot;

}

void hmclFSM::bbxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg){
    if(msg->bounding_boxes.size() > 0){
        ROS_INFO("obj found");
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
            manual_trj_switch_ = config.manual_trj_switch;
            target_x=config.target_x;
            target_y=config.target_y;
            target_z=config.target_z;
            ROS_INFO("d0 = %f, k= %f, thrust_scale = %f", d0,k0,thrust_scale);            
}


void hmclFSM::local_avoidance(){
    ROS_INFO("local avoidance activated");
    //TODO - using 2d lidar data, send avoidance manuever cmd to drone     
	float avoidance_vector_x = 0; 
	float avoidance_vector_y = 0;
	bool avoid = false;	
	for(int i=1; i<lidar_data.ranges.size(); i++)
	{    
		if(lidar_data.ranges[i] < d0 && lidar_data.ranges[i] > 0.35)
		{   
			avoid = true;
			float x = cos(lidar_data.angle_increment*i);
			float y = sin(lidar_data.angle_increment*i);
			float U = 0.5*k0*pow(((1/lidar_data.ranges[i]) - (1/d0)), 2);	

			avoidance_vector_x = avoidance_vector_x + x*U;
			avoidance_vector_y = avoidance_vector_y + y*U;
		}
	}	
	avoidance_vector_x = avoidance_vector_x*cos(current_yaw) - avoidance_vector_y*sin(current_yaw);
	avoidance_vector_y = avoidance_vector_x*sin(current_yaw) + avoidance_vector_y*cos(current_yaw);
	if(avoid)
	{   
		if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 3)
		{
			avoidance_vector_x = 3 * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
			avoidance_vector_y = 3 * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
		}		
        pose_target_.header.stamp = ros::Time::now();
        pose_target_.header.frame_id ='c';    
        pose_target_.coordinate_frame = 1;        
        pose_target_.position.x =  avoidance_vector_x + current_pose.position.x;        
        pose_target_.position.y =  avoidance_vector_y + current_pose.position.y;
        if(current_pose.position.z < 1.0 || current_pose.position.z > global_pose_z_max-0.5){
            pose_target_.position.z = 1.5;    
        }else{
            pose_target_.position.z =  current_pose.position.z;    
        }        
        pose_target_.yaw = current_yaw;
        pose_target_.yaw_rate = 0.0;
        pose_target_.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | 
                                     mavros_msgs::PositionTarget::IGNORE_VX  | 
                                     mavros_msgs::PositionTarget::IGNORE_VY  | 
                                     mavros_msgs::PositionTarget::IGNORE_VZ;
              
        avoidance_enable = true;

        target_pose.position.x = pose_target_.position.x;
        target_pose.position.y = pose_target_.position.y;
        target_pose.position.z = pose_target_.position.z;
        ROS_INFO("Avoidance Set position x = %f, y = %f, z = %f, yaw = %f", pose_target_.position.x, pose_target_.position.y, pose_target_.position.z, current_yaw);  
         
        
         
	}
	


    // and refine trajectory to follow if there is any     
    if(waypoints.points.size() > 0 ){
        refine_path_via_lidarData();        
    }
    
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
    // ROS_INFO("check mode");
    
    send_waypoint = false;
    // follow the local avoidance pose control 
    
    // if(avoidance_enable){        
    //     mpc_cmd_enable = false;
    // }else{        
    //     mpc_cmd_enable = true;
    // }
    
    if(manual_trj_switch_){
        sendManualTrajectory();  
              
    }

    return;
}

void hmclFSM::lidarTimeCallback(const ros::TimerEvent& e){
    if (mainFSM_mode == mainFSMmode::Init){
        return;
    }
    if (lidar_data.ranges.size() < 1){
        return;
    }
    //If the minimum distance is less than threshold -> activate local_avoidance    
    int size = lidar_data.ranges.size();
    int minIndex = 0;
    double minval = 999;
    for(int i = 0; i < lidar_data.ranges.size(); i++){
        if ( lidar_data.ranges[i] <= minval){
            minval = lidar_data.ranges[i];
            minIndex = i;
        }                
    }
    //Average the neighboring signals to reject any noisy signal around the minimum point
    int avg_half_width = 5;    
    double dist_tmp = -0.01;
    int qq = 0;
    for(int k = minIndex-avg_half_width; k < minIndex + avg_half_width ; k++)
    {   
        if (k > lidar_data.ranges.size())
             qq = k -lidar_data.ranges.size();
        else if(k < 0)
            qq = k+lidar_data.ranges.size();
        else
            qq = k;
        
        if (lidar_data.ranges[qq] < 100){
                dist_tmp = dist_tmp + lidar_data.ranges[qq];           
        }else{
            dist_tmp = dist_tmp + 100;           
        }
    }
    if (dist_tmp < 0){ dist_tmp = 100*avg_half_width;}
    dist_tmp = dist_tmp / (2*avg_half_width);   
    // ROS_INFO_STREAM("minimum distance (2d lidar) = " << dist_tmp);
    // ROS_INFO_STREAM(" = " << lidar_avoidance_distance_);
    if(dist_tmp < lidar_avoidance_distance_){
        ROS_INFO("Warning!! --> Obstacles approaching");        
        mainFSM_mode = mainFSMmode::Avoidance;
        local_avoidance();                
    }else{
        avoidance_enable = false;
        // set_target_pose(0,0,0.7,0);
    }
}

void hmclFSM::lidarCallback(const sensor_msgs::LaserScanConstPtr &msg){
    lidar_data = *msg;       
}

void hmclFSM::mpcCommandCallback(const mav_msgs::RollPitchYawrateThrustConstPtr &msg){
    att_clb_time_in_sec  = ros::Time::now().toSec();
    if (!att_clb_first_callback){
        att_clb_first_callback = true;  
        att_clb_time_in_sec_prev = att_clb_time_in_sec;          
        return;
    }   
    mpc_cmd = *msg; 
    att.header = msg->header;    
    // att.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE |
    //         mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE | mavros_msgs::AttitudeTarget::IGNORE_THRUST; 
    att.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE; 
    
        tf::Quaternion quat;
        tf::quaternionMsgToTF(odom_state.pose.pose.orientation, quat);

        // the tf::Quaternion has a method to acess roll pitch and yaw
        double roll_, pitch_, yaw_;
        tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);       
        double dt = att_clb_time_in_sec - att_clb_time_in_sec_prev; 
        if (dt < 0){
            ROS_WARN("att dif time is less than zero !!!!!!!!!!!");
        }
        yaw_ = yaw_ + yaw_scale * msg->yaw_rate* dt; 

        tf2::Quaternion quat_;
        quat_.setRPY(msg->roll, msg->pitch,yaw_);
        quat_=quat_.normalize();

        att.orientation.x = quat_.getX();
        att.orientation.y = quat_.getY();
        att.orientation.z = quat_.getZ();
        att.orientation.w = quat_.getW();
        
        att.body_rate.x = 0.0;
        att.body_rate.y = 0.0;
        att.body_rate.z = msg->yaw_rate; 
        att.thrust = msg->thrust.z * thrust_scale;
        
        att_clb_time_in_sec_prev  = att_clb_time_in_sec;
}



void hmclFSM::cmdloopCallback(const ros::TimerEvent &event) {   
    
    if (avoidance_enable){
        // Lidar based local avoidance activated
         position_target_pub.publish(pose_target_);         
         return; 
    }
    
    
    if (mpc_cmd_enable){                
        // mpc_cmd_pub.publish(mpc_cmd);
        rpyt_pub.publish(att);
        
        return;
    }
    
    position_target_pub.publish(pose_target_);  
    // ROS_INFO("position target pub");  
     
}


void hmclFSM::sendManualTrajectory(){
    ROS_INFO("manual input send");
    waypoints.points.clear();    
    waypoints_itr = 0;
    trajectory_msgs::MultiDOFJointTrajectoryPoint trj_point; 
    geometry_msgs::Transform tmp; 
    tmp.translation.x = target_x;     tmp.translation.y = target_y;     tmp.translation.z = target_z;
    tmp.rotation.x = 0.0;     tmp.rotation.y = 0.0;     tmp.rotation.z = 0.0;     tmp.rotation.w = 1.0;
    trj_point.transforms.push_back(tmp);    
    waypoints.points.push_back(trj_point);     
    manual_trj_pub.publish(waypoints);
    waypoints.points.clear();

    
}



void hmclFSM::waypointTimerCallback(const ros::TimerEvent &event){   
    // ROS_INFO("command_waiting_times_ = %lf" , command_waiting_times_.front().toSec());
    // ROS_INFO("waypoints_itr = %d", waypoints_itr);
     if (waypoints.points.size() > 0){        
        // ROS_INFO("waypoints iter = %d", waypoints_itr);
        if (waypoints_itr >= 0 && waypoints_itr < waypoints.points.size()){            
            pose_target_.header.stamp = ros::Time::now();
            pose_target_.header.frame_id ='c';        
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
            pose_target_.yaw_rate = 1.0;
            // position_target_pub.publish(pose_target_);   
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
//   for (int i = 0; i < msg->points.size(); i++)
//     {
//     waypoints.points.push_back(msg->points[i]);
//     }
 //  
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
            for (int i=1 ; i < msg->points.size(); i++){      
                command_waiting_times_.push_back(msg->points[i].time_from_start  - msg->points[i-1].time_from_start);
                // ROS_INFO("push time = %lf",double((msg->points[i].time_from_start  - msg->points[i-1].time_from_start).toSec()));
            }
            waypoint_iter_timer_.setPeriod(command_waiting_times_.front());
            command_waiting_times_.pop_front();
            waypoint_iter_timer_.start();
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
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
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

void hmclFSM::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void hmclFSM::set_target_pose(double x,double y, double z, double yaw){
            pose_target_.header.stamp = ros::Time::now();
            pose_target_.header.frame_id ='c';   
            pose_target_.coordinate_frame = 1; // mavros_msgs::PositionTarget::FRAME_LOCAL_NED;                        
            pose_target_.position.x = x;
            pose_target_.position.y = y;
            pose_target_.position.z = z;               
            pose_target_.yaw = yaw;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "hmcl_fsm");
    ros::NodeHandle nh;
    ros::NodeHandle cmd_nh_;
    ros::NodeHandle fsm_nh_;
    ros::NodeHandle lidar_nh_;
    ros::NodeHandle nh_private("~");     

    ros::CallbackQueue callback_queue_cmd;
    cmd_nh_.setCallbackQueue(&callback_queue_cmd);
    
    ros::CallbackQueue callback_queue_fsm;
    fsm_nh_.setCallbackQueue(&callback_queue_fsm);

    ros::CallbackQueue callback_queue_lidar;
    lidar_nh_.setCallbackQueue(&callback_queue_lidar);
    
    hmclFSM hmcl_FSM(nh,nh_private,cmd_nh_,fsm_nh_,lidar_nh_);        


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


    ros::spin();
    spinner_thread_cmd.join();
    spinner_thread_fsm.join();
    spinner_thread_lidar.join();
    
    return 0;
}