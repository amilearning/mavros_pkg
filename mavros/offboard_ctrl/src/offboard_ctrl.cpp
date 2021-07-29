/**
 * @file offb_node.cpp
 * @brief Main PLanning stack for Drone competition
 */
#include "offboard_ctrl.h"



OffboardFSM::~OffboardFSM() {}

OffboardFSM::OffboardFSM(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
: nh_(nh), nh_private_(nh_private) {    
   
    waypoints_itr = 0;
    att_clb_first_callback = false;
    mpcCommand_sub = nh_.subscribe<mav_msgs::RollPitchYawrateThrust>("/m100/setpoint_raw/roll_pitch_yawrate_thrust",1,&OffboardFSM::mpcCommandCallback,this);        
    multiDOFJointSub = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/m100/command/trajectory",10,&OffboardFSM::multiDOFJointCallback,this);        
    // odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",10, &odom_cb);
    odom_sub = nh_.subscribe<nav_msgs::Odometry>("/ground_truth_pose",10, &OffboardFSM::odom_cb,this);
    lidar_sub = nh_.subscribe<sensor_msgs::LaserScan>("/laser/scan",1,&OffboardFSM::lidarCallback,this);    
    state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardFSM::state_cb,this);
    bbx_sub   = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("/trt_yolo_ros/bounding_boxes", 10, &OffboardFSM::bbxCallback,this);
    
    pos_cmd_sub = nh_.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10, &OffboardFSM::poseCmdCallback,this);
    // lidar_timer_ = nh_.createTimer(ros::Duration(0.1), &OffboardFSM::lidarTimeCallback,this); 
    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.001), &OffboardFSM::cmdloopCallback,this); 
    waypoint_iter_timer_ = nh_.createTimer(ros::Duration(0.0), &OffboardFSM::waypointTimerCallback,this,false,true); 
    // ros::Subscriber att_thrust_sub = nh.subscribe<mav_msgs::RollPitchYawrateThrust>
    //         ("/lmpc/roll_pitch_yawrate_thrust", 10, rpyt_cb);
    manual_trj_pub =  nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/command/trajectory",5);
    mpc_cmd_pub =  nh_.advertise<mav_msgs::RollPitchYawrateThrust>("/mavros/setpoint_raw/roll_pitch_yawrate_thrust",5);
    rpyt_pub = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);
    position_target_pub = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);    
    
    local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    pose_target_.header.stamp = ros::Time::now();
    pose_target_.header.frame_id ='c';        
    pose_target_.coordinate_frame = 1; // mavros_msgs::PositionTarget::FRAME_LOCAL_NED;         

    pose_target_.position.x = 0.0; 
    pose_target_.position.y = 0.0; 
    pose_target_.position.z = 1.0; 

    
    f = boost::bind(&OffboardFSM::dyn_callback, this,_1, _2);
    server.setCallback(f);


    ros::param::set("d0",2.0);
    ros::param::set("k0",0.5);
    ros::param::set("thrust_scale",0.035);
    ros::param::set("manual_trj_switch",false);
    
    ROS_INFO_STREAM("d0 is set to be = " << d0);
    ROS_INFO_STREAM("k0 is set to be = " << k0);
    
    // initialize control method
    pose_cmd_enable = false;
    mpc_cmd_enable = false;
    send_waypoint = false;
    avoidance_enable = false;

    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20); // 10 HZ
    while(ros::ok() && !current_state.connected){
        ROS_INFO("lets try to connect");
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("PX4 connected");    
    offb_set_mode.request.custom_mode = "OFFBOARD";    
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    ROS_INFO("Now we are entering FSM");    
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            // if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){ROS_INFO("Offboard enabled");}
            if( set_mode_client.call(offb_set_mode) ){ROS_INFO("Offboard enabled");}
            last_request = ros::Time::now();
        } else {          
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0))){
                // if( arming_client.call(arm_cmd) && arm_cmd.response.success){ROS_INFO("Vehicle armed");}
                if( arming_client.call(arm_cmd)){
                    ROS_INFO("Vehicle armed");
                    }else{
                    ROS_INFO("arm failed");
                    }
                last_request = ros::Time::now();
            }
        }        
        
        // rpyt_pub.publish(att);
        // local_pos_pub.publish(pose);
        
        //TODO - update Mode status to identify the required manuever of drone        
        check_drone_status();


        // switch (FSM_mode)
        // {
        //     case FSMmode::Homing:
        //     ROS_INFO("homing");
        //     // Move_to_target_pose(target_pose);            
        //     break;

        //     case FSMmode::Exploration:
        //     ROS_INFO("Exploration");
        //     //request exploration start 
        //     break;


        //     case FSMmode::TargetApproach:
        //     ROS_INFO("TargetApproach");
        //     break;
            
        //     default:
        //     ROS_INFO("default");
        //     break;            
        // }
        
        ros::spinOnce();
        rate.sleep();
    }
}

void OffboardFSM::poseCmdCallback(const quadrotor_msgs::PositionCommandConstPtr &msg){
    ROS_INFO("pose cmd recied");
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

void OffboardFSM::bbxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg){
    if(msg->bounding_boxes.size() > 0){
        ROS_INFO("obj found");
        detected_bbx = *msg;
        // for(int i=0; i < detected_bbx.bounding_boxes.size();i++){           
        //     detected_bbx.bounding_boxes[i].z
        // }
    }
 }

void OffboardFSM::dyn_callback(const offboard_ctrl::dyn_paramsConfig &config, uint32_t level) {  
            d0 = config.d0;
            k0 = config.k0;
            thrust_scale = config.thrust_scale;
            manual_trj_switch_ = config.manual_trj_switch;
            target_x=config.target_x;
            target_y=config.target_y;
            target_z=config.target_z;
            ROS_INFO("d0 = %f, k= %f, thrust_scale = %f", d0,k0,thrust_scale);            
}


void OffboardFSM::local_avoidance(){
    ROS_INFO("local avoidance activated");
    //TODO - using 2d lidar data, send avoidance manuever cmd to drone     
	float avoidance_vector_x = 0; 
	float avoidance_vector_y = 0;
	bool avoid = false;	
	for(int i=1; i<lidar_data.ranges.size(); i++)
	{
		if(lidar_data.ranges[i] < d0 && lidar_data.ranges[i] > .35)
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
        pose_target_.position.z =  current_pose.position.z;    
        pose_target_.yaw = current_yaw;
        pose_target_.yaw_rate = 0.0;
        avoidance_enable = true;
        ROS_INFO("Avoidance Set position x = %f, y = %f, z = %f, yaw = %f", pose_target_.position.x, pose_target_.position.y, pose_target_.position.z, current_yaw);  
	}
	


    // and refine trajectory to follow if there is any     
    if(waypoints.points.size() > 0 ){
        refine_path_via_lidarData();        
    }
    
}


void OffboardFSM::refine_path_via_lidarData(){
    // delete below as this is for lidar testing
    waypoints.points.clear();
    command_waiting_times_.clear();
    waypoints_itr = 0;
    //TODO - refine path using the 2d lidar data && current pose     
    ROS_INFO("trajectory has been refined");
}

void OffboardFSM::check_drone_status(){
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

void OffboardFSM::lidarTimeCallback(const ros::TimerEvent& e){
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
    int avg_half_width = 2;    
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
        }        
    }
    if (dist_tmp < 0){ dist_tmp = 100;}
    dist_tmp = dist_tmp / (2*avg_half_width);   
    // ROS_INFO_STREAM("minimum distance (2d lidar) = " << dist_tmp);
    if(dist_tmp < lidar_avoidance_distance){
        ROS_INFO("Warning!! --> Obstacles approaching");
        local_avoidance();        
    }else{
        avoidance_enable = false;
        // set_target_pose(0,0,0.7,0);
    }
}

void OffboardFSM::lidarCallback(const sensor_msgs::LaserScanConstPtr &msg){
    lidar_data = *msg;       
}

void OffboardFSM::mpcCommandCallback(const mav_msgs::RollPitchYawrateThrustConstPtr &msg){
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

void OffboardFSM::waypointTimerCallback(const ros::TimerEvent &event){
    waypoint_iter_timer_.stop();    
    ROS_INFO("command_waiting_times_ = %f" , command_waiting_times_.front());
    ROS_INFO("waypoints_itr = %f", waypoints_itr);
     if (waypoints.points.size() > 0){        
        ROS_INFO("waypoints iter = %f", waypoints_itr);
        if (waypoints_itr >= 0 && waypoints_itr < waypoints.points.size()){            
            pose_target_.header.stamp = ros::Time::now();
            pose_target_.header.frame_id ='c';        
            pose_target_.coordinate_frame = 1; // mavros_msgs::PositionTarget::FRAME_LOCAL_NED;            
            // pose_target_.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
            // mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;            
            pose_target_.position.x = waypoints.points[waypoints_itr].transforms[0].translation.x;
            pose_target_.position.y = waypoints.points[waypoints_itr].transforms[0].translation.y;
            pose_target_.position.z = waypoints.points[waypoints_itr].transforms[0].translation.z;               
            tf::Quaternion q(waypoints.points[waypoints_itr].transforms[0].rotation.x,waypoints.points[waypoints_itr].transforms[0].rotation.y,waypoints.points[waypoints_itr].transforms[0].rotation.z,waypoints.points[waypoints_itr].transforms[0].rotation.w);
            tf::Matrix3x3 m(q);            
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);        
            pose_target_.yaw = yaw;
            // position_target_pub.publish(pose_target_);   
            send_waypoint = true;            
        }       
    } 

    if(!command_waiting_times_.empty()){           
        waypoint_iter_timer_.setPeriod(command_waiting_times_.front());
        command_waiting_times_.pop_front();
        waypoint_iter_timer_.start();
        waypoints_itr++;
    }

}


void OffboardFSM::cmdloopCallback(const ros::TimerEvent &event) {   
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
    ROS_INFO("position target pub");  
     
}


void OffboardFSM::sendManualTrajectory(){
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

void OffboardFSM::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg) {  
//   for (int i = 0; i < msg->points.size(); i++)
//     {
//     waypoints.points.push_back(msg->points[i]);
//     }
  if (msg->points.size() > 1){    
        waypoints.points = msg->points;    
        // ROS_INFO("Trjectory length = %d",msg->points.size());
    waypoints_itr = 0; 
    command_waiting_times_.clear();
    ROS_INFO("trejactory size = %f", msg->points.size());
    for (int i=1 ; i < msg->points.size(); i++){      
        command_waiting_times_.push_back(msg->points[i].time_from_start  - msg->points[i-1].time_from_start );
    }
      waypoint_iter_timer_.start();
  }
  
}


void OffboardFSM::odom_cb(const nav_msgs::OdometryConstPtr& msg){
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
}

void OffboardFSM::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void OffboardFSM::set_target_pose(double x,double y, double z, double yaw){
            pose_target_.header.stamp = ros::Time::now();
            pose_target_.header.frame_id ='c';   
            pose_target_.coordinate_frame = 1; // mavros_msgs::PositionTarget::FRAME_LOCAL_NED;                        
            pose_target_.position.x = x;
            pose_target_.position.y = y;
            pose_target_.position.z = z;               
            pose_target_.yaw = yaw;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "offboard_ctrl");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");      
    OffboardFSM offb_fsm(nh,nh_private);

    ros::spin();

    return 0;
}