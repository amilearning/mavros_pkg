/**
 * @file map_follower.cpp
 * @brief Main PLanning stack for Drone competition
 */
#include "map_follower.h"



mapfollower::~mapfollower() {}

mapfollower::mapfollower(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,const ros::NodeHandle& cmd_nh, const ros::NodeHandle& lidar_nh, const ros::NodeHandle& odom_nh)
: nh_(nh), nh_private_(nh_private), cmd_nh_(cmd_nh), lidar_nh_(lidar_nh), odom_nh_(odom_nh){     
    
    // Dynamic Configure parameters
    f = boost::bind(&mapfollower::dyn_callback, this,_1, _2);
    server.setCallback(f);

    load_FSM_Params("mapfollower"); 
    ROS_INFO("init_takeoff_ = %f",  init_takeoff_);
    ROS_INFO("lidar_avoidance_distance_ = %f", lidar_avoidance_distance_);
    ROS_INFO("global_pose_x_min = %f",global_pose_x_min); 
    ROS_INFO("global_pose_y_min = %f",global_pose_y_min); 
    ROS_INFO("global_pose_z_min = %f",global_pose_z_min); 
    ROS_INFO("global_pose_x_max = %f",global_pose_x_max); 
    ROS_INFO("global_pose_y_max = %f",global_pose_y_max); 
    ROS_INFO("global_pose_z_max = %f",global_pose_z_max); 
         
    
    ROS_INFO_STREAM("target_vel is set to be = " << target_vel);
    ROS_INFO_STREAM("cmd_hz is set to be = " << cmd_hz);
    
  
    odom_received = false;
    
    lidar_sub = lidar_nh_.subscribe<sensor_msgs::LaserScan>("/laser/scan",1,&mapfollower::lidarCallback,this);    
    
    odom_sub = odom_nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",10, &mapfollower::odom_cb,this);  
    
    
    // lidar_timer_ = lidar_nh_.createTimer(ros::Duration(1/cmd_hz), &mapfollower::lidarTimeCallback,this); //Critical -> allocate another thread 
    cmdloop_timer_ = cmd_nh_.createTimer(ros::Duration(1/cmd_hz), &mapfollower::posecmdloopCallback,this); // Critical -> allocate another thread 
    
    position_target_pub = nh_.advertise<mavros_msgs::PositionTarget>("/mapfollow/target/pose", 10);    
    
    
    
    

}

void mapfollower::posecmdloopCallback(const ros::TimerEvent &event) {   
    // if( fabs(current_yaw-target_yaw)< 0.15){   
        double move_vector_angle = current_yaw + M_PI/2.0-angle_shift_in_degree*M_PI/180.0;
        double x_vector = move_p_gain*target_vel/cmd_hz*cos(move_vector_angle);
        double y_vector = move_p_gain*target_vel/cmd_hz*sin(move_vector_angle);     
        
            double error = laser_right_min_val-laser_away_dist;
            x_vector+= away_p_gain*(laser_right_min_val-laser_away_dist)*cos(current_yaw+angle_shift_in_degree*M_PI/180.0);
            y_vector+= away_p_gain*(laser_right_min_val-laser_away_dist)*sin(current_yaw+angle_shift_in_degree*M_PI/180.0);
       
        pose_target_.position.x = current_pose.position.x + x_vector;
        pose_target_.position.y = current_pose.position.y + y_vector;           
        
    // }
    pose_target_.yaw = target_yaw;
    check_if_drone_outside_global_box();
    position_target_pub.publish(pose_target_);    
    return;
}

void mapfollower::angle_wrap(double &angle){
    while (angle < -M_PI) {
        angle += 2 * M_PI;
        }
        while (angle > M_PI) {
        angle -= 2 * M_PI;
        }
}
void mapfollower::minmaxcast(double &value,double min, double max){
    if( value <= min){
        value = min;
    }
    if(value >= max){
        value = max;
    }
}

void mapfollower::lidarCallback(const sensor_msgs::LaserScanConstPtr &msg){
    lidar_data = *msg; 
    double local_angle;
    double laser_left_min_val = lidar_data.range_max;
    double laser_left_min_idx = 0;
    //extract right half data     
    for (int i=0; i<lidar_data.ranges.size();i++){
        
        if(lidar_data.ranges[i] < lidar_data.range_min || lidar_data.ranges[i] > lidar_data.range_max){
            continue;
        }
        local_angle = lidar_data.angle_min+lidar_data.angle_increment*i;
        if(local_angle < 120*M_PI/180.0 && local_angle > 90*M_PI/180.0){
            if(laser_left_min_val > lidar_data.ranges[i]){
                laser_left_min_val = lidar_data.ranges[i];
                laser_left_min_idx = i;
            }
        }
    }
    //

    
    laser_right_min_val = lidar_data.range_max;
    laser_right_min_idx = 0;
    //extract right half data     
    for (int i=0; i<lidar_data.ranges.size();i++){
        
        if(lidar_data.ranges[i] < lidar_data.range_min || lidar_data.ranges[i] > lidar_data.range_max){
            continue;
        }
        local_angle = lidar_data.angle_min+lidar_data.angle_increment*i;
        if(local_angle > -45*M_PI/180.0 && local_angle < 0*M_PI/180.0){
            if(laser_right_min_val > lidar_data.ranges[i]){
                laser_right_min_val = lidar_data.ranges[i];
                laser_right_min_idx = i;
            }
        }
    }
    ROS_INFO("laser_right_min_idx = %d",laser_right_min_idx);
    double angle_to_rotate = lidar_data.angle_min+lidar_data.angle_increment*laser_right_min_idx+angle_shift_in_degree*3.14195/180.0;
    angle_to_rotate = angle_p_gain*angle_to_rotate;
    ROS_INFO("angle_to_rotate = %f",angle_to_rotate);    
    minmaxcast(angle_to_rotate, -0.5, 0.5);
    

    
    target_yaw = current_yaw - angle_to_rotate;
    
    ROS_INFO("current yaw = %f",current_yaw);    
    angle_wrap(target_yaw);
    ROS_INFO("target_yaw  = %f",target_yaw);
   
    
    
}


void mapfollower::odom_cb(const nav_msgs::OdometryConstPtr& msg){    
    odom_state.pose = msg->pose;
    current_pose = msg->pose.pose;
    tf::Quaternion q(current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w);
    tf::Matrix3x3 m(q);            
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);        
    current_yaw = yaw; 
    angle_wrap(current_yaw);
    if(!odom_received){     
        target_yaw = current_yaw;   
        odom_received = true;
        } 
}

void mapfollower::load_FSM_Params(std::string group){
    
    nh_private_.getParam(group+"/init_takeoff",init_takeoff_);
    nh_private_.getParam(group+"/lidar_avoidance_distance",lidar_avoidance_distance_);
    nh_private_.getParam(group+"/global_pose_x_min",global_pose_x_min);
    nh_private_.getParam(group+"/global_pose_y_min",global_pose_y_min);
    nh_private_.getParam(group+"/global_pose_z_min",global_pose_z_min);
    nh_private_.getParam(group+"/global_pose_x_max",global_pose_x_max);
    nh_private_.getParam(group+"/global_pose_y_max",global_pose_y_max);
    nh_private_.getParam(group+"/global_pose_z_max",global_pose_z_max);
}




double mapfollower::get_distance(geometry_msgs::Pose &p1,geometry_msgs::Pose &p2){
    double dist = hypot(hypot(p1.position.x-p2.position.x,p1.position.y-p2.position.y),p1.position.z-p2.position.z);
        return dist;
}

bool mapfollower::check_if_drone_outside_global_box(){
     if(!odom_received){return true;}
    if(pose_target_.position.x < global_pose_x_min){
        pose_target_.position.x = global_pose_x_min+0.5;
        return false;
    }
    if(pose_target_.position.x > global_pose_x_max){
        pose_target_.position.x = global_pose_x_max-0.5;
        return false;
    }

    if(pose_target_.position.y < global_pose_y_min){
        pose_target_.position.y = global_pose_y_min+0.5;
        return false;
    }
    if(pose_target_.position.y > global_pose_y_max){
        pose_target_.position.y = global_pose_y_max-0.5; 
        return false;
    }

    if(pose_target_.position.z < global_pose_z_min){
        pose_target_.position.z = global_pose_z_min+0.5;
        return false;
    }
    if(pose_target_.position.z > global_pose_z_max){
        pose_target_.position.z = global_pose_z_max-0.5;
        return false;
    }
    
     
    return true;
}

void mapfollower::dyn_callback(const map_follower::dyn_paramsConfig &config, uint32_t level) {  
            target_vel = config.target_vel;
            cmd_hz = config.cmd_hz;
            angle_shift_in_degree = config.angle_shift_in_degree;
            laser_away_dist = config.laser_away_dist;
            move_p_gain = config.move_p_gain;
            angle_p_gain = config.angle_p_gain;
            away_p_gain = config.away_p_gain;



}





int main(int argc, char** argv){
    ros::init(argc, argv, "map_follower");
    ros::NodeHandle nh;
    ros::NodeHandle cmd_nh_;
    ros::NodeHandle lidar_nh_;
    ros::NodeHandle odom_nh_;
    ros::NodeHandle nh_private("~");   
      

    ros::CallbackQueue callback_queue_cmd;
    cmd_nh_.setCallbackQueue(&callback_queue_cmd);    


    ros::CallbackQueue callback_queue_lidar;
    lidar_nh_.setCallbackQueue(&callback_queue_lidar);

    ros::CallbackQueue callback_queue_odom;
    odom_nh_.setCallbackQueue(&callback_queue_odom);
    
    mapfollower map_follower(nh,nh_private,cmd_nh_,lidar_nh_,odom_nh_);        


    std::thread spinner_thread_cmd([&callback_queue_cmd]() {
    ros::SingleThreadedSpinner spinner_cmd;
    spinner_cmd.spin(&callback_queue_cmd);
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
 
    spinner_thread_lidar.join();
    spinner_thread_odom.join();
    
    return 0;
}