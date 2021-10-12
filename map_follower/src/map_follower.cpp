/**
 * @file map_follower.cpp
 * @brief Main PLanning stack for Drone competition
 */
#include "map_follower.h"



mapfollower::~mapfollower() {}

mapfollower::mapfollower(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,const ros::NodeHandle& cmd_nh, const ros::NodeHandle& lidar_nh, const ros::NodeHandle& odom_nh)
: nh_(nh), nh_private_(nh_private), cmd_nh_(cmd_nh), lidar_nh_(lidar_nh), odom_nh_(odom_nh){     
    

    // idx_90=0;
    // idx_45=0;
    // idx_15=0;
    // idx_5 =0;
    // idx10 = 0;
    // idx_10 = 0;
    // idx_0 =0;
    // idx5  =0;
    // idx15 =0;
    // idx90 =0;
    // idx_85=0;
    // idx_60=0;
    
    // Dynamic Configure parameters
    f = boost::bind(&mapfollower::dyn_callback, this,_1, _2);
    server.setCallback(f);

    load_FSM_Params("mapfollower");  
    
    lidar_recieved = false;
    odom_received = false;
    
    lidar_sub = lidar_nh_.subscribe<sensor_msgs::LaserScan>("/laser/scan",1,&mapfollower::lidarCallback,this);    
    
    odom_sub = odom_nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",10, &mapfollower::odom_cb,this);  
    
    
    lidar_timer_ = lidar_nh_.createTimer(ros::Duration(1/cmd_hz), &mapfollower::lidarTimeCallback,this); //Critical -> allocate another thread 
    cmdloop_timer_ = cmd_nh_.createTimer(ros::Duration(1/cmd_hz), &mapfollower::posecmdloopCallback,this); // Critical -> allocate another thread 
    
    position_target_pub = nh_.advertise<mavros_msgs::PositionTarget>("/mapfollow/target/pose", 10);    
    position_target_pub_l = nh_.advertise<mavros_msgs::PositionTarget>("/mapfollow/target/pose_l", 10);    
   
    

}

void mapfollower::posecmdloopCallback(const ros::TimerEvent &event) {   
    if(!lidar_recieved || !odom_received){
        return;
    }
    pose_target_.header.stamp = ros::Time::now();
    pose_target_.header.frame_id ='m';        
    pose_target_.coordinate_frame = 1;
    pose_target_.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | 
                                     mavros_msgs::PositionTarget::IGNORE_VX  | 
                                     mavros_msgs::PositionTarget::IGNORE_VY  | 
                                     mavros_msgs::PositionTarget::IGNORE_VZ;                                    
    
    pose_target_.position.x = current_pose.position.x + speed_scale*forward_speed*cos(current_yaw);                             
    pose_target_.position.y = current_pose.position.y + speed_scale*forward_speed*sin(current_yaw);    
    pose_target_.position.z = 1.0;                                 
    pose_target_.yaw = current_yaw + delta_yaw*angle_scale;       
    
    position_target_pub.publish(pose_target_);    

///////////////////
    pose_target_l.header.stamp = ros::Time::now();
    pose_target_l.header.frame_id ='m';        
    pose_target_l.coordinate_frame = 1;
    pose_target_l.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | 
                                     mavros_msgs::PositionTarget::IGNORE_VX  | 
                                     mavros_msgs::PositionTarget::IGNORE_VY  | 
                                     mavros_msgs::PositionTarget::IGNORE_VZ;                                
    
    
    pose_target_l.position.x = current_pose.position.x + speed_scale*forward_speed_l*cos(current_yaw);                             
    pose_target_l.position.y = current_pose.position.y + speed_scale*forward_speed_l*sin(current_yaw);
    
    pose_target_l.position.z = 1.0;                                 
    pose_target_l.yaw = current_yaw - delta_yaw_l*angle_scale;         

    
    position_target_pub_l.publish(pose_target_l);    


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

double mapfollower::speed_mapping_from_angle(double angle) {   
    if (fabs(angle) >= 0.0 && fabs(angle) < 0.2){
            return 0.4;
    }else if(fabs(angle) >= 0.2 && fabs(angle) < 0.4 ){
                return 0.3;
    }else if(fabs(angle) >= 0.4 && fabs(angle) < 0.5 ){
                return 0.2;
    }else if(fabs(angle) >= 0.5 && fabs(angle) < max_angle ){
                return 0.05;
    }    
    return 0.1;  
}


void mapfollower::lidarTimeCallback(const ros::TimerEvent &event) {   
        if(!lidar_recieved){
            return ;
        }
        angle = 0.0;
        double P = kp*error;
        double D = kd*(error - prev_error)*cmd_hz;
        if (fabs(integral) > 1e-2 || integral < 1e10){
            integral = 0;
        }else{
            integral = integral + ki*error*1/cmd_hz;
        }            
        angle = P+D+integral;
        angle = std::max(std::min(angle, max_angle), -1.0*max_angle);        
        prev_error = error;

        delta_yaw = angle;        
        forward_speed = speed_mapping_from_angle(delta_yaw);
        
//////////////////
        angle_l = 0.0;
        double P_l = kp*error_l;
        double D_l = kd*(error_l - prev_error_l)*cmd_hz;
        if (fabs(integral_l) > 1e-2 || integral_l < 1e10){
            integral_l = 0;
        }else{
            integral_l = integral_l + ki*error_l*1/cmd_hz;
        }            
        angle_l = P_l+D_l+integral_l;
        angle_l = std::max(std::min(angle_l, max_angle), -1.0*max_angle);        
        prev_error_l = error_l;

        delta_yaw_l = angle_l;        
        forward_speed_l = speed_mapping_from_angle(delta_yaw_l);
        
        
    
}


int mapfollower::find_indx_from_anglx(double angle_in_degree){
    double input_angle;
    // input_angle = angle_in_radian;
    input_angle = angle_in_degree*M_PI/180;
    angle_wrap(input_angle);
    int found_idx = 0;
    double cur_angle;
    for (int i=0; i < lidar_data.ranges.size(); i++){
        cur_angle = lidar_data.angle_min + i*lidar_data.angle_increment;
        if(fabs(cur_angle - input_angle) <= lidar_data.angle_increment){
          found_idx = i;  
        }
    }
    if (found_idx < 0) {
        return -1;
    }
    

    return found_idx;
}


void mapfollower::lidarCallback(const sensor_msgs::LaserScanConstPtr &msg){
        lidar_data = *msg; 
        
        if(!lidar_recieved){
              idx_90 = 45*4;
             idx_85 = idx_90 + 5*4; 
             idx_80 = idx_90 + 10*4; 
             idx_60 = idx_90 + 30*4; 
             idx_45 = idx_90 + 45*4; 
             idx_15 = idx_90 + 75*4; 
             idx_10 = idx_90 + 80*4; 
             idx_5  = idx_90 + 85*4; 
             idx_0  = idx_90 + 90*4; 
             idx10  = idx_0 + 10*4; 
             idx5   = idx_0 + 5*4; 
             idx15  = idx_0 + 15*4; 
             idx90  = idx_0 + 90*4; 
                       
                  
                 lidar_recieved = true;
                 return;
            }   
        


        
    int tmp_min_idx= idx_10;
    double tmp_min= lidar_data.ranges[idx_10];
    for(int i= idx_10; i< idx10; i++){
        // finding minimum        
        if(lidar_data.ranges[i] > lidar_data.range_max || lidar_data.ranges[i] < lidar_data.range_min){
            continue;
        }
            if( tmp_min > lidar_data.ranges[i]){
                tmp_min = lidar_data.ranges[i];
                tmp_min_idx = i;
            }        
    }

    ////////////////////////
    int tmp_min_idx_= idx_60;
    double tmp_min_= lidar_data.ranges[idx_60];
    for(int i= idx_60; i< idx_0; i++){
        if(lidar_data.ranges[i] > lidar_data.range_max || lidar_data.ranges[i] < lidar_data.range_min){
            continue;
        }
        // finding minimum        
            if( tmp_min_ > lidar_data.ranges[i]){
                tmp_min_ = lidar_data.ranges[i];
                tmp_min_idx_ = i;
            }        
    }  

     
    

    ///////////////

    if(tmp_min > R2 && tmp_min_ > F1){
        follow_mode_ = FWmode::front_clear;
        
    }else{  
        bool block = false;
        for(int i = idx_45; i < idx_0; i++){
               if(lidar_data.ranges[i] > lidar_data.range_max || lidar_data.ranges[i] < lidar_data.range_min){
            continue;}
                if( lidar_data.ranges[i] < R1){
                block = true;                
                }
            } 
             if(block){
                 follow_mode_ = FWmode::front_follow;                
            }else{
                follow_mode_ = FWmode::narrow_follow;      
            }   

                  
            
    }
    ROS_INFO("mainFSM_mode = %s", stateToString(follow_mode_));    
    switch(follow_mode_){
        case FWmode::front_clear:{     
            tmp_min = fabs(lidar_data.ranges[idx_90]-R1);
            tmp_min_idx = idx_90;
            for(int i = idx_90; i < idx_0; i++){
                if(lidar_data.ranges[i] > lidar_data.range_max || lidar_data.ranges[i] < lidar_data.range_min){
            continue;
                 }
                if( tmp_min > fabs(lidar_data.ranges[i]-R1)){
                    tmp_min = fabs(lidar_data.ranges[i]-R1);
                    tmp_min_idx = i;   
                }
            }       

            double right_max_val = lidar_data.range_min;    
            int right_max_idx = idx_90;
            for(int i=idx_90;i<idx_85; i++){
                if(lidar_data.ranges[i] > lidar_data.range_max || lidar_data.ranges[i] < lidar_data.range_min){
            continue;
        }
                if( right_max_val < lidar_data.ranges[i]){
                    right_max_val = lidar_data.ranges[i];
                    right_max_idx = i;
                }
            }
                if(right_max_val > desired_distance*1.6){
            tmp_min_idx = idx_90;
            ROS_INFO("Huge Free space ");
                }
        break;
        }       

        case FWmode::narrow_follow:{                                    
            tmp_min = fabs(lidar_data.ranges[idx_90]-R1);
            tmp_min_idx = idx_90;
            for(int i = idx_90; i < idx_60; i++){
                if(lidar_data.ranges[i] > lidar_data.range_max || lidar_data.ranges[i] < lidar_data.range_min){
            continue;
        }
                if( tmp_min > fabs(lidar_data.ranges[i]-R1)){
                    tmp_min = fabs(lidar_data.ranges[i]-R1);
                    tmp_min_idx = i;   
                }
            }

            

        break;
        }

        case FWmode::front_follow:{  
            
            // tmp_min = fabs(lidar_data.ranges[idx_0]-R1);
            tmp_min = lidar_data.ranges[idx_0];
            tmp_min_idx = idx_0;

            for(int i = idx_0; i < idx90; i++){
                if(lidar_data.ranges[i] > lidar_data.range_max || lidar_data.ranges[i] < lidar_data.range_min){
            continue;}
                if( tmp_min > lidar_data.ranges[i]){
                tmp_min = lidar_data.ranges[i];
                tmp_min_idx = i;   
                }
            }  

            double right_max_val = lidar_data.range_min;    
            double right_min_val = lidar_data.range_max;
            int right_max_idx = idx_90;
            int right_min_idx = idx_60;
            for(int i=idx_90;i<idx_60; i++){
                if(lidar_data.ranges[i] > lidar_data.range_max || lidar_data.ranges[i] < lidar_data.range_min){
            continue;}
                if( right_max_val < lidar_data.ranges[i]){
                    right_max_val = lidar_data.ranges[i];
                    right_max_idx = i;
                }

                 if( right_min_val > lidar_data.ranges[i]){
                    right_min_val = lidar_data.ranges[i];
                    right_min_idx = i;
                }
            }
            if(right_max_val > desired_distance*2){
            tmp_min_idx = right_max_idx;
            ROS_INFO("RIGHT Free space ");
                }
            
        break;
        }

        default:
          ROS_INFO("default flag on..something wrong");
        break; 
    }
    
    ROS_INFO("tmp_min_idx = %d", tmp_min_idx);
    double tmp_agle_degree = (lidar_data.angle_min+tmp_min_idx*lidar_data.angle_increment)*180/M_PI;
    ROS_INFO("tmp_angle in degree  = %f", tmp_agle_degree);
    
    ROS_INFO("distance = %f", lidar_data.ranges[tmp_min_idx]);

     
    int idx60 = tmp_min_idx;   
        
      
 
    double l_dist_b = lidar_data.ranges[idx90];
    double l_angle_b  = lidar_data.angle_min + lidar_data.angle_increment*idx90;

    double l_dist_a = lidar_data.ranges[idx60];
    double l_angle_a  = lidar_data.angle_min + lidar_data.angle_increment*idx60;

    double r_dist_b = lidar_data.ranges[idx_90];
    double r_angle_b  = lidar_data.angle_min + lidar_data.angle_increment*idx_90;

    double r_dist_a = lidar_data.ranges[tmp_min_idx];
    double r_angle_a  = lidar_data.angle_min + lidar_data.angle_increment*tmp_min_idx;


    double r_theta = (r_angle_a - r_angle_b );
    double l_theta = fabs(l_angle_a - l_angle_b );
    

    double l_alpha  = atan2((l_dist_a * cos(l_theta) - l_dist_b) , (l_dist_a*sin(l_theta)));    
    double r_alpha  = atan2((r_dist_a * cos(r_theta) - r_dist_b) , (r_dist_a*sin(r_theta)));    
    
    double l_Dt = l_dist_b*cos(l_alpha);
    double r_Dt = r_dist_b*cos(r_alpha);    
    
    double Dt_l = l_Dt + lookahead*sin(l_alpha);                    
    double Dt = r_Dt + lookahead*sin(r_alpha);                    
    error = desired_distance - Dt;  
    error_l = desired_distance - Dt_l;  
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
            
            cmd_hz = config.cmd_hz;
            desired_distance = config.desired_distance;            
            lookahead = config.lookahead;
            kp = config.kp;
            kd = config.kd;
            ki = config.ki;
            speed_scale = config.speed_scale;
            angle_scale = config.angle_scale;
              R2 = config.R2; 
            R1 = config.R1; 
            DT = config.DT; 
            F1 = config.F1;



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