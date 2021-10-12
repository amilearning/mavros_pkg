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
         
    
  
    odom_received = false;
    
    lidar_sub = lidar_nh_.subscribe<sensor_msgs::LaserScan>("/laser/scan",1,&mapfollower::lidarCallback,this);    
    
    odom_sub = odom_nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",10, &mapfollower::odom_cb,this);  
    
    
    lidar_timer_ = lidar_nh_.createTimer(ros::Duration(1/cmd_hz), &mapfollower::lidarTimeCallback,this); //Critical -> allocate another thread 
    cmdloop_timer_ = cmd_nh_.createTimer(ros::Duration(1/cmd_hz), &mapfollower::posecmdloopCallback,this); // Critical -> allocate another thread 
    
    position_target_pub = nh_.advertise<mavros_msgs::PositionTarget>("/mapfollow/target/pose", 10);    
    position_target_pub_l = nh_.advertise<mavros_msgs::PositionTarget>("/mapfollow/target/pose_l", 10);    
    
    

}

void mapfollower::posecmdloopCallback(const ros::TimerEvent &event) {   
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
    
    double idx_90 = find_indx_from_anglx(-90);    
    double idx_45 = find_indx_from_anglx(-45); 
    double idx_15 = find_indx_from_anglx(-15);    
    double idx_5 = find_indx_from_anglx(-5);    
    double idx_0 = find_indx_from_anglx(0);    
    double idx5 = find_indx_from_anglx(5);    
    double idx15 = find_indx_from_anglx(15);   
    double idx90 = find_indx_from_anglx(90); 

    double idx_85 = find_indx_from_anglx(-85); 
    double idx_60 = find_indx_from_anglx(-60); 
    
    
    // double idx_90 = 45*4; 
    // double idx_60 = idx_90 + 30*4; 
    // double idx_25 = idx_90 + 65*4;    
    // double idx25 = idx_25 + 50*4; 
    // double idx60 = idx25 + 35*4;
    // double idx90 = idx25 + 65*4;
    
    double tmp_min_idx= idx_15;
    double tmp_min= lidar_data.ranges[idx_15];
    for(int i= idx_5; i< idx5; i++){
        // finding minimum        
            if( tmp_min > lidar_data.ranges[i]){
                tmp_min = lidar_data.ranges[i];
                tmp_min_idx = i;
            }        
    }

    ////////////////////////
    double tmp_min_idx_= idx_60;
    double tmp_min_= lidar_data.ranges[idx_60];
    for(int i= idx_60; i< idx_0; i++){
        // finding minimum        
            if( tmp_min_ > lidar_data.ranges[i]){
                tmp_min_ = lidar_data.ranges[i];
                tmp_min_idx_ = i;
            }        
    }  


    ///////////////

    if(tmp_min > R2 && tmp_min_ > DT){
        follow_mode_ = FWmode::front_clear;
        
    }else{
            bool block = false;
            for(int i = idx_45; i < idx_0; i++){
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
                if( tmp_min > fabs(lidar_data.ranges[i]-R1)){
                    tmp_min = fabs(lidar_data.ranges[i]-R1);
                    tmp_min_idx = i;   
                }
            }       

            double right_max_val = lidar_data.range_min;    
            int right_max_idx = idx_90;
            for(int i=idx_90;i<idx_85; i++){
                if( right_max_val < lidar_data.ranges[i]){
                    right_max_val = lidar_data.ranges[i];
                    right_max_idx = i;
                }
            }
                if(right_max_val > desired_distance*3.5){
            tmp_min_idx = idx_85;
            ROS_INFO("Huge Free space ");
                }
        break;
        }       

        case FWmode::narrow_follow:{                                    
            tmp_min = fabs(lidar_data.ranges[idx_90]-R1);
            tmp_min_idx = idx_90;
            for(int i = idx_90; i < idx_60; i++){
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
                if( tmp_min > lidar_data.ranges[i]){
                tmp_min = lidar_data.ranges[i];
                tmp_min_idx = i;   
                }
            }  
            
        break;
        }

        default:
          ROS_INFO("default flag on..something wrong");
        break; 
    }
    
        

     idx_60 = tmp_min_idx;   
    double idx60 = tmp_min_idx;   
// ///////////////////////////////////////////////////////
//     bool current_wall_follow = false;
//     if(follow_mode == wallfollowing_mode::current_wall_follow){
//          tmp_min = fabs(lidar_data.ranges[idx_90]-R1);
//          tmp_min_idx = idx_90;

//             for(int i = idx_90; i < idx_0; i++){
//                 if( tmp_min > fabs(lidar_data.ranges[i]-R1)){
//                 tmp_min = fabs(lidar_data.ranges[i]-R1);
//                 tmp_min_idx = i;   
//                 }
//             }
//              current_wall_follow = true;        

//     }else if(follow_mode = wallfollowing_mode::choose_direction){
        
//         // 1st check 
            
//             if(block){
//                follow_mode =  wallfollowing_mode::front_wall_follow;
//             }
//         // 2nd check 
//              for(int i = idx_90; i < idx_45; i++){
//                 if( lidar_data.ranges[i] < R1){
//                     follow_mode =  wallfollowing_mode::front_clearance;
//                 }
//             }
            
//     }else if (follow_mode == wallfollowing_mode::front_wall_follow){
//              tmp_min = fabs(lidar_data.ranges[idx_0]-R1);
//              tmp_min_idx = idx_0;

//             for(int i = idx_0; i < idx90; i++){
//                 if( tmp_min > fabs(lidar_data.ranges[i]-R1)){
//                 tmp_min = fabs(lidar_data.ranges[i]-R1);
//                 tmp_min_idx = i;   
//                 }
//             }
// ///////////////////////////////////////////////////////
//     bool current_wall_follow = false;
//     if(follow_mode == wallfollowing_mode::current_wall_follow){
//          tmp_min = fabs(lidar_data.ranges[idx_90]-R1);
//          tmp_min_idx = idx_90;

//             for(int i = idx_90; i < idx_0; i++){
//                 if( tmp_min > fabs(lidar_data.ranges[i]-R1)){
//                 tmp_min = fabs(lidar_data.ranges[i]-R1);
//                 tmp_min_idx = i;   
//                 }
//             }
//              current_wall_follow = true;        

//     }else if(follow_mode = wallfollowing_mode::choose_direction){
        
//         // 1st check 
            
//             if(block){
//                follow_mode =  wallfollowing_mode::front_wall_follow;
//             }
//         // 2nd check 
//              for(int i = idx_90; i < idx_45; i++){
//                 if( lidar_data.ranges[i] < R1){
//                     follow_mode =  wallfollowing_mode::front_clearance;
//                 }
//             }
            
//     }else if (follow_mode == wallfollowing_mode::front_wall_follow){
//              tmp_min = fabs(lidar_data.ranges[idx_0]-R1);
//              tmp_min_idx = idx_0;

//             for(int i = idx_0; i < idx90; i++){
//                 if( tmp_min > fabs(lidar_data.ranges[i]-R1)){
//                 tmp_min = fabs(lidar_data.ranges[i]-R1);
//                 tmp_min_idx = i;   
//                 }
//             }



//             if(current_wall_follow){
//                 follow_mode == wallfollowing_mode::current_wall_follow
//             }
//             /////////////////                
//     }
//         double front_min_val = lidar_data.range_max;
//         double front_min_idx = (int)lidar_data.ranges.size()/2-20;
//          for( int i= (int)lidar_data.ranges.size()/2-20; i <(int)lidar_data.ranges.size()/2+20; i++){
//             if(front_min_val > lidar_data.ranges[i]){
//                 front_min_val = lidar_data.ranges[i];
//                 front_min_idx = i;
//             }
//         }  
//         if (front_min_val < desired_distance*2.0){
//             idx_60 = front_min_idx;
//         }
//             if(current_wall_follow){
//                 follow_mode == wallfollowing_mode::current_wall_follow
//             }
//             /////////////////                
//     }
//         double front_min_val = lidar_data.range_max;
//         double front_min_idx = (int)lidar_data.ranges.size()/2-20;
//          for( int i= (int)lidar_data.ranges.size()/2-20; i <(int)lidar_data.ranges.size()/2+20; i++){
//             if(front_min_val > lidar_data.ranges[i]){
//                 front_min_val = lidar_data.ranges[i];
//                 front_min_idx = i;
//             }
//         }  
//         if (front_min_val < desired_distance*2.0){
//             idx_60 = front_min_idx;
//         }
    /////////////////////
    // double break_point_idx = 0;
    // for( int i= idx_90; i <(int)lidar_data.ranges.size()/2; i++){
    //         double thres_dist = sqrt(pow(lidar_data.ranges[i],2)+pow(lidar_data.ranges[i],2)-2*pow(lidar_data.ranges[i],2)
                 
    //         );
    //         if(fabs(lidar_data.ranges[i] - lidar_data.ranges[i+1]) > 
                
    //             front_min_val > lidar_data.ranges[i]){
    //             front_min_val = lidar_data.ranges[i];
    //             front_min_idx = i;
    //         }
    //     }  
    /////////////////////
 
    double l_dist_b = lidar_data.ranges[idx90];
    double l_angle_b  = lidar_data.angle_min + lidar_data.angle_increment*idx90;

    double l_dist_a = lidar_data.ranges[idx60];
    double l_angle_a  = lidar_data.angle_min + lidar_data.angle_increment*idx60;

    double r_dist_b = lidar_data.ranges[idx_90];
    double r_angle_b  = lidar_data.angle_min + lidar_data.angle_increment*idx_90;

    double r_dist_a = lidar_data.ranges[idx_60];
    double r_angle_a  = lidar_data.angle_min + lidar_data.angle_increment*idx_60;


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
    // ROS_INFO("error_l = %f",error_l);
    //  ROS_INFO("error = %f",error);
    
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