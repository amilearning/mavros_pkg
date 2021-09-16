/**
 * @file pose_fuse.cpp
 * @brief fuse pose data from 2d localization module with 1d height range data
 */
#include "pose_fuse.h"


poseFuse::~poseFuse() {}

poseFuse::poseFuse(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
: nh_(nh), nh_private_(nh_private){         
    // lidar_lite_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/ndtpso_slam_front/pose",10, &hmclFSM::ndtPoseCallback,this);    
    // imu_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/imu/data",1, &hmclFSM::imu_callback,this);        
    // ndt_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/ndtpso_slam_front/pose",10, &hmclFSM::ndtPoseCallback,this);    


//   tf::Quaternion q(current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w);
//     tf::Matrix3x3 m(q);            
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);        
//     current_yaw = yaw;

    fused_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
   
    load_Params("pf");        
    print_FSM_Params();
    
    

}

void poseFuse::load_Params(std::string group){
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


void poseFuse::dyn_callback(const pose_fuse::dyn_paramsConfig &config, uint32_t level) {  
            d0 = config.d0;
            k0 = config.k0;
            thrust_scale = config.thrust_scale;
            manual_trj_switch_ = config.manual_trj_switch;
            target_x=config.target_x;
            target_y=config.target_y;
            target_z=config.target_z;
            ROS_INFO("d0 = %f, k= %f, thrust_scale = %f", d0,k0,thrust_scale);            
}


int main(int argc, char** argv){
    ros::init(argc, argv, "pose_fuse");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");     
    poseFuse pose_fuse(nh,nh_private);  

    ros::spin();
    
    return 0;
}