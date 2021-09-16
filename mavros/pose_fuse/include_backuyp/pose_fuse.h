#include <ros/ros.h>
#include <tf/tf.h>
#include <cstdlib>
#include <math.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

#include <functional>
#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/default_topics.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_broadcaster.h>


#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>




#include <dynamic_reconfigure/server.h>
#include <pose_fuse/dyn_paramsConfig.h>




class poseFuse{
private:
    ros::NodeHandle nh_;    
    ros::NodeHandle nh_private_;
    ros::NodeHandle cmd_nh_;
    ros::NodeHandle fsm_nh_;
    ros::NodeHandle lidar_nh_;
    
    ros::Subscriber ndt_sub;
    ros::Subscriber imu_sub;    
    ros::Subscriber lidar_lite_sub;
        
    tf2_ros::Buffer tfBuffer;      
   
    tf::TransformBroadcaster tf_broadcaster;
    tf::TransformListener tf_listener_;
    tf::StampedTransform cam_to_world;
    tf::Transform cam_to_world_tf;
    tf::Transform cam_to_base_link;
    geometry_msgs::TransformStamped transformStamped_tmp;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    
    
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> ndt_sub;
    message_filters::Subscriber<sensor_msgs::Range> range_sub;
    message_filters::TimeSynchronizer<sensor_msgs::Imu,sensor_msgs::Range,geometry_msgs::PoseStamped> timeSynchronizer_;    
    

    ros::Publisher fused_pose_pub;
        
    sensor_msgs::Range lidar_lite_data;    
    
    dynamic_reconfigure::Server<pose_fuse::dyn_paramsConfig> server;
    dynamic_reconfigure::Server<pose_fuse::dyn_paramsConfig>::CallbackType f;
    
    // parameter values
    double lidar_avoidance_distance_;
    double d0;
    double k0; 
    double init_takeoff_;
    double global_pose_x_min, global_pose_x_max, global_pose_y_min, global_pose_y_max, global_pose_z_min, global_pose_z_max;
    bool verbos;

    void load_Params(std::string group);
    void dyn_callback(const pose_fuse::dyn_paramsConfig &config, uint32_t level);
    

   
public:
    poseFuse(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~poseFuse();
    
};
