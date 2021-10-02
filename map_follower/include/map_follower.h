#include <ros/ros.h>
#include <tf/tf.h>
#include <cstdlib>
#include <math.h>
#include <algorithm> 
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
// #include <mavros/setpoint_mixin.h>
// #include <mavros/frame_tf.h>
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
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <quadrotor_msgs/PositionCommand.h>

#include <sensor_msgs/PointCloud2.h>


#include <planner_msgs/pci_global.h>

#include <dynamic_reconfigure/server.h>
#include <map_follower/dyn_paramsConfig.h>






class mapfollower{

    ros::NodeHandle nh_;    
    ros::NodeHandle nh_private_;
    ros::NodeHandle cmd_nh_;
    ros::NodeHandle lidar_nh_;
    ros::NodeHandle odom_nh_;
    

    ros::Subscriber odom_sub;   

    ros::Subscriber lidar_sub;

    tf2_ros::Buffer tfBuffer;      
   
    tf::TransformBroadcaster tf_broadcaster;
    tf::TransformListener tf_listener_;
    tf::StampedTransform cam_to_world;
    tf::Transform cam_to_world_tf;
    tf::Transform cam_to_base_link;
    geometry_msgs::TransformStamped transformStamped_tmp;
    tf2_ros::TransformBroadcaster tf_broadcaster_;


    
    ros::Timer cmdloop_timer_;
    
    ros::Timer lidar_timer_;
    ros::Publisher position_target_pub, position_target_pub_l;
    
     
    mavros_msgs::State current_state;    
    mavros_msgs::PositionTarget pose_target_;    
    mavros_msgs::PositionTarget pose_target_l;    
    geometry_msgs::Pose current_pose;
    geometry_msgs::Pose target_pose;
    
    double current_yaw; 
    double target_yaw;
    nav_msgs::Odometry odom_state;     
    
    sensor_msgs::LaserScan lidar_data;
    
    dynamic_reconfigure::Server<map_follower::dyn_paramsConfig> server;
    dynamic_reconfigure::Server<map_follower::dyn_paramsConfig>::CallbackType f;
    
    
    darknet_ros_msgs::BoundingBoxes detected_bbx;

    float global_planner_target_x, global_planner_target_y, global_planner_target_z;
    //switch varialbes 
    bool armed;
    bool offboarded;
    bool odom_received;
   

     int localsearch_count;
    
    // parameter values
    double lidar_avoidance_distance_;
    double lidar_avoidance_move_distance_;
    double target_vel;
    double cmd_hz;
    double laser_right_min_val;
    int laser_right_min_idx;
    double angle_shift_in_degree;
    double laser_away_dist;
    double away_p_gain;    
    double move_p_gain;
    double angle_p_gain;
    double lookahead;
    double error, prev_error, integral;
    double desired_distance;
    double kp, kd, ki;
    double max_angle = 0.5;
    double angle, delta_yaw; 
    double forward_speed;
    double speed_scale;
    double angle_scale;
    double error_l, angle_l, prev_error_l, integral_l;
    double delta_yaw_l, forward_speed_l;

    int idx_90;
    int idx_60; 
    int idx_25;    
    int idx25; 
    int idx60;
    int idx90; 
    int idx_0; 
    int idx_80; 
    bool lidar_recieved;

    double lidar_min_threshold;
    double init_takeoff_;
    double global_pose_x_min, global_pose_x_max, global_pose_y_min, global_pose_y_max, global_pose_z_min, global_pose_z_max;
    bool verbos;
    int FSM_mode;
    
    double speed_mapping_from_angle(double angle);
    //health check
    bool check_if_drone_outside_global_box();    
     
    // utility functions 
    double get_distance(geometry_msgs::Pose &p1,geometry_msgs::Pose &p2);
    void load_FSM_Params(std::string group);      

    void angle_wrap(double &angle);
    void minmaxcast(double &value,double min=0.0, double max=100.0);
    // sensors callback
    void lidarCallback(const sensor_msgs::LaserScanConstPtr &msg);
    
    void posecmdloopCallback(const ros::TimerEvent &event);  
    void lidarTimeCallback(const ros::TimerEvent &event);  
    
    int find_indx_from_anglx(double angle_in_radian);
    
    void odom_cb(const nav_msgs::OdometryConstPtr& msg);
   
    void dyn_callback(const map_follower::dyn_paramsConfig &config, uint32_t level);
    

   
public:
    mapfollower(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, const ros::NodeHandle& cmd_nh, const ros::NodeHandle& lidar_nh, const ros::NodeHandle& odom_nh);
    ~mapfollower();
    
};
