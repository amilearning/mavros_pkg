#include <ros/ros.h>
#include <tf/tf.h>
#include <cstdlib>
#include <math.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
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
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <quadrotor_msgs/PositionCommand.h>




#include <planner_msgs/pci_global.h>

#include <dynamic_reconfigure/server.h>
#include <offboard_ctrl/dyn_paramsConfig.h>

class OffboardFSM{
private:
    ros::NodeHandle nh_;    
    ros::NodeHandle nh_private_;

    ros::ServiceClient explore_client_start;
    ros::ServiceClient explore_client_stop;
    ros::ServiceClient explore_client_home;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    ros::Subscriber multiDOFJointSub;
    ros::Subscriber odom_sub;     
    ros::Subscriber state_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber mpcCommand_sub;
    ros::Subscriber bbx_sub; 
    ros::Subscriber pos_cmd_sub;

    ros::Timer cmdloop_timer_;
    ros::Timer waypoint_iter_timer_; 
    double waypoint_time_in_sec, waypoint_time_in_sec_prev, target_duration_time_in_sec;    
    std::deque<ros::Duration> command_waiting_times_;

    ros::Timer lidar_timer_;
    
    ros::Publisher rpyt_pub;
    ros::Publisher position_target_pub;
    ros::Publisher local_pos_pub;
    ros::Publisher mpc_cmd_pub;
    ros::Publisher manual_trj_pub;
    
    quadrotor_msgs::PositionCommand pose_cmd;
    mav_msgs::RollPitchYawrateThrust mpc_cmd;
    bool mpc_cmd_enable;
    bool avoidance_enable;
    mavros_msgs::SetMode offb_set_mode;   
    mavros_msgs::CommandBool arm_cmd; 
    mavros_msgs::State current_state;    
    mavros_msgs::PositionTarget pose_target_;
    bool pose_cmd_enable;
    geometry_msgs::Pose current_pose;
    double current_yaw; 
    double yaw_scale = 1.0;
    double thrust_scale = 0.01;
    nav_msgs::Odometry odom_state; 
    mavros_msgs::AttitudeTarget att;
    trajectory_msgs::MultiDOFJointTrajectory waypoints;
    bool manual_trj_switch_;
    double target_x,target_y,target_z;
    
    sensor_msgs::LaserScan lidar_data;

    double att_clb_time_in_sec, att_clb_time_in_sec_prev;
    bool att_clb_first_callback; 
    

    dynamic_reconfigure::Server<offboard_ctrl::dyn_paramsConfig> server;
    dynamic_reconfigure::Server<offboard_ctrl::dyn_paramsConfig>::CallbackType f;
    
    
    darknet_ros_msgs::BoundingBoxes detected_bbx;

    int waypoints_itr;
    bool send_waypoint;
    int FSM_mode;
    double lidar_avoidance_distance = 1.0;
    double d0;
    double k0; 
 
    // control related callback
    void cmdloopCallback(const ros::TimerEvent &event);    
    void waypointTimerCallback(const ros::TimerEvent &event); 
    void multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg);
    void mpcCommandCallback(const mav_msgs::RollPitchYawrateThrustConstPtr &msg);
    void sendManualTrajectory();
    void poseCmdCallback(const quadrotor_msgs::PositionCommandConstPtr &msg);


    // sensors callback
    void lidarCallback(const sensor_msgs::LaserScanConstPtr &msg);
    void lidarTimeCallback(const ros::TimerEvent &event);
    void bbxCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg);


    void odom_cb(const nav_msgs::OdometryConstPtr& msg);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void check_drone_status();
    void local_avoidance();
    void refine_path_via_lidarData();
    void set_target_pose(double x,double y, double z, double yaw);
    void dyn_callback(const offboard_ctrl::dyn_paramsConfig &config, uint32_t level);
    
    
public:
    OffboardFSM(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~OffboardFSM();
    enum struct FSMmode {Homing = 0, Exploration = 1, TargetApproach = 3, Landing = 4};
     
};
