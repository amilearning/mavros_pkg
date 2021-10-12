#include <ros/ros.h>
#include <tf/tf.h>
#include <cstdlib>
#include <math.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/BatteryState.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
// #include <mavros/setpoint_mixin.h>
// #include <mavros/frame_tf.h>
#include <functional>
#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>

#include <sensor_fusion_comm/InitScale.h>
#include <mavros_msgs/CommandLong.h>


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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


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
#include <hmcl_fsm/dyn_paramsConfig.h>




enum struct mainFSMmode {Init = 0, Exploration = 1, Avoidance = 2, Landing = 3, RTB = 4};    
enum struct ExploreMode {LocalSearching = 0, GlobalSearching = 1};        
enum struct LandMode {NearHome = 0, MovetoHome = 1, Landing = 2};    
enum struct RTBmode {Init = 0, Search = 1, YawAglign = 2, Move = 3, Init_yaw = 4, MoveNear =5};        
enum struct LocalPlanMode {Planning = 0, YawMatching = 1, WaypointRequest = 2};        


inline const char* stateToString(LocalPlanMode v)
{
    switch (v)
    {
        case LocalPlanMode::Planning:   return "Planning";
        case LocalPlanMode::YawMatching:   return "YawMatching";
        case LocalPlanMode::WaypointRequest: return "WaypointRequest";              
        default:      return "[Unknown mainFSMmode]";
    }
}


inline const char* stateToString(RTBmode v)
{
    switch (v)
    {
        case RTBmode::Init:   return "Init";
        case RTBmode::Search:   return "Search";
        case RTBmode::YawAglign: return "YawAglign";
        case RTBmode::Move: return "Move";        
        case RTBmode::Init_yaw: return "Init_yaw";  
        case RTBmode::MoveNear: return "MoveNear";          
        default:      return "[Unknown mainFSMmode]";
    }
}


inline const char* stateToString(mainFSMmode v)
{
    switch (v)
    {        
        case mainFSMmode::Init:   return "Init";
        case mainFSMmode::Exploration: return "Exploration";
        case mainFSMmode::Avoidance: return "Avoidance";        
        case mainFSMmode::Landing: return "Landing";
        default:      return "[Unknown mainFSMmode]";
    }
}

inline const char* stateToString(ExploreMode v)
{
    switch (v)
    {        
        case ExploreMode::LocalSearching:   return "LocalSearching";
        case ExploreMode::GlobalSearching: return "GlobalSearching";               
        default:      return "[Unknown ExploreMode]";
    }
}



inline const char* stateToString(LandMode v)
{
    switch (v)
    {        
        case LandMode::NearHome:   return "NearHome";
        case LandMode::MovetoHome: return "MovetoHome";
        case LandMode::Landing: return "Landing";        
        default:      return "[Unknown LandMode]";
    }
}


class hmclFSM{
private:
    ros::NodeHandle nh_;    
    ros::NodeHandle nh_private_;
    ros::NodeHandle cmd_nh_;
    ros::NodeHandle fsm_nh_;
    ros::NodeHandle lidar_nh_;
    ros::NodeHandle odom_nh_;
    
  
    // Define services
    ros::ServiceClient explore_client_start;
    ros::ServiceClient explore_client_stop;
    ros::ServiceClient explore_client_home;
    ros::ServiceClient explore_client_global_planner;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient px4_cmd_client;
    ros::ServiceClient ekf_reinit_client;
    
     

    ros::Subscriber multiDOFJointSub;
    ros::Subscriber odom_sub;   
    ros::Subscriber vision_odom_sub;
    ros::Subscriber battery_state_sub;
    ros::Subscriber state_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber mpcCommand_sub;
    ros::Subscriber bbx_sub; 
    ros::Subscriber pos_cmd_sub;
    ros::Subscriber local_path_trigger_sub;
    ros::Subscriber mav_vel_sub;
    ros::Subscriber ndt_fused_pose_sub;
    ros::Subscriber global_direction_sub;

    ros::Subscriber points_sub;

    ros::Subscriber wallfollower_r_sub;
    ros::Subscriber wallfollower_l_sub;

    
    tf2_ros::Buffer tfBuffer;      
   
    tf::TransformBroadcaster tf_broadcaster;
    tf::TransformListener tf_listener_;
    tf::StampedTransform cam_to_world;
    tf::Transform cam_to_world_tf;
    tf::Transform cam_to_base_link;
    geometry_msgs::TransformStamped transformStamped_tmp;
    tf2_ros::TransformBroadcaster tf_broadcaster_;


    sensor_msgs::PointCloud2 cloud_in, cloud_out;

    
    
    int emergency_landing_count;
    ros::Timer fsm_timer_;
    ros::Timer cmdloop_timer_;
    ros::Timer waypoint_iter_timer_; 
    double waypoint_time_in_sec, waypoint_time_in_sec_prev, target_duration_time_in_sec;    
    std::deque<ros::Duration> command_waiting_times_;
    int check_count = 0;
    int pose_init_cali_succ_count; 
    sensor_fusion_comm::InitScale ekf_init_param;
    bool cali_done;
    bool re_init;
    ros::Publisher rtb_pub;
    int stuck_count;
    int control_count_tmp;
    int init_count;
    ros::Publisher rpyt_pub;
    ros::Publisher position_target_pub;
    ros::Publisher local_pos_pub;
    ros::Publisher mpc_cmd_pub;
    ros::Publisher manual_trj_pub;
    ros::Publisher vis_pos_pub;
    ros::Publisher vins_odom_pub;
    ros::Publisher camera_points_pub;
    ros::Publisher local_goal_pub;
    ros::Publisher local_avoidance_switch_pub;
    ros::Publisher avoidance_vector_vis_pub;
    
    quadrotor_msgs::PositionCommand pose_cmd;
    mav_msgs::RollPitchYawrateThrust mpc_cmd;
    
    
    
    
    
    bool mpc_cmd_enable;
    bool avoidance_enable;
    bool local_trj_switch_, local_target_send_;
    bool local_trj_enable;
    bool local_path_received;
    ros::Time last_request;
    mavros_msgs::SetMode offb_set_mode;   
    mavros_msgs::CommandBool arm_cmd; 
    mavros_msgs::CommandLong px4_reboot_cmd;
    mavros_msgs::CommandLong px4_kill_cmd;
    mavros_msgs::CommandLong px4_landinggear_cmd;
    mavros_msgs::State current_state;    
    double current_battery;    
    mavros_msgs::PositionTarget pose_target_; 
    mavros_msgs::PositionTarget tmp_target_;
    bool pose_cmd_enable;
    geometry_msgs::Pose current_pose,pose_at_request;
    geometry_msgs::Pose prev_pose_for_check;
    ros::Duration time_diff;
    ros::Time prev_update_time;
    geometry_msgs::Pose target_pose;
    geometry_msgs::Pose global_planner_target_pose;
    geometry_msgs::Pose previous_pose;
    geometry_msgs::TransformStamped vis_pose, ndt_fused_pose;
    bool ndt_fused_pose_listen;
    int goal_request_count;
    double current_yaw; 
    double yaw_rate_max = 0.35;
    double yaw_scale = 1.0;
    nav_msgs::Odometry odom_state; 
    nav_msgs::Odometry vins_odom_state; 

    mavros_msgs::AttitudeTarget att;
    trajectory_msgs::MultiDOFJointTrajectory waypoints;
    bool manual_trj_switch_;
    double target_x,target_y,target_z,target_yaw;
    double lidar_final_avoidance_distance;
    
    sensor_msgs::LaserScan lidar_data;

    double att_clb_time_in_sec, att_clb_time_in_sec_prev;
    bool att_clb_first_callback; 
    

    dynamic_reconfigure::Server<hmcl_fsm::dyn_paramsConfig> server;
    dynamic_reconfigure::Server<hmcl_fsm::dyn_paramsConfig>::CallbackType f;
    
    
    int avoid_perpendicualar_factor;

    int waypoints_itr;
    bool send_waypoint;
    float global_planner_target_x, global_planner_target_y, global_planner_target_z;
    //switch varialbes 
    bool armed;
    bool offboarded;
    bool odom_received;
   

    // State machine variables
     mainFSMmode mainFSM_mode, prev_mainFSM_mode;   
    //  mainFSMmode previous_mode;  
     RTBmode    RTB_mode;
     ExploreMode Explore_Mode;     
     LocalPlanMode LocalPlan_Mode;
     
     LandMode Land_Mode;
     int localsearch_count;
    
    
    // parameter values
    double lidar_avoidance_distance_;
    double lidar_avoidance_move_distance_, vector_avoidance_scale;
    double d0;
    double k0; 
    double lidar_min_threshold;
    double init_takeoff_;
    double global_pose_x_min, global_pose_x_max, global_pose_y_min, global_pose_y_max, global_pose_z_min, global_pose_z_max;
    bool verbos;
    int FSM_mode;
    bool wall_r_follow, wall_l_follow;
    geometry_msgs::PoseStamped avoidance_vector_display;
    double battery_thres;
    bool if_stuck;
    //////////////////////Landing //////////////////////////


    bool landing;








    //////////////////////////////////////////////////
    //////////////////////////// RTB //////////////////////////
    double total_run_distance;
    double x_check, y_check, x_dc, y_dc ;
    bool rtb;
    bool rtb_once;
    int current_min_idx, prev_min_idx;
    std::vector<float> wp_x, wp_y;
    double current_x, current_y;
    double safe_distance;     
    double prev_wp_yaw; 

    double A_sw= 45.0/180.0*M_PI;        // Angle threshold for STORE_WP
    double D_sw= 1;                      // Distance threshold for STORE_WP
    double D_sp= 0.2;                    // Distance threshold for judge sepoint arrival

    double global_goal_dir, prev_global_goal_dir;

    void check_rtb_init();
    void STORE_WP();
    bool CHECK_CLEARANCE_AVOIDANCE(int lidar_idx_c, double expected);
    int get_closest_wp_idx();
    int get_safe_wp_idx();
    //////////////////////////////////////////////////////////
    //
    double mav_vel_x, mav_vel_y;
    bool offboard_and_arm();
    void check_emergency_landing();
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    // Main FSM Callback
    void mainFSMCallback(const ros::TimerEvent &event);        
    void printFSMstate();

    //local FSM callback
    void localFSMCallback(const ros::TimerEvent &event);        
    
    //health check
    bool check_if_drone_outside_global_box();
    
    //mainFSmode related functions
    void init_takeoff();
    bool init_cali();
    bool check_if_stuck();
    void angle_wrap(double &angle);
    // utility functions 
    double get_distance(geometry_msgs::Pose &p1,geometry_msgs::Pose &p2);
    void load_FSM_Params(std::string group);
    // control related callback
    void cmdloopCallback(const ros::TimerEvent &event);    
    
    

    void wallFollowCmdCallback_r(const mavros_msgs::PositionTargetConstPtr &msg);
    void wallFollowCmdCallback_l(const mavros_msgs::PositionTargetConstPtr &msg);

    void sendManualTrajectory();
    void poseCmdCallback(const quadrotor_msgs::PositionCommandConstPtr &msg);

    void localTrajTrigCallback(const std_msgs::EmptyConstPtr &msg);

    void goaldirectionCallback(const std_msgs::Float32ConstPtr &msg);
   

    // sensors callback
    void lidarCallback(const sensor_msgs::LaserScanConstPtr &msg);    
    

    
    
    void odom_cb(const nav_msgs::OdometryConstPtr& msg);
    void ndt_fused_cb(const geometry_msgs::TransformStampedConstPtr& msg);
    
    void visCallback(const geometry_msgs::PoseStampedConstPtr& msg);    
    void battery_state_cb(const sensor_msgs::BatteryStateConstPtr& msg);
    
    void local_avoidance(double min_distance);    
    
    void dyn_callback(const hmcl_fsm::dyn_paramsConfig &config, uint32_t level);
    

   
public:
    hmclFSM(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, const ros::NodeHandle& cmd_nh, const ros::NodeHandle& fsm_nh, const ros::NodeHandle& lidar_nh, const ros::NodeHandle& odom_nh);
    ~hmclFSM();
    
};
