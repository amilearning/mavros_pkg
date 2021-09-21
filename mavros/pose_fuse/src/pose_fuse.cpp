#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>  

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_broadcaster.h>

#include <dynamic_reconfigure/server.h>
#include <pose_fuse/dyn_paramsConfig.h>
#include <list>

#define PI 3.14159265

using namespace message_filters;
double range_data_filtered_;
geometry_msgs::TransformStamped fused_pose_; 
bool sub_switch;
ros::Publisher fused_pose_pub; 
double filter_alpha;
double z_offset_;
double offset_end_range;
double offset_start_range;
double min_range;
double offset_reset_max_range;


std::vector<double> raw_range_buffer_;
int range_buffer_size_;
double outlier_thres_;
double range_offset_ = 0.0;

void dyn_callback(pose_fuse::dyn_paramsConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure done");
    filter_alpha  = config.alpha_;
    range_buffer_size_ = config.range_buffer_size_;
    outlier_thres_ = config.outlier_thres_;
    offset_end_range = config.offset_end_range;
    offset_start_range = config.offset_start_range;
    min_range = config.min_range;
    offset_reset_max_range = config.offset_reset_max_range;
    range_offset_ = 0.0;
}

void callback(const geometry_msgs::PoseStampedConstPtr& pose_data, const sensor_msgs::ImuConstPtr& imu_data, const sensor_msgs::RangeConstPtr& range_data)
{
    
    
    
    tf::Quaternion q_imu(imu_data->orientation.x,imu_data->orientation.y,imu_data->orientation.z,imu_data->orientation.w);
    tf::Matrix3x3 m_imu(q_imu);            
    double roll_imu, pitch_imu, yaw_imu;
    m_imu.getRPY(roll_imu, pitch_imu, yaw_imu);     
    
    tf::Quaternion q_ndt(pose_data->pose.orientation.x,pose_data->pose.orientation.y,pose_data->pose.orientation.z,pose_data->pose.orientation.w);
    tf::Matrix3x3 m_ndt(q_ndt);            
    double roll_ndt, pitch_ndt, yaw_ndt;
    m_ndt.getRPY(roll_ndt, pitch_ndt, yaw_ndt);     

    tf2::Quaternion fused_qaut;
    fused_qaut.setRPY(roll_imu, pitch_imu, yaw_ndt);
    fused_qaut=fused_qaut.normalize();
    

    fused_pose_.header = pose_data->header;
    
    fused_pose_.header.frame_id = "world";
    fused_pose_.child_frame_id = "base_link";    

    fused_pose_.transform.translation.x = pose_data->pose.position.x; 
    fused_pose_.transform.translation.y = pose_data->pose.position.y;
    
    double tmp_range = range_data->range*cos(roll_imu)*cos(pitch_imu);
    if (tmp_range > 3.0){
        return;
    }
    
    double distance_to_line = range_data_filtered_ - tmp_range-range_offset_;
        if (abs(distance_to_line) > outlier_thres_ && range_data_filtered_ > offset_start_range){
               range_offset_=range_offset_+ distance_to_line;          
            
               ROS_INFO("distance to line = %f", distance_to_line);
               ROS_INFO("offset re-calaulted, offset  = %f", range_offset_);
            }

    if (tmp_range < offset_end_range){
        range_offset_ = 0.0;
    }
    if (tmp_range > offset_reset_max_range){
        range_offset_ = 0.0;
    }
    tmp_range = tmp_range + range_offset_;
    if (tmp_range < min_range){
        range_data_filtered_ = min_range;
        
    }else{
       range_data_filtered_ = range_data_filtered_ - (filter_alpha * (range_data_filtered_ - tmp_range));        
        
    }
    raw_range_buffer_.push_back(range_data_filtered_);
    fused_pose_.transform.translation.z = range_data_filtered_;
    fused_pose_.transform.rotation.x = fused_qaut.getX();
    fused_pose_.transform.rotation.y = fused_qaut.getY();
    fused_pose_.transform.rotation.z = fused_qaut.getZ();
    fused_pose_.transform.rotation.w = fused_qaut.getW();    
    fused_pose_pub.publish(fused_pose_);

    if (raw_range_buffer_.size() > range_buffer_size_){        
        raw_range_buffer_.erase(raw_range_buffer_.begin());
    }
    
    
}



// void dyn_callback(const dyn_paramsConfig &config, uint32_t level) {  
//             d0 = config.d0;
//             k0 = config.k0;
//             thrust_scale = config.thrust_scale;
//             manual_trj_switch_ = config.manual_trj_switch;
//             target_x=config.target_x;
//             target_y=config.target_y;
//             target_z=config.target_z;
//             ROS_INFO("d0 = %f, k= %f, thrust_scale = %f", d0,k0,thrust_scale);            
// }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_fuse");
   
  ros::NodeHandle nh;
  fused_pose_pub = nh.advertise<geometry_msgs::TransformStamped>("ndt_fused_pose", 10);
  filter_alpha = 0.5;
  message_filters::Subscriber<geometry_msgs::PoseStamped> ndt_pose_sub(nh, "/ndtpso_slam_front/pose", 100);
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/mavros/imu/data", 100);
  message_filters::Subscriber<sensor_msgs::Range> range_sub(nh, "/mavros/distance_sensor/lidarlite_pub", 100);

  typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::Imu, sensor_msgs::Range> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), ndt_pose_sub, imu_sub,range_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    dynamic_reconfigure::Server<pose_fuse::dyn_paramsConfig> server;
    dynamic_reconfigure::Server<pose_fuse::dyn_paramsConfig>::CallbackType f;
    f = boost::bind(&dyn_callback, _1, _2);
    server.setCallback(f);

  ros::spin();

  return 0;
}