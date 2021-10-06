#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>  
#include <numeric>
#include<algorithm>

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
// /mavros/altitude
#include <mavros_msgs/Altitude.h>

#define PI 3.14159265

using namespace message_filters;
double range_data_filtered_;
double mono_filter_alpha_;
double monotonic_data_filtered_;
geometry_msgs::TransformStamped fused_pose_; 
bool sub_switch;
ros::Publisher fused_pose_pub; 
ros::Publisher vision_pose_pose_pub; 
ros::Publisher filtered_mono_pub;

double filter_alpha;
double z_offset_;
double offset_end_range;
double mono_outlier_thres_;
double offset_start_range;
double min_range;
double offset_reset_max_range;
geometry_msgs::PoseStamped ndt_pose; 
geometry_msgs::PoseStamped mono_pose; 
double roll_imu, pitch_imu, yaw_imu;
double roll_ndt, pitch_ndt, yaw_ndt;

double min_drift_range_buffer ;
double max_drift_range_buffer ;
double diff_drift_range_buffer;
double min_drift_mono_buffer ;
double max_drift_mono_buffer  ;
double diff_drift_mono_buffer;
bool offset_update;

std::vector<double> raw_range_buffer_;
std::vector<double> init_mono_buffer;

std::vector<double> drift_mono_buffer;
std::vector<double> drift_range_buffer;
double drift_range;
double drift_mono;

double mono_sum = 0;
double mono_init_mean = 0;
int range_buffer_size_;
double outlier_thres_;
double range_offset_ = 0.0;
double init_monotonic=0.0;
double escape_outlier_thres_ = 0.2;


void dyn_callback(pose_fuse::dyn_paramsConfig &config, uint32_t level) {
    // ROS_INFO("Reconfigure done");
    filter_alpha  = config.alpha_;
    range_buffer_size_ = config.range_buffer_size_;
    outlier_thres_ = config.outlier_thres_;
    escape_outlier_thres_ = config.escape_outlier_thres_;
    mono_outlier_thres_ = config.mono_outlier_thres_;
    offset_end_range = config.offset_end_range;
    offset_start_range = config.offset_start_range;
    min_range = config.min_range;
    offset_reset_max_range = config.offset_reset_max_range;
    mono_filter_alpha_ = config.mono_filter_alpha_;
    range_offset_ = 0.0;
}

void callback(const geometry_msgs::PoseStampedConstPtr &pose_data, const sensor_msgs::ImuConstPtr& imu_data, const sensor_msgs::RangeConstPtr& range_data, const mavros_msgs::AltitudeConstPtr& altitude_data)
{
    
    if(init_mono_buffer.size() < 20){ 
        init_mono_buffer.push_back(altitude_data->monotonic);       
       return;
    }else if(init_mono_buffer.size() == 20){
        init_mono_buffer.push_back(altitude_data->monotonic);   
         mono_sum = std::accumulate( init_mono_buffer.begin(),  init_mono_buffer.end(), 0.0);
         mono_init_mean = mono_sum/ init_mono_buffer.size();                
    }
        init_monotonic = mono_init_mean-min_range;
            
    
    monotonic_data_filtered_ = monotonic_data_filtered_ - (mono_filter_alpha_ * (monotonic_data_filtered_ - (altitude_data->monotonic-init_monotonic)));        


    ndt_pose = *pose_data;

    tf::Quaternion q_ndt(pose_data->pose.orientation.x,pose_data->pose.orientation.y,pose_data->pose.orientation.z,pose_data->pose.orientation.w);
    tf::Matrix3x3 m_ndt(q_ndt);       
    m_ndt.getRPY(roll_ndt, pitch_ndt, yaw_ndt);     
    

    fused_pose_.header = pose_data->header;    
    fused_pose_.header.frame_id = "world";
    fused_pose_.child_frame_id = "base_link";    
    fused_pose_.transform.translation.x = pose_data->pose.position.x; 
    fused_pose_.transform.translation.y = pose_data->pose.position.y;

    
    tf::Quaternion q_imu(imu_data->orientation.x,imu_data->orientation.y,imu_data->orientation.z,imu_data->orientation.w);
    tf::Matrix3x3 m_imu(q_imu);            
    
    m_imu.getRPY(roll_imu, pitch_imu, yaw_imu);     
    
    // tf::Quaternion q_ndt(pose_data->pose.orientation.x,pose_data->pose.orientation.y,pose_data->pose.orientation.z,pose_data->pose.orientation.w);
    // tf::Matrix3x3 m_ndt(q_ndt);           
    
    // m_ndt.getRPY(roll_ndt, pitch_ndt, yaw_ndt);     

    tf2::Quaternion fused_qaut;
    fused_qaut.setRPY(roll_imu, pitch_imu, yaw_ndt);
    fused_qaut=fused_qaut.normalize();    

    // fused_pose_.header = range_data->header;    
    fused_pose_.header.frame_id = "world";
    fused_pose_.child_frame_id = "base_link";    
    
    
    double tmp_range = range_data->range*cos(roll_imu)*cos(pitch_imu);
    double outlier_thres_tmp = outlier_thres_; //*cos(roll_imu)*cos(pitch_imu);
    if (tmp_range > 3.0){
        return;
    }    
    
 
    // if there is big jump between monotonic and raw sensor measurements
    if( fabs(monotonic_data_filtered_ - range_data_filtered_) > mono_outlier_thres_){    
        
        double distance_to_line; // = range_data_filtered_ - tmp_range-range_offset_;   
        distance_to_line = range_data_filtered_ - tmp_range-range_offset_;  
        for(int i=0; i< raw_range_buffer_.size(); i++){       
            double tmp_distance_to_line = raw_range_buffer_[i]- tmp_range-range_offset_;
            if(fabs(distance_to_line) < fabs(tmp_distance_to_line)){
                distance_to_line = tmp_distance_to_line;
            }// find max diff
        }
        if(distance_to_line < 0 && fabs(distance_to_line) > escape_outlier_thres_){  
            range_offset_ = 0.0;
            ROS_INFO("offset is updated to zero");
        }else if (fabs(distance_to_line) > outlier_thres_tmp && range_data_filtered_ > offset_start_range){                    
                           range_offset_=range_offset_+ distance_to_line;      
                           ROS_INFO(" offset is updated ");
                           ROS_INFO(" range_offset_ = %f ",range_offset_);
                                  raw_range_buffer_.empty();                                    
        }    
    }
    //  double distance_to_line = range_data_filtered_ - tmp_range-range_offset_;        
    //         if (abs(distance_to_line) > outlier_thres_tmp && range_data_filtered_ > offset_start_range){
    //             range_offset_=range_offset_+ distance_to_line;          
          
    //             }

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

//     drift_mono_buffer
// drift_range_buffer
///////////// update buffer for mono and range drift ////////////////////////
    drift_range =  drift_range - (0.1 * (drift_range - tmp_range));
    drift_range_buffer.push_back(drift_range); 
    if (drift_range_buffer.size() > 20){        
        drift_range_buffer.erase(drift_range_buffer.begin());
    }
     min_drift_range_buffer = *min_element(drift_range_buffer.begin(),drift_range_buffer.end());
     max_drift_range_buffer  = *max_element(drift_range_buffer.begin(),drift_range_buffer.end());
     diff_drift_range_buffer = max_drift_range_buffer - min_drift_range_buffer;

     
    ////////////////

    drift_mono =  drift_mono - (0.1 * (drift_mono - (altitude_data->monotonic))); 
    drift_mono_buffer.push_back(drift_mono);
    if (drift_mono_buffer.size() > 20){        
        drift_mono_buffer.erase(drift_mono_buffer.begin());
    }

     min_drift_mono_buffer = *min_element(drift_mono_buffer.begin(),drift_mono_buffer.end());
     max_drift_mono_buffer  = *max_element(drift_mono_buffer.begin(),drift_mono_buffer.end());
     diff_drift_mono_buffer = max_drift_range_buffer - min_drift_range_buffer;

     if (diff_drift_mono_buffer < 0.05 && diff_drift_range_buffer < 0.05){
          double range_drift_sum = std::accumulate( drift_range_buffer.begin(),  drift_range_buffer.end(), 0.0);
         double range_drift_mean = range_drift_sum/ drift_range_buffer.size();  
         
         double mono_drift_sum = std::accumulate( drift_mono_buffer.begin(),  drift_mono_buffer.end(), 0.0);
         double mono_drift_mean = mono_drift_sum/ drift_mono_buffer.size();  
         init_monotonic = mono_drift_mean-range_drift_mean;
         ROS_INFO("monotonic bias recalculated");
     }

///////////////////////////////////////////////////////



    raw_range_buffer_.push_back(range_data_filtered_);
    fused_pose_.transform.translation.z = range_data_filtered_;
    fused_pose_.transform.rotation.x = fused_qaut.getX();
    fused_pose_.transform.rotation.y = fused_qaut.getY();
    fused_pose_.transform.rotation.z = fused_qaut.getZ();
    fused_pose_.transform.rotation.w = fused_qaut.getW();    
    
    ndt_pose.header = fused_pose_.header;
    ndt_pose.pose.position.z = range_data_filtered_;
    ndt_pose.pose.orientation.x = fused_qaut.getX();
    ndt_pose.pose.orientation.y = fused_qaut.getY();
    ndt_pose.pose.orientation.z = fused_qaut.getZ();
    ndt_pose.pose.orientation.w = fused_qaut.getW();
    if (raw_range_buffer_.size() > range_buffer_size_){        
        raw_range_buffer_.erase(raw_range_buffer_.begin());
    }
    fused_pose_pub.publish(fused_pose_);
    vision_pose_pose_pub.publish(ndt_pose);
    mono_pose = ndt_pose;
    mono_pose.pose.position.z = monotonic_data_filtered_;
    filtered_mono_pub.publish(mono_pose);
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


// void fused_cmd_timer(const ros::TimerEvent &event) {   
//      fused_pose_pub.publish(fused_pose_);
// }

// void ndtPoseCallback(const geometry_msgs::PoseStampedConstPtr &pose_data){
//     ndt_pose = *pose_data;

//     tf::Quaternion q_ndt(pose_data->pose.orientation.x,pose_data->pose.orientation.y,pose_data->pose.orientation.z,pose_data->pose.orientation.w);
//     tf::Matrix3x3 m_ndt(q_ndt);       
//     m_ndt.getRPY(roll_ndt, pitch_ndt, yaw_ndt);     

    

//     fused_pose_.header = pose_data->header;    
//     fused_pose_.header.frame_id = "world";
//     fused_pose_.child_frame_id = "base_link";    
//     fused_pose_.transform.translation.x = pose_data->pose.position.x; 
//     fused_pose_.transform.translation.y = pose_data->pose.position.y;
    

// }

// void local_pose_callback(const geometry_msgs::PoseStampedConstPtr &pose_data){
//     // fused_pose_.transform.translation.x = pose_data->pose.position.x*0.05+fused_pose_.transform.translation.x*0.95;
//     // fused_pose_.transform.translation.y = pose_data->pose.position.y*0.05+fused_pose_.transform.translation.y*0.95;
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_fuse");
   
  ros::NodeHandle nh;
  fused_pose_pub = nh.advertise<geometry_msgs::TransformStamped>("ndt_fused_pose", 10);
  vision_pose_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose_tmp", 10);
  filtered_mono_pub = nh.advertise<geometry_msgs::PoseStamped>("/mono/z", 10);
  
  message_filters::Subscriber<geometry_msgs::PoseStamped> ndt_pose_sub(nh, "/ndtpso_slam_front/pose", 100);
  
//   ros::Timer fused_cmd = nh.createTimer(ros::Duration(0.3), fused_cmd_timer); //Critical -> allocate another thread 
//   ros::Subscriber ndt_sub = nh.subscribe("/ndtpso_slam_front/pose", 2, ndtPoseCallback);
//   ros::Subscriber px4_pose_sub = nh.subscribe("/mavros/local_position/pose", 2, local_pose_callback);

  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/mavros/imu/data", 100);
  message_filters::Subscriber<sensor_msgs::Range> range_sub(nh, "/mavros/distance_sensor/lidarlite_pub", 100);
  message_filters::Subscriber<mavros_msgs::Altitude> altitude_sub(nh, "/mavros/altitude", 100);


  typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::Imu, sensor_msgs::Range, mavros_msgs::Altitude> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), ndt_pose_sub, imu_sub,range_sub,altitude_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2,_3, _4));

    dynamic_reconfigure::Server<pose_fuse::dyn_paramsConfig> server;
    dynamic_reconfigure::Server<pose_fuse::dyn_paramsConfig>::CallbackType f;
    f = boost::bind(&dyn_callback, _1, _2);
    server.setCallback(f);

  ros::spin();

  return 0;
}