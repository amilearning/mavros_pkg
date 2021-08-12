// /**
//  * @file offb_node.cpp
//  * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
//  * Stack and tested in Gazebo SITL
//  */
// #include <tf/tf.h>
// #include <tf/transform_datatypes.h>
// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>
// #include <mavros_msgs/PositionTarget.h>
// #include <mavros_msgs/AttitudeTarget.h>
// #include <quadrotor_msgs/PositionCommand.h>
// #include <mav_msgs/RollPitchYawrateThrust.h>
// #include <mavros/setpoint_mixin.h>
// #include <mavros/frame_tf.h>
// #include <functional>
// #include <mavros/utils.h>
// #include <mavros/mavros_plugin.h>
// #include <nav_msgs/Odometry.h>

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <trajectory_msgs/MultiDOFJointTrajectory.h>
// #include <sensor_msgs/LaserScan.h>
// #include <std_srvs/Empty.h>
// #include <std_srvs/Trigger.h>

// mavros_msgs::State current_state;

// mavros_msgs::PositionTarget test_pose;

// nav_msgs::Odometry odom_state; 
// mavros_msgs::AttitudeTarget att;
// trajectory_msgs::MultiDOFJointTrajectory waypoints;



// int waypoints_itr;
// bool send_waypoint;

// void cmdloopCallback(const ros::TimerEvent &event) {
    
//     if (waypoints.points.size() > 0){        
//         if (waypoints_itr >= 0 && waypoints_itr < waypoints.points.size()){            
//             test_pose.header.stamp = ros::Time::now();
//             test_pose.header.frame_id ='c';        


//             test_pose.coordinate_frame = 1; // mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            
//             // test_pose.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
//             // mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
//             test_pose.position.x = waypoints.points[waypoints_itr].transforms[0].translation.x;
//             test_pose.position.y = waypoints.points[waypoints_itr].transforms[0].translation.y;
//             test_pose.position.z = waypoints.points[waypoints_itr].transforms[0].translation.z;               
//             tf::Quaternion q(waypoints.points[waypoints_itr].transforms[0].rotation.x,waypoints.points[waypoints_itr].transforms[0].rotation.y,waypoints.points[waypoints_itr].transforms[0].rotation.z,waypoints.points[waypoints_itr].transforms[0].rotation.w);
//             tf::Matrix3x3 m(q);            
//             double roll, pitch, yaw;
//             m.getRPY(roll, pitch, yaw);        
//             test_pose.yaw = yaw;
//             waypoints_itr++;            
//             send_waypoint = true;
            
//         }       
//     }   
   

// }

// void multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg) {
  
// //   for (int i = 0; i < msg->points.size(); i++)
// //     {
// //     waypoints.points.push_back(msg->points[i]);
// //     }
//   if (msg->points.size() > 1){    
//     waypoints.points = msg->points;    
//     // ROS_INFO("Trjectory length = %d",msg->points.size());
//   waypoints_itr = 0; 
//   }

// }


// void odom_cb(const nav_msgs::OdometryConstPtr& msg){
//     // odom_state.header = msg.header;
//     // odom_state.pose = msg.pose;
//     // odo_state.twist = msg.twist;
//     odom_state.pose = msg->pose;

//     geometry_msgs::TransformStamped transformStamped;
//     static tf2_ros::TransformBroadcaster br;  
//     transformStamped.header.stamp = ros::Time::now();
//     transformStamped.header.frame_id = "map";
//     transformStamped.child_frame_id = "base_link";
//     transformStamped.transform.translation.x = msg->pose.pose.position.x;
//     transformStamped.transform.translation.y = msg->pose.pose.position.y;
//     transformStamped.transform.translation.z = msg->pose.pose.position.z;    
//     transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
//     transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
//     transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
//     transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;
//     br.sendTransform(transformStamped);

    
//     // static tf2_ros::TransformBroadcaster base_to_camera_br;
//     // geometry_msgs::TransformStamped transformStamped_2;    
//     // transformStamped_2.header.stamp = ros::Time::now();
//     // transformStamped_2.header.frame_id = "base_link";
//     // transformStamped_2.child_frame_id = "camera_link";
//     // transformStamped_2.transform.translation.x = 0.1;
//     // transformStamped_2.transform.translation.y = 0;
//     // transformStamped_2.transform.translation.z = 0.035;    
//     // transformStamped_2.transform.rotation.x = 0.5;
//     // transformStamped_2.transform.rotation.y = 0.5;
//     // transformStamped_2.transform.rotation.z = 0.5;
//     // transformStamped_2.transform.rotation.w = -0.5;
//     // base_to_camera_br.sendTransform(transformStamped_2);

    
// }

// // void rpyt_cb(const mav_msgs::RollPitchYawrateThrustConstPtr& msg){
// // //    getX()
// //     // the masks are much more limited than the docs would suggest so we don't use them
// //     uint8_t type_mask = 0;
// //     geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(msg->roll, msg->pitch, 0);
    
// //     Eigen::Quaterniond desired_orientation;      
// //     tf::quaternionMsgToEigen(orientation, desired_orientation);
// //     // auto ned_desired_orientation = mavros::ftf::transform_orientation_enu_ned(
// //     // mavros::ftf::transform_orientation_baselink_aircraft(
// //     // desired_orientation));
// //     // Transform desired orientation to represent aircraft->NED,
// //     // MAVROS operates on orientation of base_link->ENU
// //     // ned_desired_orientation = ned_desired_orientation.normalized();

// //     att.header.stamp = ros::Time::now();
// //     att.header.frame_id="c";
// //     att.type_mask=3;
// //     Eigen::Matrix<double, 4, 1> q_tmp = desired_orientation.coeffs();    
// //     att.orientation.x=q_tmp[0];
// //     att.orientation.y=q_tmp[1];
// //     att.orientation.z=q_tmp[2];
// //     att.orientation.w=q_tmp[3];
// //     att.thrust = msg->thrust.z;
// //     att.body_rate.x = 0;
// //     att.body_rate.y = 0;
// //     att.body_rate.z = msg->yaw_rate;
    
// // }


// void state_cb(const mavros_msgs::State::ConstPtr& msg){
//     current_state = *msg;
// }
// // void ego_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg){
// //         test_pose.header.stamp = ros::Time::now();
// //         test_pose.header.frame_id ='c';        
// //         // test_pose.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
// //         // test_pose.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
// //         test_pose.position.x = msg->position.x;
// //         test_pose.position.y = msg->position.y;
// //         test_pose.position.z = msg->position.z;
// //         // test_pose.velocity.x = msg->velocity.x;
// //         // test_pose.velocity.y = msg->velocity.y;
// //         // test_pose.velocity.z = msg->velocity.z;       
// //         test_pose.yaw = msg->yaw;       
// //         test_pose.yaw_rate = msg->yaw_dot;
// // }

// int main(int argc, char **argv)
// {   ros::init(argc, argv, "offb_node");
//     ros::NodeHandle nh;
    
//     waypoints_itr = 0;
//     ros::Subscriber multiDOFJointSub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("m100/command/trajectory",10,&multiDOFJointCallback);        
//     // ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",10, &odom_cb);
//     ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth_pose",10, &odom_cb);
//     ros::Subscriber vision_odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_node/odometry",1, &vis_odom_cb);
    
//     ros::Timer cmdloop_timer_ = nh.createTimer(ros::Duration(0.05), &cmdloopCallback); 

//     // ros::Subscriber att_thrust_sub = nh.subscribe<mav_msgs::RollPitchYawrateThrust>
//     //         ("/lmpc/roll_pitch_yawrate_thrust", 10, rpyt_cb);
//     ros::Publisher rpyt_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);

//     ros::Publisher position_target_pub = nh.advertise<mavros_msgs::PositionTarget>
//             ("/mavros/setpoint_raw/local", 10);
    
//     // ros::Subscriber ego_sub = nh.subscribe<quadrotor_msgs::PositionCommand>
//     //         ("/drone_0_planning/pos_cmd", 10, ego_cb);

//     ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
//             ("mavros/state", 10, state_cb);
//     ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
//             ("mavros/setpoint_position/local", 10);
//     ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
//             ("mavros/cmd/arming");
//     ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
//             ("mavros/set_mode");

//     //the setpoint publishing rate MUST be faster than 2Hz
//     ros::Rate rate(20.0);
//     ROS_INFO("initiating");
//     // wait for FCU connection
//     while(ros::ok() && !current_state.connected){
//         ros::spinOnce();
//         rate.sleep();
//     }
//     ROS_INFO("connected");
//     geometry_msgs::PoseStamped pose;
//     pose.pose.position.x = 0;
//     pose.pose.position.y = 0;
//     pose.pose.position.z = 2;
//     // send a few setpoints before starting
//     for(int i = 100; ros::ok() && i > 0; --i){
//         local_pos_pub.publish(pose);
//         ros::spinOnce();
//         rate.sleep();
//     }
//     ROS_INFO("pose init test complete");
//     test_pose.header.stamp = ros::Time::now();
//     test_pose.header.frame_id ='c';            
//     test_pose.position.x = 0;
//     test_pose.position.y = 0;
//     test_pose.position.z = 0;    
//     test_pose.yaw = 0.0;
//     test_pose.yaw_rate = 0.0;

//     mavros_msgs::SetMode offb_set_mode;
//     offb_set_mode.request.custom_mode = "OFFBOARD";

//     mavros_msgs::CommandBool arm_cmd;
//     arm_cmd.request.value = true;

//     ros::Time last_request = ros::Time::now();
//     int seq =0; 
//     while(ros::ok()){
//         if( current_state.mode != "OFFBOARD" &&
//             (ros::Time::now() - last_request > ros::Duration(5.0))){
//             if( set_mode_client.call(offb_set_mode) &&
//                 offb_set_mode.response.mode_sent){
//                 ROS_INFO("Offboard enabled");
//             }
//             last_request = ros::Time::now();
//         } else {
//             if( !current_state.armed &&
//                 (ros::Time::now() - last_request > ros::Duration(5.0))){
//                 if( arming_client.call(arm_cmd) &&
//                     arm_cmd.response.success){
//                     ROS_INFO("Vehicle armed");
//                 }
//                 last_request = ros::Time::now();
//             }
//         }

//         seq++;
//         // if (seq > 50000){
//         //     seq = 0;
//         // }
       
//         test_pose.header.seq = seq;               
//         position_target_pub.publish(test_pose);
        
//         send_waypoint = false;
//         // rpyt_pub.publish(att);
//         // local_pos_pub.publish(pose);
//         geometry_msgs::TransformStamped transformStamped;  
//         static tf2_ros::TransformBroadcaster br;
//         transformStamped.header.stamp = ros::Time::now();
//         transformStamped.header.frame_id = "map";
//         transformStamped.child_frame_id = "base_link";
//         transformStamped.transform.translation.x = odom_state.pose.pose.position.x;
//         transformStamped.transform.translation.y = odom_state.pose.pose.position.y;
//         transformStamped.transform.translation.z = odom_state.pose.pose.position.z;    
//         transformStamped.transform.rotation.x = odom_state.pose.pose.orientation.x;
//         transformStamped.transform.rotation.y = odom_state.pose.pose.orientation.y;
//         transformStamped.transform.rotation.z = odom_state.pose.pose.orientation.z;
//         transformStamped.transform.rotation.w = odom_state.pose.pose.orientation.w;
//         br.sendTransform(transformStamped);

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }

