/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/PositionCommand.h>

mavros_msgs::State current_state;

mavros_msgs::PositionTarget test_pose;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


void ego_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg){
        test_pose.header.stamp = ros::Time::now();
        test_pose.header.frame_id ='cmd';        
        test_pose.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        test_pose.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        test_pose.position.x = msg->position.x;
        test_pose.position.y = msg->position.y;
        test_pose.position.z = msg->position.z;
        test_pose.velocity.x = msg->velocity.x;
        test_pose.velocity.y = msg->velocity.y;;
        test_pose.velocity.z = msg->velocity.z;;
        test_pose.yaw = msg->yaw;
        test_pose.yaw_rate = msg->yaw_dot;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Publisher position_target_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10);
    
    ros::Subscriber ego_sub = nh.subscribe<quadrotor_msgs::PositionCommand>
            ("/drone_0_planning/pos_cmd", 10, ego_cb);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

   
    



    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    int seq =0; 
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        seq++;
        if (seq > 50000){
            seq = 0;
        }
       
        test_pose.header.seq = seq;
        position_target_pub.publish(test_pose);


        // local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}












def setUp(self):
        super(MavrosOffboardYawrateTest, self).setUp()
        rospy.loginfo("initiate")
        self.att = AttitudeTarget()

        self.att_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.att_thread = Thread(target=self.send_att, args=())
        self.att_thread.daemon = True
        self.att_thread.start()

        # desired yawrate target
        self.des_yawrate = 0.1
        self.yawrate_tol = 0.02
        self.test_attctl()

    def tearDown(self):
        super(MavrosOffboardYawrateTest, self).tearDown()

    #
    # Helper methods
    #
    def send_att(self):
        rate = rospy.Rate(10)  # Hz
        self.att.body_rate = Vector3()
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        self.att.orientation = self.local_position.pose.orientation

        self.att.body_rate.x = 0
        self.att.body_rate.y = 0
        self.att.body_rate.z = self.des_yawrate

        self.att.thrust = 0.59

        self.att.type_mask = 3 # ignore roll and pitch rate

        while not rospy.is_shutdown():
            self.att.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(self.att)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    #
    # Test method
    #
    def test_attctl(self):
        """Test offboard yawrate control"""

        # boundary to cross
        # Stay leveled, go up, and test yawrate
        boundary_x = 5
        boundary_y = 5
        boundary_z = 10


        # make sure the simulation is ready to start the mission
        self.wait_for_topics(10)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   5, -1)

        self.log_topic_vars()
        self.set_arm(True, 3)
        self.set_mode("OFFBOARD", 3)
        rospy.loginfo("run mission")
        rospy.loginfo("attempting to cross boundary | z: {2} , stay within x: {0}  y: {1} \n   and achieve {3} yawrate".
                      format(boundary_x, boundary_y, boundary_z, self.des_yawrate))

	    # does it cross expected boundaries in 'timeout' seconds?
        timeout = 90  # (int) seconds
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        crossed = False
        for i in xrange(timeout * loop_freq):
            if (self.local_position.pose.position.x < boundary_x and
	    	    self.local_position.pose.position.x > -boundary_x and
                    self.local_position.pose.position.y < boundary_y and
		    self.local_position.pose.position.y > -boundary_y and
                    self.local_position.pose.position.z > boundary_z and
		    abs(self.imu_data.angular_velocity.z - self.des_yawrate) < self.yawrate_tol):
                rospy.loginfo("Test successful. Final altitude and yawrate achieved")
                crossed = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(crossed, (
            "took too long to finish test | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} \n " \
	    "                             | current att qx: {3:.2f}, qy: {4:.2f}, qz: {5:.2f} qw: {6:.2f}, yr: {7:.2f}| timeout(seconds): {8}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z,
		   self.imu_data.orientation.x,
		   self.imu_data.orientation.y,
		   self.imu_data.orientation.z,
		   self.imu_data.orientation.w,
		   self.imu_data.angular_velocity.z,
		   timeout)))

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   90, 0)
        self.set_arm(False, 5)