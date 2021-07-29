#include "offb_node.h"
#include <ros/ros.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "offboard_ctrl");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ROS_INFO("sdlksamds");
    OffboardFSM OffboardFSM(nh,nh_private);

    ros::spin();

    return 0;
}