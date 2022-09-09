#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "redFlag.hpp"
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "startflag");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loopRate(10);

    ROS_INFO("hello start flag!");


    RealSense::StartFlag startFalg(private_nh);
 
    return 0;
}