#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "light/traffic_light.hpp"
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "trafficlight");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loopRate(10);

    ROS_INFO("hello traffic light!");

    // RealSense::traffic_light light(private_nh);
    Realsense::traffic_light light(private_nh);
    return 0;
}