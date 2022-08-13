#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "traffic_light.hpp"
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "trafficlight");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loopRate(10);

    ROS_INFO("hello traffic light!");

    // RealSense::traffic_light light(private_nh);
    RealSense::Traffic_light light(private_nh);
    // RealSense::traffic_light TLight(private_nh);
    return 0;
}