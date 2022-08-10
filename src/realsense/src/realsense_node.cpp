#include <ros/ros.h>


#include <sensor_msgs/Image.h>
#include "realsense/realsense.hpp"
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "realsense");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loopRate(10);

    ROS_INFO("hello realsense!");

    RealSense::realsense capture(private_nh);
    capture.Run();
}