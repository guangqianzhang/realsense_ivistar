#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "realsense.hpp"

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <thread>
namespace Realsense
{
    class realsense_nodelet : public nodelet::Nodelet
    {
    private:
        std::string __NODE_NAME__;
        volatile bool running_;

        boost::shared_ptr<RealSense::realsense> realsense_;
        std::thread *m_dthread;

        std::string camera_topic_out_;
        std::string camera_topic_in_;
        ros::Publisher vision_detect_img_pub_;
        virtual void onInit()
        {
            ROS_INFO("realsense nodelet!!");
            running_ = true;

            ros::NodeHandle private_nh = getPrivateNodeHandle();
            realsense_.reset(new RealSense::realsense(private_nh));
            // realsense_->Run();
            //   std::thread newThread(realsense_->Run(),&realsense_,&RealSense::realsense);
        }
        void image_cb(const sensor_msgs::Image::ConstPtr &img)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_INFO("cv_bridge Copy failed!");
                return;
            }
            cv::Mat raw_img = cv_ptr->image;
            ros::Time start_time = cv_ptr->header.stamp;

            cv::Mat current_frame;

            ros::Time end_time = ros::Time::now();

            double fps = cv::getTickFrequency() / (end_time.nsec - start_time.nsec);
            std::cout << "FPS in nodelet: " << fps << std::endl; //

            cv_ptr->header.stamp = end_time;
            cv_ptr->encoding = "bgr8";
            cv_ptr->image = current_frame;
            vision_detect_img_pub_.publish(cv_ptr->toImageMsg());
        }

    public:
        realsense_nodelet(/* args */);
        ~realsense_nodelet();
    };
    realsense_nodelet::realsense_nodelet(/* args */)
    {
        ROS_INFO("SampleNodeletClass Constructor");
        __NODE_NAME__ = ros::this_node::getName();
    }
    realsense_nodelet::~realsense_nodelet()
    {
        ROS_INFO("SampleNodeletClass Destructor");
    }
    PLUGINLIB_EXPORT_CLASS(Realsense::realsense_nodelet, nodelet::Nodelet)
}