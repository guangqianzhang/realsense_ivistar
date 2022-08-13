#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <vector>
#include "../cvfunction.hpp"
#include "traffic_light.hpp"
#include <smartcar_msgs/ImageObj.h>
#include <smartcar_msgs/ImageRect.h>
#include <smartcar_msgs/TrafficLightResult.h>
using namespace cv;
using namespace std;
namespace Realsense
{

    class traffic_light
    {
        std::string __NODE_NAME__;
        volatile bool light_running_;
        std::string obj_type;
        int imgHeight_;
        int imgWidth_;
        enum Traffic_Light
        {
            NO_LIFHT = -1,
            UNKNOW = 0,
            RED = 1,
            YELLOW = 2,
            GREEN = 3
        };
        Traffic_Light LightType;
        bool getLight_flag_ = false;
        unsigned int lightcount = 0;
        unsigned int Rlightcount;
        unsigned int Ylightcount;
        unsigned int Glightcount;
        // boost::shared_ptr<traffic_light> light_;

        std::string light_topic_out_;
        std::string light_topic_in_;

        ros::Subscriber tracker_img_sub_;
        ros::Publisher tracker_img_pub_;
        ros::NodeHandle private_nh;

    private:
        //利用hsv模型提取image中的颜色
        cv::Mat mask_red;
        cv::Mat mask_yellow;
        cv::Mat mask_green;
        cv::Mat workimage;

    private:
        void mask(const cv::Mat &image, double minH, double maxH, double minS, double maxS, cv::Mat &mask)
        {
            cv::Mat hsv;
            cv::Mat mask1;
            cv::Mat mask2;
            cv::cvtColor(image, hsv, CV_BGR2HSV);
            cv::Mat channels[3];
            cv::split(hsv, channels);
            cv::threshold(channels[0], mask1, maxH, 255, cv::THRESH_BINARY_INV);

            cv::threshold(channels[0], mask2, minH, 255, cv::THRESH_BINARY);
            cv::Mat hueMask;
            if (minH < maxH)
                hueMask = mask1 & mask2;

            cv::threshold(channels[1], mask1, maxS, 255, cv::THRESH_BINARY_INV);
            cv::threshold(channels[1], mask2, minS, 255, cv::THRESH_BINARY);
            cv::Mat satMask;
            satMask = mask1 & mask2;

            mask = hueMask & satMask;
        }
        void image_cb(const smartcar_msgs::ImageObjConstPtr &obj);

    public:
        traffic_light(ros::NodeHandle &nh);
        ~traffic_light();
        Traffic_Light count_color()
        {
            int count_red = 0;
            int count_green = 0;
            int count_yellow = 0;

            Traffic_Light light;
            //遍历像素点
            for (int i = 0; i < mask_red.rows; i++)
            {
                for (int j = 0; j < mask_red.cols; j++)
                {
                    if (mask_red.at<uchar>(i, j) == 255)
                        count_red++;
                }
            }
            for (int i = 0; i < mask_yellow.rows; i++)
            {
                for (int j = 0; j < mask_yellow.cols; j++)
                {
                    if (mask_yellow.at<uchar>(i, j) == 255)
                        count_yellow++;
                }
            }
            for (int i = 0; i < mask_green.rows; i++)
            {
                for (int j = 0; j < mask_green.cols; j++)
                {
                    if (mask_green.at<uchar>(i, j) == 255)
                        count_green++;
                }
            }
            if ((count_red > count_yellow) && (count_red > count_green))
            {
                Rlightcount++;
            }
            else if ((count_yellow > count_red) && (count_yellow > count_green))
            {
                Ylightcount++;
            }
            else if ((count_green > count_yellow) && (count_green > count_red))
            {
                Glightcount++;
            }

            if (lightcount > 4)
            {
                lightcount = 0;
                cout << "R:" << Rlightcount << " Y:" << Ylightcount << " G:" << Glightcount << endl;
                cout << "Rc:" << count_red << " Yc:" << count_yellow << " Gc:" << count_green << endl;

                if ((Rlightcount > Ylightcount) && (Rlightcount > Glightcount))
                {
                    light = Traffic_Light::RED;
                }
                else if ((Ylightcount > Rlightcount) && (Ylightcount > Glightcount))
                {
                    light = Traffic_Light::YELLOW;
                }
                else if ((Glightcount > Ylightcount) && (Glightcount > Rlightcount))
                {
                    light = Traffic_Light::GREEN;
                }
                else
                {
                    light = Traffic_Light::UNKNOW;
                }

                Rlightcount = 0;
                Ylightcount = 0;
                Glightcount = 0;
            }
            else
            {
                light = LightType;
            }

            return light;
        }
        void setMask(const cv::Mat image1)
        {
            mask(image1, 150, 180, 43, 255, mask_red);  //红色
            mask(image1, 11, 34, 43, 255, mask_yellow); //黄色
            mask(image1, 35, 100, 43, 255, mask_green); //绿色
        }
        void ShowMask()
        {
            cv::namedWindow("raw");
            cv::imshow("raw", workimage);
            cv::namedWindow("red");
            cv::imshow("red", mask_red);
            cv::namedWindow("yellow");
            cv::imshow("yellow", mask_yellow);
            cv::namedWindow("green");
            cv::imshow("green", mask_green);
            waitKey(10);
        }
    };
    void traffic_light::image_cb(const smartcar_msgs::ImageObjConstPtr &obj)
    {
        ROS_INFO("get data!");

        auto start = std::chrono::steady_clock::now();
        sensor_msgs::Image img = obj->roi_image;
        std::vector<smartcar_msgs::ImageRect> imgRectArray = obj->obj;
        std::vector<std::string> imgLabelArray = obj->type;
        std::vector<float> ingDistanceArray = obj->distanse;

        std::vector<cv::Rect> imgcvRectArray;
        for (auto it = imgRectArray.begin(); it != imgRectArray.end(); ++it)
        {
            cv::Rect rect;
            rect.x = it->x;
            rect.y = it->y;
            rect.width = it->width;
            rect.height = it->height;
            imgcvRectArray.push_back(rect);
        }
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
        cv::Mat raw_img = (cv_ptr->image);

        smartcar_msgs::TrafficLightResultPtr light_result;
        if (imgcvRectArray.size())
        {

            for (size_t i = 0; i < imgRectArray.size(); i++)
            {
                if (imgLabelArray[i].compare(obj_type) == 0)
                // if (1)
                {
                    lightcount++;
                    getLight_flag_ = true;
                    light_result.reset(new smartcar_msgs::TrafficLightResult);
                    light_result->distance = ingDistanceArray[i];
                    workimage = raw_img(imgcvRectArray[i]);
                    imgHeight_ = workimage.rows;
                    imgWidth_ = workimage.cols;
                    // light_.reset(new traffic_light);
                    this->setMask(workimage);
                    // this->ShowMask();
                    LightType = this->count_color();

                    light_result->recognition_result = LightType;

                    private_nh.setParam("light_type", LightType);
                    private_nh.setParam("light_distance", ingDistanceArray[i]);
                    cout << "light_type:" << LightType << endl;
                    cout << "light_distance:" << ingDistanceArray[i] << endl;

                    auto end = std::chrono::steady_clock::now();
                    std::chrono::duration<double> elapsed_seconds = end - start;
                    std::cout << "get light elapsed time: " << elapsed_seconds.count() << "s\n";
                }//end of if
            }//end of for
            if (!getLight_flag_)
            {
                lightcount--;
            }
            getLight_flag_=false;
            std::cout << "lightcount:" << lightcount << std::endl;
            // dont detect light;
            if (lightcount < 1)
            {
                private_nh.setParam("light_type", Traffic_Light::NO_LIFHT);
            }
        }
    }
    traffic_light::traffic_light(ros::NodeHandle &nh) : private_nh(nh)
    {
        ROS_INFO("traffic light !");
        light_running_ = true;

        private_nh.param<std::string>("light_topic_in", light_topic_in_, "/light_in");
        private_nh.param<std::string>("light_topic_out", light_topic_out_, "/light_out");
        private_nh.param<std::string>("obj_type", obj_type, "traffic light");

        tracker_img_sub_ = private_nh.subscribe<smartcar_msgs::ImageObj>(light_topic_in_, 1, &traffic_light::image_cb, this);
        tracker_img_pub_ = private_nh.advertise<smartcar_msgs::TrafficLightResult>(light_topic_out_, 1);

        ros::spin();
    }

    traffic_light::~traffic_light()
    {
        ROS_INFO("bye Light !");
    }
}