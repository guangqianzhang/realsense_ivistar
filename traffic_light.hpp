#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <vector>
#include "realcamera/cvfunction.hpp"
#include "traffic_light.hpp"
#include <smartcar_msgs/ImageObjects.h>
#include <smartcar_msgs/ImageRect.h>
#include <smartcar_msgs/TrafficLightResult.h>
#include <geometry_msgs/Pose.h>
using namespace cv;
using namespace std;
namespace RealSense
{

    class Traffic_light
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
        int lightcount = 0;
        int Rlightcount;
        int Ylightcount;
        int Glightcount;

        std::string light_topic_out_;
        std::string light_topic_in_;

        ros::Subscriber light_img_sub_;
        ros::Publisher light_img_pub_;
        ros::NodeHandle private_nh;

    private:
        // hsv model
        Scalar RedLow = Scalar(0, 43, 46);
        Scalar RedUp = Scalar(10, 255, 255);
        Scalar RedLow2 = Scalar(156, 43, 46);
        Scalar RedUp2 = Scalar(180, 255, 255);
        Scalar YellowLow = Scalar(26, 43, 46);
        Scalar YellowUp = Scalar(34, 255, 255);
        Scalar GreenLow = Scalar(35, 43, 46);
        Scalar GreenUp = Scalar(77, 255, 255);
        //利用hsv模型提取image中的颜色
        cv::Mat mask_red;
        cv::Mat mask_yellow;
        cv::Mat mask_green;
        cv::Mat workimage;

        cv::Rect Light_roi;
        bool light_point=false;
        bool stop_point=false;

    private:
        void mask(const cv::Mat &image, const cv::Scalar low, const cv::Scalar up, cv::Mat &mask)
        {
            cv::Mat hsv;

            cv::cvtColor(image, hsv, CV_BGR2HSV);
            cv::inRange(hsv, low, up, mask);
        }
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
        void image_cb(const smartcar_msgs::ImageObjectsConstPtr &obj);
 int forptrCount(cv::Mat image, unsigned int count = 0)
        {

            int nl = image.rows;
            int nc = image.cols;
            uchar max, min;
            if (image.isContinuous())
            {
                nc = nc * nl;
                nl = 1;
            }
            for (int j = 0; j < nl; ++j)
            {
                uchar *data = image.ptr<uchar>(j);
                for (int i = 0; i < nc; ++i)
                {
                    if (*data == 255)
                        count++;
                    data++;
                }
            }
            cout << "image count:" << count << endl;
            return count;
        }
    public:
        Traffic_light(ros::NodeHandle &nh);
        ~Traffic_light();
       
        Traffic_Light count_color()
        {
            int count_red = 0;
            int count_green = 0;
            int count_yellow = 0;

            Traffic_Light light;
            //遍历像素点
            count_red = forptrCount(this->mask_red);
            count_yellow = forptrCount(this->mask_yellow);
            count_green = forptrCount(this->mask_green);

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

            if (lightcount > 6) //红绿灯 类型参数更新速度
            {
                cout << "lightcount=4" << endl;
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
        void setMask(cv::Mat image1)
        {
            mask(image1, 150, 180, 43, 255, mask_red);  //红色
            mask(image1, 11, 34, 43, 255, mask_yellow); //黄色
            mask(image1, 35, 100, 43, 255, mask_green); //绿色
        }
        void setMask1(cv::Mat image1)
        {
            cv::Mat mask_r;
            mask(image1, RedLow, RedUp, this->mask_red); //红色
            mask(image1, RedLow2, RedUp2, mask_r);
            this->mask_red = mask_r | mask_red;
            mask(image1, YellowLow, YellowUp, this->mask_yellow); //黄色
            mask(image1, GreenLow, GreenUp, this->mask_green);    //绿色
        }
        void ShowMask(cv::Mat lightimage)
        {
            cv::namedWindow("raw");
            cv::imshow("raw", lightimage);
            cv::namedWindow("red");
            cv::imshow("red", mask_red);
            cv::namedWindow("yellow");
            cv::imshow("yellow", mask_yellow);
            cv::namedWindow("green");
            cv::imshow("green", mask_green);
            waitKey(10);
        }
    };
    void Traffic_light::image_cb(const smartcar_msgs::ImageObjectsConstPtr &obj)
    {
        // ROS_INFO("get light data!");

        auto start = std::chrono::steady_clock::now();
       
        sensor_msgs::Image img = obj->roi_image;
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
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
        workimage = raw_img.clone();                                // no used
        std::vector<smartcar_msgs::ImageObj> Lightobjs = obj->Objs; // light objs
ros::param::get("/trafficlight/stop_point", stop_point) ;
ros::param::get("/trafficlight/light_point", light_point) ;
cout<<"stop_Point:"<<stop_point<<endl;
cout<<"light_point:"<<light_point<<endl;
        for (auto it = Lightobjs.begin(); it != Lightobjs.end(); ++it)
        {

            float distance = it->distanse;
            geometry_msgs::Pose pose = it->pose;
            std::string label = it->type;
            smartcar_msgs::ImageRect rect = it->obj;
            cv::Rect rect_roi = cv::Rect(rect.x, rect.y, rect.width, rect.height);

            std::stringstream stream;
            stream << std::fixed << std::setprecision(2) << "labe:" << label;

                /* 2022/9/22 for stop single */

            if(label.compare("stop_signal") == 0&& stop_point ){
                ROS_INFO("can detect signal !!");
                     cv::rectangle(workimage, rect_roi, cv::Scalar(255, 0, 255), 1);
                    cv::putText(workimage, stream.str(), cv::Point(rect_roi.x, rect_roi.y - 5), 0, 0.5, cv::Scalar(0, 0, 255), 2);
                    // getStopSingle=true;
                    private_nh.setParam("stop_signal", true);
                    private_nh.setParam("stop_distance", distance);
                }

            if (label.compare("traffic_light") == 0)
            {
                // ROS_INFO("Trafficlight");
                cv::putText(raw_img, stream.str(), cv::Point(rect_roi.x, rect_roi.y - 5), 0, 0.5, cv::Scalar(255, 0, 255), 2);
                // cv::rectangle(workimage, rect_roi, cv::Scalar(255, 0, 255), 1);
                getLight_flag_ = true; // get light
                
                cv::Rect l_roi = cv::Rect(rect_roi.x, rect_roi.y,
                                          rect_roi.width, rect_roi.height);
                // cout << "roi_before" << Light_roi << endl;
                // 1 在停止线上 2 Light_roi=0
                if (Light_roi.area() == 0 && l_roi.area() > 350 && light_point)
                    {
                ROS_INFO("can make a roi for traffic light !!");
                        Light_roi = l_roi;}

                cv::rectangle(raw_img, l_roi, cv::Scalar(0, 0, 255), 2);

            } // end if
        }     // end for
        cv::namedWindow("img", cv::WINDOW_NORMAL);
        cv::imshow("img", raw_img);
        cv::waitKey(10);
        //循环检测后 对红绿灯区域roi进行分析
        if (Light_roi.area() != 0)
        {
            lightcount++; // add count
            cv::Mat lightimage = workimage(Light_roi);
            cout<<"lightimage:"<<lightimage.size()<<endl;
            this->setMask1(lightimage);
            this->ShowMask(lightimage);
            cout << "T_Light_before:" << LightType << endl;
            LightType = this->count_color();
            cout << "T_type:" << LightType << endl;
            // light_result->recognition_result = LightType;
        }

        if (!getLight_flag_) // no light
        {
            lightcount--;
        }
        else
        {
            
        }

        // getLight_flag_ = false;
        cout << "lightcount:" << lightcount << endl;
        if (lightcount < 1)
        {
            lightcount = 0;
        }else if (lightcount>10)
        {
            lightcount=10;
        }
        
        cout << "T_type1:" << LightType << endl;
        private_nh.setParam("light_type", LightType);
        // private_nh.setParam("light_distance", light_result->distance);
       

        if (LightType == Traffic_Light::RED)
        {
            cout << "find traffic light= red !" << endl;
        }
        else if (LightType == Traffic_Light::GREEN)
        {
            cout << "find traffic light= green !" << endl;
        }
        else if (LightType == Traffic_Light::YELLOW)
        {
            cout << "find traffic light= yellow !" << endl;
        }
        else if (LightType == Traffic_Light::UNKNOW)
        {
            cout << "find traffic light = unkown !" << endl;
        }

        cout << "roi" << Light_roi << "a:" << Light_roi.area() << endl;
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        // std::cout << "get light elapsed time: " << elapsed_seconds.count() << "s\n";
    }
    Traffic_light::Traffic_light(ros::NodeHandle &nh) : private_nh(nh)
    {
        ROS_INFO("traffic light !");
        light_running_ = true;
        __NODE_NAME__ = ros::this_node::getName();

        private_nh.param<std::string>("light_topic_in", light_topic_in_, "/light_in");
        std::cout << __NODE_NAME__ << ":light_topic_in:" << light_topic_in_.c_str() << std::endl;
        private_nh.param<std::string>("light_topic_out", light_topic_out_, "/light_out");
        std::cout << __NODE_NAME__ << ":light_topic_out_:" << light_topic_out_.c_str() << std::endl;

        private_nh.param<std::string>("obj_type", obj_type, "traffic_light");
        std::cout << __NODE_NAME__ << ":obj_type:" << obj_type.c_str() << std::endl;

        light_img_sub_ = private_nh.subscribe<smartcar_msgs::ImageObjects>(light_topic_in_, 1, &Traffic_light::image_cb, this);
        light_img_pub_ = private_nh.advertise<smartcar_msgs::TrafficLightResult>(light_topic_out_, 1);
LightType=Traffic_Light::NO_LIFHT;
        ros::spin();
    }

    Traffic_light::~Traffic_light()
    {
        ROS_INFO("bye Light !");
    }
}