#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <vector>
#include "realcamera/cvfunction.hpp"

#include <smartcar_msgs/ImageObjects.h>
#include <smartcar_msgs/ImageRect.h>
namespace RealSense
{
    using namespace cv;
    using namespace std;
    class StartFlag
    {
    private:
        Scalar RedLow = Scalar(0, 43, 46);
        Scalar RedUp = Scalar(10, 255, 255);
        Scalar RedLow2 = Scalar(156, 43, 46);
        Scalar RedUp2 = Scalar(180, 255, 255);
        Scalar YellowLow = Scalar(26, 43, 46);
        Scalar YellowUp = Scalar(34, 255, 255);
        Scalar GreenLow = Scalar(35, 43, 46);
        Scalar GreenUp = Scalar(77, 255, 255);

        cv::Mat raw_img;
        cv::Mat greenMask;
        cv::Mat workimage;
        // unsigned int greenCount;
        int TrigSize = 1000;    //像素点差阈值
        int flage_A_count = 20; //读到旗帜后的计数

        bool readFlage = false;       //读到旗帜
         int staticCount = 0; //静态图像素点计数
        unsigned int averge_greenpoint = 0;

        enum Start_Flag
        {
            NONE = 0,
            APPEAR = 1,
            DISAPPEAR = 2
        } flag_state;
        
        enum Start_state
        {
            GO = 1,
            STYE = 0
        } start_state;

    private:
        std::string __NODE_NAME__;

        std::string flag_topic_in_;
        float xlimit, zlimit;
        int average_count;
        ros::Subscriber flag_img_sub_;
        ros::NodeHandle private_nh;
        bool process_flag = false;
        unsigned int process_count = 0; //空跑计数
    public:
        StartFlag(ros::NodeHandle &nh) : private_nh(nh)
        {
            ROS_INFO("start flag !");

            __NODE_NAME__ = ros::this_node::getName();
flag_state = RealSense::StartFlag::Start_Flag::NONE;
            private_nh.param<std::string>("flag_topic_in", flag_topic_in_, "/realsense_img");
            std::cout << __NODE_NAME__ << ":flag_topic_in:" << flag_topic_in_.c_str() << std::endl;
            private_nh.param<float>("xlimit", xlimit, 1.8);
            std::cout << __NODE_NAME__ << ":xlimit:" << xlimit << std::endl;
            private_nh.param<float>("zlimit", zlimit, 4);
            std::cout << __NODE_NAME__ << ":zlimit:" << zlimit << std::endl;
            private_nh.param<int>("average_count", average_count, 100);
            std::cout << __NODE_NAME__ << ":average_count:" << average_count << std::endl;
            private_nh.param<int>("TrigSize", TrigSize, 1000);
            std::cout << __NODE_NAME__ << ":TrigSize:" << TrigSize << std::endl;
            flag_img_sub_ = private_nh.subscribe<smartcar_msgs::ImageObjects>(flag_topic_in_, 1, &StartFlag::flag_cb, this);
            ros::spin();
        }
        void flag_cb(const smartcar_msgs::ImageObjectsConstPtr &obj)
        {
            // ROS_INFO("get flag data!");

            auto start = std::chrono::steady_clock::now();
            process_count++;
            if (process_count > 3) //启动后曝光，不做像素处理
                process_flag = true;
            int num = obj->car_num;
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

            for (auto it = Lightobjs.begin(); it != Lightobjs.end(); ++it)
            {
                float distance = it->distanse;
                geometry_msgs::Pose pose = it->pose;
                std::string label = it->type;
                smartcar_msgs::ImageRect rect = it->obj;
                cv::Rect rect_roi = cv::Rect(rect.x, rect.y, rect.width, rect.height);

                // cv::rectangle(workimage, rect_roi, cv::Scalar(255, 0, 255), 1);
                std::stringstream stream;
                stream << std::fixed << std::setprecision(2) << "labe:" << label;
                cv::putText(workimage, stream.str(), cv::Point(rect_roi.x, rect_roi.y - 5), 0, 0.5, cv::Scalar(0, 0, 255), 2);
                if (label.compare("person") == 0 && abs(pose.position.x) < xlimit && pose.position.z < zlimit)
                {
                    cout << "lable:" << label << endl;
                    cout << "position:" << pose.position << endl;
                    cv::rectangle(workimage, rect_roi, cv::Scalar(255, 0, 255), 1);

                    cv::Rect person_roi = cv::Rect(raw_img.cols / 4, raw_img.rows / 5,
                                                   raw_img.cols * 3 / 5, raw_img.rows * 2 / 3);
                    cout << "person_roi=" << person_roi << endl;
                    cv::rectangle(workimage, person_roi, cv::Scalar(0, 0, 0), 2);
                    cv::Mat image = workimage(person_roi);
                    if (process_flag) // has_started_state
                    {
                        this->getStaticCount(image); //计算静态图像下绿色像素
                        if (this->averge_greenpoint)
                        {
                            this->getFlagState(image); //检测比较绿色像素点
                        }
                            this->getCmd();
                    }
                }
            }
            cv::namedWindow("Fimg", cv::WINDOW_NORMAL);
            cv::imshow("Fimg", workimage);
            cv::waitKey(10);
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            // std::cout << "get flag elapsed time: " << elapsed_seconds.count() << "s\n";
        }
        void forptrCount(cv::Mat image, unsigned int &count)
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
            cout << "point:" << count << endl;
        }

        void tongjiHSV(cv::Mat src)
        {
            cv::Mat channels[3];
            cv::split(src, channels);
            double minVal;
            double maxVal;
            Point minLoc;
            Point maxLoc;
            minMaxLoc(channels[0], &minVal, &maxVal, &minLoc, &maxLoc, Mat());
            printf("channels0 min: %.2f, max: %.2f \n", minVal, maxVal);
            minMaxLoc(channels[1], &minVal, &maxVal, &minLoc, &maxLoc, Mat());
            printf("channels1 min: %.2f, max: %.2f \n", minVal, maxVal);
            minMaxLoc(channels[2], &minVal, &maxVal, &minLoc, &maxLoc, Mat());
            printf("channels2 min: %.2f, max: %.2f \n", minVal, maxVal);

            // 彩色图像 三通道的 均值与方差
            Mat means, stddev;
            meanStdDev(src, means, stddev);
            printf("H channel->> mean: %.2f, stddev: %.2f\n", means.at<double>(0, 0), stddev.at<double>(0, 0));
            printf("S channel->> mean: %.2f, stddev: %.2f\n", means.at<double>(1, 0), stddev.at<double>(1, 0));
            printf("V channel->> mean: %.2f, stddev: %.2f\n", means.at<double>(2, 0), stddev.at<double>(2, 0));
        }

        void mask(const cv::Mat &image, cv::Mat &mask)
        {
            cv::Mat hsv;

            cv::cvtColor(image, hsv, CV_BGR2HSV);
            cv::inRange(hsv, GreenLow, GreenUp, mask);
        }
        void showmask(cv::Mat img)
        {
            cv::namedWindow("raw");
            cv::imshow("raw", img);
            cv::namedWindow("greenMask");
            cv::imshow("greenMask", greenMask);
            waitKey(10);
        }
        void findcontours(cv::Mat Singimg, int size = 300)
        {
            vector<vector<Point>> contours;
            vector<Vec4i> hierarchy;
            Mat image;
            image = Singimg;
            findContours(image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
            cout << contours.size() << endl;
            Mat imageContours = Mat::zeros(image.size(), CV_8UC1);
            for (size_t i = 0; i < contours.size(); i++)
            {
                if (contourArea(contours[i]) > size)
                    drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
                cout << contourArea(contours[i]) << endl;
            }
            imshow("imageContours", imageContours);
        }

    public:
        std::vector<int> greencounts;
        /* //获取静态绿色像素点100帧平均值 out:averge_greenpoint*/
        void getStaticCount(cv::Mat img)
        {
            cv::Mat mask;
            this->mask(img, mask);
            unsigned int static_greencount = 0;
            int sum = 0;
            this->forptrCount(mask, static_greencount);
            greencounts.push_back(static_greencount);
            staticCount++;
            if (staticCount == average_count) //平均次数
            {
                for (auto it = greencounts.begin(); it != greencounts.end(); ++it)
                {
                    sum += *it;
                }
                averge_greenpoint = sum / greencounts.size();
                std::cout << "sum=" << sum << std::endl;
                std::cout << "greencounts.size=" << greencounts.size() << std::endl;
                std::cout << "static green averge=" << averge_greenpoint << std::endl;
            }
            std::cout << "staticCount:" << staticCount << std::endl;
            cv::namedWindow("Fstatic", cv::WINDOW_NORMAL);
            cv::imshow("Fstatic", mask);
            cv::waitKey(10);
        }
        /* 获取启动绿旗状态 挥动5次 out:flag_state*/
        void getFlagState(cv::Mat img)
        {
            this->mask(img, greenMask);
            unsigned int count = 0;
            forptrCount(greenMask, count);                 //计数 8000左右
            std::cout << "static green averge=" << averge_greenpoint << std::endl;
            std::cout << "count=" << count << std::endl;
            if ((int)abs(count-averge_greenpoint ) > TrigSize) //旗帜出现
            {
                flage_A_count++;
                if (flage_A_count > 10) // 20帧时间
                {
                    flage_A_count = 10;
                    flag_state = RealSense::StartFlag::Start_Flag::APPEAR;
                    readFlage = true;
                }
            }
            else //旗帜消失
            {
                if (readFlage) //前已经检测到旗帜
                {
                    flage_A_count--;
                }
                else
                {
                    flag_state = RealSense::StartFlag::Start_Flag::NONE;
                }
                if (flage_A_count < 0)
                    flage_A_count = 0;
                if (flage_A_count < 5)
                {
                    flag_state = RealSense::StartFlag::Start_Flag::DISAPPEAR;
                    readFlage = false;
                }
            }
            std::cout << "start state:" << flag_state << std::endl;
            // greenCount = count;
        }
        /* 获得启动命令 */
        void getCmd()
        {
            switch (flag_state)
            {
            case RealSense::StartFlag::Start_Flag::NONE: //旗帜未出现
                start_state = RealSense::StartFlag::Start_state::STYE;
                break;
            case RealSense::StartFlag::Start_Flag::APPEAR: //旗帜出现
                start_state = RealSense::StartFlag::Start_state::GO;
                break;
            case RealSense::StartFlag::Start_Flag::DISAPPEAR: //出现后旗帜消失
                start_state = RealSense::StartFlag::Start_state::GO;
                break;
            default:
                break;
            }
            private_nh.setParam("start_state", start_state);
            cout << "flag_state:" << flag_state << endl;
            cout << "start_state:" << start_state << endl;
        }
    };
}