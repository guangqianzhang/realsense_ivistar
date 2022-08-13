#include <iostream>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/h/rs_option.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include "realcamera/example.hpp"
#include "realcamera/cv-helpers.hpp"
#include "trt_yolo.hpp"
#include "realcamera/cvfunction.hpp"
#include <chrono>
#include <smartcar_msgs/DetectedObjectArray.h>
#include <smartcar_msgs/ImageObjects.h>

using namespace std;
using namespace cv;
using namespace rs2;
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "realsense");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher object_pub_;
    ros::Publisher Light_pub_;
    ros::Rate loopRate(10);
    object_pub_ = private_nh.advertise<smartcar_msgs::DetectedObjectArray>("/cameraObj", 1);
    Light_pub_ = private_nh.advertise<smartcar_msgs::ImageObjects>("/lightObj", 1);
    ROS_INFO("hello realsense!");

    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe;
    rs2::align align_to(RS2_STREAM_COLOR);
    auto profile = pipe.start(cfg);

    rs2::frameset data;
    auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    rs2_intrinsics depth_intrinsics = stream.get_intrinsics();
    float point[3];

    std::vector<BatchResult> batch_res;
    boost::shared_ptr<RealSense::trt_yolo> trt_detect_(new RealSense::trt_yolo());

    while (ros::ok())
    {
        auto start = std::chrono::system_clock::now();
        double start_time = (double)cv::getTickCount();
        ros::Time time = ros::Time::now();
        data = pipe.wait_for_frames(); // 33ms
        data = align_to.process(data);
        rs2::frame color_data = data.get_color_frame();
        rs2::frame depth_data = data.get_depth_frame();

        auto depth_frame = data.get_depth_frame();
        static int last_frame_number = 0;
        if (color_data.get_frame_number() == last_frame_number)
            continue;
        last_frame_number = static_cast<int>(color_data.get_frame_number());

        // auto color_mat = frame_to_mat(color_frame);//40ms-10
        cv::Mat color_mat(cv::Size(640, 480), CV_8UC3, (void *)color_data.get_data(), cv::Mat::AUTO_STEP); // 38ms-10
        auto end = std::chrono::system_clock::now();
        std::cout << "time:" << std::chrono::duration_cast<chrono::milliseconds>(end - start).count() << endl;
        // cv::Mat depth_mat = (cv::Size(640, 480), CV_16U, (void *)depth_data.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat work_frame;
        color_mat.copyTo(work_frame);
        std::vector<cv::Mat> batch_img;
        batch_img.push_back(work_frame);
        trt_detect_->Run(batch_img, batch_res);
        smartcar_msgs::DetectedObjectArray ObjArray;
        smartcar_msgs::DetectedObject result_Obj;
        smartcar_msgs::ImageObjects Lightobjs;
        smartcar_msgs::ImageObj LightObj;
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        for (int i = 0; i < batch_img.size(); ++i)
        {
            for (const auto &r : batch_res[i])
            {
                // std::cout << "batch " << i << " id:" << r.id << " prob:" << r.prob << " rect:" << r.rect << std::endl;
                cv::rectangle(batch_img[i], r.rect, cv::Scalar(255, 0, 0), 2);

                std::string label = "";
                RealSense::checkID(r.id, label);

                result_Obj.id = r.id;
                result_Obj.label = label;
                result_Obj.score = r.prob;
                int size = (r.rect.width > r.rect.height) ? r.rect.height / 2 : r.rect.width / 2;
                cv::Rect depth_roi = RealSense::rectCenterScale(r.rect, cv::Size(-size, -size));

                cv::rectangle(work_frame, depth_roi, cv::Scalar(255, 0, 255), 1);
                float pixel[2];
                RealSense::getRectCenter(depth_roi, pixel);

                auto udist = depth_frame.get_distance(pixel[0], pixel[1]);
                rs2_deproject_pixel_to_point(point, &depth_intrinsics, pixel, udist);
                result_Obj.pose.position.x = point[0];
                result_Obj.pose.position.y = point[1];
                result_Obj.pose.position.z = point[2];

                if (r.id == RealSense::Signal::traffic_light)
                {

                    LightObj.type = label;
                    // LightObj.obj = depth_roi;
                    LightObj.distanse = udist;
                    LightObj.pose.position.x = point[0];
                    LightObj.pose.position.y = point[1];
                    LightObj.pose.position.z = point[2];
                    cv_ptr->header.stamp = time;
                    cv_ptr->encoding = "bgr8";
                    cv_ptr->image = batch_img[i](depth_roi);

                    LightObj.roi_image = *cv_ptr->toImageMsg();
                    Lightobjs.Objs.push_back(LightObj);
                    Lightobjs.car_num += 1;
                    // Light_pub_.publish(LightObj);
                }
                // cout << "x:" << point[0] << " y:" << point[1] << " z:" << point[2] << endl;
                std::stringstream stream;
                stream << std::fixed << std::setprecision(2) << "id:" << r.id << "x:" << point[0] << " y:" << point[1] << " z:" << point[2];
                cv::putText(batch_img[i], stream.str(), cv::Point(r.rect.x, r.rect.y - 5), 0, 0.5, cv::Scalar(0, 0, 255), 2);
            }//end of batchs in an image
            ObjArray.objects.push_back(result_Obj);
            Light_pub_.publish(LightObj);
            cv::namedWindow("image" + std::to_string(i), cv::WINDOW_NORMAL);
            cv::imshow("image" + std::to_string(i), batch_img[i]);
            cv::waitKey(10);
        }//end of images

        // imshow("color", color_mat);
        // cv::waitKey(10);
        object_pub_.publish(ObjArray);
        double end_time = (double)cv::getTickCount();
        double fps = cv::getTickFrequency() / (end_time - start_time);
        ROS_INFO("FPS in app: %0.3f", fps); //
    }
}