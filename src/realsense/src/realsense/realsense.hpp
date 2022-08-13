#pragma once
#include <iomanip>
#include <librealsense2/rs.hpp>
#include "realsense/example-utils.hpp"
#include "realsense/cv-helpers.hpp"
#include "realsense/example.hpp"
#include "../cvfunction.hpp"

#include "../trt_yolo/modules/class_detector.h"
#include "../trt_yolo/extra/class_timer.hpp"

#include <memory>
#include <thread>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <yaml-cpp/yaml.h>
#include <smartcar_msgs/DetectedObjectArray.h>
#include <smartcar_msgs/ImageObj.h>
namespace RealSense
{
    using namespace rs2;
    using namespace cv;
    class realsense
    {
    private:

        // boost::shared_ptr<Yolo> yolo_;
        std::string __NODE_NAME__;
        std::string realsense_colorMat_T_;
        std::string realsense_ImageObj_T_;
        std::string realsense_Light_T;

        ros::Subscriber camera_raw_sub_;
        ros::Publisher vision_detect_img_pub_;
        ros::Publisher vision_detect_msg_pub_;
        ros::Publisher light_detect_msg_pub_;
        ros::NodeHandle private_nh;
        bool detection_flag_ = true;
        bool Stop_Sign = false;
         int stopcount;
         double stop_distance=0.0;
        Config config_v5;
        std::unique_ptr<Detector> detector_;
        std::vector<BatchResult> batch_res;
        Timer timer;

    public:
        realsense(ros::NodeHandle &nh);
        ~realsense();

        int captrue()
        {
            rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
            // Create a simple OpenGL window for rendering:
            window app(1280, 720, "RealSense Capture Example");
            // Declare depth colorizer for pretty visualization of depth data
            rs2::colorizer color_map;
            // Declare rates printer for showing streaming rates of the enabled streams.
            rs2::rates_printer printer;
            rs2::pipeline pipe;

            pipe.start();

            while (app) // Application still alive?
            {
                rs2::frameset data = pipe.wait_for_frames(). // Wait for next set of frames from the camera
                                     apply_filter(printer)
                                         .                    // Print each enabled stream frame rate
                                     apply_filter(color_map); // Find and colorize the depth data

                // The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
                // Each texture is displayed on different viewport according to it's stream unique id
                app.show(data);
            }

            return EXIT_SUCCESS;
        }
        void Run()
        {
            detectByDnn();
        }
        void Runhull()
        {
            // window app(1280, 720, "RealSense Capture Example");
            rs2::colorizer color_map;
            rs2::rates_printer printer;
            rs2::pipeline pipe;
            pipe.start();
            const auto window_name = "Display color_Image";
            namedWindow(window_name, WINDOW_AUTOSIZE);
            while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
            {
                rs2::frameset data = pipe.wait_for_frames();
                // rs2::frameset data=pipe.wait_for_frames().apply_filter(printer).apply_filter(color_map);
                frame frames = data.get_color_frame().apply_filter(color_map); // search for depth date
                depth_frame depth = data.get_depth_frame();

                Mat color_image = frame2Mat(frames);
                detectByHull(color_image, depth, window_name);
            }
        }
        void detectByHull(Mat &color_image, const depth_frame &depth, string window_name)
        {
            Mat start_image = color_image;
            color_image = Morphological_process(color_image);
            cout << color_image.size() << endl;
            Mat color_image2 = get_draw_contours(color_image, start_image, depth, 5000);
            // app.show(depth);
            cout << color_image2.size() << endl;

            imshow(window_name, color_image);
            imshow("Display color_Image2", color_image2);
        }
        void detectByDnn()
        {
            
            pipeline pipe;
            auto config = pipe.start();
            auto profile = config.get_stream(RS2_STREAM_COLOR)
                               .as<video_stream_profile>();
            rs2::align align_to(RS2_STREAM_COLOR);
            const auto window_name = "Display Image";
            namedWindow(window_name, WINDOW_AUTOSIZE);

            while (getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0 && ros::ok())
            {
                timer.reset();
                // Wait for the next set of frames
                auto data = pipe.wait_for_frames();
                double start_time = (double)cv::getTickCount();
                ros::Time time = ros::Time::now();
                // Make sure the frames are spatially aligned

                data = align_to.process(data);

                auto color_frame = data.get_color_frame();
                auto depth_frame = data.get_depth_frame();
                // If we only received new depth frame,
                // but the color did not update, continue
                static int last_frame_number = 0;
                if (color_frame.get_frame_number() == last_frame_number)
                    continue;
                last_frame_number = static_cast<int>(color_frame.get_frame_number());

                // Convert RealSense frame to OpenCV matrix:
                auto color_mat = frame_to_mat(color_frame);
                auto depth_mat = depth_frame_to_meters(depth_frame);
                Rect depth_mat_rect(Point(0, 0), Point(depth_mat.cols, depth_mat.rows));

                cv::Mat work_frame;
                color_mat.copyTo(work_frame);
                timer.out("carmera ");
                cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
                std::vector<cv::Mat> batch_img;
                batch_img.push_back(work_frame);

                // detection
                if (detection_flag_)
                {
                    //用于发送的消息
                    smartcar_msgs::ImageObjPtr ImageObj(new smartcar_msgs::ImageObj);
                    cv_ptr->header.stamp = time;
                    cv_ptr->encoding = "bgr8";
                    cv_ptr->image = color_mat;
                    ImageObj->roi_image = *cv_ptr->toImageMsg();

                    // detect
                    timer.reset();
                    detector_->detect(batch_img, batch_res);
                    timer.out("detect");
                    timer.reset();
                    for (int i = 0; i < batch_img.size(); ++i)
                    {
                        for (auto &r : batch_res[i])
                        {
                            // std::cout << "batch " << i << " id:" << r.id << " prob:" << r.prob << " rect:" << r.rect << std::endl;
                        
                            std::string label = "";
                            checkID(r.id, label);

                            float distance;
                            
                            checkBox(r.rect, depth_mat);
                            cv::rectangle(work_frame, r.rect, cv::Scalar(255, 0, 0), 2);
                            int size = (r.rect.width > r.rect.height) ? r.rect.height / 2 : r.rect.width / 2;
                            cv::Rect depth_roi = rectCenterScale(r.rect, cv::Size(-size, -size));
                            cv::rectangle(work_frame, depth_roi, cv::Scalar(255, 0, 255), 1);
                            auto depth_mat_Roi = depth_mat(depth_roi);
                            Scalar m = mean(depth_mat_Roi);
                            distance = m[0];
            
                                if (r.id == Signal::stop_signal ){
                                stopcount++;
                                }else {stopcount--;}
                                if (stopcount > 6 )
                                {Stop_Sign = true;
                                stop_distance=distance;
                                stopcount=6;
                                }else {Stop_Sign=false;}

                            ImageObj->type.push_back(label);

                                smartcar_msgs::ImageRect ObjRect;
                                ObjRect.x = depth_roi.x;
                                ObjRect.y = depth_roi.y;
                                ObjRect.width = depth_roi.width;
                                ObjRect.height = depth_roi.height;
                                ObjRect.score = r.prob;
                            ImageObj->obj.push_back(ObjRect);
                            ImageObj->distanse.push_back(distance);
                           
                            std::stringstream stream;
                            stream << std::fixed << std::setprecision(2) << "id:" << label << "  dis:" << distance;
                            cv::putText(work_frame, stream.str(), cv::Point(r.rect.x, r.rect.y - 5), 0, 0.5, cv::Scalar(0, 0, 255), 2);
                        }
                    }
                    private_nh.setParam("stop_type",Stop_Sign);
                    if(Stop_Sign)  {  private_nh.setParam("stop_distance",stop_distance);
                    cout<<"stop distance:"<<stop_distance<<endl;}
                     else{ private_nh.setParam("stop_distance",0);}

                    light_detect_msg_pub_.publish(*ImageObj);
                    vision_detect_msg_pub_.publish(*ImageObj);

                    timer.out("publish");
                } // end if detection_flag_
                timer.reset();
                double end_time = (double)cv::getTickCount();
                double fps = cv::getTickFrequency() / (end_time - start_time);
                ROS_INFO("FPS in app: %0.3f", fps); //
                cv::imshow(window_name, work_frame);
                timer.out("imshow");
                // cv::imshow("depth mat", depth_mat);
                if (waitKey(1) >= 0)
                    break;
            }
        }
    };

    realsense::realsense(ros::NodeHandle &nh) : private_nh(nh)
    {
        __NODE_NAME__ = ros::this_node::getName();

        private_nh.param<std::string>("camera_topic", realsense_colorMat_T_, "/realsense/image_raw");
        std::cout << __NODE_NAME__ << ":camera_topic:" << realsense_colorMat_T_.c_str() << std::endl;
        private_nh.param<std::string>("ImageObj_topic", realsense_ImageObj_T_, "/realsense/image_Obj");
        std::cout << __NODE_NAME__ << ":ImageObj_topic:" << realsense_ImageObj_T_.c_str() << std::endl;
        private_nh.param<std::string>("Light_topic", realsense_Light_T, "/light_in");
        std::cout << __NODE_NAME__ << ":Light_topic:" << realsense_Light_T.c_str() << std::endl;

        private_nh.param<bool>("detection_flag", detection_flag_, "true");
        std::cout << __NODE_NAME__ << ":detection_flag:" << detection_flag_ << std::endl;

        vision_detect_img_pub_ = private_nh.advertise<sensor_msgs::Image>(realsense_colorMat_T_, 1);
        vision_detect_msg_pub_ = private_nh.advertise<smartcar_msgs::ImageObj>(realsense_ImageObj_T_, 1);
        light_detect_msg_pub_ = private_nh.advertise<smartcar_msgs::ImageObj>(realsense_Light_T, 1);
        // modelconfig
        std::string config_file_path = "/home/agx/Documents/zgq/ros/catkin_trt/src/realsense/src/trt_yolo/modelconfig.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);
        std::string medel_path = config["medel_path"].as<std::string>();

        config_v5.net_type = YOLOV5;
        config_v5.detect_thresh = 0.5;
        config_v5.file_model_cfg = medel_path + config["file_model_cfg"].as<std::string>();
        config_v5.file_model_weights = medel_path + config["file_model_weights"].as<std::string>();
        config_v5.calibration_image_list_file_txt = medel_path + config["calibration_image_list_file_txt"].as<std::string>();
        config_v5.inference_precison = FP32;

        detector_.reset(new Detector());
        detector_->init(config_v5);
    }

    realsense::~realsense()
    {
        detector_.release();
    }

}