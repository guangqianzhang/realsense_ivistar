#pragma once
#include <chrono>
#include <memory>
#include <thread>
#include "../trt_yolo/modules/class_detector.h"
#include "../trt_yolo/extra/class_timer.hpp"
#include <yaml-cpp/yaml.h>


namespace RealSense
{
    class trt_yolo
    {
    private:
        Config config_v5;
        std::unique_ptr<Detector> detector_;
        std::vector<BatchResult> batch_res;
        Timer timer;

        // std::vector<cv::Mat> batch_img;
        // std::vector<BatchResult> batch_res;

    public:
        trt_yolo(/* args */);
        ~trt_yolo();
        void Run(std::vector<cv::Mat> &img, std::vector<BatchResult> &res)
        {
            detector_->detect(img, res);
        }
    };

    trt_yolo::trt_yolo(/* args */)
    {
                // modelconfig
        std::string config_file_path = "/home/agx/Documents/tensoort/modelconfig.yaml";
        YAML::Node config = YAML::LoadFile(config_file_path);
        std::string medel_path = config["medel_path"].as<std::string>();

        config_v5.net_type = YOLOV5;
        config_v5.detect_thresh = 0.5;
        config_v5.file_model_cfg = medel_path + config["file_model_cfg"].as<std::string>();
        config_v5.file_model_weights = medel_path + config["file_model_weights"].as<std::string>();
        config_v5.calibration_image_list_file_txt = medel_path + config["calibration_image_list_file_txt"].as<std::string>();
        config_v5.inference_precison = FP32;

        detector_.reset(new Detector);
        detector_->init(config_v5);
    }

    trt_yolo::~trt_yolo()
    {
    }
}