#pragma once
#include "class_timer.hpp"
#include "class_detector.h"

#include <memory>
#include <thread>

class TRT
{
private:
    Config config_v5;
    config_v5.net_type = YOLOV5;
    config_v5.detect_thresh = 0.5;
    config_v5.file_model_cfg = "../configs/yolov5-6.0/yolov5x.cfg";
    config_v5.file_model_weights = "../configs/yolov5-6.0/yolov5x.weights";
    config_v5.calibration_image_list_file_txt = "../configs/calibration_images.txt";
    config_v5.inference_precison = FP32;

public:
    TRT(/* args */);
    ~TRT();
};

TRT::TRT(/* args */)
{
}

TRT::~TRT()
{
}
