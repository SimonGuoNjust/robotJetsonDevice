#pragma once
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "cuda_utils.h"
#include "logging.h"
#include "model.h"
#include "postprocess.h"
#include "preprocess.h"
#include "utils.h"
using namespace nvinfer1;
class YOLOv8ImageInferencer
{
private:
    int model_bboxes;
    std::string cuda_post_process = "g";
    IRuntime* runtime = nullptr;
    ICudaEngine* engine = nullptr;
    IExecutionContext* context = nullptr;
    cudaStream_t stream;
    float* device_buffers[2];
    float* output_buffer_host = nullptr;
    float* decode_ptr_device = nullptr;
public:
    float* decode_ptr_host = nullptr;
    YOLOv8ImageInferencer(std::string model_path);
    void inferOneImage(cv::Mat& img);
    ~YOLOv8ImageInferencer();
};