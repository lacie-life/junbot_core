#ifndef YOLOV5_DETECTION_H
#define YOLOV5_DETECTION_H

#include <opencv2/core/utility.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <cmath>

#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "config.h"
#include "model.h"
#include "types.h"

using namespace nvinfer1;

class YoLoObjectDetection {
public:
    YoLoObjectDetection(const std::string _model_path);

    ~YoLoObjectDetection();

    void detectObject(const cv::Mat &_frame, std::vector<Detection> &objects);

    cv::Rect get_rect(cv::Mat &img, float bbox[4]);

    cv::Mat get_mask(cv::Mat &img, Detection &det);

private:
    void
    infer(IExecutionContext &context, cudaStream_t &stream, void **gpu_buffers, float *output1, float *output2,
                int batchSize);

    void prepare_buffers(ICudaEngine *engine, float **gpu_input_buffer, float **gpu_output_buffer1,
                         float **gpu_output_buffer2, float **cpu_output_buffer1, float **cpu_output_buffer2);

    void serialize_engine(unsigned int max_batchSize, float &gd, float &gw, std::string &wts_name,
                          std::string &engine_name);

    void
    deserialize_engine(std::string &engine_name, IRuntime **runtime, ICudaEngine **engine, IExecutionContext **context);

private:
    IRuntime *runtime;
    ICudaEngine *engine;
    IExecutionContext *context;

    cudaStream_t stream;

    // Prepare cpu and gpu buffers
    float *gpu_buffers[3];
    float *cpu_output_buffer1;
    float *cpu_output_buffer2;

    cv::Mat left_cv_rgb;
};

#endif

