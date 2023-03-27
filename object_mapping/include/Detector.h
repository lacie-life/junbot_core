//
// Created by lacie on 27/03/2023.
//

#ifndef OBJECT_MAPPING_DETECTOR_H
#define OBJECT_MAPPING_DETECTOR_H

#include <opencv2/core/utility.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <cmath>

#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "utils.hpp"
#include "common.hpp"
#include "calibrator.h"

using namespace nvinfer1;

class Detector
{
public:
    Detector(const std::string _model_path);
    ~Detector();
    void detectObject(const cv::Mat& _frame, std::vector<Detection>& objects);
    cv::Rect get_rect(cv::Mat& img, float bbox[4]);

private:
    void doInference(IExecutionContext& context, cudaStream_t& stream, void** gpu_buffers, float* output, int batchsize);
    void prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer, float** gpu_output_buffer, float** cpu_output_buffer);
    ICudaEngine* build_engine(unsigned int maxBatchSize, IBuilder* builder, IBuilderConfig* config, DataType dt, float& gd, float& gw, std::string& wts_name);
    void deserialize_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine, IExecutionContext** context);

private:
    IRuntime* runtime;
    ICudaEngine* engine;
    IExecutionContext* context;
    cudaStream_t stream;
    void* buffers[2];

    // Prepare cpu and gpu buffers
    float* gpu_buffers[2];
    float* cpu_output_buffer = nullptr;

    cv::Mat left_cv_rgb;
};

#endif //OBJECT_MAPPING_DETECTOR_H
