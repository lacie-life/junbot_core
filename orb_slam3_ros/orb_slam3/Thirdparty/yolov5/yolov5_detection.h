#ifndef YOLOV5_DETECTION_H
#define YOLOV5_DETECTION_H

#include <opencv2/core/utility.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <cmath>

#include "include/cuda_utils.h"
#include "include/logging.h"
#include "include/utils.h"
#include "include/config.h"
#include "include/model.h"
#include "include/types.h"

using namespace nvinfer1;

class YoLoObjectDetection
{
    public:
        YoLoObjectDetection(const std::string _model_path);
        ~YoLoObjectDetection();
        void detectObject(const cv::Mat& _frame, std::vector<Detection>& objects);
        cv::Rect get_rect(cv::Mat& img, float bbox[4]);
        
    private:
        void doInference(IExecutionContext& context, cudaStream_t& stream, void** gpu_buffers, float* output, int batchsize);
        void prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer, float** gpu_output_buffer, float** cpu_output_buffer);
        void serialize_engine(unsigned int max_batchsize, bool& is_p6, float& gd, float& gw, std::string& wts_name, std::string& engine_name);
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

#endif

