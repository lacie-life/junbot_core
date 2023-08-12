#include "config.h"
#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include "model.h"

#include <iostream>
#include <chrono>
#include <cmath>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>

using namespace nvinfer1;

static Logger gLogger;
const static int kOutputSize1 = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;
const static int kOutputSize2 = 32 * (kInputH / 4) * (kInputW / 4);

void prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer, float** gpu_output_buffer1, float** gpu_output_buffer2, float** cpu_output_buffer1, float** cpu_output_buffer2) {
    assert(engine->getNbBindings() == 3);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex1 = engine->getBindingIndex(kOutputTensorName);
    const int outputIndex2 = engine->getBindingIndex("proto");
    assert(inputIndex == 0);
    assert(outputIndex1 == 1);
    assert(outputIndex2 == 2);

    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void**)gpu_input_buffer, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)gpu_output_buffer1, kBatchSize * kOutputSize1 * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)gpu_output_buffer2, kBatchSize * kOutputSize2 * sizeof(float)));

    // Alloc CPU buffers
    *cpu_output_buffer1 = new float[kBatchSize * kOutputSize1];
    *cpu_output_buffer2 = new float[kBatchSize * kOutputSize2];
}

void infer(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* output1, float* output2, int batchSize) {
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output1, buffers[1], batchSize * kOutputSize1 * sizeof(float), cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaMemcpyAsync(output2, buffers[2], batchSize * kOutputSize2 * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

void deserialize_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine, IExecutionContext** context) {
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        assert(false);
    }
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    char* serialized_engine = new char[size];
    assert(serialized_engine);
    file.read(serialized_engine, size);
    file.close();

    *runtime = createInferRuntime(gLogger);
    assert(*runtime);
    *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
    assert(*engine);
    *context = (*engine)->createExecutionContext();
    assert(*context);
    delete[] serialized_engine;
}

int main(int argc, char** argv) try{
    cudaSetDevice(kGpuId);

    ros::init(argc, argv, "segmentation_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    // TensorRT preparation
    std::string engine_name = "/home/lacie/workspace/catkin_ws/src/JunBot/junbot_tools/junbot_camera/Model/yolov5s_seg.engine";

    // Deserialize the engine from file
    IRuntime* runtime = nullptr;
    ICudaEngine* engine = nullptr;
    IExecutionContext* context = nullptr;

    cudaStream_t stream;

    deserialize_engine(engine_name, &runtime, &engine, &context);

    CUDA_CHECK(cudaStreamCreate(&stream));

    // Init CUDA preprocessing
    cuda_preprocess_init(kMaxInputImageSize);

    // Prepare cpu and gpu buffers
    float* gpu_buffers[3];
    float* cpu_output_buffer1 = nullptr;
    float* cpu_output_buffer2 = nullptr;

    prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &gpu_buffers[2], &cpu_output_buffer1, &cpu_output_buffer2);

    // Realsense preparation
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    // batch predict
    while(ros::ok()) {
        // Get a batch of images
        std::vector<cv::Mat> img_batch;
        std::vector<std::string> img_name_batch;

        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame rgb = data.get_color_frame().apply_filter(color_map);

        // Query frame size (width and height)
        const int w = rgb.as<rs2::video_frame>().get_width();
        const int h = rgb.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)rgb.get_data(), cv::Mat::AUTO_STEP);

        img_batch.push_back(image);
        img_name_batch.push_back("test");

        std::cout << "batch size: " << img_batch.size() << std::endl;

        // Preprocess
        cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

        // Run inference
        auto start = std::chrono::system_clock::now();
        infer(*context, stream, (void**)gpu_buffers, cpu_output_buffer1, cpu_output_buffer2, kBatchSize);
        auto end = std::chrono::system_clock::now();
        std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

        // NMS
        std::vector<std::vector<Detection>> res_batch;
        batch_nms(res_batch, cpu_output_buffer1, img_batch.size(), kOutputSize1, kConfThresh, kNmsThresh);

        // Draw result and save image
        for (size_t b = 0; b < img_name_batch.size(); b++) {
            auto& res = res_batch[b];
            auto masks = process_mask(&cpu_output_buffer2[b * kOutputSize2], kOutputSize2, res);
            draw_mask_bbox(image, res, masks);
        }

        cv::imshow("Result", image);

        if(cv::waitKey(1) == 27)
        {
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(gpu_buffers[0]));
    CUDA_CHECK(cudaFree(gpu_buffers[1]));
    CUDA_CHECK(cudaFree(gpu_buffers[2]));
    delete[] cpu_output_buffer1;
    delete[] cpu_output_buffer2;
    cuda_preprocess_destroy();
    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
