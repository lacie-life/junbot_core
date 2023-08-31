#include "config.h"
#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include "model.h"
#include <string>
#include <iostream>
#include <chrono>
#include <cmath>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <mutex>

using namespace nvinfer1;
static Logger gLogger;
const static int kOutputSize1 = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;
const static int kOutputSize2 = 32 * (kInputH / 4) * (kInputW / 4);

static std::vector<uint32_t> colors = {0xFF3838, 0xFF9D97, 0xFF701F, 0xFFB21D, 0xCFD231, 0x48F90A,
                                       0x92CC17, 0x3DDB86, 0x1A9334, 0x00D4BB, 0x2C99A8, 0x00C2FF,
                                       0x344593, 0x6473FF, 0x0018EC, 0x8438FF, 0x520085, 0xCB38FF,
                                       0xFF95C8, 0xFF37C7};

class D455_camera
{
public:
    rs2::colorizer color_map;
    rs2::pipeline pipelines;
    rs2::pipeline_profile selections;
    std::string serials;
    rs2::stream_profile depth_stream;
    rs2::stream_profile color_stream;
    rs2_extrinsics depth_to_color_extrin;
    rs2_extrinsics color_to_depth_extrin;

    // Apply extrinsics to the origin
    float origin[3]{0.f, 0.f, 0.f};
    float target[3];
    float depth_scale;
    rs2::video_stream_profile depth_stream_;
    rs2::video_stream_profile color_stream_;
    rs2_intrinsics depth_intrin;
    rs2_intrinsics color_intrin;
};

void prepare_buffers(ICudaEngine *engine, float **gpu_input_buffer, float **gpu_output_buffer1, float **gpu_output_buffer2, float **cpu_output_buffer1, float **cpu_output_buffer2)
{
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
    CUDA_CHECK(cudaMalloc((void **)gpu_input_buffer, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void **)gpu_output_buffer1, kBatchSize * kOutputSize1 * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void **)gpu_output_buffer2, kBatchSize * kOutputSize2 * sizeof(float)));

    // Alloc CPU buffers
    *cpu_output_buffer1 = new float[kBatchSize * kOutputSize1];
    *cpu_output_buffer2 = new float[kBatchSize * kOutputSize2];
}

void serialize_engine(unsigned int max_batchsize, float &gd, float &gw, std::string &wts_name, std::string &engine_name)
{
    // Create builder
    IBuilder *builder = createInferBuilder(gLogger);
    IBuilderConfig *config = builder->createBuilderConfig();

    // Create model to populate the network, then set the outputs and create an engine
    ICudaEngine *engine = nullptr;

    engine = build_seg_engine(max_batchsize, builder, config, DataType::kFLOAT, gd, gw, wts_name);

    assert(engine != nullptr);

    // Serialize the engine
    IHostMemory *serialized_engine = engine->serialize();
    assert(serialized_engine != nullptr);

    // Save engine to file
    std::ofstream p(engine_name, std::ios::binary);
    if (!p)
    {
        std::cerr << "Could not open plan output file" << std::endl;
        assert(false);
    }
    p.write(reinterpret_cast<const char *>(serialized_engine->data()), serialized_engine->size());

    // Close everything down
    engine->destroy();
    builder->destroy();
    config->destroy();
    serialized_engine->destroy();
}

void infer(IExecutionContext &context, cudaStream_t &stream, void **buffers, float *output1, float *output2, int batchSize)
{
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output1, buffers[1], batchSize * kOutputSize1 * sizeof(float), cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaMemcpyAsync(output2, buffers[2], batchSize * kOutputSize2 * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

void deserialize_engine(std::string &engine_name, IRuntime **runtime, ICudaEngine **engine, IExecutionContext **context)
{
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good())
    {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        assert(false);
    }
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    char *serialized_engine = new char[size];
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

int main(int argc, char **argv)
try
{
    cudaSetDevice(kGpuId);

    std::mutex m_mutex;

    ros::init(argc, argv, "segmentation_node");
    ros::NodeHandle n;
    ros::Publisher obstacle_pub = n.advertise<std_msgs::String>("/object_detected", 1000);
    ros::Publisher d455_state = n.advertise<std_msgs::String>("/d455_state", 1000);
    ros::Rate loop_rate(100);

    // TensorRT preparation
    std::string engine_name = "/home/junbot/junbot_ws/src/JunBot/junbot_tools/junbot_camera/Model/yolov5s_seg.engine";

    // Deserialize the engine from file
    IRuntime *runtime = nullptr;
    ICudaEngine *engine = nullptr;
    IExecutionContext *context = nullptr;

    cudaStream_t stream;

    deserialize_engine(engine_name, &runtime, &engine, &context);

    CUDA_CHECK(cudaStreamCreate(&stream));

    // Init CUDA preprocessing
    cuda_preprocess_init(kMaxInputImageSize);

    // Prepare cpu and gpu buffers
    float *gpu_buffers[3];
    float *cpu_output_buffer1 = nullptr;
    float *cpu_output_buffer2 = nullptr;

    prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &gpu_buffers[2], &cpu_output_buffer1, &cpu_output_buffer2);

    // Realsense preparation
    rs2::context ctx;
    std::vector<D455_camera> D455;
    D455.resize(2);

    // Capture serial numbers before opening streaming
    D455.at(0).serials = "117122250794";
    D455.at(1).serials = "117122250006";
    // Start a streaming pipe per each connected device
    for (int i = 0; i < D455.size(); i++)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        rs2::pipeline_profile selection;

        cfg.enable_device(D455.at(i).serials);

        selection = pipe.start(cfg);
        D455.at(i).pipelines = pipe;
        D455.at(i).selections = selection;

        // Map from each device's serial number to a different colorizer
        D455.at(i).color_map = rs2::colorizer();
    }
    // Get the depth stream's unique ID
    for (int i = 0; i < D455.size(); i++)
    {
        D455.at(i).depth_stream = D455.at(i).selections.get_stream(RS2_STREAM_DEPTH);
        D455.at(i).color_stream = D455.at(i).selections.get_stream(RS2_STREAM_COLOR);
        D455.at(i).depth_to_color_extrin = D455.at(i).depth_stream.get_extrinsics_to(D455.at(i).color_stream);
        D455.at(i).color_to_depth_extrin = D455.at(i).color_stream.get_extrinsics_to(D455.at(i).depth_stream);
        rs2_transform_point_to_point(D455.at(i).target, &D455.at(i).depth_to_color_extrin, D455.at(i).origin);
        rs2_transform_point_to_point(D455.at(i).target, &D455.at(i).color_to_depth_extrin, D455.at(i).origin);
        rs2::depth_sensor sensor = D455.at(i).selections.get_device().first<rs2::depth_sensor>();
        ;
        D455.at(i).depth_scale = sensor.get_depth_scale();
        D455.at(i).depth_stream_ = D455.at(i).selections.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        D455.at(i).depth_intrin = D455.at(i).depth_stream_.get_intrinsics();
        D455.at(i).color_stream_ = D455.at(i).selections.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        D455.at(i).color_intrin = D455.at(i).color_stream_.get_intrinsics();
    }
    double depth_min = 0.11; // meter
    double depth_max = 5.0;  // meter

    // batch predict
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        // Get a batch of images
        std::vector<cv::Mat> img_results;

        // Collect the new frames from all the connected devices
        std::vector<rs2::frameset> new_frames;

        int _count = 0;

        for (int i = 0; i < D455.size(); i++)
        {
            rs2::frameset fs;
            if (D455.at(i).pipelines.poll_for_frames(&fs))
            {
                new_frames.push_back(fs);
                _count++;
            }
        }

        ss << "{\"d455_state \": " << _count << "}";
        msg.data = ss.str();
        d455_state.publish(msg);

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        for (int i = 0; i < new_frames.size(); i++)
        {
            std::vector<cv::Mat> img_batch;
            std::vector<std::string> img_name_batch;

            rs2::frameset data = new_frames.at(i);

            auto serial = rs2::sensor_from_frame(data.get_color_frame())->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

            int check_serial = 0;

            if (serial == D455.at(0).serials)
                check_serial = 0;
            else if (serial == D455.at(1).serials)
                check_serial = 1;

            rs2::frame rgb = data.get_color_frame().apply_filter(D455.at(check_serial).color_map);
            rs2::frame depth = data.get_depth_frame().apply_filter(D455.at(check_serial).color_map);
            rs2::depth_frame df = data.get_depth_frame();

            // Query frame size (width and height)
            const int w = rgb.as<rs2::video_frame>().get_width();
            const int h = rgb.as<rs2::video_frame>().get_height();

            const int w_depth = depth.as<rs2::video_frame>().get_width();
            const int h_depth = depth.as<rs2::video_frame>().get_height();
            // Create OpenCV matrix of size (w,h) from the colorized depth data
            cv::Mat image(cv::Size(w, h), CV_8UC3, (void *)rgb.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat frame_depth(cv::Size(w_depth, h_depth), CV_8UC3, (void *)depth.get_data(), cv::Mat::AUTO_STEP);

            // Update the window with new data
            cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
            img_batch.push_back(image);
            img_name_batch.push_back(serial);

            // std::cout << "batch size: " << img_batch.size() << std::endl;

            // Preprocess
            cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

            // Run inference
            auto start = std::chrono::system_clock::now();
            infer(*context, stream, (void **)gpu_buffers, cpu_output_buffer1, cpu_output_buffer2, kBatchSize);
            auto end = std::chrono::system_clock::now();

            // std::cout << img_name_batch[0] << " inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

            // NMS
            std::vector<std::vector<Detection>> res_batch;
            batch_nms(res_batch, cpu_output_buffer1, img_batch.size(), kOutputSize1, kConfThresh, kNmsThresh);

            // Draw result and save image
            if (res_batch.size() > 0)
            {
                try
                {
                    auto &res = res_batch[0];

                    auto masks = process_mask(&cpu_output_buffer2[0 * kOutputSize2], kOutputSize2, res);

                    std::vector<cv::Mat> scaled_masks = get_mask(image, res, masks);

                    // if(masks.size() > 0)
                    // {
                    //     draw_mask_bbox(image, res, masks);

                    // }
                    //     cv::imshow(serial, image);
                    for (size_t i = 0; i < res.size(); i++)
                    {
                        cv::Mat img_mask = scale_mask(scaled_masks[i], image);
                        auto color = colors[(int)res[i].class_id % colors.size()];
                        auto bgr = cv::Scalar(color & 0xFF, color >> 8 & 0xFF, color >> 16 & 0xFF);

                        cv::Rect r = get_rect(image, res[i].bbox);

                        float distance_medium = 0;
                        float distance_min = 999;
                        float x_temp = r.x + r.width;
                        float y_temp = r.y + r.height;
                        if (x_temp >= w)
                        {
                            x_temp = w - 1;
                        }
                        if (y_temp >= h)
                        {
                            y_temp = h - 1;
                        }
                        int count = 0;
                        for (int x = r.x; x < x_temp; x=x+5)
                        {
                            for (int y = r.y; y < y_temp; y=y+5)
                            {
                                // if(!r.contains(cv::Point(x, y))) continue;
                                
                                float val = img_mask.at<float>(y, x);
                                if (val <= 0.5)
                                    continue;
                                float pixel_depth[2];
                                float pixel_color[2] = {(float)x, (float)y};
                                // std::cout << "Star \n";
                                rs2_project_color_pixel_to_depth_pixel(
                                    pixel_depth, reinterpret_cast<const uint16_t *>(df.get_data()), D455.at(check_serial).depth_scale,
                                    depth_min, depth_max,
                                    &D455.at(check_serial).depth_intrin,
                                    &D455.at(check_serial).color_intrin,
                                    &D455.at(check_serial).depth_to_color_extrin,
                                    &D455.at(check_serial).color_to_depth_extrin, pixel_color);

                                // std::cout << "End \n";
                                if (df.get_distance(pixel_depth[0], pixel_depth[1]) < 0.1 || df.get_distance(pixel_depth[0], pixel_depth[1]) > 2)
                                {
                                    continue;
                                }

                                distance_medium = distance_medium + df.get_distance(pixel_depth[0], pixel_depth[1]);
                                
                                // std::cout << df.get_distance(pixel_depth[0], pixel_depth[1]) << std::endl;
                                count++;
                                if (df.get_distance(pixel_depth[0], pixel_depth[1]) < distance_min)
                                {
                                    distance_min = df.get_distance(pixel_depth[0], pixel_depth[1]);
                                }
                            }
                        }
                        if (count != 0){
                            distance_medium = distance_medium / count;
                            // std::cout << distance_medium << ":   " << count << std::endl;
                            if (distance_medium < 100)
                            {
                                std::string tmp = "{";
                                tmp += "\"id\":" + std::to_string(res[i].class_id) + ",";
                                tmp += "\"distance\":" + std::to_string(distance_medium);
                                tmp += "}";
                                std::cout << tmp << std::endl;
                                msg.data = tmp;
                                obstacle_pub.publish(msg);
                            }
                        }
                        
                    }
                }
                catch (const std::exception &e)
                {
                    std::cerr << e.what() << std::endl;
                }
            }
        }

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        // std::cout<<"Time: " << time_used.count() << "seconds" << std::endl;

        if (cv::waitKey(1) == 27)
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
catch (const rs2::error &e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
