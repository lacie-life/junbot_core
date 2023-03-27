#include <iostream>
#include <chrono>
#include <cmath>

#include "GLViewer.hpp"
#include "Detector.h"

#include <sl/Camera.hpp>



bool parse_args(int argc, char** argv, std::string& wts, std::string& engine, bool& is_p6, float& gd, float& gw) {
    if (argc < 4) return false;
    if (std::string(argv[1]) == "-s" && (argc == 5 || argc == 7)) {
        wts = std::string(argv[2]);
        engine = std::string(argv[3]);
        auto net = std::string(argv[4]);
        if (net[0] == 'n') {
            gd = 0.33;
            gw = 0.25;
        } else if (net[0] == 's') {
            gd = 0.33;
            gw = 0.50;
        } else if (net[0] == 'm') {
            gd = 0.67;
            gw = 0.75;
        } else if (net[0] == 'l') {
            gd = 1.0;
            gw = 1.0;
        } else if (net[0] == 'x') {
            gd = 1.33;
            gw = 1.25;
        } else if (net[0] == 'c' && argc == 7) {
            gd = atof(argv[5]);
            gw = atof(argv[6]);
        } else {
            return false;
        }
        if (net.size() == 2 && net[1] == '6') {
            is_p6 = true;
        }
    } else if (std::string(argv[1]) == "-d") {
        engine = std::string(argv[2]);
    } else {
        return false;
    }
    return true;
}

void print(std::string msg_prefix, sl::ERROR_CODE err_code, std::string msg_suffix) {
    std::cout << "[Sample] ";
    if (err_code != sl::ERROR_CODE::SUCCESS)
        std::cout << "[Error] ";
    std::cout << msg_prefix << " ";
    if (err_code != sl::ERROR_CODE::SUCCESS) {
        std::cout << " | " << toString(err_code) << " : ";
        std::cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        std::cout << " " << msg_suffix;
    std::cout << std::endl;
}

std::vector<sl::uint2> cvt(const cv::Rect &bbox_in){
    std::vector<sl::uint2> bbox_out(4);
    bbox_out[0] = sl::uint2(bbox_in.x, bbox_in.y);
    bbox_out[1] = sl::uint2(bbox_in.x + bbox_in.width, bbox_in.y);
    bbox_out[2] = sl::uint2(bbox_in.x + bbox_in.width, bbox_in.y + bbox_in.height);
    bbox_out[3] = sl::uint2(bbox_in.x, bbox_in.y + bbox_in.height);
    return bbox_out;
}

int main(int argc, char** argv) {

    std::string wts_name = "";
    std::string engine_name = "";
    bool is_p6 = false;
    float gd = 0.0f, gw = 0.0f;

    if (!parse_args(argc, argv, wts_name, engine_name, is_p6, gd, gw)) {
        std::cerr << "arguments not right!" << std::endl;
        std::cerr << "./object_mapping -s [.wts] [.engine] [n/s/m/l/x/n6/s6/m6/l6/x6 or c/c6 gd gw]  // serialize model to plan file" << std::endl;
        std::cerr << "./object_mapping -d [.engine] [zed camera id / optional svo filepath]  // deserialize plan file and run inference" << std::endl;
        return -1;
    }

    // create a model using the API directly and serialize it to a stream
    if (!wts_name.empty()) {
        cudaSetDevice(DEVICE);
        IHostMemory * modelStream{ nullptr};
        APIToModel(BATCH_SIZE, &modelStream, is_p6, gd, gw, wts_name);
        assert(modelStream != nullptr);
        std::ofstream p(engine_name, std::ios::binary);
        if (!p) {
            std::cerr << "could not open plan output file" << std::endl;
            return -1;
        }
        p.write(reinterpret_cast<const char*> (modelStream->data()), modelStream->size());
        modelStream->destroy();
        return 0;
    }

    /// Opening the ZED camera before the model deserialization to avoid cuda context issue
    sl::Camera zed;
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD1080;
    init_parameters.sdk_verbose = true;
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed

    if (argc > 3) {
        std::string zed_opt = argv[3];
        if (zed_opt.find(".svo") != std::string::npos)
            init_parameters.input.setFromSVOFile(zed_opt.c_str());
    }
    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        print("Camera Open", returned_state, "Exit program.");
        return EXIT_FAILURE;
    }
    zed.enablePositionalTracking();
    // Custom OD
    sl::ObjectDetectionParameters detection_parameters;
    detection_parameters.enable_tracking = true;
    detection_parameters.enable_mask_output = false; // designed to give person pixel mask
    detection_parameters.detection_model = sl::DETECTION_MODEL::CUSTOM_BOX_OBJECTS;
    returned_state = zed.enableObjectDetection(detection_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        print("enableObjectDetection", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }
    auto camera_config = zed.getCameraInformation().camera_configuration;
    sl::Resolution pc_resolution(std::min((int) camera_config.resolution.width, 720), std::min((int) camera_config.resolution.height, 404));
    auto camera_info = zed.getCameraInformation(pc_resolution).camera_configuration;
    // Create OpenGL Viewer
    GLViewer viewer;
    viewer.init(argc, argv, camera_info.calibration_parameters.left_cam, true);
    // ---------

    // deserialize the .engine and run inference
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        return -1;
    }
    char *trtModelStream = nullptr;
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    trtModelStream = new char[size];
    assert(trtModelStream);
    file.read(trtModelStream, size);
    file.close();

    // prepare input data ---------------------------
    static float data[BATCH_SIZE * 3 * INPUT_H * INPUT_W];
    static float prob[BATCH_SIZE * OUTPUT_SIZE];
    IRuntime* runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    ICudaEngine* engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    IExecutionContext* context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;
    assert(engine->getNbBindings() == 2);
    void* buffers[2];
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
    const int outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc(&buffers[inputIndex], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof (float)));
    CUDA_CHECK(cudaMalloc(&buffers[outputIndex], BATCH_SIZE * OUTPUT_SIZE * sizeof (float)));
    // Create stream
    cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));

    assert(BATCH_SIZE == 1); // This sample only support batch 1 for now

    sl::Mat left_sl, point_cloud;
    cv::Mat left_cv_rgb;
    sl::ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
    sl::Objects objects;
    sl::Pose cam_w_pose;
    cam_w_pose.pose_data.setIdentity();

    while (viewer.isAvailable()) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {

            zed.retrieveImage(left_sl, sl::VIEW::LEFT);

            // Preparing inference
            cv::Mat left_cv_rgba = slMat2cvMat(left_sl);
            cv::cvtColor(left_cv_rgba, left_cv_rgb, cv::COLOR_BGRA2BGR);
            if (left_cv_rgb.empty()) continue;
            cv::Mat pr_img = preprocess_img(left_cv_rgb, INPUT_W, INPUT_H); // letterbox BGR to RGB
            int i = 0;
            int batch = 0;
            for (int row = 0; row < INPUT_H; ++row) {
                uchar* uc_pixel = pr_img.data + row * pr_img.step;
                for (int col = 0; col < INPUT_W; ++col) {
                    data[batch * 3 * INPUT_H * INPUT_W + i] = (float) uc_pixel[2] / 255.0;
                    data[batch * 3 * INPUT_H * INPUT_W + i + INPUT_H * INPUT_W] = (float) uc_pixel[1] / 255.0;
                    data[batch * 3 * INPUT_H * INPUT_W + i + 2 * INPUT_H * INPUT_W] = (float) uc_pixel[0] / 255.0;
                    uc_pixel += 3;
                    ++i;
                }
            }

            // Running inference
            doInference(*context, stream, buffers, data, prob, BATCH_SIZE);
            std::vector<std::vector < Yolo::Detection >> batch_res(BATCH_SIZE);
            auto& res = batch_res[batch];
            nms(res, &prob[batch * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);

            // Preparing for ZED SDK ingesting
            std::vector<sl::CustomBoxObjectData> objects_in;
            for (auto &it : res) {
                sl::CustomBoxObjectData tmp;
                cv::Rect r = get_rect(left_cv_rgb, it.bbox);
                // Fill the detections into the correct format
                tmp.unique_object_id = sl::generate_unique_id();
                tmp.probability = it.conf;
                tmp.label = (int) it.class_id;
                tmp.bounding_box_2d = cvt(r);
                tmp.is_grounded = ((int) it.class_id == 0); // Only the first class (person) is grounded, that is moving on the floor plane
                // others are tracked in full 3D space                
                objects_in.push_back(tmp);
            }
            // Send the custom detected boxes to the ZED
            zed.ingestCustomBoxObjects(objects_in);


            // Displaying 'raw' objects
            for (size_t j = 0; j < res.size(); j++) {
                cv::Rect r = get_rect(left_cv_rgb, res[j].bbox);
                cv::rectangle(left_cv_rgb, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                cv::putText(left_cv_rgb, std::to_string((int) res[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            }
            cv::imshow("Objects", left_cv_rgb);
            cv::waitKey(10);

            // Retrieve the tracked objects, with 2D and 3D attributes
            zed.retrieveObjects(objects, objectTracker_parameters_rt);
            // GL Viewer
            zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA, sl::MEM::GPU, pc_resolution);
            zed.getPosition(cam_w_pose, sl::REFERENCE_FRAME::WORLD);
            viewer.updateData(point_cloud, objects.object_list, cam_w_pose.pose_data);
        }
    }

    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(buffers[inputIndex]));
    CUDA_CHECK(cudaFree(buffers[outputIndex]));
    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();
    viewer.exit();

    return 0;
}
