#include "yolov5_detection.h"
#include "preprocess.h"
#include "postprocess.h"

using namespace nvinfer1;

static Logger gLogger;

const static int kOutputSize1 = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;
const static int kOutputSize2 = 32 * (kInputH / 4) * (kInputW / 4);

YoLoObjectDetection::YoLoObjectDetection(const std::string _model_path) {

    cudaSetDevice(kGpuId);

    runtime = nullptr;
    engine = nullptr;
    context = nullptr;

    cpu_output_buffer1 = nullptr;
    cpu_output_buffer2 = nullptr;

    // Deserialize the engine from file
    deserialize_engine(const_cast<std::string &>(_model_path), &runtime, &engine, &context);

    std::cout << "Deserialize engine done \n";

    CUDA_CHECK(cudaStreamCreate(&stream));

    std::cout << "Create stream done \n";

    // Init CUDA preprocessing
    cuda_preprocess_init(kMaxInputImageSize);

    // Prepare cpu and gpu buffers
    prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1],
                    &gpu_buffers[2], &cpu_output_buffer1,
                    &cpu_output_buffer2);
}

YoLoObjectDetection::~YoLoObjectDetection() {
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
}

void YoLoObjectDetection::infer(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* output1, float* output2, int batchSize) {
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output1, buffers[1], batchSize * kOutputSize1 * sizeof(float), cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaMemcpyAsync(output2, buffers[2], batchSize * kOutputSize2 * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

void YoLoObjectDetection::detectObject(const cv::Mat &_frame, std::vector<Detection> &objects) {
    cv::Mat img = _frame.clone();

    std::vector<cv::Mat> img_batch;
    std::vector<std::string> image_name_batch;

    // TODO: Can improve infer speed with batch size
    img_batch.push_back(img);
    image_name_batch.push_back("0");

    std::cout << "Batch size: " << img_batch.size() << std::endl;

    cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

    // Time check
    auto start = std::chrono::system_clock::now();

    infer(*context, stream, (void **) gpu_buffers, cpu_output_buffer1, cpu_output_buffer2, kBatchSize);

    auto end = std::chrono::system_clock::now();
    std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

    // NMS
    std::vector<std::vector<Detection>> res_batch;
    batch_nms(res_batch, cpu_output_buffer1, img_batch.size(), kOutputSize1, kConfThresh, kNmsThresh);

    std::cout << "Batch size: " << res_batch.size() << std::endl;

    for(size_t b = 0; b < image_name_batch.size(); b++)
    {
        auto& res = res_batch[b];
        cv::Mat img = img_batch[b];

        std::cout << "[YoLoObjectDetection] " << res.size() << std::endl;

        for (int i = 0; i < res.size(); i++) {
            Detection det;
            det = res[i];
            objects.push_back(det);
        }
    }
    std::cout << "Size: " << objects.size() << std::endl;
}

void YoLoObjectDetection::prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer, float** gpu_output_buffer1, float** gpu_output_buffer2, float** cpu_output_buffer1, float** cpu_output_buffer2) {
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

void YoLoObjectDetection::serialize_engine(unsigned int max_batchsize, float& gd, float& gw, std::string& wts_name, std::string& engine_name) {
    // Create builder
    IBuilder* builder = createInferBuilder(gLogger);
    IBuilderConfig* config = builder->createBuilderConfig();

    // Create model to populate the network, then set the outputs and create an engine
    ICudaEngine *engine = nullptr;

    engine = build_seg_engine(max_batchsize, builder, config, DataType::kFLOAT, gd, gw, wts_name);

    assert(engine != nullptr);

    // Serialize the engine
    IHostMemory* serialized_engine = engine->serialize();
    assert(serialized_engine != nullptr);

    // Save engine to file
    std::ofstream p(engine_name, std::ios::binary);
    if (!p) {
        std::cerr << "Could not open plan output file" << std::endl;
        assert(false);
    }
    p.write(reinterpret_cast<const char*>(serialized_engine->data()), serialized_engine->size());

    // Close everything down
    engine->destroy();
    builder->destroy();
    config->destroy();
    serialized_engine->destroy();
}

void YoLoObjectDetection::deserialize_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine, IExecutionContext** context) {
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

cv::Rect YoLoObjectDetection::get_rect(cv::Mat &img, float bbox[4]) {
    float l, r, t, b;
    float r_w = kInputW / (img.cols * 1.0);
    float r_h = kInputH / (img.rows * 1.0);
    if (r_h > r_w) {
        l = bbox[0] - bbox[2] / 2.f;
        r = bbox[0] + bbox[2] / 2.f;
        t = bbox[1] - bbox[3] / 2.f - (kInputH - r_w * img.rows) / 2;
        b = bbox[1] + bbox[3] / 2.f - (kInputH - r_w * img.rows) / 2;
        l = l / r_w;
        r = r / r_w;
        t = t / r_w;
        b = b / r_w;
    } else {
        l = bbox[0] - bbox[2] / 2.f - (kInputW - r_h * img.cols) / 2;
        r = bbox[0] + bbox[2] / 2.f - (kInputW - r_h * img.cols) / 2;
        t = bbox[1] - bbox[3] / 2.f;
        b = bbox[1] + bbox[3] / 2.f;
        l = l / r_h;
        r = r / r_h;
        t = t / r_h;
        b = b / r_h;
    }
    return cv::Rect(round(l), round(t), round(r - l), round(b - t));
}

cv::Mat YoLoObjectDetection::get_mask(cv::Mat &img, Detection &det) {
    cv::Mat mask = getObjectMask(&cpu_output_buffer2[0 * kOutputSize2], kOutputSize2, det);

    cv::Mat img_mask = scale_mask(mask, img);

    return img_mask;
}


