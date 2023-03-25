#include "yolov5_detection.h"
#include "include/preprocess.h"
#include "include/postprocess.h"

const static int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;
static Logger gLogger;

using namespace nvinfer1;

YoLoObjectDetection::YoLoObjectDetection(const std::string _model_path)
{   
    cudaSetDevice(kGpuId);

    // Init Cuda Engine
    runtime = nullptr;
    engine = nullptr;
    context = nullptr;

    // Deserialize the engine from file
    deserialize_engine(const_cast<std::string &>(_model_path), &runtime, &engine, &context);

    std::cout << "Deserializa engine done \n";

    CUDA_CHECK(cudaStreamCreate(&stream));

    // Init CUDA preprocessing
    cuda_preprocess_init(kMaxInputImageSize);

    // Prepare cpu and gpu buffers
    prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &cpu_output_buffer);
}

YoLoObjectDetection::~YoLoObjectDetection()
{
    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(gpu_buffers[0]));
    CUDA_CHECK(cudaFree(gpu_buffers[1]));

    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();
}

void YoLoObjectDetection::doInference(IExecutionContext& context, cudaStream_t& stream, void** gpu_buffers, float* output, int batchsize) {
  context.enqueue(batchsize, gpu_buffers, stream, nullptr);
  CUDA_CHECK(cudaMemcpyAsync(output, gpu_buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost, stream));
  cudaStreamSynchronize(stream);
}

void YoLoObjectDetection::detectObject(const cv::Mat& _frame, std::vector<Detection>& objects)
{
    cv::Mat img = _frame.clone();

    std::vector<cv::Mat> img_batch;

    // TODO: Can improve infer speed with batch size
    img_batch.push_back(img);

    cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

    // Time check
//     auto start = std::chrono::system_clock::now();
    doInference(*context, stream, (void**)gpu_buffers, cpu_output_buffer, kBatchSize);
//     auto end = std::chrono::system_clock::now();
//     std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

    // NMS
    std::vector<std::vector<Detection>> res_batch;
    batch_nms(res_batch, cpu_output_buffer, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);

    if(res_batch.size() > 0)
    {
//        std::cout << res_batch.size() << std::endl;
        auto& res = res_batch[0];
        std::cout << "[YoLoObjectDetection] " << res.size() << std::endl;
        for(int i = 0; i < res.size(); i++)
        {
            Detection det;
            det = res[i];
            objects.push_back(det);
        }
    }

//    std::cout << "Size: " << objects.size() << std::endl;
}

void YoLoObjectDetection::prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer, float** gpu_output_buffer, float** cpu_output_buffer)
{
    assert(engine->getNbBindings() == 2);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void**)gpu_input_buffer, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)gpu_output_buffer, kBatchSize * kOutputSize * sizeof(float)));

    *cpu_output_buffer = new float[kBatchSize * kOutputSize];
}

void YoLoObjectDetection::serialize_engine(unsigned int max_batchsize, bool& is_p6, float& gd, float& gw, std::string& wts_name, std::string& engine_name) {
    // Create builder
    IBuilder* builder = createInferBuilder(gLogger);
    IBuilderConfig* config = builder->createBuilderConfig();

    // Create model to populate the network, then set the outputs and create an engine
    ICudaEngine *engine = nullptr;
    if (is_p6) {
        engine = build_det_p6_engine(max_batchsize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
    } else {
        engine = build_det_engine(max_batchsize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
    }
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

    std::cout << "Model path: " << engine_name << "\n";

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

cv::Rect YoLoObjectDetection::get_rect(cv::Mat& img, float bbox[4]) {
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


