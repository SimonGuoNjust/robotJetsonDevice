
#include <yolov8_det.h>

Logger gLogger;
using namespace nvinfer1;
const int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;

void serialize_engine(std::string& wts_name, std::string& engine_name, int& is_p, std::string& sub_type, float& gd,
                      float& gw, int& max_channels) {
    IBuilder* builder = createInferBuilder(gLogger);
    IBuilderConfig* config = builder->createBuilderConfig();
    IHostMemory* serialized_engine = nullptr;

    if (is_p == 6) {
        serialized_engine = buildEngineYolov8DetP6(builder, config, DataType::kFLOAT, wts_name, gd, gw, max_channels);
    } else if (is_p == 2) {
        serialized_engine = buildEngineYolov8DetP2(builder, config, DataType::kFLOAT, wts_name, gd, gw, max_channels);
    } else {
        serialized_engine = buildEngineYolov8Det(builder, config, DataType::kFLOAT, wts_name, gd, gw, max_channels);
    }

    assert(serialized_engine);
    std::ofstream p(engine_name, std::ios::binary);
    if (!p) {
        std::cout << "could not open plan output file" << std::endl;
        assert(false);
    }
    p.write(reinterpret_cast<const char*>(serialized_engine->data()), serialized_engine->size());

    delete serialized_engine;
    delete config;
    delete builder;
}

void deserialize_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine,
                        IExecutionContext** context) {
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

void prepare_buffer(ICudaEngine* engine, float** input_buffer_device, float** output_buffer_device,
                    float** output_buffer_host, float** decode_ptr_host, float** decode_ptr_device,
                    std::string cuda_post_process) {
    assert(engine->getNbBindings() == 2);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void**)input_buffer_device, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)output_buffer_device, kBatchSize * kOutputSize * sizeof(float)));
    if (cuda_post_process == "c") {
        *output_buffer_host = new float[kBatchSize * kOutputSize];
    } else if (cuda_post_process == "g") {
        if (kBatchSize > 1) {
            std::cerr << "Do not yet support GPU post processing for multiple batches" << std::endl;
            exit(0);
        }
        // Allocate memory for decode_ptr_host and copy to device
        *decode_ptr_host = new float[1 + kMaxNumOutputBbox * bbox_element];
        CUDA_CHECK(cudaMalloc((void**)decode_ptr_device, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element)));
    }
}

void infer(IExecutionContext& context, cudaStream_t& stream, void** buffers, float* output, int batchsize,
           float* decode_ptr_host, float* decode_ptr_device, int model_bboxes, std::string cuda_post_process) {
    // infer on the batch asynchronously, and DMA output back to host
    auto start = std::chrono::system_clock::now();
    context.enqueue(batchsize, buffers, stream, nullptr);
    if (cuda_post_process == "c") {
        CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost,
                                   stream));
        auto end = std::chrono::system_clock::now();
        std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                  << "ms" << std::endl;
    } else if (cuda_post_process == "g") {
        CUDA_CHECK(
                cudaMemsetAsync(decode_ptr_device, 0, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), stream));
        cuda_decode((float*)buffers[1], model_bboxes, kConfThresh, decode_ptr_device, kMaxNumOutputBbox, stream);
        cuda_nms(decode_ptr_device, kNmsThresh, kMaxNumOutputBbox, stream);  //cuda nms
        CUDA_CHECK(cudaMemcpyAsync(decode_ptr_host, decode_ptr_device,
                                   sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), cudaMemcpyDeviceToHost,
                                   stream));
        auto end = std::chrono::system_clock::now();
        std::cout << "inference and gpu postprocess time: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    }

    CUDA_CHECK(cudaStreamSynchronize(stream));
}

bool parse_args(int argc, char** argv, std::string& wts, std::string& engine, int& is_p, std::string& img_dir,
                std::string& sub_type, std::string& cuda_post_process, float& gd, float& gw, int& max_channels) {
    if (argc < 4)
        return false;
    if (std::string(argv[1]) == "-s" && (argc == 5 || argc == 7)) {
        wts = std::string(argv[2]);
        engine = std::string(argv[3]);
        auto sub_type = std::string(argv[4]);

        if (sub_type[0] == 'n') {
            gd = 0.33;
            gw = 0.25;
            max_channels = 1024;
        } else if (sub_type[0] == 's') {
            gd = 0.33;
            gw = 0.50;
            max_channels = 1024;
        } else if (sub_type[0] == 'm') {
            gd = 0.67;
            gw = 0.75;
            max_channels = 576;
        } else if (sub_type[0] == 'l') {
            gd = 1.0;
            gw = 1.0;
            max_channels = 512;
        } else if (sub_type[0] == 'x') {
            gd = 1.0;
            gw = 1.25;
            max_channels = 640;
        } else {
            return false;
        }
        if (sub_type.size() == 2 && sub_type[1] == '6') {
            is_p = 6;
        } else if (sub_type.size() == 2 && sub_type[1] == '2') {
            is_p = 2;
        }
    } else if (std::string(argv[1]) == "-d" && argc == 5) {
        engine = std::string(argv[2]);
        img_dir = std::string(argv[3]);
        cuda_post_process = std::string(argv[4]);
    } else {
        return false;
    }
    return true;
}


YOLOv8ImageInferencer::YOLOv8ImageInferencer(std::string model_path)
    {
        deserialize_engine(model_path, &runtime, &engine, &context);
        CUDA_CHECK(cudaStreamCreate(&stream));
        cuda_preprocess_init(kMaxInputImageSize);
        auto out_dims = engine->getBindingDimensions(1);
        model_bboxes = out_dims.d[0];
        // Prepare cpu and gpu buffers
        prepare_buffer(engine, &device_buffers[0], &device_buffers[1], &output_buffer_host, &decode_ptr_host,
                   &decode_ptr_device, cuda_post_process);
    }

void YOLOv8ImageInferencer::inferOneImage(cv::Mat& img)
    {   
        // std::vector<Detection> res;
        int dst_size = kInputW * kInputH * 3;
        cuda_preprocess(img.ptr(), img.cols, img.rows, &device_buffers[0][0], kInputW,
                        kInputH, stream);
        CUDA_CHECK(cudaStreamSynchronize(stream));

        infer(*context, stream, (void**)device_buffers, output_buffer_host, kBatchSize, decode_ptr_host,
              decode_ptr_device, model_bboxes, cuda_post_process);
        // int count = static_cast<int>(*decode_ptr_host);
        // process_decode_ptr_host(res, &decode_ptr_host[0], bbox_element, img, count);
        // std::cout << res.size() << std::endl;
    }

YOLOv8ImageInferencer::~YOLOv8ImageInferencer()
    {
        cudaStreamDestroy(stream);
        CUDA_CHECK(cudaFree(device_buffers[0]));
        CUDA_CHECK(cudaFree(device_buffers[1]));
        CUDA_CHECK(cudaFree(decode_ptr_device));
        delete[] decode_ptr_host;
        delete[] output_buffer_host;
        cuda_preprocess_destroy();
        // Destroy the engine
        delete context;
        delete engine;
        delete runtime;
    }



// int main(int argc, char** argv) {
//     cudaSetDevice(kGpuId);
//     std::string wts_name = "";
//     std::string engine_name = "";
//     std::string img_dir;
//     std::string sub_type = "";
//     std::string cuda_post_process = "";
//     int model_bboxes;
//     int is_p = 0;
//     float gd = 0.0f, gw = 0.0f;
//     int max_channels = 0;

//     if (!parse_args(argc, argv, wts_name, engine_name, is_p, img_dir, sub_type, cuda_post_process, gd, gw,
//                     max_channels)) {
//         std::cerr << "Arguments not right!" << std::endl;
//         std::cerr << "./yolov8 -s [.wts] [.engine] [n/s/m/l/x/n2/s2/m2/l2/x2/n6/s6/m6/l6/x6]  // serialize model to "
//                      "plan file"
//                   << std::endl;
//         std::cerr << "./yolov8 -d [.engine] ../samples  [c/g]// deserialize plan file and run inference" << std::endl;
//         return -1;
//     }

//     // Create a model using the API directly and serialize it to a file
//     if (!wts_name.empty()) {
//         serialize_engine(wts_name, engine_name, is_p, sub_type, gd, gw, max_channels);
//         return 0;
//     }

//     // Deserialize the engine from file
//     IRuntime* runtime = nullptr;
//     ICudaEngine* engine = nullptr;
//     IExecutionContext* context = nullptr;
//     deserialize_engine(engine_name, &runtime, &engine, &context);
//     cudaStream_t stream;
//     CUDA_CHECK(cudaStreamCreate(&stream));
//     cuda_preprocess_init(kMaxInputImageSize);
//     auto out_dims = engine->getBindingDimensions(1);
//     model_bboxes = out_dims.d[0];
//     // Prepare cpu and gpu buffers
//     float* device_buffers[2];
//     float* output_buffer_host = nullptr;
//     float* decode_ptr_host = nullptr;
//     float* decode_ptr_device = nullptr;

//     // Read images from directory
//     std::vector<std::string> file_names;
//     if (read_files_in_dir(img_dir.c_str(), file_names) < 0) {
//         std::cerr << "read_files_in_dir failed." << std::endl;
//         return -1;
//     }

//     prepare_buffer(engine, &device_buffers[0], &device_buffers[1], &output_buffer_host, &decode_ptr_host,
//                    &decode_ptr_device, cuda_post_process);

//     // batch predict
//     for (size_t i = 0; i < file_names.size(); i += kBatchSize) {
//         // Get a batch of images
//         std::vector<cv::Mat> img_batch;
//         std::vector<std::string> img_name_batch;
//         for (size_t j = i; j < i + kBatchSize && j < file_names.size(); j++) {
//             cv::Mat img = cv::imread(img_dir + "/" + file_names[j]);
//             img_batch.push_back(img);
//             img_name_batch.push_back(file_names[j]);
//         }
//         // Preprocess
//         cuda_batch_preprocess(img_batch, device_buffers[0], kInputW, kInputH, stream);
//         // Run inference
//         infer(*context, stream, (void**)device_buffers, output_buffer_host, kBatchSize, decode_ptr_host,
//               decode_ptr_device, model_bboxes, cuda_post_process);
//         std::vector<std::vector<Detection>> res_batch;
//         if (cuda_post_process == "c") {
//             // NMS
//             batch_nms(res_batch, output_buffer_host, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);
//         } else if (cuda_post_process == "g") {
//             //Process gpu decode and nms results
//             batch_process(res_batch, decode_ptr_host, img_batch.size(), bbox_element, img_batch);
//         }
//         // Draw bounding boxes
//         draw_bbox(img_batch, res_batch);
//         // Save images
//         for (size_t j = 0; j < img_batch.size(); j++) {
//             cv::imwrite("_" + img_name_batch[j], img_batch[j]);
//         }
//     }

//     // Release stream and buffers
//     cudaStreamDestroy(stream);
//     CUDA_CHECK(cudaFree(device_buffers[0]));
//     CUDA_CHECK(cudaFree(device_buffers[1]));
//     CUDA_CHECK(cudaFree(decode_ptr_device));
//     delete[] decode_ptr_host;
//     delete[] output_buffer_host;
//     cuda_preprocess_destroy();
//     // Destroy the engine
//     delete context;
//     delete engine;
//     delete runtime;

//     // Print histogram of the output distribution
//     //std::cout << "\nOutput:\n\n";
//     //for (unsigned int i = 0; i < kOutputSize; i++)
//     //{
//     //    std::cout << prob[i] << ", ";
//     //    if (i % 10 == 0) std::cout << std::endl;
//     //}
//     //std::cout << std::endl;

//     return 0;
// }
