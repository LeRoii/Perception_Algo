#ifndef _IDETECTOR_H_

#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include "model.h"

#include <iostream>
#include <chrono>
#include <cmath>

using namespace nvinfer1;

class idetector
{
    public:
    idetector(const std::string &enginePath);
    ~idetector();
    void process(cv::Mat &img, cv::Mat &ret);
    void process(cv::Mat &img, cv::Mat &ret, std::vector<bbox_t> &boxs);


    private:
    std::vector<cv::Mat> img_batch;
    IRuntime* runtime = nullptr;
    ICudaEngine* engine = nullptr;
    IExecutionContext* context = nullptr;
    float* gpu_buffers[2];
    float* cpu_output_buffer = nullptr;
    cudaStream_t stream;
};

#endif