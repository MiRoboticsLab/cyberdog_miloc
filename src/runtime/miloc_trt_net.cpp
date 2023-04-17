// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <glog/logging.h>
#include <NvInferPlugin.h>

#include <string>
#include <memory>
#include <vector>

#include "runtime/miloc_trt_net.hpp"

namespace cyberdog
{
namespace miloc
{

// Logger for TensorRT info/warning/errors
class Logger : public nvinfer1::ILogger
{
public:
  Logger()
  : Logger(Severity::kWARNING) {}

  explicit Logger(Severity severity)
  : reportableSeverity(severity) {}

  void log(Severity severity, const char * msg) noexcept override
  {
    // suppress messages with severity enum value greater than the reportable
    if (severity > reportableSeverity) {return;}

    switch (severity) {
      case Severity::kINTERNAL_ERROR: std::cerr << "INTERNAL_ERROR: "; break;
      case Severity::kERROR: std::cerr << "ERROR: "; break;
      case Severity::kWARNING: std::cerr << "WARNING: "; break;
      case Severity::kINFO: std::cerr << "INFO: "; break;
      default: std::cerr << "UNKNOWN: "; break;
    }
    std::cerr << msg << std::endl;
  }

  Severity reportableSeverity{Severity::kWARNING};
} logger;

TrtNet::TrtNet()
: runtime_(nullptr), engine_(nullptr), context_(nullptr), input_count_(0), iteration_time_(0)
{
}
TrtNet::~TrtNet()
{
}

int TrtNet::LoadModel(const std::string & engine_file)
{
  initLibNvInferPlugins(&logger, "");
  std::ifstream fin(engine_file);
  std::string cached_engine = "";
  while (fin.peek() != EOF) {
    std::stringstream stream_buffer;
    stream_buffer << fin.rdbuf();
    cached_engine.append(stream_buffer.str());
  }
  fin.close();
  if (0 == cached_engine.size()) {
    LOG(ERROR) << "read model file failed";
    return SLAM_ERROR;
  }

  runtime_ = std::shared_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger));
  if (nullptr == runtime_) {
    LOG(ERROR) << "creat runtime failed";
    return SLAM_NULL_PTR;
  }

  engine_ =
    std::shared_ptr<nvinfer1::ICudaEngine>(
    runtime_->deserializeCudaEngine(
      cached_engine.data(),
      cached_engine.size()));
  if (nullptr == engine_) {
    LOG(ERROR) << "creat engine failed";
    return SLAM_NULL_PTR;
  }

  context_ = std::shared_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
  if (nullptr == context_) {
    LOG(ERROR) << "creat context failed";
    return SLAM_NULL_PTR;
  }

  int nbindings = engine_->getNbBindings();
  buffer_.resize(nbindings);

  for (int i = 0; i < nbindings; i++) {
    if (engine_->bindingIsInput(i)) {
      input_count_++;
    }
  }

  return SLAM_OK;
}

int TrtNet::SetDimensions(const std::string & input_name, nvinfer1::Dims dim)
{
  int index = engine_->getBindingIndex(input_name.c_str());
  int status = context_->setBindingDimensions(index, dim);
  if (status != SLAM_TRUE) {
    LOG(ERROR) << "setBindingDimensions failed";
    return SLAM_ERROR;
  }
  return SLAM_OK;
}

int TrtNet::Inference(std::vector<MilocMat *> & input_data, std::vector<MilocMat *> & output_data)
{
  if (input_data.empty() || output_data.empty()) {
    LOG(ERROR) << "src | dst is empty";
    return SLAM_ERROR;
  }

  if ((input_data.size() != (uint32_t)input_count_) ||
    ((input_data.size() + output_data.size()) != buffer_.size()))
  {
    LOG(ERROR) << "the length of src/dst is not match";
    return SLAM_ERROR;
  }

  for (int i = 0; i < static_cast<int>(buffer_.size()); i++) {
    if (i < input_count_) {
      buffer_[i] = input_data[i]->data_;
    } else {
      buffer_[i] = output_data[i - input_count_]->data_;
    }
  }

  int status = context_->executeV2(&buffer_[0]);
  if (status != SLAM_TRUE) {
    LOG(ERROR) << "inference failed";
    return SLAM_ERROR;
  }

  iteration_time_++;
  return SLAM_OK;
}

}  // namespace miloc
}  // namespace cyberdog
