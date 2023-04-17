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

#include <math.h>

#include <chrono>
#include <string>
#include <functional>
#include <vector>
#include <algorithm>

#include "runtime/miloc_trt_runtime.hpp"
#include "runtime/miloc_mat.hpp"
#include "runtime/miloc_model_pre_process.hpp"

namespace cyberdog
{
namespace miloc
{

int LocalModel::Load(const std::string & model_path)
{
  int status = SLAM_ERROR;
  status = net_.LoadModel(model_path);
  if (status != SLAM_OK) {
    LOG(ERROR) << "load local model failed";
    return SLAM_ERROR;
  }
  return status;
}

int LocalModel::FilterKps(LocalFeatureData & output)
{
  auto iter = std::upper_bound(
    output.scores.begin(),
    output.scores.end(), m_params.mask_th, std::greater<float>());
  if (iter == output.scores.end()) {
    std::vector<float>().swap(output.scores);
    return SLAM_OK;
  } else {
    auto dis_kp = iter - output.scores.begin();
    output.keypoints.erase(output.keypoints.begin() + dis_kp, output.keypoints.end());
    auto dis_feature = dis_kp * 96;
    output.features.erase(output.features.begin() + dis_feature, output.features.end());
    std::vector<float>().swap(output.scores);
  }
  return SLAM_OK;
}

int LocalModel::Process(const RawImage & input_raw, LocalFeatureData & ouput)
{
  int status = SLAM_OK;

  int channel = input_raw.channel;
  int height = input_raw.height;
  int width = input_raw.width;

  std::vector<int> shape_src = {channel, height, width};
  std::vector<int> shape_kps = {1, 1500, 2};
  std::vector<int> shape_desc = {1, 1500, 96};
  std::vector<int> shape_score = {1, 1500};

  int format = SLAM_MAT_GPU | SLAM_MAT_CH_FIRST;

  MilocMat pre_src, model_src;
  MilocMat model_dst_kps, model_dst_scores, model_dst_desc;

  status = pre_src.Init(shape_src, SLAM_MAT_U8, format);
  status |= model_src.Init(shape_src, SLAM_MAT_F32, format);

  status |= model_dst_kps.Init(shape_kps, SLAM_MAT_F32, format);
  status |= model_dst_scores.Init(shape_score, SLAM_MAT_F32, format);
  status |= model_dst_desc.Init(shape_desc, SLAM_MAT_F32, format);

  if (status != SLAM_OK) {
    LOG(ERROR) << "mat init failed";
    return SLAM_ERROR;
  }
  cudaMemcpy(pre_src.data_, input_raw.data, pre_src.data_size_, cudaMemcpyHostToDevice);
  status = LocalModelAPreProcess(pre_src, model_src);
  if (status != SLAM_OK) {
    LOG(ERROR) << "LocalModel PreProcess failed";
    return SLAM_ERROR;
  }
  std::vector<MilocMat *> src_list, dst_list;
  src_list.reserve(1);
  dst_list.reserve(3);
  src_list.push_back(&model_src);
  dst_list.push_back(&model_dst_scores);
  dst_list.push_back(&model_dst_desc);
  dst_list.push_back(&model_dst_kps);

  status = net_.Inference(src_list, dst_list);
  if (status != SLAM_OK) {
    LOG(ERROR) << "inference model failed";
    return SLAM_ERROR;
  }

  ouput.features.resize(1500 * 96);
  ouput.keypoints.resize(1500);
  ouput.scores.resize(1500);

  cudaMemcpy(
    ouput.features.data(), model_dst_desc.data_, model_dst_desc.data_size_, cudaMemcpyDeviceToHost);
  cudaMemcpy(
    ouput.keypoints.data(), model_dst_kps.data_, model_dst_kps.data_size_, cudaMemcpyDeviceToHost);
  cudaMemcpy(
    ouput.scores.data(), model_dst_scores.data_,
    model_dst_scores.data_size_, cudaMemcpyDeviceToHost);

  FilterKps(ouput);

  return status;
}

int GlobalModel::Load(const std::string & model_path)
{
  int status = SLAM_ERROR;
  status = net_.LoadModel(model_path);
  if (status != SLAM_OK) {
    LOG(ERROR) << "load global model failed";
    return SLAM_ERROR;
  }
  return status;
}

int GlobalModel::Process(const RawImage & input_raw, GlobalFeatureData & ouput)
{
  int status = SLAM_OK;
  // preprocess
  int channel = input_raw.channel;
  int height = input_raw.height;
  int width = input_raw.width;

  std::vector<int> shape_src = {channel, height, width};
  std::vector<int> shape_dst = {1, 1024};
  int format = SLAM_MAT_GPU | SLAM_MAT_CH_FIRST;
  MilocMat pre_src, model_src, model_dst;
  status = pre_src.Init(shape_src, SLAM_MAT_U8, format);
  status |= model_src.Init(shape_src, SLAM_MAT_F32, format);
  status |= model_dst.Init(shape_dst, SLAM_MAT_F32, format);
  if (status != SLAM_OK) {
    LOG(ERROR) << "mat init failed";
    return SLAM_ERROR;
  }

  cudaMemcpy(pre_src.data_, input_raw.data, pre_src.data_size_, cudaMemcpyHostToDevice);

  status = LocalModelAPreProcess(pre_src, model_src);
  if (status != SLAM_OK) {
    LOG(ERROR) << "global PreProcess failed";
    return SLAM_ERROR;
  }
  // model inference
  std::vector<MilocMat *> src_list, dst_list;
  src_list.reserve(1);
  dst_list.reserve(1);
  src_list.push_back(&model_src);
  dst_list.push_back(&model_dst);
  status = net_.Inference(src_list, dst_list);
  if (status != SLAM_OK) {
    LOG(ERROR) << "inference model failed";
    return SLAM_ERROR;
  }

  // output
  cudaMemcpy(ouput.feature.data(), model_dst.data_, model_dst.data_size_, cudaMemcpyDeviceToHost);
  return status;
}

int MatchModel::Load(const std::string & model_path)
{
  int status = SLAM_ERROR;
  status = net_.LoadModel(model_path);
  if (status != SLAM_OK) {
    LOG(ERROR) << "load match model failed";
    return SLAM_FALSE;
  }

  return status;
}

int MatchModel::Process(LocalFeatureData & input1, LocalFeatureData & input2, MatchData & output)
{
  int status = SLAM_OK;

  int num_k1 = input1.keypoints.size();
  int num_k2 = input2.keypoints.size();

  if (num_k1 < 5 || num_k2 < 5) {
    output.match_idxs.resize(0);
    return SLAM_OK;
  }

  status |= net_.SetDimensions("d1", nvinfer1::Dims3(1, num_k1, 96));
  status |= net_.SetDimensions("d2", nvinfer1::Dims3(1, num_k2, 96));

  if (status != SLAM_OK) {
    LOG(ERROR) << "SetDimensions failed";
    return SLAM_ERROR;
  }

  // model inference
  std::vector<int> shape_d1 = {1, num_k1, 96};
  std::vector<int> shape_d2 = {1, num_k2, 96};
  std::vector<int> shape_match = {1, num_k1};

  int format = SLAM_MAT_GPU | SLAM_MAT_CH_FIRST;
  MilocMat mat_d1, mat_d2, mat_dst_match;
  status |= mat_d1.Init(shape_d1, SLAM_MAT_F32, format);
  status |= mat_d2.Init(shape_d2, SLAM_MAT_F32, format);

  status |= mat_dst_match.Init(shape_match, SLAM_MAT_S32, format);

  if (status != SLAM_OK) {
    LOG(ERROR) << "mat init failed";
    return SLAM_ERROR;
  }

  cudaMemcpy(mat_d1.data_, input1.features.data(), mat_d1.data_size_, cudaMemcpyHostToDevice);
  cudaMemcpy(mat_d2.data_, input2.features.data(), mat_d2.data_size_, cudaMemcpyHostToDevice);

  std::vector<MilocMat *> src_list, dst_list;
  src_list.reserve(2);
  dst_list.reserve(1);
  src_list.push_back(&mat_d1);
  src_list.push_back(&mat_d2);

  dst_list.push_back(&mat_dst_match);

  status = net_.Inference(src_list, dst_list);
  if (status != SLAM_OK) {
    LOG(ERROR) << "inference model failed";
    return SLAM_ERROR;
  }

  // output
  output.match_idxs.resize(num_k1);
  cudaMemcpy(
    output.match_idxs.data(), mat_dst_match.data_,
    mat_dst_match.data_size_, cudaMemcpyDeviceToHost);

  return status;
}

}  // namespace miloc
}  // namespace cyberdog
