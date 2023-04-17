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

#ifndef CYBERDOG_MILOC__MILOC_TYPES_HPP_
#define CYBERDOG_MILOC__MILOC_TYPES_HPP_

#include <opencv2/opencv.hpp>

#include <string>

#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace miloc
{

typedef struct ImgData_
{
  ImgData_() {}
  ~ImgData_() {}
  cv::Mat img_front;
  cv::Mat img_left;
  cv::Mat img_right;
  int64_t time_stamp;
} ImgData;

typedef enum
{
  kUNINITIALIZED = 0,
  kINACTIVE,
  kMAPPABLE,
  kRELOCALIZABLE,
  kINITIALIZING,
  kRELOCATING,
  kRECONSTRUCTING,
  kMAPPING,
  kSUBSCRIBING,
  kUNSUBSCRIBING,
  kLOADING,
  kUNLOADING,
  kUPDATING,
} MilocStatus;

typedef enum
{
  kSUCCESS = 1,
  kEXCEPTION,
  kFIELD_ERROR,
  kFILE_ERROR,
  kSTATUS_ERROR,
} ReplyStatus;

template<typename T>
T param(
  rclcpp::Node * nh, const std::string & name, const T & defaultValue,
  bool silent = false)
{
  T v;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  if (!nh->has_parameter(name)) {
    nh->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
  }
  if (nh->get_parameter(name, v)) {
    if (!silent) {
      INFO_STREAM(
        "Found parameter: " << name << ", value: " << v);
    }
    return v;
  }
  if (!silent) {
    WARN_STREAM(
      "Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
  }
  return defaultValue;
}

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__MILOC_TYPES_HPP_
