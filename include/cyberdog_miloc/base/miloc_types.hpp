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

#ifndef CYBERDOG_MILOC__BASE__MILOC_TYPES_HPP_
#define CYBERDOG_MILOC__BASE__MILOC_TYPES_HPP_

#include <stdio.h>

#include <string>
#include <array>
#include <vector>
#include <unordered_set>
#include <unordered_map>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace cyberdog
{
namespace miloc
{

struct RealCamera
{
  int camera_id = -1;
  int camera_type = -1;
  int height = 0;
  int width = 0;
  std::string model_name;
  std::string prefix;
  std::vector<double> camera_intrinsic;
  std::array<double, 16> camera_extrinsic;
};

struct RawImage
{
  int height;
  int width;
  int raw_height;
  int raw_width;
  int channel;
  int pitch;
  int elem_type;   // sizeof element type
  void * data = nullptr;
};

struct ImageBundle
{
  bool LOCAL_FLAG = true;
  int camera_id;
  int64_t timestamp;
  cv::Mat image;
};

struct TwoViewGeometry
{
  int rows;
  int cols;
  std::vector<int> match_data;
  int config;
  std::array<double, 9> F;
  std::array<double, 9> E;
  std::array<double, 9> H;
  std::array<double, 4> qvec;
  std::array<double, 3> tvec;
};

struct MapperParams
{
  double translation_interval = 0.;
  double orientation_interval = 0.;
  double max_reproj_error = 0.;
  double min_tri_angle = 0.;
  double min_track_len = 0.;
  int num_matched = 0;
};

struct ModelParams
{
  float mask_th = 0.2;
};

struct SceneParam
{
  int retrieval_num = 0;
  int inlier_num = 0;
  int ransac_thresh = 0;
};

typedef std::array<float, 2> Point2f;
typedef std::array<float, 3> Point3f;

struct MapInfo
{
  std::string map_name;
  std::string map_url;
  int status;
  int scene;
  float version;
  int64_t create_time;
  int64_t update_time;
  int slam_flag;
};

struct OccMapData
{
  uint64_t map_load_time;
  float resolution;
  uint32_t width;
  uint32_t height;
  double position_x;
  double position_y;
  double position_z;
  Eigen::Quaterniond quater;
  std::vector<int8_t> data;
};

struct DbImage
{
  int image_id;
  std::string name;
  std::vector<int> point3d_ids;
  std::vector<Point2f> xys;
  std::vector<float> tvec;
  std::vector<float> qvec;
};
typedef std::vector<DbImage> DBImages;

struct DbPoints3D
{
  int point_id;
  Point3f xyz;
  std::vector<int> image_ids;
  std::vector<int> point2d_idxs;
};
typedef std::vector<DbPoints3D> DBPoints3D;

typedef Eigen::Quaterniond Quat;
typedef Eigen::Vector3d Trans;

struct MilocConfig
{
  MapperParams mapper_params;
  std::string camera_model_path;
  std::string map_path;
  std::string map_table_name;
  std::string local_model_path;
  std::string global_model_path;
  std::string match_model_path;

  std::vector<float> confidence;
  double position_threshold;

  ModelParams local_model_params;

  SceneParam reloc_param;

  bool traj_from_visual;

  bool GetTrajFromVisual()
  {
    return this->traj_from_visual;
  }

  void SetTrajFromVisual(bool visual_slam)
  {
    this->traj_from_visual = visual_slam;
  }
};

typedef enum
{
  SECOND = 0,
  MILLISECOND,
  MICROSECOND,
  NANOSECOND,
} TimeStamp;

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
  SUCCESS = 1,
  EXCEPTION,
  FIELD_ERROR,
  FILE_ERROR,
  STATUS_ERROR,
} ReplyStatus;

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__BASE__MILOC_TYPES_HPP_
