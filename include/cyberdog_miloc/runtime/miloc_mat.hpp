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

#ifndef CYBERDOG_MILOC__RUNTIME__MILOC_MAT_HPP_
#define CYBERDOG_MILOC__RUNTIME__MILOC_MAT_HPP_

#include <unistd.h>

#include <iostream>
#include <vector>
#include <string>


namespace cyberdog
{
namespace miloc
{

typedef enum
{
  SLAM_MAT_U8  = 0,   /*!< element type is u8 */
  SLAM_MAT_S8  = 1,   /*!< element type is s8 */
  SLAM_MAT_U16 = 2,   /*!< element type is u16 */
  SLAM_MAT_S16 = 3,   /*!< element type is s16 */
  SLAM_MAT_U32 = 4,   /*!< element type is u32 */
  SLAM_MAT_S32 = 5,   /*!< element type is s32 */
  SLAM_MAT_U64 = 6,   /*!< element type is u64 */
  SLAM_MAT_S64 = 7,   /*!< element type is s64 */
  SLAM_MAT_F32 = 8,   /*!< element type is f32 */
  SLAM_MAT_F64 = 9,   /*!< element type is f64 */
} MilocMatType;

typedef enum
{
  SLAM_MAT_CH_FIRST = 0,
  SLAM_MAT_CH_LAST  = 1,
  SLAM_MAT_CPU      = 0x000F0000,
  SLAM_MAT_GPU      = 0x00F00000,
} MilocMatFormat;

class MilocMat
{
public:
  MilocMat();

  ~MilocMat();

  int Init(std::vector<int> & shape_in, MilocMatType type_in, int format_in);

  int Clone(MilocMat & src, bool deep_clone = false);

  int ConvertTo();

  int Save(const std::string & filename);

  bool flag_malloc_;
  std::vector<int> shape_;
  int dims_;
  MilocMatType type_;
  int format_;
  void * data_;
  int64_t data_size_;
};

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__RUNTIME__MILOC_MAT_HPP_
