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

#ifndef CYBERDOG_MILOC__BASE__MILOC_RESTRUCT_HPP_
#define CYBERDOG_MILOC__BASE__MILOC_RESTRUCT_HPP_

#include <vector>
#include <string>

#include "base/miloc_types.hpp"
#include "runtime/miloc_trt_runtime.hpp"

namespace cyberdog
{
namespace miloc
{

class RestructMapper
{
public:
  RestructMapper()
  : status_(-1)
  {
  }

  // init the RestructMapper class with model info, construct params and camera infos
  int Init(MapperParams * map_params, MilocConfig * config, std::vector<RealCamera> * cameras);

  int InitModel(LocalModel * lmodel, GlobalModel * gmodel, MatchModel * mmodel)
  {
    local_model_ = lmodel;
    global_model_ = gmodel;
    match_model_ = mmodel;
    return SLAM_OK;
  }

  void ReleaseModel()
  {
    local_model_ = nullptr;
    global_model_ = nullptr;
    match_model_ = nullptr;
  }

  // construct map in given map url
  int ConstructMap(const std::string & map_url);

private:
  int status_;

  MapperParams * map_params_;
  std::vector<RealCamera> * cameras_;
  LocalModel * local_model_;
  GlobalModel * global_model_;
  MatchModel * match_model_;
  MilocConfig * config_;
};

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__BASE__MILOC_RESTRUCT_HPP_
