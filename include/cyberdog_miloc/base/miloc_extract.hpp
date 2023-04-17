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

#ifndef CYBERDOG_MILOC__BASE__MILOC_EXTRACT_HPP_
#define CYBERDOG_MILOC__BASE__MILOC_EXTRACT_HPP_

#include <string>
#include <vector>
#include <unordered_map>

#include "base/miloc_types.hpp"
#include "base/miloc_colmap_helper.hpp"
#include "utils/miloc_database.hpp"

namespace cyberdog
{
namespace miloc
{

int ExtractFeatures(
  const std::string & map_url, const std::vector<std::string> & image_list,
  SparseMapDB * map_db, std::unordered_map<std::string, int> & name_map,
  const std::string golbal_data_dir,
  GlobalModel * global_model, LocalModel * local_model);

void FilterMatchData(
  const MatchData & match_data, std::vector<std::array<int,
  2UL>> & match_pair);

int ExtractMatches(
  const std::vector<std::string> & image_list, int sample_num,
  std::vector<std::string> & name_pair_list, SparseMapDB * map_db,
  std::unordered_map<std::string, int> & name_map, MatchModel * match_model);

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__BASE__MILOC_EXTRACT_HPP_
