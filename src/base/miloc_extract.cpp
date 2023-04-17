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

#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <utility>

#include <glog/logging.h>
#include <colmap/lib/SQLite/sqlite3.h>
#include <boost/algorithm/string.hpp>

#include "base/miloc_extract.hpp"
#include "base/miloc_io.hpp"
#include "utils/miloc_basic.hpp"
#include "utils/miloc_database.hpp"
#include "utils/miloc_convertor.hpp"

namespace cyberdog
{
namespace miloc
{

int ExtractFeatures(
  const std::string & map_url, const std::vector<std::string> & image_list,
  SparseMapDB * map_db, std::unordered_map<std::string, int> & name_map,
  const std::string golbal_data_dir,
  GlobalModel * global_model, LocalModel * local_model)
{
  int ret = SLAM_OK;
  std::string mask_folder = map_url + "/masks";
  std::string image_folder = map_url + "/images";

  GlobalFeatureMap global_data;
  RawImage cur_image;
  LocalFeatureData cur_local_feature_data;
  GlobalFeatureData cur_global_feature_data;
  const int image_num = image_list.size();
  map_db->Begin();
  for (int i = 0; i < image_num && ret == SLAM_OK; i++) {
    if (name_map.find(image_list[i]) == name_map.end()) {
      continue;
    }
    ret |= LoadJpg(image_folder + "/" + image_list[i], cur_image);
    cur_local_feature_data.image_name = image_list[i];
    cur_global_feature_data.image_name = image_list[i];

    ret |= global_model->Process(cur_image, cur_global_feature_data);
    ret |= local_model->Process(cur_image, cur_local_feature_data);

    KeypointRecover(cur_local_feature_data, cur_image.raw_height, cur_image.raw_width);

    // global_data.push_back(cur_global_feature_data);
    global_data.insert(
      std::make_pair(
        cur_global_feature_data.image_name,
        cur_global_feature_data.feature));
    int colmap_id = name_map[image_list[i]];
    ret |= map_db->WriteLocalFeature_nocommit(colmap_id, cur_local_feature_data);
  }
  map_db->Commit();

  ReleaseImg(cur_image);

  ret |= RawSave(global_data, golbal_data_dir);

  return ret;
}

void FilterMatchData(const MatchData & match_data, std::vector<std::array<int, 2UL>> & match_pair)
{
  const std::vector<int> & match_idxs = match_data.match_idxs;
  int match_num = match_idxs.size();
  match_pair.clear();
  for (int i = 0; i < match_num; i++) {
    if (match_idxs[i] != -1) {
      match_pair.emplace_back(std::array<int, 2UL>{i, match_idxs[i]});
    }
  }
}

int ExtractMatches(
  const std::vector<std::string> & image_list, int sample_num,
  std::vector<std::string> & name_pair_list, SparseMapDB * map_db,
  std::unordered_map<std::string, int> & name_map, MatchModel * match_model)
{
  LocalFeatureData feature1, feature2;
  MatchData match_data;
  int ret = SLAM_OK;
  int image_num_front, image_num_left, image_num_right, image_num_unknown;
  image_num_front = image_num_left = image_num_right = image_num_unknown = 0;
  std::vector<std::string> image_list_front, image_list_left, image_list_right;
  std::vector<std::string> fields;
  std::vector<std::array<int, 2UL>> match_pair;

  for (auto image_name : image_list) {
    if (name_map.find(image_name) == name_map.end()) {
      continue;
    }
    boost::split(fields, image_name, boost::is_any_of("/"));
    if ("front" == fields.front()) {
      ++image_num_front;
      image_list_front.push_back(image_name);
    } else if ("left" == fields.front()) {
      ++image_num_left;
      image_list_left.push_back(image_name);
    } else if ("right" == fields.front()) {
      ++image_num_right;
      image_list_right.push_back(image_name);
    } else {
      ++image_num_unknown;
    }
  }

  map_db->Begin();
  for (int i = 0; i < image_num_front && ret == SLAM_OK; ++i) {
    int image_id1 = name_map[image_list_front[i]];
    ret |= map_db->ReadLocalFeature(image_id1, feature1);

    // process front pair
    const int end = std::min(i + sample_num, image_num_front - 1);
    for (int j = i + 1; j <= end; ++j) {
      int image_id2 = name_map[image_list_front[j]];

      ret |= map_db->ReadLocalFeature(image_id2, feature2);
      match_data.match_pair_name = std::move(image_list_front[i] + " " + image_list_front[j]);

      if (image_id1 < image_id2) {
        ret |= match_model->Process(feature1, feature2, match_data);
      } else {
        ret |= match_model->Process(feature2, feature1, match_data);
      }
      FilterMatchData(match_data, match_pair);
      if (match_pair.size() >= 15) {
        ret |=
          map_db->WriteMatchFeature_nocommit(
          colmap::Database::ImagePairToPairId(
            image_id1,
            image_id2),
          match_pair);
        name_pair_list.push_back(match_data.match_pair_name);
      }
    }
  }

  // process left pair
  for (int i = 0; i < image_num_left && ret == SLAM_OK; ++i) {
    int image_id1 = name_map[image_list_left[i]];
    ret |= map_db->ReadLocalFeature(image_id1, feature1);

    boost::split(fields, image_list_left[i], boost::is_any_of("/"));

    const int end = std::min(i + sample_num, image_num_left - 1);
    for (int j = i + 1; j <= end; ++j) {
      int image_id2 = name_map[image_list_left[j]];
      ret |= map_db->ReadLocalFeature(image_id2, feature2);

      boost::split(fields, image_list_left[j], boost::is_any_of("/"));

      match_data.match_pair_name = std::move(image_list_left[i] + " " + image_list_left[j]);

      if (image_id1 < image_id2) {
        ret |= match_model->Process(feature1, feature2, match_data);
      } else {
        ret |= match_model->Process(feature2, feature1, match_data);
      }
      FilterMatchData(match_data, match_pair);
      if (match_pair.size() >= 15) {
        ret |=
          map_db->WriteMatchFeature_nocommit(
          colmap::Database::ImagePairToPairId(
            image_id1,
            image_id2),
          match_pair);
        name_pair_list.push_back(match_data.match_pair_name);
      }
    }
  }

  // process right pair
  for (int i = 0; i < image_num_right && ret == SLAM_OK; ++i) {
    int image_id1 = name_map[image_list_right[i]];
    ret |= map_db->ReadLocalFeature(image_id1, feature1);

    boost::split(fields, image_list_right[i], boost::is_any_of("/"));

    const int end = std::min(i + sample_num, image_num_right - 1);
    for (int j = i + 1; j <= end; ++j) {
      int image_id2 = name_map[image_list_right[j]];
      ret |= map_db->ReadLocalFeature(image_id2, feature2);

      boost::split(fields, image_list_right[j], boost::is_any_of("/"));

      match_data.match_pair_name = std::move(image_list_right[i] + " " + image_list_right[j]);

      if (image_id1 < image_id2) {
        ret |= match_model->Process(feature1, feature2, match_data);
      } else {
        ret |= match_model->Process(feature2, feature1, match_data);
      }
      FilterMatchData(match_data, match_pair);
      if (match_pair.size() >= 15) {
        ret |=
          map_db->WriteMatchFeature_nocommit(
          colmap::Database::ImagePairToPairId(
            image_id1,
            image_id2),
          match_pair);
        name_pair_list.push_back(match_data.match_pair_name);
      }
    }
  }
  map_db->Commit();

  return ret;
}

}  // namespace miloc
}  // namespace cyberdog
