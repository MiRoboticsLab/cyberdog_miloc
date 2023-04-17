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

#ifndef CYBERDOG_MILOC__UTILS__MILOC_KDTREE_HPP_
#define CYBERDOG_MILOC__UTILS__MILOC_KDTREE_HPP_

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>
#include <utility>

namespace cyberdog
{
namespace miloc
{

using point_t = std::vector<double>;
using indexArr = std::vector<size_t>;
using pointIndex = typename std::pair<std::vector<double>, size_t>;

class KDNode
{
public:
  using KDNodePtr = std::shared_ptr<KDNode>;
  size_t index;
  point_t x;
  KDNodePtr left;
  KDNodePtr right;

  // initializer
  KDNode();
  KDNode(
    const point_t &, const size_t &, const KDNodePtr &,
    const KDNodePtr &);
  KDNode(const pointIndex &, const KDNodePtr &, const KDNodePtr &);
  ~KDNode();

  // getter
  double coord(const size_t &);

  // conversions
  explicit operator bool();
  explicit operator point_t();
  explicit operator size_t();
  explicit operator pointIndex();
};

using KDNodePtr = std::shared_ptr<KDNode>;

KDNodePtr NewKDNodePtr();

// square euclidean distance
inline double dist2(const point_t &, const point_t &);
inline double dist2(const KDNodePtr &, const KDNodePtr &);

// euclidean distance
inline double dist(const point_t &, const point_t &);
inline double dist(const KDNodePtr &, const KDNodePtr &);

// Need for sorting
class comparer
{
public:
  size_t idx;
  explicit comparer(size_t idx_);
  inline bool compare_idx(
    const std::pair<std::vector<double>, size_t> &,         //
    const std::pair<std::vector<double>, size_t> &         //
  );
};

using pointIndexArr = typename std::vector<pointIndex>;

inline void sort_on_idx(
  const pointIndexArr::iterator &,                       //
  const pointIndexArr::iterator &,                       //
  size_t idx);

using pointVec = std::vector<point_t>;

class KDTree
{
  KDNodePtr root;
  KDNodePtr leaf;

  KDNodePtr make_tree(
    const pointIndexArr::iterator & begin,                    //
    const pointIndexArr::iterator & end,                      //
    const size_t & length,                                    //
    const size_t & level                                      //
  );

public:
  KDTree() = default;
  explicit KDTree(pointVec point_array);

private:
  KDNodePtr nearest_(
    //
    const KDNodePtr & branch,    //
    const point_t & pt,          //
    const size_t & level,        //
    const KDNodePtr & best,      //
    const double & best_dist     //
  );

  // default caller
  KDNodePtr nearest_(const point_t & pt);

public:
  point_t nearest_point(const point_t & pt);
  size_t nearest_index(const point_t & pt);
  pointIndex nearest_pointIndex(const point_t & pt);

private:
  pointIndexArr neighborhood_(
    //
    const KDNodePtr & branch,    //
    const point_t & pt,          //
    const double & rad,          //
    const size_t & level         //
  );

public:
  pointIndexArr neighborhood(
    //
    const point_t & pt,         //
    const double & rad);

  pointVec neighborhood_points(
    //
    const point_t & pt,           //
    const double & rad);

  indexArr neighborhood_indices(
    //
    const point_t & pt,            //
    const double & rad);
};

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__UTILS__MILOC_KDTREE_HPP_
