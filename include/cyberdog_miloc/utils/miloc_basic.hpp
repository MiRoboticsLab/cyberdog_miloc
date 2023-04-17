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

#ifndef CYBERDOG_MILOC__UTILS__MILOC_BASIC_HPP_
#define CYBERDOG_MILOC__UTILS__MILOC_BASIC_HPP_

namespace cyberdog
{
namespace miloc
{

// errorno
#define SLAM_OK                   (0)         // everithing is ok
#define SLAM_ERROR                (-1)        // func call error
#define SLAM_IO_ERROR             (-2)        // io error
#define SLAM_NO_MEM               (-3)        // insufficient memory
#define SLAM_NULL_PTR             (-4)        // null pointer
#define SLAM_BAD_ARG              (-5)        // arg/param is bad
#define SLAM_BAD_OPT              (-6)        // bad operation

// bool
#define SLAM_TRUE                 (1)
#define SLAM_FALSE                (0)

// basic func
#define SLAM_ALIGN(x, align)      (((x) + (align) - 1) & -(align))
#define SLAM_UNUSED(v)            (void)(v)


}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__UTILS__MILOC_BASIC_HPP_
