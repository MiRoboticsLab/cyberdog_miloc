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

#ifndef CYBERDOG_MILOC__RUNTIME__MILOC_MODEL_PRE_PROCESS_HPP_
#define CYBERDOG_MILOC__RUNTIME__MILOC_MODEL_PRE_PROCESS_HPP_

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <algorithm>

#include "cuda_runtime.h"

#include "runtime/miloc_mat.hpp"
#include "utils/miloc_basic.hpp"

namespace cyberdog
{
namespace miloc
{

#define CUDA_NUM_THREADS 1024

int LocalModelAPreProcess(MilocMat & src, MilocMat & dst);
int GlobalModelAPreProcess(MilocMat & src, MilocMat & dst);

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__RUNTIME__MILOC_MODEL_PRE_PROCESS_HPP_
