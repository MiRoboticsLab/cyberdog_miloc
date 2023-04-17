# Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# min cmake version
cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0072 NEW)
cmake_policy(SET CMP0074 NEW)
project(cyberdog_miloc)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC ${OpenMP_CXX_FLAGS})
endif()

# set install
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")
endif()
message(STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")

# set build type
if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "release" CACHE STRING "release build type" FORCE)
endif()
if(CMAKE_BUILD_TYPE MATCHES "(Release|RELEASE|release)")
    add_definitions(-DRELEASE)
else()
    add_definitions(-DDEBUG)
endif()
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-10.2/)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
message(STATUS "${CMAKE_MODULE_PATH}")

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(protocol REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(colmap REQUIRED)
find_package(Ceres REQUIRED)
find_package(rapidjson REQUIRED)
find_package(image_transport REQUIRED)
find_package(cyberdog_visions_interfaces REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(CUDA ${CUDA_VERSION} REQUIRED)

set(CUDA_VERBOSE_BUILD ON)
set(CMAKE_CUDA_ARCHITECTURES 72)
set(CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER})
set(CUDA_LIBRARY cuda cudart cudnn cublas cublasLt)
set(TENSORRT_LIBRARY nvinfer nvinfer_plugin nvparsers)

set(THIRDPARTY_INC_DIC
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${COLMAP_INCLUDE_DIRS}
  ${rapidjson_INCLUDE_DIRS})
set(THIRDPARTY_LIB_DIC
  ${YAML_CPP_LINK_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${COLMAP_LINK_DIRS})
set(THIRDPARTY_LIBS
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
  ${COLMAP_LIBRARIES})

set(RUNTIME_INC_DIR
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR})
set(RUNTIME_LIBRARY
  ${CUDA_LIBRARY}
  ${TENSORRT_LIBRARY})

set_property(GLOBAL PROPERTY THIRDPARTY_INC_DIC ${THIRDPARTY_INC_DIC})
set_property(GLOBAL PROPERTY THIRDPARTY_LIB_DIC ${THIRDPARTY_LIB_DIC})
set_property(GLOBAL PROPERTY THIRDPARTY_LIBS ${THIRDPARTY_LIBS})

# add modules macro
macro(add_module new_module)
  set(NEW_MODULE_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/${new_module}")
  aux_source_directory(${NEW_MODULE_SRC_DIR} LIB_SRCS)
endmacro()

add_module(base)
add_module(utils)
add_module(misc)
add_module(api)
add_module(runtime)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}
  ${THIRDPARTY_INC_DIC}
  ${RUNTIME_INC_DIR})
link_directories(
  ${THIRDPARTY_LIB_DIC}
  ${CUDA_TOOLKIT_ROOT_DIR}/lib64)

cuda_add_library(miloc_core STATIC ${LIB_SRCS})
target_link_libraries(miloc_core
  ${THIRDPARTY_LIBS})

add_executable(miloc_server
  src/miloc_server.cpp
)
target_link_libraries(miloc_server
  miloc_core
  ${THIRDPARTY_LIBS}
  ${RUNTIME_LIBRARY})
ament_target_dependencies(miloc_server
  std_msgs
  sensor_msgs
  rclcpp
  protocol
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  cv_bridge
  image_transport
  cyberdog_visions_interfaces
  cyberdog_common)

install(TARGETS
  miloc_server
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()