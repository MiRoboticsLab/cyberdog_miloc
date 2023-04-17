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

include(FindPackageHandleStandardArgs)

get_filename_component(COLMAP_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_FILE} PATH)
set(COLMAP_INSTALL_PREFIX "${COLMAP_INSTALL_PREFIX}/../")

set(COLMAP_FOUND FALSE)

set(EIGEN3_INCLUDE_DIR_HINTS)

set(FREEIMAGE_INCLUDE_DIR_HINTS)
set(FREEIMAGE_LIBRARY_DIR_HINTS)

set(GLEW_INCLUDE_DIR_HINTS)
set(GLEW_LIBRARY_DIR_HINTS)

set(GLOG_INCLUDE_DIR_HINTS)
set(GLOG_LIBRARY_DIR_HINTS)

# Find dependency packages.

set(TEMP_CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${COLMAP_INSTALL_PREFIX}/cmake/cmake)

if(COLMAP_FIND_QUIETLY)
    find_package(Ceres QUIET)

    find_package(Boost COMPONENTS
                program_options
                filesystem
                system
                unit_test_framework
                QUIET)

    find_package(Eigen3 QUIET)

    find_package(FreeImage QUIET)

    find_package(Glog QUIET)

    find_package(OpenGL QUIET)
    find_package(Glew QUIET)
else()
    find_package(Ceres REQUIRED)

    find_package(Boost COMPONENTS
                program_options
                filesystem
                system
                unit_test_framework
                REQUIRED)

    find_package(Eigen3 REQUIRED)

    find_package(FreeImage REQUIRED)

    find_package(Glog REQUIRED)

    find_package(OpenGL REQUIRED)
    find_package(Glew REQUIRED)
endif()

# Set the exported variables.
set(COLMAP_FOUND TRUE)

set(COLMAP_VERSION 3.8)

set(COLMAP_OPENMP_ENABLED ON)

set(COLMAP_INCLUDE_DIRS
    /usr/local/include/
    ${COLMAP_INSTALL_PREFIX}/lib/colmap/include
    ${COLMAP_INSTALL_PREFIX}/lib/colmap/include/colmap
    ${COLMAP_INSTALL_PREFIX}/lib/colmap/include/colmap/lib
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${FREEIMAGE_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
)

set(COLMAP_LINK_DIRS
    ${COLMAP_INSTALL_PREFIX}/lib/colmap/lib
    ${Boost_LIBRARY_DIRS}
)

set(COLMAP_INTERNAL_LIBRARIES
    flann
    sqlite3
)

set(COLMAP_EXTERNAL_LIBRARIES
    ${CMAKE_DL_LIBS}
    ${GLOG_LIBRARIES}
    ${FREEIMAGE_LIBRARIES}
    ${CERES_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
)

if(UNIX)
    list(APPEND COLMAP_EXTERNAL_LIBRARIES
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_PROGRAM_OPTIONS_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        pthread)
endif()

if(COLMAP_OPENMP_ENABLED)
    find_package(OpenMP QUIET)
    add_definitions("-DOPENMP_ENABLED")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(COLMAP_LIBRARIES
    colmap
    ${COLMAP_INTERNAL_LIBRARIES}
    ${COLMAP_EXTERNAL_LIBRARIES}
)

# Cleanup of configuration variables.

set(CMAKE_MODULE_PATH ${TEMP_CMAKE_MODULE_PATH})

unset(COLMAP_INSTALL_PREFIX)
unset(EIGEN3_INCLUDE_DIR_HINTS)
unset(FREEIMAGE_INCLUDE_DIR_HINTS)
unset(FREEIMAGE_LIBRARY_DIR_HINTS)
unset(GLEW_INCLUDE_DIR_HINTS)
unset(GLEW_LIBRARY_DIR_HINTS)
unset(GLOG_INCLUDE_DIR_HINTS)
unset(GLOG_LIBRARY_DIR_HINTS)
