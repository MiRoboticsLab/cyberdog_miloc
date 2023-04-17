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

set(EIGEN3_INCLUDE_DIR_HINTS "" CACHE PATH "Eigen include directory")

unset(EIGEN3_FOUND)
unset(EIGEN3_VERSION)
unset(EIGEN3_INCLUDE_DIRS)

if(NOT Eigen3_FIND_VERSION)
    if(NOT Eigen3_FIND_VERSION_MAJOR)
        set(Eigen3_FIND_VERSION_MAJOR 2)
    endif()

    if(NOT Eigen3_FIND_VERSION_MINOR)
        set(Eigen3_FIND_VERSION_MINOR 91)
    endif()

    if(NOT Eigen3_FIND_VERSION_PATCH)
        set(Eigen3_FIND_VERSION_PATCH 0)
    endif()

    set(Eigen3_FIND_VERSION "${Eigen3_FIND_VERSION_MAJOR}.${Eigen3_FIND_VERSION_MINOR}.${Eigen3_FIND_VERSION_PATCH}")
endif()

macro(_EIGEN3_CHECK_VERSION)
    file(READ "${EIGEN3_INCLUDE_DIRS}/Eigen/src/Core/util/Macros.h" _EIGEN3_VERSION_HEADER)

    string(REGEX MATCH "define[ \t]+EIGEN_WORLD_VERSION[ \t]+([0-9]+)" _EIGEN3_WORLD_VERSION_MATCH "${_EIGEN3_VERSION_HEADER}")
    set(EIGEN3_WORLD_VERSION "${CMAKE_MATCH_1}")
    string(REGEX MATCH "define[ \t]+EIGEN_MAJOR_VERSION[ \t]+([0-9]+)" _EIGEN3_MAJOR_VERSION_MATCH "${_EIGEN3_VERSION_HEADER}")
    set(EIGEN3_MAJOR_VERSION "${CMAKE_MATCH_1}")
    string(REGEX MATCH "define[ \t]+EIGEN_MINOR_VERSION[ \t]+([0-9]+)" _EIGEN3_MINOR_VERSION_MATCH "${_EIGEN3_VERSION_HEADER}")
    set(EIGEN3_MINOR_VERSION "${CMAKE_MATCH_1}")

    set(EIGEN3_VERSION ${EIGEN3_WORLD_VERSION}.${EIGEN3_MAJOR_VERSION}.${EIGEN3_MINOR_VERSION})

    if(${EIGEN3_VERSION} VERSION_LESS ${Eigen3_FIND_VERSION})
        set(EIGEN3_VERSION_OK FALSE)
    else()
        set(EIGEN3_VERSION_OK TRUE)
    endif()

    if(NOT EIGEN3_VERSION_OK)
        message(STATUS "Eigen version ${EIGEN3_VERSION} found in ${EIGEN3_INCLUDE_DIRS}, "
                       "but at least version ${Eigen3_FIND_VERSION} is required.")
    endif()
endmacro()

if(EIGEN3_INCLUDE_DIRS)
    _EIGEN3_CHECK_VERSION()
    set(EIGEN3_FOUND ${EIGEN3_VERSION_OK})
else()
    find_path(EIGEN3_INCLUDE_DIRS
        NAMES
        signature_of_eigen3_matrix_library
        PATHS
        ${EIGEN3_INCLUDE_DIR_HINTS}
        /usr/include
        /usr/local/include
        /opt/include
        /opt/local/include
        PATH_SUFFIXES
        eigen3
        eigen
    )

    if(EIGEN3_INCLUDE_DIRS)
        _EIGEN3_CHECK_VERSION()
    endif()

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(Eigen3 DEFAULT_MSG EIGEN3_INCLUDE_DIRS EIGEN3_VERSION_OK)

    mark_as_advanced(EIGEN3_INCLUDE_DIRS)
endif()

if(EIGEN3_FOUND)
    message(STATUS "Found Eigen")
    message(STATUS "  Includes : ${EIGEN3_INCLUDE_DIRS}")
else()
    if(Eigen3_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find Eigen")
    endif()
endif()