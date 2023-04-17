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

set(GLEW_INCLUDE_DIR_HINTS "" CACHE PATH "Glew include directory")
set(GLEW_LIBRARY_DIR_HINTS "" CACHE PATH "Glew library directory")

unset(GLEW_FOUND)
unset(GLEW_INCLUDE_DIRS)
unset(GLEW_LIBRARIES)

find_path(GLEW_INCLUDE_DIRS
    NAMES
    GL/glew.h
    PATHS
    ${GLEW_INCLUDE_DIR_HINTS}
    /usr/include
    /usr/local/include
    /sw/include
    /opt/include
    /opt/local/include)
find_library(GLEW_LIBRARIES
    NAMES
    GLEW
    Glew
    glew
    glew32
    PATHS
    ${GLEW_LIBRARY_DIR_HINTS}
    /usr/lib64
    /usr/lib
    /usr/local/lib64
    /usr/local/lib
    /sw/lib
    /opt/lib
    /opt/local/lib)

if(GLEW_INCLUDE_DIRS AND GLEW_LIBRARIES)
    set(GLEW_FOUND TRUE)
    message(STATUS "Found Glew")
    message(STATUS "  Includes : ${GLEW_INCLUDE_DIRS}")
    message(STATUS "  Libraries : ${GLEW_LIBRARIES}")
else()
    set(GLEW_FOUND FALSE)
    if(Glew_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find Glew")
    endif()
endif()
