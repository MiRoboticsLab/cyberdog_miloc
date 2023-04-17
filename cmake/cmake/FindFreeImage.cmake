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

set(FREEIMAGE_INCLUDE_DIR_HINTS "" CACHE PATH "FreeImage include directory")
set(FREEIMAGE_LIBRARY_DIR_HINTS "" CACHE PATH "FreeImage library directory")

unset(FREEIMAGE_FOUND)
unset(FREEIMAGE_INCLUDE_DIRS)
unset(FREEIMAGE_LIBRARIES)

list(APPEND FREEIMAGE_CHECK_INCLUDE_DIRS
    ${FREEIMAGE_INCLUDE_DIR_HINTS}
    /usr/include
    /usr/local/include
    /opt/include
    /opt/local/include
)

list(APPEND FREEIMAGE_CHECK_LIBRARY_DIRS
    ${FREEIMAGE_LIBRARY_DIR_HINTS}
    /usr/lib
    /usr/local/lib
    /opt/lib
    /opt/local/lib
)

find_path(FREEIMAGE_INCLUDE_DIRS
    NAMES
    FreeImage.h
    PATHS
    ${FREEIMAGE_CHECK_INCLUDE_DIRS})
find_library(FREEIMAGE_LIBRARIES
    NAMES
    freeimage
    PATHS
    ${FREEIMAGE_CHECK_LIBRARY_DIRS})

if(FREEIMAGE_INCLUDE_DIRS AND FREEIMAGE_LIBRARIES)
    set(FREEIMAGE_FOUND TRUE)
endif()

if(FREEIMAGE_FOUND)
    message(STATUS "Found FreeImage")
    message(STATUS "  Includes : ${FREEIMAGE_INCLUDE_DIRS}")
    message(STATUS "  Libraries : ${FREEIMAGE_LIBRARIES}")
else()
    if(FreeImage_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find FreeImage")
    endif()
endif()
