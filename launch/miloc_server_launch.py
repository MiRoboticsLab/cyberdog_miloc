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

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from calibration_parser import parse_calibr_file


def generate_launch_description():
    share_dir = get_package_share_directory("cyberdog_miloc")
    config_file = os.path.join(share_dir, "config/config.yml")
    parse_calibr_file("/params/camera/calibration", os.path.join(share_dir, "config"))
    namespace = LaunchConfiguration("namespace", default="")
    return LaunchDescription(
        [
            Node(
                package="cyberdog_miloc",
                executable="miloc_server",
                name="miloc_server",
                output="screen",
                emulate_tty=True,
                namespace=namespace,
                parameters=[
                    {
                        "config_path": config_file,
                        "cam0_topic": "image_rgb",
                        "cam1_topic": "image_left",
                        "cam2_topic": "image_right",
                        "odom_slam": "odom_slam",
                        "odom_out": "odom_out",
                        "reloc_failure_threshold": 20,
                        "immediately_reconstruct": True,
                    }
                ],
            )
        ]
    )
