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
import numpy as np
from argparse import ArgumentParser
import json
import yaml


# Default camera parameters
camera_params = {
    "front": {
        "camera_id": 1,
        "camera_model": "OPENCV",
        "width": 640,
        "height": 480,
        "camera_params": [
            457.55644572907505,
            457.190400941217,
            322.2551003474975,
            229.56894408223258,
            0.025852137769511972,
            -0.035249332885501264,
            -0.0009384555727436185,
            0.00045420552326488976,
        ],
        "transform": [
            [-0.01231423, -0.9998885, -0.00844572, 0.00113042],
            [-0.17892178, 0.01051341, -0.98380713, 0.173745],
            [0.98378624, -0.01060371, -0.17903129, -0.23316083],
            [0.0, 0.0, 0.0, 1.0],
        ],
    },
    "left": {
        "camera_id": 2,
        "camera_model": "MEI_FISHEYE",
        "width": 500,
        "height": 400,
        "camera_params": [
            471.6538714331181,
            471.3570399597825,
            249.53647898674774,
            201.35902130792937,
            1.175811280568886,
            -0.040729180767331466,
            0.05589553983176914,
            -0.0005013553546627219,
            -0.002690470794127832,
        ],
        "transform": [
            [0.99999625, 0.00013452, 0.00273578, 0.07065711],
            [0.00263377, 0.22706787, -0.97387537, -0.02829377],
            [-0.00075221, 0.97387892, 0.22706667, -0.08906161],
            [0.0, 0.0, 0.0, 1.0],
        ],
    },
    "right": {
        "camera_id": 3,
        "camera_model": "MEI_FISHEYE",
        "width": 500,
        "height": 400,
        "camera_params": [
            458.2523512519358,
            458.7769028744702,
            248.59019367838644,
            202.362205150993,
            1.1216011654831164,
            -0.0630205474701937,
            0.06487751875734442,
            0.00015888117424251786,
            0.00036892456072691463,
        ],
        "transform": [
            [-0.99999905, 0.00013149, 0.00137589, -0.07138776],
            [-0.00137634, -0.00352451, -0.99999284, -0.01186412],
            [-0.00012664, -0.99999377, 0.00352468, -0.09159569],
            [0.0, 0.0, 0.0, 1.0],
        ],
    },
}


def load_yaml_file(base_dir):
    try:
        with open(
            os.path.join(base_dir, "params_intrinsic.yaml"), "r"
        ) as intrinsic_file:
            intrinsics = yaml.load(intrinsic_file, Loader=yaml.SafeLoader)

        with open(
            os.path.join(base_dir, "params_extrinsic.yaml"), "r"
        ) as extrinsic_file:
            extrinsics = yaml.load(extrinsic_file, Loader=yaml.SafeLoader)

        with open(
            os.path.join(base_dir, "params_bodyimu_extrinsic.yaml"), "r"
        ) as odom_extrinsic_file:
            odom_extrinsic = yaml.load(
                odom_extrinsic_file, Loader=yaml.SafeLoader)

        # camera intrinsics
        cam_intrinsics = {}
        cams = {"cam1": "front", "cam4": "left", "cam5": "right"}
        for cam in cams.keys():
            intrinsic = intrinsics[cam]["intrinsics"]
            distortion = intrinsics[cam]["distortion_coeffs"]
            camera_model = intrinsics[cam]["camera_model"]
            distortion.pop(2)
            if camera_model == "pinhole":
                cam_intrinsics[cams[cam]] = intrinsic + distortion
            elif camera_model == "omni":
                intrinsic.append(intrinsic[0])
                intrinsic.pop(0)
                cam_intrinsics[cams[cam]] = intrinsic + distortion

        # camera extrinsics
        T_crgb_c0 = np.array(extrinsics["cam1"]["T_cn_c0"])
        T_cleft_c0 = np.array(extrinsics["cam4"]["T_cn_c0"])
        T_cright_c0 = np.array(extrinsics["cam5"]["T_cn_c0"])

        # odom extrinsics
        T_cam_odom = odom_extrinsic["cam0"]["T_cam_imu"]
        T_cam0_odom = np.array(T_cam_odom)

        T_crgb_odom = np.matmul(T_crgb_c0, T_cam0_odom)
        T_cleft_odom = np.matmul(T_cleft_c0, T_cam0_odom)
        T_cright_odom = np.matmul(T_cright_c0, T_cam0_odom)

        cam_extrinsics = {}
        cam_extrinsics["front"] = T_crgb_odom.tolist()
        cam_extrinsics["left"] = T_cleft_odom.tolist()
        cam_extrinsics["right"] = T_cright_odom.tolist()

        return cam_intrinsics, cam_extrinsics
    except:
        print("[miloc] [calibration_parser.py] Load calibration files failed")
        return None, None


def parse_calibr_file(base_dir, output_dir):
    if os.path.exists(os.path.join(output_dir, "camera_model.json")):
        print("[miloc] [calibration_parser.py] Camera model configure already exists")
        return

    cam_intrinsics, cam_extrinsics = load_yaml_file(base_dir)
    if cam_intrinsics != None and cam_extrinsics != None:
        cams = ["front", "left", "right"]
        for cam in cams:
            camera_params[cam]["camera_params"] = cam_intrinsics[cam]
            camera_params[cam]["transform"] = cam_extrinsics[cam]

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
    else:
        print("[miloc] [calibration_parser.py] Use default value")

    # Write into json file
    with open(os.path.join(output_dir, "camera_model.json"), "w") as f:
        print("[miloc] [calibration_parser.py] Generate camera model json file")
        json.dump(camera_params, f, indent=4, ensure_ascii=False)


if __name__ == "__main__":
    parse = ArgumentParser()
    parse.add_argument("--base_dir")
    parse.add_argument("--output_dir")
    args = parse.parse_args()
    parse_calibr_file(args.base_dir, args.output_dir)
