## CYBERDOG_MILOC
cyberdog_miloc is a visual mapping and localization module based on a trinocular camera system, used for visual mapping and navigation. During the mapping process, it utilizes robot poses output from MIVINS and images for sparse reconstruction. During the navigation process, it provides the pose located in the map。

## Prerequisites

### system
Ubuntu 18.04/20.04
### Ros
ROS2:galactic
### Deep learning
Depend on Jetpack 4.6，include CUDA 10.2、cuDNN 8.2.1、TensorRT 8.0.1
### others 
- OpenCV 4.2.0
- yaml-cpp
- ceres
- eigen3
- colmap 4.7: in cyberdog_miloc/lib/colmap

## Build Cyberdog_miloc

```shell
colcon build --merge-install --install-base /opt/ros2/cyberdog
```

## Miloc Models

Miloc models needs to be manually updated from [GitHub](https://github.com/MiRoboticsLab/model_files/tree/main/miloc/models/2.0)

```shell
# delete old models
rm -rf /SSD/miloc/models/*
# down load 3 models  (global_models.trt local_model.trt match_models.trt) 
# from GitHub to /SSD/miloc/models/

# create version file and add version number
touch /SSD/miloc/models/version.toml
echo 'version = "2.0"' > /SSD/miloc/models/version.toml

```

## RUN

```shell
source /opt/ros2/cyberdog/setup.bash
ros2 launch cyberdog_miloc miloc_server_launch.py
```
miloc_server will check and update the deep learning model when it is connected to the network.

- Mapping mode

depend on cyberdog_occmap、cyberdog_mivins、and camera images

- Reloc mode

depend on camera images
