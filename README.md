## CYBERDOG_MILOC
cyberdog_miloc is a visual mapping and localization module based on a trinocular camera system, used for visual mapping and navigation. During the mapping process, it utilizes robot poses output from MIVINS and images for sparse reconstruction. During the navigation process, it provides the pose located in the map。

## Prerequisites

### system
Ubuntu 18.04/20.04
### Ros
ROS2:galactic
### Deep learning
Depend on Jetpad 4.6，includ CUDA 10.2、cuDNN 8.2.1、TensorRT 8.0.1
### others 
- OpenCV 4.2.0
- yaml-cpp
- ceres
- eigen3
- colmap 4.7: in cyberdog_miloc/lib/colmap

## Build Cyberdog_miloc

```shell
colcon build -merge-install --install-base /opt/ros2/cyberdog
```

## RUN

```shell
source /opt/ros2/cyberdog/setup.bash
ros2 launch cyberdog_miloc miloc_server_launch.py
```
miloc_server will check and update the deep learning model when it is connected to the network.

- Mapping mode

depend on cyberdog_occmap、MIVINS、and camera images

- Reloc mode

depend on camera images