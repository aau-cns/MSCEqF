[![License](https://img.shields.io/badge/License-AAUCNS-336B81.svg)](./LICENSE)

![MSCEqF logo](./resources/msceqf_logo.png)

> MSCEqF: Multi State Constraint Equivariant Filter

MSCEqF is a multi-state constraint equivariant filter for visual-inertial navigation. 
It is based on the recent advances in equivaraint inertial navigation systems [[1](#1), [2](#2), [3](#3), [4](#3)].

## Features
### Design features

- Developed as a pure C++ library with ROS1 and ROS2 wrappers available

### Filter features

- Supports online camera extrinsic and intrinsic parameters calibration
- Supports unit-plane projection method
- Supports anchored euclidean, anchored inverse depth and anchored polar feature representation methods
- Includes a static initialization routine as well as parametric initialization with custom origin
- Includes an equivariant zero velocity update routine

### Vision frontend features

- OpenCV based
- Supports a grid-based multi-thread parallel feature extraction 
- Supports different features detector including FAST and Shi-Tomasi
- Supports different image enhancment tecniques, including Histogram and CLAHE

### Future roadmap

 - [x] ROS1 wrapper
 - [x] ROS2 wrapper
 - [x] Equivariant Zero velocity Update (EqZVU)
 - [ ] Unit-sphere projection method support
 - [ ] Equivariant Persistent (SLAM) features update support

## Documentation

Doxygen documentation is available here: [MSCEqF documentation](https://aau-cns.github.io/MSCEqF)

## Dependencies

MSCEqF has the following dependencies which are automatically downloaded and linked against:

- [Eigen](https://gitlab.com/libeigen/eigen.git)
- [Lie++](https://github.com/aau-cns/Lie-plusplus)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [googletest](https://github.com/google/googletest.git)
- [Boost](https://github.com/boostorg/boost.git)
- [OpenCV](https://github.com/opencv/opencv.git)

## Getting started
### ROS free setup
```sh
$ git clone https://github.com/aau-cns/MSCEqF.git msceqf
$ cd msceqf
$ export BUILD_TYPE=<TYPE>  # Replace <TYPE> with one of these: Release, Debug, RelWithDebInfo, ...
$ mkdir -p build/$BUILD_TYPE
$ cd build/$BUILD_TYPE && cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DBUILD_TESTS=ON ../..
$ cmake --build . --config $BUILD_TYPE --target all -j && cd ../..
```

### Run tests
```sh
$ cd msceqf/build/$BUILD_TYPE
$ ./msceqf_tests
```

### Run example (Euroc)

After downloading the [Euroc](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) follows

```sh
$ cd msceqf/build/$BUILD_TYPE
$ ./msceqf_euroc <sequence_name> <euroc_dataset_folder> <euroc_example_folder>
```

### ROS1 setup
```sh
$ cd ws/src
$ git clone https://github.com/aau-cns/MSCEqF.git msceqf
$ cd msceqf
$ export BUILD_TYPE=<TYPE>  # Replace <TYPE> with one of these: Release, Debug, RelWithDebInfo, ...
$ catkin build -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DROS_BUILD=ON
```

### ROS2 setup
```sh
$ cd ws/src
$ git clone https://github.com/aau-cns/MSCEqF.git msceqf
$ cd msceqf
$ export BUILD_TYPE=<TYPE>  # Replace <TYPE> with one of these: Release, Debug, RelWithDebInfo, ...
$ colcon build --event-handlers console_cohesion+ --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE --cmake-args -DROS_BUILD=ON
```

### Docker setup
```sh
$ sudo apt update
$ sudo apt install -y nvidia-docker2
$ sudo systemctl restart docker
$ cd <path_to_msceqf_folder>
$ export ROS_VERSION=<Version>  # Enter either 1 or 2 (e.g. ROS_VERSION=1)
$ docker build --network=host -t msceqf:ros$ROS_VERSION -f docker/Dockerfile_ros$ROS_VERSION
$ xhost +
$ docker run --net=host -it --gpus all --env="NVIDIA_DRIVER_CAPABILITIES=all" --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" msceqf:ros$ROS_VERSION .
```

If Nvidia drivere are not supported, simply run docker as follows

```sh
$ docker run --net=host -it --gpus all --env="NVIDIA_DRIVER_CAPABILITIES=all" --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" msceqf:ros$ROS_VERSION .
```

## Usage with custom dataset and/or with ROS

Utilizing MSCEqF with a custom dataset or specific sensors is a straightforward process. Follow these steps for seamless integration:

### Dataset/Sensor preparation

Ensure you possess the camera intrinsic and extrinsic parameters calibration if working with a custom dataset. In case you are working with real sensors, perform a camera calibration before starting. We recommend using [Kalibr](https://github.com/ethz-asl/kalibr) for efficient camera calibration.

### MSCEqF configuration file

Navigate to the desired location to store the filter configuration file:

```sh
cd <path_where_to_store_the_filter_configfile>
nano <configfile_name>.yaml
```

Populate your configuration file with the following settings:

```yaml
# Initial standard deviations (attitude, velocity, position, bias, extrinsics, instrinsics)
extended_pose_std: [1.0e-1, 1.0e-1, 1.0e-9, 1.0e-1, 1.0e-1, 1.0e-1, 1.0e-9, 1.0e-9, 1.0e-9]
bias_std: [1.0e-1, 1.0e-1, 1.0e-1, 1.0e-1, 1.0e-1, 1.0e-1]
extrinsics_std: [1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2, 1.0e-2]
intrinsics_std: [1.0, 1.0, 1.0, 1.0]

# IMU noise statistics
accelerometer_noise_density: 1.0-2
accelerometer_random_walk:   1.0e-3
gyroscope_noise_density: 1.0e-3
gyroscope_random_walk:   1.0e-4

# Camera calibration (according to kalibr format, both T_imu_cam and T_cam_imu)
distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
distortion_model: radtan
resolution: [320, 240]
intrinsics: [250.0, 250.0, 160.0, 120.0]
T_imu_cam: 
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, -1.0, 0.0, 0.0]
  - [0.0, 0.0, -1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]

# Initializer options
# For IMU only motion detection set static_initializer_disparity_threshold: 0.0
# For DISPARITY only motion detection set static_initializer_acc_threshold: 0.0
static_initializer_imu_window: 1.0
static_initializer_disparity_window: 0.5
static_initializer_acc_threshold: 0.25
static_initializer_disparity_threshold: 1.0

# Propagator options
# For numerical exponential computation (costly) set state_transition_order: -1
# For approximated first-order exponential computation (recommended on low-power hardware) set state_transition_order: 0
state_transition_order: 0
imu_buffer_max_size: 1000

# Updater options
# Possible options for zero_velocity_update are enabled, disabled, beginning
refine_traingulation: true
feature_min_depth: 0.1
feature_max_depth: 20
feature_refinement_max_iterations: 20
feature_refinement_tollerance: 1e-10
measurement_projection_method: unit_plane
feature_representation: anchored_inverse_depth
pixel_standerd_deviation: 1.0
curvature_correction: true
zero_velocity_update: enabled

# State options
enable_camera_intrinsic_calibration: false
gravity: 9.81
num_clones: 11

# Tracker options
# Possible options for feature_detector are fast and shi-tomasi
equalization_method: histogram
optical_flow_pyramid_levels: 3
detector_pyramid_levels: 1
feature_detector: fast
grid_x_size: 4
grid_y_size: 4
min_feature_pixel_distance: 15
min_features: 100
max_features: 120
fast_threshold: 20
shi_tomasi_quality_level: 0.75

# Track Manager
max_track_length: 400

# Logger level 
# Possible levels are 0: Full, 1: INFO, 2: WARN, 3: ERR, 4: INACTIVE
logger_level: 1
```

Adjust values as needed and customize settings thresholds based on your specific requirements.

## License

This software is made available to the public to use (source-available), licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, the full terms of which are made available in the [LICENSE](LICENSE) file. 

### Usage for academic purposes
If you use this software in an academic research setting, please cite the corresponding papers.

```latex
@article{fornasier2023msceqf,
  title={MSCEqF: A Multi State Constraint Equivariant Filter for Vision-aided Inertial Navigation},
  author={Fornasier, Alessandro and van Goor, Pieter and Allak, Eren and Mahony, Robert and Weiss, Stephan},
  journal={arXiv preprint arXiv:2311.11649},
  year={2023}
}

@article{fornasier2023equivariant,
  title={Equivariant Symmetries for Inertial Navigation Systems},
  author={Fornasier, Alessandro and Ge, Yixiao and van Goor, Pieter and Mahony, Robert and Weiss, Stephan},
  journal={arXiv preprint arXiv:2309.03765},
  year={2023}
}
```

## References
<a id="1">[1]</a> van Goor, Pieter, Tarek Hamel, and Robert Mahony. "Equivariant filter (eqf)." IEEE Transactions on Automatic Control (2022).

<a id="2">[2]</a> Fornasier, Alessandro, et al. "Equivariant filter design for inertial navigation systems with input measurement biases." 2022 International Conference on Robotics and Automation (ICRA). IEEE, 2022.

<a id="3">[3]</a> Fornasier, Alessandro, et al. "Overcoming Bias: Equivariant Filter Design for Biased Attitude Estimation with Online Calibration." IEEE Robotics and Automation Letters 7.4 (2022): 12118-12125.

<a id="4">[4]</a> Fornasier, Alessandro, et al. "Equivariant Symmetries for Inertial Navigation Systems." ArXiv preprint.
