[![License](https://img.shields.io/badge/License-AAUCNS-336B81.svg)](./LICENSE)

![MSCEqF logo](./resources/msceqf_logo.png)

> MSCEqF: Multi State Constraint Equivariant Filter

MSCEqF is a multi-state constraint equivariant filter for visual-inertial navigation. 
It is based on the recent advances in equivaraint inertial navigation systems [[1](#1), [2](#2), [3](#3), [4](#3)].

## Features
### Design features

- Developed as a pure C++ library with ROS1 and ROS2 wrappers available
- Includes a completely independednt Lie group library

### Filter features

- Supports online camera extrinsic and intrinsic parameters calibration
- Supports unit-plane projection method
- Supports anchored euclidean, anchored inverse depth and anchored polar feature representation methods
- Includes a static initialization routine as well as parametric initialization with custom origin
- Includes an equivariant zero-velocity update routine

### Vision frontend features

- Supports a grid-based multi-thread parallel feature extraction 
- Supports different features detector including FAST and Shi-Tomasi
- Supports different image enhancment tecniques, including Histogram and CLAHE

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
$ git clone <url> msceqf
$ cd msceqf
$ mkdir -p build/<build_type>
$ cd build/<build_type> && cmake -DCMAKE_BUILD_TYPE=<build_type> -DBUILD_TESTS=ON ../..
$ cmake --build . --config Debug --target all -j && cd ../..
```

### Run tests
```sh
$ cd msceqf/build/<build_type>
$ ./msceqf_tests
```

### Run example
```sh
$ cd msceqf/build/<build_type>
$ ./msceqf_euroc <sequence_name> <euroc_dataset_folder> <euroc_example_folder>
```

### ROS1 setup
```sh
$ cd ws/src
$ git clone <url> msceqf
$ cd msceqf
$ catkin build -DCMAKE_BUILD_TYPE=<build_type> -DROS_BUILD=ON
```

### ROS2 setup
```sh
$ cd ws/src
$ git clone <url> msceqf
$ cd msceqf
$ colcon build --event-handlers console_cohesion+ --cmake-args " -DCMAKE_BUILD_TYPE=<build_type>" --cmake-args " -DROS_BUILD=ON"
```

### Docker setup
```sh
$ sudo apt update
$ sudo apt install -y nvidia-docker2
$ sudo systemctl restart docker
$ cd <path_to_msceqf_folder>
$ docker build --network=host -t msceqf:ros<ros_version> -f docker/Dockerfile_ros<ros_version>
$ xhost +
$ docker run --net=host -it --gpus all --env="NVIDIA_DRIVER_CAPABILITIES=all" --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" msceqf:ros<ros_version>
```

### ROS1 setup
```sh
$ cd ws/src                                                                             # Move into ROS workspace
$ git clone <url> msceqf                                                                # Get the code
$ cd msceqf                                                                             # Move into msceqf folder
$ catkin build -DCMAKE_BUILD_TYPE=<build_type> -DROS_BUILD=ON                           # ROS1 build
```

### ROS2 setup
```sh
$ cd ws/src                                                                             # Move into ROS workspace
$ git clone <url> msceqf                                                                # Get the code
$ cd msceqf                                                                             # Move into msceqf folder
$ colcon build -DCMAKE_BUILD_TYPE=<build_type> -DROS_BUILD=ON                           # ROS2 build
```

## License

This software is made available to the public to use (source-available), licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, the full terms of which are made available in the [LICENSE](LICENSE) file. 

### Usage for academic purposes
If you use this software in an academic research setting, please cite the corresponding papers.

```latex
@article{fornasier2023msceqf,
  title={MSCEqF: A Multi State Constraint Equivariant Filter for Vision-aided Inertial Navigation},
  author={Fornasier, Alessandro and van Goor, Pieter and Allak, Eren and Mahony, Robert and Weiss, Stephan},
  journal={arXiv preprint arXiv:XXXX.XXXXX},
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
