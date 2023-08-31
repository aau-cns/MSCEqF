# MSCEqF

MSCEqF is an equivariant filter for visual-inertial navigation. 
It is based on the recent advances in equivaraint inertial navigation systems [[1](#1), [2](#2), [3](#3)], and it is the first ever presented multi-state constraint equivaraint filter.

## Features
### Design features

- Developed as a pure C++ library with ROS1 and ROS2 wrappers available
- Includes a completely independednt Lie group library
- State covariance access based on a indexing system that makes it very easy to work with
- Includes a multi-level logger

### Filter features

- Supports online camera extrinsic and intrinsic parameters calibration
- Supports unit-plane and unit-sphere projection methods
- Supports anchored euclidean, anchored inverse depth and anchored polar feature representation methods
- Includes a static initialization routine as well as custom origin

### Vision frontend features

- Supports a grid-based multi-thread parallel feature extraction 
- Supports different features detector including FAST and Shi-Tomasi
- Supports different image enhancment tecniques, including Histogram and CLAHE
- Includes a completely independednt Lie group library
- Includes a completely independednt Lie group library
- Includes a completely independednt Lie group library

## Dependencies

*MSCEqF* has the following dependencies which are automatically downloaded and linked against:

- Eigen
- yaml-cpp
- googletest
- Boost
- OpenCV

## Getting started
### Setup
```sh
$ git clone <url> msceqf                                                                # Get the code
$ cd msceqf                                                                             # Move into msceqf folder
$ mkdir -p build/<build_type>                                                           # Create folder for build
$ cd build/<build_type> && cmake -DCMAKE_BUILD_TYPE=<build_type> -DBUILD_TESTS=ON ../.. # Configure build
$ cmake --build . --config Debug --target all -j && cd ../..                            # Build
```

### Run tests
```sh
$ cd msceqf/build/<build_type>  # Move into build folder
$ ./msceqf_tests                # Run tests
```

### Run example
```sh
$ cd msceqf/build/<build_type>                                                  # Move into build folder
$ ./msceqf_euroc <sequence_name> <euroc_dataset_folder> <euroc_example_folder>  # Run Euroc example
```

## License

This software is made available to the public to use (source-available), licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, the full terms of which are made available in the [LICENSE](LICENSE) file. 

## References
<a id="1">[1]</a> van Goor, Pieter, Tarek Hamel, and Robert Mahony. "Equivariant filter (eqf)." IEEE Transactions on Automatic Control (2022).
<a id="2">[2]</a> Fornasier, Alessandro, et al. "Equivariant filter design for inertial navigation systems with input measurement biases." 2022 International Conference on Robotics and Automation (ICRA). IEEE, 2022.
<a id="3">[3]</a> Fornasier, Alessandro, et al. "Overcoming Bias: Equivariant Filter Design for Biased Attitude Estimation with Online Calibration." IEEE Robotics and Automation Letters 7.4 (2022): 12118-12125.
