cmake_minimum_required(VERSION 3.13)

project(
    msceqf
    VERSION 1.0
    DESCRIPTION "Multi State Constraint Equivariant Filter for visual inertial navigation"
    LANGUAGES CXX
)

## Options
if (NOT BUILD_TESTS)
    set(BUILD_TESTS OFF)
endif()
if (NOT ROS_BUILD)
    set(ROS_BUILD OFF)
endif()

## Include and set up external libraries
include(FetchContent)

# Googletest
if(${BUILD_TESTS})
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY  https://github.com/google/googletest.git
        GIT_TAG         release-1.12.1
        GIT_SHALLOW     TRUE
        GIT_PROGRESS    TRUE
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    list(APPEND external googletest)
    list(APPEND include_dirs ${googletest_INCLUDE_DIR})
    list(APPEND libs ${googletest_LIBRARIES})
endif()

# Eigen
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY  https://gitlab.com/libeigen/eigen.git
    GIT_TAG         3.4.0
    GIT_SHALLOW     TRUE
    GIT_PROGRESS    TRUE
)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(EIGEN_BUILD_DOC OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
list(APPEND external Eigen)
list(APPEND include_dirs ${EIGEN3_INCLUDE_DIR})
list(APPEND libs Eigen3::Eigen ${EIGEN3_LIBRARIES})

# YamlCPP
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY  https://github.com/jbeder/yaml-cpp.git
    GIT_TAG         master
    GIT_SHALLOW     TRUE
    GIT_PROGRESS    TRUE
)
list(APPEND external yaml-cpp)
list(APPEND include_dirs ${YAML_CPP_INCLUDE_DIR})
list(APPEND libs yaml-cpp::yaml-cpp ${YAML_CPP_LIBRARIES})

# Boost
# FetchContent_Declare(
#     Boost
#     GIT_REPOSITORY  https://github.com/boostorg/boost.git
#     GIT_TAG         boost-1.81.0 
#     GIT_SHALLOW     TRUE
#     GIT_PROGRESS    TRUE
# )
# list(APPEND external Boost)
find_package(Boost REQUIRED)
list(APPEND include_dirs ${Boost_INCLUDE_DIRS})
list(APPEND libs ${Boost_LIBRARIES})

# OpenCV
# FetchContent_Declare(
#     opencv
#     GIT_REPOSITORY https://github.com/opencv/opencv.git
#     GIT_TAG        4.7.0
#     GIT_SHALLOW     TRUE
#     GIT_PROGRESS    TRUE
# )
# list(APPEND external opencv)
find_package(OpenCV 4 REQUIRED)
list(APPEND include_dirs ${OpenCV_INCLUDE_DIRS})
list(APPEND libs ${OpenCV_LIBRARIES})

FetchContent_MakeAvailable(${external})

message(STATUS "Eigen:    " ${EIGEN3_VERSION})
message(STATUS "Yaml-cpp: " ${YAML_CPP})
message(STATUS "Boost:    " ${Boost_VERSION})
message(STATUS "OpenCV:   " ${OpenCV_VERSION})

message(STATUS "BUILD TESTS = ${BUILD_TESTS}")
message(STATUS "ROS BUILD = ${ROS_BUILD}")

## Try to compile with C++ 17 (make sure GCC>=7, clang>=5, MSVC>=19.12)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    set(CMAKE_CXX_STANDARD 17)
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no c++17 support. Please use a different C++ compiler.")
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "Set compiler ${CMAKE_CXX_COMPILER}.")

## Define build type, Release by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE}.")

# Set CMAKE compile flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -DNDEBUG -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -fno-unsafe-math-optimizations -pthread")
# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -Wextra -Werror -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -fsanitize=thread -pthread")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g3 -Wall -Wextra -Werror -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -fsanitize=address -fsanitize-recover=address -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -Wextra -Werror -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -fsanitize=undefined -pthread")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g3 -Wall -Wextra -Werror -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -Wpadded -pthread")
message(STATUS "CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}.")

## List source files
list(
    APPEND lib_sources
    source/msceqf/msceqf.cpp
    source/msceqf/options/msceqf_option_parser.cpp
    source/msceqf/state/state.cpp
    source/msceqf/system/system.cpp
    source/msceqf/symmetry/symmetry.cpp
    source/msceqf/filter/propagator/propagator.cpp
    source/msceqf/filter/updater/updater.cpp
    source/msceqf/filter/updater/updater_helper.cpp
    source/msceqf/filter/initializer/static_initializer.cpp
    source/vision/camera.cpp
    source/vision/tracker.cpp
    source/vision/track_manager.cpp
)

## Define includes
list(APPEND include_dirs ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${include_dirs})

# Define build
if (ROS_BUILD)
    find_package(catkin QUIET COMPONENTS roscpp)
    find_package(ament_cmake QUIET)
    if (catkin_FOUND)
        message(STATUS "ROS 1 version found, building with ROS1")
        include(${CMAKE_CURRENT_SOURCE_DIR}/wrappers/ros/ros1/cmake/ros1.cmake)
    elseif (ament_cmake_FOUND)
        message(STATUS "ROS 2 version found, building with ROS2")
        include(${CMAKE_CURRENT_SOURCE_DIR}/wrappers/ros/ros2/cmake/ros2.cmake)
    else ()
        message(STATUS "No ROS versions found, building native C++")
        include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/native.cmake)
    endif ()
else ()
    message(STATUS "Building native C++")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/native.cmake)
endif ()
