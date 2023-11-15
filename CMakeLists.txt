cmake_minimum_required(VERSION 3.13)

project(
    msceqf
    VERSION 1.0
    DESCRIPTION "Multi State Constraint Equivariant Filter for visual inertial navigation"
    LANGUAGES CXX
)

# Set build type, Release by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

## Options
option(MSCEQF_TESTS "Build MSCEqF tests" OFF)
option(ROS_BUILD "Build MSCEqF with ROS" OFF)
option(ENABLE_ADDRESS_SANITIZER "Enable address sanitizer" OFF)
option(ENABLE_UNDEFINED_SANITIZER "Enable undefined behavior sanitizer" OFF)
option(ENABLE_THREAD_SANITIZER "Enable thread sanitizer" OFF)

## Include and set up external libraries
include(FetchContent)

# Googletest
if(MSCEQF_TESTS)
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

# Lie++ (and Eigen)
FetchContent_Declare(
    LiePlusPlus
    GIT_REPOSITORY  https://github.com/aau-cns/Lie-plusplus
    GIT_TAG         main
    GIT_SHALLOW     TRUE
    GIT_PROGRESS    TRUE
)
set(LIEPLUSPLUS_TESTS ${MSCEQF_TESTS})
list(APPEND external LiePlusPlus) 
list(APPEND include_dirs ${LIEPLUSPLUS_INCLUDE_DIR})
list(APPEND libs LiePlusPlus Eigen3::Eigen)

# # Eigen
# FetchContent_Declare(
#     Eigen
#     GIT_REPOSITORY  https://gitlab.com/libeigen/eigen.git
#     GIT_TAG         3.4.0
#     GIT_SHALLOW     TRUE
#     GIT_PROGRESS    TRUE
# )
# set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
# set(EIGEN_BUILD_DOC OFF)
# set(EIGEN_BUILD_PKGCONFIG OFF)
# set(EIGEN_TEST_CXX11 ON)
# set(EIGEN_HAS_CXX11_MATH ON)
# list(APPEND external Eigen)
# list(APPEND libs Eigen3::Eigen)

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

message(STATUS "Eigen VER: " ${EIGEN3_VERSION})
message(STATUS "Lie++ VER: " ${LiePlusPlus_VERSION})
message(STATUS "Yaml-cpp VER: " ${YAML_CPP_VERSION})
message(STATUS "Boost VER: " ${Boost_VERSION})
message(STATUS "OpenCV VER: " ${OpenCV_VERSION})

include(CheckCXXCompilerFlag)

# Set compiler flags
set(RELEASE_FLAGS "-DNDEBUG" "-O3" "-fsee" "-fomit-frame-pointer" "-fno-signed-zeros" "-fno-math-errno" "-funroll-loops" "-fno-unsafe-math-optimizations" "-flto" "-march=native")
set(DEBUG_FLAGS "-O0" "-g3" "-Wall" "-Wextra" "-Werror" "-Wuninitialized" "-Wmaybe-uninitialized" "-pedantic" "-fno-omit-frame-pointer")

CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no c++17 support. Please use a different C++ compiler.")
endif()
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    foreach(FLAG ${DEBUG_FLAGS})
        string(REPLACE "-" "" FLAG_NAME ${FLAG})
        string(TOUPPER ${FLAG_NAME} FLAG_VAR_NAME)
        CHECK_CXX_COMPILER_FLAG("${FLAG}" COMPILER_SUPPORTS_${FLAG_VAR_NAME})
        if (COMPILER_SUPPORTS_${FLAG_VAR_NAME})
            set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${FLAG}")
        else()
            message("${FLAG} was requested but is not supported.")
        endif()
    endforeach()
    if (ENABLE_ADDRESS_SANITIZER)
        CHECK_CXX_COMPILER_FLAG("-fsanitize=address" COMPILER_SUPPORTS_ADDRESS_SANITIZER)
        if (COMPILER_SUPPORTS_ADDRESS_SANITIZER)
            set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address -fsanitize-recover=address")
        else()
            message("-fsanitize=address -fsanitize-recover=address were requested but is not supported.")
        endif()
    elseif (ENABLE_UNDEFINED_SANITIZER)
        CHECK_CXX_COMPILER_FLAG("-fsanitize=undefined" COMPILER_SUPPORTS_UNDEFINED_SANITIZER)
        if (COMPILER_SUPPORTS_UNDEFINED_SANITIZER)
            set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=undefined")
        else()
            message("-fsanitize=undefined was requested but is not supported.")
        endif()
    elseif(ENABLE_THREAD_SANITIZER)
        CHECK_CXX_COMPILER_FLAG("-fsanitize=thread" COMPILER_SUPPORTS_THREAD_SANITIZER)
        if (COMPILER_SUPPORTS_THREAD_SANITIZER)
            set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=thread")
        else()
            message("-fsanitize=thread was requested but is not supported.")
        endif()
    endif()
elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
    foreach(FLAG ${RELEASE_FLAGS})
        string(REPLACE "-" "" FLAG_NAME ${FLAG})
        string(TOUPPER ${FLAG_NAME} FLAG_VAR_NAME)
        CHECK_CXX_COMPILER_FLAG("${FLAG}" COMPILER_SUPPORTS_${FLAG_VAR_NAME})
        if (COMPILER_SUPPORTS_${FLAG_VAR_NAME})
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${FLAG}")
        else()
            message("${FLAG} was requested but is not supported.")
        endif()
    endforeach()
endif()

## List source files
list(
    APPEND lib_sources
    source/msceqf/msceqf.cpp
    source/msceqf/options/msceqf_option_parser.cpp
    source/msceqf/state/state.cpp
    source/msceqf/system/system.cpp
    source/msceqf/symmetry/symmetry.cpp
    source/msceqf/filter/propagator/propagator.cpp
    source/msceqf/filter/updater/zero_velocity_updater.cpp
    source/msceqf/filter/updater/updater.cpp
    source/msceqf/filter/updater/updater_helper.cpp
    source/msceqf/filter/checker/checker.cpp
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
