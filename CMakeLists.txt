cmake_minimum_required(VERSION 3.13)
project(
    msceqf
    VERSION 1.0
    DESCRIPTION "Multi State Constraint Equivariant Filter for visual inertial navigation"
    LANGUAGES CXX
)

## Options
# option(BUILD_TESTS "Build tests")
set(BUILD_TESTS ON)
message(STATUS "BUILD TESTS = ${BUILD_TESTS}")

## Include libraries
include(CMakeFindDependencyMacro)
find_dependency(Eigen3 3.4)

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
# FetchContent_GetProperties(Eigen)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(EIGEN_BUILD_DOC OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
list(APPEND external Eigen)
list(APPEND include_dirs ${EIGEN3_INCLUDE_DIR})
list(APPEND libs ${EIGEN3_LIBRARIES})

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

# # OpenCV
# FetchContent_Declare(
#     opencv
#     GIT_REPOSITORY https://github.com/opencv/opencv.git
#     GIT_TAG        4.7.0
#     GIT_SHALLOW     TRUE
#     GIT_PROGRESS    TRUE
# )
# list(APPEND external opencv)
find_package(OpenCV 3 REQUIRED)
list(APPEND include_dirs ${OpenCV_INCLUDE_DIRS})
list(APPEND libs ${OpenCV_LIBRARIES})

FetchContent_MakeAvailable(${external})

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
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -Wextra -Werror -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -fsanitize=thread")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g3 -Wall -Wextra -Werror -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -fsanitize=address -fsanitize-recover=address")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -Wextra -Werror -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -fsanitize=undefined")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g3 -Wall -Wextra -Werror -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -Wpadded")
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
    source/msceqf/filter/initializer/static_initializer.cpp
    source/msceqf/vision/camera.cpp
    source/msceqf/vision/tracker.cpp
    source/msceqf/vision/track_manager.cpp
)

## Declare a C++ library
add_library(${PROJECT_NAME} STATIC ${lib_sources})
target_link_libraries(${PROJECT_NAME} ${libs})

## Define includes
list(APPEND include_dirs ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${include_dirs})

## Declare C++ tests
if(${BUILD_TESTS})
    add_executable(msceqf_tests tests/tests.cpp)
    target_include_directories(msceqf_tests PRIVATE ${include_dirs})
    target_link_libraries(msceqf_tests ${PROJECT_NAME} GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(msceqf_tests)
endif()