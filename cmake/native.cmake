## Declare a C++ library
add_library(${PROJECT_NAME}_lib STATIC ${lib_sources})
target_link_libraries(${PROJECT_NAME}_lib ${libs})

## Declare C++ tests
if(${MSCEQF_TESTS})
    message(STATUS "Building MSCEqF tests")
    add_executable(msceqf_tests tests/tests.cpp)
    target_include_directories(msceqf_tests PRIVATE ${include_dirs})
    target_link_libraries(msceqf_tests ${PROJECT_NAME}_lib GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(msceqf_tests)
endif()

# Declare C++ examples
add_executable(msceqf_euroc examples/euroc/euroc.cpp)
target_include_directories(msceqf_euroc PRIVATE ${include_dirs})
target_link_libraries(msceqf_euroc ${PROJECT_NAME}_lib pthread)

add_executable(msceqf_uzhfpv examples/uzhfpv/uzhfpv.cpp)
target_include_directories(msceqf_uzhfpv PRIVATE ${include_dirs})
target_link_libraries(msceqf_uzhfpv ${PROJECT_NAME}_lib pthread)

add_executable(msceqf_synthetic examples/synthetic/synthetic.cpp)
target_include_directories(msceqf_synthetic PRIVATE ${include_dirs})
target_link_libraries(msceqf_synthetic ${PROJECT_NAME}_lib pthread)

add_executable(msceqf_synthetic_images examples/synthetic_images/synthetic_images.cpp)
target_include_directories(msceqf_synthetic_images PRIVATE ${include_dirs})
target_link_libraries(msceqf_synthetic_images ${PROJECT_NAME}_lib pthread)