find_package(catkin REQUIRED COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs image_transport cv_bridge)

catkin_package(
  INCLUDE_DIRS wrappers/ros/ros1/include
  CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs image_transport cv_bridge
  LIBRARIES ${PROJECT_NAME}_lib
)

list(APPEND ros1_lib_sources
        wrappers/ros/ros1/source/msceqf_ros.cpp
)

add_library(${PROJECT_NAME}_ros1_lib STATIC ${ros1_lib_sources})
target_link_libraries(${PROJECT_NAME}_ros1_lib ${PROJECT_NAME}_lib)
target_include_directories(${PROJECT_NAME}_ros1_lib PUBLIC ${include_dirs} ${catkin_INCLUDE_DIRS} wrappers/ros/ros1/include)

add_executable(msceqf_ros1 wrappers/ros/ros1/source/msceqf_ros_node.cpp)
target_link_libraries(msceqf_ros1 ${PROJECT_NAME}_ros1_lib ${catkin_LIBRARIES})