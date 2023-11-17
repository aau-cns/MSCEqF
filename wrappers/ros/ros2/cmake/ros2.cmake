find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

list(APPEND lib_sources
        wrappers/ros/ros2/source/msceqf_ros.cpp
)

list(APPEND ament_libraries
        rclcpp
        rosbag2_cpp
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        visualization_msgs
        cv_bridge
        image_transport
)

add_library(${PROJECT_NAME}_lib SHARED ${lib_sources})
ament_target_dependencies(${PROJECT_NAME}_lib ${ament_libraries})
target_link_libraries(${PROJECT_NAME}_lib ${libs})
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${include_dirs} wrappers/ros/ros2/include)
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY ${include_dirs} wrappers/ros/ros2/include
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)

add_executable(msceqf_ros2 wrappers/ros/ros2/source/msceqf_ros_node.cpp)
ament_target_dependencies(msceqf_ros2 ${ament_libraries})
target_link_libraries(msceqf_ros2 ${PROJECT_NAME}_lib)

ament_package()