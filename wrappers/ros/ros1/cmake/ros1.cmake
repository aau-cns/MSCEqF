find_package(catkin REQUIRED COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs image_transport cv_bridge)

catkin_package(
  INCLUDE_DIRS ${include_dirs} wrappers/ros/ros1/include
  CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs image_transport cv_bridge
  LIBRARIES ${PROJECT_NAME}_lib
)

list(APPEND lib_sources
        wrappers/ros/ros1/source/msceqf_ros.cpp
)

add_library(${PROJECT_NAME}_lib STATIC ${lib_sources})
target_link_libraries(${PROJECT_NAME}_lib ${libs})
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${catkin_INCLUDE_DIRS} ${include_dirs} wrappers/ros/ros1/include)
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY ${include_dirs} wrappers/ros/ros1/include
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

add_executable(msceqf_ros1 wrappers/ros/ros1/source/msceqf_ros_node.cpp)
target_link_libraries(msceqf_ros1 ${PROJECT_NAME}_lib ${catkin_LIBRARIES})

add_executable(msceqf_ros1_serial wrappers/ros/ros1/source/msceqf_ros_serial.cpp)
target_link_libraries(msceqf_ros1_serial ${PROJECT_NAME}_lib ${catkin_LIBRARIES})