# Copyright Yamaha 2021
# MIT License
# https://github.com/yamaha-bps/ros2_pcl_utils/blob/master/LICENSE

cmake_minimum_required(VERSION 3.15)
project(pcl_utils VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED imgproc)

# ---------------------------------------------------------------------------------------
# TARGETS
# ---------------------------------------------------------------------------------------

add_library(${PROJECT_NAME} SHARED
  src/filter.cpp
  src/feature.cpp
)
target_include_directories(${PROJECT_NAME}
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
PUBLIC
  Eigen3::Eigen
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
)
target_compile_options(${PROJECT_NAME}
PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

add_library(${PROJECT_NAME}_nodes SHARED
  src/pcl_image_overlay_node.cpp
  src/pcl_seg_node.cpp
  src/pcl_feature_node.cpp
)
target_include_directories(${PROJECT_NAME}_nodes
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}_nodes
PRIVATE
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${OpenCV_LIBRARIES}
  ${PROJECT_NAME}
)
target_compile_options(${PROJECT_NAME}_nodes
PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN      "cbr::PclSegNode"
  EXECUTABLE  seg_node
)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN      "cbr::PclImageOverlayNode"
  EXECUTABLE  image_overlay_node
)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN      "cbr::PclFeatureNode"
  EXECUTABLE  feature_node
)

# ---------------------------------------------------------------------------------------
# INSTALLATION
# ---------------------------------------------------------------------------------------

install(
  TARGETS               ${PROJECT_NAME} ${PROJECT_NAME}_nodes
  EXPORT                ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION   lib
  LIBRARY DESTINATION   lib
)

install(
  DIRECTORY     include
)

install(
  DIRECTORY     launch gazebo
  DESTINATION   share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------------------
# TESTING
# ---------------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_compile_options(-Wall -Wextra -Wpedantic -Werror)

  ament_add_gtest(test_filter test/test_filter.cpp)
  target_link_libraries(test_filter ${PROJECT_NAME})

  ament_add_gtest(test_feature test/test_feature.cpp)
  target_link_libraries(test_feature ${PROJECT_NAME})

  find_package(launch_testing_ament_cmake)
  add_launch_test(test/bringup.test.py TIMEOUT 60)

  target_link_libraries(test_feature ${PROJECT_NAME})
endif()

# ---------------------------------------------------------------------------------------
# CMAKE CONFIGURATION
# ---------------------------------------------------------------------------------------

ament_export_targets(${PROJECT_NAME}_targets)

ament_package()

