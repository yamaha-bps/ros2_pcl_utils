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
find_package(OpenCV REQUIRED)

# ---------------------------------------------------------------------------------------
# TARGETS
# ---------------------------------------------------------------------------------------

# Library

add_library(${PROJECT_NAME}_lib SHARED
  src/filter.cpp
  src/feature.cpp
)
target_include_directories(${PROJECT_NAME}_lib
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${sensor_msgs_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_lib
PUBLIC
  Eigen3::Eigen
  ${sensor_msgs_LIBRARIES}
)
target_compile_options(${PROJECT_NAME}_lib
PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

# Components

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
)
ament_target_dependencies(${PROJECT_NAME}_nodes
  rclcpp
  rclcpp_components
  tf2_ros
)
target_link_libraries(${PROJECT_NAME}_nodes
  ${OpenCV_LIBRARIES}
  ${PROJECT_NAME}_lib
)
target_compile_options(${PROJECT_NAME}_nodes
PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "cbr::PclSegNode"
  EXECUTABLE seg_node
)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "cbr::PclImageOverlayNode"
  EXECUTABLE image_overlay_node
)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "cbr::PclFeatureNode"
  EXECUTABLE feature_node
)

# ---------------------------------------------------------------------------------------
# INSTALLATION
# ---------------------------------------------------------------------------------------

install(TARGETS
  ${PROJECT_NAME}_lib ${PROJECT_NAME}_nodes
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch gazebo
  DESTINATION share/${PROJECT_NAME}/
)

# ---------------------------------------------------------------------------------------
# TESTING
# ---------------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_filter
    test/test_filter.cpp
  )
  target_link_libraries(test_filter ${PROJECT_NAME}_lib)
  ament_add_gtest(test_feature
    test/test_feature.cpp
  )
  target_link_libraries(test_feature ${PROJECT_NAME}_lib)
endif()

# ---------------------------------------------------------------------------------------
# CMAKE CONFIGURATION
# ---------------------------------------------------------------------------------------

ament_package()

