cmake_minimum_required(VERSION 3.15)
project(pcl_utils VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ---------------------------------------------------------------------------------------
# TARGETS
# ---------------------------------------------------------------------------------------

# Library

add_library(pcl_utils SHARED
  src/filter.cpp
  src/feature.cpp
)
target_include_directories(pcl_utils
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${sensor_msgs_INCLUDE_DIRS}
)
target_link_libraries(pcl_utils
PUBLIC
  Sophus::Sophus
  Eigen3::Eigen
  ${sensor_msgs_LIBRARIES}
)
target_compile_options(pcl_utils
PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

# Components

add_library(seg_component SHARED
  src/pcl_seg_component.cpp
)
target_include_directories(seg_component
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(seg_component
  rclcpp
  rclcpp_components
  tf2_ros
)
target_link_libraries(seg_component
  pcl_utils
)
target_compile_options(seg_component
PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
rclcpp_components_register_nodes(seg_component "cbr::PclSegComponent")


add_library(feature_component SHARED
  src/feature.cpp
  src/pcl_feature_component.cpp
)
target_include_directories(feature_component
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
)
target_link_libraries(feature_component
PUBLIC
  ${rclcpp_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
PRIVATE
  pcl_utils
)
target_compile_options(feature_component
PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
rclcpp_components_register_nodes(feature_component "cbr::PclFeatureComponent")


add_library(image_overlay_component SHARED
  src/pcl_image_overlay_component.cpp
)
target_include_directories(image_overlay_component
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(image_overlay_component
  tf2_ros
  rclcpp
  rclcpp_components
)
target_link_libraries(image_overlay_component
  ${OpenCV_LIBRARIES}
  pcl_utils
)
target_compile_options(image_overlay_component
PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
rclcpp_components_register_nodes(image_overlay_component "cbr::PclImageOverlayComponent")

# Nodes

add_executable(seg_node
  src/pcl_seg_node.cpp
)
target_link_libraries(seg_node
PRIVATE
  seg_component
)

add_executable(feature_node
  src/pcl_feature_node.cpp
)
target_link_libraries(feature_node
PRIVATE
  feature_component
)

add_executable(image_overlay_node
  src/pcl_image_overlay_node.cpp
)
target_link_libraries(image_overlay_node
PRIVATE
  image_overlay_component
)

# ---------------------------------------------------------------------------------------
# INSTALLATION
# ---------------------------------------------------------------------------------------

# Libraries
install(TARGETS
  pcl_utils seg_component feature_component image_overlay_component
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

# To run with ros2 run
install(TARGETS
  seg_node feature_node image_overlay_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install directiories
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
  target_link_libraries(test_filter pcl_utils)
  ament_add_gtest(test_feature
    test/test_feature.cpp
  )
  target_link_libraries(test_feature pcl_utils)
endif()

# ---------------------------------------------------------------------------------------
# CMAKE CONFIGURATION
# ---------------------------------------------------------------------------------------

ament_package()

