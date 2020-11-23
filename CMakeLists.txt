cmake_minimum_required(VERSION 3.15)
project(pcl_seg)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(bps_library REQUIRED)
find_package(bps_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV 4.2 REQUIRED)

add_library(pcl_seg_component SHARED
  src/filter.cpp
  src/pcl_seg_component.cpp
)
target_include_directories(pcl_seg_component
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(pcl_seg_component
  bps_library
  bps_msgs
  Eigen3
  rclcpp
  rclcpp_components
  tf2_ros
)
target_link_libraries(pcl_seg_component
  ${OpenCV_LIBRARIES}
)
target_compile_options(pcl_seg_component
PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

rclcpp_components_register_nodes(pcl_seg_component "bps::PclSegComponent")


add_executable(pcl_seg_node
  src/pcl_seg_node.cpp
)
target_link_libraries(pcl_seg_node
PRIVATE
  pcl_seg_component
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_filter
    test/test_filter.cpp
  )
  target_include_directories(test_filter
  PRIVATE
    src/
  )
  target_link_libraries(test_filter
    pcl_seg_component
  )
endif()


# Libraries
install(TARGETS
  pcl_seg_component
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

# To run with ros2 run
install(TARGETS
  pcl_seg_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install directiories
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
