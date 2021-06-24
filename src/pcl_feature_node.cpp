// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/ros2_pcl_utils/blob/master/LICENSE

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "pcl_utils/pcl_feature_component.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto opts = rclcpp::NodeOptions{}.use_intra_process_comms(false);
  auto node = std::make_shared<cbr::PclFeatureComponent>(opts);

  rclcpp::executors::SingleThreadedExecutor exec{};
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
