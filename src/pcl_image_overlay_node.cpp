// Copyright 2020 Yamaha Motor Corporation, USA

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "pcl_utils/pcl_image_overlay_component.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto opts = rclcpp::NodeOptions{}.use_intra_process_comms(false);
  auto node = std::make_shared<cbr::PclImageOverlayComponent>(opts);

  rclcpp::executors::SingleThreadedExecutor exec{};
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
