// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/ros2_pcl_utils/blob/master/LICENSE

#ifndef PCL_UTILS__PCL_IMAGE_OVERLAY_NODE_HPP_
#define PCL_UTILS__PCL_IMAGE_OVERLAY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <utility>

#include "feature.hpp"

namespace cbr
{

class PclImageOverlayNode : public rclcpp::Node
{
public:
  explicit PclImageOverlayNode(const rclcpp::NodeOptions & opts);
  ~PclImageOverlayNode();

private:
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_calib_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  void cb_img_(sensor_msgs::msg::Image::UniquePtr);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;
  void cb_pcl_(sensor_msgs::msg::PointCloud2::UniquePtr);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_overlay_;

  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buf_;

  struct Impl;
  std::unique_ptr<Impl> pImpl;
};

}  // namespace cbr

#endif  // PCL_UTILS__PCL_IMAGE_OVERLAY_NODE_HPP_
