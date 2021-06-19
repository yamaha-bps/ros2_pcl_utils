// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/ros2_pcl_utils/blob/master/LICENSE

#ifndef PCL_UTILS__PCL_FEATURE_COMPONENT_HPP_
#define PCL_UTILS__PCL_FEATURE_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <utility>

#include "feature.hpp"

namespace cbr
{

class PclFeatureComponent : public rclcpp::Node
{
public:
  explicit PclFeatureComponent(const rclcpp::NodeOptions & opts);
  ~PclFeatureComponent();

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;
  void cb_pcl_(sensor_msgs::msg::PointCloud2::UniquePtr);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_edge_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_plane_;

  PclFeatureParams prm_;
};

}  // namespace cbr

#endif  // PCL_UTILS__PCL_FEATURE_COMPONENT_HPP_
