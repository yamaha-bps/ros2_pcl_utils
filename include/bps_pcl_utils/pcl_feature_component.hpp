// Copyright 2020 Yamaha Motor Corporation, USA
#ifndef BPS_PCL_UTILS__PCL_FEATURE_COMPONENT_HPP_
#define BPS_PCL_UTILS__PCL_FEATURE_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <utility>

#include "feature.hpp"

namespace bps
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

}  // namespace bps

#endif  // BPS_PCL_UTILS__PCL_FEATURE_COMPONENT_HPP_
