// Copyright 2020 Yamaha Motor Corporation, USA

#include "bps_pcl_utils/pcl_feature_component.hpp"

#include <boost/fusion/adapted/struct/adapt_struct.hpp>
#include <bps_library/parameter.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <utility>


BOOST_FUSION_ADAPT_STRUCT(
  bps::PclFeatureParams,
  window, ang_disc_thresh, rel_disc_thresh,
  ang_inc_thresh, max_angle, min_depth, max_depth, min_intensity, max_intensity,
  plane_thresh, edge_thresh
)

namespace bps
{

PclFeatureComponent::PclFeatureComponent(const rclcpp::NodeOptions & opts)
: Node("pcl_feature", opts)
{
  initParamsInNode(*this, prm_);

  sub_pcl_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud",
    rclcpp::SensorDataQoS(), std::bind(&PclFeatureComponent::cb_pcl_, this, std::placeholders::_1));
  pub_edge_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "feature/edge", rclcpp::SensorDataQoS());
  pub_plane_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "feature/plane", rclcpp::SensorDataQoS());
}

PclFeatureComponent::~PclFeatureComponent()
{}

void PclFeatureComponent::cb_pcl_(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  auto[edges, planar] = pcl_features(*msg, prm_);

  if (edges) {
    pub_edge_->publish(std::move(edges));
  }
  if (planar) {
    pub_plane_->publish(std::move(planar));
  }
}

}  // namespace bps

RCLCPP_COMPONENTS_REGISTER_NODE(bps::PclFeatureComponent)
