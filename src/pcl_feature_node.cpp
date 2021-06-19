// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/ros2_pcl_utils/blob/master/LICENSE

#include "pcl_utils/pcl_feature_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <utility>

namespace cbr
{

PclFeatureNode::PclFeatureNode(const rclcpp::NodeOptions & opts)
: Node("pcl_feature", opts)
{
  declare_parameter<int>("window", prm_.window);
  declare_parameter<double>("ang_disc_thresh", prm_.ang_disc_thresh);
  declare_parameter<double>("rel_disc_thresh", prm_.rel_disc_thresh);
  declare_parameter<double>("ang_inc_thresh", prm_.ang_inc_thresh);
  declare_parameter<double>("max_angle", prm_.max_angle);
  declare_parameter<double>("min_depth", prm_.min_depth);
  declare_parameter<double>("max_depth", prm_.max_depth);
  declare_parameter<double>("min_intensity", prm_.min_intensity);
  declare_parameter<double>("max_intensity", prm_.max_intensity);
  declare_parameter<double>("plane_thresh", prm_.plane_thresh);
  declare_parameter<double>("edge_thresh", prm_.edge_thresh);

  prm_.window = get_parameter("window").as_int();
  prm_.ang_disc_thresh = get_parameter("ang_disc_thresh").as_double();
  prm_.rel_disc_thresh = get_parameter("rel_disc_thresh").as_double();
  prm_.ang_inc_thresh = get_parameter("ang_inc_thresh").as_double();
  prm_.max_angle = get_parameter("max_angle").as_double();
  prm_.min_depth = get_parameter("min_depth").as_double();
  prm_.max_depth = get_parameter("max_depth").as_double();
  prm_.min_intensity = get_parameter("min_intensity").as_double();
  prm_.plane_thresh = get_parameter("plane_thresh").as_double();
  prm_.edge_thresh = get_parameter("edge_thresh").as_double();

  sub_pcl_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&PclFeatureNode::cb_pcl_, this, std::placeholders::_1));
  pub_edge_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "feature/edge", rclcpp::SensorDataQoS());
  pub_plane_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "feature/plane", rclcpp::SensorDataQoS());

  RCLCPP_INFO(get_logger(), "Started node");
}

PclFeatureNode::~PclFeatureNode()
{
  RCLCPP_INFO(get_logger(), "Closing node");
}

void PclFeatureNode::cb_pcl_(
  sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  auto [edges, planar] = pcl_features(*msg, prm_);

  if (edges) {
    pub_edge_->publish(std::move(edges));
  }
  if (planar) {
    pub_plane_->publish(std::move(planar));
  }
}

}  // namespace cbr

RCLCPP_COMPONENTS_REGISTER_NODE(cbr::PclFeatureNode)
