// Copyright 2020 Yamaha Motor Corporation, USA
#ifndef BPS_PCL_UTILS__PCL_SEG_COMPONENT_HPP_
#define BPS_PCL_UTILS__PCL_SEG_COMPONENT_HPP_


#include <boost/circular_buffer.hpp>

#include <bps_msgs/msg/stereo_calibration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

namespace bps
{

class PclSegComponent : public rclcpp::Node
{
public:
  explicit PclSegComponent(const rclcpp::NodeOptions & opts);
  ~PclSegComponent();

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;
  void cb_pcl_(sensor_msgs::msg::PointCloud2::UniquePtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  void cb_img_(sensor_msgs::msg::Image::UniquePtr msg);

  rclcpp::Subscription<bps_msgs::msg::MonoCalibration>::SharedPtr sub_calib_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_;

  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buf_;

  void work();
  std::thread worker_thread_;
  std::atomic<bool> canceled_{false};

  struct Impl;
  std::unique_ptr<Impl> pImpl;
};

}  // namespace bps

#endif  // BPS_PCL_UTILS__PCL_SEG_COMPONENT_HPP_
