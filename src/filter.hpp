// Copyright 2020 Yamaha Motor Corporation, USA

#ifndef FILTER_HPP_
#define FILTER_HPP_

#include <bps_msgs/msg/mono_calibration.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <sophus/se3.hpp>

#include <unordered_set>

namespace bps
{

/**
 * @brief Filter a pointcloud with respect to an image
 *
 * @param[in] img a segmentation image of type mono8
 * @param calib calibration of the camera
 * @param good_idx the image indices to preserve in the filtered point cloud
 * @param[in, out] pcl incoming pointcloud
 */
void filter(
  const sensor_msgs::msg::Image & img,
  const bps_msgs::msg::MonoCalibration & calib,
  const std::unordered_set<uint8_t> & good_idx,
  const Sophus::SE3f & P_CL,
  sensor_msgs::msg::PointCloud2 & pcl
);

}  // namespace bps

#endif  // FILTER_HPP_
