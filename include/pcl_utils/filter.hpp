// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/ros2_pcl_utils/blob/master/LICENSE

#ifndef PCL_UTILS__FILTER_HPP_
#define PCL_UTILS__FILTER_HPP_

#include <Eigen/Geometry>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <unordered_set>

namespace cbr
{

/**
 * @brief Filter a pointcloud with respect to an image
 *
 * @param[in] img a segmentation image of type mono8
 * @param cam_info calibration info for the camera
 * @param good_idx the image indices to preserve in the filtered point cloud
 * @param T_CF translation from camera to frame
 * @param R_CF rotation from camera to frame
 * @param[in, out] pcl incoming pointcloud
 */
void filter(
  const sensor_msgs::msg::Image & img,
  const sensor_msgs::msg::CameraInfo & cam_info,
  const std::unordered_set<uint8_t> & good_idx,
  const Eigen::Vector3f & T_CF,
  const Eigen::Quaternionf & R_CF,
  sensor_msgs::msg::PointCloud2 & pcl);

}  // namespace cbr

#endif  // PCL_UTILS__FILTER_HPP_
