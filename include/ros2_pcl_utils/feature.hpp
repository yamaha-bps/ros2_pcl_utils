// Copyright 2020 Yamaha Motor Corporation, USA
#ifndef ROS2_PCL_UTILS__FEATURE_HPP_
#define ROS2_PCL_UTILS__FEATURE_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <limits>
#include <utility>

namespace cbr {

struct PclFeatureParams {
  // Size of window for feature detection on each side
  uint32_t window{5};

  // Angular discontinuity threshold [rad] (between 0 and PI)
  // Points in window around a discontinuity are marked as invalid
  // Leave as M_PI if scan is known to be continuous
  float ang_disc_thresh{M_PI};

  // Relative distance discontinuity threshold
  // Points in window on the far side of a relative discontinuity are marked as
  // invalid since they may be occluded by a small sensor movement
  float rel_disc_thresh{std::numeric_limits<float>::max()};

  // Max angle of ray w.r.t. Y-Z plane [rad] (between 0 and PI / 2)
  // A small angle indicates that the surface is facing the lidar
  // Points with high angle angle on both sides are marked as ineligible
  float ang_inc_thresh{M_PI_2};

  // Max angle w.r.t. lidar x axis [rad] (between 0 and PI)
  // Points with larger angle w.r.t. lidar center are marked as ineligible
  float max_angle{M_PI};

  // Max distance from sensor
  float min_depth{0}, max_depth{std::numeric_limits<float>::max()};

  // Intensity range for valid points
  float min_intensity{0.}, max_intensity{std::numeric_limits<float>::max()};

  // Upper cvalue threshold for planar features
  float plane_thresh{2e-4};

  // Lower cvalue threshold for edge features
  float edge_thresh{1e-2};
};

/**
 * @brief Extract edge and plane features from pointcloud
 *
 * Input msg must be from a continuous laser scan
 *
 * Returns nullptr if feature extraction fails
 *
 * @param msg input pointcloud
 * @param prm parameters
 * @return edge and planar feature pointclouds
 */
std::pair<sensor_msgs::msg::PointCloud2::UniquePtr,
          sensor_msgs::msg::PointCloud2::UniquePtr>
pcl_features(const sensor_msgs::msg::PointCloud2 &msg,
             const PclFeatureParams &prm);

} // namespace cbr

#endif // ROS2_PCL_UTILS__FEATURE_HPP_
