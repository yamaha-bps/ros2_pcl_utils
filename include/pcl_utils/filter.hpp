// Copyright 2020 Yamaha Motor Corporation, USA

#ifndef PCL_UTILS__FILTER_HPP_
#define PCL_UTILS__FILTER_HPP_

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <sophus/se3.hpp>

#include <unordered_set>

namespace cbr
{

/**
 * @brief Camera projection with OpenCV model
 *
 * https://docs.opencv.org/4.2.0/d9/d0c/group__calib3d.html
 *
 * @tparam T scalar data type
 * @param pt_CAM 3D point in camera frame
 * @param cam camera information
 * @return Eigen::Matrix<T, 2, 1> pixel coordinates
 */
template<typename T>
Eigen::Matrix<T, 2, 1> cameraProject(
  const Eigen::Matrix<T, 3, 1> & pt_CAM,
  const sensor_msgs::msg::CameraInfo & cam)
{
  const T xp = pt_CAM.x() / pt_CAM.z();
  const T yp = pt_CAM.y() / pt_CAM.z();

  const T r2 = xp * xp + yp * yp;
  const T r4 = r2 * r2;
  const T ratio = 1. + cam.d[0] * r2 + cam.d[1] * r4 + cam.d[4] * (r2 * r4);
  const T xpp =
    xp * ratio + 2 * cam.d[2] * xp * yp + cam.d[3] * (r2 + 2 * xp * xp);
  const T ypp =
    yp * ratio + cam.d[2] * (r2 + 2 * yp * yp) + 2 * cam.d[3] * xp * yp;

  return Eigen::Matrix<T, 2, 1>(
    cam.k[0] * xpp + cam.k[2],
    cam.k[4] * ypp + cam.k[5]);
}

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
  const sensor_msgs::msg::CameraInfo & calib,
  const std::unordered_set<uint8_t> & good_idx,
  const Sophus::SE3f & P_CL, sensor_msgs::msg::PointCloud2 & pcl);

}  // namespace cbr

#endif  // PCL_UTILS__FILTER_HPP_
