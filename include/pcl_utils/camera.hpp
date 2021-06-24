// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/ros2_pcl_utils/blob/master/LICENSE

#ifndef PCL_UTILS__CAMERA_HPP_
#define PCL_UTILS__CAMERA_HPP_

#include <Eigen/Core>
#include <sensor_msgs/msg/camera_info.hpp>

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

#endif  // PCL_UTILS__CAMERA_HPP_
