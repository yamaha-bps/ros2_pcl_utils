// Copyright 2020 Yamaha Motor Corporation, USA

#include "filter.hpp"

#include <opencv2/calib3d.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <unordered_set>
#include <vector>

namespace bps
{

void filter(
  const sensor_msgs::msg::Image & img,
  const bps_msgs::msg::MonoCalibration & calib,
  const std::unordered_set<uint8_t> & good_idx,
  const Sophus::SE3f & P_CL,
  sensor_msgs::msg::PointCloud2 & pcl
)
{
  if (img.encoding != sensor_msgs::image_encodings::MONO8) {
    std::cerr << "Error: Image is not uint8_t" << std::endl;
    return;
  }

  // check that pointcloud begins with x, y, z
  if (pcl.fields.size() < 3 || pcl.fields[0].name != "x" ||
    pcl.fields[1].name != "y" || pcl.fields[2].name != "z")
  {
    std::cerr << "Error: Pointcloud does not have expected xyz fields" << std::endl;
    return;
  }

  if (pcl.fields[0].datatype != sensor_msgs::msg::PointField::FLOAT32 ||
    pcl.fields[1].datatype != sensor_msgs::msg::PointField::FLOAT32 ||
    pcl.fields[2].datatype != sensor_msgs::msg::PointField::FLOAT32)
  {
    std::cerr << "Error: Only float32 supported" << std::endl;
    return;
  }

  std::size_t intensity_idx = std::numeric_limits<std::size_t>::max();
  auto it = std::find_if(
    pcl.fields.begin(), pcl.fields.end(), [](const sensor_msgs::msg::PointField & field) {
      return field.name == "intensity";
    });
  if (it != pcl.fields.end()) {
    if (it->datatype != sensor_msgs::msg::PointField::FLOAT32) {
      std::cerr << "Error: Only float32 supported" << std::endl;
      return;
    }
    intensity_idx = std::distance(pcl.fields.begin(), it);
  }

  cv::Mat CM = (cv::Mat_<double>(3, 3) << calib.fx, 0, calib.cx, 0, calib.fy, calib.cy, 0, 0, 1);
  cv::Mat D = (cv::Mat_<double>(1, 5) << calib.k1, calib.k2, calib.p1, calib.p2, calib.k3);

  // loop through the pointcloud and keep points that either have negative intensity
  // or that project to "good pixels"
  auto first = pcl.data.begin();  // inserting iterator
  for (auto it = first; it < pcl.data.end(); it += pcl.point_step) {
    uint8_t * ptr = &*it;

    // add points with negative intensity (misses)
    if (intensity_idx < std::numeric_limits<std::size_t>::max()) {
      float intensity = *reinterpret_cast<float *>(ptr + pcl.fields[intensity_idx].offset);
      if (intensity < 0) {
        first = std::copy(it, it + pcl.point_step, first);
        continue;
      }
    }

    // point in lidar frame
    const Eigen::Vector3f pt_L{
      *reinterpret_cast<float *>(ptr + pcl.fields[0].offset),
      *reinterpret_cast<float *>(ptr + pcl.fields[1].offset),
      *reinterpret_cast<float *>(ptr + pcl.fields[2].offset)
    };
    // transform to camera frame
    const Eigen::Vector3f pt_C = P_CL * pt_L;

    // project into image plane
    std::vector<cv::Point2f> out_pt;
    cv::projectPoints(
      std::vector<cv::Point3f>{cv::Point3f{pt_C.x(), pt_C.y(), pt_C.z()}},
      cv::Vec3f::zeros(),
      cv::Vec3f::zeros(),
      CM,
      D,
      out_pt
    );

    // check that all bounding pixels are "good pixels"
    const auto xdn = std::lround(std::floor(out_pt[0].x));
    const auto xup = std::lround(std::ceil(out_pt[0].x));

    const auto ydn = std::lround(std::floor(out_pt[0].y));
    const auto yup = std::lround(std::ceil(out_pt[0].y));

    if (xdn < 0 || xup >= img.width || ydn < 0 || yup >= img.height) {
      // lidar point not visible in image, do not copy
      continue;
    }

    // check all four surrounding pixels, do not copy if one is bad
    if (good_idx.find(img.data[ydn * img.step + xdn]) == good_idx.end()) {
      continue;
    }
    if (good_idx.find(img.data[ydn * img.step + xup]) == good_idx.end()) {
      continue;
    }
    if (good_idx.find(img.data[yup * img.step + xdn]) == good_idx.end()) {
      continue;
    }
    if (good_idx.find(img.data[yup * img.step + xup]) == good_idx.end()) {
      continue;
    }

    // all pixels are in the good set, keep it!
    first = std::copy(it, it + pcl.point_step, first);
  }

  // discard remaining points
  pcl.data.resize(std::distance(pcl.data.begin(), first));
  pcl.width = pcl.data.size() / pcl.point_step;
}

}  // namespace bps
