// Copyright 2020 Yamaha Motor Corporation, USA

#include "pcl_utils/filter.hpp"

#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <unordered_set>
#include <vector>

namespace cbr
{

void filter(
  const sensor_msgs::msg::Image & img,
  const sensor_msgs::msg::CameraInfo & cam,
  const std::unordered_set<uint8_t> & good_labels,
  const Eigen::Vector3f & T_CL,
  const Eigen::Quaternionf & R_CL,
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

  // loop through the pointcloud and keep points that either have negative intensity
  // or that project to pixels with "good labels"
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

    const Eigen::Vector3f pt_C = R_CL * pt_L + T_CL;
    if (pt_C.z() <= 0) {
      continue;  // behind camera
    }

    const Eigen::Vector2f uv = cameraProject(pt_C, cam);

    // check closest pixel
    const auto x = std::lround(uv(0));
    const auto y = std::lround(uv(1));

    if (x < 0 || x >= img.width || y < 0 || y >= img.height) {
      // lidar point not visible in image, do not copy
      continue;
    }

    // check label,
    const auto px_label = img.data[y * img.step + x];
    bool keep_point{false};
    for (const auto idx : good_labels) {
      if (idx == px_label) {
        keep_point = true;
        break;
      }
    }

    if (keep_point) {
      first = std::copy(it, it + pcl.point_step, first);
    }
  }

  // discard remaining points
  pcl.data.resize(std::distance(pcl.data.begin(), first));
  pcl.width = pcl.data.size() / pcl.point_step;
}

}  // namespace cbr
