// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/ros2_pcl_utils/blob/master/LICENSE

#ifndef PCL_ITERATOR_HPP_
#define PCL_ITERATOR_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Core>

#include <cstdint>


struct PclIterator
{
  inline PclIterator(uint32_t pt_step, uint8_t const * data)
  : pt_step_(pt_step), data_ptr_(data)
  {}

  explicit inline PclIterator(const sensor_msgs::msg::PointCloud2 & msg)
  : pt_step_(msg.point_step),
    data_ptr_(msg.data.data())
  {
    if (msg.fields[0].name != "x" || msg.fields[0].offset != 0 ||
      msg.fields[0].datatype != sensor_msgs::msg::PointField::FLOAT32)
    {
      throw std::runtime_error("x field not proper");
    }
    if (msg.fields[1].name != "y" || msg.fields[1].offset != 4 ||
      msg.fields[1].datatype != sensor_msgs::msg::PointField::FLOAT32)
    {
      throw std::runtime_error("y field not proper");
    }
    if (msg.fields[2].name != "z" || msg.fields[2].offset != 8 ||
      msg.fields[2].datatype != sensor_msgs::msg::PointField::FLOAT32)
    {
      throw std::runtime_error("z field not proper");
    }
    if (msg.fields[3].name != "intensity" || msg.fields[3].offset != 12 ||
      msg.fields[3].datatype != sensor_msgs::msg::PointField::FLOAT32)
    {
      throw std::runtime_error("intensity field not proper");
    }
  }

  inline Eigen::Map<const Eigen::Vector3f> operator*() const
  {
    return Eigen::Map<const Eigen::Vector3f>(reinterpret_cast<const float *>(data_ptr_));
  }

  inline float intensity() const
  {
    return *reinterpret_cast<const float *>(data_ptr_ + 12);
  }

  inline PclIterator & operator++()
  {
    data_ptr_ += pt_step_;
    return *this;
  }

  inline PclIterator & operator+=(std::size_t i)
  {
    data_ptr_ += i * pt_step_;
    return *this;
  }

  inline PclIterator operator+(std::size_t i) const
  {
    return PclIterator(pt_step_, data_ptr_ + i * pt_step_);
  }

  uint32_t pt_step_;
  uint8_t const * data_ptr_;
};

#endif  // PCL_ITERATOR_HPP_
