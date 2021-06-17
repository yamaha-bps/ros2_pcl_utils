// Copyright 2020 Yamaha Motor Corporation, USA

#include <gtest/gtest.h>

#include "pcl_utils/feature.hpp"

TEST(PclFeature, Corner)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.fields.resize(4);
  msg.fields[0].name = "x";
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].offset = 0;
  msg.fields[1].name = "y";
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].offset = 4;
  msg.fields[2].name = "z";
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].offset = 8;
  msg.fields[3].name = "intensity";
  msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[3].offset = 12;

  msg.point_step = 16;

  for (double y = 0; y < 1; y += 0.01) {
    msg.data.resize(msg.data.size() + msg.point_step);
    float * xp = reinterpret_cast<float *>(msg.data.data() + msg.data.size() - 16);
    float * yp = reinterpret_cast<float *>(msg.data.data() + msg.data.size() - 12);
    float * zp = reinterpret_cast<float *>(msg.data.data() + msg.data.size() - 8);
    float * ip = reinterpret_cast<float *>(msg.data.data() + msg.data.size() - 4);

    *xp = 0;
    *yp = y;
    *zp = 1;
    *ip = 0.4;
  }
  uint32_t half_size = msg.data.size() / msg.point_step;
  for (double x = 0; x < 1; x += 0.01) {
    msg.data.resize(msg.data.size() + msg.point_step);
    float * xp = reinterpret_cast<float *>(msg.data.data() + msg.data.size() - 16);
    float * yp = reinterpret_cast<float *>(msg.data.data() + msg.data.size() - 12);
    float * zp = reinterpret_cast<float *>(msg.data.data() + msg.data.size() - 8);
    float * ip = reinterpret_cast<float *>(msg.data.data() + msg.data.size() - 4);

    *xp = x;
    *yp = 1;
    *zp = 1;
    *ip = 0.4;
  }

  msg.height = 1;
  msg.width = msg.data.size() / msg.point_step;

  cbr::PclFeatureParams prm;
  prm.window = 5;
  prm.ang_disc_thresh = M_PI;
  prm.rel_disc_thresh = 0.1;
  prm.ang_inc_thresh = M_PI_2;
  prm.max_angle = M_PI;
  prm.min_intensity = 0.01;
  prm.max_intensity = 1;
  prm.plane_thresh = 2e-4;
  prm.edge_thresh = 1e-2;

  // on regular cloud
  {
    auto [edges, planar] = cbr::pcl_features(msg, prm);
    ASSERT_EQ(edges->width, 1u);
    ASSERT_GE(planar->width, 5u);
    for (auto i = 0u; i != 16; ++i) {
      ASSERT_EQ(edges->data[i], msg.data[half_size * msg.point_step + i]);
    }
  }

  // on reversed cloud
  {
    auto msg_rev = msg;
    for (auto i = 0u; i != msg.width; ++i) {
      std::copy(
        msg.data.begin() + i * msg.point_step,
        msg.data.begin() + (i + 1) * msg.point_step,
        msg_rev.data.begin() + msg.data.size() - (1 + i) * msg.point_step
      );
    }
    auto [edges, planar] = cbr::pcl_features(msg_rev, prm);
    ASSERT_EQ(edges->width, 1u);
    ASSERT_GE(planar->width, 5u);
    for (auto i = 0u; i != 16; ++i) {
      ASSERT_EQ(edges->data[i], msg.data[half_size * msg.point_step + i]);
    }
  }
}
