// Copyright 2020 Yamaha Motor Corporation, USA

#include <gtest/gtest.h>

#include <sensor_msgs/image_encodings.hpp>

#include <unordered_set>

#include "pcl_utils/filter.hpp"

class ProjectionTest : public ::testing::Test
{
protected:
  void SetUp()
  {
    calib.k.fill(0);
    calib.k[0] = 1;      // fx
    calib.k[2] = 320.5;  // cx
    calib.k[4] = 1;      // fy
    calib.k[5] = 240.5;  // cy

    calib.d.resize(5);
    calib.d[0] = 0;
    calib.d[1] = 0;
    calib.d[2] = 0;
    calib.d[3] = 0;
    calib.d[4] = 0;

    img.encoding = sensor_msgs::image_encodings::MONO8;
    img.height = 480;
    img.width = 640;
    img.step = 640;
    img.data.resize(img.height * img.step);

    for (int h = 0; h != 480; ++h) {
      for (int w = 0; w != 640; ++w) {
        if (w > 320.5 && h > 240.5) {
          img.data[h * img.step + w] = 1;  // first quadrant
        } else if (w < 320.5 && h > 240.5) {
          img.data[h * img.step + w] = 2;  // second quadrant
        } else if (w < 320.5 && h < 240.5) {
          img.data[h * img.step + w] = 3;  // third quadrant
        } else if (w > 320.5 && h < 240.5) {
          img.data[h * img.step + w] = 4;  // fourth quadrant
        }
      }
    }

    pcl.fields.resize(3);
    pcl.fields[0].name = "x";
    pcl.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl.fields[0].count = 1;
    pcl.fields[0].offset = 0;

    pcl.fields[1].name = "y";
    pcl.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl.fields[1].count = 1;
    pcl.fields[1].offset = 4;

    pcl.fields[2].name = "z";
    pcl.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl.fields[2].count = 1;
    pcl.fields[2].offset = 8;

    pcl.point_step = 16;

    // add four points
    pcl.data.resize(4 * pcl.point_step);

    float * x1 =
      reinterpret_cast<float *>(pcl.data.data() + 0 * pcl.point_step + 0);
    float * y1 =
      reinterpret_cast<float *>(pcl.data.data() + 0 * pcl.point_step + 4);
    float * z1 =
      reinterpret_cast<float *>(pcl.data.data() + 0 * pcl.point_step + 8);

    float * x2 =
      reinterpret_cast<float *>(pcl.data.data() + 1 * pcl.point_step + 0);
    float * y2 =
      reinterpret_cast<float *>(pcl.data.data() + 1 * pcl.point_step + 4);
    float * z2 =
      reinterpret_cast<float *>(pcl.data.data() + 1 * pcl.point_step + 8);

    float * x3 =
      reinterpret_cast<float *>(pcl.data.data() + 2 * pcl.point_step + 0);
    float * y3 =
      reinterpret_cast<float *>(pcl.data.data() + 2 * pcl.point_step + 4);
    float * z3 =
      reinterpret_cast<float *>(pcl.data.data() + 2 * pcl.point_step + 8);

    float * x4 =
      reinterpret_cast<float *>(pcl.data.data() + 3 * pcl.point_step + 0);
    float * y4 =
      reinterpret_cast<float *>(pcl.data.data() + 3 * pcl.point_step + 4);
    float * z4 =
      reinterpret_cast<float *>(pcl.data.data() + 3 * pcl.point_step + 8);

    *x1 = 1;
    *y1 = 1;
    *z1 = 1;

    *x2 = 1;
    *y2 = -1;
    *z2 = 1;

    *x3 = -1;
    *y3 = 1;
    *z3 = 1;

    *x4 = -1;
    *y4 = -1;
    *z4 = 1;
  }

  void TearDown() {}

  sensor_msgs::msg::CameraInfo calib;
  sensor_msgs::msg::Image img;
  sensor_msgs::msg::PointCloud2 pcl;
};

TEST_F(ProjectionTest, FirstQuadrant) {
  cbr::filter(img, calib, std::unordered_set<uint8_t>{1}, Sophus::SE3f{}, pcl);
  ASSERT_EQ(pcl.data.size(), pcl.point_step);
  ASSERT_FLOAT_EQ(*reinterpret_cast<float *>(pcl.data.data() + 0), 1);
  ASSERT_FLOAT_EQ(*reinterpret_cast<float *>(pcl.data.data() + 4), 1);
  ASSERT_FLOAT_EQ(*reinterpret_cast<float *>(pcl.data.data() + 8), 1);
}

TEST_F(ProjectionTest, SecondQuadrant) {
  cbr::filter(img, calib, std::unordered_set<uint8_t>{2}, Sophus::SE3f{}, pcl);
  ASSERT_EQ(pcl.data.size(), pcl.point_step);
  ASSERT_FLOAT_EQ(*reinterpret_cast<float *>(pcl.data.data() + 0), -1);
  ASSERT_FLOAT_EQ(*reinterpret_cast<float *>(pcl.data.data() + 4), 1);
  ASSERT_FLOAT_EQ(*reinterpret_cast<float *>(pcl.data.data() + 8), 1);
}

TEST_F(ProjectionTest, ThirdQuadrant) {
  cbr::filter(img, calib, std::unordered_set<uint8_t>{3}, Sophus::SE3f{}, pcl);
  ASSERT_EQ(pcl.data.size(), pcl.point_step);
  ASSERT_FLOAT_EQ(*reinterpret_cast<float *>(pcl.data.data() + 0), -1);
  ASSERT_FLOAT_EQ(*reinterpret_cast<float *>(pcl.data.data() + 4), -1);
  ASSERT_FLOAT_EQ(*reinterpret_cast<float *>(pcl.data.data() + 8), 1);
}

TEST_F(ProjectionTest, FourthQuadrant) {
  cbr::filter(img, calib, std::unordered_set<uint8_t>{4}, Sophus::SE3f{}, pcl);
  ASSERT_EQ(pcl.data.size(), pcl.point_step);
  ASSERT_FLOAT_EQ(*reinterpret_cast<float *>(pcl.data.data() + 0), 1);
  ASSERT_FLOAT_EQ(*reinterpret_cast<float *>(pcl.data.data() + 4), -1);
  ASSERT_FLOAT_EQ(*reinterpret_cast<float *>(pcl.data.data() + 8), 1);
}

TEST_F(ProjectionTest, AllQuadrants) {
  auto copy = pcl.data;
  cbr::filter(
    img, calib, std::unordered_set<uint8_t>{1, 2, 3, 4},
    Sophus::SE3f{}, pcl);

  ASSERT_EQ(copy.size(), pcl.data.size());

  for (int i = 0u; i != copy.size(); ++i) {
    ASSERT_EQ(copy[i], pcl.data[i]);
  }
}
