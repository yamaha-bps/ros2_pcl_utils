// Copyright 2020 Yamaha Motor Corporation, USA

#include "bps_pcl_utils/feature.hpp"

#include <Eigen/Core>

#include <memory>
#include <utility>
#include <vector>

namespace bps
{

struct ScanInfo
{
  uint32_t idx = 0;  // index in scan
  bool ok = false;   // if eligible to be a feature
  float cvalue = 0;  // estimate of smoothness
};


struct PclIterator
{
  PclIterator(uint32_t pt_step, uint8_t const * data)
  : pt_step_(pt_step), data_ptr_(data)
  {}

  explicit PclIterator(const sensor_msgs::msg::PointCloud2 & msg)
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

  Eigen::Map<const Eigen::Vector3f> operator*() const
  {
    return Eigen::Map<const Eigen::Vector3f>(reinterpret_cast<const float *>(data_ptr_));
  }

  float intensity() const
  {
    return *reinterpret_cast<const float *>(data_ptr_ + 12);
  }

  PclIterator & operator++()
  {
    data_ptr_ += pt_step_;
    return *this;
  }

  PclIterator & operator+=(std::size_t i)
  {
    data_ptr_ += i * pt_step_;
    return *this;
  }

  PclIterator operator+(std::size_t i) const
  {
    return PclIterator(pt_step_, data_ptr_ + i * pt_step_);
  }

  uint32_t pt_step_;
  uint8_t const * data_ptr_;
};


std::pair<sensor_msgs::msg::PointCloud2::UniquePtr, sensor_msgs::msg::PointCloud2::UniquePtr>
pcl_features(const sensor_msgs::msg::PointCloud2 & msg, const PclFeatureParams & prm)
{
  const uint32_t N = msg.height * msg.width;
  const uint32_t W = prm.window;

  if (N < 2 * W + 1) {
    return {nullptr, nullptr};  // too few points to process
  }


  //////////////////////////
  // STEP 1: VALID POINTS //
  //////////////////////////

  std::vector<bool> valid(N, false);

  std::vector<bool> ang_discont(N - 1, false);  // angle discontinuity
  std::vector<int8_t> dist_discont(N - 1, 0);   // distance discontinuity
  std::vector<bool> inc_ang_viol(N - 1, 0);     // incidence angle bound violation

  const auto cos_ang_disc_thresh = std::cos(prm.ang_disc_thresh);
  const auto sin_ang_inc_thresh = std::sin(prm.ang_inc_thresh);

  PclIterator it(msg);
  for (auto i = 0u; i != N - 1; ++i, ++it) {
    const auto p0 = *it;
    const auto p1 = *(it + 1);
    const auto r0 = p0.norm();
    const auto r1 = p1.norm();
    const auto r01 = (p1 - p0).norm();

    // incidence angle difference from 90 (perpendicular incidence)
    //  | alpha - pi/2 | <= b
    //  | cos(alpha) | <= sin(b)
    if (std::abs<float>(p0.dot(p1 - p0)) / (r0 * r01) > sin_ang_inc_thresh) {
      inc_ang_viol[i] = true;
    }

    // angular discontinuity
    // cos alpha = dot(p0, p1) / ||p1|| ||p2||
    // |alpha| > b  <==>  cos(alpha) < cos(b) <==>  dot(p0, p1) / (||p1|| ||p2||) < cos(b)
    if (p0.dot(p1) / (r0 * r1) < cos_ang_disc_thresh) {
      ang_discont[i] = true;
    }

    // distance discontinuity
    if (2 * r01 > prm.rel_disc_thresh * (r0 + r1)) {
      if (r0 < r1) {
        dist_discont[i] = -1;  // p1 further away than p0
      } else {
        dist_discont[i] = 1;   // p1 closer than p0
      }
    }
  }

  // sliding window count of number of angular discontinuities
  uint32_t num_ang_discont = 0;
  for (auto i = 0u; i < 2u * W; ++i) {
    num_ang_discont += uint32_t(ang_discont[i]);
  }

  // sliding window count of distance discontinuities
  uint32_t num_dist_discont_neg_left = 0;  // count decreasing on the left
  uint32_t num_dist_discont_pos_rght = 0;  // count increasing on the right
  for (auto i = 0u; i < W; ++i) {
    num_dist_discont_neg_left += uint32_t(dist_discont[i] == -1);
  }
  for (auto i = W; i < 2 * W; ++i) {
    num_dist_discont_pos_rght += uint32_t(dist_discont[i] == 1);
  }

  it = PclIterator(msg) + W;
  for (auto i = W; i + W < N; ++i, ++it) {
    valid[i] = true;

    if (num_dist_discont_neg_left > 0 || num_dist_discont_pos_rght > 0) {
      // distance discontinuity condition (robustly visible)
      valid[i] = false;
    } else if (num_ang_discont > 0) {
      // no angular discontinuities in window
      valid[i] = false;
    } else if (inc_ang_viol[i - 1] && inc_ang_viol[i]) {
      // incidence angle condition
      valid[i] = false;
    } else if (it.intensity() < prm.min_intensity || it.intensity() > prm.max_intensity) {
      // intensity condition
      valid[i] = false;
    } else if (std::atan2((*it).tail<2>().norm(), (*it).x()) > prm.max_angle) {
      // angle condition
      valid[i] = false;
    } else {
      // depth condition
      auto norm = (*it).norm();
      if (norm < prm.min_depth || norm > prm.max_depth) {
        valid[i] = false;
      }
    }

    // update sliding window counters
    num_ang_discont -= ang_discont[i - W];
    num_ang_discont += ang_discont[i + W];
    num_dist_discont_neg_left -= uint32_t(dist_discont[i - W] == -1);
    num_dist_discont_neg_left += uint32_t(dist_discont[i] == -1);
    num_dist_discont_pos_rght -= uint32_t(dist_discont[i] == 1);
    num_dist_discont_pos_rght += uint32_t(dist_discont[i + W] == 1);
  }

  //////////////////////
  // STEP 2: FEATURES //
  //////////////////////

  // cumulative vector sum in window
  Eigen::Vector3f csum(0, 0, 0);

  std::vector<ScanInfo> cands;

  // fast iterator: point one past right end of interval
  auto i_f = 0u;
  auto it_f = PclIterator(msg);
  for (; i_f != 2 * W + 1; ++i_f, ++it_f) {
    csum += *it_f;
  }

  // mid iterator: points to active point
  auto it_m = PclIterator(msg) + W;

  // slow iterator: points to left end of interval
  auto i_s = 0u;
  auto it_s = PclIterator(msg);

  // calculate c-values:
  //  c = || (2*W + 1) * p - \sum_i p_i || / ( 2 * W * ||p|| )
  //    = || 2 * W * p - \sum_{i != 0} p_i || / (2 * W * ||p||  )
  //    = || p  -  (1/2W) \sum_{i != 0} p_i || / || p ||

  // if c-value is small it is likely a planar point (p close to average of surrounding pts)
  // if c-value is large it is likely an edge point (p far from average of surrounding pts)
  for (; i_f != N; ++i_s, ++i_f, ++it_s, ++it_m, ++it_f) {
    uint32_t i_mid = i_s + W;

    if (valid[i_mid]) {
      cands.push_back(
        {i_mid, true, ((2 * W + 1) * (*it_m) - csum).norm() / (2 * W * (*it_m).norm())}
      );
    }

    csum -= *it_s;
    csum += *it_f;
  }

  // sort candidates by c-value
  std::sort(
    cands.begin(), cands.end(), [](const auto & item1, const auto & item2) {
      return item1.cvalue < item2.cvalue;
    });

  // extract features

  // edges from back
  auto edges = std::make_unique<sensor_msgs::msg::PointCloud2>();
  edges->fields = msg.fields;
  edges->point_step = msg.point_step;
  edges->is_bigendian = msg.is_bigendian;
  edges->header = msg.header;

  for (auto it = cands.crbegin(); it != cands.crend() && it->cvalue > prm.edge_thresh; ++it) {
    auto &[idx, ok, cvalue] = *it;
    if (valid[idx]) {
      std::copy(
        msg.data.begin() + idx * msg.point_step,
        msg.data.begin() + (idx + 1) * msg.point_step,
        std::back_insert_iterator(edges->data)
      );
      for (auto idx_i = idx - W; idx_i != idx + W + 1; ++idx_i) {
        valid[idx_i] = false;  // mark surrounding as taken
      }
    }
  }

  edges->width = edges->data.size() / edges->point_step;
  edges->row_step = edges->width;
  edges->height = 1;
  edges->is_dense = 1;

  // planar from front
  auto planar = std::make_unique<sensor_msgs::msg::PointCloud2>();
  planar->fields = msg.fields;
  planar->point_step = msg.point_step;
  planar->is_bigendian = msg.is_bigendian;
  planar->header = msg.header;

  for (auto it = cands.cbegin(); it != cands.cend() && it->cvalue < prm.plane_thresh; ++it) {
    auto &[idx, ok, cvalue] = *it;
    if (valid[idx]) {
      std::copy(
        msg.data.begin() + idx * msg.point_step,
        msg.data.begin() + (idx + 1) * msg.point_step,
        std::back_insert_iterator(planar->data)
      );
      for (auto idx_i = idx - W; idx_i != idx + W + 1; ++idx_i) {
        valid[idx_i] = false;  // mark surrounding as taken
      }
    }
  }

  planar->width = planar->data.size() / planar->point_step;
  planar->row_step = planar->width;
  planar->height = 1;
  planar->is_dense = 1;

  return {std::move(edges), std::move(planar)};
}

}  // namespace bps
