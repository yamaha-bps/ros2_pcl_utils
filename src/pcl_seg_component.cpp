// Copyright 2020 Yamaha Motor Corporation, USA

#include "pcl_seg/pcl_seg_component.hpp"

#include <boost/circular_buffer.hpp>
#include <bps_library/msg_utils.hpp>
#include <bps_library/fmt.hpp>
#include <fmt/format.h>
#include <sophus/se3.hpp>

#include <condition_variable>
#include <deque>
#include <limits>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include "filter.hpp"

namespace bps
{

struct PclSegComponent::Impl
{
  std::deque<sensor_msgs::msg::PointCloud2::UniquePtr> pcl_queue_{};
  std::deque<sensor_msgs::msg::Image::UniquePtr> img_queue_{};

  std::mutex mtx_;
  std::condition_variable condition_;

  std::atomic<bool> calib_received_;
  bps_msgs::msg::MonoCalibration calib_;

  // class indices to pass through
  std::unordered_set<uint8_t> classes_;
};


PclSegComponent::PclSegComponent(const rclcpp::NodeOptions & opts)
: Node("pcl_seg", opts),
  pImpl(std::make_unique<Impl>())
{
  declare_parameter<std::vector<int64_t>>("classes", std::vector<int64_t>{0});
  auto classes_vec = get_parameter("classes").as_integer_array();

  for (auto c : classes_vec) {
    if (c < 0 || c > std::numeric_limits<uint8_t>::max()) {
      RCLCPP_ERROR(get_logger(), "Class index %d not a uint8", c);
    } else {
      pImpl->classes_.insert(static_cast<uint8_t>(c));
    }
  }

  RCLCPP_INFO(
    get_logger(), "Filtering classes [%s]", fmt::format(
      "{}", fmt::join(pImpl->classes_, ", ")).c_str());


  sub_pcl_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud",
    rclcpp::SensorDataQoS(),
    std::bind(&PclSegComponent::cb_pcl_, this, std::placeholders::_1)
  );

  sub_img_ = create_subscription<sensor_msgs::msg::Image>(
    "segmentation",
    rclcpp::SensorDataQoS(),
    std::bind(&PclSegComponent::cb_img_, this, std::placeholders::_1)
  );

  sub_calib_ = create_subscription<bps_msgs::msg::MonoCalibration>(
    "calibration",
    rclcpp::SystemDefaultsQoS(),
    [this](const bps_msgs::msg::MonoCalibration::SharedPtr msg) {
      if (!pImpl->calib_received_) {
        std::lock_guard lock(pImpl->mtx_);
        pImpl->calib_ = *msg;
        pImpl->calib_received_ = true;
        RCLCPP_INFO(get_logger(), "Received calibration");
      }
    });

  pub_pcl_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "pointcloud_filtered",
    rclcpp::SensorDataQoS()
  );

  tf2_buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buf_);

  worker_thread_ = std::thread(std::bind(&PclSegComponent::work, this));

  RCLCPP_INFO(get_logger(), "Pointcloud segmentation node started");
}


PclSegComponent::~PclSegComponent()
{
  canceled_.store(true);
  pImpl->condition_.notify_one();

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }

  RCLCPP_INFO(get_logger(), "Closing pointcloud segmentation node");
}


void PclSegComponent::cb_pcl_(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  std::lock_guard lock(pImpl->mtx_);

  rclcpp::Time msg_t(msg->header.stamp);
  if (!pImpl->pcl_queue_.empty() && msg_t < rclcpp::Time(pImpl->pcl_queue_.back()->header.stamp)) {
    RCLCPP_WARN(get_logger(), "Pointcloud arrived out of order, discarding...");
    return;
  }

  pImpl->pcl_queue_.push_back(std::move(msg));

  if (pImpl->pcl_queue_.size() > 2000) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "PCLs accumulating (n=%d)", pImpl->pcl_queue_.size());
  }

  pImpl->condition_.notify_one();
}


void PclSegComponent::cb_img_(sensor_msgs::msg::Image::UniquePtr msg)
{
  std::lock_guard lock(pImpl->mtx_);

  rclcpp::Time msg_t(msg->header.stamp);
  if (!pImpl->img_queue_.empty() && msg_t < rclcpp::Time(pImpl->img_queue_.back()->header.stamp)) {
    RCLCPP_WARN(get_logger(), "Image arrived out of order, discarding...");
    return;
  }

  pImpl->img_queue_.push_back(std::move(msg));

  if (pImpl->img_queue_.size() > 2000) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Images accumulating (n=%d)", pImpl->img_queue_.size());
  }
}


void PclSegComponent::work()
{
  while (!canceled_) {
    std::unique_lock<std::mutex> lock(pImpl->mtx_);

    pImpl->condition_.wait(lock);

    // ensure that we have at least one pointcloud
    if (pImpl->pcl_queue_.empty()) {
      continue;
    }

    if (!pImpl->calib_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for calibration");
      continue;
    }

    // discard old pointclouds (should only occur at the beginning)
    while (!pImpl->img_queue_.empty() && !pImpl->pcl_queue_.empty()) {
      rclcpp::Time img_t(pImpl->img_queue_.front()->header.stamp);
      rclcpp::Time pcl_t(pImpl->pcl_queue_.front()->header.stamp);
      if (pcl_t < img_t) {
        pImpl->pcl_queue_.pop_front();
      } else {
        break;
      }
    }

    // this is the pointcloud of interest
    auto & pcl = pImpl->pcl_queue_.front();
    rclcpp::Time pcl_t(pcl->header.stamp);

    // discard non-relevant images
    while (pImpl->img_queue_.size() >= 2 &&
      rclcpp::Time(pImpl->img_queue_[1]->header.stamp) <= pcl_t)
    {
      pImpl->img_queue_.pop_front();
    }

    // ensure that we have an image before and after the pointcloud
    if (pImpl->img_queue_.size() < 2) {
      continue;
    }

    // here we should be guaranteed to have one image before and one image after the pcl
    rclcpp::Time img0_t(pImpl->img_queue_[0]->header.stamp);
    rclcpp::Time img1_t(pImpl->img_queue_[1]->header.stamp);

    if (!(img0_t <= pcl_t && pcl_t <= img1_t)) {
      RCLCPP_ERROR(get_logger(), "This should never happen!");
      pImpl->pcl_queue_.pop_front();
      continue;
    }

    // pick segmentation image closest to the pointcloud
    std::size_t idx =
      std::abs((img0_t - pcl_t).seconds()) < std::abs((img1_t - pcl_t).seconds()) ?
      0 :
      1;

    // pull transform from tf one time
    static bool transform_received = false;
    static Sophus::SE3f P_CL;
    if (!transform_received) {
      // look up transform from (L)idar sensor to (C)amera sensor
      if (!tf2_buf_->canTransform(
          pImpl->img_queue_[idx]->header.frame_id, pcl->header.frame_id,
          pcl->header.stamp))
      {
        RCLCPP_WARN(
          get_logger(), "Can not transform from %s to %s",
          pcl->header.frame_id.c_str(), pImpl->img_queue_[idx]->header.frame_id.c_str()
        );
        pImpl->pcl_queue_.pop_front();
        continue;
      }

      auto tf = tf2_buf_->lookupTransform(
        pImpl->img_queue_[idx]->header.frame_id, pcl->header.frame_id,
        pcl->header.stamp);

      P_CL = msgs::from_msg(tf.transform).cast<float>();
      RCLCPP_INFO(get_logger(), "%s", fmt::format("Found transform {}", P_CL).c_str());

      transform_received = true;

      // no more use for tf so we kill it
      tf2_listener_ = nullptr;
      tf2_buf_ = nullptr;
    }


    // filter the pointcloud with respect to the image
    filter(*pImpl->img_queue_[idx], pImpl->calib_, pImpl->classes_, P_CL, *pcl);

    // re-publish filtered pointcloud
    pub_pcl_->publish(std::move(pcl));
    pImpl->pcl_queue_.pop_front();
  }
}


}  // namespace bps
