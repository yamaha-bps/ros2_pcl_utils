// Copyright 2020 Yamaha Motor Corporation, USA

#include "pcl_seg/pcl_seg_component.hpp"

#include <boost/circular_buffer.hpp>
#include <bps_library/msg_utils.hpp>
#include <bps_library/fmt.hpp>
#include <fmt/format.h>
#include <sophus/se3.hpp>

#include <condition_variable>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <unordered_set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "filter.hpp"

namespace bps
{

struct PclSegComponent::Impl
{
  std::mutex pcl_queue_mtx;
  boost::circular_buffer<sensor_msgs::msg::PointCloud2::UniquePtr> pcl_queue{};

  std::mutex img_queue_mtx;
  std::deque<sensor_msgs::msg::Image::UniquePtr> img_queue{};

  std::condition_variable condition_;

  std::atomic<bool> calib_received_;
  bps_msgs::msg::MonoCalibration calib;

  std::atomic<bool> camera_frame_received;
  std::string camera_frame;

  // class indices to pass through
  std::unordered_set<uint8_t> classes_;

  std::mutex frames_mtx;
  std::unordered_map<std::string, Sophus::SE3f, std::hash<std::string>, std::equal_to<std::string>,
    Eigen::aligned_allocator<std::pair<std::string, Sophus::SE3f>>> frames;
};


PclSegComponent::PclSegComponent(const rclcpp::NodeOptions & opts)
: Node("pcl_seg", opts),
  pImpl(std::make_unique<Impl>())
{
  declare_parameter<std::vector<int64_t>>("classes", std::vector<int64_t>{0});
  auto classes_vec = get_parameter("classes").as_integer_array();

  declare_parameter<int>("n_pclbuf", 500);

  pImpl->pcl_queue.set_capacity(get_parameter("n_pclbuf").as_int());

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
        pImpl->calib = *msg;
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
  if (!pImpl->calib_received_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "Waiting for calibration");
    return;
  }

  if (!pImpl->camera_frame_received) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "Waiting for segmentation image");
    return;
  }

  // look up transform
  if (pImpl->frames.find(msg->header.frame_id) == pImpl->frames.end()) {
    if (!tf2_buf_->canTransform(pImpl->camera_frame, msg->header.frame_id, msg->header.stamp)) {
      RCLCPP_WARN(
        get_logger(), "Can not transform from %s to %s",
        msg->header.frame_id.c_str(), pImpl->camera_frame.c_str());
      return;
    }

    auto tf =
      tf2_buf_->lookupTransform(pImpl->camera_frame, msg->header.frame_id, msg->header.stamp);

    std::lock_guard lock(pImpl->frames_mtx);
    pImpl->frames[msg->header.frame_id] = msgs::from_msg(tf.transform).cast<float>();
  }

  {
    std::lock_guard lock(pImpl->pcl_queue_mtx);

    if (pImpl->pcl_queue.full()) {
      RCLCPP_WARN(
        get_logger(), "PCL buffer reached capacity %d, dropping oldest",
        pImpl->pcl_queue.capacity());
    }
    pImpl->pcl_queue.push_back(std::move(msg));
  }

  pImpl->condition_.notify_one();
}


void PclSegComponent::cb_img_(sensor_msgs::msg::Image::UniquePtr msg)
{
  std::lock_guard lock(pImpl->img_queue_mtx);

  rclcpp::Time msg_t(msg->header.stamp);
  if (!pImpl->img_queue.empty() && msg_t < rclcpp::Time(pImpl->img_queue.back()->header.stamp)) {
    RCLCPP_WARN(get_logger(), "Seg image arrived out of order, discarding...");
    return;
  }

  if (!pImpl->camera_frame_received) {
    pImpl->camera_frame = msg->header.frame_id;
    pImpl->camera_frame_received = true;
  }

  pImpl->img_queue.push_back(std::move(msg));
}


void PclSegComponent::work()
{
  while (!canceled_) {
    std::unique_ptr<sensor_msgs::msg::PointCloud2> pcl;

    // wait only if queue is empty
    {
      std::unique_lock<std::mutex> lock(pImpl->pcl_queue_mtx);
      if (pImpl->pcl_queue.empty()) {
        pImpl->condition_.wait(lock, [this] {return canceled_ || !pImpl->pcl_queue.empty();});
      }
    }

    if (canceled_) {
      break;
    }

    // here we are guaranteed the following:
    //  - all pcls in queue have frame transform information stored in pImpl->frames
    //  - pImpl->calib exists

    if (!pImpl->calib_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Waiting for calibration");
      continue;
    }

    {
      // discard old pointclouds (should only occur at the beginning)
      std::lock_guard l1(pImpl->img_queue_mtx);
      std::lock_guard l2(pImpl->pcl_queue_mtx);
      while (!pImpl->img_queue.empty() && !pImpl->pcl_queue.empty()) {
        rclcpp::Time img_t(pImpl->img_queue.front()->header.stamp);
        rclcpp::Time pcl_t(pImpl->pcl_queue.front()->header.stamp);
        if (pcl_t < img_t) {
          pImpl->pcl_queue.pop_front();
        } else {
          break;
        }
      }

      // discard non-relevant images
      while (pImpl->img_queue.size() >= 2 &&
        rclcpp::Time(pImpl->img_queue[1]->header.stamp) <=
        rclcpp::Time(pImpl->pcl_queue.front()->header.stamp))
      {
        pImpl->img_queue.pop_front();
      }
      // ensure that we have one image before and after the pointcloud
      if (pImpl->img_queue.size() < 2) {
        continue;
      }

      // grab next pointcloud
      pcl = std::move(pImpl->pcl_queue.front());
      pImpl->pcl_queue.pop_front();
    }

    // it's fine to not lock img_queue_mtx here since we are only reading,
    // and other thread is just adding which does not invalidate pointers
    // (https://en.cppreference.com/w/cpp/container/deque)

    rclcpp::Time pcl_t(pcl->header.stamp);

    std::lock_guard flock(pImpl->frames_mtx);
    auto it = pImpl->frames.find(pcl->header.frame_id);
    if (it != pImpl->frames.end()) {
      const auto P_CAM_LID = it->second;

      // here we are guaranteed to have one image before and one image after the pcl
      rclcpp::Time img0_t(pImpl->img_queue[0]->header.stamp);
      rclcpp::Time img1_t(pImpl->img_queue[1]->header.stamp);

      if (!(img0_t <= pcl_t && pcl_t <= img1_t)) {
        // something went wrong (maybe pointclouds arrived out of order, discard)
        continue;
      }

      // pick segmentation image that is closest to the pointcloud in time
      std::size_t idx =
        std::abs((img0_t - pcl_t).seconds()) < std::abs((img1_t - pcl_t).seconds()) ?
        0 :
        1;

      // filter the pointcloud with respect to the image
      filter(*pImpl->img_queue[idx], pImpl->calib, pImpl->classes_, P_CAM_LID, *pcl);

      // re-publish filtered pointcloud
      pub_pcl_->publish(std::move(pcl));
    }
  }
}

}  // namespace bps
