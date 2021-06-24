// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/ros2_pcl_utils/blob/master/LICENSE

#include "pcl_utils/pcl_image_overlay_node.hpp"

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pcl_utils/camera.hpp"
#include "pcl_utils/pcl_iterator.hpp"

namespace cbr
{

struct PclImageOverlayNode::Impl
{
  std::atomic<bool> calib_received{false};
  sensor_msgs::msg::CameraInfo cam{};

  std::atomic<bool> frame_received{false};
  std::string img_frame{};

  std::mutex points_accum_mtx;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
  points_accum{};

  std::size_t k_max_size{};
  float k_cmap_min{}, k_cmap_range{};
  int k_draw_radius{};
};

PclImageOverlayNode::PclImageOverlayNode(const rclcpp::NodeOptions & opts)
: Node("image_overlay", opts), pImpl(std::make_unique<Impl>())
{
  declare_parameter<int>("max_size", 10000);
  pImpl->k_max_size = get_parameter("max_size").as_int();

  declare_parameter<double>("cmap_min", 2.);
  pImpl->k_cmap_min = get_parameter("cmap_min").as_double();
  declare_parameter<double>("cmap_max", 20.);
  pImpl->k_cmap_range = get_parameter("cmap_max").as_double() - pImpl->k_cmap_min;
  declare_parameter<int>("draw_radius", 2);
  pImpl->k_draw_radius = get_parameter("draw_radius").as_int();

  sub_img_ = create_subscription<sensor_msgs::msg::Image>(
    "image", rclcpp::SystemDefaultsQoS(),
    std::bind(&PclImageOverlayNode::cb_img_, this, std::placeholders::_1));

  sub_calib_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      if (!pImpl->calib_received) {
        pImpl->cam = *msg;
        pImpl->calib_received = true;
        RCLCPP_INFO(get_logger(), "Received camera info");
      }
    });

  sub_pcl_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&PclImageOverlayNode::cb_pcl_, this, std::placeholders::_1));

  pub_overlay_ = create_publisher<sensor_msgs::msg::Image>(
    "image_overlay", rclcpp::SystemDefaultsQoS());

  tf2_buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buf_);

  RCLCPP_INFO(get_logger(), "Started node");
}

PclImageOverlayNode::~PclImageOverlayNode()
{
  RCLCPP_INFO(get_logger(), "Closing node");
}

void PclImageOverlayNode::cb_img_(sensor_msgs::msg::Image::UniquePtr msg)
{
  if (!pImpl->frame_received) {
    pImpl->img_frame = msg->header.frame_id;
    pImpl->frame_received = true;
    RCLCPP_INFO(get_logger(), "Got image frame %s", pImpl->img_frame.c_str());
  }

  if (!pImpl->calib_received) {
    return;
  }

  {
    if (msg->encoding != sensor_msgs::image_encodings::RGB8) {
      RCLCPP_WARN(get_logger(), "Only RGB8 supported");
    }

    // draw points on image
    cv::Mat img(msg->height, msg->width, CV_8UC3, msg->data.data());

    // draw points on image
    std::lock_guard lock(pImpl->points_accum_mtx);
    for (const auto & pt_CAM : pImpl->points_accum) {
      // project to pixel coordinaimagetes
      Eigen::Vector2f pt_PX = cameraProject(pt_CAM, pImpl->cam);
      const auto cl = std::clamp<float>(
        static_cast<float>((pt_CAM.z() - pImpl->k_cmap_min) /
        pImpl->k_cmap_range),
        0., 1.);
      cv::circle(
        img, cv::Point2f(pt_PX.x(), pt_PX.y()),
        pImpl->k_draw_radius, cv::Scalar(255 * cl, 0, 255 * (1 - cl))
      );
    }

    pImpl->points_accum.clear();
  }

  // publish image
  pub_overlay_->publish(std::move(msg));
}

void PclImageOverlayNode::cb_pcl_(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  if (!pImpl->frame_received) {
    return;
  }

  if (!tf2_buf_->canTransform(pImpl->img_frame, msg->header.frame_id, rclcpp::Time(0))) {
    return;
  }

  // transform cloud to sensor frame
  auto tf = tf2_buf_->lookupTransform(
    pImpl->img_frame, msg->header.frame_id, rclcpp::Time(0)
    ).transform;

  Eigen::Quaternionf q_CAM_LIDAR(tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z);
  Eigen::Vector3f t_CAM_LIDAR(tf.translation.x, tf.translation.y, tf.translation.z);

  PclIterator it(*msg);

  std::lock_guard lock(pImpl->points_accum_mtx);

  if (pImpl->points_accum.size() > pImpl->k_max_size) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Accumulated %lu points without getting in image, resetting...",
      pImpl->points_accum.size());
    pImpl->points_accum.clear();
  }

  for (auto i = 0u; i != msg->width; ++i, ++it) {
    if (it.intensity() >= 0) {
      pImpl->points_accum.push_back(q_CAM_LIDAR * *it + t_CAM_LIDAR);
    }
  }
}

}  // namespace cbr

RCLCPP_COMPONENTS_REGISTER_NODE(cbr::PclImageOverlayNode)
