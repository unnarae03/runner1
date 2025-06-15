#include "runner/CaptureImageNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <chrono>

using namespace std::chrono_literals;

namespace runner
{

CaptureImageNode::CaptureImageNode(const std::string& name, const BT::NodeConfiguration& config)
: BT::AsyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("capture_image_node_bt");
  image_received_ = false;

  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", 10,
    [this](sensor_msgs::msg::Image::SharedPtr msg) {
      latest_image_ = msg;
      image_received_ = true;
    });
}

BT::NodeStatus CaptureImageNode::tick()
{
  int capture_id;
  if (!getInput("capture_id", capture_id)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing input [capture_id]");
    return BT::NodeStatus::FAILURE;
  }

  image_received_ = false;
  rclcpp::Rate rate(10);
  int tries = 30;
  while (rclcpp::ok() && !image_received_ && tries-- > 0) {
    rclcpp::spin_some(node_);
    rate.sleep();
  }

  if (!image_received_ || !latest_image_) {
    RCLCPP_ERROR(node_->get_logger(), "No image received");
    return BT::NodeStatus::FAILURE;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(latest_image_, "bgr8");
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  std::string save_path = "/home/hayeon/runner1/logs/images/current/" + std::to_string(capture_id) + ".jpg";
  if (!cv::imwrite(save_path, cv_ptr->image)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to save image to %s", save_path.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "Image saved: %s", save_path.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace runner
