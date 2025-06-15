#include "runner/CaptureImageNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <chrono>
#include <fstream>  // 🔹 추가됨
#include <yaml-cpp/yaml.h>  // 🔹 추가됨
#include <tf2/LinearMath/Matrix3x3.h>  // 🔹 추가됨
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // 🔹 추가됨

using namespace std::chrono_literals;

namespace runner  // 🔹 이 namespace 안에 tick()도 포함되어야 함
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

  // 🔹 pose_sub_ 추가됨
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
      latest_pose_ = msg;
    });
}

// 🔹 tick() 위치가 잘못되어 있던 부분을 namespace 안으로 옮김
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

  if (!image_received_ || !latest_image_ || !latest_pose_) {  // 🔹 pose 체크도 추가됨
    RCLCPP_ERROR(node_->get_logger(), "No image or pose received");
    return BT::NodeStatus::FAILURE;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(latest_image_, "bgr8");
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  std::string filename = std::to_string(capture_id) + ".jpg";
  std::string save_path = "/home/hayeon/runner1/logs/images/current/" + filename;

  if (!cv::imwrite(save_path, cv_ptr->image)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to save image to %s", save_path.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 🔹 timestamp 생성
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  char time_str[100];
  std::strftime(time_str, sizeof(time_str), "%F %T", std::localtime(&now_c));

  // 🔹 orientation → RPY 변환
  tf2::Quaternion q(
    latest_pose_->pose.pose.orientation.x,
    latest_pose_->pose.pose.orientation.y,
    latest_pose_->pose.pose.orientation.z,
    latest_pose_->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // 🔹 YAML 저장
  std::string yaml_path = "/home/hayeon/runner1/logs/images/current/" + std::to_string(capture_id) + ".yaml";

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "capture_id" << YAML::Value << capture_id;
  out << YAML::Key << "timestamp" << YAML::Value << std::string(time_str);
  out << YAML::Key << "position" << YAML::Value << YAML::Flow << YAML::BeginSeq
      << latest_pose_->pose.pose.position.x
      << latest_pose_->pose.pose.position.y
      << latest_pose_->pose.pose.position.z << YAML::EndSeq;
  out << YAML::Key << "orientation" << YAML::Value << YAML::Flow << YAML::BeginSeq
      << roll << pitch << yaw << YAML::EndSeq;
  out << YAML::Key << "image_filename" << YAML::Value << filename;
  out << YAML::Key << "success" << YAML::Value << true;
  out << YAML::EndMap;

  std::ofstream fout(yaml_path);
  fout << out.c_str();
  fout.close();

  RCLCPP_INFO(node_->get_logger(), "Image and metadata saved: %s", save_path.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace runner
