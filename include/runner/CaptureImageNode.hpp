#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"  // 🔹 추가됨
#include "runner1/srv/capture_image.hpp"

namespace runner
{

class CaptureImageNode : public BT::AsyncActionNode
{
public:
  CaptureImageNode(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("capture_id") };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;  // 🔹 추가됨

  sensor_msgs::msg::Image::SharedPtr latest_image_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_;  // 🔹 추가됨
  bool image_received_;
};

}  // namespace runner
