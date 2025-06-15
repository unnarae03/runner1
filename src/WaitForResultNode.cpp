#include "runner/WaitForResultNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

WaitForResultNode::WaitForResultNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config),
      tf_buffer_(rclcpp::Clock::make_shared()),
      tf_listener_(tf_buffer_)
{
  node_ = rclcpp::Node::make_shared("wait_for_result_node");
}

BT::PortsList WaitForResultNode::providedPorts()
{
  return {
    BT::InputPort<double>("target_x"),
    BT::InputPort<double>("target_y"),
    BT::InputPort<double>("threshold", 0.15, "Distance threshold to goal")
  };
}

BT::NodeStatus WaitForResultNode::tick()
{
  double target_x = 0.0;
  double target_y = 0.0;
  double threshold = 0.15;  // 기본값

  if (!getInput<double>("target_x", target_x) || !getInput<double>("target_y", target_y)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing input [target_x] or [target_y]");
    return BT::NodeStatus::FAILURE;
  }

  getInput<double>("threshold", threshold);  // 선택 입력, 없으면 기본값 사용

  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  double dx = target_x - transform.transform.translation.x;
  double dy = target_y - transform.transform.translation.y;
  double distance = std::hypot(dx, dy);

  RCLCPP_INFO(node_->get_logger(), "Distance to goal: %.3f (threshold: %.3f)", distance, threshold);

  if (distance <= threshold) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}
