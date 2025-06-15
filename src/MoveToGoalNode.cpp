#include "runner/MoveToGoalNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <memory>

MoveToGoal::MoveToGoal(const std::string& name, const BT::NodeConfiguration& config)
  : BT::AsyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("move_to_goal_bt_node");
  action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");

  while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for NavigateToPose action server...");
  }
}

BT::PortsList MoveToGoal::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::PoseStamped>("goal") };
}

BT::NodeStatus MoveToGoal::tick()
{
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    throw BT::RuntimeError("Missing required input [goal]");
  }

  nav2_msgs::action::NavigateToPose::Goal nav_goal;
  nav_goal.pose = goal;

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  auto future_goal_handle = action_client_->async_send_goal(nav_goal, send_goal_options);

  if (rclcpp::spin_until_future_complete(node_, future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Send goal call failed");
    return BT::NodeStatus::FAILURE;  // 통신 실패일 경우에만 실패
  }

  // 이동 명령만 내리고 결과는 무시. 다음 노드에서 판단.
  return BT::NodeStatus::SUCCESS;
}

void MoveToGoal::halt()
{
  RCLCPP_WARN(node_->get_logger(), "MoveToGoal halt called");
}
