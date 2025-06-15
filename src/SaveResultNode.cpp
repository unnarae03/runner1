#include <behaviortree_cpp_v3/bt_factory.h>
#include "runner/SaveResultNode.hpp"

SaveResultNode::SaveResultNode(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("save_result_node");
  client_ = node_->create_client<runner_interfaces::srv::ImageResult>("check_image_match");
}

BT::PortsList SaveResultNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("photo_id"),
    BT::OutputPort<std::string>("image_status"),
    BT::OutputPort<std::string>("failure_reason"),
    BT::OutputPort<std::string>("first_fail_reason"),
    BT::OutputPort<std::string>("image_path")
  };
}

BT::NodeStatus SaveResultNode::tick()
{
  std::string photo_id;
  if (!getInput("photo_id", photo_id)) {
    throw BT::RuntimeError("Missing required input [photo_id]");
  }

  if (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "Service not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<runner_interfaces::srv::ImageResult::Request>();
  request->photo_id = photo_id;

  auto result = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Service call failed");
    return BT::NodeStatus::FAILURE;
  }

  auto response = result.get();
  setOutput("image_status", response->image_status);
  setOutput("failure_reason", response->failure_reason);
  setOutput("first_fail_reason", response->first_fail_reason);
  setOutput("image_path", response->image_path);

  return BT::NodeStatus::SUCCESS;
}
