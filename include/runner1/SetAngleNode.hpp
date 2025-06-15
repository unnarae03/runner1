#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "runner1/action/rotate_angle.hpp"

class SetAngleNode : public BT::SyncActionNode
{
public:
  using RotateAngle = runner1::action::RotateAngle;
  using GoalHandleRotate = rclcpp_action::ClientGoalHandle<RotateAngle>;

  SetAngleNode(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Client<RotateAngle>::SharedPtr action_client_;
};
