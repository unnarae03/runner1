#pragma once
#include <behaviortree_cpp/action_node.h>

class CaptureImageNode : public BT::SyncActionNode
{
public:
  CaptureImageNode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

