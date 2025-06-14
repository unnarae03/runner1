#pragma once
#include <behaviortree_cpp/action_node.h>

class WaitForResultNode : public BT::SyncActionNode
{
public:
  WaitForResultNode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

