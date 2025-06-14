#pragma once
#include <behaviortree_cpp/action_node.h>

class SetAngleNode : public BT::SyncActionNode
{
public:
  SetAngleNode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

