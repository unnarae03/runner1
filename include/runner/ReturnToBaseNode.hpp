#pragma once
#include <behaviortree_cpp/action_node.h>

class ReturnToBaseNode : public BT::SyncActionNode
{
public:
  ReturnToBaseNode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

