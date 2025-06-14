#pragma once
#include <behaviortree_cpp/action_node.h>

class SetFailureReasonNode : public BT::SyncActionNode
{
public:
  SetFailureReasonNode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

