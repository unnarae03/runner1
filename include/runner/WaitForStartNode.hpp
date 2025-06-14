#pragma once
#include <behaviortree_cpp_v3/action_node.h>

class WaitForStartNode : public BT::SyncActionNode
{
public:
  WaitForStartNode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

