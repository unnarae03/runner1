#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <string>

class CheckImageMatch : public BT::SyncActionNode
{
public:
  CheckImageMatch(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
