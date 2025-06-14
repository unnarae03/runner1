#pragma once
#include <behaviortree_cpp/action_node.h>

class MoveToGoalNode : public BT::SyncActionNode
{
public:
  MoveToGoalNode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

