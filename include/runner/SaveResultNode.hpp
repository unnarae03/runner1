#pragma once
#include <behaviortree_cpp/action_node.h>

class SaveResultNode : public BT::SyncActionNode
{
public:
  SaveResultNode(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

