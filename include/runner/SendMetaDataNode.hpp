#pragma once
#include <behaviortree_cpp/action_node.h>

class SendMetaDataNode : public BT::SyncActionNode
{
public:
  SendMetaDataNode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

