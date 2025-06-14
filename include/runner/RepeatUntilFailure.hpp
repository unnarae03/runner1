#pragma once

#include <behaviortree_cpp_v3/decorator_node.h>

class RepeatUntilFailure : public BT::DecoratorNode
{
public:
  RepeatUntilFailure(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};
