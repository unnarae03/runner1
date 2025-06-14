#include "runner/RepeatUntilFailure.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>

RepeatUntilFailure::RepeatUntilFailure(const std::string& name, const BT::NodeConfiguration& config)
  : BT::DecoratorNode(name, config)
{}

BT::PortsList RepeatUntilFailure::providedPorts()
{
  return { BT::InputPort<bool>("abort_repeat_loop") };
}

BT::NodeStatus RepeatUntilFailure::tick()
{
  bool abort = false;
  getInput("abort_repeat_loop", abort);

  if (abort)
  {
    return BT::NodeStatus::SUCCESS;
  }

  const BT::NodeStatus status = child_node_->executeTick();

  if (status == BT::NodeStatus::FAILURE || status == BT::NodeStatus::SUCCESS)
  {
    return BT::NodeStatus::RUNNING;
  }

  return status;
}

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<RepeatUntilFailure>("RepeatUntilFailure");
}
