#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <string>

namespace runner
{
class CheckImageMatch : public BT::SyncActionNode
{
public:
  CheckImageMatch(const std::string& name, const BT::NodeConfiguration& config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

private:
  std::string result_file_;
};
}  // namespace runner
