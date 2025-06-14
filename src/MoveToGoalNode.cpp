// src/MoveToGoalNode.cpp
#include "runner/MoveToGoalNode.hpp"
#include <iostream>

MoveToGoalNode::MoveToGoalNode(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList MoveToGoalNode::providedPorts()
{
  return {}; // input/output 포트가 있을 경우 여기에 추가
}

BT::NodeStatus MoveToGoalNode::tick()
{
  std::cout << "🏃 [MoveToGoal] 목표 지점으로 이동 중..." << std::endl;
  
  // 실제 이동 코드 대신 성공했다고 가정
  return BT::NodeStatus::SUCCESS;
}
