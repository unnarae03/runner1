#pragma once

#include <behaviortree_cpp_v3/action_node.h>  

class SaveResultNode : public BT::SyncActionNode
{
public:
  // 생성자
  SaveResultNode(const std::string& name, const BT::NodeConfiguration& config);

  // 포트 설정 (받을 변수들)
  static BT::PortsList providedPorts();

  // 노드가 실행될 때 호출되는 tick() 함수
  BT::NodeStatus tick() override;

private:
  // 필요한 ROS2 노드와 퍼블리셔 선언
  rclcpp::Node::SharedPtr node_;  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // 로그를 파일에 기록하는 함수
  void write_log_to_file(const std::string& timestamp,
                         const std::string& photo_id,
                         const std::string& status,
                         const std::string& icon,
                         const std::string& reason,
                         const std::string& first_fail_reason,
                         const std::string& image_path);
};
