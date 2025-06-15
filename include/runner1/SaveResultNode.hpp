#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "runner1_interfaces/srv/image_result.hpp"
#include <crow.h>  // Web dashboard framework
#include <mutex>
#include <nlohmann/json.hpp>

class SaveResultNode : public BT::SyncActionNode
{
public:
  SaveResultNode(const std::string& name, const BT::NodeConfiguration& config);

  // 포트 정의 (입력 없음, 출력만 사용)
  static BT::PortsList providedPorts();

  // tick(): 노드가 호출될 때 실행
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<runner1_interfaces::srv::ImageResult>::SharedPtr client_;

  // 웹 대시보드 관련
  std::thread crow_thread_;
  std::mutex json_mutex_;
  nlohmann::json latest_json_;

  // 로그 파일 저장 함수
  void write_log_to_file(const std::string& timestamp,
                         const std::string& status,
                         const std::string& icon,
                         const std::string& reason,
                         const std::string& first_fail_reason,
                         const std::string& ref_path,
                         const std::string& cur_path,
                         const std::string& compare_image_path,
                         const std::string& yaml_path);

  // 웹 서버 시작
  void start_crow_server();
};
