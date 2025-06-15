#include "runner/CheckImageMatch.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>  
#include <fstream>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <iostream>


using json = nlohmann::json;
namespace fs = std::filesystem;

namespace runner
{

CheckImageMatch::CheckImageMatch(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  // JSON 파일 이름 결정 (가장 최근 파일 1개를 기준으로 사용)
  for (const auto& entry : fs::directory_iterator("logs/images/result"))
  {
    if (entry.path().extension() == ".json")
    {
      result_file_ = entry.path().string();
      break;
    }
  }
}

BT::PortsList CheckImageMatch::providedPorts()
{
  return {
    BT::InputPort<std::string>("code")
  };
}

BT::NodeStatus CheckImageMatch::tick()
{
  if (result_file_.empty())
  {
    std::cerr << "[CheckImageMatch] No result file found." << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  std::ifstream f(result_file_);
  if (!f.is_open())
  {
    std::cerr << "[CheckImageMatch] Failed to open JSON: " << result_file_ << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  json j;
  f >> j;

  if (j.contains("anomaly") && j["anomaly"].get<bool>() == true)
  {
    std::cout << "[CheckImageMatch] Anomaly detected. Failing." << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  std::cout << "[CheckImageMatch] No anomaly. Success." << std::endl;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace runner
