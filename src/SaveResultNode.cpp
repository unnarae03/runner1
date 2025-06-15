#include <behaviortree_cpp_v3/bt_factory.h>
#include "runner/SaveResultNode.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>

SaveResultNode::SaveResultNode(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("save_result_node");
  client_ = node_->create_client<runner_interfaces::srv::ImageResult>("check_image_match");
  crow_thread_ = std::thread([this]() { this->start_crow_server(); });
}

BT::PortsList SaveResultNode::providedPorts()
{
  return {
    BT::OutputPort<std::string>("image_status"),
    BT::OutputPort<std::string>("failure_reason"),
    BT::OutputPort<std::string>("first_fail_reason"),
    BT::OutputPort<std::string>("compare_image_path"),
    BT::OutputPort<std::string>("cur_path"),
    BT::OutputPort<std::string>("ref_path"),
    BT::OutputPort<std::string>("yaml_path")
  };
}

BT::NodeStatus SaveResultNode::tick()
{
  auto request = std::make_shared<runner_interfaces::srv::ImageResult::Request>();
  if (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "Service not available");
    return BT::NodeStatus::FAILURE;
  }

  auto result = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Service call failed");
    return BT::NodeStatus::FAILURE;
  }

  auto response = result.get();
  std::string status = response->image_status;
  std::string reason = response->failure_reason;
  std::string first_reason = response->first_fail_reason;
  std::string compare_path = response->compare_image_path;
  std::string cur_path = response->cur_path;
  std::string ref_path = response->ref_path;
  std::string yaml_path = response->yaml_path;

  setOutput("image_status", status);
  setOutput("failure_reason", reason);
  setOutput("first_fail_reason", first_reason);
  setOutput("compare_image_path", compare_path);
  setOutput("cur_path", cur_path);
  setOutput("ref_path", ref_path);
  setOutput("yaml_path", yaml_path);

  auto now = std::chrono::system_clock::now();
  std::time_t time_now = std::chrono::system_clock::to_time_t(now);
  std::ostringstream oss;
  oss << std::put_time(std::localtime(&time_now), "%Y-%m-%d %H:%M:%S");
  std::string timestamp = oss.str();

  std::string icon = (status == "OK") ? "✅" : (status == "ANOMALY") ? "❌" : "⚠️";
  std::string log_msg = "[" + timestamp + "] " + icon + " 상태: " + status +
                        " | 이유: " + reason + " | 현재이미지: " + cur_path +
                        " | 비교이미지: " + compare_path + " | 좌표(YAML): " + yaml_path;

  RCLCPP_INFO(node_->get_logger(), "%s", log_msg.c_str());
  std_msgs::msg::String msg;
  msg.data = log_msg;
  publisher_->publish(msg);

  write_log_to_file(timestamp, status, icon, reason, first_reason, compare_path, cur_path, ref_path, yaml_path);

  {
    std::lock_guard<std::mutex> lock(json_mutex_);
    latest_json_ = {
      {"image_status", status},
      {"failure_reason", reason},
      {"first_fail_reason", first_reason},
      {"compare_image_path", compare_path},
      {"cur_path", cur_path},
      {"ref_path", ref_path},
      {"yaml_path", yaml_path},
      {"timestamp", timestamp},
      {"icon", icon}
    };
  }

  return BT::NodeStatus::SUCCESS;
}

void SaveResultNode::write_log_to_file(const std::string& timestamp,
                                       const std::string& status,
                                       const std::string& icon,
                                       const std::string& reason,
                                       const std::string& first_fail_reason,
                                       const std::string& compare_path,
                                       const std::string& cur_path,
                                       const std::string& ref_path,
                                       const std::string& yaml_path)
{
  std::string path = "/home/geonha/dashboard_log.csv";
  std::ofstream file(path, std::ios::app);
  if (file.tellp() == 0) {
    file << "timestamp,status,icon,fail_reason,first_fail_reason,compare_image_path,cur_path,ref_path,yaml_path\n";
  }
  file << timestamp << "," << status << "," << icon << "," << reason << ","
       << first_fail_reason << "," << compare_path << "," << cur_path << ","
       << ref_path << "," << yaml_path << "\n";
}

void SaveResultNode::start_crow_server()
{
  crow::SimpleApp app;

  CROW_ROUTE(app, "/status")
  ([this]() {
    std::lock_guard<std::mutex> lock(json_mutex_);
    return crow::response(latest_json_.dump(2));
  });

  CROW_ROUTE(app, "/dashboard")
  ([this]() {
    std::lock_guard<std::mutex> lock(json_mutex_);
    std::string html = R"(
      <!DOCTYPE html>
      <html lang=\"ko\">
        <head>
          <meta charset=\"UTF-8\">
          <title>📸 Image Dashboard</title>
        </head>
        <body>
          <h1>📸 Image Dashboard</h1>
    )";
    html += "<p>" + latest_json_["icon"].get<std::string>() + " " +
            latest_json_["image_status"].get<std::string>() + "</p>";
    html += "<p>좌표(YAML): " + latest_json_["yaml_path"].get<std::string>() + "</p>";
    html += "<p>실패 이유: " + latest_json_["failure_reason"].get<std::string>() + "</p>";
    html += "<p>첫 실패: " + latest_json_["first_fail_reason"].get<std::string>() + "</p>";
    html += "<div><img src='/images/" + latest_json_["ref_path"].get<std::string>() + "' width='300'> 참조이미지</div>";
    html += "<div><img src='/images/" + latest_json_["cur_path"].get<std::string>() + "' width='300'> 현재이미지</div>";
    html += "<div><img src='/images/" + latest_json_["compare_image_path"].get<std::string>() + "' width='300'> 비교이미지</div>";
    html += "</body></html>";
    return crow::response(html);
  });

  CROW_ROUTE(app, "/images/<string>")([](const std::string& filename) {
    std::string full_path = "/home/geonha/web_root/images/" + filename;
    std::ifstream file(full_path, std::ios::binary);
    if (!file) return crow::response(404);
    std::ostringstream ss;
    ss << file.rdbuf();
    auto res = crow::response(ss.str());
    res.set_header("Content-Type", "image/jpeg");
    return res;
  });

  app.port(8000).multithreaded().run();
}
