#include "runner1/SetAngleNode.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>
#include <sstream>
#include <random>
#include "behaviortree_cpp_v3/behavior_tree.h"

SetAngleNode::SetAngleNode(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config),
    tf_buffer_(rclcpp::Clock::make_shared()),
    tf_listener_(tf_buffer_)
{
  node_ = rclcpp::Node::make_shared("set_angle_node");
  action_client_ = rclcpp_action::create_client<RotateAngle>(node_, "rotate_angle");
}

BT::PortsList SetAngleNode::providedPorts()
{
  return {
    BT::InputPort<int>("index"),
    BT::InputPort<bool>("retry_mode", false),
    BT::InputPort<int>("retry_attempt", 0),
    BT::InputPort<double>("stored_x"),
    BT::InputPort<double>("stored_y"),
    BT::InputPort<double>("stored_theta"),
    BT::OutputPort<double>("stored_x"),
    BT::OutputPort<double>("stored_y"),
    BT::OutputPort<double>("stored_theta"),
    BT::OutputPort<bool>("abort_repeat_loop")
  };
}

BT::NodeStatus SetAngleNode::tick()
{
  int index;
  bool retry_mode = false;
  int retry_attempt = 0;

  getInput("index", index);
  getInput("retry_mode", retry_mode);
  getInput("retry_attempt", retry_attempt);

  geometry_msgs::msg::Pose target;
  double theta = 0.0;

  if (retry_mode)
  {
    double x, y;
    if (!getInput("stored_x", x) ||
        !getInput("stored_y", y) ||
        !getInput("stored_theta", theta))
    {
      RCLCPP_ERROR(node_->get_logger(), "Retry mode인데 저장된 waypoint 정보 없음");
      return BT::NodeStatus::FAILURE;
    }

    target.position.x = x;
    target.position.y = y;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.05, 0.2);

    double delta = dist(gen);
    double modified_theta = theta + ((retry_attempt % 2 == 0) ? delta : -delta);

    tf2::Quaternion q;
    q.setRPY(0, 0, modified_theta);
    target.orientation = tf2::toMsg(q);

    RCLCPP_INFO(node_->get_logger(), "재시도 #%d: θ %.4f → %.4f (delta %.4f)",
                retry_attempt, theta, modified_theta, delta);
  }
  else
  {
    std::string package_path = ament_index_cpp::get_package_share_directory("runner1");
    std::stringstream filename;
    filename << package_path << "/config/waypoint" << index << ".yaml";

    YAML::Node yaml;
    try {
      yaml = YAML::LoadFile(filename.str());
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "파일 없음: %s", filename.str().c_str());
      setOutput("abort_repeat_loop", true);
      return BT::NodeStatus::SUCCESS;
    }

    target.position.x = yaml["pose"]["x"].as<double>();
    target.position.y = yaml["pose"]["y"].as<double>();
    theta = yaml["pose"]["theta"].as<double>();

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    target.orientation = tf2::toMsg(q);

    setOutput("stored_x", target.position.x);
    setOutput("stored_y", target.position.y);
    setOutput("stored_theta", theta);

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "Transform 실패");
      return BT::NodeStatus::FAILURE;
    }

    double dx = target.position.x - transform.transform.translation.x;
    double dy = target.position.y - transform.transform.translation.y;
    double dist = std::hypot(dx, dy);
    const double threshold = 0.5;

    if (dist >= threshold) {
      RCLCPP_INFO(node_->get_logger(), "거리 %.2f: 너무 멀어서 각도조정 안함", dist);
      setOutput("abort_repeat_loop", false);
      return BT::NodeStatus::SUCCESS;
    }
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "rotate_angle 액션 서버 연결 실패");
    return BT::NodeStatus::FAILURE;
  }

  auto goal = RotateAngle::Goal();
  goal.target_yaw = tf2::getYaw(target.orientation);

  action_client_->async_send_goal(goal);

  setOutput("abort_repeat_loop", false);
  return BT::NodeStatus::SUCCESS;
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<SetAngleNode>("SetAngleNode");
}
