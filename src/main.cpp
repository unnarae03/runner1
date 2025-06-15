#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>  // ✅ 추가

// 각 노드 헤더 include
#include "runner1/MoveToGoalNode.hpp"
#include "runner1/CaptureImageNode.hpp"
#include "runner1/SetAngleNode.hpp"
#include "runner1/WaitForResultNode.hpp"
#include "runner1/SaveResultNode.hpp"
#include "runner1/ReturnToBaseNode.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;

    // 노드 등록
    factory.registerNodeType<MoveToGoalNode>("MoveToGoal");
    factory.registerNodeType<SetAngleNode>("SetAngle");
    factory.registerNodeType<CaptureImageNode>("CaptureImage");
    factory.registerNodeType<WaitForResultNode>("WaitForResult");
    factory.registerNodeType<SaveResultNode>("SaveResult");
    factory.registerNodeType<ReturnToBaseNode>("ReturnToBase");
    
    // 트리 로딩 - 패키지 경로에서 절대 경로로 XML 찾기
    std::string package_path = ament_index_cpp::get_package_share_directory("runner1");
    std::string tree_path = package_path + "/behavior_trees/patrol_bt.xml";
    auto tree = factory.createTreeFromFile(tree_path);

    // ROS2 노드 생성
    auto node = rclcpp::Node::make_shared("bt_loop");

    RCLCPP_INFO(node->get_logger(), "Behavior Tree 실행 시작");

    rclcpp::Rate loop_rate(10); // 10 Hz

    while (rclcpp::ok())
    {
        tree.rootNode()->executeTick();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
