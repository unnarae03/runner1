#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>

// 각 노드 헤더 include
#include "runner/MoveToGoalNode.hpp"
#include "runner/CaptureImageNode.hpp"
#include "runner/SetAngleNode.hpp"
#include "runner/WaitForResultNode.hpp"
#include "runner/SaveResultNode.hpp"
#include "runner/SetFailureReasonNode.hpp"
#include "runner/ReturnToBaseNode.hpp"
#include "runner/SendMetaDataNode.hpp"
#include "runner/WaitForStartNode.hpp"

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
    factory.registerNodeType<SetFailureReasonNode>("SetFailureReason");
    factory.registerNodeType<ReturnToBaseNode>("ReturnToBase");
    factory.registerNodeType<SendMetaDataNode>("SendMetaData");
    factory.registerNodeType<WaitForStartNode>("WaitForStart");

    // 트리 로딩
    auto tree = factory.createTreeFromFile("behavior_trees/patrol_bt.xml");

    // ROS2 노드 생성
    auto node = rclcpp::Node::make_shared("bt_loop");

    rclcpp::Rate loop_rate(10); // 10 Hz

    while (rclcpp::ok())
    {
        // ✅ 최신 방식: 루트 노드를 직접 실행
        tree.rootNode()->executeTick();

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

