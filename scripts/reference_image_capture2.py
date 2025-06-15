import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge

import os
import yaml
import cv2
import time

from rclpy.action import ActionClient
from tf_transformations import euler_from_quaternion

class ReferenceImageCapture(Node):
    def __init__(self):
        super().__init__('reference_image_capture')

        # 👉 Nav2 Action Client 생성
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_action_client.wait_for_server()

        # 👉 이미지 수신 준비
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_sub = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )

        # 👉 Pose 수신 준비
        self.current_pose = None
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # ✅ 경로 설정
        self.image_dir = os.path.expanduser('~/ros2_ws/src/runner1/logs/images/reference')
        os.makedirs(self.image_dir, exist_ok=True)
        self.pose_dir = os.path.expanduser('~/ros2_ws/src/runner1/config')
        os.makedirs(self.pose_dir, exist_ok=True)

        self.image_saved = False
        self.get_logger().info('📸 ReferenceImageCapture Node Ready — waiting for result...')

        # 👉 Nav2 Goal 결과 수신을 위한 타이머 (폴링 방식)
        self.timer = self.create_timer(0.5, self.check_nav2_result)

        self.nav_result_received = False

    def image_callback(self, msg):
        self.latest_image = msg

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def check_nav2_result(self):
        if not self.nav_result_received:
            return

        if self.image_saved:
            return

        if self.latest_image is None or self.current_pose is None:
            self.get_logger().warn('⚠️ Image or Pose not ready. Waiting...')
            return

        # 👉 저장
        self.save_image_and_pose()
        self.image_saved = True

        self.get_logger().info('🎉 Done. Shutting down node.')
        rclpy.shutdown()

    def save_image_and_pose(self):
        # ✅ 인덱스 계산
        image_index = len([f for f in os.listdir(self.image_dir) if f.endswith('.jpg')]) + 1
        pose_index = len([f for f in os.listdir(self.pose_dir) if f.startswith('waypoint')]) + 1

        # ✅ 이미지 저장
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        image_path = os.path.join(self.image_dir, f'{image_index}.jpg')
        cv2.imwrite(image_path, cv_image)
        self.get_logger().info(f'✅ Saved image: {image_path}')

        # ✅ 쿼터니언 → yaw 변환
        q = [
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(q)

        # ✅ 포즈 저장
        pose_data = {
            'pose': {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                'theta': yaw
            }
        }

        pose_path = os.path.join(self.pose_dir, f'waypoint{pose_index}.yaml')
        with open(pose_path, 'w') as f:
            yaml.dump(pose_data, f)
        self.get_logger().info(f'✅ Saved pose: {pose_path}')

    def send_dummy_goal_and_wait(self):
        # ✅ 현재 목적은 Goal 도달 감지이므로 실제 Goal은 RViZ에서 보내고, 우리는 결과만 구독
        self.get_logger().info('📡 Waiting for Nav2 goal to complete...')
        future = self.nav_action_client.send_goal_async(NavigateToPose.Goal())
        future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal was rejected')
            return
        self.get_logger().info('🚀 Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_goal_result_callback)

    def nav_goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('🏁 Nav2 goal reached — triggering image save')
        self.nav_result_received = True

def main(args=None):
    rclpy.init(args=args)
    node = ReferenceImageCapture()
    node.send_dummy_goal_and_wait()  # RViZ goal과 연결되지는 않지만 Action Client 작동하게 함
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()

