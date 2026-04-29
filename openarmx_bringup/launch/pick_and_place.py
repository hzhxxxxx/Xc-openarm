#!/usr/bin/env python3
"""
抓取工作流脚本
按顺序执行一系列动作，等待每个动作完成后再执行下一个
"""
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from openarmx_interfaces.msg import JointCommand
import threading
import time


class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')

        # 发布者
        self.right_gripper_pub = self.create_publisher(
            String, '/right_gripper_named_target', 10)
        self.right_arm_joint_pub = self.create_publisher(
            JointCommand, '/right_arm_joint_target', 10)

        # 订阅执行完成信号
        self.done_event = threading.Event()
        self.expected_done = None
        self.done_sub = self.create_subscription(
            String, '/moveit_execution_done', self.done_callback, 10)

        self.get_logger().info('节点已初始化，等待连接建立...')

        # 等待发布者与订阅者连接建立
        time.sleep(2.0)
        self.get_logger().info('连接已建立')

    def done_callback(self, msg):
        """执行完成回调"""
        self.get_logger().info(f'收到完成信号: {msg.data}')
        if self.expected_done is None or msg.data == self.expected_done:
            self.done_event.set()

    def wait_for_done(self, expected, timeout=60.0):
        """等待指定动作完成"""
        self.expected_done = expected
        self.done_event.clear()

        # 在等待期间保持spin
        start_time = self.get_clock().now()
        while not self.done_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().warn(f'等待 {expected} 超时')
                return False
        return True

    def move_gripper(self, target_name):
        """控制夹爪"""
        self.get_logger().info(f'夹爪动作: {target_name}')
        msg = String()
        msg.data = target_name
        self.right_gripper_pub.publish(msg)
        self.wait_for_done('right_gripper_named')
        self.get_logger().info(f'夹爪动作完成: {target_name}')
        time.sleep(2.0)  # 动作完成后等待2秒

    def move_arm_joint(self, joint_positions):
        """控制手臂到指定关节状态"""
        self.get_logger().info(f'手臂关节移动: {joint_positions}')
        msg = JointCommand()
        msg.joint_positions = joint_positions
        self.right_arm_joint_pub.publish(msg)
        self.wait_for_done('right_arm_joint')
        self.get_logger().info('手臂移动完成')
        time.sleep(2.0)  # 动作完成后等待2秒

    def run_workflow(self):
        """执行完整工作流"""
        self.get_logger().info('========== 开始执行工作流 ==========')

        # 步骤1: 打开夹爪
        self.get_logger().info('步骤 1/6: 打开夹爪')
        self.move_gripper('open')

        # 步骤2: 移动到位置1
        self.get_logger().info('步骤 2/6: 移动到位置1')
        self.move_arm_joint([-0.523055, 0.003644, -0.002877, 0.996048, 0.079599, 0.033566, 1.149876])

        # 步骤3: 移动到位置2
        self.get_logger().info('步骤 3/6: 移动到位置2')
        self.move_arm_joint([0.393010, 0.030114, -0.000192, 0.251841, 0.089190, 0.045074, 0.941959])

        # 步骤4: 半闭合夹爪（抓取）
        self.get_logger().info('步骤 4/6: 夹爪半闭合')
        self.move_gripper('half_closed')

        # 步骤5: 移动到位置3
        self.get_logger().info('步骤 5/6: 移动到位置3')
        self.move_arm_joint([0.036635, 0.044691, 0.025510, 1.121873, 0.112974, 0.010166, 0.427919])

        # 步骤6: 移动到位置4
        self.get_logger().info('步骤 6/6: 移动到位置4')
        self.move_arm_joint([-0.762812, 0.013618, 0.022825, 1.567630, 0.095328, 0.028579, 0.802708])

        self.get_logger().info('========== 工作流执行完成 ==========')


def main():
    rclpy.init()
    node = PickAndPlaceNode()

    try:
        node.run_workflow()
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
