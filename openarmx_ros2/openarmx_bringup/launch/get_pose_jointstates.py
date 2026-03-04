#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import JointState
import math
import sys

def quaternion_to_euler(x, y, z, w):
    """四元数转欧拉角"""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

class PoseJointReader(Node):
    def __init__(self):
        super().__init__('pose_joint_reader')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.joint_states = None
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

    def joint_callback(self, msg):
        self.joint_states = msg

    def get_tcp_pose(self, base_frame, tcp_frame):
        """获取TCP位姿"""
        try:
            # 等待 TF 可用，然后查询
            if not self.tf_buffer.can_transform(base_frame, tcp_frame, rclpy.time.Time()):
                # 如果不能直接变换，等待一下让 TF 缓存建立
                for _ in range(10):
                    rclpy.spin_once(self, timeout_sec=0.2)
                    if self.tf_buffer.can_transform(base_frame, tcp_frame, rclpy.time.Time()):
                        break

            trans = self.tf_buffer.lookup_transform(
                base_frame,
                tcp_frame,
                rclpy.time.Time()
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)

            return [x, y, z, roll, pitch, yaw]
        except Exception as e:
            self.get_logger().error(f"TF lookup failed for {base_frame} -> {tcp_frame}: {str(e)}")
            return None

    def get_arm_joints(self, arm_prefix):
        """获取手臂关节角度"""
        if self.joint_states is None:
            return None

        joint_names = [
            f'openarmx_{arm_prefix}_joint1',
            f'openarmx_{arm_prefix}_joint2',
            f'openarmx_{arm_prefix}_joint3',
            f'openarmx_{arm_prefix}_joint4',
            f'openarmx_{arm_prefix}_joint5',
            f'openarmx_{arm_prefix}_joint6',
            f'openarmx_{arm_prefix}_joint7',
        ]

        positions = []
        for name in joint_names:
            try:
                idx = self.joint_states.name.index(name)
                positions.append(self.joint_states.position[idx])
            except ValueError:
                return None

        return positions

def main():
    rclpy.init()
    node = PoseJointReader()

    # 等待数据（增加等待时间和次数）
    print("Waiting for data...")
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.joint_states is not None:
            break

    print("\n以下pose都是夹爪tcp在各自link0基坐标系下的坐标(moveit中posetarget也做了适配,其实也是夹爪tcp的坐标)")

    # 读取左臂
    print("\n=== LEFT ARM ===")
    left_pose = node.get_tcp_pose('openarmx_left_link0', 'openarmx_left_hand_tcp')
    left_joints = node.get_arm_joints('left')

    if left_pose:
        print(f"pose: [{left_pose[0]:.6f}, {left_pose[1]:.6f}, {left_pose[2]:.6f}, {left_pose[3]:.6f}, {left_pose[4]:.6f}, {left_pose[5]:.6f}]")
    else:
        print("pose: [unavailable]")

    if left_joints:
        joint_str = ", ".join([f"{j:.6f}" for j in left_joints])
        print(f"joint_states: [{joint_str}]")
    else:
        print("joint_states: [unavailable]")

    if left_pose:
        print(f'\npose_command:\nros2 topic pub --once /left_arm_pose_target openarmx_interfaces/msg/PoseCommand "{{x: {left_pose[0]:.6f}, y: {left_pose[1]:.6f}, z: {left_pose[2]:.6f}, roll: {left_pose[3]:.6f}, pitch: {left_pose[4]:.6f}, yaw: {left_pose[5]:.6f}, cartesian_path: false}}"')

    # 读取右臂
    print("\n=== RIGHT ARM ===")
    right_pose = node.get_tcp_pose('openarmx_right_link0', 'openarmx_right_hand_tcp')
    right_joints = node.get_arm_joints('right')

    if right_pose:
        print(f"pose: [{right_pose[0]:.6f}, {right_pose[1]:.6f}, {right_pose[2]:.6f}, {right_pose[3]:.6f}, {right_pose[4]:.6f}, {right_pose[5]:.6f}]")
    else:
        print("pose: [unavailable]")

    if right_joints:
        joint_str = ", ".join([f"{j:.6f}" for j in right_joints])
        print(f"joint_states: [{joint_str}]")
    else:
        print("joint_states: [unavailable]")

    if right_pose:
        print(f'\npose_command:\nros2 topic pub --once /right_arm_pose_target openarmx_interfaces/msg/PoseCommand "{{x: {right_pose[0]:.6f}, y: {right_pose[1]:.6f}, z: {right_pose[2]:.6f}, roll: {right_pose[3]:.6f}, pitch: {right_pose[4]:.6f}, yaw: {right_pose[5]:.6f}, cartesian_path: false}}"')

    print()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
