#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
给定相机检测到的目标物体6D位姿，计算目标在机械臂基坐标系下的位姿
变换链: T_base_obj = T_base_end × gHc × T_camera_obj
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import math


# ===================== 手眼标定矩阵 gHc（相机相对于末端）=====================
# 原始平移单位为mm，这里直接除以1000转为m
GHC = np.array([
    [-5.47867133e-02, -8.31994131e-01, -5.52072624e-01,  34.0596055e-03],
    [ 9.97954728e-01, -2.73881686e-02, -5.77602703e-02,   4.57252503e-03],
    [ 3.29359478e-02, -5.54107981e-01,  8.31792985e-01, -76.6015754e-03],
    [ 0.0,             0.0,             0.0,             1.0            ]
])

# 标定结果中的旋转角（绕末端Z轴的安装偏差），用于模式2抵消
# angle=97.1918°，抵消方向取负
GHC_Z_OFFSET_RAD = -math.radians(97.1918)
# ==========================================================================


# ===================== 末端位置补偿（实测误差修正）========================
# 在最终结果上叠加固定偏移，补偿手眼标定残差（相对于基坐标系，x轴朝前；y轴朝上；z轴朝右）
OFFSET_X =  -0.009  # m
OFFSET_Y =   0.087  # m
OFFSET_Z =   0.000  # m
# ==========================================================================


def rot_z(angle_rad):
    """绕Z轴旋转的4x4齐次矩阵（仅旋转，平移为0）"""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([
        [ c, -s, 0.0, 0.0],
        [ s,  c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])


def euler_to_matrix(roll, pitch, yaw):
    """RPY欧拉角（弧度）转4x4齐次旋转矩阵（平移为0）"""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)

    R = np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr            ]
    ])

    T = np.eye(4)
    T[:3, :3] = R
    return T


def matrix_to_euler(T):
    """4x4齐次矩阵提取RPY欧拉角（弧度）"""
    R = T[:3, :3]
    pitch = math.asin(-R[2, 0])
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw  = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw  = 0.0
    return roll, pitch, yaw


def quaternion_to_euler(x, y, z, w):
    """四元数转RPY欧拉角（弧度）"""
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


def pose6d_to_matrix(x, y, z, roll, pitch, yaw):
    """6D位姿（位置m，角度rad）转4x4齐次矩阵"""
    T = euler_to_matrix(roll, pitch, yaw)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def print_result(res_x, res_y, res_z, res_roll, res_pitch, res_yaw):
    """统一输出结果"""
    print("\n========== 计算结果：目标物体在基坐标系下的位姿 ==========")
    print(f"  x     = {res_x:.4f} m")
    print(f"  y     = {res_y:.4f} m")
    print(f"  z     = {res_z:.4f} m")
    print(f"  roll  = {res_roll:.4f} rad  ({math.degrees(res_roll):.2f}°)")
    print(f"  pitch = {res_pitch:.4f} rad  ({math.degrees(res_pitch):.2f}°)")
    print(f"  yaw   = {res_yaw:.4f} rad  ({math.degrees(res_yaw):.2f}°)")
    print("==========================================================")
    print(f'\nros2 topic pub --once /right_arm_pose_target openarmx_interfaces/msg/PoseCommand "{{x: {res_x:.6f}, y: {res_y:.6f}, z: {res_z:.6f}, roll: {res_roll:.6f}, pitch: {res_pitch:.6f}, yaw: {res_yaw:.6f}, cartesian_path: false}}"')

    # 欧拉角转四元数，用于static_transform_publisher
    cr, sr = math.cos(res_roll/2),  math.sin(res_roll/2)
    cp, sp = math.cos(res_pitch/2), math.sin(res_pitch/2)
    cy, sy = math.cos(res_yaw/2),   math.sin(res_yaw/2)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    print(f'\nros2 run tf2_ros static_transform_publisher {res_x:.6f} {res_y:.6f} {res_z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f} openarmx_right_link0 target_object')


class PoseCalculator(Node):
    def __init__(self):
        super().__init__('camera_to_base_calculator')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_T_base_end(self):
        """从TF读取末端在基坐标系下的4x4变换矩阵（单位m）"""
        base_frame = 'openarmx_right_link0'
        tcp_frame  = 'openarmx_right_hand_tcp'

        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.tf_buffer.can_transform(base_frame, tcp_frame, rclpy.time.Time()):
                break

        trans = self.tf_buffer.lookup_transform(base_frame, tcp_frame, rclpy.time.Time())

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.w

        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        return pose6d_to_matrix(x, y, z, roll, pitch, yaw)


def main():
    rclpy.init()
    node = PoseCalculator()

    # ---------- 选择模式 ----------
    print("请选择输出模式:")
    print("  1 - 只取xyz（姿态保持当前末端姿态）")
    print("  2 - 完整6D位姿（xyz + 旋转全部计算，并抵消gHc绕Z轴安装偏差）")
    mode = input("请输入 1 或 2: ").strip()
    while mode not in ('1', '2'):
        mode = input("输入有误，请输入 1 或 2: ").strip()

    # ---------- 输入：相机检测到的目标物体6D位姿 ----------
    # 位置单位：mm（会自动转m）；角度单位：rad
    print("请输入相机检测到的目标物体6D位姿:")
    cam_x     = float(input("  x (mm): "))
    cam_y     = float(input("  y (mm): "))
    cam_z     = float(input("  z (mm): "))
    cam_roll  = float(input("  roll  (rad): "))
    cam_pitch = float(input("  pitch (rad): "))
    cam_yaw   = float(input("  yaw   (rad): "))

    # mm -> m
    cam_x /= 1000.0
    cam_y /= 1000.0
    cam_z /= 1000.0

    # ---------- 步骤1：构造相机坐标系下目标矩阵 ----------
    T_camera_obj = pose6d_to_matrix(cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw)

    # ---------- 步骤2：从TF读取末端在基坐标系下的矩阵 ----------
    print("\n正在读取机械臂末端位姿...")
    T_base_end = node.get_T_base_end()
    end_roll, end_pitch, end_yaw = matrix_to_euler(T_base_end)
    print(f"末端位姿(base系): x={T_base_end[0,3]:.4f}m  y={T_base_end[1,3]:.4f}m  z={T_base_end[2,3]:.4f}m")
    print(f"                  roll={math.degrees(end_roll):.2f}°  pitch={math.degrees(end_pitch):.2f}°  yaw={math.degrees(end_yaw):.2f}°")

    # ---------- 步骤3：完整变换链 ----------
    # T_base_obj = T_base_end × gHc × T_camera_obj
    T_base_obj = T_base_end @ GHC @ T_camera_obj

    # ---------- 输出结果 ----------
    res_x = T_base_obj[0, 3] + OFFSET_X
    res_y = T_base_obj[1, 3] + OFFSET_Y
    res_z = T_base_obj[2, 3] + OFFSET_Z

    if mode == '1':
        res_roll, res_pitch, res_yaw = end_roll, end_pitch, end_yaw
        print("\n  [模式1] 姿态保持当前末端姿态不变")
        print_result(res_x, res_y, res_z, res_roll, res_pitch, res_yaw)

    else:
        # 模式2：在结果坐标系下绕末端Z轴旋转抵消gHc安装偏差
        # T_base_end的旋转部分提取出来，构造绕末端Z轴的修正矩阵
        # 修正方式：T_corrected = T_base_obj × R_end_z(offset)
        # 即在目标坐标系的Z轴方向上旋转，抵消安装偏差，位置不变
        R_end_z = rot_z(GHC_Z_OFFSET_RAD)
        T_corrected = T_base_obj @ R_end_z

        res_roll, res_pitch, res_yaw = matrix_to_euler(T_corrected)
        # 位置用原始结果（旋转修正不改变位置）
        print(f"\n  [模式2] 完整6D位姿，已抵消gHc绕Z轴安装偏差 ({math.degrees(GHC_Z_OFFSET_RAD):.2f}°)")
        print_result(res_x, res_y, res_z, res_roll, res_pitch, res_yaw)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
