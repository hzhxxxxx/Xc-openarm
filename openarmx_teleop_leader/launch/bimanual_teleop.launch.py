#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenArm 双臂遥操作启动文件
从主动端双臂（can0, can1）读取位置，发送到从动端双臂
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明启动参数
    leader_right_can_arg = DeclareLaunchArgument(
        'leader_right_can',
        default_value='can0',
        description='主动端右臂CAN接口 (默认: can0)'
    )

    leader_left_can_arg = DeclareLaunchArgument(
        'leader_left_can',
        default_value='can1',
        description='主动端左臂CAN接口 (默认: can1)'
    )

    follower_right_prefix_arg = DeclareLaunchArgument(
        'follower_right_prefix',
        default_value='right',
        description='从动端右臂前缀 (默认: right)'
    )

    follower_left_prefix_arg = DeclareLaunchArgument(
        'follower_left_prefix',
        default_value='left',
        description='从动端左臂前缀 (默认: left)'
    )

    control_rate_hz_arg = DeclareLaunchArgument(
        'control_rate_hz',
        default_value='200',
        description='控制循环频率 (Hz, 默认: 200)'
    )

    # 创建遥操作节点
    teleop_node = Node(
        package='openarmx_teleop_leader',
        executable='teleop_leader_node',
        name='teleop_leader_node',
        output='screen',
        parameters=[{
            'leader_right_can': LaunchConfiguration('leader_right_can'),
            'leader_left_can': LaunchConfiguration('leader_left_can'),
            'follower_right_prefix': LaunchConfiguration('follower_right_prefix'),
            'follower_left_prefix': LaunchConfiguration('follower_left_prefix'),
            'control_rate_hz': LaunchConfiguration('control_rate_hz'),
        }]
    )

    return LaunchDescription([
        leader_right_can_arg,
        leader_left_can_arg,
        follower_right_prefix_arg,
        follower_left_prefix_arg,
        control_rate_hz_arg,
        teleop_node,
    ])
