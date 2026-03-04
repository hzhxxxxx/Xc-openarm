# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Launch MoveIt commander node for OpenArmX."""

    # 加载 MoveIt 配置（包含 robot_description 和 robot_description_semantic）
    moveit_config = MoveItConfigsBuilder(
        "openarm", package_name="openarmx_bimanual_moveit_config"
    ).to_moveit_configs()

    moveit_commander_node = Node(
        package="openarmx_commander",
        executable="moveit_commander",
        name="moveit_commander",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        moveit_commander_node,
    ])
