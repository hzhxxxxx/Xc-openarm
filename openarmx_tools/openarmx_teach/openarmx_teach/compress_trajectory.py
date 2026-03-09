#!/usr/bin/env python3
"""
轨迹压缩工具：去除静止段，保留运动段
用法: ros2 run openarmx_teach compress_trajectory <input.yaml> <output.yaml>
"""

import argparse
import yaml
import numpy as np
from typing import List, Dict


def detect_motion_segments(points: List[Dict],
                          position_threshold: float = 0.005,
                          min_static_frames: int = 10) -> tuple:
    """
    检测运动段和静止段

    Args:
        points: 轨迹点列表
        position_threshold: 位置变化阈值（弧度），小于此值认为静止
        min_static_frames: 最少连续静止帧数才被认为是"停顿"

    Returns:
        (keep_mask, static_segments):
            - keep_mask: 布尔列表，True表示该帧应保留
            - static_segments: 静止段列表，每个元素为(start_idx, end_idx)
    """
    n = len(points)
    if n < 2:
        return [True] * n, []

    # 计算每帧的运动幅度（相对上一帧）
    motion_magnitudes = [0.0]  # 第一帧默认为0
    for i in range(1, n):
        pos_curr = np.array(points[i]['positions'])
        pos_prev = np.array(points[i-1]['positions'])
        diff = np.linalg.norm(pos_curr - pos_prev)
        motion_magnitudes.append(diff)

    # 标记运动/静止
    is_moving = [mag > position_threshold for mag in motion_magnitudes]

    # 查找连续静止段
    keep_mask = [True] * n  # 默认全保留
    static_segments = []  # 记录静止段的起止索引
    static_start = None

    for i in range(n):
        if not is_moving[i]:
            if static_start is None:
                static_start = i
        else:
            if static_start is not None:
                static_length = i - static_start
                if static_length >= min_static_frames:
                    # 这是一个长停顿段，只保留首尾帧
                    static_segments.append((static_start, i - 1))
                    for j in range(static_start + 1, i):
                        keep_mask[j] = False
                static_start = None

    # 处理末尾静止段
    if static_start is not None:
        static_length = n - static_start
        if static_length >= min_static_frames:
            static_segments.append((static_start, n - 1))
            for j in range(static_start + 1, n):
                keep_mask[j] = False

    # 总是保留第一帧和最后一帧
    keep_mask[0] = True
    keep_mask[-1] = True

    return keep_mask, static_segments


def compress_trajectory(input_file: str,
                       output_file: str,
                       position_threshold: float = 0.005,
                       min_static_frames: int = 10,
                       resample_dt: float = None,
                       pause_duration: float = 0.0) -> None:
    """
    压缩轨迹：去除静止段

    Args:
        input_file: 输入YAML文件
        output_file: 输出YAML文件
        position_threshold: 位置变化阈值
        min_static_frames: 最少连续静止帧数
        resample_dt: 重采样时间间隔（秒），None表示不重采样
        pause_duration: 在每个静止段保留的停顿时长（秒），0表示完全去除
    """
    # 加载数据
    with open(input_file, 'r') as f:
        data = yaml.safe_load(f)

    joint_names = data['joint_names']
    points = data['points']

    print(f"原始轨迹: {len(points)} 个点")

    # 检测运动段
    keep_mask, static_segments = detect_motion_segments(points, position_threshold, min_static_frames)

    # 过滤点
    filtered_points = [p for i, p in enumerate(points) if keep_mask[i]]
    print(f"去除静止段后: {len(filtered_points)} 个点 (保留 {100*len(filtered_points)/len(points):.1f}%)")
    print(f"检测到 {len(static_segments)} 个静止段")

    # 重新分配时间戳（压缩时间 + 插入停顿）
    if resample_dt is not None:
        # 均匀重采样
        new_points = []
        for i, p in enumerate(filtered_points):
            new_points.append({
                'positions': p['positions'],
                'time_from_start': (i + 1) * resample_dt
            })
        filtered_points = new_points
        total_time = len(filtered_points) * resample_dt
        print(f"重采样到均匀时间间隔 {resample_dt}s，总时长: {total_time:.2f}s")
    else:
        # 保持原时间间隔比例，但压缩到更短的总时长
        # 同时在静止段的边界插入固定停顿
        original_times = [p['time_from_start'] for p in filtered_points]
        if len(original_times) > 1:
            time_diffs = [original_times[i+1] - original_times[i]
                         for i in range(len(original_times)-1)]
            avg_dt = np.mean(time_diffs)

            # 建立原始索引到过滤后索引的映射
            original_indices = [i for i, keep in enumerate(keep_mask) if keep]

            # 标记哪些过滤后的点对应静止段（在其后加停顿）
            # 逻辑：静止段的起始帧保留了，在该帧后加停顿
            pause_after = [False] * len(filtered_points)
            for start_idx, end_idx in static_segments:
                # 静止段的起始点（start_idx）应该被保留了
                try:
                    filtered_idx = original_indices.index(start_idx)
                    if filtered_idx < len(filtered_points) - 1:  # 不在最后一个点加停顿
                        pause_after[filtered_idx] = True
                        if pause_duration > 0:
                            print(f"  静止段 [{start_idx}:{end_idx}] -> 在过滤后索引 {filtered_idx} (原始索引{start_idx}) 后加 {pause_duration}s 停顿")
                except ValueError:
                    if pause_duration > 0:
                        print(f"  静止段 [{start_idx}:{end_idx}] -> 起始索引未找到，跳过")

            new_points = []
            cumulative_time = 0.0
            for i, p in enumerate(filtered_points):
                if i == 0:
                    cumulative_time = avg_dt
                else:
                    # 保持相对时间间隔比例
                    original_dt = original_times[i] - original_times[i-1]
                    cumulative_time += min(original_dt, avg_dt * 3)  # 限制最大间隔

                    # 如果上一个点是静止段结束点，加入停顿时间
                    if pause_after[i-1] and pause_duration > 0:
                        cumulative_time += pause_duration

                new_points.append({
                    'positions': p['positions'],
                    'time_from_start': float(cumulative_time)
                })
            filtered_points = new_points

            pause_info = f"，插入了 {sum(pause_after)} 个 {pause_duration}s 的停顿" if pause_duration > 0 else ""
            print(f"压缩后总时长: {cumulative_time:.2f}s (原始: {original_times[-1]:.2f}s){pause_info}")

    # 保存
    output_data = {
        'joint_names': joint_names,
        'points': filtered_points
    }

    with open(output_file, 'w') as f:
        yaml.safe_dump(output_data, f, sort_keys=False)

    print(f"✓ 已保存到: {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description='压缩轨迹：去除静止段，缩短总时长'
    )
    parser.add_argument('input', help='输入YAML文件')
    parser.add_argument('output', help='输出YAML文件')
    parser.add_argument('--threshold', type=float, default=0.005,
                       help='位置变化阈值（弧度），默认0.005')
    parser.add_argument('--min-static-frames', type=int, default=10,
                       help='最少连续静止帧数，默认10')
    parser.add_argument('--resample-dt', type=float, default=None,
                       help='重采样时间间隔（秒），默认None（保持原比例）')
    parser.add_argument('--pause-duration', type=float, default=0.0,
                       help='在每个静止段保留的停顿时长（秒），默认0（完全去除）')

    args = parser.parse_args()

    compress_trajectory(
        args.input,
        args.output,
        position_threshold=args.threshold,
        min_static_frames=args.min_static_frames,
        resample_dt=args.resample_dt,
        pause_duration=args.pause_duration
    )


if __name__ == '__main__':
    main()
