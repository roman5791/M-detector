#!/usr/bin/env python3
"""
ROS2 Launch file for display_prediction node

This launch file demonstrates how to run the display_prediction node
with parameters in ROS2.

Example usage:
    ros2 launch m_detector display_prediction.launch.py \
        pc_topic:=/velodyne_points \
        pred_file:=/path/to/predictions/ \
        frame_id:=camera_init
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    pc_topic_arg = DeclareLaunchArgument(
        'pc_topic',
        default_value='/velodyne_points',
        description='Input point cloud topic'
    )
    
    pred_file_arg = DeclareLaunchArgument(
        'pred_file',
        default_value='',
        description='Path to folder containing .label prediction files'
    )
    
    pc_file_arg = DeclareLaunchArgument(
        'pc_file',
        default_value='',
        description='Path to point cloud files (optional)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_init',
        description='TF frame ID for published messages'
    )

    # Create the display_prediction node
    display_prediction_node = Node(
        package='m_detector',
        executable='display_prediction',
        name='display_prediction',
        output='screen',
        parameters=[{
            'dyn_obj.pc_topic': LaunchConfiguration('pc_topic'),
            'dyn_obj.pred_file': LaunchConfiguration('pred_file'),
            'dyn_obj.pc_file': LaunchConfiguration('pc_file'),
            'dyn_obj.frame_id': LaunchConfiguration('frame_id'),
        }]
    )

    return LaunchDescription([
        pc_topic_arg,
        pred_file_arg,
        pc_file_arg,
        frame_id_arg,
        display_prediction_node,
    ])
