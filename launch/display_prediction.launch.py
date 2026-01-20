"""
ROS2 Launch file for displaying M-detector predictions
This launch file starts the display_prediction node for visualizing detection results
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
    )
    
    dataset_arg = DeclareLaunchArgument(
        'dataset',
        default_value='kitti',
        description='Dataset type: avia, kitti, nuscenes, or waymo'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Config file name (e.g., kitti.yaml)'
    )
    
    pred_file_arg = DeclareLaunchArgument(
        'pred_file',
        default_value='',
        description='Prediction file path'
    )
    
    pc_file_arg = DeclareLaunchArgument(
        'pc_file',
        default_value='',
        description='Point cloud file path'
    )
    
    pc_topic_arg = DeclareLaunchArgument(
        'pc_topic',
        default_value='',
        description='Point cloud topic name'
    )
    
    # Get package share directory
    pkg_share = FindPackageShare('m_detector')
    
    # Construct config file path
    config_file_path = PathJoinSubstitution([
        pkg_share,
        'config',
        LaunchConfiguration('dataset'),
        [LaunchConfiguration('dataset'), TextSubstitution(text='.yaml')]
    ])
    
    # Display prediction node
    display_node = Node(
        package='m_detector',
        executable='display_prediction',
        name='display_prediction',
        output='screen',
        parameters=[
            config_file_path,
            {
                'pred_file': LaunchConfiguration('pred_file'),
                'pc_file': LaunchConfiguration('pc_file'),
                'pc_topic': LaunchConfiguration('pc_topic'),
            }
        ]
    )
    
    # RViz node
    rviz_config_path = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'display.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )
    
    return LaunchDescription([
        rviz_arg,
        dataset_arg,
        config_file_arg,
        pred_file_arg,
        pc_file_arg,
        pc_topic_arg,
        display_node,
        rviz_node,
    ])
