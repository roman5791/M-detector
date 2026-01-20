"""ROS2 launch file for M-Detector KITTI dataset."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for M-Detector with KITTI dataset."""
    
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
    )
    
    time_file_arg = DeclareLaunchArgument(
        'time_file',
        default_value='',
        description='Time file path'
    )
    
    out_path_arg = DeclareLaunchArgument(
        'out_path',
        default_value='',
        description='Output path for results'
    )
    
    out_origin_path_arg = DeclareLaunchArgument(
        'out_origin_path',
        default_value='',
        description='Output path for origin results'
    )
    
    pose_log_arg = DeclareLaunchArgument(
        'pose_log',
        default_value='false',
        description='Enable pose logging'
    )
    
    pose_log_file_arg = DeclareLaunchArgument(
        'pose_log_file',
        default_value='',
        description='Pose log file path'
    )
    
    cluster_out_file_arg = DeclareLaunchArgument(
        'cluster_out_file',
        default_value='',
        description='Cluster output file path'
    )
    
    time_breakdown_file_arg = DeclareLaunchArgument(
        'time_breakdown_file',
        default_value='',
        description='Time breakdown file path'
    )
    
    # Get package share directory
    pkg_share = FindPackageShare('m_detector')
    
    # Config file path
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'kitti',
        'kitti1.yaml'
    ])
    
    # DynFilter node
    dynfilter_node = Node(
        package='m_detector',
        executable='dynfilter_ros2',
        name='dynfilter',
        output='screen',
        parameters=[
            config_file,
            {
                'dyn_obj.out_file': LaunchConfiguration('out_path'),
                'dyn_obj.out_file_origin': LaunchConfiguration('out_origin_path'),
                'dyn_obj.time_file': LaunchConfiguration('time_file'),
                'dyn_obj.pose_log': LaunchConfiguration('pose_log'),
                'dyn_obj.pose_log_file': LaunchConfiguration('pose_log_file'),
                'dyn_obj.cluster_out_file': LaunchConfiguration('cluster_out_file'),
                'dyn_obj.time_breakdown_file': LaunchConfiguration('time_breakdown_file'),
            }
        ]
    )
    
    # RViz node
    rviz_config = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'demo.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        rviz_arg,
        time_file_arg,
        out_path_arg,
        out_origin_path_arg,
        pose_log_arg,
        pose_log_file_arg,
        cluster_out_file_arg,
        time_breakdown_file_arg,
        dynfilter_node,
        rviz_node,
    ])
