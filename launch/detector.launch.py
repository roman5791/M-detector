"""
ROS2 Launch file for M-detector
This launch file starts the dynfilter node for moving object detection
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
        default_value='avia',
        description='Dataset type: avia, kitti, nuscenes, or waymo'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='0',
        description='Config file number (0, 1, 2, etc.)'
    )
    
    time_file_arg = DeclareLaunchArgument(
        'time_file',
        default_value='',
        description='Time log file path'
    )
    
    out_path_arg = DeclareLaunchArgument(
        'out_path',
        default_value='',
        description='Output path for frame-out results'
    )
    
    out_origin_path_arg = DeclareLaunchArgument(
        'out_origin_path',
        default_value='',
        description='Output path for point-out results'
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
    
    # Construct config file path
    config_file_path = PathJoinSubstitution([
        pkg_share,
        'config',
        LaunchConfiguration('dataset'),
        [LaunchConfiguration('dataset'), LaunchConfiguration('config_file'), TextSubstitution(text='.yaml')]
    ])
    
    # Dynfilter node
    dynfilter_node = Node(
        package='m_detector',
        executable='dynfilter',
        name='dynfilter',
        output='screen',
        parameters=[
            config_file_path,
            {
                'dyn_obj/out_file': LaunchConfiguration('out_path'),
                'dyn_obj/out_file_origin': LaunchConfiguration('out_origin_path'),
                'dyn_obj/time_file': LaunchConfiguration('time_file'),
                'dyn_obj/pose_log': LaunchConfiguration('pose_log'),
                'dyn_obj/pose_log_file': LaunchConfiguration('pose_log_file'),
                'dyn_obj/cluster_out_file': LaunchConfiguration('cluster_out_file'),
                'dyn_obj/time_breakdown_file': LaunchConfiguration('time_breakdown_file'),
            }
        ]
    )
    
    # RViz node
    rviz_config_path = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'demo.rviz'
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
