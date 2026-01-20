"""
ROS2 Launch file for calculating recall/IoU metrics
This launch file starts the cal_recall node for evaluating detection results
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz visualization'
    )
    
    dataset_arg = DeclareLaunchArgument(
        'dataset',
        default_value='',
        description='Dataset type: 0 for kitti, 1 for nuscenes, 2 for waymo, 3 for avia'
    )
    
    dataset_folder_arg = DeclareLaunchArgument(
        'dataset_folder',
        default_value='',
        description='Path to the dataset folder'
    )
    
    start_param_arg = DeclareLaunchArgument(
        'start_param',
        default_value='-1',
        description='First parameter file number for calculation'
    )
    
    end_param_arg = DeclareLaunchArgument(
        'end_param',
        default_value='-1',
        description='Last parameter file number for calculation'
    )
    
    start_se_arg = DeclareLaunchArgument(
        'start_se',
        default_value='-1',
        description='First sequence number for calculation'
    )
    
    end_se_arg = DeclareLaunchArgument(
        'end_se',
        default_value='-1',
        description='Last sequence number for calculation'
    )
    
    is_origin_arg = DeclareLaunchArgument(
        'is_origin',
        default_value='false',
        description='true for point-out results, false for frame-out results'
    )
    
    # Cal recall node
    cal_recall_node = Node(
        package='m_detector',
        executable='cal_recall',
        name='cal_recall',
        output='screen',
        parameters=[{
            'dyn_obj/dataset': LaunchConfiguration('dataset'),
            'dyn_obj/dataset_folder': LaunchConfiguration('dataset_folder'),
            'dyn_obj/start_param': LaunchConfiguration('start_param'),
            'dyn_obj/end_param': LaunchConfiguration('end_param'),
            'dyn_obj/start_se': LaunchConfiguration('start_se'),
            'dyn_obj/end_se': LaunchConfiguration('end_se'),
            'dyn_obj/is_origin': LaunchConfiguration('is_origin'),
        }]
    )
    
    # Get package share directory
    pkg_share = FindPackageShare('m_detector')
    
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
        dataset_folder_arg,
        start_param_arg,
        end_param_arg,
        start_se_arg,
        end_se_arg,
        is_origin_arg,
        cal_recall_node,
        rviz_node,
    ])
