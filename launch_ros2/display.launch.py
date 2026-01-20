"""ROS2 launch file for M-Detector display prediction."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for display prediction."""
    
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
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
    
    # Config file path
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'kitti',
        'kitti.yaml'
    ])
    
    # Display prediction node
    display_node = Node(
        package='m_detector',
        executable='display_prediction_ros2',
        name='display_prediction',
        output='screen',
        parameters=[
            config_file,
            {
                'dyn_obj.pred_file': LaunchConfiguration('pred_file'),
                'dyn_obj.pc_file': LaunchConfiguration('pc_file'),
                'dyn_obj.pc_topic': LaunchConfiguration('pc_topic'),
            }
        ]
    )
    
    # RViz node
    rviz_config = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'display.rviz'
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
        pred_file_arg,
        pc_file_arg,
        pc_topic_arg,
        display_node,
        rviz_node,
    ])
