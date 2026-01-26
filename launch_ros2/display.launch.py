from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rviz = LaunchConfiguration('rviz', default='true')
    pred_file = LaunchConfiguration('pred_file', default='')
    pc_file = LaunchConfiguration('pc_file', default='')
    pc_topic = LaunchConfiguration('pc_topic', default='')
    prefix = LaunchConfiguration('prefix', default='')

    kitti_config = PathJoinSubstitution([FindPackageShare('m_detector'), 'config', 'kitti', 'kitti.yaml'])
    rviz_config = PathJoinSubstitution([FindPackageShare('m_detector'), 'rviz', 'display.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Launch rviz2 if true'),
        DeclareLaunchArgument('pred_file', default_value=''),
        DeclareLaunchArgument('pc_file', default_value=''),
        DeclareLaunchArgument('pc_topic', default_value=''),
        DeclareLaunchArgument('prefix', default_value='', description='Optional command prefix'),

        Node(
            package='m_detector',
            executable='display_prediction',
            name='display_prediction',
            output='screen',
            prefix=prefix,
            parameters=[
                kitti_config,
                {
                    'dyn_obj.pred_file': pred_file,
                    'dyn_obj.pc_file': pc_file,
                    'dyn_obj.pc_topic': pc_topic
                }
            ]
        ),

        Node(
            condition=IfCondition(rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            prefix='nice',
            arguments=['-d', rviz_config]
        )
    ])
