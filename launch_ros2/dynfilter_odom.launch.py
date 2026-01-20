from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pc_topic = LaunchConfiguration('pc_topic', default='/cloud_registered_body')
    odom_topic = LaunchConfiguration('odom_topic', default='/aft_mapped_to_init')
    out_path = LaunchConfiguration('out_path', default='')
    out_origin_path = LaunchConfiguration('out_origin_path', default='')

    return LaunchDescription([
        DeclareLaunchArgument('pc_topic', default_value=pc_topic),
        DeclareLaunchArgument('odom_topic', default_value=odom_topic),
        DeclareLaunchArgument('out_path', default_value=out_path),
        DeclareLaunchArgument('out_origin_path', default_value=out_origin_path),

        Node(
            package='m_detector',
            executable='dynfilter_odom',
            name='dynfilter_odom',
            output='screen',
            parameters=[{
                'dyn_obj.points_topic': pc_topic,
                'dyn_obj.odom_topic': odom_topic,
                'dyn_obj.out_file': out_path,
                'dyn_obj.out_file_origin': out_origin_path
            }]
        )
    ])
