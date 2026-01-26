from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch args (mirror ROS1 defaults)
    rviz = LaunchConfiguration('rviz', default='true')
    time_file = LaunchConfiguration('time_file', default='')
    out_path = LaunchConfiguration('out_path', default='')
    out_origin_path = LaunchConfiguration('out_origin_path', default='')
    pose_log = LaunchConfiguration('pose_log', default='false')
    pose_log_file = LaunchConfiguration('pose_log_file', default='')
    cluster_out_file = LaunchConfiguration('cluster_out_file', default='')
    time_breakdown_file = LaunchConfiguration('time_breakdown_file', default='')
    prefix = LaunchConfiguration('prefix', default='')  # optional launch prefix (e.g. "gdb -ex run --args")

    # config YAML and rviz paths in package share
    kitti_config = PathJoinSubstitution([FindPackageShare('m_detector'), 'config', 'kitti', 'kitti1.yaml'])
    rviz_config = PathJoinSubstitution([FindPackageShare('m_detector'), 'rviz', 'demo.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Launch rviz2 if true'),
        DeclareLaunchArgument('time_file', default_value=''),
        DeclareLaunchArgument('out_path', default_value=''),
        DeclareLaunchArgument('out_origin_path', default_value=''),
        DeclareLaunchArgument('pose_log', default_value='false'),
        DeclareLaunchArgument('pose_log_file', default_value=''),
        DeclareLaunchArgument('cluster_out_file', default_value=''),
        DeclareLaunchArgument('time_breakdown_file', default_value=''),
        DeclareLaunchArgument('prefix', default_value='', description='Optional command prefix'),

        # dynfilter node (mapped to ROS2 executable dynfilter_odom; keep node name 'dynfilter' for compatibility)
        Node(
            package='m_detector',
            executable='dynfilter_odom',
            name='dynfilter',
            output='screen',
            prefix=prefix,
            parameters=[
                # load YAML first, then overrides
                kitti_config,
                {
                    'dyn_obj.out_file': out_path,
                    'dyn_obj.out_file_origin': out_origin_path,
                    'dyn_obj.time_file': time_file,
                    'dyn_obj.pose_log': pose_log,
                    'dyn_obj.pose_log_file': pose_log_file,
                    'dyn_obj.cluster_out_file': cluster_out_file,
                    'dyn_obj.time_breakdown_file': time_breakdown_file
                }
            ]
        ),

        # optional RViz2 (only launched when rviz arg is true)
        Node(
            condition=IfCondition(rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            prefix='nice',
            arguments=[
                '-d',
                rviz_config
            ]
        )
    ])
