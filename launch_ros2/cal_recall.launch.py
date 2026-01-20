from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments (defaults mirror the ROS1 launch)
    rviz = LaunchConfiguration('rviz', default='false')
    dataset = LaunchConfiguration('dataset', default='')            # int in node
    dataset_folder = LaunchConfiguration('dataset_folder', default='')
    start_param = LaunchConfiguration('start_param', default='-1')
    end_param = LaunchConfiguration('end_param', default='-1')
    start_se = LaunchConfiguration('start_se', default='-1')
    end_se = LaunchConfiguration('end_se', default='-1')
    is_origin = LaunchConfiguration('is_origin', default='false')  # bool in node
    prefix = LaunchConfiguration('prefix', default='')             # optional launch prefix (e.g. "gdb -ex run --args")

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='Launch rviz2 if true'),
        DeclareLaunchArgument('dataset', default_value=dataset),
        DeclareLaunchArgument('dataset_folder', default_value=dataset_folder),
        DeclareLaunchArgument('start_param', default_value=start_param),
        DeclareLaunchArgument('end_param', default_value=end_param),
        DeclareLaunchArgument('start_se', default_value=start_se),
        DeclareLaunchArgument('end_se', default_value=end_se),
        DeclareLaunchArgument('is_origin', default_value=is_origin),
        DeclareLaunchArgument('prefix', default_value=prefix, description='Optional command prefix'),

        # cal_recall node (converted executable name: cal_recall_multi)
        Node(
            package='m_detector',
            executable='cal_recall_multi',
            name='cal_recall',
            output='screen',
            prefix=prefix,
            parameters=[{
                'dyn_obj.dataset': dataset,
                'dyn_obj.is_origin': is_origin,
                'dyn_obj.dataset_folder': dataset_folder,
                'dyn_obj.start_param': start_param,
                'dyn_obj.end_param': end_param,
                'dyn_obj.start_se': start_se,
                'dyn_obj.end_se': end_se
            }]
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
                PathJoinSubstitution([FindPackageShare('m_detector'), 'rviz', 'display.rviz'])
            ]
        )
    ])
