from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    dataset = LaunchConfiguration('dataset', default='-1')
    dataset_folder = LaunchConfiguration('dataset_folder', default='/')
    start_param = LaunchConfiguration('start_param', default='-1')
    end_param = LaunchConfiguration('end_param', default='0')
    start_se = LaunchConfiguration('start_se', default='-1')
    end_se = LaunchConfiguration('end_se', default='0')
    is_origin = LaunchConfiguration('is_origin', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('dataset', default_value=dataset),
        DeclareLaunchArgument('dataset_folder', default_value=dataset_folder),
        DeclareLaunchArgument('start_param', default_value=start_param),
        DeclareLaunchArgument('end_param', default_value=end_param),
        DeclareLaunchArgument('start_se', default_value=start_se),
        DeclareLaunchArgument('end_se', default_value=end_se),
        DeclareLaunchArgument('is_origin', default_value=is_origin),

        Node(
            package='m_detector',
            executable='cal_recall_multi',
            name='cal_recall_multi',
            output='screen',
            parameters=[{
                'dyn_obj.dataset': dataset,
                'dyn_obj.is_origin': is_origin,
                'dyn_obj.dataset_folder': dataset_folder,
                'dyn_obj.start_param': start_param,
                'dyn_obj.end_param': end_param,
                'dyn_obj.start_se': start_se,
                'dyn_obj.end_se': end_se
            }]
        )
    ])
