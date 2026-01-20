from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pc_file = LaunchConfiguration('pc_file', default='')
    pred_file = LaunchConfiguration('pred_file', default='')
    pc_topic = LaunchConfiguration('pc_topic', default='/velodyne_points')
    frame_id = LaunchConfiguration('frame_id', default='camera_init')

    return LaunchDescription([
        DeclareLaunchArgument('pc_file', default_value=pc_file),
        DeclareLaunchArgument('pred_file', default_value=pred_file),
        DeclareLaunchArgument('pc_topic', default_value=pc_topic),
        DeclareLaunchArgument('frame_id', default_value=frame_id),

        Node(
            package='m_detector',
            executable='display_prediction',
            name='display_prediction',
            output='screen',
            parameters=[{
                'dyn_obj.pc_file': pc_file,
                'dyn_obj.pred_file': pred_file,
                'dyn_obj.pc_topic': pc_topic,
                'dyn_obj.frame_id': frame_id
            }]
        )
    ])
