from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pc_topic = LaunchConfiguration('pc_topic', default='/cloud_registered_body')
    frame_id = LaunchConfiguration('frame_id', default='camera_init')
    pc_file = LaunchConfiguration('pc_file', default='')
    pred_file = LaunchConfiguration('pred_file', default='')
    out_path = LaunchConfiguration('out_path', default='')
    out_origin_path = LaunchConfiguration('out_origin_path', default='')

    return LaunchDescription([
        DeclareLaunchArgument('pc_topic', default_value=pc_topic),
        DeclareLaunchArgument('frame_id', default_value=frame_id),
        DeclareLaunchArgument('pc_file', default_value=pc_file),
        DeclareLaunchArgument('pred_file', default_value=pred_file),
        DeclareLaunchArgument('out_path', default_value=out_path),
        DeclareLaunchArgument('out_origin_path', default_value=out_origin_path),

        # dynfilter_odom node (processing node)
        Node(
            package='m_detector',
            executable='dynfilter_odom',
            name='dynfilter_odom',
            output='screen',
            parameters=[{
                'dyn_obj.points_topic': pc_topic,
                'dyn_obj.odom_topic': '/aft_mapped_to_init',
                'dyn_obj.out_file': out_path,
                'dyn_obj.out_file_origin': out_origin_path
            }]
        ),

        # display_prediction node (visualization / viewer)
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
