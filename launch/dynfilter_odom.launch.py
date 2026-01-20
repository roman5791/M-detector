#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Dynfilter with odometry launch file for M-detector
Launches the dynfilter node with odometry support
"""

import roslaunch
import rospy
import sys
import os

def main():
    # Initialize roslaunch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    
    # Get launch arguments
    rviz = rospy.get_param('~rviz', True)
    config_file = rospy.get_param('~config_file', '')
    time_file = rospy.get_param('~time_file', '')
    out_path = rospy.get_param('~out_path', '')
    out_origin_path = rospy.get_param('~out_origin_path', '')
    pose_log = rospy.get_param('~pose_log', False)
    pose_log_file = rospy.get_param('~pose_log_file', '')
    cluster_out_file = rospy.get_param('~cluster_out_file', '')
    time_breakdown_file = rospy.get_param('~time_breakdown_file', '')
    
    # Create launch
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.parent = roslaunch.parent.ROSLaunchParent(uuid, [], is_core=False)
    launch.start()
    
    # Load config file if specified
    if config_file:
        if os.path.exists(config_file):
            import rosparam
            rosparam.load_file(config_file, default_namespace='/')
        else:
            rospy.logwarn(f"Config file not found: {config_file}")
    
    # Set parameters
    rospy.set_param('/dyn_obj/out_file', out_path)
    rospy.set_param('/dyn_obj/out_file_origin', out_origin_path)
    rospy.set_param('/dyn_obj/time_file', time_file)
    rospy.set_param('/dyn_obj/pose_log', pose_log)
    rospy.set_param('/dyn_obj/pose_log_file', pose_log_file)
    rospy.set_param('/dyn_obj/cluster_out_file', cluster_out_file)
    rospy.set_param('/dyn_obj/time_breakdown_file', time_breakdown_file)
    
    # Launch dynfilter node with odometry
    node = roslaunch.core.Node(
        package='m_detector',
        node_type='dynfilter',
        name='dynfilter',
        output='screen'
    )
    process = launch.launch(node)
    
    # Launch RViz if requested
    if rviz:
        rviz_config = roslaunch.substitution_args.resolve_args(
            "$(find m_detector)/rviz/demo.rviz"
        )
        rviz_node = roslaunch.core.Node(
            package='rviz',
            node_type='rviz',
            name='rviz',
            args=f'-d {rviz_config}',
            launch_prefix='nice'
        )
        rviz_process = launch.launch(rviz_node)
    
    # Keep running
    try:
        launch.spin()
    finally:
        launch.shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
