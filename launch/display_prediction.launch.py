#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Display prediction launch file for M-detector
Launches the display_prediction node to visualize prediction results
"""

import roslaunch
import rospy
import os

def main():
    # Initialize roslaunch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    
    # Get launch arguments
    rviz = rospy.get_param('~rviz', True)
    config_file = rospy.get_param('~config_file', '')
    pred_file = rospy.get_param('~pred_file', '')
    pc_file = rospy.get_param('~pc_file', '')
    pc_topic = rospy.get_param('~pc_topic', '')
    
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
    if pred_file:
        rospy.set_param('/dyn_obj/pred_file', pred_file)
    if pc_file:
        rospy.set_param('/dyn_obj/pc_file', pc_file)
    if pc_topic:
        rospy.set_param('/dyn_obj/pc_topic', pc_topic)
    
    # Launch display_prediction node
    node = roslaunch.core.Node(
        package='m_detector',
        node_type='display_prediction',
        name='display_prediction',
        output='screen'
    )
    process = launch.launch(node)
    
    # Launch RViz if requested
    if rviz:
        rviz_config = roslaunch.substitution_args.resolve_args(
            "$(find m_detector)/rviz/display.rviz"
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
