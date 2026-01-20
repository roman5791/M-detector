#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Calculate recall launch file for M-detector
Launches the cal_recall node to calculate IoU and recall metrics
"""

import roslaunch
import rospy

def main():
    # Initialize roslaunch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    
    # Get launch arguments
    rviz = rospy.get_param('~rviz', False)
    dataset = rospy.get_param('~dataset', 0)
    dataset_folder = rospy.get_param('~dataset_folder', '')
    start_param = rospy.get_param('~start_param', -1)
    end_param = rospy.get_param('~end_param', -1)
    start_se = rospy.get_param('~start_se', -1)
    end_se = rospy.get_param('~end_se', -1)
    is_origin = rospy.get_param('~is_origin', False)
    
    # Create launch
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.parent = roslaunch.parent.ROSLaunchParent(uuid, [], is_core=False)
    launch.start()
    
    # Set parameters
    rospy.set_param('/dyn_obj/dataset', dataset)
    rospy.set_param('/dyn_obj/dataset_folder', dataset_folder)
    rospy.set_param('/dyn_obj/start_param', start_param)
    rospy.set_param('/dyn_obj/end_param', end_param)
    rospy.set_param('/dyn_obj/start_se', start_se)
    rospy.set_param('/dyn_obj/end_se', end_se)
    rospy.set_param('/dyn_obj/is_origin', is_origin)
    
    # Launch cal_recall node
    node = roslaunch.core.Node(
        package='m_detector',
        node_type='cal_recall',
        name='cal_recall',
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
