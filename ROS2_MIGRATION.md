# M-Detector ROS2 Migration

This directory contains the ROS2 Jazzy migration of the M-Detector package.

## Migration Overview

This migration maintains the original ROS1 source files while adding ROS2-specific versions:

### ROS2 Build Files
- `package_ros2.xml` - ROS2 package manifest (format 3)
- `CMakeLists_ros2.txt` - ament_cmake build configuration

### ROS2 Node Sources
- `src/display_prediction_ros2.cpp` - ROS2 version of display_prediction node
- `src/dynfilter_with_odom_ros2.cpp` - ROS2 version of dynfilter node
- `src/cal_recall_multi_ros2.cpp` - ROS2 version of cal_recall node
- `src/DynObjFilter_ros2.cpp` - ROS2 wrapper for DynObjFilter class
- `include/m-detector/DynObjFilter_ros2.h` - ROS2 header for DynObjFilter wrapper

### ROS2 Launch Files
- `launch_ros2/detector_kitti.launch.py` - Python launch file for KITTI detector
- `launch_ros2/display.launch.py` - Python launch file for display prediction

## Building for ROS2

To build this package for ROS2 Jazzy:

1. **Backup the ROS1 files**:
   ```bash
   cp package.xml package_ros1.xml
   cp CMakeLists.txt CMakeLists_ros1.txt
   ```

2. **Use the ROS2 build files**:
   ```bash
   cp package_ros2.xml package.xml
   cp CMakeLists_ros2.txt CMakeLists.txt
   ```

3. **Source ROS2 Jazzy**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

4. **Build with colcon**:
   ```bash
   cd /path/to/workspace
   colcon build --packages-select m_detector
   ```

5. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

## Running ROS2 Nodes

### DynFilter Node
```bash
ros2 launch m_detector detector_kitti.launch.py
```

### Display Prediction Node
```bash
ros2 launch m_detector display.launch.py
```

### Cal Recall Node
```bash
ros2 run m_detector cal_recall_ros2
```

## Key Differences from ROS1

### API Changes
- `ros::NodeHandle` → `rclcpp::Node`
- `ros::Publisher` → `rclcpp::Publisher<T>::SharedPtr`
- `ros::Subscriber` → `rclcpp::Subscription<T>::SharedPtr`
- `ros::Timer` → `rclcpp::TimerBase::SharedPtr`
- `ros::Time::now()` → `node->now()`
- `tf::` → `tf2::`

### Parameter Handling
- `nh.param<T>()` → `node->declare_parameter<T>()` + `node->get_parameter()`

### Message Types
- `#include <sensor_msgs/PointCloud2.h>` → `#include <sensor_msgs/msg/point_cloud2.hpp>`
- `sensor_msgs::PointCloud2` → `sensor_msgs::msg::PointCloud2`

### Launch Files
- XML launch files → Python launch files
- `roslaunch` → `ros2 launch`

## Known Limitations

1. **Livox Driver**: The livox_ros_driver custom message support has been removed from display_prediction_ros2.cpp as the ROS2 version may have different message definitions.

2. **DynObjFilter Integration**: The DynObjFilter class still has ROS1 dependencies. A wrapper (DynObjFilter_ros2) provides basic ROS2 compatibility, but full integration requires:
   - Updating DynObjFilter::init() to not require ros::NodeHandle
   - Updating DynObjFilter::publish_dyn() to work with ROS2 publishers
   - OR making DynObjFilter template-based to support both ROS1 and ROS2

3. **Cal Recall Node**: The cal_recall_multi_ros2.cpp contains a basic node structure but the full recall calculation logic from the 1400+ line original file needs to be integrated.

## Migration Checklist

- [x] Create ROS2 package.xml
- [x] Create ROS2 CMakeLists.txt
- [x] Convert display_prediction node to ROS2
- [x] Convert dynfilter_with_odom node to ROS2
- [x] Convert cal_recall_multi node to ROS2 (basic structure)
- [x] Create ROS2 wrapper for DynObjFilter
- [x] Create Python launch files
- [ ] Test build with colcon
- [ ] Full DynObjFilter ROS2 integration
- [ ] Complete cal_recall_multi ROS2 implementation
- [ ] Test runtime with actual data
- [ ] Update documentation

## Contributing

When contributing to the ROS2 migration:
1. Keep original ROS1 files intact
2. Add ROS2-specific files with `_ros2` suffix or in `launch_ros2/` directory
3. Update this README with any new changes
4. Test builds on Ubuntu 24.04 with ROS2 Jazzy

## Support

For issues specific to the ROS2 migration, please file issues on the GitHub repository with the `ros2-migration` label.
