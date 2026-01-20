# Switching Between ROS1 and ROS2

This package supports both ROS1 (Melodic) and ROS2 (Jazzy). The original ROS1 source files are kept intact, and ROS2-specific files have been added with `_ros2` suffix or in separate directories.

## To Build for ROS1 (Original)

The package builds normally for ROS1 Melodic using the original files:

```bash
# Source ROS1
source /opt/ros/melodic/setup.bash

# Build with catkin
cd /path/to/catkin_ws
catkin_make

# Or with catkin tools
catkin build m_detector
```

The ROS1 build uses:
- `package.xml` (original)
- `CMakeLists.txt` (original)
- Original node sources: `src/display_prediction.cpp`, `src/dynfilter_with_odom.cpp`, `src/cal_recall_multi.cpp`

## To Build for ROS2 Jazzy

### One-time Setup

1. **Backup and replace build files**:
   ```bash
   cd /path/to/m_detector
   
   # Backup ROS1 files (if not already done)
   cp package.xml package_ros1.xml.bak
   cp CMakeLists.txt CMakeLists_ros1.txt.bak
   
   # Use ROS2 files
   cp package_ros2.xml package.xml
   cp CMakeLists_ros2.txt CMakeLists.txt
   ```

2. **Source ROS2 Jazzy**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

3. **Build with colcon**:
   ```bash
   cd /path/to/ros2_ws/src
   ln -s /path/to/m_detector .  # If not already in workspace
   
   cd /path/to/ros2_ws
   colcon build --packages-select m_detector
   ```

4. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

### Running ROS2 Nodes

```bash
# DynFilter with KITTI dataset
ros2 launch m_detector detector_kitti.launch.py

# Display prediction
ros2 launch m_detector display.launch.py

# Cal recall
ros2 run m_detector cal_recall_ros2
```

## Switching Back to ROS1

Simply restore the original files:

```bash
cd /path/to/m_detector
cp package_ros1.xml.bak package.xml
cp CMakeLists_ros1.txt.bak CMakeLists.txt
```

Then rebuild with catkin.

## Files Overview

### ROS1 Files (Original)
- `package.xml` - ROS1 manifest
- `CMakeLists.txt` - catkin build config
- `src/display_prediction.cpp`
- `src/dynfilter_with_odom.cpp`
- `src/cal_recall_multi.cpp`
- `src/DynObjFilter.cpp`
- `src/DynObjCluster.cpp`
- `launch/*.launch` - XML launch files

### ROS2 Files (New)
- `package_ros2.xml` - ROS2 manifest (format 3)
- `CMakeLists_ros2.txt` - ament_cmake build config
- `src/display_prediction_ros2.cpp`
- `src/dynfilter_with_odom_ros2.cpp`
- `src/cal_recall_multi_ros2.cpp`
- `src/DynObjFilter_ros2.cpp` - ROS2 wrapper
- `include/m-detector/DynObjFilter_ros2.h`
- `launch_ros2/*.launch.py` - Python launch files
- `ROS2_MIGRATION.md` - Detailed migration documentation

### Shared Files (Used by Both)
- `src/DynObjFilter.cpp` - Core algorithm (has some ROS1 dependencies)
- `src/DynObjCluster.cpp` - Core clustering
- `include/` - Headers
- `config/` - Configuration YAML files
- `rviz/` - RViz configs

## Important Notes

1. **Do not modify** the original ROS1 files (`src/display_prediction.cpp`, etc.) unless fixing bugs that apply to both versions.

2. **ROS2-specific changes** should go in the `_ros2` files.

3. **DynObjFilter** currently has ROS1 dependencies. The `DynObjFilter_ros2.cpp` wrapper provides basic ROS2 compatibility, but full integration is pending.

4. See `ROS2_MIGRATION.md` for detailed migration information and known limitations.
