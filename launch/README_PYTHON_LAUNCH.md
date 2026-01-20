# Python Launch Files for M-detector

This directory contains Python launch files that provide an alternative to the XML launch files.

## Available Launch Files

### 1. detector.launch.py
Generic detector launch file with configurable parameters.

**Usage:**
```bash
rosrun m_detector detector.launch.py _config_file:=/path/to/config.yaml _rviz:=true
```

**Parameters:**
- `config_file`: Path to YAML configuration file
- `rviz`: Launch RViz visualization (default: true)
- `time_file`: Path to time log file
- `out_path`: Path for frame-out results
- `out_origin_path`: Path for point-out results
- `pose_log`: Enable pose logging (default: false)
- `pose_log_file`: Path to pose log file
- `cluster_out_file`: Path to cluster output file
- `time_breakdown_file`: Path to time breakdown file

### 2. dynfilter_odom.launch.py
Launch file for dynfilter with odometry support.

**Usage:**
```bash
rosrun m_detector dynfilter_odom.launch.py _config_file:=/path/to/config.yaml
```

**Parameters:** Same as detector.launch.py

### 3. display_prediction.launch.py
Launch file for displaying prediction results.

**Usage:**
```bash
rosrun m_detector display_prediction.launch.py _pred_file:=/path/to/predictions
```

**Parameters:**
- `config_file`: Path to YAML configuration file
- `rviz`: Launch RViz visualization (default: true)
- `pred_file`: Path to prediction file
- `pc_file`: Path to point cloud file
- `pc_topic`: Point cloud topic name

### 4. cal_recall.launch.py
Launch file for calculating IoU and recall metrics.

**Usage:**
```bash
rosrun m_detector cal_recall.launch.py _dataset:=0 _dataset_folder:=/path/to/dataset
```

**Parameters:**
- `rviz`: Launch RViz visualization (default: false)
- `dataset`: Dataset type (0: kitti, 1: nuscenes, 2: waymo, 3: avia)
- `dataset_folder`: Path to dataset folder
- `start_param`: First parameter file number for calculation
- `end_param`: Last parameter file number for calculation
- `start_se`: First sequence number for calculation
- `end_se`: Last sequence number for calculation
- `is_origin`: Use point-out results (default: false for frame-out)

## Notes

- These Python launch files use the roslaunch Python API
- They provide equivalent functionality to the XML launch files
- Make sure the files are executable: `chmod +x launch/*.launch.py`
- Requires ROS1 (tested with ROS Melodic and later)

## Comparison with XML Launch Files

The Python launch files offer:
- Programmatic control over launch logic
- Easier integration with Python scripts
- More flexible parameter handling
- Better suited for complex launch sequences

The XML launch files remain the standard and recommended approach for simple use cases.
