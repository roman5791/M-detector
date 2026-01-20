#ifndef DYN_OBJ_FLT_ROS2_H
#define DYN_OBJ_FLT_ROS2_H

// ROS2 compatibility header for DynObjFilter
// This file provides ROS2 adaptations for the DynObjFilter class

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>
#include <types.h>

// Forward declaration
class DynObjFilter;

namespace m_detector_ros2 {

class DynObjFilterWrapper
{
public:
    DynObjFilterWrapper(rclcpp::Node::SharedPtr node);
    ~DynObjFilterWrapper() = default;

    // Initialize the filter with ROS2 parameters
    void init();

    // Filter function
    void filter(std::shared_ptr<PointCloudXYZI> cur_pc, 
                const M3D& cur_rot, 
                const V3D& cur_pos, 
                const double& cur_time);

    // Publish dynamic objects with ROS2 publishers
    void publishDyn(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_point_out,
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_frame_out,
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_steady_points,
        const double& scan_end_time);

    // Set output file paths
    void setPath(const std::string& file_name, const std::string& file_name_origin);

    // Get the underlying DynObjFilter
    std::shared_ptr<DynObjFilter> getFilter() { return filter_; }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<DynObjFilter> filter_;
};

} // namespace m_detector_ros2

#endif // DYN_OBJ_FLT_ROS2_H
