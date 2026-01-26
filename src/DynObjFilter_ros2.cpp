#include <m-detector/DynObjFilter_ros2.h>
#include <m-detector/DynObjFilter.h>
#include <pcl_conversions/pcl_conversions.h>

namespace m_detector_ros2 {

DynObjFilterWrapper::DynObjFilterWrapper(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    filter_ = std::make_shared<DynObjFilter>();
}

void DynObjFilterWrapper::init()
{
    // Read all parameters from ROS2 node and set them in the filter
    // This replaces the ROS1 nh.param calls
    
    double buffer_delay = node_->declare_parameter("dyn_obj.buffer_delay", 0.1);
    int buffer_size = node_->declare_parameter("dyn_obj.buffer_size", 300000);
    int points_num_perframe = node_->declare_parameter("dyn_obj.points_num_perframe", 150000);
    double depth_map_dur = node_->declare_parameter("dyn_obj.depth_map_dur", 0.2);
    int max_depth_map_num = node_->declare_parameter("dyn_obj.max_depth_map_num", 5);
    int max_pixel_points = node_->declare_parameter("dyn_obj.max_pixel_points", 50);
    double frame_dur = node_->declare_parameter("dyn_obj.frame_dur", 0.1);
    int dataset = node_->declare_parameter("dyn_obj.dataset", 0);
    float self_x_f = node_->declare_parameter("dyn_obj.self_x_f", 0.15f);
    float self_x_b = node_->declare_parameter("dyn_obj.self_x_b", 0.15f);
    float self_y_l = node_->declare_parameter("dyn_obj.self_y_l", 0.15f);
    float self_y_r = node_->declare_parameter("dyn_obj.self_y_r", 0.5f);
    float blind_dis = node_->declare_parameter("dyn_obj.blind_dis", 0.15f);
    float fov_up = node_->declare_parameter("dyn_obj.fov_up", 0.15f);
    float fov_down = node_->declare_parameter("dyn_obj.fov_down", 0.15f);
    float fov_cut = node_->declare_parameter("dyn_obj.fov_cut", 0.15f);
    float fov_left = node_->declare_parameter("dyn_obj.fov_left", 180.0f);
    float fov_right = node_->declare_parameter("dyn_obj.fov_right", -180.0f);
    int checkneighbor_range = node_->declare_parameter("dyn_obj.checkneighbor_range", 1);
    bool stop_object_detect = node_->declare_parameter("dyn_obj.stop_object_detect", false);
    
    // Additional parameters would be declared here based on DynObjFilter::init()
    // For a complete migration, all parameters from the init() method should be ported
    
    RCLCPP_INFO(node_->get_logger(), 
        "DynObjFilter initialized with buffer_delay=%.2f, buffer_size=%d, dataset=%d",
        buffer_delay, buffer_size, dataset);
    
    // Note: The actual parameter setting in DynObjFilter needs to be done via public methods
    // or the DynObjFilter class needs to be updated to accept parameters differently
}

void DynObjFilterWrapper::filter(std::shared_ptr<PointCloudXYZI> cur_pc,
                                   const M3D& cur_rot,
                                   const V3D& cur_pos,
                                   const double& cur_time)
{
    // Forward to the underlying filter
    if (filter_) {
        filter_->filter(cur_pc, cur_rot, cur_pos, cur_time);
    }
}

void DynObjFilterWrapper::publishDyn(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_point_out,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_frame_out,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_steady_points,
    const double& scan_end_time)
{
    // This is a ROS2 adaptation of the publish_dyn method
    // Since DynObjFilter::publish_dyn uses ROS1 publishers, we need to recreate the logic here
    
    if (!filter_) {
        RCLCPP_WARN(node_->get_logger(), "Filter not initialized");
        return;
    }
    
    // Access the filter's internal point clouds
    // Note: This requires DynObjFilter to expose these members or provide getters
    // For now, this is a placeholder implementation
    
    RCLCPP_INFO(node_->get_logger(), "Publishing dynamic objects for time %.3f", scan_end_time);
    
    // TODO: Complete implementation requires either:
    // 1. Updating DynObjFilter to expose internal state, OR
    // 2. Making DynObjFilter template-based to work with both ROS1 and ROS2 publishers, OR
    // 3. Creating a full ROS2-native DynObjFilter class
}

void DynObjFilterWrapper::setPath(const std::string& file_name, 
                                   const std::string& file_name_origin)
{
    if (filter_) {
        filter_->set_path(file_name, file_name_origin);
    }
}

} // namespace m_detector_ros2
