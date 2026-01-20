#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <csignal>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <m-detector/DynObjFilter.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <unistd.h>
#include <dirent.h>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

// Global maps for object types
std::unordered_map<int, std::string> objects_types_map_kitti;
std::unordered_map<int, std::string> objects_types_map_nuscenes;
std::unordered_map<int, int> objects_class_map_nuscenes;
std::unordered_map<int, std::string> objects_types_map_waymo;

void Init()
{
    objects_types_map_kitti[0] = "Person";
    objects_types_map_kitti[1] = "Truck";
    objects_types_map_kitti[2] = "Car";
    objects_types_map_kitti[3] = "Tram";
    objects_types_map_kitti[4] = "Pedestrain";
    objects_types_map_kitti[5] = "Cyclist";
    objects_types_map_kitti[6] = "Van";

    objects_types_map_nuscenes[0] = "Animal";
    objects_types_map_nuscenes[1] = "Pedestrian";
    objects_types_map_nuscenes[2] = "Movable_object";
    objects_types_map_nuscenes[3] = "Bicycle";
    objects_types_map_nuscenes[4] = "Bus";
    objects_types_map_nuscenes[5] = "Car";
    objects_types_map_nuscenes[6] = "Emergency";
    objects_types_map_nuscenes[7] = "Motorcycle";
    objects_types_map_nuscenes[8] = "Trailer";
    objects_types_map_nuscenes[9] = "Truck";
    objects_types_map_nuscenes[10] = "Ego";
    
    objects_class_map_nuscenes[1] = 0;
    objects_class_map_nuscenes[2] = 1;
    objects_class_map_nuscenes[3] = 1;
    objects_class_map_nuscenes[4] = 1;
    objects_class_map_nuscenes[5] = 1;
    objects_class_map_nuscenes[6] = 1;
    objects_class_map_nuscenes[7] = 1;
    objects_class_map_nuscenes[8] = 1;
    objects_class_map_nuscenes[9] = 2;
    objects_class_map_nuscenes[10] = 2;
    objects_class_map_nuscenes[11] = 2;
    objects_class_map_nuscenes[12] = 2;
    objects_class_map_nuscenes[14] = 3;
    objects_class_map_nuscenes[15] = 4;
    objects_class_map_nuscenes[16] = 4;
    objects_class_map_nuscenes[17] = 5;
    objects_class_map_nuscenes[18] = 6;
    objects_class_map_nuscenes[19] = 6;
    objects_class_map_nuscenes[20] = 6;
    objects_class_map_nuscenes[21] = 7;
    objects_class_map_nuscenes[22] = 8;
    objects_class_map_nuscenes[23] = 9;
    objects_class_map_nuscenes[31] = 10;

    objects_types_map_waymo[0] = "Vehicle";
    objects_types_map_waymo[1] = "Pedestrian";
    objects_types_map_waymo[2] = "Cyclist";
}

class CalRecallNode : public rclcpp::Node
{
public:
    CalRecallNode() : Node("cal_recall")
    {
        Init();

        // Declare parameters
        this->declare_parameter<int>("dyn_obj.dataset", -1);
        this->declare_parameter<bool>("dyn_obj.is_origin", false);
        this->declare_parameter<std::string>("dyn_obj.dataset_folder", "/");
        this->declare_parameter<int>("dyn_obj.start_param", -1);
        this->declare_parameter<int>("dyn_obj.end_param", 0);
        this->declare_parameter<int>("dyn_obj.start_se", -1);
        this->declare_parameter<int>("dyn_obj.end_se", 0);

        this->get_parameter("dyn_obj.dataset", dataset_);
        this->get_parameter("dyn_obj.is_origin", is_origin_);
        this->get_parameter("dyn_obj.dataset_folder", dataset_folder_);
        this->get_parameter("dyn_obj.start_param", start_param_);
        this->get_parameter("dyn_obj.end_param", end_param_);
        this->get_parameter("dyn_obj.start_se", start_se_);
        this->get_parameter("dyn_obj.end_se", end_se_);

        // Create publishers
        pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cal_recall/pointcloud", 10);
        pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/cal_recall/marker", 10);
        pub_iou_view_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cal_recall/iou_view", 10);

        RCLCPP_INFO(this->get_logger(), "CalRecall Node initialized");
        RCLCPP_INFO(this->get_logger(), "Dataset: %d, Dataset folder: %s", 
                    dataset_, dataset_folder_.c_str());
        
        // Note: The actual recall calculation logic would be called here
        // This is a placeholder for the full migration
        RCLCPP_WARN(this->get_logger(), 
            "Recall calculation logic needs full implementation. "
            "This ROS2 node provides the basic structure.");
    }

private:
    // Member variables
    int dataset_ = -1;
    int start_param_ = 0;
    int end_param_ = 0;
    int start_se_ = 0;
    int end_se_ = 0;
    bool is_origin_ = false;
    std::string dataset_folder_;
    std::string pred_folder_;
    std::string label_folder_;
    std::string recall_folder_;
    std::string recall_origin_folder_;
    std::string recall_file_;
    std::string recall_origin_file_;
    std::string semantic_folder_;
    std::string out_folder_;

    int total_tp_origin_ = 0;
    int total_fn_origin_ = 0;
    int total_fp_origin_ = 0;
    int total_op_origin_ = 0;
    int total_tn_origin_ = 0;

    pcl::PointCloud<pcl::PointXYZINormal> lastcloud_;
    PointCloudXYZI::Ptr last_pc_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_iou_view_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalRecallNode>();
    
    // Note: In the original file, the main processing happens in main() after parameter loading
    // In ROS2, this could be restructured as a service call or timer callback
    // For now, we just spin the node
    
    RCLCPP_WARN(node->get_logger(), 
        "Full recall calculation logic from cal_recall_multi.cpp needs to be "
        "integrated into this ROS2 node structure. The original file contains "
        "extensive offline processing that should be refactored for ROS2.");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
