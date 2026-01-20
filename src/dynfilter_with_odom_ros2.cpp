#include <rclcpp/rclcpp.hpp>
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <Eigen/Core>
#include <types.h>
#include <m-detector/DynObjFilter.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <pcl/filters/random_sample.h>
#include <Eigen/Eigen>
#include <deque>

using namespace std;

class DynFilterNode : public rclcpp::Node
{
public:
    DynFilterNode() : Node("dynfilter_odom")
    {
        // Declare parameters
        this->declare_parameter<std::string>("dyn_obj.points_topic", "");
        this->declare_parameter<std::string>("dyn_obj.odom_topic", "");
        this->declare_parameter<std::string>("dyn_obj.out_file", "");
        this->declare_parameter<std::string>("dyn_obj.out_file_origin", "");

        this->get_parameter("dyn_obj.points_topic", points_topic_);
        this->get_parameter("dyn_obj.odom_topic", odom_topic_);
        this->get_parameter("dyn_obj.out_file", out_folder_);
        this->get_parameter("dyn_obj.out_file_origin", out_folder_origin_);

        // Initialize DynObjFilter with ROS2 node
        DynObjFilt_ = std::make_shared<DynObjFilter>();
        initDynObjFilter();

        // Create publishers
        pub_pcl_dyn_extend_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/frame_out", 10000);
        pub_pcl_dyn_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/point_out", 100000);
        pub_pcl_std_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/std_points", 100000);

        // Create subscribers
        sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            points_topic_, 200000,
            std::bind(&DynFilterNode::pointsCallback, this, std::placeholders::_1));
        
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 200000,
            std::bind(&DynFilterNode::odomCallback, this, std::placeholders::_1));

        // Create timer (10ms = 0.01s)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&DynFilterNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "DynFilter Node initialized");
    }

private:
    void initDynObjFilter()
    {
        // Initialize DynObjFilter with parameters from ROS2 node
        // Since the original init() expects a ros::NodeHandle, we need to manually set parameters
        // This is a simplified version - you may need to add more parameters based on your needs
        
        // For now, we'll skip the init and let the filter use default values
        // In a full migration, you'd need to update DynObjFilter class to support ROS2
        RCLCPP_WARN(this->get_logger(), 
            "DynObjFilter initialized with default values. Full parameter migration needed.");
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr cur_odom)
    {
        Eigen::Quaterniond cur_q;
        cur_q.x() = cur_odom->pose.pose.orientation.x;
        cur_q.y() = cur_odom->pose.pose.orientation.y;
        cur_q.z() = cur_odom->pose.pose.orientation.z;
        cur_q.w() = cur_odom->pose.pose.orientation.w;
        
        cur_rot_ = cur_q.matrix();
        cur_pos_ << cur_odom->pose.pose.position.x, 
                    cur_odom->pose.pose.position.y, 
                    cur_odom->pose.pose.position.z;
        
        buffer_rots_.push_back(cur_rot_);
        buffer_poss_.push_back(cur_pos_);
        
        lidar_end_time_ = rclcpp::Time(cur_odom->header.stamp).seconds();
        buffer_times_.push_back(lidar_end_time_);
    }

    void pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_in)
    {
        std::shared_ptr<PointCloudXYZI> feats_undistort(new PointCloudXYZI());
        pcl::fromROSMsg(*msg_in, *feats_undistort);
        buffer_pcs_.push_back(feats_undistort);
    }

    void timerCallback()
    {
        if(buffer_pcs_.size() > 0 && buffer_poss_.size() > 0 && 
           buffer_rots_.size() > 0 && buffer_times_.size() > 0)
        {
            std::shared_ptr<PointCloudXYZI> cur_pc = buffer_pcs_.at(0);
            buffer_pcs_.pop_front();
            auto cur_rot = buffer_rots_.at(0);
            buffer_rots_.pop_front();
            auto cur_pos = buffer_poss_.at(0);
            buffer_poss_.pop_front();
            auto cur_time = buffer_times_.at(0);
            buffer_times_.pop_front();

            string file_name = out_folder_;
            stringstream ss;
            ss << setw(6) << setfill('0') << cur_frame_;
            file_name += ss.str();
            file_name.append(".label");
            
            string file_name_origin = out_folder_origin_;
            stringstream sss;
            sss << setw(6) << setfill('0') << cur_frame_;
            file_name_origin += sss.str();
            file_name_origin.append(".label");

            if(file_name.length() > 15 || file_name_origin.length() > 15)
                DynObjFilt_->set_path(file_name, file_name_origin);

            DynObjFilt_->filter(cur_pc, cur_rot, cur_pos, cur_time);
            publishDyn(cur_time);
            cur_frame_++;
        }
    }

    void publishDyn(const double & scan_end_time)
    {
        // This is a ROS2 adaptation of the publish_dyn method
        // Since we can't directly call the ROS1 version, we need to publish manually
        // This is a simplified version - in full migration, update DynObjFilter class
        
        RCLCPP_INFO(this->get_logger(), "Processing frame %d", cur_frame_);
        
        // Note: The actual publishing logic would need access to DynObjFilter's internal state
        // For now, this is a placeholder that needs to be completed with proper ROS2 publishers
        // In a full migration, you'd update DynObjFilter to work with ROS2 publishers
    }

    // Member variables
    std::string points_topic_;
    std::string odom_topic_;
    std::string out_folder_;
    std::string out_folder_origin_;
    
    std::shared_ptr<DynObjFilter> DynObjFilt_;
    M3D cur_rot_ = Eigen::Matrix3d::Identity();
    V3D cur_pos_ = Eigen::Vector3d::Zero();
    double lidar_end_time_ = 0;
    int cur_frame_ = 0;

    std::deque<M3D> buffer_rots_;
    std::deque<V3D> buffer_poss_;
    std::deque<double> buffer_times_;
    std::deque<std::shared_ptr<PointCloudXYZI>> buffer_pcs_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_dyn_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_dyn_extend_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_std_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
