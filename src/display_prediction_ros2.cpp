
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <types.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <pcl/filters/random_sample.h>
#include <unistd.h> 
#include <dirent.h> 
#include <iomanip>

// Livox ROS2 driver support - conditionally compiled if available
#ifdef HAVE_LIVOX_ROS2
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

using namespace std;

class DisplayPredictionNode : public rclcpp::Node
{
public:
    DisplayPredictionNode() : Node("display_prediction")
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("dyn_obj.pc_file", "");
        this->declare_parameter<std::string>("dyn_obj.pred_file", "");
        this->declare_parameter<std::string>("dyn_obj.pc_topic", "/velodyne_points");
        this->declare_parameter<std::string>("dyn_obj.frame_id", "camera_init");

        this->get_parameter("dyn_obj.pc_file", pc_folder_);
        this->get_parameter("dyn_obj.pred_file", pred_folder_);
        this->get_parameter("dyn_obj.pc_topic", points_topic_);
        this->get_parameter("dyn_obj.frame_id", frame_id_);

        // Count prediction files
        int pred_num = 0;
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder_.c_str());
        if (pred_dir != nullptr) {
            struct dirent* pred_ptr;
            while((pred_ptr = readdir(pred_dir)) != NULL)
            {
                if(pred_ptr->d_name[0] == '.') {continue;}
                pred_num++;
            }
            closedir(pred_dir);
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not open prediction folder: %s", pred_folder_.c_str());
        }

        minus_num_ = 0;

        // Create publishers with appropriate QoS
        pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/result_view", 100000);
        pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/m_detector/text_view", 10);
        pub_iou_view_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/iou_view", 100000);
        pub_static_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/std_points", 100000);
        pub_dynamic_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/dyn_points", 100000);

        // Create subscriber with SensorDataQoS for point cloud topics
        auto qos = rclcpp::SensorDataQoS();
        
#ifdef HAVE_LIVOX_ROS2
        // Check if this is a Livox topic
        if(points_topic_ == "/livox/lidar")
        {
            sub_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                points_topic_, qos,
                std::bind(&DisplayPredictionNode::aviaPointsCallback, this, std::placeholders::_1));
        }
        else
#endif
        {
            sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                points_topic_, qos,
                std::bind(&DisplayPredictionNode::pointsCallback, this, std::placeholders::_1));
        }

        RCLCPP_INFO(this->get_logger(), "Display prediction node initialized");
        RCLCPP_INFO(this->get_logger(), "Listening to topic: %s", points_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Prediction folder: %s", pred_folder_.c_str());
        RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
    }

private:
    void pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_in)
    {
        PointCloudXYZI::Ptr points_in(new PointCloudXYZI());
        pcl::fromPCL(pcl_conversions::toPCL(*msg_in), *points_in);

        if(frames_ < minus_num_)
        {
            sensor_msgs::msg::PointCloud2 pcl_ros_msg;
            pcl::toROSMsg(*points_in, pcl_ros_msg);
            pcl_ros_msg.header.frame_id = frame_id_;
            pcl_ros_msg.header.stamp = this->now();
            pub_pointcloud_->publish(pcl_ros_msg);
        }
        else
        {   
            cout << "frame: " << frames_ << endl;

            string pred_file = pred_folder_;
            stringstream sss;
            sss << setw(6) << setfill('0') << frames_-minus_num_ ;
            pred_file += sss.str(); 
            pred_file.append(".label");

            std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
            if(!pred_input.good())
            {
                RCLCPP_ERROR(this->get_logger(), "Could not read prediction file: %s", pred_file.c_str());
                frames_++;
                return;
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr points_out (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr iou_out (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_out (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr static_out (new pcl::PointCloud<pcl::PointXYZI>);
            
            int tp = 0, fn = 0, fp = 0, count = 0;
            float iou = 0.0f;
            for (size_t i=0; i<points_in->points.size(); i++) 
            {
                pcl::PointXYZI point;
                point.x = points_in->points[i].x;
                point.y = points_in->points[i].y;
                point.z = points_in->points[i].z;

                int pred_num;
                pred_input.read((char *) &pred_num, sizeof(int));
                pred_num = pred_num & 0xFFFF;

                point.intensity = 0;
                
                if(pred_num >= 251)
                {
                    point.intensity = 10;
                    iou_out->push_back(point);
                    dynamic_out->push_back(point);
                }
                else
                {
                    point.intensity = 20;
                    iou_out->push_back(point);
                    static_out->push_back(point);
                }
                   
                points_out->push_back(point);
            }

            sensor_msgs::msg::PointCloud2 pcl_ros_msg;
            pcl::toROSMsg(*points_out, pcl_ros_msg);
            pcl_ros_msg.header.frame_id = frame_id_;
            pcl_ros_msg.header.stamp = this->now();
            pub_pointcloud_->publish(pcl_ros_msg);

            sensor_msgs::msg::PointCloud2 pcl_msg;
            pcl::toROSMsg(*iou_out, pcl_msg);
            pcl_msg.header.frame_id = frame_id_;
            pcl_msg.header.stamp = this->now();
            pub_iou_view_->publish(pcl_msg); 

            sensor_msgs::msg::PointCloud2 dynamic_msg;
            pcl::toROSMsg(*dynamic_out, dynamic_msg);
            dynamic_msg.header.frame_id = frame_id_;
            dynamic_msg.header.stamp = this->now();
            pub_dynamic_->publish(dynamic_msg); 

            sensor_msgs::msg::PointCloud2 static_msg;
            pcl::toROSMsg(*static_out, static_msg);
            static_msg.header.frame_id = frame_id_;
            static_msg.header.stamp = this->now();
            pub_static_->publish(static_msg); 

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = this->now();
            marker.ns = "basic_shapes";
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

            marker.scale.z = 0.2;
            marker.color.b = 0;
            marker.color.g = 0;
            marker.color.r = 255;
            marker.color.a = 1;  
            geometry_msgs::msg::Pose pose;
            if (!points_out->points.empty()) {
                pose.position.x = points_out->points[0].x;
                pose.position.y = points_out->points[0].y;
                pose.position.z = points_out->points[0].z;
            }
            ostringstream str;
            str << "tp: " << tp << " fn: " << fn << " fp: " << fp << " count: " << count << " iou: " << iou;
            marker.text = str.str();
            marker.pose = pose;
            pub_marker_->publish(marker);
        }
        
        frames_++;
    }

#ifdef HAVE_LIVOX_ROS2
    void aviaPointsCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg_in)
    {   
        PointCloudXYZI::Ptr points_in(new PointCloudXYZI());
        points_in->resize(msg_in->point_num);
        std::cout << "points size: " << msg_in->point_num << std::endl;
        if(msg_in->point_num == 0) return;
        for(uint32_t i = 0; i < msg_in->point_num; i++)
        {
            points_in->points[i].x = msg_in->points[i].x;
            points_in->points[i].y = msg_in->points[i].y;
            points_in->points[i].z = msg_in->points[i].z;
        }

        if(frames_ < minus_num_)
        {
            sensor_msgs::msg::PointCloud2 pcl_ros_msg;
            pcl::toROSMsg(*points_in, pcl_ros_msg);
            pcl_ros_msg.header.frame_id = frame_id_;
            pcl_ros_msg.header.stamp = this->now();
            pub_pointcloud_->publish(pcl_ros_msg);
        }
        else
        {   
            cout << "frame: " << frames_ << endl;

            string pred_file = pred_folder_;
            stringstream sss;
            sss << setw(6) << setfill('0') << frames_-minus_num_ ;
            pred_file += sss.str(); 
            pred_file.append(".label");

            std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
            if(!pred_input.good())
            {
                RCLCPP_ERROR(this->get_logger(), "Could not read prediction file: %s", pred_file.c_str());
                frames_++;
                return;
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr points_out (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr iou_out (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_out (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr static_out (new pcl::PointCloud<pcl::PointXYZI>);
            
            int tp = 0, fn = 0, fp = 0, count = 0;
            float iou = 0.0f;
            for (size_t i=0; i<points_in->points.size(); i++) 
            {
                pcl::PointXYZI point;
                point.x = points_in->points[i].x;
                point.y = points_in->points[i].y;
                point.z = points_in->points[i].z;

                int pred_num;
                pred_input.read((char *) &pred_num, sizeof(int));
                pred_num = pred_num & 0xFFFF;

                point.intensity = 0;
                
                if(pred_num >= 251)
                {
                    point.intensity = 10;
                    iou_out->push_back(point);
                    dynamic_out->push_back(point);
                }
                else
                {
                    point.intensity = 20;
                    iou_out->push_back(point);
                    static_out->push_back(point);
                }
                   
                points_out->push_back(point);
            }

            sensor_msgs::msg::PointCloud2 pcl_ros_msg;
            pcl::toROSMsg(*points_out, pcl_ros_msg);
            pcl_ros_msg.header.frame_id = frame_id_;
            pcl_ros_msg.header.stamp = this->now();
            pub_pointcloud_->publish(pcl_ros_msg);

            sensor_msgs::msg::PointCloud2 pcl_msg;
            pcl::toROSMsg(*iou_out, pcl_msg);
            pcl_msg.header.frame_id = frame_id_;
            pcl_msg.header.stamp = this->now();
            pub_iou_view_->publish(pcl_msg); 

            sensor_msgs::msg::PointCloud2 dynamic_msg;
            pcl::toROSMsg(*dynamic_out, dynamic_msg);
            dynamic_msg.header.frame_id = frame_id_;
            dynamic_msg.header.stamp = this->now();
            pub_dynamic_->publish(dynamic_msg); 

            sensor_msgs::msg::PointCloud2 static_msg;
            pcl::toROSMsg(*static_out, static_msg);
            static_msg.header.frame_id = frame_id_;
            static_msg.header.stamp = this->now();
            pub_static_->publish(static_msg); 

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = this->now();
            marker.ns = "basic_shapes";
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

            marker.scale.z = 0.2;
            marker.color.b = 0;
            marker.color.g = 0;
            marker.color.r = 255;
            marker.color.a = 1;  
            geometry_msgs::msg::Pose pose;
            if (!points_out->points.empty()) {
                pose.position.x = points_out->points[0].x;
                pose.position.y = points_out->points[0].y;
                pose.position.z = points_out->points[0].z;
            }
            ostringstream str;
            str << "tp: " << tp << " fn: " << fn << " fp: " << fp << " count: " << count << " iou: " << iou;
            marker.text = str.str();
            marker.pose = pose;
            pub_marker_->publish(marker);
        }
        
        frames_++;
    }
#endif

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_iou_view_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_static_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dynamic_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;
#ifdef HAVE_LIVOX_ROS2
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox_;
#endif

    // Parameters
    std::string points_topic_;
    std::string frame_id_;
    std::string pc_folder_;
    std::string pred_folder_;
    
    // State
    int frames_ = 0;
    int minus_num_ = 1;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DisplayPredictionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
