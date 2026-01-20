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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <pcl/filters/random_sample.h>
#include <Eigen/Eigen>
#include <tf2_eigen/tf2_eigen.hpp>

#include <deque>

using namespace std;

class DynFilterOdomNode : public rclcpp::Node
{
public:
    DynFilterOdomNode() : Node("dynfilter_odom")
    {
        // Initialize the DynObjFilter
        DynObjFilt = std::make_shared<DynObjFilter>();
        
        // Declare and get parameters (ROS2 style with dot-separated keys)
        this->declare_parameter<std::string>("dyn_obj.points_topic", "");
        this->declare_parameter<std::string>("dyn_obj.odom_topic", "");
        this->declare_parameter<std::string>("dyn_obj.out_file", "");
        this->declare_parameter<std::string>("dyn_obj.out_file_origin", "");
        
        this->get_parameter("dyn_obj.points_topic", points_topic);
        this->get_parameter("dyn_obj.odom_topic", odom_topic);
        this->get_parameter("dyn_obj.out_file", out_folder);
        this->get_parameter("dyn_obj.out_file_origin", out_folder_origin);
        
        // Initialize DynObjFilter with ROS2 node handle adapter
        InitDynObjFilter();
        
        // ROS2 publishers
        pub_pcl_dyn_extend = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/frame_out", 10000);
        pub_pcl_dyn = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/point_out", 100000);
        pub_pcl_std = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/std_points", 100000);
        
        // ROS2 subscribers with appropriate QoS
        // Use SensorDataQoS for point cloud subscription
        auto qos_sensor = rclcpp::SensorDataQoS();
        sub_pcl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            points_topic, qos_sensor,
            std::bind(&DynFilterOdomNode::PointsCallback, this, std::placeholders::_1));
        
        // Use default QoS for odometry
        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 200000,
            std::bind(&DynFilterOdomNode::OdomCallback, this, std::placeholders::_1));
        
        // Timer for processing (10ms = 100Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&DynFilterOdomNode::TimerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "DynFilterOdomNode initialized");
        RCLCPP_INFO(this->get_logger(), "Points topic: %s", points_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Odom topic: %s", odom_topic.c_str());
    }

private:
    void InitDynObjFilter()
    {
        // Initialize DynObjFilter parameters from ROS2 parameter server
        // Since DynObjFilter::init expects ros::NodeHandle and we can't modify the original
        // source files, we manually set all parameters directly on the DynObjFilter object
        
        // Declare all DynObjFilter parameters with defaults matching the original ROS1 code
        this->declare_parameter<int>("dyn_obj.dataset", 0);
        this->declare_parameter<double>("dyn_obj.buffer_delay", 0.1);
        this->declare_parameter<int>("dyn_obj.buffer_size", 300000);
        this->declare_parameter<int>("dyn_obj.points_num_perframe", 150000);
        this->declare_parameter<double>("dyn_obj.depth_map_dur", 0.2);
        this->declare_parameter<int>("dyn_obj.max_depth_map_num", 5);
        this->declare_parameter<int>("dyn_obj.max_pixel_points", 50);
        this->declare_parameter<double>("dyn_obj.frame_dur", 0.1);
        this->declare_parameter<double>("dyn_obj.hor_resolution_max", 0.0025);
        this->declare_parameter<double>("dyn_obj.ver_resolution_max", 0.0025);
        this->declare_parameter<double>("dyn_obj.fov_up", 0.15);
        this->declare_parameter<double>("dyn_obj.fov_down", 0.15);
        this->declare_parameter<double>("dyn_obj.fov_cut", 0.15);
        this->declare_parameter<double>("dyn_obj.fov_left", 180.0);
        this->declare_parameter<double>("dyn_obj.fov_right", -180.0);
        this->declare_parameter<double>("dyn_obj.blind_dis", 0.15);
        this->declare_parameter<int>("dyn_obj.occluded_map_thr1", 3);
        this->declare_parameter<double>("dyn_obj.map_cons_hor_thr1", 0.01);
        this->declare_parameter<double>("dyn_obj.map_cons_ver_thr1", 0.01);
        this->declare_parameter<bool>("dyn_obj.cluster_coupled", false);
        this->declare_parameter<bool>("dyn_obj.cluster_future", false);
        this->declare_parameter<bool>("dyn_obj.dyn_filter_en", true);
        this->declare_parameter<std::string>("dyn_obj.frame_id", "camera_init");
        
        // Get parameters and set them in DynObjFilter
        DynObjFilt->dataset = this->get_parameter("dyn_obj.dataset").as_int();
        DynObjFilt->buffer_delay = this->get_parameter("dyn_obj.buffer_delay").as_double();
        DynObjFilt->buffer_size = this->get_parameter("dyn_obj.buffer_size").as_int();
        DynObjFilt->points_num_perframe = this->get_parameter("dyn_obj.points_num_perframe").as_int();
        DynObjFilt->depth_map_dur = this->get_parameter("dyn_obj.depth_map_dur").as_double();
        DynObjFilt->max_depth_map_num = this->get_parameter("dyn_obj.max_depth_map_num").as_int();
        DynObjFilt->max_pixel_points = this->get_parameter("dyn_obj.max_pixel_points").as_int();
        DynObjFilt->frame_dur = this->get_parameter("dyn_obj.frame_dur").as_double();
        DynObjFilt->hor_resolution_max = this->get_parameter("dyn_obj.hor_resolution_max").as_double();
        DynObjFilt->ver_resolution_max = this->get_parameter("dyn_obj.ver_resolution_max").as_double();
        DynObjFilt->fov_up = this->get_parameter("dyn_obj.fov_up").as_double();
        DynObjFilt->fov_down = this->get_parameter("dyn_obj.fov_down").as_double();
        DynObjFilt->fov_cut = this->get_parameter("dyn_obj.fov_cut").as_double();
        DynObjFilt->fov_left = this->get_parameter("dyn_obj.fov_left").as_double();
        DynObjFilt->fov_right = this->get_parameter("dyn_obj.fov_right").as_double();
        DynObjFilt->blind_dis = this->get_parameter("dyn_obj.blind_dis").as_double();
        DynObjFilt->occluded_map_thr1 = this->get_parameter("dyn_obj.occluded_map_thr1").as_int();
        DynObjFilt->map_cons_hor_thr1 = this->get_parameter("dyn_obj.map_cons_hor_thr1").as_double();
        DynObjFilt->map_cons_ver_thr1 = this->get_parameter("dyn_obj.map_cons_ver_thr1").as_double();
        DynObjFilt->cluster_coupled = this->get_parameter("dyn_obj.cluster_coupled").as_bool();
        DynObjFilt->cluster_future = this->get_parameter("dyn_obj.cluster_future").as_bool();
        DynObjFilt->dyn_filter_en = this->get_parameter("dyn_obj.dyn_filter_en").as_bool();
        DynObjFilt->frame_id = this->get_parameter("dyn_obj.frame_id").as_string();
        
        // Initialize internal structures (similar to what init() does)
        DynObjFilt->max_ind = floor(3.1415926 * 2 / DynObjFilt->hor_resolution_max);
        
        if (DynObjFilt->pcl_his_list.size() == 0)
        {
            PointCloudXYZI::Ptr first_frame(new PointCloudXYZI());
            first_frame->reserve(400000);
            DynObjFilt->pcl_his_list.push_back(first_frame);
            DynObjFilt->laserCloudSteadObj_hist = PointCloudXYZI::Ptr(new PointCloudXYZI());
            DynObjFilt->laserCloudSteadObj = PointCloudXYZI::Ptr(new PointCloudXYZI());
            DynObjFilt->laserCloudDynObj = PointCloudXYZI::Ptr(new PointCloudXYZI());
            DynObjFilt->laserCloudDynObj_world = PointCloudXYZI::Ptr(new PointCloudXYZI());
            DynObjFilt->laserCloudDynObj_clus = PointCloudXYZI::Ptr(new PointCloudXYZI());
            DynObjFilt->laserCloudSteadObj_clus = PointCloudXYZI::Ptr(new PointCloudXYZI());
            
            int xy_ind[2] = {-1, 1};
            for (int ind_hor = 0; ind_hor < 2*DynObjFilt->hor_num + 1; ind_hor++)
            {
                for (int ind_ver = 0; ind_ver < 2*DynObjFilt->ver_num + 1; ind_ver++)
                {
                    DynObjFilt->pos_offset.push_back(((ind_hor)/2 + ind_hor%2)*xy_ind[ind_hor%2] * MAX_1D_HALF + 
                                                      ((ind_ver)/2 + ind_ver%2)*xy_ind[ind_ver%2]);
                }
            }
        }
        
        // Calculate derived parameters
        DynObjFilt->map_cons_hor_num1 = ceil(DynObjFilt->map_cons_hor_thr1/DynObjFilt->hor_resolution_max);
        DynObjFilt->map_cons_ver_num1 = ceil(DynObjFilt->map_cons_ver_thr1/DynObjFilt->ver_resolution_max);
        
        // Resize buffer
        DynObjFilt->buffer.resize(DynObjFilt->buffer_size);
        
        // Allocate point_soph pointers
        DynObjFilt->max_pointers_num = DynObjFilt->buffer_size * 3;
        DynObjFilt->point_soph_pointers.resize(DynObjFilt->max_pointers_num);
        for(int i = 0; i < DynObjFilt->max_pointers_num; i++)
        {
            DynObjFilt->point_soph_pointers[i] = new point_soph();
        }
    }

    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr cur_odom)
    {
        Eigen::Quaterniond cur_q;
        cur_q.x() = cur_odom->pose.pose.orientation.x;
        cur_q.y() = cur_odom->pose.pose.orientation.y;
        cur_q.z() = cur_odom->pose.pose.orientation.z;
        cur_q.w() = cur_odom->pose.pose.orientation.w;
        
        cur_rot = cur_q.matrix();
        cur_pos << cur_odom->pose.pose.position.x, 
                   cur_odom->pose.pose.position.y, 
                   cur_odom->pose.pose.position.z;
        
        buffer_rots.push_back(cur_rot);
        buffer_poss.push_back(cur_pos);
        
        lidar_end_time = rclcpp::Time(cur_odom->header.stamp).seconds();
        buffer_times.push_back(lidar_end_time);
    }

    void PointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_in)
    {
        boost::shared_ptr<PointCloudXYZI> feats_undistort(new PointCloudXYZI());
        pcl::fromROSMsg(*msg_in, *feats_undistort);
        buffer_pcs.push_back(feats_undistort);
    }

    void TimerCallback()
    {
        if(buffer_pcs.size() > 0 && buffer_poss.size() > 0 && 
           buffer_rots.size() > 0 && buffer_times.size() > 0)
        {
            boost::shared_ptr<PointCloudXYZI> cur_pc = buffer_pcs.at(0);
            buffer_pcs.pop_front();
            auto cur_rot_local = buffer_rots.at(0);
            buffer_rots.pop_front();
            auto cur_pos_local = buffer_poss.at(0);
            buffer_poss.pop_front();
            auto cur_time = buffer_times.at(0);
            buffer_times.pop_front();
            
            string file_name = out_folder;
            stringstream ss;
            ss << setw(6) << setfill('0') << cur_frame;
            file_name += ss.str();
            file_name.append(".label");
            
            string file_name_origin = out_folder_origin;
            stringstream sss;
            sss << setw(6) << setfill('0') << cur_frame;
            file_name_origin += sss.str();
            file_name_origin.append(".label");

            if(file_name.length() > 15 || file_name_origin.length() > 15)
                DynObjFilt->set_path(file_name, file_name_origin);

            DynObjFilt->filter(cur_pc, cur_rot_local, cur_pos_local, cur_time);
            PublishDyn(cur_time);
            cur_frame++;
        }
    }

    void PublishDyn(const double& scan_end_time)
    {
        // Publish dynamic objects (point-out)
        if(DynObjFilt->laserCloudDynObj->size() > 0)
        {
            sensor_msgs::msg::PointCloud2 laserCloudmsg;
            pcl::toROSMsg(*DynObjFilt->laserCloudDynObj, laserCloudmsg);
            laserCloudmsg.header.stamp = rclcpp::Time(static_cast<int64_t>(scan_end_time * 1e9));
            laserCloudmsg.header.frame_id = DynObjFilt->frame_id;
            pub_pcl_dyn->publish(laserCloudmsg);
        }
        
        // Publish dynamic objects (frame-out)
        if(DynObjFilt->laserCloudDynObj_clus->size() > 0)
        {
            sensor_msgs::msg::PointCloud2 laserCloudmsg;
            pcl::toROSMsg(*DynObjFilt->laserCloudDynObj_clus, laserCloudmsg);
            laserCloudmsg.header.stamp = rclcpp::Time(static_cast<int64_t>(scan_end_time * 1e9));
            laserCloudmsg.header.frame_id = DynObjFilt->frame_id;
            pub_pcl_dyn_extend->publish(laserCloudmsg);
        }
        
        // Publish static/steady points
        if(DynObjFilt->laserCloudSteadObj->size() > 0)
        {
            sensor_msgs::msg::PointCloud2 laserCloudmsg;
            pcl::toROSMsg(*DynObjFilt->laserCloudSteadObj, laserCloudmsg);
            laserCloudmsg.header.stamp = rclcpp::Time(static_cast<int64_t>(scan_end_time * 1e9));
            laserCloudmsg.header.frame_id = DynObjFilt->frame_id;
            pub_pcl_std->publish(laserCloudmsg);
        }
    }

    // Member variables
    std::shared_ptr<DynObjFilter> DynObjFilt;
    M3D cur_rot = Eigen::Matrix3d::Identity();
    V3D cur_pos = Eigen::Vector3d::Zero();
    
    std::string points_topic, odom_topic;
    std::string out_folder, out_folder_origin;
    double lidar_end_time = 0;
    int cur_frame = 0;
    
    std::deque<M3D> buffer_rots;
    std::deque<V3D> buffer_poss;
    std::deque<double> buffer_times;
    std::deque<boost::shared_ptr<PointCloudXYZI>> buffer_pcs;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_dyn;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_dyn_extend;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_std;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynFilterOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
