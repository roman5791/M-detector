#include <rclcpp/rclcpp.hpp>
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
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

#define PI_MATH (3.141593f)

using namespace std;

class DynFilterOdomNode : public rclcpp::Node
{
public:
    DynFilterOdomNode() : Node("dynfilter_odom")
    {
        // Initialize DynObjFilter
        DynObjFilt = std::make_shared<DynObjFilter>();
        
        // Declare and get parameters
        this->declare_parameter("dyn_obj.points_topic", "");
        this->declare_parameter("dyn_obj.odom_topic", "");
        this->declare_parameter("dyn_obj.out_file", "");
        this->declare_parameter("dyn_obj.out_file_origin", "");
        
        points_topic = this->get_parameter("dyn_obj.points_topic").as_string();
        odom_topic = this->get_parameter("dyn_obj.odom_topic").as_string();
        out_folder = this->get_parameter("dyn_obj.out_file").as_string();
        out_folder_origin = this->get_parameter("dyn_obj.out_file_origin").as_string();
        
        // Initialize DynObjFilter with ROS2 node
        initDynObjFilter();
        
        // Create publishers
        pub_pcl_dyn_extend = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/frame_out", 10000);
        pub_pcl_dyn = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/point_out", 100000);
        pub_pcl_std = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/m_detector/std_points", 100000);
        
        // Create subscribers with appropriate QoS
        auto qos_sensor = rclcpp::SensorDataQoS();
        sub_pcl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            points_topic, qos_sensor,
            std::bind(&DynFilterOdomNode::pointsCallback, this, std::placeholders::_1));
        
        // Odometry typically uses reliable QoS
        auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(200000));
        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, qos_reliable,
            std::bind(&DynFilterOdomNode::odomCallback, this, std::placeholders::_1));
        
        // Create timer (10ms = 0.01s)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&DynFilterOdomNode::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "DynFilterOdomNode initialized");
        RCLCPP_INFO(this->get_logger(), "Points topic: %s", points_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Odom topic: %s", odom_topic.c_str());
    }

private:
    void initDynObjFilter()
    {
        // Declare and read all parameters used by DynObjFilter::init()
        this->declare_parameter("dyn_obj.buffer_delay", 0.1);
        this->declare_parameter("dyn_obj.buffer_size", 300000);
        this->declare_parameter("dyn_obj.points_num_perframe", 150000);
        this->declare_parameter("dyn_obj.depth_map_dur", 0.2);
        this->declare_parameter("dyn_obj.max_depth_map_num", 5);
        this->declare_parameter("dyn_obj.max_pixel_points", 50);
        this->declare_parameter("dyn_obj.frame_dur", 0.1);
        this->declare_parameter("dyn_obj.dataset", 0);
        this->declare_parameter("dyn_obj.self_x_f", 0.15f);
        this->declare_parameter("dyn_obj.self_x_b", 0.15f);
        this->declare_parameter("dyn_obj.self_y_l", 0.15f);
        this->declare_parameter("dyn_obj.self_y_r", 0.5f);
        this->declare_parameter("dyn_obj.blind_dis", 0.15f);
        this->declare_parameter("dyn_obj.fov_up", 0.15f);
        this->declare_parameter("dyn_obj.fov_down", 0.15f);
        this->declare_parameter("dyn_obj.fov_cut", 0.15f);
        this->declare_parameter("dyn_obj.fov_left", 180.0f);
        this->declare_parameter("dyn_obj.fov_right", -180.0f);
        this->declare_parameter("dyn_obj.checkneighbor_range", 1);
        this->declare_parameter("dyn_obj.stop_object_detect", false);
        this->declare_parameter("dyn_obj.depth_thr1", 0.15f);
        this->declare_parameter("dyn_obj.enter_min_thr1", 0.15f);
        this->declare_parameter("dyn_obj.enter_max_thr1", 0.15f);
        this->declare_parameter("dyn_obj.map_cons_depth_thr1", 0.5f);
        this->declare_parameter("dyn_obj.map_cons_hor_thr1", 0.01f);
        this->declare_parameter("dyn_obj.map_cons_ver_thr1", 0.01f);
        this->declare_parameter("dyn_obj.map_cons_hor_dis1", 0.2f);
        this->declare_parameter("dyn_obj.map_cons_ver_dis1", 0.1f);
        this->declare_parameter("dyn_obj.depth_cons_depth_thr1", 0.5f);
        this->declare_parameter("dyn_obj.depth_cons_depth_max_thr1", 0.5f);
        this->declare_parameter("dyn_obj.depth_cons_hor_thr1", 0.02f);
        this->declare_parameter("dyn_obj.depth_cons_ver_thr1", 0.01f);
        this->declare_parameter("dyn_obj.enlarge_z_thr1", 0.05f);
        this->declare_parameter("dyn_obj.enlarge_angle", 2.0f);
        this->declare_parameter("dyn_obj.enlarge_depth", 3.0f);
        this->declare_parameter("dyn_obj.occluded_map_thr1", 3);
        this->declare_parameter("dyn_obj.case1_interp_en", false);
        this->declare_parameter("dyn_obj.k_depth_min_thr1", 0.0f);
        this->declare_parameter("dyn_obj.d_depth_min_thr1", 0.15f);
        this->declare_parameter("dyn_obj.k_depth_max_thr1", 0.0f);
        this->declare_parameter("dyn_obj.d_depth_max_thr1", 0.15f);
        this->declare_parameter("dyn_obj.v_min_thr2", 0.5f);
        this->declare_parameter("dyn_obj.acc_thr2", 1.0f);
        this->declare_parameter("dyn_obj.map_cons_depth_thr2", 0.15f);
        this->declare_parameter("dyn_obj.map_cons_hor_thr2", 0.02f);
        this->declare_parameter("dyn_obj.map_cons_ver_thr2", 0.01f);
        this->declare_parameter("dyn_obj.occ_depth_thr2", 0.15f);
        this->declare_parameter("dyn_obj.occ_hor_thr2", 0.02f);
        this->declare_parameter("dyn_obj.occ_ver_thr2", 0.01f);
        this->declare_parameter("dyn_obj.depth_cons_depth_thr2", 0.5f);
        this->declare_parameter("dyn_obj.depth_cons_depth_max_thr2", 0.5f);
        this->declare_parameter("dyn_obj.depth_cons_hor_thr2", 0.02f);
        this->declare_parameter("dyn_obj.depth_cons_ver_thr2", 0.01f);
        this->declare_parameter("dyn_obj.k_depth2", 0.005f);
        this->declare_parameter("dyn_obj.occluded_times_thr2", 3);
        this->declare_parameter("dyn_obj.case2_interp_en", false);
        this->declare_parameter("dyn_obj.k_depth_max_thr2", 0.0f);
        this->declare_parameter("dyn_obj.d_depth_max_thr2", 0.15f);
        this->declare_parameter("dyn_obj.v_min_thr3", 0.5f);
        this->declare_parameter("dyn_obj.acc_thr3", 1.0f);
        this->declare_parameter("dyn_obj.map_cons_depth_thr3", 0.15f);
        this->declare_parameter("dyn_obj.map_cons_hor_thr3", 0.02f);
        this->declare_parameter("dyn_obj.map_cons_ver_thr3", 0.01f);
        this->declare_parameter("dyn_obj.occ_depth_thr3", 0.15f);
        this->declare_parameter("dyn_obj.occ_hor_thr3", 0.02f);
        this->declare_parameter("dyn_obj.occ_ver_thr3", 0.01f);
        this->declare_parameter("dyn_obj.depth_cons_depth_thr3", 0.5f);
        this->declare_parameter("dyn_obj.depth_cons_depth_max_thr3", 0.5f);
        this->declare_parameter("dyn_obj.depth_cons_hor_thr3", 0.02f);
        this->declare_parameter("dyn_obj.depth_cons_ver_thr3", 0.01f);
        this->declare_parameter("dyn_obj.k_depth3", 0.005f);
        this->declare_parameter("dyn_obj.occluding_times_thr3", 3);
        this->declare_parameter("dyn_obj.case3_interp_en", false);
        this->declare_parameter("dyn_obj.k_depth_max_thr3", 0.0f);
        this->declare_parameter("dyn_obj.d_depth_max_thr3", 0.15f);
        this->declare_parameter("dyn_obj.interp_hor_thr", 0.01f);
        this->declare_parameter("dyn_obj.interp_ver_thr", 0.01f);
        this->declare_parameter("dyn_obj.interp_thr1", 1.0f);
        this->declare_parameter("dyn_obj.interp_static_max", 10.0f);
        this->declare_parameter("dyn_obj.interp_start_depth1", 20.0f);
        this->declare_parameter("dyn_obj.interp_kp1", 0.1f);
        this->declare_parameter("dyn_obj.interp_kd1", 1.0f);
        this->declare_parameter("dyn_obj.interp_thr2", 0.15f);
        this->declare_parameter("dyn_obj.interp_thr3", 0.15f);
        this->declare_parameter("dyn_obj.dyn_filter_en", true);
        this->declare_parameter("dyn_obj.debug_publish", true);
        this->declare_parameter("dyn_obj.laserCloudSteadObj_accu_limit", 5);
        this->declare_parameter("dyn_obj.voxel_filter_size", 0.1f);
        this->declare_parameter("dyn_obj.cluster_coupled", false);
        this->declare_parameter("dyn_obj.cluster_future", false);
        this->declare_parameter("dyn_obj.cluster_extend_pixel", 2);
        this->declare_parameter("dyn_obj.cluster_min_pixel_number", 4);
        this->declare_parameter("dyn_obj.cluster_thrustable_thresold", 0.3f);
        this->declare_parameter("dyn_obj.cluster_Voxel_revolusion", 0.3f);
        this->declare_parameter("dyn_obj.cluster_debug_en", false);
        this->declare_parameter("dyn_obj.cluster_out_file", "");
        this->declare_parameter("dyn_obj.ver_resolution_max", 0.0025f);
        this->declare_parameter("dyn_obj.hor_resolution_max", 0.0025f);
        this->declare_parameter("dyn_obj.buffer_dur", 0.1f);
        this->declare_parameter("dyn_obj.point_index", 0);
        this->declare_parameter("dyn_obj.frame_id", "camera_init");
        this->declare_parameter("dyn_obj.time_file", "");
        this->declare_parameter("dyn_obj.time_breakdown_file", "");
        
        // Get all parameters and assign to DynObjFilt members
        DynObjFilt->buffer_delay = this->get_parameter("dyn_obj.buffer_delay").as_double();
        DynObjFilt->buffer_size = this->get_parameter("dyn_obj.buffer_size").as_int();
        DynObjFilt->points_num_perframe = this->get_parameter("dyn_obj.points_num_perframe").as_int();
        DynObjFilt->depth_map_dur = this->get_parameter("dyn_obj.depth_map_dur").as_double();
        DynObjFilt->max_depth_map_num = this->get_parameter("dyn_obj.max_depth_map_num").as_int();
        DynObjFilt->max_pixel_points = this->get_parameter("dyn_obj.max_pixel_points").as_int();
        DynObjFilt->frame_dur = this->get_parameter("dyn_obj.frame_dur").as_double();
        DynObjFilt->dataset = this->get_parameter("dyn_obj.dataset").as_int();
        DynObjFilt->self_x_f = this->get_parameter("dyn_obj.self_x_f").as_double();
        DynObjFilt->self_x_b = this->get_parameter("dyn_obj.self_x_b").as_double();
        DynObjFilt->self_y_l = this->get_parameter("dyn_obj.self_y_l").as_double();
        DynObjFilt->self_y_r = this->get_parameter("dyn_obj.self_y_r").as_double();
        DynObjFilt->blind_dis = this->get_parameter("dyn_obj.blind_dis").as_double();
        DynObjFilt->fov_up = this->get_parameter("dyn_obj.fov_up").as_double();
        DynObjFilt->fov_down = this->get_parameter("dyn_obj.fov_down").as_double();
        DynObjFilt->fov_cut = this->get_parameter("dyn_obj.fov_cut").as_double();
        DynObjFilt->fov_left = this->get_parameter("dyn_obj.fov_left").as_double();
        DynObjFilt->fov_right = this->get_parameter("dyn_obj.fov_right").as_double();
        DynObjFilt->checkneighbor_range = this->get_parameter("dyn_obj.checkneighbor_range").as_int();
        DynObjFilt->stop_object_detect = this->get_parameter("dyn_obj.stop_object_detect").as_bool();
        DynObjFilt->depth_thr1 = this->get_parameter("dyn_obj.depth_thr1").as_double();
        DynObjFilt->enter_min_thr1 = this->get_parameter("dyn_obj.enter_min_thr1").as_double();
        DynObjFilt->enter_max_thr1 = this->get_parameter("dyn_obj.enter_max_thr1").as_double();
        DynObjFilt->map_cons_depth_thr1 = this->get_parameter("dyn_obj.map_cons_depth_thr1").as_double();
        DynObjFilt->map_cons_hor_thr1 = this->get_parameter("dyn_obj.map_cons_hor_thr1").as_double();
        DynObjFilt->map_cons_ver_thr1 = this->get_parameter("dyn_obj.map_cons_ver_thr1").as_double();
        DynObjFilt->map_cons_hor_dis1 = this->get_parameter("dyn_obj.map_cons_hor_dis1").as_double();
        DynObjFilt->map_cons_ver_dis1 = this->get_parameter("dyn_obj.map_cons_ver_dis1").as_double();
        DynObjFilt->depth_cons_depth_thr1 = this->get_parameter("dyn_obj.depth_cons_depth_thr1").as_double();
        DynObjFilt->depth_cons_depth_max_thr1 = this->get_parameter("dyn_obj.depth_cons_depth_max_thr1").as_double();
        DynObjFilt->depth_cons_hor_thr1 = this->get_parameter("dyn_obj.depth_cons_hor_thr1").as_double();
        DynObjFilt->depth_cons_ver_thr1 = this->get_parameter("dyn_obj.depth_cons_ver_thr1").as_double();
        DynObjFilt->enlarge_z_thr1 = this->get_parameter("dyn_obj.enlarge_z_thr1").as_double();
        DynObjFilt->enlarge_angle = this->get_parameter("dyn_obj.enlarge_angle").as_double();
        DynObjFilt->enlarge_depth = this->get_parameter("dyn_obj.enlarge_depth").as_double();
        DynObjFilt->occluded_map_thr1 = this->get_parameter("dyn_obj.occluded_map_thr1").as_int();
        DynObjFilt->case1_interp_en = this->get_parameter("dyn_obj.case1_interp_en").as_bool();
        DynObjFilt->k_depth_min_thr1 = this->get_parameter("dyn_obj.k_depth_min_thr1").as_double();
        DynObjFilt->d_depth_min_thr1 = this->get_parameter("dyn_obj.d_depth_min_thr1").as_double();
        DynObjFilt->k_depth_max_thr1 = this->get_parameter("dyn_obj.k_depth_max_thr1").as_double();
        DynObjFilt->d_depth_max_thr1 = this->get_parameter("dyn_obj.d_depth_max_thr1").as_double();
        DynObjFilt->v_min_thr2 = this->get_parameter("dyn_obj.v_min_thr2").as_double();
        DynObjFilt->acc_thr2 = this->get_parameter("dyn_obj.acc_thr2").as_double();
        DynObjFilt->map_cons_depth_thr2 = this->get_parameter("dyn_obj.map_cons_depth_thr2").as_double();
        DynObjFilt->map_cons_hor_thr2 = this->get_parameter("dyn_obj.map_cons_hor_thr2").as_double();
        DynObjFilt->map_cons_ver_thr2 = this->get_parameter("dyn_obj.map_cons_ver_thr2").as_double();
        DynObjFilt->occ_depth_thr2 = this->get_parameter("dyn_obj.occ_depth_thr2").as_double();
        DynObjFilt->occ_hor_thr2 = this->get_parameter("dyn_obj.occ_hor_thr2").as_double();
        DynObjFilt->occ_ver_thr2 = this->get_parameter("dyn_obj.occ_ver_thr2").as_double();
        DynObjFilt->depth_cons_depth_thr2 = this->get_parameter("dyn_obj.depth_cons_depth_thr2").as_double();
        DynObjFilt->depth_cons_depth_max_thr2 = this->get_parameter("dyn_obj.depth_cons_depth_max_thr2").as_double();
        DynObjFilt->depth_cons_hor_thr2 = this->get_parameter("dyn_obj.depth_cons_hor_thr2").as_double();
        DynObjFilt->depth_cons_ver_thr2 = this->get_parameter("dyn_obj.depth_cons_ver_thr2").as_double();
        DynObjFilt->k_depth2 = this->get_parameter("dyn_obj.k_depth2").as_double();
        DynObjFilt->occluded_times_thr2 = this->get_parameter("dyn_obj.occluded_times_thr2").as_int();
        DynObjFilt->case2_interp_en = this->get_parameter("dyn_obj.case2_interp_en").as_bool();
        DynObjFilt->k_depth_max_thr2 = this->get_parameter("dyn_obj.k_depth_max_thr2").as_double();
        DynObjFilt->d_depth_max_thr2 = this->get_parameter("dyn_obj.d_depth_max_thr2").as_double();
        DynObjFilt->v_min_thr3 = this->get_parameter("dyn_obj.v_min_thr3").as_double();
        DynObjFilt->acc_thr3 = this->get_parameter("dyn_obj.acc_thr3").as_double();
        DynObjFilt->map_cons_depth_thr3 = this->get_parameter("dyn_obj.map_cons_depth_thr3").as_double();
        DynObjFilt->map_cons_hor_thr3 = this->get_parameter("dyn_obj.map_cons_hor_thr3").as_double();
        DynObjFilt->map_cons_ver_thr3 = this->get_parameter("dyn_obj.map_cons_ver_thr3").as_double();
        DynObjFilt->occ_depth_thr3 = this->get_parameter("dyn_obj.occ_depth_thr3").as_double();
        DynObjFilt->occ_hor_thr3 = this->get_parameter("dyn_obj.occ_hor_thr3").as_double();
        DynObjFilt->occ_ver_thr3 = this->get_parameter("dyn_obj.occ_ver_thr3").as_double();
        DynObjFilt->depth_cons_depth_thr3 = this->get_parameter("dyn_obj.depth_cons_depth_thr3").as_double();
        DynObjFilt->depth_cons_depth_max_thr3 = this->get_parameter("dyn_obj.depth_cons_depth_max_thr3").as_double();
        DynObjFilt->depth_cons_hor_thr3 = this->get_parameter("dyn_obj.depth_cons_hor_thr3").as_double();
        DynObjFilt->depth_cons_ver_thr3 = this->get_parameter("dyn_obj.depth_cons_ver_thr3").as_double();
        DynObjFilt->k_depth3 = this->get_parameter("dyn_obj.k_depth3").as_double();
        DynObjFilt->occluding_times_thr3 = this->get_parameter("dyn_obj.occluding_times_thr3").as_int();
        DynObjFilt->case3_interp_en = this->get_parameter("dyn_obj.case3_interp_en").as_bool();
        DynObjFilt->k_depth_max_thr3 = this->get_parameter("dyn_obj.k_depth_max_thr3").as_double();
        DynObjFilt->d_depth_max_thr3 = this->get_parameter("dyn_obj.d_depth_max_thr3").as_double();
        DynObjFilt->interp_hor_thr = this->get_parameter("dyn_obj.interp_hor_thr").as_double();
        DynObjFilt->interp_ver_thr = this->get_parameter("dyn_obj.interp_ver_thr").as_double();
        DynObjFilt->interp_thr1 = this->get_parameter("dyn_obj.interp_thr1").as_double();
        DynObjFilt->interp_static_max = this->get_parameter("dyn_obj.interp_static_max").as_double();
        DynObjFilt->interp_start_depth1 = this->get_parameter("dyn_obj.interp_start_depth1").as_double();
        DynObjFilt->interp_kp1 = this->get_parameter("dyn_obj.interp_kp1").as_double();
        DynObjFilt->interp_kd1 = this->get_parameter("dyn_obj.interp_kd1").as_double();
        DynObjFilt->interp_thr2 = this->get_parameter("dyn_obj.interp_thr2").as_double();
        DynObjFilt->interp_thr3 = this->get_parameter("dyn_obj.interp_thr3").as_double();
        DynObjFilt->dyn_filter_en = this->get_parameter("dyn_obj.dyn_filter_en").as_bool();
        DynObjFilt->debug_en = this->get_parameter("dyn_obj.debug_publish").as_bool();
        DynObjFilt->laserCloudSteadObj_accu_limit = this->get_parameter("dyn_obj.laserCloudSteadObj_accu_limit").as_int();
        DynObjFilt->voxel_filter_size = this->get_parameter("dyn_obj.voxel_filter_size").as_double();
        DynObjFilt->cluster_coupled = this->get_parameter("dyn_obj.cluster_coupled").as_bool();
        DynObjFilt->cluster_future = this->get_parameter("dyn_obj.cluster_future").as_bool();
        DynObjFilt->Cluster.cluster_extend_pixel = this->get_parameter("dyn_obj.cluster_extend_pixel").as_int();
        DynObjFilt->Cluster.cluster_min_pixel_number = this->get_parameter("dyn_obj.cluster_min_pixel_number").as_int();
        DynObjFilt->Cluster.thrustable_thresold = this->get_parameter("dyn_obj.cluster_thrustable_thresold").as_double();
        DynObjFilt->Cluster.Voxel_revolusion = this->get_parameter("dyn_obj.cluster_Voxel_revolusion").as_double();
        DynObjFilt->Cluster.debug_en = this->get_parameter("dyn_obj.cluster_debug_en").as_bool();
        DynObjFilt->Cluster.out_file = this->get_parameter("dyn_obj.cluster_out_file").as_string();
        DynObjFilt->hor_resolution_max = this->get_parameter("dyn_obj.hor_resolution_max").as_double();
        DynObjFilt->ver_resolution_max = this->get_parameter("dyn_obj.ver_resolution_max").as_double();
        DynObjFilt->buffer_dur = this->get_parameter("dyn_obj.buffer_dur").as_double();
        DynObjFilt->point_index = this->get_parameter("dyn_obj.point_index").as_int();
        DynObjFilt->frame_id = this->get_parameter("dyn_obj.frame_id").as_string();
        DynObjFilt->time_file = this->get_parameter("dyn_obj.time_file").as_string();
        DynObjFilt->time_breakdown_file = this->get_parameter("dyn_obj.time_breakdown_file").as_string();
        
        // Calculate derived values (from DynObjFilter::init() lines 117-175)
        DynObjFilt->max_ind = floor(3.1415926 * 2 / DynObjFilt->hor_resolution_max);
        
        // Initialize point cloud storage and data structures (lines 118-135)
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
                    DynObjFilt->pos_offset.push_back(((ind_hor)/2 + ind_hor%2)*xy_ind[ind_hor%2] * MAX_1D_HALF + ((ind_ver)/2 + ind_ver%2)*xy_ind[ind_ver%2]);
                }   
            }
        }
        
        // Calculate pixel numbers for various thresholds (lines 136-151)
        DynObjFilt->map_cons_hor_num1 = ceil(DynObjFilt->map_cons_hor_thr1/DynObjFilt->hor_resolution_max);
        DynObjFilt->map_cons_ver_num1 = ceil(DynObjFilt->map_cons_ver_thr1/DynObjFilt->ver_resolution_max);
        DynObjFilt->interp_hor_num = ceil(DynObjFilt->interp_hor_thr/DynObjFilt->hor_resolution_max);
        DynObjFilt->interp_ver_num = ceil(DynObjFilt->interp_ver_thr/DynObjFilt->ver_resolution_max);
        DynObjFilt->map_cons_hor_num2 = ceil(DynObjFilt->map_cons_hor_thr2/DynObjFilt->hor_resolution_max);
        DynObjFilt->map_cons_ver_num2 = ceil(DynObjFilt->map_cons_ver_thr2/DynObjFilt->ver_resolution_max);
        DynObjFilt->occ_hor_num2 = ceil(DynObjFilt->occ_hor_thr2/DynObjFilt->hor_resolution_max);
        DynObjFilt->occ_ver_num2 = ceil(DynObjFilt->occ_ver_thr2/DynObjFilt->ver_resolution_max);
        DynObjFilt->depth_cons_hor_num2 = ceil(DynObjFilt->depth_cons_hor_thr2/DynObjFilt->hor_resolution_max);
        DynObjFilt->depth_cons_ver_num2 = ceil(DynObjFilt->depth_cons_ver_thr2/DynObjFilt->ver_resolution_max);
        DynObjFilt->map_cons_hor_num3 = ceil(DynObjFilt->map_cons_hor_thr3/DynObjFilt->hor_resolution_max);
        DynObjFilt->map_cons_ver_num3 = ceil(DynObjFilt->map_cons_ver_thr3/DynObjFilt->ver_resolution_max);
        DynObjFilt->occ_hor_num3 = ceil(DynObjFilt->occ_hor_thr3/DynObjFilt->hor_resolution_max);
        DynObjFilt->occ_ver_num3 = ceil(DynObjFilt->occ_ver_thr3/DynObjFilt->ver_resolution_max);
        DynObjFilt->depth_cons_hor_num3 = ceil(DynObjFilt->depth_cons_hor_thr3/DynObjFilt->hor_resolution_max);
        DynObjFilt->depth_cons_ver_num3 = ceil(DynObjFilt->depth_cons_ver_thr3/DynObjFilt->ver_resolution_max);
        
        // Initialize buffer (line 152)
        DynObjFilt->buffer.init(DynObjFilt->buffer_size);
        
        // Calculate FOV pixels (lines 154-158)
        DynObjFilt->pixel_fov_up = floor((DynObjFilt->fov_up/180.0*PI_MATH + 0.5 * PI_MATH)/DynObjFilt->ver_resolution_max);
        DynObjFilt->pixel_fov_down = floor((DynObjFilt->fov_down/180.0*PI_MATH + 0.5 * PI_MATH)/DynObjFilt->ver_resolution_max);
        DynObjFilt->pixel_fov_cut = floor((DynObjFilt->fov_cut/180.0*PI_MATH + 0.5 * PI_MATH)/DynObjFilt->ver_resolution_max);
        DynObjFilt->pixel_fov_left = floor((DynObjFilt->fov_left/180.0*PI_MATH + PI_MATH)/DynObjFilt->hor_resolution_max);
        DynObjFilt->pixel_fov_right = floor((DynObjFilt->fov_right/180.0*PI_MATH + PI_MATH)/DynObjFilt->hor_resolution_max);
        
        // Initialize point_soph_pointers (lines 159-165)
        DynObjFilt->max_pointers_num = round((DynObjFilt->max_depth_map_num * DynObjFilt->depth_map_dur + DynObjFilt->buffer_delay)/DynObjFilt->frame_dur) + 1;
        DynObjFilt->point_soph_pointers.reserve(DynObjFilt->max_pointers_num);
        for (int i = 0; i < DynObjFilt->max_pointers_num; i++)
        {
            point_soph* p = new point_soph[DynObjFilt->points_num_perframe];
            DynObjFilt->point_soph_pointers.push_back(p);
        }
        
        // Open time files if specified (lines 166-173)
        if(DynObjFilt->time_file != "")
        {
            DynObjFilt->time_out.open(DynObjFilt->time_file, ios::out); 
        }
        if(DynObjFilt->time_breakdown_file != "")
        {
            DynObjFilt->time_breakdown_out.open(DynObjFilt->time_breakdown_file, ios::out); 
        }
        
        // Initialize Cluster (line 174)
        DynObjFilt->Cluster.Init();
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr cur_odom)
    {
        Eigen::Quaterniond cur_q;
        geometry_msgs::msg::Quaternion tmp_q;
        tmp_q = cur_odom->pose.pose.orientation;
        cur_q = Eigen::Quaterniond(tmp_q.w, tmp_q.x, tmp_q.y, tmp_q.z);
        cur_rot = cur_q.matrix();
        cur_pos << cur_odom->pose.pose.position.x, 
                   cur_odom->pose.pose.position.y, 
                   cur_odom->pose.pose.position.z;
        buffer_rots.push_back(cur_rot);
        buffer_poss.push_back(cur_pos);
        lidar_end_time = rclcpp::Time(cur_odom->header.stamp).seconds();
        buffer_times.push_back(lidar_end_time);
    }
    
    void pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_in)
    {
        boost::shared_ptr<PointCloudXYZI> feats_undistort(new PointCloudXYZI());
        pcl::fromROSMsg(*msg_in, *feats_undistort);
        buffer_pcs.push_back(feats_undistort);
    }
    
    void timerCallback()
    {
        if(buffer_pcs.size() > 0 && buffer_poss.size() > 0 && 
           buffer_rots.size() > 0 && buffer_times.size() > 0)
        {
            boost::shared_ptr<PointCloudXYZI> cur_pc = buffer_pcs.at(0);
            buffer_pcs.pop_front();
            auto cur_rot = buffer_rots.at(0);
            buffer_rots.pop_front();
            auto cur_pos = buffer_poss.at(0);
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
            
            DynObjFilt->filter(cur_pc, cur_rot, cur_pos, cur_time);
            publishDyn(cur_time);
            cur_frame++;
        }
    }
    
    void publishDyn(const double& scan_end_time)
    {
        // ROS2 adaptation of DynObjFilter::publish_dyn() (lines 1821-1903)
        
        // Print dynamic object statistics
        if(DynObjFilt->cluster_coupled)
        {    
            cout<<"Found Dynamic Objects, numbers: " << DynObjFilt->laserCloudDynObj_clus->points.size() 
                << " Total time: " << DynObjFilt->time_total 
                << " Average total time: "<< DynObjFilt->time_total_avr << endl;
        }
        else
        {
            cout<<"Found Dynamic Objects, numbers: " << DynObjFilt->laserCloudDynObj->points.size() 
                << " Total time: " << DynObjFilt->time_total 
                << " Average total time: "<< DynObjFilt->time_total_avr << endl;
        }
        cout<<"case1 num: "<<DynObjFilt->case1_num<<" case2 num: "<<DynObjFilt->case2_num
            <<" case3 num: "<<DynObjFilt->case3_num<<endl;
        DynObjFilt->case1_num = 0;
        DynObjFilt->case2_num = 0;
        DynObjFilt->case3_num = 0;
        
        // Publish dynamic objects in world frame
        sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*DynObjFilt->laserCloudDynObj_world, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = rclcpp::Time(static_cast<int64_t>(scan_end_time * 1e9));
        laserCloudFullRes3.header.frame_id = DynObjFilt->frame_id;
        pub_pcl_dyn->publish(laserCloudFullRes3);
        
        // Publish clustered dynamic objects if clustering is enabled
        if(DynObjFilt->cluster_coupled || DynObjFilt->cluster_future)
        {
            sensor_msgs::msg::PointCloud2 laserCloudFullRes4;
            pcl::toROSMsg(*DynObjFilt->laserCloudDynObj_clus, laserCloudFullRes4);
            laserCloudFullRes4.header.stamp = rclcpp::Time(static_cast<int64_t>(scan_end_time * 1e9));
            laserCloudFullRes4.header.frame_id = DynObjFilt->frame_id;
            pub_pcl_dyn_extend->publish(laserCloudFullRes4);
        }
        
        // Prepare and publish steady/static objects
        sensor_msgs::msg::PointCloud2 laserCloudFullRes2;
        PointCloudXYZI::Ptr laserCloudSteadObj_pub(new PointCloudXYZI);
        
        if(DynObjFilt->cluster_coupled)
        {
            // Accumulate steady objects with clustering
            if(DynObjFilt->laserCloudSteadObj_accu_times < DynObjFilt->laserCloudSteadObj_accu_limit)
            {
                DynObjFilt->laserCloudSteadObj_accu_times++;
                DynObjFilt->laserCloudSteadObj_accu.push_back(DynObjFilt->laserCloudSteadObj_clus);
                for(int i = 0; i < DynObjFilt->laserCloudSteadObj_accu.size(); i++)
                {
                    *laserCloudSteadObj_pub += *DynObjFilt->laserCloudSteadObj_accu[i];
                }
            }
            else
            {   
                DynObjFilt->laserCloudSteadObj_accu.pop_front();
                DynObjFilt->laserCloudSteadObj_accu.push_back(DynObjFilt->laserCloudSteadObj_clus);
                for(int i = 0; i < DynObjFilt->laserCloudSteadObj_accu.size(); i++)
                {
                    *laserCloudSteadObj_pub += *DynObjFilt->laserCloudSteadObj_accu[i];
                }
            }
            // Apply voxel filter to downsample
            pcl::VoxelGrid<PointType> downSizeFiltermap;
            downSizeFiltermap.setLeafSize(DynObjFilt->voxel_filter_size, 
                                         DynObjFilt->voxel_filter_size, 
                                         DynObjFilt->voxel_filter_size);
            downSizeFiltermap.setInputCloud(laserCloudSteadObj_pub);
            PointCloudXYZI laserCloudSteadObj_down;
            downSizeFiltermap.filter(laserCloudSteadObj_down);
            pcl::toROSMsg(laserCloudSteadObj_down, laserCloudFullRes2);
        }
        else
        {
            cout<<"Found Steady Objects, numbers: " << DynObjFilt->laserCloudSteadObj->points.size() << endl;
            // Accumulate steady objects without clustering
            if(DynObjFilt->laserCloudSteadObj_accu_times < DynObjFilt->laserCloudSteadObj_accu_limit)
            {
                DynObjFilt->laserCloudSteadObj_accu_times++;
                DynObjFilt->laserCloudSteadObj_accu.push_back(DynObjFilt->laserCloudSteadObj);
                for(int i = 0; i < DynObjFilt->laserCloudSteadObj_accu.size(); i++)
                {
                    *laserCloudSteadObj_pub += *DynObjFilt->laserCloudSteadObj_accu[i];
                }
            }
            else
            {   
                DynObjFilt->laserCloudSteadObj_accu.pop_front();
                DynObjFilt->laserCloudSteadObj_accu.push_back(DynObjFilt->laserCloudSteadObj);
                for(int i = 0; i < DynObjFilt->laserCloudSteadObj_accu.size(); i++)
                {
                    *laserCloudSteadObj_pub += *DynObjFilt->laserCloudSteadObj_accu[i];
                }
            }
            pcl::toROSMsg(*laserCloudSteadObj_pub, laserCloudFullRes2);
        }
        laserCloudFullRes2.header.stamp = rclcpp::Time(static_cast<int64_t>(scan_end_time * 1e9));
        laserCloudFullRes2.header.frame_id = DynObjFilt->frame_id;
        pub_pcl_std->publish(laserCloudFullRes2);
    }

    // Member variables
    shared_ptr<DynObjFilter> DynObjFilt;
    M3D cur_rot = Eigen::Matrix3d::Identity();
    V3D cur_pos = Eigen::Vector3d::Zero();
    
    string points_topic, odom_topic;
    string out_folder, out_folder_origin;
    double lidar_end_time = 0;
    int cur_frame = 0;
    
    deque<M3D> buffer_rots;
    deque<V3D> buffer_poss;
    deque<double> buffer_times;
    deque<boost::shared_ptr<PointCloudXYZI>> buffer_pcs;
    
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
