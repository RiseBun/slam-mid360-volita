// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <functional>
#include <random>
#include <atomic>

// ROS2 lib
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Livox ROS2 driver
#include <livox_ros_driver2/msg/custom_msg.hpp>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>

// ROS2 service
#include <std_srvs/srv/trigger.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "common/utility.h"
#include "preprocess/cloud_convert/cloud_convert2.h"
#include "lio/lidarodom.h"

#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "log/" + name))

static inline double toSec(const builtin_interfaces::msg::Time &stamp)
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

static inline builtin_interfaces::msg::Time fromSec(double t)
{
    builtin_interfaces::msg::Time msg;
    msg.sec = static_cast<int32_t>(t);
    msg.nanosec = static_cast<uint32_t>((t - msg.sec) * 1e9);
    return msg;
}

class AdaptiveLioNode : public rclcpp::Node
{
public:
    // Safe voxel downsample: shifts cloud centroid to origin before filtering
    // to avoid PCL VoxelGrid int32 index overflow on large-extent maps.
    static bool safeVoxelDownsample(
        pcl::PointCloud<pcl::PointXYZI>::Ptr input,
        pcl::PointCloud<pcl::PointXYZI>::Ptr output,
        float leaf_size)
    {
        if (!input || input->empty())
            return false;

        // Compute centroid to shift coordinates near origin
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*input, centroid);

        // Shift input to origin (avoid int32 overflow in VoxelGrid hash)
        for (auto &pt : input->points)
        {
            pt.x -= centroid[0];
            pt.y -= centroid[1];
            pt.z -= centroid[2];
        }

        pcl::VoxelGrid<pcl::PointXYZI> vf;
        vf.setInputCloud(input);
        vf.setLeafSize(leaf_size, leaf_size, leaf_size);
        vf.filter(*output);

        // Shift filtered result back to world coordinates
        for (auto &pt : output->points)
        {
            pt.x += centroid[0];
            pt.y += centroid[1];
            pt.z += centroid[2];
        }

        // Also restore input (it was modified in-place)
        for (auto &pt : input->points)
        {
            pt.x += centroid[0];
            pt.y += centroid[1];
            pt.z += centroid[2];
        }

        return true;
    }

    AdaptiveLioNode() : Node("adaptive_lio_node")
    {
        // glog
        google::InitGoogleLogging("adaptive_lio_node");
        FLAGS_stderrthreshold = google::INFO;
        FLAGS_colorlogtostderr = true;

        // global map (kept continuously downsampled for fast export)
        global_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        map_save_path_ = std::string(ROOT_DIR) + "map/";
        global_map_frame_count_ = 0;
        global_map_downsample_interval_ = 5;  // downsample every 5 frames (was 50)

        // Segmented map saving for memory management
        segment_distance_threshold_ = 50.0;  // Save and clear map every 50 meters
        segment_index_ = 0;
        last_segment_position_ = Eigen::Vector3d::Zero();
        segment_initialized_ = false;
        max_map_points_ = 2000000;  // Hard limit: 2 million points (~64MB)

        // config file
        std::string config_file = std::string(ROOT_DIR) + "config/mapping_m.yaml";
        RCLCPP_INFO(this->get_logger(), "config_file: %s", config_file.c_str());

        // init core algorithm
        lio_ = new zjloc::lidarodom_m();
        if (!lio_->init(config_file))
        {
            RCLCPP_ERROR(this->get_logger(), "LIO init failed!");
            rclcpp::shutdown();
            return;
        }

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // Publishers
        pub_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan", 10);
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 100);
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/odometry_path", 5);
        pub_vel_ = this->create_publisher<std_msgs::msg::Float32>("/velocity", 1);
        pub_dist_ = this->create_publisher<std_msgs::msg::Float32>("/move_dist", 1);
        pub_imu_repub_ = this->create_publisher<sensor_msgs::msg::Imu>("/repub_imu", 1);

        // Setup std::function callbacks for core algorithm
        setupCallbacks();

        // Cloud converter
        convert_ = new zjloc::CloudConvert2();
        convert_->LoadFromYAML(config_file);
        lio_->setCloudConvert(convert_);

        RCLCPP_INFO(this->get_logger(), "init successful");

        // Read topics from yaml
        auto yaml = YAML::LoadFile(config_file);
        std::string laser_topic = yaml["common"]["lid_topic"].as<std::string>();
        std::string imu_topic = yaml["common"]["imu_topic"].as<std::string>();
        gnorm_ = yaml["common"]["gnorm"].as<double>();

        // Read map save path from config (empty string means use default)
        if (yaml["common"]["map_save_path"])
        {
            std::string custom_path = yaml["common"]["map_save_path"].as<std::string>();
            if (!custom_path.empty())
            {
                // Ensure path ends with /
                if (custom_path.back() != '/')
                    custom_path += "/";
                map_save_path_ = custom_path;
                RCLCPP_INFO(this->get_logger(), "Map save path (from config): %s", map_save_path_.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Map save path (default): %s", map_save_path_.c_str());
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Map save path (default): %s", map_save_path_.c_str());
        }

        // Read map voxel size from config (controls point density)
        if (yaml["common"]["map_voxel_size"])
        {
            map_voxel_size_ = yaml["common"]["map_voxel_size"].as<float>();
            if (map_voxel_size_ < 0.01f) map_voxel_size_ = 0.01f;  // minimum 1cm
            if (map_voxel_size_ > 1.0f) map_voxel_size_ = 1.0f;    // maximum 1m
        }
        RCLCPP_INFO(this->get_logger(), "Map voxel size: %.3f m (smaller = denser points)", map_voxel_size_);

        // Subscribers
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 500,
            std::bind(&AdaptiveLioNode::imuCallback, this, std::placeholders::_1));

        if (convert_->lidar_type_ == zjloc::CloudConvert2::LidarType::AVIA)
        {
            sub_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                laser_topic, 100,
                std::bind(&AdaptiveLioNode::livoxCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribing Livox CustomMsg on: %s", laser_topic.c_str());
        }
        else
        {
            sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                laser_topic, 100,
                std::bind(&AdaptiveLioNode::standardPclCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribing PointCloud2 on: %s", laser_topic.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "IMU topic: %s, gnorm: %f", imu_topic.c_str(), gnorm_);

        // Save map service
        save_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "save_map",
            std::bind(&AdaptiveLioNode::saveMapCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Save map service: ros2 service call /save_map std_srvs/srv/Trigger");

        // Save map status service (check if async save is complete)
        save_map_status_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "save_map_status",
            std::bind(&AdaptiveLioNode::saveMapStatusCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Streaming snapshot timer: writes map to disk every 30 seconds in background
        snapshot_timer_ = this->create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&AdaptiveLioNode::snapshotTimerCallback, this));
        RCLCPP_INFO(this->get_logger(), "Streaming snapshot enabled (every 30s)");

        // Start processing thread
        process_thread_ = std::thread(&zjloc::lidarodom_m::run, lio_);
    }

    ~AdaptiveLioNode()
    {
        // Stop streaming snapshot timer
        snapshot_timer_.reset();

        // Wait for any background save/snapshot/segment threads to finish
        if (save_thread_.joinable())
            save_thread_.join();
        if (snapshot_thread_.joinable())
            snapshot_thread_.join();
        if (segment_save_thread_.joinable())
            segment_save_thread_.join();

        // Auto-save remaining map as final segment
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (global_map_ && !global_map_->empty())
            {
                std::string cmd = "mkdir -p " + map_save_path_;
                system(cmd.c_str());

                // Save remaining points as final segment
                std::string segment_path = map_save_path_ + "segment_" + 
                                           std::to_string(segment_index_) + ".pcd";
                auto start = std::chrono::steady_clock::now();
                if (pcl::io::savePCDFileBinary(segment_path, *global_map_) == 0)
                {
                    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          std::chrono::steady_clock::now() - start)
                                          .count();
                    RCLCPP_INFO(this->get_logger(),
                                "[Memory Management] Final segment %d saved: %s (%zu points, %.1fs)",
                                segment_index_, segment_path.c_str(), global_map_->size(), elapsed_ms / 1000.0);
                }
            }
        }

        // Merge all segments into final global_map.pcd
        mergeAllSegments();

        if (process_thread_.joinable())
            process_thread_.detach();

        zjloc::common::Timer::PrintAll();
        zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt"));

        if (lio_)
            delete lio_;
        if (convert_)
            delete convert_;
    }

    // Merge all segment files into a single global_map.pcd
    // Uses batch merging to avoid memory exhaustion on large maps
    void mergeAllSegments()
    {
        if (segment_index_ < 0)
        {
            RCLCPP_WARN(this->get_logger(), "[Memory Management] No segments to merge (segment_index_=%d)", segment_index_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "[Memory Management] Starting merge of %d segment files...", segment_index_ + 1);

        pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>());
        int loaded_count = 0;
        int failed_count = 0;
        const int BATCH_SIZE = 5;  // Merge and downsample every 5 segments to control memory

        for (int i = 0; i <= segment_index_; i++)
        {
            std::string segment_path = map_save_path_ + "segment_" + std::to_string(i) + ".pcd";
            pcl::PointCloud<pcl::PointXYZI>::Ptr segment(new pcl::PointCloud<pcl::PointXYZI>());

            int ret = pcl::io::loadPCDFile(segment_path, *segment);
            if (ret == 0 && !segment->empty())
            {
                *merged += *segment;
                loaded_count++;
                RCLCPP_INFO(this->get_logger(), "[Memory Management] Loaded segment_%d.pcd (%zu points, merged total: %zu)",
                            i, segment->size(), merged->size());
            }
            else
            {
                failed_count++;
                RCLCPP_WARN(this->get_logger(), "[Memory Management] Failed to load segment_%d.pcd (ret=%d)", i, ret);
            }

            // Release segment memory immediately
            segment.reset();

            // Batch downsample every BATCH_SIZE segments to control memory usage
            if ((i + 1) % BATCH_SIZE == 0 && !merged->empty())
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
                if (safeVoxelDownsample(merged, filtered, map_voxel_size_))
                {
                    RCLCPP_INFO(this->get_logger(), "[Memory Management] Batch downsample: %zu -> %zu points",
                                merged->size(), filtered->size());
                    merged.swap(filtered);
                }
            }
        }

        if (loaded_count == 0 || merged->empty())
        {
            RCLCPP_ERROR(this->get_logger(), "[Memory Management] No segments loaded successfully (loaded=%d, failed=%d)",
                         loaded_count, failed_count);
            return;
        }

        // Final downsample
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        if (safeVoxelDownsample(merged, filtered, map_voxel_size_))
        {
            std::string final_path = map_save_path_ + "global_map.pcd";
            auto start = std::chrono::steady_clock::now();
            if (pcl::io::savePCDFileBinary(final_path, *filtered) == 0)
            {
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::steady_clock::now() - start)
                                      .count();
                RCLCPP_INFO(this->get_logger(),
                            "[Memory Management] SUCCESS: Merged %d segments -> %s (%zu points, %.1fs)",
                            loaded_count, final_path.c_str(), filtered->size(), elapsed_ms / 1000.0);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "[Memory Management] Failed to save final map to %s", final_path.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "[Memory Management] Final downsample failed");
        }
    }

private:
    void setupCallbacks()
    {
        // Cloud publish callback
        auto cloud_pub_func = std::function<bool(std::string &topic_name, zjloc::CloudPtr &cloud, double time)>(
            [this](std::string &topic_name, zjloc::CloudPtr &cloud, double time) -> bool
            {
                sensor_msgs::msg::PointCloud2 cloud_msg;
                pcl::toROSMsg(*cloud, cloud_msg);
                cloud_msg.header.stamp = fromSec(time);
                cloud_msg.header.frame_id = "map";
                if (topic_name == "laser")
                {
                    pub_scan_->publish(cloud_msg);
                    // Accumulate to global map with aggressive periodic downsampling
                    std::lock_guard<std::mutex> lock(map_mutex_);
                    *global_map_ += *cloud;
                    global_map_frame_count_++;
                    // Downsample every 5 frames to keep map export-ready at all times
                    if (global_map_frame_count_ % global_map_downsample_interval_ == 0)
                    {
                        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
                        if (safeVoxelDownsample(global_map_, filtered, map_voxel_size_))
                            global_map_.swap(filtered);
                    }
                }
                return true;
            });

        // Pose publish callback (odometry + tf + path)
        auto pose_pub_func = std::function<bool(std::string &topic_name, SE3 &pose, double stamp)>(
            [this](std::string &topic_name, SE3 &pose, double stamp) -> bool
            {
                Eigen::Quaterniond q_current(pose.so3().matrix());
                Eigen::Vector3d current_pos = pose.translation();

                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header.stamp = fromSec(stamp);

                if (topic_name == "laser")
                {
                    tf_msg.header.frame_id = "map";
                    tf_msg.child_frame_id = "base_link";
                    tf_msg.transform.translation.x = current_pos.x();
                    tf_msg.transform.translation.y = current_pos.y();
                    tf_msg.transform.translation.z = current_pos.z();
                    tf_msg.transform.rotation.x = q_current.x();
                    tf_msg.transform.rotation.y = q_current.y();
                    tf_msg.transform.rotation.z = q_current.z();
                    tf_msg.transform.rotation.w = q_current.w();
                    tf_broadcaster_->sendTransform(tf_msg);

                    // Publish odometry
                    nav_msgs::msg::Odometry odom;
                    odom.header.frame_id = "map";
                    odom.child_frame_id = "base_link";
                    odom.header.stamp = fromSec(stamp);
                    odom.pose.pose.orientation.x = q_current.x();
                    odom.pose.pose.orientation.y = q_current.y();
                    odom.pose.pose.orientation.z = q_current.z();
                    odom.pose.pose.orientation.w = q_current.w();
                    odom.pose.pose.position.x = current_pos.x();
                    odom.pose.pose.position.y = current_pos.y();
                    odom.pose.pose.position.z = current_pos.z();
                    pub_odom_->publish(odom);

                    // Publish path
                    geometry_msgs::msg::PoseStamped laser_pose;
                    laser_pose.header = odom.header;
                    laser_pose.pose = odom.pose.pose;
                    laser_odo_path_.header.stamp = odom.header.stamp;
                    laser_odo_path_.poses.push_back(laser_pose);
                    laser_odo_path_.header.frame_id = "map";
                    pub_path_->publish(laser_odo_path_);

                    // Check if we need to save segment and clear memory
                    checkAndSaveSegment(current_pos);
                }
                else if (topic_name == "world")
                {
                    tf_msg.header.frame_id = "world";
                    tf_msg.child_frame_id = "map";
                    tf_msg.transform.translation.x = pose.translation().x();
                    tf_msg.transform.translation.y = pose.translation().y();
                    tf_msg.transform.translation.z = pose.translation().z();
                    tf_msg.transform.rotation.x = q_current.x();
                    tf_msg.transform.rotation.y = q_current.y();
                    tf_msg.transform.rotation.z = q_current.z();
                    tf_msg.transform.rotation.w = q_current.w();
                    tf_broadcaster_->sendTransform(tf_msg);
                }
                return true;
            });

        // Data publish callback (velocity, distance)
        auto data_pub_func = std::function<bool(std::string &topic_name, double time1, double time2)>(
            [this](std::string &topic_name, double time1, double time2) -> bool
            {
                std_msgs::msg::Float32 msg;
                msg.data = time1;
                if (topic_name == "velocity")
                    pub_vel_->publish(msg);
                else
                    pub_dist_->publish(msg);
                return true;
            });

        lio_->setFunc(cloud_pub_func);
        lio_->setFunc(pose_pub_func);
        lio_->setFunc(data_pub_func);
    }

    void livoxCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
    {
        std::vector<std::vector<point3D>> cloud_vec;
        std::vector<double> t_out;
        zjloc::common::Timer::Evaluate([&]()
                                       { convert_->Process(msg, cloud_vec, t_out); },
                                       "laser convert");

        double header_time = toSec(msg->header.stamp);
        for (size_t i = 0; i < cloud_vec.size(); i++)
        {
            auto &cloud_out = cloud_vec[i];
            double sample_size = lio_->getIndex() < 20 ? 0.01 : 0.01;
            std::mt19937_64 g;
            zjloc::common::Timer::Evaluate([&]()
                                           { std::shuffle(cloud_out.begin(), cloud_out.end(), g);
                                             subSampleFrame(cloud_out, sample_size);
                                             std::shuffle(cloud_out.begin(), cloud_out.end(), g); },
                                           "laser ds");

            lio_->pushData(cloud_out, std::make_pair(header_time + t_out[i] - t_out[0], t_out[0]));
        }
    }

    void standardPclCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        std::vector<std::vector<point3D>> cloud_vec;
        std::vector<double> t_out;
        zjloc::common::Timer::Evaluate([&]()
                                       { convert_->Process(msg, cloud_vec, t_out); },
                                       "laser convert");

        double header_time = toSec(msg->header.stamp);
        for (size_t i = 0; i < cloud_vec.size(); i++)
        {
            auto &cloud_out = cloud_vec[i];
            double sample_size = lio_->getIndex() < 30 ? 0.02 : 0.1;
            std::mt19937_64 g;
            zjloc::common::Timer::Evaluate([&]()
                                           { std::shuffle(cloud_out.begin(), cloud_out.end(), g);
                                             sub_sample_frame(cloud_out, sample_size);
                                             std::shuffle(cloud_out.begin(), cloud_out.end(), g); },
                                           "laser ds");

            lio_->pushData(cloud_out, std::make_pair(header_time + t_out[i] - t_out[0], t_out[0]));
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        double timestamp = toSec(msg->header.stamp);
        IMUPtr imu = std::make_shared<zjloc::IMU>(
            timestamp,
            Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
            Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * gnorm_);
        lio_->pushData(imu);

        // Republish IMU with scaled acceleration
        sensor_msgs::msg::Imu imu_repub = *msg;
        imu_repub.linear_acceleration.x *= gnorm_;
        imu_repub.linear_acceleration.y *= gnorm_;
        imu_repub.linear_acceleration.z *= gnorm_;
        pub_imu_repub_->publish(imu_repub);
    }

    void saveMapCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (save_in_progress_.load())
        {
            response->success = false;
            response->message = "Previous save still in progress, check /save_map_status";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // Copy map under short lock (map is already downsampled during runtime)
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_copy;
        size_t point_count;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (global_map_->empty())
            {
                response->success = false;
                response->message = "Global map is empty, nothing to save.";
                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                return;
            }
            map_copy.reset(new pcl::PointCloud<pcl::PointXYZI>(*global_map_));
            point_count = map_copy->size();
        }
        // No redundant voxel downsampling here - map is continuously downsampled
        // during runtime every 5 frames, so it's already export-ready.

        save_in_progress_ = true;

        // Join previous save thread if it finished
        if (save_thread_.joinable())
            save_thread_.join();

        // Async PCD write in background thread
        save_thread_ = std::thread([this, map_copy, point_count]()
                                   {
            std::string cmd = "mkdir -p " + map_save_path_;
            system(cmd.c_str());

            std::string pcd_path = map_save_path_ + "global_map.pcd";
            auto start = std::chrono::steady_clock::now();
            int ret = pcl::io::savePCDFileBinary(pcd_path, *map_copy);
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() - start)
                                  .count();

            if (ret == 0)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Map saved to %s (%zu points, %.1fs)",
                            pcd_path.c_str(), point_count, elapsed_ms / 1000.0);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to save PCD to %s", pcd_path.c_str());
            }
            save_in_progress_ = false; });

        response->success = true;
        response->message = "Map save started asynchronously (" +
                            std::to_string(point_count) + " points, no downsampling needed). "
                                                          "Check /save_map_status for completion.";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void saveMapStatusCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (save_in_progress_.load())
        {
            response->success = false;
            response->message = "Save is still in progress...";
        }
        else
        {
            response->success = true;
            response->message = "No save in progress. Last save completed.";
        }
    }

    void snapshotTimerCallback()
    {
        // Skip if a snapshot write is already running
        if (snapshot_in_progress_.load())
            return;

        // Copy map under short lock
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_copy;
        size_t point_count;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (!global_map_ || global_map_->empty())
                return;
            map_copy.reset(new pcl::PointCloud<pcl::PointXYZI>(*global_map_));
            point_count = map_copy->size();
        }

        snapshot_in_progress_ = true;

        // Join previous snapshot thread if it finished
        if (snapshot_thread_.joinable())
            snapshot_thread_.join();

        // Write snapshot in background thread
        snapshot_thread_ = std::thread([this, map_copy, point_count]()
                                       {
            std::string cmd = "mkdir -p " + map_save_path_;
            system(cmd.c_str());

            std::string snapshot_path = map_save_path_ + "global_map_snapshot.pcd";
            auto start = std::chrono::steady_clock::now();
            int ret = pcl::io::savePCDFileBinary(snapshot_path, *map_copy);
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() - start)
                                  .count();

            if (ret == 0)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Snapshot saved: %s (%zu points, %.1fs)",
                            snapshot_path.c_str(), point_count, elapsed_ms / 1000.0);
            }
            snapshot_in_progress_ = false; });
    }

    // Check distance traveled and save map segment if threshold exceeded
    void checkAndSaveSegment(const Eigen::Vector3d &current_pos)
    {
        // Initialize segment position on first call
        if (!segment_initialized_)
        {
            last_segment_position_ = current_pos;
            segment_initialized_ = true;
            return;
        }

        double distance = (current_pos - last_segment_position_).norm();
        size_t current_points = 0;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            current_points = global_map_ ? global_map_->size() : 0;
        }

        // Trigger segment save if distance threshold OR point count threshold exceeded
        bool distance_trigger = distance >= segment_distance_threshold_;
        bool memory_trigger = current_points >= max_map_points_;

        if (distance_trigger || memory_trigger)
        {
            saveMapSegmentAndClear(current_pos, distance_trigger ? "distance" : "memory");
            last_segment_position_ = current_pos;
        }
    }

    // Save current map segment to disk and clear memory
    void saveMapSegmentAndClear(const Eigen::Vector3d &current_pos, const std::string &trigger_reason)
    {
        // Skip if segment save is already in progress
        if (segment_save_in_progress_.load())
            return;

        pcl::PointCloud<pcl::PointXYZI>::Ptr segment_cloud;
        size_t point_count;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (!global_map_ || global_map_->empty())
                return;

            // Move the entire map to segment cloud
            segment_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
            segment_cloud.swap(global_map_);
            global_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            point_count = segment_cloud->size();
        }

        segment_save_in_progress_ = true;
        int current_segment = segment_index_++;

        // Join previous segment save thread if finished
        if (segment_save_thread_.joinable())
            segment_save_thread_.join();

        // Save segment in background thread
        segment_save_thread_ = std::thread([this, segment_cloud, point_count, current_segment, current_pos, trigger_reason]()
                                           {
            std::string cmd = "mkdir -p " + map_save_path_;
            system(cmd.c_str());

            // Generate segment filename with index
            std::string segment_path = map_save_path_ + "segment_" + 
                                       std::to_string(current_segment) + ".pcd";
            
            auto start = std::chrono::steady_clock::now();
            int ret = pcl::io::savePCDFileBinary(segment_path, *segment_cloud);
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() - start)
                                  .count();

            if (ret == 0)
            {
                RCLCPP_INFO(this->get_logger(),
                            "[Memory Management] Segment %d saved: %s (%zu points, %.1fs, trigger: %s, pos: %.1f,%.1f,%.1f)",
                            current_segment, segment_path.c_str(), point_count, elapsed_ms / 1000.0,
                            trigger_reason.c_str(), current_pos.x(), current_pos.y(), current_pos.z());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to save segment %d to %s", 
                             current_segment, segment_path.c_str());
            }
            segment_save_in_progress_ = false; });
    }

    // Core algorithm
    zjloc::lidarodom_m *lio_ = nullptr;
    zjloc::CloudConvert2 *convert_ = nullptr;
    double gnorm_ = 1.0;
    std::thread process_thread_;

    // Path accumulator
    nav_msgs::msg::Path laser_odo_path_;

    // TF
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_vel_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_dist_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_repub_;

    // Map saving
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;
    std::mutex map_mutex_;
    std::string map_save_path_;
    float map_voxel_size_ = 0.1f;  // voxel size for downsampling (configurable)
    int global_map_frame_count_;
    int global_map_downsample_interval_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_status_srv_;

    // Async save
    std::thread save_thread_;
    std::atomic<bool> save_in_progress_{false};

    // Streaming snapshot
    rclcpp::TimerBase::SharedPtr snapshot_timer_;
    std::thread snapshot_thread_;
    std::atomic<bool> snapshot_in_progress_{false};

    // Segmented map saving for memory management
    double segment_distance_threshold_;  // meters
    size_t max_map_points_;              // hard limit on points in memory
    int segment_index_;
    Eigen::Vector3d last_segment_position_;
    bool segment_initialized_;
    std::thread segment_save_thread_;
    std::atomic<bool> segment_save_in_progress_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdaptiveLioNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
