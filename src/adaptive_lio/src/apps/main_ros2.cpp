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
#include <unordered_map>
#include <fstream>

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

// Runtime package path resolution (replaces compile-time ROOT_DIR)
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include "common/utility.h"
#include "preprocess/cloud_convert/cloud_convert2.h"
#include "lio/lidarodom.h"
#include "tools/saveTrajectory.hpp"

// Resolve the package source directory at runtime.
// With --symlink-install, individual files inside config/ are symlinks to the source tree,
// so we resolve one to find the actual source root.
static std::string get_package_root_dir()
{
    std::string share_dir = ament_index_cpp::get_package_share_directory("adaptive_lio");
    std::filesystem::path config_dir = std::filesystem::path(share_dir) / "config";
    try
    {
        for (auto const& entry : std::filesystem::directory_iterator(config_dir))
        {
            if (std::filesystem::is_symlink(entry.path()))
            {
                // e.g. install/.../config/mapping_m.yaml -> <source>/config/mapping_m.yaml
                std::filesystem::path resolved = std::filesystem::canonical(entry.path());
                // resolved = <source>/config/mapping_m.yaml  =>  parent.parent = <source>/
                return resolved.parent_path().parent_path().string() + "/";
            }
        }
    }
    catch (...)
    {
    }
    // Fallback: use share directory (non-symlink install)
    return share_dir + "/";
}

#define DEBUG_FILE_DIR(name) (get_package_root_dir() + "log/" + (name))

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
    // Dense point: lightweight struct for full-resolution accumulation
    struct DensePoint { float x, y, z, intensity; };

    // Hash-based voxel downsample: uses unordered_map to avoid int32 overflow
    // that occurs with PCL VoxelGrid on large-extent maps.
    static bool safeVoxelDownsample(
        pcl::PointCloud<pcl::PointXYZI>::Ptr input,
        pcl::PointCloud<pcl::PointXYZI>::Ptr output,
        float leaf_size)
    {
        if (!input || input->empty())
            return false;

        output->clear();
        output->reserve(input->size() / 4);  // rough estimate

        // Use hash map with voxel key (supports arbitrary coordinate range)
        struct VoxelKey
        {
            int64_t x, y, z;
            bool operator==(const VoxelKey &other) const
            {
                return x == other.x && y == other.y && z == other.z;
            }
        };
        struct VoxelKeyHash
        {
            size_t operator()(const VoxelKey &k) const
            {
                // FNV-1a inspired hash
                size_t h = 14695981039346656037ULL;
                h ^= std::hash<int64_t>()(k.x);
                h *= 1099511628211ULL;
                h ^= std::hash<int64_t>()(k.y);
                h *= 1099511628211ULL;
                h ^= std::hash<int64_t>()(k.z);
                return h;
            }
        };

        float inv_leaf = 1.0f / leaf_size;
        std::unordered_map<VoxelKey, pcl::PointXYZI, VoxelKeyHash> voxel_map;
        voxel_map.reserve(input->size() / 4);

        for (const auto &pt : input->points)
        {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                continue;

            VoxelKey key;
            key.x = static_cast<int64_t>(std::floor(pt.x * inv_leaf));
            key.y = static_cast<int64_t>(std::floor(pt.y * inv_leaf));
            key.z = static_cast<int64_t>(std::floor(pt.z * inv_leaf));

            // Keep first point in each voxel (simple but fast)
            if (voxel_map.find(key) == voxel_map.end())
            {
                voxel_map[key] = pt;
            }
        }

        // Copy results to output
        for (const auto &pair : voxel_map)
        {
            output->points.push_back(pair.second);
        }
        output->width = output->size();
        output->height = 1;
        output->is_dense = true;

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
        map_save_path_ = get_package_root_dir() + "map/";
        global_map_frame_count_ = 0;
        global_map_downsample_interval_ = 5;  // downsample every 5 frames (was 50)

        // Dense map mode: full-resolution PCD export (no downsampling)
        // SLAM tracking map stays lightweight; only the save/export branch changes.
        dense_map_mode_ = false;

        // Segmented map saving for memory management
        segment_distance_threshold_ = 50.0;  // Save and clear map every 50 meters
        segment_index_ = 0;
        last_segment_position_ = Eigen::Vector3d::Zero();
        segment_initialized_ = false;
        has_saved_any_data_ = false;  // Track if any data was saved
        last_merged_segment_index_ = -1;  // No manual merge done yet
        max_map_points_ = 2000000;  // Hard limit: 2 million points (~64MB)

        // config file - support environment variable override
        std::string config_file;
        const char* env_config = std::getenv("ADAPTIVE_LIO_CONFIG");
        if (env_config != nullptr && std::strlen(env_config) > 0)
        {
            config_file = env_config;
            RCLCPP_INFO(this->get_logger(), "Using config from env: %s", config_file.c_str());
        }
        else
        {
            config_file = get_package_root_dir() + "config/mapping_m.yaml";
            RCLCPP_INFO(this->get_logger(), "Using default config: %s", config_file.c_str());
        }

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

        // Sliding window save map config
        near_buffer_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        if (yaml["common"]["enable_save_sliding_window"])
            enable_save_sliding_window_ = yaml["common"]["enable_save_sliding_window"].as<bool>();
        if (yaml["common"]["save_near_range"])
            save_near_range_ = yaml["common"]["save_near_range"].as<double>();
        if (yaml["common"]["archive_trigger_distance"])
            archive_trigger_distance_ = yaml["common"]["archive_trigger_distance"].as<double>();
        if (yaml["common"]["max_near_buffer_points"])
            max_near_buffer_points_ = yaml["common"]["max_near_buffer_points"].as<size_t>();
        if (enable_save_sliding_window_)
            RCLCPP_INFO(this->get_logger(), "Save sliding window: near_range=%.1f, archive_dist=%.1f, max_buffer=%zu",
                        save_near_range_, archive_trigger_distance_, max_near_buffer_points_);

        // Dense map mode config (YAML + env var override)
        if (yaml["common"]["dense_map_mode"])
            dense_map_mode_ = yaml["common"]["dense_map_mode"].as<bool>();
        if (yaml["common"]["dense_save_segment_points"])
            dense_save_segment_points_ = yaml["common"]["dense_save_segment_points"].as<size_t>();
        if (yaml["common"]["dense_voxel_size"])
            dense_voxel_size_ = yaml["common"]["dense_voxel_size"].as<float>();
        // Environment variable override: ADAPTIVE_LIO_DENSE_MAP=1 forces dense mode
        const char* dense_env = std::getenv("ADAPTIVE_LIO_DENSE_MAP");
        if (dense_env != nullptr && std::string(dense_env) == "1")
            dense_map_mode_ = true;
        const char* voxel_env = std::getenv("ADAPTIVE_LIO_DENSE_VOXEL");
        if (voxel_env != nullptr)
            dense_voxel_size_ = std::stof(std::string(voxel_env));
        if (dense_map_mode_)
        {
            dense_buffer_.reserve(dense_save_segment_points_);
            RCLCPP_INFO(this->get_logger(),
                        "Dense map mode ENABLED: segment=%zu pts, merge_voxel=%.3f %s",
                        dense_save_segment_points_, dense_voxel_size_,
                        dense_voxel_size_ > 0 ? "(downsample on merge)" : "(raw, no downsample)");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Dense map mode DISABLED (sparse mode with downsampling)");
        }

        // Open trajectory file AFTER map_save_path_ is finalized
        {
            std::string mkdir_cmd = "mkdir -p " + map_save_path_;
            system(mkdir_cmd.c_str());
            std::string traj_path = map_save_path_ + "trajectory.txt";
            traj_fout_.open(traj_path, std::ios::out);
            if (traj_fout_.is_open())
            {
                traj_fout_ << "# TUM format: timestamp tx ty tz qx qy qz qw" << std::endl;
                RCLCPP_INFO(this->get_logger(), "Trajectory file: %s", traj_path.c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to open trajectory file: %s", traj_path.c_str());
            }
        }

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

        // Start processing thread
        process_thread_ = std::thread(&zjloc::lidarodom_m::run, lio_);
    }

    ~AdaptiveLioNode()
    {
        // Wait for any background save/segment threads to finish
        if (save_thread_.joinable())
            save_thread_.join();
        if (segment_save_thread_.joinable())
            segment_save_thread_.join();
        if (dense_save_thread_.joinable())
            dense_save_thread_.join();

        if (dense_map_mode_)
        {
            // Dense mode: flush remaining buffer as final segment, then stream-merge
            if (!dense_buffer_.empty())
                saveDenseSegmentSync();

            if (dense_segment_index_ > 0)
                mergeDenseSegments();
            else
                RCLCPP_INFO(this->get_logger(), "[Dense] No dense segments to merge");
        }
        else
        {
            // Sparse mode: original map saving logic
            if (enable_save_sliding_window_ && near_buffer_ && !near_buffer_->empty())
            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
                if (safeVoxelDownsample(near_buffer_, filtered, map_voxel_size_))
                    *global_map_ += *filtered;
                else
                    *global_map_ += *near_buffer_;
                near_buffer_->clear();
                RCLCPP_INFO(this->get_logger(), "[Memory Management] Flushed near_buffer_ to global_map_ before final save");
            }

            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                if (global_map_ && !global_map_->empty())
                {
                    std::string cmd = "mkdir -p " + map_save_path_;
                    system(cmd.c_str());

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
                        has_saved_any_data_ = true;
                    }
                }
            }

            if (has_saved_any_data_)
            {
                if (last_merged_segment_index_ >= 0 && segment_index_ == last_merged_segment_index_)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "[Memory Management] Skipping redundant merge (already merged up to segment_%d)",
                                last_merged_segment_index_);
                }
                else
                {
                    mergeAllSegments();
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "[Memory Management] No data was processed, skipping merge");
            }
        }

        // Signal processing thread to stop and discard queued data
        if (lio_)
            lio_->requestStop();
        if (process_thread_.joinable())
            process_thread_.join();

        zjloc::common::Timer::PrintAll();
        {
            std::string log_dir = get_package_root_dir() + "log/";
            std::string mkdir_cmd = "mkdir -p " + log_dir;
            system(mkdir_cmd.c_str());
        }
        zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt"));

        // Close trajectory file
        if (traj_fout_.is_open())
        {
            traj_fout_.close();
            RCLCPP_INFO(this->get_logger(), "Trajectory saved to: %strajectory.txt", map_save_path_.c_str());
        }

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
                            "\n\n"
                            "============================================================\n"
                            "  MAP MERGE COMPLETE\n"
                            "  Output: %s\n"
                            "  Points: %zu  |  Segments: %d  |  Time: %.1fs\n"
                            "============================================================\n",
                            final_path.c_str(), filtered->size(), loaded_count, elapsed_ms / 1000.0);
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

                    if (dense_map_mode_)
                    {
                        // Dense mode: accumulate raw points without any downsampling
                        for (const auto &pt : cloud->points)
                        {
                            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                                continue;
                            dense_buffer_.push_back({pt.x, pt.y, pt.z, pt.intensity});
                        }
                        dense_total_points_ += cloud->size();

                        // Trigger async segment save when buffer is full
                        if (dense_buffer_.size() >= dense_save_segment_points_)
                            saveDenseSegment();
                    }
                    else
                    {
                        std::lock_guard<std::mutex> lock(map_mutex_);

                        if (enable_save_sliding_window_)
                        {
                            // Sliding window: accumulate into near_buffer_ without downsampling
                            *near_buffer_ += *cloud;

                            // Memory protection: lightweight downsample if buffer exceeds limit
                            if (near_buffer_->size() > max_near_buffer_points_)
                            {
                                pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
                                if (safeVoxelDownsample(near_buffer_, filtered, 0.02f))
                                    near_buffer_.swap(filtered);
                            }
                        }
                        else
                        {
                            // Original logic: accumulate and periodically downsample
                            *global_map_ += *cloud;
                            global_map_frame_count_++;
                            if (global_map_frame_count_ % global_map_downsample_interval_ == 0)
                            {
                                pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
                                if (safeVoxelDownsample(global_map_, filtered, map_voxel_size_))
                                    global_map_.swap(filtered);
                            }
                        }
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
                    // Sliding window: keep only recent poses to bound memory and serialization cost
                    const size_t kMaxPathPoses = 5000;
                    if (laser_odo_path_.poses.size() > kMaxPathPoses)
                        laser_odo_path_.poses.erase(laser_odo_path_.poses.begin());
                    laser_odo_path_.header.frame_id = "map";
                    pub_path_->publish(laser_odo_path_);

                    // Save trajectory in TUM format
                    if (traj_fout_.is_open())
                    {
                        std::string stamp_str = std::to_string(stamp);
                        zjloc::saveTrajectoryTUMformat(traj_fout_, stamp_str,
                                                       current_pos.x(), current_pos.y(), current_pos.z(),
                                                       q_current.x(), q_current.y(), q_current.z(), q_current.w());
                    }

                    // Sparse mode: archive far-away points and check segment save
                    if (!dense_map_mode_)
                    {
                        archiveNearBuffer(current_pos);
                        checkAndSaveSegment(current_pos);
                    }
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

        // Wait for any pending segment save to finish
        if (segment_save_thread_.joinable())
            segment_save_thread_.join();

        // Save current in-memory global_map_ as a new segment (if non-empty)
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (global_map_ && !global_map_->empty())
            {
                std::string cmd = "mkdir -p " + map_save_path_;
                system(cmd.c_str());

                std::string segment_path = map_save_path_ + "segment_" +
                                           std::to_string(segment_index_) + ".pcd";
                if (pcl::io::savePCDFileBinary(segment_path, *global_map_) == 0)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "[SaveMap] Flushed current buffer as segment_%d (%zu points)",
                                segment_index_, global_map_->size());
                }
                // Do NOT increment segment_index_ or clear global_map_ here,
                // so normal processing can continue accumulating into it.
                // mergeAllSegments uses 0..segment_index_ inclusive, which now
                // includes this just-saved buffer.
            }
        }

        save_in_progress_ = true;

        // Join previous save thread if it finished
        if (save_thread_.joinable())
            save_thread_.join();

        // Merge all segments + current buffer into global_map.pcd in background
        save_thread_ = std::thread([this]()
                                   {
            RCLCPP_INFO(this->get_logger(), "[SaveMap] Merging %d+1 segments into global_map.pcd...",
                        segment_index_);
            mergeAllSegments();
            last_merged_segment_index_ = segment_index_;  // Record that we merged up to this segment
            save_in_progress_ = false; });

        response->success = true;
        response->message = "Map save started: merging " +
                            std::to_string(segment_index_ + 1) +
                            " segments. Check /save_map_status for completion.";
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
                has_saved_any_data_ = true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to save segment %d to %s", 
                             current_segment, segment_path.c_str());
            }
            segment_save_in_progress_ = false; });
    }

    // ---- Dense map mode methods ----

    // Async: swap buffer and save segment in background thread
    void saveDenseSegment()
    {
        if (dense_save_in_progress_.load() || dense_buffer_.empty())
            return;

        // Move buffer out; re-reserve for next batch
        std::vector<DensePoint> save_data;
        save_data.swap(dense_buffer_);
        dense_buffer_.reserve(dense_save_segment_points_);

        dense_save_in_progress_ = true;
        int current_segment = dense_segment_index_++;

        if (dense_save_thread_.joinable())
            dense_save_thread_.join();

        dense_save_thread_ = std::thread([this, save_data = std::move(save_data), current_segment]()
        {
            writeDenseSegmentFile(save_data, current_segment);
            dense_save_in_progress_ = false;
        });
    }

    // Sync: save remaining buffer (called from destructor)
    void saveDenseSegmentSync()
    {
        if (dense_buffer_.empty())
            return;

        if (dense_save_thread_.joinable())
            dense_save_thread_.join();

        int current_segment = dense_segment_index_++;
        writeDenseSegmentFile(dense_buffer_, current_segment);
        dense_buffer_.clear();
    }

    // Write a dense segment to PCD file
    void writeDenseSegmentFile(const std::vector<DensePoint> &points, int segment_idx)
    {
        std::string cmd = "mkdir -p " + map_save_path_;
        system(cmd.c_str());

        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.resize(points.size());
        for (size_t i = 0; i < points.size(); i++)
        {
            cloud.points[i].x = points[i].x;
            cloud.points[i].y = points[i].y;
            cloud.points[i].z = points[i].z;
            cloud.points[i].intensity = points[i].intensity;
        }
        cloud.width = cloud.size();
        cloud.height = 1;
        cloud.is_dense = true;

        std::string path = map_save_path_ + "dense_segment_" + std::to_string(segment_idx) + ".pcd";
        auto start = std::chrono::steady_clock::now();
        if (pcl::io::savePCDFileBinary(path, cloud) == 0)
        {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count();
            RCLCPP_INFO(this->get_logger(),
                "[Dense] Segment %d saved: %s (%zu points, %.1fs)",
                segment_idx, path.c_str(), cloud.size(), elapsed / 1000.0);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "[Dense] Failed to save segment %d", segment_idx);
        }
    }

    // Stream-based merge: read binary data segment by segment without full-memory load
    void mergeDenseSegments()
    {
        if (dense_segment_index_ <= 0)
        {
            RCLCPP_INFO(this->get_logger(), "[Dense] No segments to merge");
            return;
        }

        const bool do_downsample = (dense_voxel_size_ > 0.0f);
        RCLCPP_INFO(this->get_logger(), "[Dense] Merging %d segments %s...",
                    dense_segment_index_,
                    do_downsample ? ("(voxel=" + std::to_string(dense_voxel_size_) + "m)").c_str() : "(raw)");

        std::string final_path = map_save_path_ + "global_map.pcd";
        auto merge_start = std::chrono::steady_clock::now();

        if (!do_downsample)
        {
            // ---- Fast path: stream raw binary, no PCL loading ----
            size_t total_points = 0;
            int valid_segments = 0;
            for (int i = 0; i < dense_segment_index_; i++)
            {
                std::string path = map_save_path_ + "dense_segment_" + std::to_string(i) + ".pcd";
                pcl::PCLPointCloud2 cloud2;
                Eigen::Vector4f origin;
                Eigen::Quaternionf orientation;
                int version, type;
                unsigned int data_idx;
                pcl::PCDReader reader;
                if (reader.readHeader(path, cloud2, origin, orientation, version, type, data_idx) == 0)
                {
                    total_points += cloud2.width * cloud2.height;
                    valid_segments++;
                }
            }
            if (total_points == 0) { RCLCPP_WARN(this->get_logger(), "[Dense] No points"); return; }

            std::ofstream ofs(final_path, std::ios::binary);
            if (!ofs.is_open()) { RCLCPP_ERROR(this->get_logger(), "[Dense] Cannot open %s", final_path.c_str()); return; }

            ofs << "# .PCD v0.7 - Point Cloud Data file format\n"
                << "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
                << "TYPE F F F F\nCOUNT 1 1 1 1\n"
                << "WIDTH " << total_points << "\nHEIGHT 1\n"
                << "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << total_points << "\nDATA binary\n";

            for (int i = 0; i < dense_segment_index_; i++)
            {
                std::string path = map_save_path_ + "dense_segment_" + std::to_string(i) + ".pcd";
                std::ifstream ifs(path, std::ios::binary);
                if (!ifs.is_open()) continue;
                std::string line;
                while (std::getline(ifs, line))
                    if (line.find("DATA binary") != std::string::npos) break;
                const size_t CHUNK = 1024 * 1024;
                std::vector<char> buf(CHUNK);
                while (ifs.good()) { ifs.read(buf.data(), CHUNK); auto n = ifs.gcount(); if (n > 0) ofs.write(buf.data(), n); }
                RCLCPP_INFO(this->get_logger(), "[Dense] Streamed segment %d", i);
            }
            ofs.close();

            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - merge_start).count();
            RCLCPP_INFO(this->get_logger(),
                "\n============================================================\n"
                "  DENSE MAP MERGE COMPLETE (RAW, NO DOWNSAMPLE)\n"
                "  Output: %s\n"
                "  Points: %zu  |  Segments: %d  |  Time: %.1fs\n"
                "============================================================",
                final_path.c_str(), total_points, valid_segments, elapsed / 1000.0);
        }
        else
        {
            // ---- Downsample path: load each segment, voxel filter, write to temp, then assemble ----
            std::string tmp_path = final_path + ".tmp";
            std::ofstream tmp(tmp_path, std::ios::binary);
            if (!tmp.is_open()) { RCLCPP_ERROR(this->get_logger(), "[Dense] Cannot open tmp"); return; }

            size_t total_points = 0;
            int valid_segments = 0;
            pcl::PCDReader reader;

            for (int i = 0; i < dense_segment_index_; i++)
            {
                std::string path = map_save_path_ + "dense_segment_" + std::to_string(i) + ".pcd";
                pcl::PointCloud<pcl::PointXYZI>::Ptr seg(new pcl::PointCloud<pcl::PointXYZI>());
                if (reader.read(path, *seg) != 0 || seg->empty())
                    continue;

                size_t before = seg->size();
                pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
                if (safeVoxelDownsample(seg, filtered, dense_voxel_size_))
                    seg = filtered;

                // Write raw binary: each point = 16 bytes (x,y,z,intensity as float)
                for (const auto &pt : seg->points)
                {
                    tmp.write(reinterpret_cast<const char*>(&pt.x), 4);
                    tmp.write(reinterpret_cast<const char*>(&pt.y), 4);
                    tmp.write(reinterpret_cast<const char*>(&pt.z), 4);
                    tmp.write(reinterpret_cast<const char*>(&pt.intensity), 4);
                }

                total_points += seg->size();
                valid_segments++;
                RCLCPP_INFO(this->get_logger(), "[Dense] Segment %d: %zu -> %zu pts (voxel=%.3f)",
                            i, before, seg->size(), dense_voxel_size_);
            }
            tmp.close();

            if (total_points == 0)
            {
                std::remove(tmp_path.c_str());
                RCLCPP_WARN(this->get_logger(), "[Dense] No points after downsample");
                return;
            }

            // Assemble final PCD: header + tmp binary data
            std::ofstream ofs(final_path, std::ios::binary);
            ofs << "# .PCD v0.7 - Point Cloud Data file format\n"
                << "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
                << "TYPE F F F F\nCOUNT 1 1 1 1\n"
                << "WIDTH " << total_points << "\nHEIGHT 1\n"
                << "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << total_points << "\nDATA binary\n";

            std::ifstream tmp_in(tmp_path, std::ios::binary);
            const size_t CHUNK = 1024 * 1024;
            std::vector<char> buf(CHUNK);
            while (tmp_in.good()) { tmp_in.read(buf.data(), CHUNK); auto n = tmp_in.gcount(); if (n > 0) ofs.write(buf.data(), n); }
            tmp_in.close();
            ofs.close();
            std::remove(tmp_path.c_str());

            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - merge_start).count();
            RCLCPP_INFO(this->get_logger(),
                "\n============================================================\n"
                "  DENSE MAP MERGE COMPLETE (VOXEL=%.3f)\n"
                "  Output: %s\n"
                "  Points: %zu  |  Segments: %d  |  Time: %.1fs\n"
                "============================================================",
                dense_voxel_size_, final_path.c_str(), total_points, valid_segments, elapsed / 1000.0);
        }
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

    // Dense map mode: full-resolution PCD export (no downsampling)
    bool dense_map_mode_ = false;
    float dense_voxel_size_ = 0.0f;   // 0=raw, >0=downsample on merge
    size_t dense_save_segment_points_ = 2000000;  // 2M points per segment (~32MB)
    std::vector<DensePoint> dense_buffer_;
    int dense_segment_index_ = 0;
    size_t dense_total_points_ = 0;
    std::thread dense_save_thread_;
    std::atomic<bool> dense_save_in_progress_{false};

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

    // Segmented map saving for memory management
    double segment_distance_threshold_;  // meters
    size_t max_map_points_;              // hard limit on points in memory
    int segment_index_;
    Eigen::Vector3d last_segment_position_;
    bool segment_initialized_;
    bool has_saved_any_data_;  // Track if any data was actually processed
    int last_merged_segment_index_;  // Track last manually merged segment to avoid redundant merge
    std::thread segment_save_thread_;
    std::atomic<bool> segment_save_in_progress_{false};

    // Trajectory saving
    std::fstream traj_fout_;

    // Sliding window for saved map - preserves near-field detail
    bool enable_save_sliding_window_ = false;
    double save_near_range_ = 30.0;
    double archive_trigger_distance_ = 10.0;
    size_t max_near_buffer_points_ = 3000000;
    pcl::PointCloud<pcl::PointXYZI>::Ptr near_buffer_;
    Eigen::Vector3d last_archive_position_;
    bool archive_initialized_ = false;

    // Archive near_buffer_: move far-away points into global_map_ with downsampling
    void archiveNearBuffer(const Eigen::Vector3d &current_pos)
    {
        if (!enable_save_sliding_window_)
            return;

        if (!archive_initialized_)
        {
            last_archive_position_ = current_pos;
            archive_initialized_ = true;
            return;
        }

        if ((current_pos - last_archive_position_).norm() < archive_trigger_distance_)
            return;

        last_archive_position_ = current_pos;

        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!near_buffer_ || near_buffer_->empty())
            return;

        double sq_range = save_near_range_ * save_near_range_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr keep(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr archive(new pcl::PointCloud<pcl::PointXYZI>());

        keep->reserve(near_buffer_->size());
        archive->reserve(near_buffer_->size() / 4);

        for (const auto &pt : near_buffer_->points)
        {
            double dx = pt.x - current_pos.x();
            double dy = pt.y - current_pos.y();
            double dz = pt.z - current_pos.z();
            if (dx * dx + dy * dy + dz * dz <= sq_range)
                keep->points.push_back(pt);
            else
                archive->points.push_back(pt);
        }

        if (!archive->empty())
        {
            archive->width = archive->size();
            archive->height = 1;
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
            if (safeVoxelDownsample(archive, filtered, map_voxel_size_))
                *global_map_ += *filtered;
            else
                *global_map_ += *archive;
        }

        keep->width = keep->size();
        keep->height = 1;
        near_buffer_.swap(keep);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdaptiveLioNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
