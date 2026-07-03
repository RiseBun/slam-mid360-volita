#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct VoxelKey
{
  int64_t x{};
  int64_t y{};
  int64_t z{};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  size_t operator()(const VoxelKey & key) const
  {
    size_t h = 14695981039346656037ULL;
    h ^= std::hash<int64_t>{}(key.x);
    h *= 1099511628211ULL;
    h ^= std::hash<int64_t>{}(key.y);
    h *= 1099511628211ULL;
    h ^= std::hash<int64_t>{}(key.z);
    h *= 1099511628211ULL;
    return h;
  }
};

bool isFiniteTransform(const Eigen::Isometry3d & transform)
{
  return transform.matrix().array().isFinite().all();
}

double rotationAngleRad(const Eigen::Isometry3d & lhs, const Eigen::Isometry3d & rhs)
{
  Eigen::Matrix3d delta = lhs.rotation().transpose() * rhs.rotation();
  Eigen::AngleAxisd angle_axis(delta);
  return std::abs(angle_axis.angle());
}

std::string yesNo(bool value)
{
  return value ? "yes" : "no";
}
}  // namespace

class PoseGuardMapperNode : public rclcpp::Node
{
public:
  PoseGuardMapperNode() : Node("pose_guard_mapper")
  {
    loadParameters();

    lidar_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      lidar_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PoseGuardMapperNode::lidarOdomCallback, this, std::placeholders::_1));
    camera_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      camera_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PoseGuardMapperNode::cameraOdomCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lidar_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PoseGuardMapperNode::cloudCallback, this, std::placeholders::_1));

    trusted_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(trusted_odom_topic_, 50);
    guarded_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(guarded_map_topic_, 2);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    save_map_srv_ = create_service<std_srvs::srv::Trigger>(
      save_map_service_,
      std::bind(
        &PoseGuardMapperNode::saveMapCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "pose_guard_mapper started");
    RCLCPP_INFO(get_logger(), "lidar_odom=%s camera_odom=%s cloud=%s",
      lidar_odom_topic_.c_str(), camera_odom_topic_.c_str(), lidar_cloud_topic_.c_str());
  }

private:
  enum class SourceMode
  {
    Lidar,
    Camera
  };

  struct PoseSample
  {
    rclcpp::Time stamp;
    rclcpp::Time received;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    bool stamp_ok = false;
    bool pose_ok = false;
  };

  struct TrustedPose
  {
    rclcpp::Time stamp;
    rclcpp::Time received;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  };

  void loadParameters()
  {
    lidar_odom_topic_ = declare_parameter<std::string>("lidar_odom_topic", "/odom");
    camera_odom_topic_ = declare_parameter<std::string>("camera_odom_topic", "/S1/vio_odom");
    lidar_cloud_topic_ = declare_parameter<std::string>("lidar_cloud_topic", "/livox/lidar");
    trusted_odom_topic_ = declare_parameter<std::string>("trusted_odom_topic", "/trusted_odom");
    guarded_map_topic_ = declare_parameter<std::string>("guarded_map_topic", "/guarded_map");
    status_topic_ = declare_parameter<std::string>("status_topic", "/pose_guard/status");
    save_map_service_ = declare_parameter<std::string>("save_map_service", "/pose_guard/save_map");
    save_map_path_ = declare_parameter<std::string>(
      "save_map_path", "/home/li/slam-mid360-volita/map/guarded_map.pcd");

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    position_threshold_m_ = declare_parameter<double>("position_threshold_m", 1.0);
    orientation_threshold_rad_ =
      declare_parameter<double>("orientation_threshold_deg", 20.0) * kPi / 180.0;
    recovery_position_threshold_m_ =
      declare_parameter<double>("recovery_position_threshold_m", 0.45);
    recovery_orientation_threshold_rad_ =
      declare_parameter<double>("recovery_orientation_threshold_deg", 8.0) * kPi / 180.0;
    reject_count_threshold_ = declare_parameter<int>("reject_count_threshold", 3);
    recover_count_threshold_ = declare_parameter<int>("recover_count_threshold", 20);

    max_time_diff_sec_ = declare_parameter<double>("max_time_diff_sec", 0.10);
    require_header_time_near_now_ =
      declare_parameter<bool>("require_header_time_near_now", true);
    max_wall_stamp_skew_sec_ = declare_parameter<double>("max_wall_stamp_skew_sec", 5.0);
    cloud_pose_max_age_sec_ = declare_parameter<double>("cloud_pose_max_age_sec", 1.0);

    max_reasonable_translation_m_ =
      declare_parameter<double>("max_reasonable_translation_m", 10000.0);
    map_voxel_size_m_ = declare_parameter<double>("map_voxel_size_m", 0.10);
    max_map_voxels_ = static_cast<size_t>(
      std::max<int64_t>(1, declare_parameter<int64_t>("max_map_voxels", 2000000)));
    publish_every_n_clouds_ = std::max<int>(
      1, static_cast<int>(declare_parameter<int>("publish_every_n_clouds", 10)));

    const auto camera_to_lidar_t = declare_parameter<std::vector<double>>(
      "camera_child_to_lidar_base_translation", std::vector<double>{0.0, 0.0, 0.0});
    const auto camera_to_lidar_rpy_deg = declare_parameter<std::vector<double>>(
      "camera_child_to_lidar_base_rpy_deg", std::vector<double>{0.0, 0.0, 0.0});
    camera_child_to_lidar_base_ =
      makeTransformFromTranslationRpy(camera_to_lidar_t, camera_to_lidar_rpy_deg);
  }

  Eigen::Isometry3d makeTransformFromTranslationRpy(
    const std::vector<double> & translation,
    const std::vector<double> & rpy_deg) const
  {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    if (translation.size() == 3) {
      transform.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);
    }

    if (rpy_deg.size() == 3) {
      const double roll = rpy_deg[0] * kPi / 180.0;
      const double pitch = rpy_deg[1] * kPi / 180.0;
      const double yaw = rpy_deg[2] * kPi / 180.0;
      Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
      transform.linear() = (rz * ry * rx).toRotationMatrix();
    }
    return transform;
  }

  PoseSample odomToSample(const nav_msgs::msg::Odometry & msg)
  {
    PoseSample sample;
    sample.stamp = rclcpp::Time(msg.header.stamp);
    sample.received = now();
    sample.stamp_ok = stampLooksUsable(sample.stamp);
    sample.pose = poseMsgToEigen(msg.pose.pose);
    sample.pose_ok = poseLooksUsable(sample.pose);
    return sample;
  }

  Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::msg::Pose & pose) const
  {
    Eigen::Quaterniond q(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    if (std::isfinite(q.norm()) && q.norm() > 1e-6) {
      q.normalize();
      transform.linear() = q.toRotationMatrix();
    }
    transform.translation() =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    return transform;
  }

  geometry_msgs::msg::Pose eigenToPoseMsg(const Eigen::Isometry3d & pose) const
  {
    geometry_msgs::msg::Pose msg;
    Eigen::Quaterniond q(pose.rotation());
    q.normalize();
    msg.position.x = pose.translation().x();
    msg.position.y = pose.translation().y();
    msg.position.z = pose.translation().z();
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    return msg;
  }

  bool stampLooksUsable(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() <= 0) {
      return false;
    }
    if (!require_header_time_near_now_) {
      return true;
    }
    return std::abs((now() - stamp).seconds()) <= max_wall_stamp_skew_sec_;
  }

  bool poseLooksUsable(const Eigen::Isometry3d & pose) const
  {
    if (!isFiniteTransform(pose)) {
      return false;
    }
    return pose.translation().norm() <= max_reasonable_translation_m_;
  }

  void cameraOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    PoseSample sample = odomToSample(*msg);
    sample.pose = sample.pose * camera_child_to_lidar_base_;
    sample.pose_ok = sample.pose_ok && poseLooksUsable(sample.pose);

    camera_buffer_.push_back(sample);
    while (camera_buffer_.size() > 1000) {
      camera_buffer_.pop_front();
    }
  }

  void lidarOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    PoseSample lidar = odomToSample(*msg);
    if (!lidar.pose_ok) {
      publishStatus("lidar_pose_invalid", false, false, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN());
      return;
    }

    PoseSample camera;
    double camera_time_diff = std::numeric_limits<double>::infinity();
    const bool camera_ok = findNearestCamera(lidar.stamp, camera, camera_time_diff);

    double position_error = std::numeric_limits<double>::quiet_NaN();
    double orientation_error = std::numeric_limits<double>::quiet_NaN();
    bool comparable = false;

    Eigen::Isometry3d camera_in_map = Eigen::Isometry3d::Identity();
    if (camera_ok) {
      if (!alignment_ready_) {
        map_from_camera_odom_ = lidar.pose * camera.pose.inverse();
        alignment_ready_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Initialized camera-to-LiDAR odom alignment. This assumes the rig is static at startup.");
      }
      camera_in_map = map_from_camera_odom_ * camera.pose;
      position_error = (lidar.pose.translation() - camera_in_map.translation()).norm();
      orientation_error = rotationAngleRad(lidar.pose, camera_in_map);
      comparable = true;
    }

    updateSourceMode(camera_ok, comparable, position_error, orientation_error);

    Eigen::Isometry3d trusted_pose = lidar.pose;
    std::string reason = "lidar";
    if (source_mode_ == SourceMode::Camera && camera_ok && alignment_ready_) {
      trusted_pose = camera_in_map;
      reason = "camera";
    } else if (!camera_ok) {
      reason = "lidar_camera_unavailable";
    } else if (!comparable) {
      reason = "lidar_not_comparable";
    }

    publishTrustedOdom(*msg, trusted_pose, reason);
    pushTrustedPose(lidar.stamp, trusted_pose);
    publishStatus(reason, camera_ok, comparable, position_error, orientation_error, camera_time_diff);
  }

  bool findNearestCamera(const rclcpp::Time & target, PoseSample & out, double & best_diff) const
  {
    if (!stampLooksUsable(target)) {
      return false;
    }

    bool found = false;
    best_diff = std::numeric_limits<double>::infinity();
    for (const auto & sample : camera_buffer_) {
      if (!sample.stamp_ok || !sample.pose_ok) {
        continue;
      }
      const double diff = std::abs((sample.stamp - target).seconds());
      if (diff < best_diff) {
        best_diff = diff;
        out = sample;
        found = true;
      }
    }

    return found && best_diff <= max_time_diff_sec_;
  }

  void updateSourceMode(
    bool camera_ok, bool comparable, double position_error, double orientation_error)
  {
    if (!camera_ok) {
      if (source_mode_ == SourceMode::Camera) {
        RCLCPP_WARN(get_logger(), "Camera fallback lost; returning to LiDAR odometry.");
      }
      source_mode_ = SourceMode::Lidar;
      reject_count_ = 0;
      recover_count_ = 0;
      return;
    }

    const bool divergent = comparable &&
      (position_error > position_threshold_m_ || orientation_error > orientation_threshold_rad_);
    const bool recovered = comparable &&
      (position_error < recovery_position_threshold_m_ &&
       orientation_error < recovery_orientation_threshold_rad_);

    if (source_mode_ == SourceMode::Lidar) {
      reject_count_ = divergent ? reject_count_ + 1 : 0;
      recover_count_ = 0;
      if (reject_count_ >= reject_count_threshold_) {
        source_mode_ = SourceMode::Camera;
        recover_count_ = 0;
        RCLCPP_WARN(get_logger(), "LiDAR odometry rejected; using camera VIO fallback.");
      }
    } else {
      recover_count_ = recovered ? recover_count_ + 1 : 0;
      if (recover_count_ >= recover_count_threshold_) {
        source_mode_ = SourceMode::Lidar;
        reject_count_ = 0;
        RCLCPP_WARN(get_logger(), "LiDAR odometry recovered; returning to LiDAR source.");
      }
    }
  }

  void publishTrustedOdom(
    const nav_msgs::msg::Odometry & reference,
    const Eigen::Isometry3d & trusted_pose,
    const std::string & reason)
  {
    nav_msgs::msg::Odometry odom = reference;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose = eigenToPoseMsg(trusted_pose);
    odom.twist = reference.twist;

    if (reason == "camera") {
      for (double & value : odom.pose.covariance) {
        value = 0.0;
      }
      odom.pose.covariance[0] = 0.05;
      odom.pose.covariance[7] = 0.05;
      odom.pose.covariance[14] = 0.05;
      odom.pose.covariance[21] = 0.02;
      odom.pose.covariance[28] = 0.02;
      odom.pose.covariance[35] = 0.02;
    }

    trusted_odom_pub_->publish(odom);
  }

  void pushTrustedPose(const rclcpp::Time & stamp, const Eigen::Isometry3d & pose)
  {
    TrustedPose trusted;
    trusted.stamp = stamp;
    trusted.received = now();
    trusted.pose = pose;
    trusted_poses_.push_back(trusted);
    latest_trusted_pose_ = trusted;
    latest_trusted_pose_ready_ = true;

    while (trusted_poses_.size() > 500) {
      trusted_poses_.pop_front();
    }
  }

  void cloudCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    TrustedPose pose;
    if (!selectPoseForCloud(rclcpp::Time(msg->header.stamp), pose)) {
      publishStatus("skip_cloud_no_trusted_pose", false, false,
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
      return;
    }

    for (const auto & point : msg->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const Eigen::Vector3d p_map = pose.pose * Eigen::Vector3d(point.x, point.y, point.z);
      insertMapPoint(p_map, static_cast<float>(point.reflectivity));
    }

    cloud_count_++;
    if (cloud_count_ % publish_every_n_clouds_ == 0) {
      publishGuardedMap(msg->header.stamp);
    }
  }

  bool selectPoseForCloud(const rclcpp::Time & stamp, TrustedPose & out) const
  {
    if (!latest_trusted_pose_ready_) {
      return false;
    }

    bool found_by_stamp = false;
    double best_diff = std::numeric_limits<double>::infinity();
    if (stampLooksUsable(stamp)) {
      for (const auto & pose : trusted_poses_) {
        const double diff = std::abs((pose.stamp - stamp).seconds());
        if (diff < best_diff) {
          best_diff = diff;
          out = pose;
          found_by_stamp = true;
        }
      }
    }

    if (found_by_stamp && best_diff <= max_time_diff_sec_) {
      return true;
    }

    const double age = (now() - latest_trusted_pose_.received).seconds();
    if (age <= cloud_pose_max_age_sec_) {
      out = latest_trusted_pose_;
      return true;
    }
    return false;
  }

  void insertMapPoint(const Eigen::Vector3d & point, float intensity)
  {
    const double inv = 1.0 / std::max(0.01, map_voxel_size_m_);
    VoxelKey key;
    key.x = static_cast<int64_t>(std::floor(point.x() * inv));
    key.y = static_cast<int64_t>(std::floor(point.y() * inv));
    key.z = static_cast<int64_t>(std::floor(point.z() * inv));

    if (map_.find(key) != map_.end()) {
      return;
    }
    if (map_.size() >= max_map_voxels_) {
      return;
    }

    pcl::PointXYZI pcl_point;
    pcl_point.x = static_cast<float>(point.x());
    pcl_point.y = static_cast<float>(point.y());
    pcl_point.z = static_cast<float>(point.z());
    pcl_point.intensity = intensity;
    map_.emplace(key, pcl_point);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr buildMapCloud() const
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    cloud->reserve(map_.size());
    for (const auto & item : map_) {
      cloud->push_back(item.second);
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  void publishGuardedMap(const builtin_interfaces::msg::Time & stamp)
  {
    auto cloud = buildMapCloud();
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    guarded_map_pub_->publish(msg);
  }

  void saveMapCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (map_.empty()) {
      response->success = false;
      response->message = "guarded map is empty";
      return;
    }

    std::filesystem::path output_path(save_map_path_);
    if (!output_path.parent_path().empty()) {
      std::filesystem::create_directories(output_path.parent_path());
    }

    auto cloud = buildMapCloud();
    const int ret = pcl::io::savePCDFileBinary(save_map_path_, *cloud);
    response->success = ret == 0;
    response->message = ret == 0 ? ("saved " + save_map_path_) : ("failed to save " + save_map_path_);
  }

  void publishStatus(
    const std::string & reason,
    bool camera_ok,
    bool comparable,
    double position_error,
    double orientation_error,
    double time_diff = std::numeric_limits<double>::quiet_NaN())
  {
    std_msgs::msg::String msg;
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(3);
    oss << "source=" << (source_mode_ == SourceMode::Camera ? "camera" : "lidar")
        << " reason=" << reason
        << " camera_ok=" << yesNo(camera_ok)
        << " comparable=" << yesNo(comparable)
        << " aligned=" << yesNo(alignment_ready_)
        << " reject_count=" << reject_count_
        << " recover_count=" << recover_count_
        << " map_voxels=" << map_.size();
    if (std::isfinite(position_error)) {
      oss << " pos_error_m=" << position_error;
    }
    if (std::isfinite(orientation_error)) {
      oss << " rot_error_deg=" << orientation_error * 180.0 / kPi;
    }
    if (std::isfinite(time_diff)) {
      oss << " time_diff_s=" << time_diff;
    }
    msg.data = oss.str();
    status_pub_->publish(msg);
  }

  std::mutex mutex_;

  std::string lidar_odom_topic_;
  std::string camera_odom_topic_;
  std::string lidar_cloud_topic_;
  std::string trusted_odom_topic_;
  std::string guarded_map_topic_;
  std::string status_topic_;
  std::string save_map_service_;
  std::string save_map_path_;
  std::string map_frame_;
  std::string base_frame_;

  double position_threshold_m_ = 1.0;
  double orientation_threshold_rad_ = 20.0 * kPi / 180.0;
  double recovery_position_threshold_m_ = 0.45;
  double recovery_orientation_threshold_rad_ = 8.0 * kPi / 180.0;
  int reject_count_threshold_ = 3;
  int recover_count_threshold_ = 20;
  double max_time_diff_sec_ = 0.10;
  bool require_header_time_near_now_ = true;
  double max_wall_stamp_skew_sec_ = 5.0;
  double cloud_pose_max_age_sec_ = 1.0;
  double max_reasonable_translation_m_ = 10000.0;
  double map_voxel_size_m_ = 0.10;
  size_t max_map_voxels_ = 2000000;
  int publish_every_n_clouds_ = 10;

  Eigen::Isometry3d camera_child_to_lidar_base_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d map_from_camera_odom_ = Eigen::Isometry3d::Identity();
  bool alignment_ready_ = false;

  SourceMode source_mode_ = SourceMode::Lidar;
  int reject_count_ = 0;
  int recover_count_ = 0;

  std::deque<PoseSample> camera_buffer_;
  std::deque<TrustedPose> trusted_poses_;
  TrustedPose latest_trusted_pose_;
  bool latest_trusted_pose_ready_ = false;

  std::unordered_map<VoxelKey, pcl::PointXYZI, VoxelKeyHash> map_;
  uint64_t cloud_count_ = 0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr camera_odom_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr trusted_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr guarded_map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseGuardMapperNode>());
  rclcpp::shutdown();
  return 0;
}
