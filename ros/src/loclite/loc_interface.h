// Fixed-map localization (loclite), ported from Super-LIO's run_loclite_online.
//
// ROS 2 surface shared by every run_loclite_online: parameters, outputs, TF,
// /initialpose, ~/set_active and the input watchdog. Topics, frames, state
// numbering and parameter names follow Super-LIO (and hikari_loclite).
#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include "loclite/common.h"
#include "loclite/reloc_options.h"

namespace loclite {

struct LocParams {
    std::string map_file;  ///< resolved global.pcd
    std::string map_frame = "map";
    std::string lidar_frame = "livox_frame";
    std::string base_frame = "livox_frame";
    std::string level_frame = "level_frame";
    Eigen::Matrix4d T_lidar_base = Eigen::Matrix4d::Identity();
    bool publish_tf = true, publish_odom = true, publish_path = true;
    double imu_timeout = 1.0, lidar_timeout = 2.0;
    RelocOptions reloc;
};

/// Reads every localization parameter through get(name, default), which returns
/// a value of the default's type: a ROS parameter declaration or a YAML lookup.
/// Throws std::runtime_error on a missing map or an invalid value.
template <class Get>
LocParams LoadLocParams(Get &&get) {
    LocParams p;
    // Map path priority: system.map_path (= --map_path) -> fixed_map.global_pcd.
    const auto map_dir = get("system.map_path", std::string());
    const auto pcd = get("fixed_map.global_pcd", std::string());
    const auto map_file = map_dir.empty() ? std::filesystem::path(pcd)
                                          : std::filesystem::path(map_dir) / "global.pcd";
    if (map_file.empty() || !std::filesystem::is_regular_file(map_file))
        throw std::runtime_error(
            "Fixed map missing: provide --map_path <directory containing global.pcd> or "
            "fixed_map.global_pcd");
    p.map_file = std::filesystem::absolute(map_file).string();

    p.map_frame = get("common.map_frame_id", p.map_frame);
    p.lidar_frame = get("system.lidar_frame_id", p.lidar_frame);
    p.base_frame = get("system.base_frame_id", p.base_frame);
    p.level_frame = get("system.level_frame_id", p.level_frame);
    if (p.map_frame.empty() || p.lidar_frame.empty() || p.base_frame.empty() ||
        p.level_frame.empty() || p.map_frame == p.base_frame || p.map_frame == p.lidar_frame ||
        p.level_frame == p.lidar_frame || p.level_frame == p.base_frame ||
        p.level_frame == p.map_frame)
        throw std::runtime_error("Localization TF frame names must form a valid tree");
    const double x = get("system.base_to_lidar_x", 0.0);
    const double y = get("system.base_to_lidar_y", 0.0);
    const double z = get("system.base_to_lidar_z", 0.0);
    const double roll = get("system.base_to_lidar_roll", 0.0);
    const double pitch = get("system.base_to_lidar_pitch", 0.0);
    const double yaw = get("system.base_to_lidar_yaw", 0.0);
    const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                                         .toRotationMatrix();
    Eigen::Matrix4d base_lidar = Eigen::Matrix4d::Identity();
    base_lidar.block<3, 3>(0, 0) = rotation;
    base_lidar.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
    if (!base_lidar.allFinite()) throw std::runtime_error("Invalid base-to-lidar extrinsic");
    if (p.base_frame == p.lidar_frame &&
        (Eigen::Vector3d(x, y, z).norm() > 1e-6 || (rotation - Eigen::Matrix3d::Identity()).norm() > 1e-6))
        throw std::runtime_error("Identical base/lidar frames require identity extrinsic");
    p.T_lidar_base = base_lidar.inverse();
    p.publish_tf = get("runtime.publish_tf", p.publish_tf);
    p.publish_odom = get("runtime.publish_odom", p.publish_odom);
    p.publish_path = get("runtime.publish_path", p.publish_path);
    p.imu_timeout = get("runtime.imu_timeout_sec", p.imu_timeout);
    p.lidar_timeout = get("runtime.lidar_timeout_sec", p.lidar_timeout);

    auto &r = p.reloc;
    r.cold_start.enable = get("reloc.cold_start.enable", r.cold_start.enable);
    r.cold_start.max_attempts = get("reloc.cold_start.max_attempts", r.cold_start.max_attempts);
    r.origin_prior.enable = get("reloc.origin_prior.enable", r.origin_prior.enable);
    r.origin_prior.x = get("reloc.origin_prior.x", r.origin_prior.x);
    r.origin_prior.y = get("reloc.origin_prior.y", r.origin_prior.y);
    r.origin_prior.z = get("reloc.origin_prior.z", r.origin_prior.z);
    r.origin_prior.yaw_step_deg = get("reloc.origin_prior.yaw_step_deg", r.origin_prior.yaw_step_deg);
    r.origin_prior.max_delta_trans = get("reloc.origin_prior.max_delta_trans", r.origin_prior.max_delta_trans);
    r.kiss.voxel_sizes = get("reloc.kiss.voxel_sizes", r.kiss.voxel_sizes);
    r.kiss.use_quatro = get("reloc.kiss.use_quatro", r.kiss.use_quatro);
    r.kiss.target_pre_voxel = get("reloc.kiss.target_pre_voxel", r.kiss.target_pre_voxel);
    r.kiss.max_target_points = static_cast<std::size_t>(
        get("reloc.kiss.max_target_points", static_cast<int>(r.kiss.max_target_points)));
    r.kiss.min_rotation_inliers = static_cast<std::size_t>(
        get("reloc.kiss.min_rotation_inliers", static_cast<int>(r.kiss.min_rotation_inliers)));
    r.kiss.min_final_inliers = static_cast<std::size_t>(
        get("reloc.kiss.min_final_inliers", static_cast<int>(r.kiss.min_final_inliers)));
    r.kiss.min_agreeing_candidates = get("reloc.kiss.min_agreeing_candidates", r.kiss.min_agreeing_candidates);
    r.kiss.max_tilt_deg = get("reloc.kiss.max_tilt_deg", r.kiss.max_tilt_deg);
    r.genz.map_voxel = get("reloc.genz.map_voxel", r.genz.map_voxel);
    r.genz.max_points_per_voxel = get("reloc.genz.max_points_per_voxel", r.genz.max_points_per_voxel);
    r.genz.planarity_threshold = get("reloc.genz.planarity_threshold", r.genz.planarity_threshold);
    r.genz.source_voxel = get("reloc.genz.source_voxel", r.genz.source_voxel);
    r.genz.max_iterations = get("reloc.genz.max_iterations", r.genz.max_iterations);
    r.genz.convergence = get("reloc.genz.convergence", r.genz.convergence);
    r.genz.coarse_max_corr = get("reloc.genz.coarse_max_corr", r.genz.coarse_max_corr);
    r.genz.fine_max_corr = get("reloc.genz.fine_max_corr", r.genz.fine_max_corr);
    r.fc.map_voxel = get("reloc.fc.map_voxel", r.fc.map_voxel);
    r.fc.inlier_dist = get("reloc.fc.inlier_dist", r.fc.inlier_dist);
    r.fc.min_inlier_ratio = get("reloc.fc.min_inlier_ratio", r.fc.min_inlier_ratio);
    r.fc.wall_band_min = get("reloc.fc.wall_band_min", r.fc.wall_band_min);
    r.fc.wall_band_max = get("reloc.fc.wall_band_max", r.fc.wall_band_max);
    r.fc.min_wall_inlier_ratio = get("reloc.fc.min_wall_inlier_ratio", r.fc.min_wall_inlier_ratio);
    r.fc.min_coverage = get("reloc.fc.min_coverage", r.fc.min_coverage);
    r.fc.min_sector_points = get("reloc.fc.min_sector_points", r.fc.min_sector_points);
    r.fc.max_rmse = get("reloc.fc.max_rmse", r.fc.max_rmse);
    r.fc.max_delta_trans_manual = get("reloc.fc.max_delta_trans_manual", r.fc.max_delta_trans_manual);
    r.fc.max_delta_rot_deg_manual = get("reloc.fc.max_delta_rot_deg_manual", r.fc.max_delta_rot_deg_manual);
    r.fc.max_delta_trans_kiss = get("reloc.fc.max_delta_trans_kiss", r.fc.max_delta_trans_kiss);
    r.fc.max_delta_rot_deg_kiss = get("reloc.fc.max_delta_rot_deg_kiss", r.fc.max_delta_rot_deg_kiss);
    r.fc.ambiguity_trans = get("reloc.fc.ambiguity_trans", r.fc.ambiguity_trans);
    r.fc.ambiguity_rot_deg = get("reloc.fc.ambiguity_rot_deg", r.fc.ambiguity_rot_deg);
    const bool kiss_voxels_ok =
        !r.kiss.voxel_sizes.empty() &&
        std::all_of(r.kiss.voxel_sizes.begin(), r.kiss.voxel_sizes.end(), [](double v) { return v >= 0.05; });
    if (r.cold_start.max_attempts < 0 || !kiss_voxels_ok || r.origin_prior.yaw_step_deg <= 0 ||
        r.origin_prior.yaw_step_deg > 360 || r.origin_prior.max_delta_trans <= 0 ||
        r.kiss.min_agreeing_candidates < 1 || r.genz.map_voxel <= 0 || r.genz.max_points_per_voxel < 1 ||
        r.genz.max_iterations < 1 || r.genz.coarse_max_corr <= 0 || r.genz.fine_max_corr <= 0 ||
        r.fc.map_voxel <= 0 || r.fc.inlier_dist <= 0 || r.fc.wall_band_min >= r.fc.wall_band_max ||
        r.fc.min_sector_points < 1)
        throw std::runtime_error("Invalid reloc.* parameters");
    if (p.imu_timeout <= 0 || p.lidar_timeout <= 0)
        throw std::runtime_error("Invalid localization watchdog parameters");
    return p;
}

/// Loads <map_file> as XYZI and drops NaNs. Throws on a missing, unreadable or empty map.
CloudPtr LoadFixedMap(const std::string &map_file);

class LocInterface {
public:
    /// on_active(true/false) is called from ~/set_active when the state flips:
    /// the node creates/destroys its sensor subscriptions and drops its buffers.
    LocInterface(rclcpp::Node &node, LocParams params, std::function<void(bool)> on_active);

    const LocParams &params() const { return params_; }

    /// /pcdmap: reliable, transient-local, depth 1.
    void PublishMap(const Cloud &map);

    /// /hikari_loc/odom, /hikari_loc/path and TF for one tracked scan.
    /// T_imu_lidar: p_imu = R * p_lidar + t. v_world: IMU velocity in the map.
    void Publish(double stamp, const Eigen::Matrix4d &T_map_imu, const Eigen::Matrix4d &T_imu_lidar,
                 const Eigen::Vector3d &v_world);
    void ClearPath();

    void SetState(int state) { state_.store(state); }
    int State() const { return state_.load(); }
    bool Active() const { return active_.load(); }

    void ImuReceived();
    void LidarReceived();
    /// A long blocking alignment must not look like a sensor dropout.
    void ResetWatchdog();

    /// Latest valid /initialpose (map <- lidar_frame), consumed once.
    bool TakeInitialPose(Eigen::Matrix4d &T_map_lidar);

private:
    void PublishStatus();

    rclcpp::Node &node_;
    LocParams params_;
    std::function<void(bool)> on_active_;
    std::atomic<int> state_{kWaitForInitialPose};
    std::atomic<bool> active_{true};

    std::mutex mutex_;  // guards everything below
    double last_imu_received_ = -1, last_lidar_received_ = -1;
    double last_output_stamp_ = -1, last_path_stamp_ = -1, fps_ = 0;
    nav_msgs::msg::Path path_;
    bool has_initial_pose_ = false;
    Eigen::Matrix4d initial_pose_ = Eigen::Matrix4d::Identity();

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr loc_state_pub_, ndt_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr loc_status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loc_marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_active_srv_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

/// run_loclite_online command line: --config <yaml> [--map_path <dir>] [--ros-args ...].
/// Both "--flag value" and "--flag=value" work; flags after --ros-args are ROS's.
struct CommandLine {
    bool help = false;
    std::string config, map_path;
    std::vector<std::string> args;  ///< argv[0] and everything not consumed above
};
CommandLine ParseCommandLine(int argc, char **argv);

}  // namespace loclite
