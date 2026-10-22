#include "loclite/loc_interface.h"

#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace loclite {

CloudPtr LoadFixedMap(const std::string &map_file) {
    CloudPtr map(new Cloud());
    if (pcl::io::loadPCDFile<PointType>(map_file, *map) == -1)
        throw std::runtime_error("Failed to load fixed map " + map_file);
    std::vector<int> unused;
    map->is_dense = false;
    pcl::removeNaNFromPointCloud(*map, *map, unused);
    if (map->empty()) throw std::runtime_error("Fixed map " + map_file + " is empty");
    LOCLITE_INFO("Load map success. File: " << map_file << ", " << map->size() << " points");
    return map;
}

LocInterface::LocInterface(rclcpp::Node &node, LocParams params, std::function<void(bool)> on_active)
    : node_(node), params_(std::move(params)), on_active_(std::move(on_active)) {
    odom_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>("hikari_loc/odom", 10);
    path_pub_ = node_.create_publisher<nav_msgs::msg::Path>("hikari_loc/path", 10);
    loc_state_pub_ = node_.create_publisher<std_msgs::msg::Int32>("hikari_loc/loc_state", 10);
    ndt_status_pub_ = node_.create_publisher<std_msgs::msg::Int32>("hikari_loc/ndt_status", 10);
    loc_status_pub_ = node_.create_publisher<std_msgs::msg::Float32MultiArray>("hikari_loc/status", 10);
    loc_marker_pub_ = node_.create_publisher<visualization_msgs::msg::Marker>("hikari_loc/loc_status", 10);
    global_map_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
        "/pcdmap", rclcpp::QoS(1).reliable().transient_local());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    path_.header.frame_id = params_.map_frame;

    init_pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            const auto &pose = msg->pose.pose;
            const Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);
            Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z);
            if (!t.allFinite() || !q.coeffs().allFinite() || q.norm() < 1e-6 ||
                (!msg->header.frame_id.empty() && msg->header.frame_id != params_.map_frame)) {
                RCLCPP_WARN(node_.get_logger(), "Rejected invalid initialpose or non-map frame");
                return;
            }
            q.normalize();
            std::lock_guard<std::mutex> lock(mutex_);
            initial_pose_.setIdentity();
            initial_pose_.block<3, 3>(0, 0) = q.toRotationMatrix();
            initial_pose_.block<3, 1>(0, 3) = t;
            has_initial_pose_ = true;
            LOCLITE_INFO("GET Initial guess: " << t.transpose() << " yaw: "
                                               << std::atan2(initial_pose_(1, 0), initial_pose_(0, 0)));
        });

    set_active_srv_ = node_.create_service<std_srvs::srv::SetBool>(
        "~/set_active", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
            const bool was = active_.exchange(req->data);
            if (was != req->data) {
                on_active_(req->data);
                LOCLITE_INFO("[Control] " << (req->data ? "active" : "standby"));
            }
            res->success = true;
            res->message = std::string(req->data ? "active" : "standby") +
                           (was == req->data ? " (unchanged)" : "");
        });

    status_timer_ = node_.create_wall_timer(std::chrono::milliseconds(200), [this]() { PublishStatus(); });
}

void LocInterface::PublishMap(const Cloud &map) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(map, msg);
    msg.header.frame_id = params_.map_frame;
    msg.header.stamp = node_.now();
    global_map_pub_->publish(msg);
}

void LocInterface::ImuReceived() {
    const double time = node_.now().seconds();
    std::lock_guard<std::mutex> lock(mutex_);
    last_imu_received_ = time;
}

void LocInterface::LidarReceived() {
    const double time = node_.now().seconds();
    std::lock_guard<std::mutex> lock(mutex_);
    last_lidar_received_ = time;
}

void LocInterface::ResetWatchdog() {
    const double time = node_.now().seconds();
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_imu_received_ >= 0) last_imu_received_ = time;
    if (last_lidar_received_ >= 0) last_lidar_received_ = time;
}

bool LocInterface::TakeInitialPose(Eigen::Matrix4d &T_map_lidar) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_initial_pose_) return false;
    has_initial_pose_ = false;
    T_map_lidar = initial_pose_;
    return true;
}

void LocInterface::ClearPath() {
    std::lock_guard<std::mutex> lock(mutex_);
    path_.poses.clear();
    last_output_stamp_ = last_path_stamp_ = -1;
    fps_ = 0;
}

void LocInterface::Publish(double stamp_sec, const Eigen::Matrix4d &T_map_imu,
                           const Eigen::Matrix4d &T_imu_lidar, const Eigen::Vector3d &v_world) {
    const builtin_interfaces::msg::Time stamp = rclcpp::Time(static_cast<int64_t>(stamp_sec * 1e9));
    const Eigen::Matrix4d map_lidar = T_map_imu * T_imu_lidar;
    const Eigen::Matrix3d R = map_lidar.block<3, 3>(0, 0);
    const Eigen::Vector3d t = map_lidar.block<3, 1>(0, 3);
    Eigen::Quaterniond q(R);
    q.normalize();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = params_.map_frame;
    odom.child_frame_id = params_.lidar_frame;
    odom.pose.pose.position.x = t.x();
    odom.pose.pose.position.y = t.y();
    odom.pose.pose.position.z = t.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    // Same convention as hikari_loclite: world IMU velocity rotated into lidar.
    const Eigen::Vector3d velocity = R.transpose() * v_world;
    odom.twist.twist.linear.x = velocity.x();
    odom.twist.twist.linear.y = velocity.y();
    odom.twist.twist.linear.z = velocity.z();
    if (params_.publish_odom) odom_pub_->publish(odom);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (params_.publish_path && (last_path_stamp_ < 0 || stamp_sec - last_path_stamp_ >= 0.1)) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = odom.header;
            pose.pose = odom.pose.pose;
            path_.header = odom.header;
            path_.poses.push_back(pose);
            if (path_.poses.size() > 5000) path_.poses.erase(path_.poses.begin());
            path_pub_->publish(path_);
            last_path_stamp_ = stamp_sec;
        }
        if (last_output_stamp_ >= 0 && stamp_sec > last_output_stamp_)
            fps_ = 1.0 / (stamp_sec - last_output_stamp_);
        last_output_stamp_ = stamp_sec;
    }
    if (!params_.publish_tf) return;
    auto send = [&](const std::string &parent, const std::string &child, const Eigen::Matrix4d &pose) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = parent;
        tf.child_frame_id = child;
        tf.transform.translation.x = pose(0, 3);
        tf.transform.translation.y = pose(1, 3);
        tf.transform.translation.z = pose(2, 3);
        Eigen::Quaterniond rotation(Eigen::Matrix3d(pose.block<3, 3>(0, 0)));
        rotation.normalize();
        tf.transform.rotation.x = rotation.x();
        tf.transform.rotation.y = rotation.y();
        tf.transform.rotation.z = rotation.z();
        tf.transform.rotation.w = rotation.w();
        tf_broadcaster_->sendTransform(tf);
    };
    send(params_.map_frame, params_.base_frame, map_lidar * params_.T_lidar_base);
    // Preserve lidar yaw while removing roll/pitch in the gravity-aligned map.
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    Eigen::Matrix4d lidar_level = Eigen::Matrix4d::Identity();
    lidar_level.block<3, 3>(0, 0) =
        R.transpose() * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    send(params_.lidar_frame, params_.level_frame, lidar_level);
}

void LocInterface::PublishStatus() {
    const double time = node_.now().seconds();
    double imu_age, lidar_age, fps;
    geometry_msgs::msg::Point last_position;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        imu_age = last_imu_received_ < 0 ? -1 : std::max(0.0, time - last_imu_received_);
        lidar_age = last_lidar_received_ < 0 ? -1 : std::max(0.0, time - last_lidar_received_);
        fps = fps_;
        if (!path_.poses.empty()) last_position = path_.poses.back().pose.position;
    }
    if (!Active()) {
        state_.store(kWaitForInitialPose);
    } else if (imu_age < 0 || lidar_age < 0 || imu_age > params_.imu_timeout ||
               lidar_age > params_.lidar_timeout) {
        int good = kGood;
        state_.compare_exchange_strong(good, kLost);
    }
    const int loc_state = State();
    std_msgs::msg::Int32 state;
    state.data = loc_state;
    loc_state_pub_->publish(state);
    state.data = 0;
    ndt_status_pub_->publish(state);
    std_msgs::msg::Float32MultiArray status;
    status.layout.dim.resize(1);
    status.layout.dim[0].label = "state;ndt_conf;imu_age_s;lidar_age_s;fps;in_map";
    status.layout.dim[0].size = status.layout.dim[0].stride = 6;
    // Registration fitness is not hikari's NDT confidence: explicitly report unavailable.
    status.data = {static_cast<float>(loc_state), -1.f, static_cast<float>(imu_age),
                   static_cast<float>(lidar_age), loc_state == kGood ? static_cast<float>(fps) : 0.f,
                   loc_state == kGood ? 1.f : 0.f};
    loc_status_pub_->publish(status);
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = params_.map_frame;
    marker.header.stamp = node_.now();
    marker.ns = "loc_status";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = last_position;
    marker.pose.position.z += 2;
    marker.pose.orientation.w = 1;
    marker.scale.z = 1.5;
    marker.color.a = 1;
    marker.color.r = loc_state == kGood ? 0 : 1;
    marker.color.g = loc_state == kGood ? 1 : 0;
    marker.text = loc_state == kGood            ? "GOOD"
                  : loc_state == kInitializing ? "INIT"
                  : loc_state == kLost         ? "LOST"
                                               : "WAIT FOR /initialpose";
    loc_marker_pub_->publish(marker);
}

CommandLine ParseCommandLine(int argc, char **argv) {
    CommandLine cli;
    cli.args.push_back(argv[0]);
    bool ros_args = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--ros-args") ros_args = true;
        if (!ros_args && (arg == "--help" || arg == "-h")) {
            cli.help = true;
            return cli;
        }
        if (!ros_args && (arg == "--config" || arg == "--map_path" || arg.rfind("--config=", 0) == 0 ||
                          arg.rfind("--map_path=", 0) == 0)) {
            const auto equal = arg.find('=');
            const auto flag = arg.substr(0, equal);
            std::string value;
            if (equal != std::string::npos)
                value = arg.substr(equal + 1);
            else if (i + 1 < argc)
                value = argv[++i];
            if (value.empty() || value.rfind("--", 0) == 0)
                throw std::runtime_error("Missing value for " + flag);
            (flag == "--config" ? cli.config : cli.map_path) = value;
        } else {
            cli.args.push_back(arg);
        }
    }
    return cli;
}

}  // namespace loclite
