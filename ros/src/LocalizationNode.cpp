// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "LocalizationNode.hpp"

#include "CloudConversion.hpp"
#include "config/ConfigIo.hpp"

#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <optional>
#include <stdexcept>

namespace genz_lio {
namespace ros_wrapper {
namespace {

/// "a.b.c" in a nested YAML map; a literal "a.b.c" (or "b.c") key also counts.
std::optional<YAML::Node> lookup(const YAML::Node &node, const std::string &dotted) {
    if (!node.IsMap()) return std::nullopt;
    if (const YAML::Node literal = node[dotted]) return literal;
    const auto dot = dotted.find('.');
    if (dot == std::string::npos) return std::nullopt;
    const YAML::Node child = node[dotted.substr(0, dot)];
    if (!child) return std::nullopt;
    return lookup(child, dotted.substr(dot + 1));
}

template <class T>
T yamlValue(const YAML::Node &root, const std::string &dotted, const T &fallback) {
    const auto node = lookup(root, dotted);
    if (!node || node->IsNull()) return fallback;
    try {
        return node->as<T>();
    } catch (const YAML::Exception &error) {
        throw std::runtime_error("config key '" + dotted + "': " + error.what());
    }
}

}  // namespace

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("genz_lio_loc", options) {
    const auto config_path = declare_parameter<std::string>("config_path", "");
    if (config_path.empty())
        throw std::runtime_error("--config is required (use config/loclite_livox.yaml)");

    std::vector<std::string> warnings;
    config_ = loadConfig(config_path, &warnings);
    for (const auto &warning : warnings) RCLCPP_WARN(get_logger(), "config: %s", warning.c_str());
    readNodeOptions(config_path);
    if (config_.preprocess.lidar_type == LidarType::Livox) {
#ifndef GENZ_LIO_WITH_LIVOX
        throw std::runtime_error(
            "lidar_type is 'livox', which needs the livox_ros_driver2 CustomMsg, but GenZ-LIO "
            "was built without it. Install and source livox_ros_driver2 and rebuild, or "
            "publish the sensor as PointCloud2 and set lidar_type to 'livox_pcl'.");
#endif
    }

    // Localization surface (map, frames, outputs, reloc.*), same keys as
    // Super-LIO; YAML values are the defaults, `--ros-args -p` overrides them.
    const YAML::Node root = YAML::LoadFile(config_path);
    auto get = [this, &root](const std::string &name, auto fallback) {
        using T = decltype(fallback);
        return declare_parameter<T>(name, yamlValue<T>(root, name, fallback));
    };
    auto params = loclite::LoadLocParams(get);
    const double map_voxel = get("fixed_map.voxel_size", 0.1);
    const double map_point_cov = get("fixed_map.point_cov", 0.0004);
    if (map_point_cov <= 0 || config_.mapping.voxel_size <= 0)
        throw std::runtime_error("Invalid localization sensor/filter parameters");

    buffer_.setPreprocessConfig(config_.preprocess);
    preprocessor_ = std::make_unique<Preprocessor>(config_.preprocess);
    T_imu_lidar_.block<3, 3>(0, 0) = config_.mapping.extrinsic_r;
    T_imu_lidar_.block<3, 1>(0, 3) = config_.mapping.extrinsic_t;

    const auto map = loclite::LoadFixedMap(params.map_file);
    const auto started = std::chrono::steady_clock::now();
    {
        // The fixed map is the one GenZ-LIO tracks against: never folded into, never pruned.
        loclite::CloudPtr ds = map;
        if (map_voxel > 0) {
            const float leaf = static_cast<float>(map_voxel);
            pcl::VoxelGrid<loclite::PointType> grid;
            grid.setLeafSize(leaf, leaf, leaf);
            grid.setInputCloud(map);
            ds.reset(new loclite::Cloud());
            grid.filter(*ds);
        }
        std::vector<Vec3> points;
        points.reserve(ds->size());
        for (const auto &p : ds->points) points.emplace_back(p.x, p.y, p.z);
        pipeline_ = std::make_unique<GenZLIO>(config_);
        pipeline_->setFixedMap(points, map_point_cov);
        RCLCPP_INFO(get_logger(), "Map init success: %zu points -> %zu voxel-map roots in %ld ms",
                    ds->size(), pipeline_->map().size(),
                    static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                          std::chrono::steady_clock::now() - started)
                                          .count()));
    }

    relocalizer_ = std::make_unique<loclite::Relocalizer>(params.reloc);
    if (!relocalizer_->SetMap(map)) throw std::runtime_error("Reloc target init failed");
    RCLCPP_INFO(get_logger(), "[Reloc]: targets ready; cold start %s", relocalizer_->ColdStartMode());

    loc_ = std::make_unique<loclite::LocInterface>(*this, std::move(params), [this](bool active) {
        if (active) {
            createSensorSubscriptions();
        } else {
            destroySensorSubscriptions();
        }
        // The processing thread starts a fresh session on the same map.
        buffer_.clear();
        restart_ = true;
    });
    loc_->PublishMap(*map);

    // The processing group belongs to a dedicated executor (see run_loclite_online).
    processing_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    timer_ = create_wall_timer(std::chrono::milliseconds(2), [this] { spinOnce(); }, processing_group_);
    createSensorSubscriptions();
    GENZ_LOG_INFO(this, "GenZ-LIO localization listening on %s and %s", lidar_topic_.c_str(),
                  imu_topic_.c_str());
}

void LocalizationNode::readNodeOptions(const std::string &config_path) {
    // Same keys as OdometryServer::readNodeOptions.
    const YAML::Node root = YAML::LoadFile(config_path);
    if (const auto common = root["common"]) {
        if (common["lidar_topic"]) lidar_topic_ = common["lidar_topic"].as<std::string>();
        if (common["imu_topic"]) imu_topic_ = common["imu_topic"].as<std::string>();
        if (common["time_offset"]) time_offset_ = common["time_offset"].as<double>();
        if (common["qos_reliability"]) {
            const auto value = common["qos_reliability"].as<std::string>();
            if (value == "reliable") {
                qos_reliable_ = true;
            } else if (value == "best_effort") {
                qos_reliable_ = false;
            } else {
                GENZ_LOG_WARN(this, "unknown qos_reliability '%s'; expected reliable or "
                                    "best_effort, keeping reliable", value.c_str());
            }
        }
        if (common["qos_depth"]) qos_depth_ = common["qos_depth"].as<int>();
    }
    const auto lidar_override = declare_parameter<std::string>("lidar_topic", "");
    const auto imu_override = declare_parameter<std::string>("imu_topic", "");
    if (!lidar_override.empty()) lidar_topic_ = lidar_override;
    if (!imu_override.empty()) imu_topic_ = imu_override;
}

void LocalizationNode::createSensorSubscriptions() {
    if (imu_sub_) return;
    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<std::size_t>(qos_depth_))};
    if (qos_reliable_) {
        qos.reliable();
    } else {
        qos.best_effort();
    }
    if (config_.preprocess.lidar_type == LidarType::Livox) {
#ifdef GENZ_LIO_WITH_LIVOX
        livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lidar_topic_, qos,
            [this](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { onLivox(*msg); });
#endif
    } else {
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, qos, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { onCloud(*msg); });
    }
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { onImu(*msg); });
}

void LocalizationNode::destroySensorSubscriptions() {
    imu_sub_.reset();
    cloud_sub_.reset();
#ifdef GENZ_LIO_WITH_LIVOX
    livox_sub_.reset();
#endif
}

void LocalizationNode::onImu(const Imu &msg) {
    loc_->ImuReceived();
    ImuSample sample;
    sample.timestamp = toSeconds(msg.header.stamp);
    sample.linear_acceleration =
        Vec3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    sample.angular_velocity = Vec3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    const auto event = buffer_.push(sample);
    if (event == ScanBuffer::Event::ImuStampsWentBackwards) {
        GENZ_LOG_WARN(this, "IMU stamps went backwards; cleared both streams, localization restarts");
    } else if (event == ScanBuffer::Event::InvalidImu) {
        GENZ_LOG_WARN(this, "non-finite IMU sample skipped");
    }
}

void LocalizationNode::onCloud(const PointCloud2 &msg) {
    loc_->LidarReceived();
    auto converted = toRawScan(msg, config_.preprocess.lidar_type);
    if (!converted.warning.empty()) GENZ_LOG_WARN(this, "point cloud: %s", converted.warning.c_str());
    if (converted.scan.points.empty()) return;
    const double stamp = toSeconds(msg.header.stamp) + time_offset_;
    const auto event = buffer_.push(std::move(converted.scan), stamp, converted.duration);
    if (event == ScanBuffer::Event::ScanStampsWentBackwards) {
        GENZ_LOG_WARN(this, "LiDAR stamps went backwards; cleared both streams, localization restarts");
    } else if (event == ScanBuffer::Event::InvalidScan) {
        GENZ_LOG_WARN(this, "LiDAR scan has no usable point timing; scan skipped");
    }
}

#ifdef GENZ_LIO_WITH_LIVOX
void LocalizationNode::onLivox(const livox_ros_driver2::msg::CustomMsg &msg) {
    loc_->LidarReceived();
    // Same conversion as OdometryServer::onLivox.
    RawScan scan;
    scan.has_time_offsets = true;
    scan.points.reserve(msg.point_num);
    for (std::size_t i = 0; i < msg.point_num; ++i) {
        const auto &point = msg.points[i];
        const bool good_return = (point.tag & 0x30) == 0x10 || (point.tag & 0x30) == 0x00;
        if (!good_return) continue;
        Point out;
        out.position = Vec3(point.x, point.y, point.z);
        if (!out.position.allFinite()) continue;
        out.intensity = point.reflectivity;
        out.time_offset = static_cast<float>(point.offset_time * 1e-9);
        scan.points.push_back(out);
    }
    if (scan.points.empty()) return;
    const double stamp = toSeconds(msg.header.stamp) + time_offset_;
    const auto event = buffer_.push(std::move(scan), stamp, 0.0);
    if (event == ScanBuffer::Event::ScanStampsWentBackwards) {
        GENZ_LOG_WARN(this, "LiDAR stamps went backwards; cleared both streams, localization restarts");
    }
}
#endif

void LocalizationNode::resetLocalization() {
    tracking_ = false;
    relocalizer_->Reset();
    pipeline_->resetOdometry();
    loc_->ClearPath();
    loc_->SetState(loclite::kWaitForInitialPose);
}

void LocalizationNode::spinOnce() {
    ScanBuffer::BufferedScan scan;
    std::vector<ImuSample> imu;
    while (loc_->Active()) {
        if (restart_.exchange(false)) {
            resetLocalization();
            epoch_synced_ = false;
        }
        // /initialpose (map <- lidar) interrupts a cold start or the current track.
        Eigen::Matrix4d T_map_lidar;
        if (loc_->TakeInitialPose(T_map_lidar)) {
            resetLocalization();
            relocalizer_->RequestManual(T_map_lidar * T_imu_lidar_.inverse());
            loc_->SetState(loclite::kInitializing);
        }
        // Once tracking is lost, require a validated new initial pose.
        if (tracking_ && loc_->State() == loclite::kLost) resetLocalization();

        if (!buffer_.pop(scan, imu)) return;
        if (scan.epoch != pipeline_epoch_) {
            // Stamps went backwards (after a restart: the epoch it started).
            pipeline_epoch_ = scan.epoch;
            if (epoch_synced_) resetLocalization();
        }
        epoch_synced_ = true;
        if (tracking_) {
            track(scan, imu);
        } else {
            initialize(scan, imu);
        }
    }
}

void LocalizationNode::initialize(const ScanBuffer::BufferedScan &scan, const std::vector<ImuSample> &imu) {
    const PointCloud points = preprocessor_->process(scan.scan);
    const Mat3 R = T_imu_lidar_.block<3, 3>(0, 0);
    const Vec3 t = T_imu_lidar_.block<3, 1>(0, 3);
    loclite::Cloud scan_imu;
    scan_imu.reserve(points.size());
    for (const auto &point : points) {
        const Vec3 p = R * point.position + t;
        loclite::PointType q;
        q.x = static_cast<float>(p.x());
        q.y = static_cast<float>(p.y());
        q.z = static_cast<float>(p.z());
        q.intensity = point.intensity;
        scan_imu.push_back(q);
    }
    std::vector<Vec3> acc, gyr;
    for (const auto &sample : imu) {
        acc.push_back(sample.linear_acceleration);
        gyr.push_back(sample.angular_velocity);
    }

    using Status = loclite::Relocalizer::Status;
    switch (relocalizer_->AddFrame(scan_imu, acc, gyr)) {
        case Status::kIdle:
            break;
        case Status::kAccumulating:
            loc_->SetState(loclite::kInitializing);
            break;
        case Status::kFailed:
            loc_->ResetWatchdog();
            break;
        case Status::kGaveUp:
            loc_->ResetWatchdog();
            loc_->SetState(loclite::kWaitForInitialPose);
            break;
        case Status::kAccepted: {
            loc_->ResetWatchdog();
            // The filter re-initializes its IMU on the next scans (device still
            // static) and starts at the validated pose.
            Pose pose;
            pose.rotation = relocalizer_->Result().block<3, 3>(0, 0);
            pose.translation = relocalizer_->Result().block<3, 1>(0, 3);
            pipeline_->resetOdometry();
            pipeline_->setInitialPose(pose);
            tracking_ = true;
            loc_->SetState(loclite::kGood);
            break;
        }
    }
}

void LocalizationNode::track(const ScanBuffer::BufferedScan &scan, const std::vector<ImuSample> &imu) {
    const auto result = pipeline_->registerScan(scan.scan, scan.begin_time, scan.end_time, imu);
    if (!result.valid) return;  // IMU still initializing, or too few points in the scan

    const int matched = result.plane_matches + result.point_matches;
    if (result.update_rejected || matched < 10 || !result.pose.rotation.allFinite() ||
        !result.pose.translation.allFinite() || !result.velocity.allFinite()) {
        RCLCPP_WARN(get_logger(), "[Track] lost: %d matched points%s", matched,
                    result.update_rejected ? ", update rejected" : "");
        loc_->SetState(loclite::kLost);
        return;
    }
    loc_->SetState(loclite::kGood);
    Eigen::Matrix4d T_map_imu = Eigen::Matrix4d::Identity();
    T_map_imu.block<3, 3>(0, 0) = result.pose.rotation;
    T_map_imu.block<3, 1>(0, 3) = result.pose.translation;
    // GenZ-LIO keeps the velocity in the world (= map) frame.
    loc_->Publish(result.timestamp, T_map_imu, T_imu_lidar_, result.velocity);
}

}  // namespace ros_wrapper
}  // namespace genz_lio
