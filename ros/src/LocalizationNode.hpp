// This file is part of GenZ-LIO, released under the GNU GPL v2.
//
// Fixed-map pure localization (ROS 2 only): GenZ-LIO tracks an existing
// global.pcd without updating or saving it. Command line, topics, TF and the
// cold-start relocalization follow Super-LIO's run_loclite_online; see
// ros/localization.md.
#pragma once

#include "RosCompat.hpp"
#include "core/GenZLIO.hpp"
#include "core/ScanBuffer.hpp"
#include "loclite/loc_interface.h"
#include "loclite/relocalizer.h"

#ifdef GENZ_LIO_WITH_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace genz_lio {
namespace ros_wrapper {

class LocalizationNode : public rclcpp::Node {
public:
    /// The GenZ-LIO YAML comes from the `config_path` parameter. Throws
    /// std::runtime_error on a missing/empty map or invalid parameters.
    explicit LocalizationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /// The scan processing timer; spin it on its own executor.
    rclcpp::CallbackGroup::SharedPtr processingGroup() const { return processing_group_; }

private:
    void readNodeOptions(const std::string &config_path);
    void createSensorSubscriptions();
    void destroySensorSubscriptions();

    void onImu(const Imu &msg);
    void onCloud(const PointCloud2 &msg);
#ifdef GENZ_LIO_WITH_LIVOX
    void onLivox(const livox_ros_driver2::msg::CustomMsg &msg);
#endif

    /// Drains whatever complete scan/IMU pairs the buffer holds.
    void spinOnce();
    /// Super-LIO SuperLIOReLoc::Reset(): drop the track, keep the fixed map.
    void resetLocalization();
    void initialize(const ScanBuffer::BufferedScan &scan, const std::vector<ImuSample> &imu);
    void track(const ScanBuffer::BufferedScan &scan, const std::vector<ImuSample> &imu);

    Config config_;
    std::unique_ptr<GenZLIO> pipeline_;
    ScanBuffer buffer_;
    std::unique_ptr<Preprocessor> preprocessor_;
    std::uint64_t pipeline_epoch_ = 0;
    /// Set by ~/set_active: start a fresh session; the buffer epoch it bumped
    /// is then adopted without a second reset (which would drop /initialpose).
    std::atomic<bool> restart_{false};
    bool epoch_synced_ = true;
    Eigen::Matrix4d T_imu_lidar_ = Eigen::Matrix4d::Identity();

    std::string lidar_topic_ = "/points";
    std::string imu_topic_ = "/imu";
    double time_offset_ = 0.0;
    bool qos_reliable_ = true;
    int qos_depth_ = 200;

    std::unique_ptr<loclite::LocInterface> loc_;
    std::unique_ptr<loclite::Relocalizer> relocalizer_;
    bool tracking_ = false;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
#ifdef GENZ_LIO_WITH_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
#endif
    rclcpp::CallbackGroup::SharedPtr processing_group_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ros_wrapper
}  // namespace genz_lio
