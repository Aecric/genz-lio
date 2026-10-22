// Fixed-map localization (loclite), ported from Super-LIO's run_loclite_online.
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/logging.hpp>

namespace loclite {

using PointType = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointType>;
using CloudPtr = Cloud::Ptr;

/// loc_state values, same numbering as hikari_loclite / Super-LIO.
enum LocState : int {
    kUninitialized = 0,
    kInitializing = 1,
    kGood = 2,
    kDegraded = 3,
    kLost = 4,
    kWaitForInitialPose = 5,
};

inline rclcpp::Logger Logger() { return rclcpp::get_logger("loclite"); }

}  // namespace loclite

#define LOCLITE_INFO(stream) RCLCPP_INFO_STREAM(::loclite::Logger(), stream)
#define LOCLITE_WARN(stream) RCLCPP_WARN_STREAM(::loclite::Logger(), stream)
#define LOCLITE_ERROR(stream) RCLCPP_ERROR_STREAM(::loclite::Logger(), stream)
