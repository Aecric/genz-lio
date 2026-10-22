// Fixed-map localization (loclite), ported from Super-LIO's run_loclite_online.
#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <pcl/registration/ndt.h>

#include "loclite/common.h"
#include "loclite/fc_gate.h"
#include "loclite/genz_refiner.h"
#include "loclite/kiss_global_matcher.h"
#include "loclite/reloc_options.h"

namespace loclite {

/// Initialization against the fixed map: Super-LIO SuperLIOReLoc::kf_init +
/// alignToMap + selectCandidate, independent of the odometry core.
///
/// While the device is static, AddFrame() accumulates 10 scans (in the IMU
/// frame) and at least 20 IMU samples, levels the window with the mean
/// acceleration and verifies T_map_imu candidates:
///   /initialpose : the given guess
///   cold start   : map-origin prior with yaw hypotheses, then KISS-Matcher
/// each one NDT coarse -> GenZ-ICP -> FC.
class Relocalizer {
public:
    enum class Status {
        kIdle,          ///< cold start disarmed and no /initialpose: nothing to do
        kAccumulating,  ///< collecting the static window
        kFailed,        ///< window rejected; retrying with the next one
        kGaveUp,        ///< cold start out of attempts; waiting for /initialpose
        kAccepted,      ///< Result() holds the validated T_map_imu
    };

    explicit Relocalizer(const RelocOptions &options);

    /// Builds the NDT / GenZ-ICP / FC (and, with cold start, KISS) targets.
    bool SetMap(const CloudPtr &map);
    /// Human-readable cold-start mode after SetMap(), for the startup log.
    const char *ColdStartMode() const;

    /// Super-LIO Reset(): forget the window, the /initialpose and the attempt count.
    void Reset();
    /// /initialpose, already converted to T_map_imu. Restarts the window.
    void RequestManual(const Eigen::Matrix4d &T_map_imu);
    bool Manual() const { return manual_; }
    bool ColdStartArmed() const;

    /// One scan in the IMU frame plus the IMU samples (acc, gyro) that came with it.
    Status AddFrame(const Cloud &scan_imu, const std::vector<Eigen::Vector3d> &acc,
                    const std::vector<Eigen::Vector3d> &gyr);

    /// Valid after kAccepted.
    const Eigen::Matrix4d &Result() const { return result_; }
    const Eigen::Vector3d &MeanAcc() const { return mean_acc_; }
    const Eigen::Vector3d &MeanGyr() const { return mean_gyr_; }
    int Attempts() const { return cold_start_attempts_; }

private:
    enum class Selection { kAccepted, kNone, kAmbiguous };

    void ClearWindow();
    /// level_rot rotates the IMU frame into the gravity-levelled frame.
    bool AlignToMap(bool manual, const Eigen::Matrix3d &level_rot, Eigen::Matrix4d &T_map_imu);
    /// Verify every T_map_imu guess (NDT coarse -> GenZ-ICP -> FC) and pick the
    /// best passing one. kNone also when fewer than min_passed pass.
    Selection SelectCandidate(const CloudPtr &query, const std::vector<Eigen::Matrix4d> &guesses,
                              double max_delta_trans, double max_delta_rot_deg,
                              std::size_t min_passed, const char *label,
                              Eigen::Matrix4d &T_map_imu);

    RelocOptions options_;
    std::unique_ptr<KissGlobalMatcher> kiss_;
    std::unique_ptr<GenzRefiner> genz_;
    std::unique_ptr<FcGate> fc_;
    pcl::NormalDistributionsTransform<PointType, PointType>::Ptr ndt_;

    CloudPtr window_{new Cloud()};
    int frames_ = 0;
    int imu_count_ = 0;
    Eigen::Vector3d mean_acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gyr_ = Eigen::Vector3d::Zero();

    bool manual_ = false;
    Eigen::Matrix4d manual_guess_ = Eigen::Matrix4d::Identity();
    int cold_start_attempts_ = 0;
    bool announced_ = false;
    Eigen::Matrix4d result_ = Eigen::Matrix4d::Identity();
};

}  // namespace loclite
