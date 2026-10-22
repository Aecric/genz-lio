#include "loclite/relocalizer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace loclite {
namespace {
constexpr int kInitFrames = 10;
constexpr int kInitImuSamples = 20;
}  // namespace

Relocalizer::Relocalizer(const RelocOptions &options) : options_(options) {}

bool Relocalizer::SetMap(const CloudPtr &map) {
    ndt_.reset(new pcl::NormalDistributionsTransform<PointType, PointType>());
    ndt_->setTransformationEpsilon(1e-4);
    ndt_->setMaximumIterations(25);
    ndt_->setResolution(1.0);
    ndt_->setInputTarget(map);
    genz_ = std::make_unique<GenzRefiner>(options_.genz);
    fc_ = std::make_unique<FcGate>(options_.fc);
    if (!genz_->SetTarget(map) || !fc_->SetTarget(map)) return false;
    kiss_.reset();
    if (options_.cold_start.enable) {
        kiss_ = std::make_unique<KissGlobalMatcher>(options_.kiss);
        if (!kiss_->SetTarget(map)) kiss_.reset();
    }
    return true;
}

const char *Relocalizer::ColdStartMode() const {
    if (!options_.cold_start.enable) return "off";
    if (options_.origin_prior.enable) return kiss_ ? "origin prior + KISS" : "origin prior";
    return kiss_ ? "KISS" : "off";
}

void Relocalizer::ClearWindow() {
    window_->clear();
    frames_ = 0;
    imu_count_ = 0;
    mean_acc_.setZero();
    mean_gyr_.setZero();
}

void Relocalizer::Reset() {
    ClearWindow();
    manual_ = false;
    cold_start_attempts_ = 0;
    announced_ = false;
}

void Relocalizer::RequestManual(const Eigen::Matrix4d &T_map_imu) {
    ClearWindow();
    manual_ = true;
    manual_guess_ = T_map_imu;
}

bool Relocalizer::ColdStartArmed() const {
    const int max_attempts = options_.cold_start.max_attempts;
    return options_.cold_start.enable && (kiss_ || options_.origin_prior.enable) &&
           (max_attempts == 0 || cold_start_attempts_ < max_attempts);
}

Relocalizer::Status Relocalizer::AddFrame(const Cloud &scan_imu,
                                          const std::vector<Eigen::Vector3d> &acc,
                                          const std::vector<Eigen::Vector3d> &gyr) {
    if (!manual_) {
        if (!ColdStartArmed()) return Status::kIdle;
        if (!announced_) {
            LOCLITE_INFO("[Reloc] cold start: keep still, accumulating");
            announced_ = true;
        }
    }

    if (frames_ < kInitFrames) *window_ += scan_imu;
    ++frames_;
    for (std::size_t i = 0; i < acc.size() && i < gyr.size(); ++i) {
        ++imu_count_;
        mean_gyr_ += (gyr[i] - mean_gyr_) / imu_count_;
        mean_acc_ += (acc[i] - mean_acc_) / imu_count_;
    }
    if (imu_count_ < kInitImuSamples || frames_ < kInitFrames) return Status::kAccumulating;

    LOCLITE_INFO("[Reloc] INIT start... obs_data size: " << window_->size());
    if (!mean_acc_.allFinite() || mean_acc_.norm() < 1e-3) return Status::kAccumulating;

    // Rotation that levels the IMU frame, with its yaw removed.
    const Eigen::Vector3d gravity = -mean_acc_.normalized();
    const Eigen::Matrix3d init_rot =
        Eigen::Quaterniond::FromTwoVectors(gravity, Eigen::Vector3d(0, 0, -1)).toRotationMatrix();
    const Eigen::Vector3d n = init_rot.col(0);
    const double yaw = std::atan2(n(1), n(0));
    const Eigen::Matrix3d level_rot =
        Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * init_rot;

    Eigen::Matrix4d T_map_imu;
    if (!AlignToMap(manual_, level_rot, T_map_imu)) {
        ClearWindow();
        if (!manual_ && !ColdStartArmed()) {
            LOCLITE_WARN("[Reloc] cold start gave up after " << cold_start_attempts_
                                                             << " attempts; waiting for /initialpose");
            return Status::kGaveUp;
        }
        return Status::kFailed;
    }
    result_ = T_map_imu;
    LOCLITE_INFO("[Reloc] T_map_imu:\n" << result_);
    return Status::kAccepted;
}

bool Relocalizer::AlignToMap(bool manual, const Eigen::Matrix3d &level_rot,
                             Eigen::Matrix4d &T_map_imu) {
    const auto start = std::chrono::steady_clock::now();
    auto elapsed = [&start]() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - start)
            .count();
    };

    const CloudPtr &src_imu = window_;
    CloudPtr query(new Cloud());
    {
        const float leaf = static_cast<float>(options_.genz.source_voxel);
        pcl::VoxelGrid<PointType> grid;
        grid.setLeafSize(leaf, leaf, leaf);
        grid.setInputCloud(src_imu);
        grid.filter(*query);
    }

    Eigen::Matrix4d T;
    Selection selection = Selection::kNone;
    if (manual) {
        selection = SelectCandidate(query, {manual_guess_}, options_.fc.max_delta_trans_manual,
                                    options_.fc.max_delta_rot_deg_manual, 1, "initialpose", T);
    } else {
        ++cold_start_attempts_;
        Eigen::Matrix4d T_level_imu = Eigen::Matrix4d::Identity();
        T_level_imu.block<3, 3>(0, 0) = level_rot;

        const auto &origin = options_.origin_prior;
        if (origin.enable) {
            std::vector<Eigen::Matrix4d> guesses;
            for (double yaw = 0; yaw < 360.0 - 1e-6; yaw += origin.yaw_step_deg) {
                Eigen::Matrix4d T_map_level = Eigen::Matrix4d::Identity();
                T_map_level.block<3, 3>(0, 0) =
                    Eigen::AngleAxisd(yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                T_map_level.block<3, 1>(0, 3) = Eigen::Vector3d(origin.x, origin.y, origin.z);
                guesses.push_back(T_map_level * T_level_imu);
            }
            selection = SelectCandidate(query, guesses, origin.max_delta_trans,
                                        origin.yaw_step_deg / 2 + 10.0, 1, "origin", T);
        }
        // An ambiguous scene stays ambiguous for KISS: retry with the next window.
        if (selection == Selection::kNone && kiss_) {
            CloudPtr level(new Cloud());
            pcl::transformPointCloud(*src_imu, *level, T_level_imu.cast<float>());
            std::vector<Eigen::Matrix4d> guesses;
            for (const auto &kiss : kiss_->Match(level))
                if (kiss.valid) guesses.push_back(kiss.T_map_query * T_level_imu);
            selection = SelectCandidate(
                query, guesses, options_.fc.max_delta_trans_kiss, options_.fc.max_delta_rot_deg_kiss,
                static_cast<std::size_t>(options_.kiss.min_agreeing_candidates), "kiss", T);
        }
    }

    if (selection != Selection::kAccepted) {
        LOCLITE_WARN("[FC-Init] " << (manual ? "initialpose" : "cold start attempt ")
                                  << (manual ? std::string() : std::to_string(cold_start_attempts_))
                                  << " failed (" << elapsed() << " ms)");
        return false;
    }
    LOCLITE_INFO("[FC-Init] accepted (" << elapsed() << " ms)");
    T_map_imu = T;
    return true;
}

Relocalizer::Selection Relocalizer::SelectCandidate(const CloudPtr &query,
                                                    const std::vector<Eigen::Matrix4d> &guesses,
                                                    double max_delta_trans, double max_delta_rot_deg,
                                                    std::size_t min_passed, const char *label,
                                                    Eigen::Matrix4d &T_map_imu) {
    struct Passed {
        Eigen::Matrix4d T;
        FcGate::Report report;
    };
    std::vector<Passed> passed;
    for (std::size_t i = 0; i < guesses.size(); ++i) {
        // NDT widens the basin before GenZ-ICP: hand-placed, origin and KISS
        // guesses can be a metre or more and 10-20 degrees off.
        Eigen::Matrix4d refined = guesses[i];
        CloudPtr unused(new Cloud());
        ndt_->setInputSource(query);
        ndt_->align(*unused, guesses[i].cast<float>());
        const Eigen::Matrix4d coarse = ndt_->getFinalTransformation().cast<double>();
        if (ndt_->hasConverged() && coarse.allFinite()) refined = coarse;
        FcGate::Report report;
        if (genz_->Refine(query, refined, refined))
            report = fc_->Check(query, guesses[i], refined, max_delta_trans, max_delta_rot_deg);
        const Eigen::Vector3d t = refined.block<3, 1>(0, 3);
        const auto line = std::string("[FC-Init] ") + label + " candidate " + std::to_string(i + 1) +
                          "/" + std::to_string(guesses.size()) + (report.pass ? " PASS" : " REJECT");
        if (report.pass) {
            LOCLITE_INFO(line << " t=[" << t.transpose() << "]: " << report.Describe());
            passed.push_back({refined, report});
        } else {
            LOCLITE_WARN(line << " t=[" << t.transpose() << "]: " << report.Describe());
        }
    }
    if (passed.empty()) return Selection::kNone;

    const auto best = std::max_element(passed.begin(), passed.end(), [](const auto &a, const auto &b) {
        return a.report.wall_inlier_ratio < b.report.wall_inlier_ratio;
    });
    for (const auto &other : passed) {
        const double dt = (other.T.block<3, 1>(0, 3) - best->T.block<3, 1>(0, 3)).norm();
        const double dr = Eigen::AngleAxisd(Eigen::Matrix3d(best->T.block<3, 3>(0, 0).transpose() *
                                                            other.T.block<3, 3>(0, 0)))
                              .angle() *
                          180.0 / M_PI;
        if (dt > options_.fc.ambiguity_trans || dr > options_.fc.ambiguity_rot_deg) {
            LOCLITE_WARN("[FC-Init] " << label << ": ambiguous, passing candidates " << dt << " m / "
                                      << dr << " deg apart");
            return Selection::kAmbiguous;
        }
    }
    if (passed.size() < min_passed) {
        LOCLITE_WARN("[FC-Init] " << label << ": " << passed.size() << " passing candidate(s), need "
                                  << min_passed << " that agree");
        return Selection::kNone;
    }
    T_map_imu = best->T;
    return Selection::kAccepted;
}

}  // namespace loclite
