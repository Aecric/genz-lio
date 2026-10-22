// Fixed-map localization (loclite), ported from Super-LIO's run_loclite_online.
#pragma once

#include <string>

#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>

#include "loclite/common.h"
#include "loclite/reloc_options.h"

namespace loclite {

/// FC: geometric false-positive barrier for an initialization candidate, after
/// lightning-lm's [FC-Init]/[FC-GeoGate]. A candidate is accepted only if the
/// refined pose stays near its coarse guess and the query cloud lands on the
/// map densely, precisely and from enough directions.
class FcGate {
public:
    struct Report {
        bool pass = false;
        double inlier_ratio = 0, wall_inlier_ratio = 0, coverage = 0, rmse = 0;
        double delta_trans = 0, delta_rot_deg = 0;
        std::string Describe() const;
    };

    explicit FcGate(const RelocOptions::Fc &options) : options_(options) {}

    bool SetTarget(const CloudPtr &map);
    bool TargetReady() const { return ready_; }

    /// source in the query frame; guess / refined are T_map_query. The refined
    /// pose may move at most max_delta_trans / max_delta_rot_deg from the guess.
    Report Check(const CloudPtr &source, const Eigen::Matrix4d &guess,
                 const Eigen::Matrix4d &refined, double max_delta_trans,
                 double max_delta_rot_deg) const;

private:
    RelocOptions::Fc options_;
    pcl::KdTreeFLANN<PointType> tree_;
    bool ready_ = false;
};

}  // namespace loclite
