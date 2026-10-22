// Fixed-map localization (loclite), ported from Super-LIO's run_loclite_online.
#pragma once

#include <memory>

#include <Eigen/Core>

#include "loclite/common.h"
#include "loclite/reloc_options.h"

namespace genz_icp {
struct VoxelHashMap;
}

namespace loclite {

/// GenZ-ICP (adaptive point-to-plane / point-to-point) refinement of a
/// candidate pose against a static voxel hash of the fixed map.
class GenzRefiner {
public:
    explicit GenzRefiner(const RelocOptions::Genz &options);
    ~GenzRefiner();

    bool SetTarget(const CloudPtr &map);
    bool TargetReady() const;

    /// source is in the query frame; returns the refined T_map_query.
    /// Coarse then fine correspondence radius. false on degenerate input.
    bool Refine(const CloudPtr &source, const Eigen::Matrix4d &guess,
                Eigen::Matrix4d &refined) const;

private:
    RelocOptions::Genz options_;
    std::unique_ptr<genz_icp::VoxelHashMap> map_;
};

}  // namespace loclite
