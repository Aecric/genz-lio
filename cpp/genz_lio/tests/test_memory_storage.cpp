// This file is part of GenZ-LIO, released under the GNU GPL v2.
#include "core/VoxelMap.hpp"
#include "core/VoxelPrunedSearch.hpp"
#include "Check.hpp"

#include <cstdio>

using namespace genz_lio;

PointWithCov sample(int i) {
    PointWithCov p;
    p.point = p.point_world = Vec3(.1 + .1 * (i % 3), .1 + .1 * ((i / 3) % 2), 0.);
    p.cov = Mat3::Identity() * 1e-4;
    p.cov_lidar = Mat3::Identity() * 2e-4;
    return p;
}

void frozenPlaneKeepsMatches(bool hybrid) {
    OctoTree cell(3, 0, {5, 5, 5}, 12, 12, .001f, 1., 128, 4, hybrid);
    cell.voxel_center_[0] = cell.voxel_center_[1] = cell.voxel_center_[2] = .5;
    cell.quater_length_ = .25;
    GENZ_CHECK(cell.plane_ptr_ == nullptr);
    GENZ_CHECK(!cell.plane().is_plane);
    for (int i = 0; i < 3; ++i) cell.update(sample(i));
    GENZ_CHECK(cell.plane_ptr_ == nullptr);
    for (int i = 3; i < 12; ++i) cell.update(sample(i));
    GENZ_CHECK(cell.plane_ptr_ && cell.plane().is_plane && !cell.update_enable_);
    const auto center = cell.plane().center;
    const auto cov = cell.plane().plane_cov;
    // Hybrid cells have candidates; plane-only cells retain visualization samples.
    if (hybrid) GENZ_CHECK(cell.temp_points_.empty());
    else GENZ_CHECK(!cell.temp_points_.empty());
    for (int i = 12; i < 200; ++i) cell.update(sample(i));
    GENZ_CHECK((cell.plane().center - center).norm() == 0);
    GENZ_CHECK((cell.plane().plane_cov - cov).norm() == 0);
    if (hybrid) {
        auto query = sample(0);
        query.point_world.x() += .001;
        bool found = false;
        double nearest = 1;
        int count = 0;
        PointToPointMatch match;
        buildPointResidual(query, &cell, 1., found, nearest, match, count);
        GENZ_CHECK(found);
        GENZ_CHECK((match.point_cov - sample(0).cov).norm() == 0);
        GENZ_CHECK((match.cov_lidar - query.cov_lidar).norm() == 0);
    }
}

void subdivideAndPrune() {
    for (int repeat = 0; repeat < 20; ++repeat) {
        VoxelMapType map;
        std::vector<PointWithCov> points;
        for (int x = 0; x < 2; ++x) {
            for (int y = 0; y < 2; ++y) {
                for (int z = 0; z < 2; ++z) {
                    auto p = sample(0);
                    p.point = Vec3(.1 + .7 * x, .1 + .7 * y, .1 + .7 * z);
                    points.push_back(p);
                }
            }
        }
        buildVoxelMap(points, 1., 4, {5, 5, 5, 5}, 100, 100, 1e-5f, 128, 4, true, map);
        GENZ_CHECK(map.size() == 1);
        const auto *root = map.begin()->second;
        GENZ_CHECK(!root->plane().is_plane);
        for (const auto *child : root->leaves_) {
            GENZ_CHECK(child != nullptr);
            GENZ_CHECK(child->layer_point_size_.storageIdentity() ==
                       root->layer_point_size_.storageIdentity());
            GENZ_CHECK(child->plane_ptr_ == nullptr);
        }
        removePointsFarFromLocation(Vec3(1000, 1000, 1000), map, 250.);
        GENZ_CHECK(map.empty());
    }
}

int main() {
    frozenPlaneKeepsMatches(false);
    frozenPlaneKeepsMatches(true);
    subdivideAndPrune();
    std::puts("memory storage: frozen matches, lazy planes, shared thresholds and pruning passed");
}
