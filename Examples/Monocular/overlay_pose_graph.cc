#include "overlay_pose_graph.h"

#include <algorithm>

namespace overlay {

void OverlayPoseGraph::recordLivePose(unsigned long map_id,
                                      uint64_t frame_id,
                                      double timestamp,
                                      const Sophus::SE3f& Twc_slam,
                                      bool is_keyframe) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<LiveNode>& nodes = live_nodes_[map_id];
    nodes.push_back({frame_id, timestamp, is_keyframe, Twc_slam});
    if (nodes.size() > kMaxLiveNodesPerMap) {
        const size_t overflow = nodes.size() - kMaxLiveNodesPerMap;
        nodes.erase(nodes.begin(), nodes.begin() + static_cast<std::ptrdiff_t>(overflow));
    }
}

void OverlayPoseGraph::addPortalHardFactor(unsigned long map_id, const PortalHardFactor& factor) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<PortalHardFactor>& factors = portal_factors_[map_id];
    factors.push_back(factor);
    if (factors.size() > kMaxPortalFactorsPerMap) {
        const size_t overflow = factors.size() - kMaxPortalFactorsPerMap;
        factors.erase(factors.begin(), factors.begin() + static_cast<std::ptrdiff_t>(overflow));
    }
}

void OverlayPoseGraph::addPriorSoftEdgeFactor(unsigned long map_id,
                                              const PriorSoftEdgeFactor& factor) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<PriorSoftEdgeFactor>& factors = prior_soft_factors_[map_id];
    factors.push_back(factor);
    if (factors.size() > kMaxPriorSoftFactorsPerMap) {
        const size_t overflow = factors.size() - kMaxPriorSoftFactorsPerMap;
        factors.erase(factors.begin(), factors.begin() + static_cast<std::ptrdiff_t>(overflow));
    }
    prior_soft_edges_accepted_++;
    last_prior_soft_edge_ = factor;
}

uint64_t OverlayPoseGraph::priorSoftEdgesAccepted() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return prior_soft_edges_accepted_;
}

void OverlayPoseGraph::setMapPin(unsigned long map_id, const MapSim3Pin& pin) {
    std::lock_guard<std::mutex> lock(mutex_);
    map_pins_[map_id] = pin;
}

bool OverlayPoseGraph::isMapPinned(unsigned long map_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = map_pins_.find(map_id);
    return it != map_pins_.end() && it->second.pinned;
}

MapSim3Pin OverlayPoseGraph::mapPin(unsigned long map_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = map_pins_.find(map_id);
    if (it == map_pins_.end())
        return MapSim3Pin{};
    return it->second;
}

std::vector<unsigned long> OverlayPoseGraph::pinnedMapIds() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<unsigned long> ids;
    for (const auto& kv : map_pins_) {
        if (kv.second.pinned)
            ids.push_back(kv.first);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

Sophus::SE3f OverlayPoseGraph::applyPinToTwc(unsigned long map_id,
                                             const Sophus::SE3f& Twc_slam) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = map_pins_.find(map_id);
    if (it == map_pins_.end() || !it->second.pinned)
        return Twc_slam;

    const MapSim3Pin& pin = it->second;
    const Eigen::Matrix3f R = pin.R_domain_slam * Twc_slam.so3().matrix();
    const Eigen::Vector3f t =
        pin.scale * (pin.R_domain_slam * Twc_slam.translation()) + pin.t_domain_slam;
    return Sophus::SE3f(R, t);
}

NavPoseResult OverlayPoseGraph::navPoseFromSlam(unsigned long map_id,
                                                const Sophus::SE3f& Twc_slam) const {
    NavPoseResult out;
    out.valid = true;
    out.Twc = Twc_slam;
    out.pose_source = "slam";

    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = map_pins_.find(map_id);
    if (it == map_pins_.end() || !it->second.pinned)
        return out;

    const MapSim3Pin& pin = it->second;
    const Eigen::Matrix3f R = pin.R_domain_slam * Twc_slam.so3().matrix();
    const Eigen::Vector3f t =
        pin.scale * (pin.R_domain_slam * Twc_slam.translation()) + pin.t_domain_slam;
    out.Twc = Sophus::SE3f(R, t);
    out.pose_source = "overlay";
    return out;
}

json OverlayPoseGraph::statusJson(unsigned long active_map_id) const {
    std::lock_guard<std::mutex> lock(mutex_);

    json out;
    out["enabled"] = true;
    out["active_map_id"] = active_map_id;

    json pinned_ids = json::array();
    for (const auto& kv : map_pins_) {
        if (kv.second.pinned)
            pinned_ids.push_back(kv.first);
    }
    out["pinned_map_ids"] = pinned_ids;

    const auto active_pin_it = map_pins_.find(active_map_id);
    out["active_map_pinned"] =
        active_pin_it != map_pins_.end() && active_pin_it->second.pinned;

    out["prior_soft_edges_accepted"] = prior_soft_edges_accepted_;
    if (prior_soft_edges_accepted_ > 0) {
        json last_edge;
        last_edge["frame_id"] = last_prior_soft_edge_.frame_id;
        last_edge["prior_image_id"] = last_prior_soft_edge_.prior_image_id;
        last_edge["inlier_count"] = last_prior_soft_edge_.inlier_count;
        last_edge["mean_reproj_px"] = last_prior_soft_edge_.mean_reproj_px;
        last_edge["weight"] = last_prior_soft_edge_.weight;
        out["prior_soft_edges_last"] = last_edge;
    }

    json maps = json::array();
    for (const auto& kv : map_pins_) {
        const unsigned long map_id = kv.first;
        const MapSim3Pin& pin = kv.second;
        json row;
        row["map_id"] = map_id;
        row["pinned"] = pin.pinned;
        row["live_node_count"] = live_nodes_.count(map_id) ? live_nodes_.at(map_id).size() : 0;
        row["portal_factor_count"] = pin.portal_factor_count;
        row["prior_soft_edge_count"] =
            prior_soft_factors_.count(map_id) ? prior_soft_factors_.at(map_id).size() : 0;
        if (pin.pinned) {
            row["scale"] = pin.scale;
            row["scale_source"] = pin.scale_source;
            row["last_portal_short_id"] = pin.last_portal_short_id;
            row["last_reprojection_error_px"] = pin.last_reproj_px;
        }
        maps.push_back(row);
    }
    out["maps"] = maps;
    return out;
}

}  // namespace overlay
