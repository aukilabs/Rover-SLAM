/**
 * Thin overlay pose graph for slam_server (hybrid prior-map initiative S4).
 * Stores live frame nodes, portal hard factors, and per-map live↔domain Sim(3) pins.
 */

#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <sophus/se3.hpp>

#include "../../include/third_party/nlohmann/json.hpp"

namespace overlay {

using json = nlohmann::json;

/** Lightweight live SLAM frame / keyframe node in the overlay graph. */
struct LiveNode {
    uint64_t frame_id = 0;
    double timestamp = 0.0;
    bool is_keyframe = false;
    Sophus::SE3f Twc_slam;
};

/** Portal absolute pin factor (hard constraint from QR PnP → domain pose). */
struct PortalHardFactor {
    uint64_t frame_id = 0;
    double timestamp = 0.0;
    std::string short_id;
    Eigen::Vector3f p_slam = Eigen::Vector3f::Zero();
    Eigen::Vector3f p_domain = Eigen::Vector3f::Zero();
    Eigen::Matrix3f R_domain_slam = Eigen::Matrix3f::Identity();
    float reproj_px = 0.0f;
    float weight = 0.0f;
};

/** Verified prior match soft factor (gated LightGlue + track reprojection). */
struct PriorSoftEdgeFactor {
    uint64_t frame_id = 0;
    double timestamp = 0.0;
    int prior_image_id = 0;
    int inlier_count = 0;
    float mean_reproj_px = 0.0f;
    float weight = 0.0f;
};

/** Per-map live↔domain Sim(3) pin established by portal alignment. */
struct MapSim3Pin {
    bool pinned = false;
    Eigen::Matrix3f R_domain_slam = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t_domain_slam = Eigen::Vector3f::Zero();
    float scale = 1.0f;
    std::string scale_source = "none";
    std::string last_portal_short_id;
    float last_reproj_px = 0.0f;
    size_t portal_factor_count = 0;
};

struct NavPoseResult {
    bool valid = false;
    Sophus::SE3f Twc;
    const char* pose_source = "slam";
};

class OverlayPoseGraph {
public:
    void recordLivePose(unsigned long map_id,
                        uint64_t frame_id,
                        double timestamp,
                        const Sophus::SE3f& Twc_slam,
                        bool is_keyframe);

    void addPortalHardFactor(unsigned long map_id, const PortalHardFactor& factor);

    void addPriorSoftEdgeFactor(unsigned long map_id, const PriorSoftEdgeFactor& factor);

    uint64_t priorSoftEdgesAccepted() const;

    void setMapPin(unsigned long map_id, const MapSim3Pin& pin);

    bool isMapPinned(unsigned long map_id) const;

    MapSim3Pin mapPin(unsigned long map_id) const;

    std::vector<unsigned long> pinnedMapIds() const;

    Sophus::SE3f applyPinToTwc(unsigned long map_id, const Sophus::SE3f& Twc_slam) const;

    NavPoseResult navPoseFromSlam(unsigned long map_id, const Sophus::SE3f& Twc_slam) const;

    json statusJson(unsigned long active_map_id) const;

private:
    static constexpr size_t kMaxLiveNodesPerMap = 2048;
    static constexpr size_t kMaxPortalFactorsPerMap = 64;
    static constexpr size_t kMaxPriorSoftFactorsPerMap = 128;

    mutable std::mutex mutex_;
    std::unordered_map<unsigned long, std::vector<LiveNode>> live_nodes_;
    std::unordered_map<unsigned long, std::vector<PortalHardFactor>> portal_factors_;
    std::unordered_map<unsigned long, std::vector<PriorSoftEdgeFactor>> prior_soft_factors_;
    std::unordered_map<unsigned long, MapSim3Pin> map_pins_;
    uint64_t prior_soft_edges_accepted_ = 0;
    PriorSoftEdgeFactor last_prior_soft_edge_;
};

}  // namespace overlay
