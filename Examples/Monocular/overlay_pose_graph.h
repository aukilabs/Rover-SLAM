/**
 * Overlay pose graph for slam_server (hybrid prior-map initiative S9 single Sim(3)/map).
 * One Sim(3) vertex per atlas map at SLAM world origin; portal corners + prior soft absolutes.
 * No Atlas write-back — overlay optimizes domain nav poses only.
 */

#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>

#include "../../include/third_party/nlohmann/json.hpp"

namespace overlay {

using json = nlohmann::json;

/** Sentinel vertex id — one Sim(3) per map at SLAM world origin; not an atlas keyframe id. */
constexpr unsigned long kMapOriginVertexId = std::numeric_limits<unsigned long>::max();

/** One soft portal corner 3D→2D residual (high weight). */
struct PortalCornerFactor {
    uint64_t frame_id = 0;
    uint64_t vertex_keyframe_id = 0;
    double timestamp = 0.0;
    std::string short_id;
    int corner_index = 0;
    Eigen::Vector3f p_domain = Eigen::Vector3f::Zero();
    Eigen::Vector2f pixel = Eigen::Vector2f::Zero();
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    /** Observation frame SLAM pose Tcw_slam (world → camera). */
    Sophus::SE3f Tcr = Sophus::SE3f();
    float weight = 0.0f;
};

/** One 3D→2D prior-map observation bundled into a soft edge factor. */
struct PriorSoftObservation {
    Eigen::Vector3f p_domain = Eigen::Vector3f::Zero();
    Eigen::Vector2f pixel = Eigen::Vector2f::Zero();
};

/** Verified prior match soft factor (gated LightGlue + track reprojection). */
struct PriorSoftEdgeFactor {
    uint64_t frame_id = 0;
    uint64_t vertex_keyframe_id = 0;
    double timestamp = 0.0;
    int prior_image_id = 0;
    int inlier_count = 0;
    float mean_reproj_px = 0.0f;
    float weight = 0.0f;
    /** Observation frame SLAM pose Tcw_slam (world → camera). */
    Sophus::SE3f Tcr = Sophus::SE3f();
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    std::vector<PriorSoftObservation> observations;
};

/** Summary pin + diagnostics (status JSON / viewer); not the optimizer state. */
struct MapSim3Pin {
    bool pinned = false;
    bool seeded = false;
    Eigen::Matrix3f R_domain_slam = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t_domain_slam = Eigen::Vector3f::Zero();
    float scale = 1.0f;
    float seed_scale = 1.0f;
    std::string scale_source = "none";
    std::string last_portal_short_id;
    float last_reproj_px = 0.0f;
    size_t portal_corner_factor_count = 0;
    size_t live_relative_edge_count = 0;
    size_t optimization_passes = 0;
    float last_corner_rmse_px = 0.0f;
    float last_live_edge_rmse_m = 0.0f;
    std::string last_refine_source;
    size_t keyframe_vertex_count = 0;
    size_t frozen_keyframe_count = 0;
    bool keyframe_cap_engaged = false;
    float gauge_min_hessian_eigenvalue = 0.0f;
    std::string gauge_null_dominant_param;
    size_t dense_observation_count = 0;
    size_t dropped_unresolved_observations = 0;
};

struct NavPoseResult {
    bool valid = false;
    Sophus::SE3f Twc;
    const char* pose_source = "slam";
};

/** Input for one portal PnP sighting (seed + corner factors). */
struct PortalSightInput {
    uint64_t frame_id = 0;
    double timestamp = 0.0;
    std::string short_id;
    Sophus::SE3f Twc_slam;
    Sophus::SE3f Twc_domain;
    Eigen::Matrix3f R_domain_slam = Eigen::Matrix3f::Identity();
    std::array<Eigen::Vector3f, 4> corners_domain{};
    std::array<Eigen::Vector2f, 4> corners_pixel{};
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    float reproj_px = 0.0f;
    float physical_size_m = 0.0f;
    float metric_depth_m = 0.0f;
};

struct RefineResult {
    bool seeded_or_refined = false;
    bool published = false;
    bool portal_corners_added = false;
    MapSim3Pin pin;
    std::string status;
    std::string scale_update_status;
};

class OverlayPoseGraph {
public:
    RefineResult recordLivePose(unsigned long map_id,
                                uint64_t frame_id,
                                double timestamp,
                                const Sophus::SE3f& Twc_slam,
                                bool is_keyframe);

    void addPortalCornerFactors(unsigned long map_id, const PortalSightInput& sight);

    RefineResult addPriorSoftEdgeFactor(unsigned long map_id, const PriorSoftEdgeFactor& factor);

    void recordPriorSoftEdgeReject(const std::string& reason);

    RefineResult ingestPortalSight(unsigned long map_id, const PortalSightInput& sight);

    void clearMapAlignment(unsigned long map_id);

    /** Atlas owns domain frame; overlay Sim(3) stays identity and refine is skipped. */
    void setAtlasDomainized(unsigned long map_id, bool domainized);
    bool isAtlasDomainized(unsigned long map_id) const;

    uint64_t priorSoftEdgesAccepted() const;

    uint64_t priorSoftEdgesRejected() const;

    void setMapPin(unsigned long map_id, const MapSim3Pin& pin);

    bool isMapPinned(unsigned long map_id) const;

    MapSim3Pin mapPin(unsigned long map_id) const;

    std::vector<unsigned long> pinnedMapIds() const;

    Sophus::SE3f applyPinToTwc(unsigned long map_id, const Sophus::SE3f& Twc_slam) const;

    NavPoseResult navPoseFromSlam(unsigned long map_id, const Sophus::SE3f& Twc_slam) const;

    json statusJson(unsigned long active_map_id) const;

    /**
     * Runtime refine knobs (holdout sweeps).
     * age_tau_sec <= 0 disables age decay (weight=1).
     * Otherwise w = (1 + age_sec/tau)^(-power) with age_sec = Δframe/30.
     */
    void setRefineBalance(float portal_sight_equivalent_soft_edges,
                          float age_tau_sec,
                          float age_power);

    float portalSightEquivalentSoftEdges() const { return portal_sight_eq_; }
    float ageTauSec() const { return age_tau_sec_; }
    float agePower() const { return age_power_; }

private:
    static constexpr size_t kMaxPortalCornerFactorsPerMap = 512;
    static constexpr size_t kMaxPriorSoftFactorsPerMap = 512;
    static constexpr size_t kMaxPriorSoftObservationsPerFactor = 64;
    /** Per-corner base at reproj_px=1; 4 corners ≈ one soft edge at ~25 inliers before sight scale. */
    static constexpr float kPortalCornerBaseWeight = 2.5f;
    static constexpr float kPriorSoftBaseWeight = 2.0f;
    /** Default: one portal sight ≈ this many soft edges at kPriorSoftBaseWeight*sqrt(25). */
    static constexpr float kDefaultPortalSightEquivalentSoftEdges = 8.0f;
    /** Default age profile ≈ prior 1/(0.5+0.5*age) shape (tau=1,power=1 → 1/(1+age)). */
    static constexpr float kDefaultAgeTauSec = 1.0f;
    static constexpr float kDefaultAgePower = 1.0f;
    static constexpr size_t kLiveEdgeRefineCadence = 8;
    static constexpr float kMinPortalSightBaselineSlam = 0.12f;

    float portal_sight_eq_ = kDefaultPortalSightEquivalentSoftEdges;
    float age_tau_sec_ = kDefaultAgeTauSec;
    float age_power_ = kDefaultAgePower;

    struct KeyframeVertex {
        unsigned long keyframe_id = 0;
        Sophus::Sim3f Siw;
    };

    struct MapGraph {
        std::vector<PortalCornerFactor> portal_corners;
        std::vector<PriorSoftEdgeFactor> prior_soft_factors;
        KeyframeVertex origin_vertex;
        bool has_origin_vertex = false;
        MapSim3Pin pin;
        bool atlas_domainized = false;
        /** Max frame_id among portal corners and prior soft factors (for age weighting). */
        uint64_t latest_constraint_frame_id = 0;
    };

    MapGraph& graphFor(unsigned long map_id);

    void ensureOriginVertex(MapGraph& graph);

    bool refineMapSim3(MapGraph& graph, const char* refine_source);

    /** Mirror pin into origin_vertex for status only; pin is never derived from Siw. */
    void syncOriginVertexFromPin(MapGraph& graph);

    void updatePinCounts(MapSim3Pin& pin, const MapGraph& graph);

    mutable std::mutex mutex_;
    std::unordered_map<unsigned long, MapGraph> maps_;
    uint64_t prior_soft_edges_accepted_ = 0;
    uint64_t prior_soft_edges_rejected_ = 0;
    PriorSoftEdgeFactor last_prior_soft_edge_;
    std::string last_prior_soft_reject_reason_;
};

}  // namespace overlay
