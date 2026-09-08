/**
 * Overlay pose graph for slam_server (hybrid prior-map initiative S4′/S10).
 * Per-keyframe Sim(3) vertices, atlas relative edges, soft portal/prior absolutes.
 * No Atlas write-back — overlay optimizes domain nav poses only.
 */

#pragma once

#include <array>
#include <cstdint>
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

/** Atlas keyframe node snapshot (mirrors ORB_SLAM3::System::KeyFrameGraphSample). */
struct KeyFrameGraphSample {
    unsigned long id = 0;
    double timestamp = 0.0;
    Sophus::SE3f Tcw;
    unsigned long parent_id = 0;
    std::vector<std::pair<unsigned long, int>> covisibles;
};

/** Per-frame reference for dense pose reconstruction. */
struct FrameRefSample {
    unsigned long frame_id = 0;
    unsigned long ref_keyframe_id = 0;
    Sophus::SE3f Tcr = Sophus::SE3f();
    bool is_keyframe = false;
};

/** Cheap atlas snapshot taken each refine (fixes stale SLAM pose snapshots). */
struct AtlasSnapshot {
    std::vector<KeyFrameGraphSample> keyframes;
    std::vector<FrameRefSample> frame_refs;
};

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
    /** Non-identity when observation landed on a dense (non-keyframe) frame. */
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
    /** ORB internal frame / keyframe ids for atlas anchor (not client frame_id). */
    unsigned long orb_frame_mn_id = 0;
    unsigned long ref_keyframe_id = 0;
    Sophus::SE3f Tcr = Sophus::SE3f();
    bool has_overlay_anchor = false;
};

struct RefineResult {
    bool seeded_or_refined = false;
    bool published = false;
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
                                bool is_keyframe,
                                const AtlasSnapshot& atlas);

    void addPortalCornerFactors(unsigned long map_id,
                                const PortalSightInput& sight,
                                const AtlasSnapshot& atlas);

    RefineResult addPriorSoftEdgeFactor(unsigned long map_id,
                                        const PriorSoftEdgeFactor& factor,
                                        const AtlasSnapshot& atlas);

    void recordPriorSoftEdgeReject(const std::string& reason);

    RefineResult ingestPortalSight(unsigned long map_id,
                                   const PortalSightInput& sight,
                                   const AtlasSnapshot& atlas);

    void clearMapAlignment(unsigned long map_id);

    uint64_t priorSoftEdgesAccepted() const;

    uint64_t priorSoftEdgesRejected() const;

    void setMapPin(unsigned long map_id, const MapSim3Pin& pin);

    bool isMapPinned(unsigned long map_id) const;

    MapSim3Pin mapPin(unsigned long map_id) const;

    std::vector<unsigned long> pinnedMapIds() const;

    Sophus::SE3f applyPinToTwc(unsigned long map_id, const Sophus::SE3f& Twc_slam) const;

    Sophus::SE3f applyPinToTwc(unsigned long map_id,
                               const Sophus::SE3f& Twc_slam,
                               uint64_t frame_id,
                               unsigned long ref_keyframe_id,
                               const Sophus::SE3f& Tcr,
                               bool is_keyframe) const;

    NavPoseResult navPoseFromSlam(unsigned long map_id, const Sophus::SE3f& Twc_slam) const;

    json statusJson(unsigned long active_map_id) const;

private:
    static constexpr size_t kMaxPortalCornerFactorsPerMap = 512;
    static constexpr size_t kMaxPriorSoftFactorsPerMap = 512;
    static constexpr size_t kMaxPriorSoftObservationsPerFactor = 64;
    static constexpr size_t kMaxKeyframeVertices = 250;
    static constexpr int kCovisMinFeat = 100;
    static constexpr float kPortalCornerBaseWeight = 10.0f;
    static constexpr float kPriorSoftBaseWeight = 0.8f;
    /** Relative rotation σ (rad): monocular bearing is tight but not identity-weighted. */
    static constexpr float kSigmaRotRad = 0.04f;
    /** Relative translation σ as a fraction of edge length (mono depth noise scales with baseline). */
    static constexpr float kSigmaRelFraction = 0.12f;
    /** Log-scale σ per edge (~1.5%): allows slow drift along chains, blocks jumps. */
    static constexpr float kSigmaLogScale = 0.015f;
    static constexpr size_t kLiveEdgeRefineCadence = 8;
    static constexpr float kMinPortalSightBaselineSlam = 0.12f;
    static constexpr float kWeakFirstKfPriorWeight = 1e-4f;
    static constexpr size_t kMaxRelativeEdgesInRefine = 48;
    static constexpr int kGraphOptimizeIterations = 5;

    struct KeyframeVertex {
        unsigned long keyframe_id = 0;
        Sophus::Sim3f Siw;
        bool frozen = false;
    };

    struct MapGraph {
        std::vector<PortalCornerFactor> portal_corners;
        std::vector<PriorSoftEdgeFactor> prior_soft_factors;
        std::unordered_map<unsigned long, KeyframeVertex> vertices;
        MapSim3Pin pin;
        unsigned long first_keyframe_id = 0;
        bool has_first_keyframe_id = false;
    };

    MapGraph& graphFor(unsigned long map_id);

    bool resolveObservationAnchor(const AtlasSnapshot& atlas,
                                  uint64_t frame_id,
                                  unsigned long& vertex_keyframe_id,
                                  Sophus::SE3f& Tcr) const;

    void syncVerticesFromAtlas(MapGraph& graph, const AtlasSnapshot& atlas);

    void ensureFactorAnchorVertices(MapGraph& graph, const AtlasSnapshot& atlas);

    bool refinePoseGraph(MapGraph& graph,
                         const AtlasSnapshot& atlas,
                         const char* refine_source);

    bool refineSingleVertexFromCorners(MapGraph& graph, unsigned long keyframe_id);

    void updatePinSummaryFromVertices(MapGraph& graph);

    void updatePinCounts(MapSim3Pin& pin, const MapGraph& graph);

    mutable std::mutex mutex_;
    std::unordered_map<unsigned long, MapGraph> maps_;
    uint64_t prior_soft_edges_accepted_ = 0;
    uint64_t prior_soft_edges_rejected_ = 0;
    PriorSoftEdgeFactor last_prior_soft_edge_;
    std::string last_prior_soft_reject_reason_;
};

}  // namespace overlay
