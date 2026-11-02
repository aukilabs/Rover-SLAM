/**
 * Accepted portal corner sights for Atlas BA (fuse-prior-into-ba slice A).
 * Thread-safe append + query; no dependency on slam_server or overlay.
 */

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

namespace ORB_SLAM3 {

/** Sentinel when observation is dense-only or reference keyframe is unavailable (not mnId 0). */
constexpr unsigned long kUnresolvedPortalKeyframeId =
    std::numeric_limits<unsigned long>::max();

/** Unit-weight 7×7 normal-equation observability (matches _probe_sim3_conditioning.py). */
struct Sim3HessianObservability {
    bool valid = false;
    size_t residual_count = 0;
    std::array<float, 7> eigenvalues{};
    float min_eigenvalue = 0.0f;
    float condition_number = 0.0f;
    std::string null_dominant_param;
    std::array<float, 7> null_direction_loadings{};
    /** 1σ fractional scale uncertainty; NaN when unobservable (never 0 for singular H). */
    float sigma_log_s = std::numeric_limits<float>::quiet_NaN();
    /** 1σ per pin parameter (omega, log_scale, t); NaN when unobservable. */
    std::array<float, 7> sigma_params{};
};

/** Read current Twc for a sight's kf_id; return false if missing or bad (no invented pose). */
using PortalKeyframePoseResolver =
    std::function<bool(unsigned long kf_id, Eigen::Matrix3f& R_wc, Eigen::Vector3f& t_wc)>;

/** One 3D→2D prior-pack track observation (slice L pooled pin + BA). */
struct PriorSoftObservation3D2D {
    Eigen::Vector3f p_domain = Eigen::Vector3f::Zero();
    Eigen::Vector2f pixel = Eigen::Vector2f::Zero();
};

/** One gated prior-soft match (many pack-track residuals) for pooled pin / BA. */
struct PriorSoftSight {
    unsigned long map_id = 0;
    unsigned long kf_id = kUnresolvedPortalKeyframeId;
    int prior_image_id = -1;
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    float weight = 0.0f;
    uint64_t frame_id = 0;
    double timestamp = 0.0;
    float reproj_px = -1.0f;
    Eigen::Matrix3f R_wc_slam = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t_wc_slam = Eigen::Vector3f::Zero();
    Eigen::Matrix3f R_domain_slam = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t_domain_slam_seed = Eigen::Vector3f::Zero();
    bool pose_in_domain_gauge = false;
    std::vector<PriorSoftObservation3D2D> observations;
};

/** One accepted portal QR sight (4 corners) for later BA injection. */
struct PortalCornerSight {
    unsigned long map_id = 0;
    unsigned long kf_id = kUnresolvedPortalKeyframeId;
    std::string short_id;
    std::array<Eigen::Vector3f, 4> corners_domain{};
    std::array<Eigen::Vector2f, 4> corners_pixel{};
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    float weight = 0.0f;
    uint64_t frame_id = 0;
    double timestamp = 0.0;
    /** Optional mean reprojection (px); negative if unset. */
    float reproj_px = -1.0f;
    /** SLAM camera pose Twc at sight time (world → camera). */
    Eigen::Matrix3f R_wc_slam = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t_wc_slam = Eigen::Vector3f::Zero();
    /** PnP rotation map→domain; used to seed pooled solve (scale=1 gauge). */
    Eigen::Matrix3f R_domain_slam = Eigen::Matrix3f::Identity();
    /** t_domain for seed_scale=1: Twc_domain.t − R_domain_slam @ Twc_slam.t */
    Eigen::Vector3f t_domain_slam_seed = Eigen::Vector3f::Zero();
    /**
     * Degraded-mode snapshot after bulk rewrite: only for sights that had a bound kf_id but
     * whose keyframe was later culled. Never used for never-bound (pre-init) sights.
     */
    bool pose_in_domain_gauge = false;
};

/** Stored vs resolvable sight counts plus pooled residual (slice K1 / §5 #5(b)). */
struct PortalSightResolvability {
    size_t stored = 0;
    size_t resolvable = 0;
    /** Last pooled solve solved_pin.mean_reproj_px; negative if no solve yet. */
    float solved_reproj_px = -1.0f;
};

/** Overlay-style portal corner weight: kPortalCornerBaseWeight / max(1, reproj_px). */
constexpr float kPortalCornerBaseWeight = 2.5f;

inline float portalCornerWeightFromReproj(float reproj_px)
{
    return kPortalCornerBaseWeight / std::max(1.0f, reproj_px);
}

/**
 * Pooled Sim(3) solve + Hessian diagnostics.
 *
 * The weighted LM solve uses per-sight reproj weights (overlay-style). Hessians use
 * unit pixel weights with sigma_px = 1.0, matching scripts/_probe_sim3_conditioning.py.
 * Observability gate fields (valid, sigma_log_s, …) come from the seed pin (scale=1, pre-LM).
 * Domainize (slice H) applies solved_pin R/t/scale when the pooled gate passes.
 */
/** Gauge for pooled Sim(3) solve. */
struct PooledSim3SolveOptions {
    enum class Gauge { AbsoluteSlamToDomain, IncrementalDomain };
    Gauge gauge = Gauge::AbsoluteSlamToDomain;
    /** Hessian/LM operating pin (incremental mode: cumulative live Δ; absolute: unused). */
    Eigen::Matrix3f operating_R = Eigen::Matrix3f::Identity();
    Eigen::Vector3f operating_t = Eigen::Vector3f::Zero();
    float operating_scale = 1.0f;
};

struct PooledSim3ObservabilityReport {
    /** Primary gate — mirrors seed_pin.hessian.valid. */
    bool valid = false;
    unsigned long map_id = 0;
    size_t sight_count = 0;
    /** "absolute" (pre-domainize) or "incremental" (post-domainize). */
    std::string solve_gauge = "absolute";

    struct PinDiagnostics {
        Eigen::Matrix3f R_domain_slam = Eigen::Matrix3f::Identity();
        Eigen::Vector3f t_domain_slam = Eigen::Vector3f::Zero();
        float scale = 1.0f;
        float mean_reproj_px = 0.0f;
        Sim3HessianObservability hessian;
    };

    /** scale=1 PnP seed, before LM — operating point for slice H gate. */
    PinDiagnostics seed_pin;
    /** After weighted LM (diagnostic only; not applied to Atlas). */
    PinDiagnostics solved_pin;

    size_t portal_sight_count = 0;
    size_t prior_soft_sight_count = 0;
    size_t prior_soft_residual_count = 0;

    // Top-level mirrors seed_pin.hessian for /status backward compatibility.
    size_t residual_count = 0;
    std::array<float, 7> hessian_eigenvalues{};
    float condition_number = 0.0f;
    std::string null_dominant_param;
    std::array<float, 7> null_direction_loadings{};
    float sigma_log_s = std::numeric_limits<float>::quiet_NaN();
    float scale = 1.0f;
    float mean_reproj_px = 0.0f;
};

class PortalObservationStore {
public:
    static PortalObservationStore& Instance();

    /**
     * Append one accepted sight with a bound observing keyframe.
     * Rejects never-bound sights (pre-init / no reference KF). Returns store size, or 0.
     */
    size_t tryAppend(const PortalCornerSight& sight);

    /** Append one gated prior-soft sight; rejects unbound kf_id. Returns store size or 0. */
    size_t tryAppendPriorSoft(const PriorSoftSight& sight);

    /** Drop never-bound rows for map_id (e.g. legacy pre-init); returns purged count. */
    size_t purgeUnboundSights(unsigned long map_id);

    std::vector<PortalCornerSight> queryByMap(unsigned long map_id) const;
    std::vector<PortalCornerSight> queryByKeyframeIds(
        const std::unordered_set<unsigned long>& kf_ids) const;
    std::vector<PortalCornerSight> queryByMapAndKeyframeIds(
        unsigned long map_id,
        const std::unordered_set<unsigned long>& kf_ids) const;

    std::vector<PriorSoftSight> queryPriorSoftByMapAndKeyframeIds(
        unsigned long map_id,
        const std::unordered_set<unsigned long>& kf_ids) const;

    size_t count() const;
    size_t countByMap(unsigned long map_id) const;
    size_t countPriorSoftByMap(unsigned long map_id) const;

    void clearMap(unsigned long map_id);
    void clearAll();

    /**
     * Estimate map→domain Sim(3) from all stored sights for map_id and report
     * 7×7 normal-equation conditioning. slam_server applies solved_pin on gate pass (slice H).
     */
    PooledSim3ObservabilityReport solvePooledSim3Diagnostics(
        unsigned long map_id,
        const PortalKeyframePoseResolver& resolve_pose = PortalKeyframePoseResolver(),
        const PooledSim3SolveOptions& options = PooledSim3SolveOptions());

    /** Last pooled-solve report for map_id (empty/invalid if never run). */
    PooledSim3ObservabilityReport pooledObservability(unsigned long map_id) const;

    /** True when kf_id is referenced by a recent stored sight (anti-cull, bounded). */
    bool isKeyframeAntiCulled(unsigned long map_id, unsigned long kf_id) const;

    /**
     * Rewrite sight Twc poses under the same bulk Sim(3) as Map::ApplyScaledRotation.
     * Call after domainize / live-scale correction on that map.
     */
    void applyScaledRotationToSightPoses(unsigned long map_id,
                                         const Eigen::Matrix3f& R_domain_slam,
                                         const Eigen::Vector3f& t_domain_slam,
                                         float scale);

    /** Count sights resolvable under the same rules as solvePooledSim3Diagnostics. */
    PortalSightResolvability countResolvableSights(
        unsigned long map_id,
        const PortalKeyframePoseResolver& resolve_pose,
        const PooledSim3SolveOptions& options) const;

    /** Last resolvability snapshot from pooled solve (or countResolvable if never solved). */
    PortalSightResolvability lastResolvability(unsigned long map_id) const;

private:
    PortalObservationStore() = default;

    void rebuildProtectedKeyframes(unsigned long map_id);

    mutable std::mutex mutex_;
    std::vector<PortalCornerSight> observations_;
    std::vector<PriorSoftSight> prior_soft_sights_;
    std::unordered_map<unsigned long, PooledSim3ObservabilityReport> last_pooled_reports_;
    std::unordered_map<unsigned long, PortalSightResolvability> last_resolvability_;
    /** Per-map kf mnIds exempt from KeyFrameCulling (rebuilt from recent sights). */
    std::unordered_map<unsigned long, std::unordered_set<unsigned long>> protected_kf_ids_;
};

} // namespace ORB_SLAM3
