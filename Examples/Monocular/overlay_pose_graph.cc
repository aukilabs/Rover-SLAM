#include "overlay_pose_graph.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "Thirdparty/g2o/g2o/types/sim3.h"

namespace overlay {

namespace {

constexpr float kMinScale = 0.01f;
constexpr float kMaxScale = 100.0f;
constexpr float kRefineLmLambda = 1e-3f;
constexpr float kCornerHuberDeltaPx = 8.0f;
constexpr float kMaxCornerRmsePx = 80.0f;
constexpr float kCornerHuberDeltaSq = kCornerHuberDeltaPx * kCornerHuberDeltaPx;
constexpr float kCornerHuberWeightFloor = 0.20f;
constexpr float kPriorSoftCauchyScalePx = 6.0f;
constexpr float kPriorSoftCauchyScaleSq = kPriorSoftCauchyScalePx * kPriorSoftCauchyScalePx;

bool isValidScale(float scale)
{
    return std::isfinite(scale) && scale >= kMinScale && scale <= kMaxScale;
}

float clampScaleAbsolute(float scale)
{
    if (!std::isfinite(scale))
        return 1.0f;
    return std::max(kMinScale, std::min(kMaxScale, scale));
}

float huberWeightFromSqError(float sq_err, float delta_sq, float weight_floor)
{
    if (sq_err <= 1e-12f)
        return 1.0f;
    if (sq_err <= delta_sq)
        return 1.0f;
    return std::max(weight_floor, std::sqrt(delta_sq / sq_err));
}

float cauchyWeightFromSqError(float sq_err, float c_sq)
{
    if (sq_err <= 1e-12f)
        return 1.0f;
    return 1.0f / (1.0f + sq_err / c_sq);
}

Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& v)
{
    Eigen::Matrix3f m;
    m << 0.0f, -v.z(), v.y(), v.z(), 0.0f, -v.x(), -v.y(), v.x(), 0.0f;
    return m;
}

g2o::Sim3 toG2oSim3(const Sophus::Sim3f& s)
{
    const Eigen::Quaterniond q(s.rxso3().quaternion().cast<double>());
    return g2o::Sim3(q, s.translation().cast<double>(), static_cast<double>(s.scale()));
}

Sophus::Sim3f fromG2oSim3(const g2o::Sim3& s)
{
    const Eigen::Quaternionf q(s.rotation().cast<float>());
    return Sophus::Sim3f(Sophus::RxSO3f(static_cast<float>(s.scale()), Sophus::SO3f(q)),
                         s.translation().cast<float>());
}

g2o::Sim3 sim3FromTcw(const Sophus::SE3f& Tcw, float scale = 1.0f)
{
    const Eigen::Quaterniond q(Tcw.unit_quaternion().cast<double>());
    return g2o::Sim3(q, Tcw.translation().cast<double>(), static_cast<double>(scale));
}

Sophus::SE3f twcFromSim3Vertex(const Sophus::Sim3f& Siw)
{
    const float s = Siw.scale();
    const Eigen::Matrix3f R = Siw.rxso3().rotationMatrix();
    const Eigen::Vector3f t = Siw.translation();
    const Eigen::Vector3f t_wc = -(1.0f / std::max(s, 1e-6f)) * R.transpose() * t;
    const Eigen::Matrix3f R_wc = R.transpose();
    return Sophus::SE3f(R_wc, t_wc);
}

Sophus::Sim3f composeDenseObservationSim3(const Sophus::Sim3f& S_rw, const Sophus::SE3f& Tcr)
{
    // T_cr is Tcw_slam in SLAM units; vertex scale scales the relative translation.
    const float s_r = S_rw.scale();
    const Eigen::Matrix3f R_rw = S_rw.rxso3().rotationMatrix();
    const Eigen::Vector3f t_cr_domain = s_r * R_rw * Tcr.translation();
    const Sophus::Sim3f S_cr(Sophus::RxSO3f(1.0f, Sophus::SO3f(Tcr.unit_quaternion())), t_cr_domain);
    return S_cr * S_rw;
}

Eigen::Vector2f projectDomainPoint(const Eigen::Vector3f& p_domain,
                                   const Sophus::SE3f& Twc_domain,
                                   float fx,
                                   float fy,
                                   float cx,
                                   float cy)
{
    const Sophus::SE3f Tcw = Twc_domain.inverse();
    const Eigen::Vector3f p_cam = Tcw.so3().matrix() * p_domain + Tcw.translation();
    if (!p_cam.allFinite() || p_cam.z() <= 1e-6f)
        return Eigen::Vector2f::Constant(std::numeric_limits<float>::quiet_NaN());
    return Eigen::Vector2f(fx * p_cam.x() / p_cam.z() + cx, fy * p_cam.y() / p_cam.z() + cy);
}

Eigen::Matrix<float, 3, 7> jacobianPCamWrtPin(const MapSim3Pin& pin,
                                               const Sophus::SE3f& Twc_slam,
                                               const Eigen::Vector3f& p_domain,
                                               const Eigen::Matrix3f& R_cw)
{
    const Eigen::Matrix3f R_slam = Twc_slam.so3().matrix();
    const Eigen::Vector3f t_slam = Twc_slam.translation();

    Eigen::Matrix<float, 3, 7> J;
    J.setZero();
    // Match _probe_sim3_conditioning.py / proven S9 single-pin normal equations.
    J.block<3, 3>(0, 0) = R_cw * skewSymmetric(p_domain - pin.t_domain_slam);
    J.col(3) = -pin.scale * (R_slam.transpose() * t_slam);
    J.block<3, 3>(0, 4) = -R_cw;
    return J;
}

struct ObservabilityReport {
    float min_eigenvalue = 0.0f;
    std::string dominant_param;
};

ObservabilityReport computeObservability(const Eigen::MatrixXf& H)
{
    ObservabilityReport report;
    if (H.rows() == 0)
        return report;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> es(H);
    if (es.info() != Eigen::Success)
        return report;
    const Eigen::VectorXf& ev = es.eigenvalues();
    report.min_eigenvalue = ev.minCoeff();
    int min_i = 0;
    ev.minCoeff(&min_i);
    const Eigen::VectorXf vec = es.eigenvectors().col(min_i);
    static const char* names[] = {"omega_x", "omega_y", "omega_z", "log_scale", "t_x", "t_y", "t_z"};
    int best = 0;
    float best_abs = 0.0f;
    for (int i = 0; i < vec.size() && i < 7; ++i) {
        const float a = std::abs(vec(i));
        if (a > best_abs) {
            best_abs = a;
            best = i;
        }
    }
    report.dominant_param = names[best];
    return report;
}

Sophus::Sim3f pinToSim3Vertex(const MapSim3Pin& pin)
{
    // Status-only Siw mirror; publish path uses applyPinFormula(pin, Twc_slam) directly.
    const Sophus::SE3f Twc_domain(pin.R_domain_slam, pin.t_domain_slam);
    return fromG2oSim3(sim3FromTcw(Twc_domain.inverse(), pin.scale));
}

Sophus::SE3f applyPinFormula(const MapSim3Pin& pin, const Sophus::SE3f& Twc_slam)
{
    const Eigen::Matrix3f R = pin.R_domain_slam * Twc_slam.so3().matrix();
    const Eigen::Vector3f t =
        pin.scale * (pin.R_domain_slam * Twc_slam.translation()) + pin.t_domain_slam;
    return Sophus::SE3f(R, t);
}

}  // namespace

OverlayPoseGraph::MapGraph& OverlayPoseGraph::graphFor(unsigned long map_id)
{
    return maps_[map_id];
}

void OverlayPoseGraph::ensureOriginVertex(MapGraph& graph)
{
    if (graph.has_origin_vertex || !graph.pin.seeded)
        return;

    syncOriginVertexFromPin(graph);
}

bool OverlayPoseGraph::refineMapSim3(MapGraph& graph, const char* refine_source)
{
    if (!graph.pin.seeded || graph.portal_corners.empty())
        return false;

    ensureOriginVertex(graph);
    if (!graph.has_origin_vertex)
        return false;

    const size_t n_corner = graph.portal_corners.size();
    const size_t n_prior = graph.prior_soft_factors.size();
    std::cerr << "[overlay] refine factors corner=" << n_corner << " prior_soft=" << n_prior
              << " source=" << (refine_source ? refine_source : "") << std::endl;

    MapSim3Pin& pin = graph.pin;
    float last_corner_rmse = 0.0f;

    for (int iter = 0; iter < 8; ++iter) {
        Eigen::Matrix<float, 7, 7> H = Eigen::Matrix<float, 7, 7>::Zero();
        Eigen::Matrix<float, 7, 1> b = Eigen::Matrix<float, 7, 1>::Zero();
        float corner_sq = 0.0f;
        float corner_w = 0.0f;

        for (const auto& factor : graph.portal_corners) {
            const Sophus::SE3f Twc_slam = factor.Tcr.inverse();
            const Sophus::SE3f Twc_domain = applyPinFormula(pin, Twc_slam);
            const Eigen::Vector2f projected =
                projectDomainPoint(factor.p_domain, Twc_domain, factor.fx, factor.fy, factor.cx, factor.cy);
            if (!projected.allFinite())
                continue;
            const Eigen::Vector2f residual = projected - factor.pixel;
            const float sq = residual.squaredNorm();
            const float w = factor.weight *
                            huberWeightFromSqError(sq, kCornerHuberDeltaSq, kCornerHuberWeightFloor);
            if (w <= 1e-8f)
                continue;
            const Sophus::SE3f Tcw = Twc_domain.inverse();
            const Eigen::Matrix3f R_cw = Tcw.so3().matrix();
            const Eigen::Vector3f p_cam = R_cw * factor.p_domain + Tcw.translation();
            const float z = p_cam.z();
            if (z <= 1e-4f)
                continue;
            Eigen::Matrix<float, 2, 3> J_proj;
            J_proj << factor.fx / z, 0.0f, -factor.fx * p_cam.x() / (z * z), 0.0f, factor.fy / z,
                -factor.fy * p_cam.y() / (z * z);
            const Eigen::Matrix<float, 2, 7> J =
                J_proj * jacobianPCamWrtPin(pin, Twc_slam, factor.p_domain, R_cw);
            if (!J.allFinite())
                continue;
            H += w * J.transpose() * J;
            b += w * J.transpose() * residual;
            corner_sq += w * sq;
            corner_w += w;
        }

        for (const auto& factor : graph.prior_soft_factors) {
            const Sophus::SE3f Twc_slam = factor.Tcr.inverse();
            const Sophus::SE3f Twc_domain = applyPinFormula(pin, Twc_slam);
            for (const auto& obs_pt : factor.observations) {
                const Eigen::Vector2f projected = projectDomainPoint(
                    obs_pt.p_domain, Twc_domain, factor.fx, factor.fy, factor.cx, factor.cy);
                if (!projected.allFinite())
                    continue;
                const Eigen::Vector2f residual = projected - obs_pt.pixel;
                const float sq = residual.squaredNorm();
                const float w = factor.weight * cauchyWeightFromSqError(sq, kPriorSoftCauchyScaleSq);
                if (w <= 1e-8f)
                    continue;
                const Sophus::SE3f Tcw = Twc_domain.inverse();
                const Eigen::Matrix3f R_cw = Tcw.so3().matrix();
                const Eigen::Vector3f p_cam = R_cw * obs_pt.p_domain + Tcw.translation();
                const float z = p_cam.z();
                if (z <= 1e-4f)
                    continue;
                Eigen::Matrix<float, 2, 3> J_proj;
                J_proj << factor.fx / z, 0.0f, -factor.fx * p_cam.x() / (z * z), 0.0f, factor.fy / z,
                    -factor.fy * p_cam.y() / (z * z);
                const Eigen::Matrix<float, 2, 7> J =
                    J_proj * jacobianPCamWrtPin(pin, Twc_slam, obs_pt.p_domain, R_cw);
                if (!J.allFinite())
                    continue;
                H += w * J.transpose() * J;
                b += w * J.transpose() * residual;
            }
        }

        H.diagonal().array() += kRefineLmLambda * (H.diagonal().array().abs() + 1.0f);
        if (H.diagonal().maxCoeff() <= 1e-12f)
            break;

        if (iter + 1 == 8 && H.allFinite()) {
            const ObservabilityReport obs = computeObservability(H);
            graph.pin.gauge_min_hessian_eigenvalue = obs.min_eigenvalue;
            graph.pin.gauge_null_dominant_param = obs.dominant_param;
        }

        Eigen::Matrix<float, 7, 1> delta = H.ldlt().solve(-b);
        if (!delta.allFinite())
            break;
        // Pin tangent order matches jacobianPCamWrtPin: [omega(3), log_scale, t(3)] —
        // not Sophus Sim3::exp ([upsilon, omega, sigma]).
        const Eigen::Matrix3f dR = Sophus::SO3f::exp(delta.segment<3>(0)).matrix();
        pin.R_domain_slam = (dR * pin.R_domain_slam).eval();
        pin.scale = clampScaleAbsolute(pin.scale * std::exp(delta(3)));
        pin.t_domain_slam += delta.segment<3>(4);
        last_corner_rmse = corner_w > 0.0f ? std::sqrt(corner_sq / corner_w) : 0.0f;
    }

    graph.pin.last_corner_rmse_px = last_corner_rmse;
    graph.pin.last_live_edge_rmse_m = 0.0f;
    graph.pin.last_refine_source = refine_source ? refine_source : "";
    graph.pin.optimization_passes++;
    syncOriginVertexFromPin(graph);
    return true;
}

RefineResult OverlayPoseGraph::recordLivePose(unsigned long map_id,
                                              uint64_t frame_id,
                                              double timestamp,
                                              const Sophus::SE3f& Twc_slam,
                                              bool is_keyframe)
{
    (void)timestamp;
    (void)Twc_slam;
    std::lock_guard<std::mutex> lock(mutex_);
    RefineResult result;
    MapGraph& graph = graphFor(map_id);
    MapSim3Pin& pin = graph.pin;

    const bool should_refine =
        pin.pinned && pin.seeded && !graph.portal_corners.empty() &&
        pin.last_corner_rmse_px <= kMaxCornerRmsePx &&
        is_keyframe && (frame_id % kLiveEdgeRefineCadence == 0);
    if (should_refine) {
        if (refineMapSim3(graph, "live_edge")) {
            updatePinCounts(pin, graph);
            result.seeded_or_refined = true;
            result.published = true;
            result.pin = pin;
            result.status = "aligned_refining";
            result.scale_update_status = "refined";
        }
    }
    return result;
}

void OverlayPoseGraph::addPortalCornerFactors(unsigned long map_id, const PortalSightInput& sight)
{
    MapGraph& graph = graphFor(map_id);
    if (!sight.Twc_slam.matrix().allFinite()) {
        graph.pin.dropped_unresolved_observations++;
        return;
    }

    for (const auto& existing : graph.portal_corners) {
        if (existing.frame_id == sight.frame_id)
            return;
    }
    unsigned long latest_sight_frame = 0;
    Sophus::SE3f latest_twc_slam;
    for (const auto& existing : graph.portal_corners) {
        if (existing.corner_index != 0)
            continue;
        if (existing.frame_id >= latest_sight_frame) {
            latest_sight_frame = existing.frame_id;
            latest_twc_slam = existing.Tcr.inverse();
        }
    }
    if (!graph.portal_corners.empty() &&
        (latest_twc_slam.translation() - sight.Twc_slam.translation()).norm() <
            kMinPortalSightBaselineSlam) {
        return;
    }

    const Sophus::SE3f Tcw_slam = sight.Twc_slam.inverse();

    const float weight = kPortalCornerBaseWeight / std::max(1.0f, sight.reproj_px);
    for (int i = 0; i < 4; ++i) {
        PortalCornerFactor factor;
        factor.frame_id = sight.frame_id;
        factor.vertex_keyframe_id = kMapOriginVertexId;
        factor.timestamp = sight.timestamp;
        factor.short_id = sight.short_id;
        factor.corner_index = i;
        factor.p_domain = sight.corners_domain[i];
        factor.pixel = sight.corners_pixel[i];
        factor.fx = sight.fx;
        factor.fy = sight.fy;
        factor.cx = sight.cx;
        factor.cy = sight.cy;
        factor.Tcr = Tcw_slam;
        factor.weight = weight;
        graph.portal_corners.push_back(factor);
    }

    if (graph.portal_corners.size() > kMaxPortalCornerFactorsPerMap) {
        const size_t overflow = graph.portal_corners.size() - kMaxPortalCornerFactorsPerMap;
        graph.portal_corners.erase(
            graph.portal_corners.begin(),
            graph.portal_corners.begin() + static_cast<std::ptrdiff_t>(overflow));
    }

    updatePinCounts(graph.pin, graph);
}

void OverlayPoseGraph::syncOriginVertexFromPin(MapGraph& graph)
{
    if (!graph.pin.seeded)
        return;

    graph.origin_vertex.keyframe_id = kMapOriginVertexId;
    graph.origin_vertex.Siw = pinToSim3Vertex(graph.pin);
    graph.has_origin_vertex = true;
}

void OverlayPoseGraph::updatePinCounts(MapSim3Pin& pin, const MapGraph& graph)
{
    pin.portal_corner_factor_count = graph.portal_corners.size();
    pin.keyframe_vertex_count = graph.has_origin_vertex ? 1 : 0;
    pin.frozen_keyframe_count = 0;
    pin.keyframe_cap_engaged = false;
    pin.live_relative_edge_count = 0;
    pin.dropped_unresolved_observations = graph.pin.dropped_unresolved_observations;
}

RefineResult OverlayPoseGraph::ingestPortalSight(unsigned long map_id, const PortalSightInput& sight)
{
    std::lock_guard<std::mutex> lock(mutex_);
    RefineResult result;
    result.status = "portal_factors_accumulating";

    MapGraph& graph = graphFor(map_id);
    const size_t corners_before = graph.portal_corners.size();
    addPortalCornerFactors(map_id, sight);
    const bool added_corners = graph.portal_corners.size() > corners_before;

    MapSim3Pin& pin = graph.pin;
    pin.last_portal_short_id = sight.short_id;
    pin.last_reproj_px = sight.reproj_px;

    const bool had_pin = pin.seeded && pin.pinned;
    const Eigen::Vector3f p_slam = sight.Twc_slam.translation();
    const Eigen::Vector3f p_domain = sight.Twc_domain.translation();

    if (!had_pin) {
        pin.R_domain_slam = sight.R_domain_slam;
        pin.scale = 1.0f;
        pin.seed_scale = 1.0f;
        pin.t_domain_slam = p_domain - pin.scale * pin.R_domain_slam * p_slam;
        pin.scale_source = "pnp_seed_init";
        pin.seeded = true;
        pin.pinned = true;
        graph.has_origin_vertex = false;
        ensureOriginVertex(graph);
        result.scale_update_status = "initialized";
    }

    const MapSim3Pin pin_before_refine = pin;
    const bool refined = (added_corners || !had_pin) && refineMapSim3(graph, "portal_corner");
    if (!refined)
        pin = pin_before_refine;
    else {
        pin.scale_source = "corner_refine";
        if (!had_pin)
            pin.seed_scale = pin.scale;
    }
    updatePinCounts(pin, graph);

    pin.pinned = true;
    pin.seeded = true;

    result.seeded_or_refined = true;
    result.published = true;
    result.pin = pin;
    result.status = had_pin ? "aligned_refining" : "aligned_seeded";
    return result;
}

void OverlayPoseGraph::clearMapAlignment(unsigned long map_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    maps_.erase(map_id);
}

RefineResult OverlayPoseGraph::addPriorSoftEdgeFactor(unsigned long map_id,
                                                      const PriorSoftEdgeFactor& factor)
{
    std::lock_guard<std::mutex> lock(mutex_);
    RefineResult result;
    MapGraph& graph = graphFor(map_id);
    if (factor.observations.empty())
        return result;
    if (!factor.Tcr.matrix().allFinite()) {
        graph.pin.dropped_unresolved_observations++;
        recordPriorSoftEdgeReject("missing_observation_pose");
        return result;
    }

    PriorSoftEdgeFactor stored = factor;
    stored.vertex_keyframe_id = kMapOriginVertexId;

    graph.prior_soft_factors.push_back(stored);
    if (graph.prior_soft_factors.size() > kMaxPriorSoftFactorsPerMap) {
        const size_t overflow = graph.prior_soft_factors.size() - kMaxPriorSoftFactorsPerMap;
        graph.prior_soft_factors.erase(
            graph.prior_soft_factors.begin(),
            graph.prior_soft_factors.begin() + static_cast<std::ptrdiff_t>(overflow));
    }
    prior_soft_edges_accepted_++;
    last_prior_soft_edge_ = stored;

    MapSim3Pin& pin = graph.pin;
    if (pin.seeded && !graph.portal_corners.empty()) {
        if (refineMapSim3(graph, "prior_soft")) {
            updatePinCounts(pin, graph);
            result.seeded_or_refined = true;
            result.published = true;
            result.pin = pin;
            result.status = "aligned_refining";
            result.scale_update_status = "prior_soft";
        }
    }
    return result;
}

void OverlayPoseGraph::recordPriorSoftEdgeReject(const std::string& reason)
{
    std::lock_guard<std::mutex> lock(mutex_);
    prior_soft_edges_rejected_++;
    last_prior_soft_reject_reason_ = reason;
}

uint64_t OverlayPoseGraph::priorSoftEdgesAccepted() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return prior_soft_edges_accepted_;
}

uint64_t OverlayPoseGraph::priorSoftEdgesRejected() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return prior_soft_edges_rejected_;
}

void OverlayPoseGraph::setMapPin(unsigned long map_id, const MapSim3Pin& pin)
{
    std::lock_guard<std::mutex> lock(mutex_);
    MapGraph& graph = graphFor(map_id);
    graph.pin = pin;
    graph.has_origin_vertex = false;
    ensureOriginVertex(graph);
}

bool OverlayPoseGraph::isMapPinned(unsigned long map_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = maps_.find(map_id);
    return it != maps_.end() && it->second.pin.pinned;
}

MapSim3Pin OverlayPoseGraph::mapPin(unsigned long map_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = maps_.find(map_id);
    if (it == maps_.end())
        return MapSim3Pin{};
    return it->second.pin;
}

std::vector<unsigned long> OverlayPoseGraph::pinnedMapIds() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<unsigned long> ids;
    for (const auto& kv : maps_) {
        if (kv.second.pin.pinned)
            ids.push_back(kv.first);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

Sophus::SE3f OverlayPoseGraph::applyPinToTwc(unsigned long map_id,
                                             const Sophus::SE3f& Twc_slam) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = maps_.find(map_id);
    if (it == maps_.end() || !it->second.pin.pinned)
        return Twc_slam;

    return applyPinFormula(it->second.pin, Twc_slam);
}

NavPoseResult OverlayPoseGraph::navPoseFromSlam(unsigned long map_id,
                                                const Sophus::SE3f& Twc_slam) const
{
    NavPoseResult out;
    out.valid = true;
    out.Twc = Twc_slam;
    out.pose_source = "slam";

    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = maps_.find(map_id);
    if (it == maps_.end() || !it->second.pin.pinned)
        return out;

    out.Twc = applyPinFormula(it->second.pin, Twc_slam);
    out.pose_source = "overlay";
    return out;
}

json OverlayPoseGraph::statusJson(unsigned long active_map_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    json out;
    out["enabled"] = true;
    out["active_map_id"] = active_map_id;
    out["model"] = "single_sim3_per_map";

    json pinned_ids = json::array();
    for (const auto& kv : maps_) {
        if (kv.second.pin.pinned)
            pinned_ids.push_back(kv.first);
    }
    out["pinned_map_ids"] = pinned_ids;

    const auto active_it = maps_.find(active_map_id);
    out["active_map_pinned"] = active_it != maps_.end() && active_it->second.pin.pinned;

    out["prior_soft_edges_accepted"] = prior_soft_edges_accepted_;
    out["prior_soft_edges_rejected"] = prior_soft_edges_rejected_;
    if (!last_prior_soft_reject_reason_.empty())
        out["prior_soft_edges_last_reject"] = last_prior_soft_reject_reason_;
    if (prior_soft_edges_accepted_ > 0) {
        json last_edge;
        last_edge["frame_id"] = last_prior_soft_edge_.frame_id;
        last_edge["prior_image_id"] = last_prior_soft_edge_.prior_image_id;
        last_edge["inlier_count"] = last_prior_soft_edge_.inlier_count;
        last_edge["mean_reproj_px"] = last_prior_soft_edge_.mean_reproj_px;
        last_edge["weight"] = last_prior_soft_edge_.weight;
        last_edge["observation_count"] = last_prior_soft_edge_.observations.size();
        out["prior_soft_edges_last"] = last_edge;
    }

    json maps = json::array();
    for (const auto& kv : maps_) {
        const unsigned long map_id = kv.first;
        const MapGraph& graph = kv.second;
        const MapSim3Pin& pin = graph.pin;
        json row;
        row["map_id"] = map_id;
        row["pinned"] = pin.pinned;
        row["seeded"] = pin.seeded;
        row["keyframe_vertex_count"] = pin.keyframe_vertex_count;
        row["frozen_keyframe_count"] = pin.frozen_keyframe_count;
        row["keyframe_cap_engaged"] = pin.keyframe_cap_engaged;
        row["portal_corner_factor_count"] = pin.portal_corner_factor_count;
        row["live_relative_edge_count"] = pin.live_relative_edge_count;
        row["prior_soft_edge_count"] = graph.prior_soft_factors.size();
        row["dense_observation_count"] = pin.dense_observation_count;
        row["dropped_unresolved_observations"] = pin.dropped_unresolved_observations;
        row["optimization_passes"] = pin.optimization_passes;
        row["last_corner_rmse_px"] = pin.last_corner_rmse_px;
        row["gauge_min_hessian_eigenvalue"] = pin.gauge_min_hessian_eigenvalue;
        row["gauge_null_dominant_param"] = pin.gauge_null_dominant_param;
        if (!pin.last_refine_source.empty())
            row["last_refine_source"] = pin.last_refine_source;
        if (pin.pinned) {
            row["scale"] = pin.scale;
            row["seed_scale"] = pin.seed_scale;
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
