#include "overlay_pose_graph.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
constexpr float kMinRelativeTranslationM = 1e-3f;
constexpr float kNumericJacobianEps = 1e-5f;
constexpr float kSigmaRotRad = 0.04f;
constexpr float kSigmaRelFraction = 0.12f;
constexpr float kSigmaLogScale = 0.015f;

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
    // T_cr is in SLAM units; reference keyframe scale must scale the relative translation.
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

Eigen::Matrix<float, 3, 7> jacobianPCamWrtSim3(const Sophus::Sim3f& Siw,
                                               const Eigen::Vector3f& p_domain,
                                               const Eigen::Matrix3f& R_cw)
{
    const Sophus::SE3f Twc = twcFromSim3Vertex(Siw);
    const float scale = Siw.scale();
    const Eigen::Matrix3f R_wc = Twc.so3().matrix();
    const Eigen::Vector3f t_wc = Twc.translation();

    Eigen::Matrix<float, 3, 7> J;
    J.setZero();
    // Match proven single-pin normal equations (_probe_sim3_conditioning.py), per-vertex Siw.
    J.block<3, 3>(0, 0) = R_cw * skewSymmetric(p_domain - Siw.translation());
    J.col(3) = -scale * (R_wc.transpose() * t_wc);
    J.block<3, 3>(0, 4) = -R_cw;
    return J;
}

Eigen::Matrix<float, 7, 1> sim3LogResidual(const g2o::Sim3& measurement,
                                            const g2o::Sim3& Siw,
                                            const g2o::Sim3& Sjw)
{
    const g2o::Sim3 err = measurement * Siw * Sjw.inverse();
    const g2o::Vector7d logv = err.log();
    Eigen::Matrix<float, 7, 1> out;
    for (int i = 0; i < 7; ++i) {
        const double v = logv(i);
        out(i) = std::isfinite(v) ? static_cast<float>(v) : 0.0f;
    }
    return out;
}

Eigen::Matrix<float, 7, 7> relativeEdgeSqrtInformation(float covis_weight, float t_ji_norm)
{
    Eigen::Matrix<float, 7, 7> info = Eigen::Matrix<float, 7, 7>::Zero();
    const float rot_w = std::sqrt(std::max(1.0f, covis_weight));
    const float inv_sigma_rot = rot_w / kSigmaRotRad;
    info.block<3, 3>(0, 0).diagonal().setConstant(inv_sigma_rot * inv_sigma_rot);

    const float t_norm = std::max(kMinRelativeTranslationM, t_ji_norm);
    const float inv_sigma_t = 1.0f / (kSigmaRelFraction * t_norm);
    info.block<3, 3>(4, 4).diagonal().setConstant(inv_sigma_t * inv_sigma_t);

    const float inv_sigma_logs = 1.0f / kSigmaLogScale;
    info(3, 3) = inv_sigma_logs * inv_sigma_logs;
    return info.llt().matrixL();
}

void numericRelativeJacobian(const g2o::Sim3& measurement,
                             const g2o::Sim3& Siw,
                             const g2o::Sim3& Sjw,
                             Eigen::Matrix<float, 7, 7>& Ji,
                             Eigen::Matrix<float, 7, 7>& Jj)
{
    Ji.setZero();
    Jj.setZero();
    const Eigen::Matrix<float, 7, 1> r0 = sim3LogResidual(measurement, Siw, Sjw);
    for (int k = 0; k < 7; ++k) {
        g2o::Sim3 Siw_p = Siw;
        g2o::Sim3 Sjw_p = Sjw;
        g2o::Vector7d delta = g2o::Vector7d::Zero();
        delta[k] = kNumericJacobianEps;
        Siw_p = g2o::Sim3(delta) * Siw_p;
        const Eigen::Matrix<float, 7, 1> ri = sim3LogResidual(measurement, Siw_p, Sjw);
        Ji.col(k) = (ri - r0) / kNumericJacobianEps;

        delta.setZero();
        delta[k] = kNumericJacobianEps;
        Sjw_p = g2o::Sim3(delta) * Sjw_p;
        const Eigen::Matrix<float, 7, 1> rj = sim3LogResidual(measurement, Siw, Sjw_p);
        Jj.col(k) = (rj - r0) / kNumericJacobianEps;
    }
}

struct RelativeSim3Edge {
    unsigned long id_i = 0;
    unsigned long id_j = 0;
    g2o::Sim3 measurement;
    float covis_weight = 1.0f;
    float t_ji_norm = 1.0f;
};

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
    const int idx = ev.size() - 1;
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
    (void)idx;
    report.dominant_param = names[best];
    return report;
}

struct RefineCosts {
    float corner_sq = 0.0f;
    float corner_w = 0.0f;
    float prior_soft_sq = 0.0f;
    float prior_soft_w = 0.0f;
    float relative_sq = 0.0f;
    float relative_w = 0.0f;

    float cornerRmsePx() const
    {
        return corner_w > 0.0f ? std::sqrt(corner_sq / corner_w) : 0.0f;
    }

    float totalCost() const { return corner_sq + prior_soft_sq + relative_sq; }
};

bool resolveSightAnchor(const PortalSightInput& sight,
                        const AtlasSnapshot& atlas,
                        unsigned long& vertex_keyframe_id,
                        Sophus::SE3f& Tcr)
{
    if (sight.has_overlay_anchor) {
        vertex_keyframe_id = sight.ref_keyframe_id;
        Tcr = sight.Tcr;
        return true;
    }
    const unsigned long lookup_id =
        sight.orb_frame_mn_id != 0 ? sight.orb_frame_mn_id : sight.frame_id;
    for (const auto& ref : atlas.frame_refs) {
        if (ref.frame_id == lookup_id) {
            vertex_keyframe_id = ref.ref_keyframe_id;
            Tcr = ref.Tcr;
            return true;
        }
    }
    for (const auto& kf : atlas.keyframes) {
        if (kf.id == lookup_id) {
            vertex_keyframe_id = kf.id;
            Tcr = Sophus::SE3f();
            return true;
        }
    }
    if (!atlas.keyframes.empty()) {
        vertex_keyframe_id = atlas.keyframes.back().id;
        Tcr = Sophus::SE3f();
        return true;
    }
    // Pre-init / empty atlas: ORB init keyframe is always mnId 0.
    vertex_keyframe_id = 0;
    Tcr = Sophus::SE3f();
    return true;
}

}  // namespace

OverlayPoseGraph::MapGraph& OverlayPoseGraph::graphFor(unsigned long map_id)
{
    return maps_[map_id];
}

bool OverlayPoseGraph::resolveObservationAnchor(const AtlasSnapshot& atlas,
                                              uint64_t frame_id,
                                              unsigned long& vertex_keyframe_id,
                                              Sophus::SE3f& Tcr) const
{
    Tcr = Sophus::SE3f();
    for (const auto& kf : atlas.keyframes) {
        if (kf.id == frame_id) {
            vertex_keyframe_id = kf.id;
            return true;
        }
    }
    for (const auto& ref : atlas.frame_refs) {
        if (ref.frame_id == frame_id) {
            vertex_keyframe_id = ref.ref_keyframe_id;
            Tcr = ref.Tcr;
            return true;
        }
    }
    return false;
}

void OverlayPoseGraph::syncVerticesFromAtlas(MapGraph& graph, const AtlasSnapshot& atlas)
{
    if (atlas.keyframes.empty())
        return;

    if (!graph.has_first_keyframe_id) {
        graph.first_keyframe_id = atlas.keyframes.front().id;
        graph.has_first_keyframe_id = true;
    }

    const MapSim3Pin& pin = graph.pin;
    for (const auto& kf : atlas.keyframes) {
        auto it = graph.vertices.find(kf.id);
        if (it != graph.vertices.end())
            continue;

        KeyframeVertex vtx;
        vtx.keyframe_id = kf.id;
        if (pin.seeded) {
            const Sophus::SE3f Twc_slam = kf.Tcw.inverse();
            const Eigen::Matrix3f R = pin.R_domain_slam * Twc_slam.so3().matrix();
            const Eigen::Vector3f t =
                pin.scale * (pin.R_domain_slam * Twc_slam.translation()) + pin.t_domain_slam;
            const Sophus::SE3f Tcw_domain = Sophus::SE3f(R, t).inverse();
            vtx.Siw = fromG2oSim3(sim3FromTcw(Tcw_domain, 1.0f));
        } else {
            vtx.Siw = fromG2oSim3(sim3FromTcw(kf.Tcw, 1.0f));
        }
        graph.vertices.emplace(kf.id, vtx);
    }

    if (graph.vertices.size() <= kMaxKeyframeVertices)
        return;

    graph.pin.keyframe_cap_engaged = true;
    std::vector<unsigned long> ids;
    ids.reserve(graph.vertices.size());
    for (const auto& kv : graph.vertices)
        ids.push_back(kv.first);
    std::sort(ids.begin(), ids.end());
    const size_t freeze_count = ids.size() - kMaxKeyframeVertices;
    for (size_t i = 0; i < freeze_count; ++i) {
        graph.vertices[ids[i]].frozen = true;
    }
}

void OverlayPoseGraph::ensureFactorAnchorVertices(MapGraph& graph, const AtlasSnapshot& atlas)
{
    auto ensure_one = [&](unsigned long kf_id) {
        if (graph.vertices.count(kf_id))
            return;

        Sophus::SE3f Tcw;
        bool found_tcw = false;
        for (const auto& kf : atlas.keyframes) {
            if (kf.id == kf_id) {
                Tcw = kf.Tcw;
                found_tcw = true;
                break;
            }
        }

        KeyframeVertex vtx;
        vtx.keyframe_id = kf_id;
        const MapSim3Pin& pin = graph.pin;
        if (found_tcw) {
            if (pin.seeded) {
                const Sophus::SE3f Twc_slam = Tcw.inverse();
                const Eigen::Matrix3f R = pin.R_domain_slam * Twc_slam.so3().matrix();
                const Eigen::Vector3f t =
                    pin.scale * (pin.R_domain_slam * Twc_slam.translation()) + pin.t_domain_slam;
                const Sophus::SE3f Tcw_domain = Sophus::SE3f(R, t).inverse();
                vtx.Siw = fromG2oSim3(sim3FromTcw(Tcw_domain, 1.0f));
            } else {
                vtx.Siw = fromG2oSim3(sim3FromTcw(Tcw, 1.0f));
            }
        } else if (pin.seeded) {
            const Sophus::SE3f Twc_domain(pin.R_domain_slam, pin.t_domain_slam);
            vtx.Siw = fromG2oSim3(sim3FromTcw(Twc_domain.inverse(), 1.0f));
        } else {
            return;
        }
        graph.vertices.emplace(kf_id, vtx);
    };

    for (const auto& factor : graph.portal_corners)
        ensure_one(factor.vertex_keyframe_id);
    for (const auto& factor : graph.prior_soft_factors)
        ensure_one(factor.vertex_keyframe_id);
}

bool OverlayPoseGraph::refineSingleVertexFromCorners(MapGraph& graph, unsigned long keyframe_id)
{
    auto vtx_it = graph.vertices.find(keyframe_id);
    if (vtx_it == graph.vertices.end())
        return false;

    Sophus::Sim3f& Siw = vtx_it->second.Siw;
    float last_corner_rmse = 0.0f;

    for (int iter = 0; iter < 8; ++iter) {
        Eigen::Matrix<float, 7, 7> H = Eigen::Matrix<float, 7, 7>::Zero();
        Eigen::Matrix<float, 7, 1> b = Eigen::Matrix<float, 7, 1>::Zero();
        float corner_sq = 0.0f;
        float corner_w = 0.0f;

        for (const auto& factor : graph.portal_corners) {
            if (factor.vertex_keyframe_id != keyframe_id)
                continue;
            const Sophus::Sim3f obs = composeDenseObservationSim3(Siw, factor.Tcr);
            const Sophus::SE3f Twc_domain = twcFromSim3Vertex(obs);
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
                J_proj * jacobianPCamWrtSim3(Siw, factor.p_domain, R_cw);
            if (!J.allFinite())
                continue;
            H += w * J.transpose() * J;
            b += w * J.transpose() * residual;
            corner_sq += w * sq;
            corner_w += w;
        }

        for (const auto& factor : graph.prior_soft_factors) {
            if (factor.vertex_keyframe_id != keyframe_id)
                continue;
            const Sophus::Sim3f obs = composeDenseObservationSim3(Siw, factor.Tcr);
            const Sophus::SE3f Twc_domain = twcFromSim3Vertex(obs);
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
                    J_proj * jacobianPCamWrtSim3(Siw, obs_pt.p_domain, R_cw);
                if (!J.allFinite())
                    continue;
                H += w * J.transpose() * J;
                b += w * J.transpose() * residual;
            }
        }

        H.diagonal().array() += kRefineLmLambda * (H.diagonal().array().abs() + 1.0f);
        if (H.diagonal().maxCoeff() <= 1e-12f)
            break;

        if (iter + 1 == 8 && H.allFinite() && false) {
            const ObservabilityReport obs = computeObservability(H);
            graph.pin.gauge_min_hessian_eigenvalue = obs.min_eigenvalue;
            graph.pin.gauge_null_dominant_param = obs.dominant_param;
        }

        Eigen::Matrix<float, 7, 1> delta = H.ldlt().solve(-b);
        if (!delta.allFinite())
            break;
        const Sophus::Sim3f updated = Sophus::Sim3f::exp(delta) * Siw;
        const float s = clampScaleAbsolute(updated.scale());
        Siw = Sophus::Sim3f(Sophus::RxSO3f(s, Sophus::SO3f(updated.rxso3().quaternion())), updated.translation());
        last_corner_rmse = corner_w > 0.0f ? std::sqrt(corner_sq / corner_w) : 0.0f;
    }

    graph.pin.last_corner_rmse_px = last_corner_rmse;
    return true;
}

RefineResult OverlayPoseGraph::recordLivePose(unsigned long map_id,
                                              uint64_t frame_id,
                                              double timestamp,
                                              const Sophus::SE3f& Twc_slam,
                                              bool is_keyframe,
                                              const AtlasSnapshot& atlas)
{
    (void)frame_id;
    (void)timestamp;
    (void)Twc_slam;
    (void)is_keyframe;
    std::lock_guard<std::mutex> lock(mutex_);
    RefineResult result;
    MapGraph& graph = graphFor(map_id);
    MapSim3Pin& pin = graph.pin;

    syncVerticesFromAtlas(graph, atlas);

    const bool should_refine =
        pin.pinned && pin.seeded && !graph.portal_corners.empty() &&
        pin.last_corner_rmse_px <= kMaxCornerRmsePx &&
        is_keyframe && (frame_id % kLiveEdgeRefineCadence == 0);
    if (should_refine) {
        if (refinePoseGraph(graph, atlas, "live_edge")) {
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

void OverlayPoseGraph::addPortalCornerFactors(unsigned long map_id,
                                              const PortalSightInput& sight,
                                              const AtlasSnapshot& atlas)
{
    MapGraph& graph = graphFor(map_id);
    unsigned long vertex_kf = 0;
    Sophus::SE3f Tcr;
    if (!resolveSightAnchor(sight, atlas, vertex_kf, Tcr))
        return;

    const auto vtx_it = graph.vertices.find(vertex_kf);
    if (vtx_it != graph.vertices.end()) {
        const Sophus::Sim3f obs = composeDenseObservationSim3(vtx_it->second.Siw, Tcr);
        const Sophus::SE3f Twc_obs = twcFromSim3Vertex(obs);
        for (const auto& existing : graph.portal_corners) {
            const auto ex_it = graph.vertices.find(existing.vertex_keyframe_id);
            if (ex_it == graph.vertices.end())
                continue;
            const Sophus::Sim3f existing_sim =
                composeDenseObservationSim3(ex_it->second.Siw, existing.Tcr);
            const Sophus::SE3f existing_twc = twcFromSim3Vertex(existing_sim);
            if ((existing_twc.translation() - Twc_obs.translation()).norm() <
                kMinPortalSightBaselineSlam) {
                updatePinCounts(graph.pin, graph);
                return;
            }
        }
    }

    const float weight = kPortalCornerBaseWeight / std::max(1.0f, sight.reproj_px);
    for (int i = 0; i < 4; ++i) {
        PortalCornerFactor factor;
        factor.frame_id = sight.frame_id;
        factor.vertex_keyframe_id = vertex_kf;
        factor.timestamp = sight.timestamp;
        factor.short_id = sight.short_id;
        factor.corner_index = i;
        factor.p_domain = sight.corners_domain[i];
        factor.pixel = sight.corners_pixel[i];
        factor.fx = sight.fx;
        factor.fy = sight.fy;
        factor.cx = sight.cx;
        factor.cy = sight.cy;
        factor.Tcr = Tcr;
        factor.weight = weight;
        graph.portal_corners.push_back(factor);
    }

    if (graph.portal_corners.size() > kMaxPortalCornerFactorsPerMap) {
        const size_t overflow = graph.portal_corners.size() - kMaxPortalCornerFactorsPerMap;
        graph.portal_corners.erase(
            graph.portal_corners.begin(),
            graph.portal_corners.begin() + static_cast<std::ptrdiff_t>(overflow));
    }

    if (!Tcr.matrix().isApprox(Eigen::Matrix4f::Identity(), 1e-5f))
        graph.pin.dense_observation_count++;

    updatePinCounts(graph.pin, graph);
}

bool OverlayPoseGraph::refinePoseGraph(MapGraph& graph,
                                       const AtlasSnapshot& atlas,
                                       const char* refine_source)
{
    try {
    if (!graph.pin.seeded || graph.portal_corners.empty())
        return false;

    const size_t absolute_count =
        graph.portal_corners.size() + graph.prior_soft_factors.size();
    if (absolute_count == 0) {
        graph.pin.last_refine_source = "no_absolute_constraints";
        return false;
    }

    syncVerticesFromAtlas(graph, atlas);
    ensureFactorAnchorVertices(graph, atlas);

    std::vector<unsigned long> active_ids;
    active_ids.reserve(graph.vertices.size());
    for (const auto& kv : graph.vertices) {
        if (!kv.second.frozen)
            active_ids.push_back(kv.first);
    }
    std::sort(active_ids.begin(), active_ids.end());
    if (active_ids.empty())
        return false;

    if (active_ids.size() == 1) {
        if (refineSingleVertexFromCorners(graph, active_ids.front())) {
            updatePinSummaryFromVertices(graph);
            graph.pin.last_live_edge_rmse_m = 0.0f;
            graph.pin.last_refine_source = refine_source ? refine_source : "";
            graph.pin.optimization_passes++;
            return true;
        }
        return false;
    }

    std::unordered_map<unsigned long, int> id_to_slot;
    for (size_t i = 0; i < active_ids.size(); ++i)
        id_to_slot[active_ids[i]] = static_cast<int>(i);

    const int n_params = static_cast<int>(active_ids.size()) * 7;
    std::vector<Sophus::Sim3f> trial_vertices(active_ids.size());
    for (size_t i = 0; i < active_ids.size(); ++i)
        trial_vertices[i] = graph.vertices.at(active_ids[i]).Siw;

    std::unordered_map<unsigned long, g2o::Sim3> atlas_siw;
    for (const auto& kf : atlas.keyframes)
        atlas_siw[kf.id] = sim3FromTcw(kf.Tcw, 1.0f);

    std::vector<RelativeSim3Edge> rel_edges;
    std::unordered_set<uint64_t> edge_keys;
    auto add_edge = [&](unsigned long id_i, unsigned long id_j, float covis_w) {
        if (!id_to_slot.count(id_i) || !id_to_slot.count(id_j))
            return;
        if (!atlas_siw.count(id_i) || !atlas_siw.count(id_j))
            return;
        const uint64_t lo = std::min(id_i, id_j);
        const uint64_t hi = std::max(id_i, id_j);
        const uint64_t key = (lo << 32) | hi;
        if (!edge_keys.insert(key).second)
            return;
        const g2o::Sim3& Siw = atlas_siw.at(id_i);
        const g2o::Sim3& Sjw = atlas_siw.at(id_j);
        const g2o::Sim3 Sji = Sjw * Siw.inverse();
        RelativeSim3Edge edge;
        edge.id_i = id_i;
        edge.id_j = id_j;
        edge.measurement = Sji;
        edge.covis_weight = covis_w;
        edge.t_ji_norm = static_cast<float>(Sji.translation().norm());
        rel_edges.push_back(edge);
    };

    for (const auto& kf : atlas.keyframes) {
        if (kf.parent_id != kf.id)
            add_edge(kf.id, kf.parent_id, 1.0f);
        for (const auto& cov : kf.covisibles)
            add_edge(kf.id, cov.first, static_cast<float>(cov.second));
    }
    graph.pin.live_relative_edge_count = rel_edges.size();
    if (rel_edges.size() > kMaxRelativeEdgesInRefine) {
        rel_edges.erase(rel_edges.begin(),
                        rel_edges.end() - static_cast<std::ptrdiff_t>(kMaxRelativeEdgesInRefine));
    }

    if (active_ids.size() > 1) {
        const unsigned long anchor_kf = graph.portal_corners.empty()
                                            ? active_ids.back()
                                            : graph.portal_corners.back().vertex_keyframe_id;
        if (refineSingleVertexFromCorners(graph, anchor_kf)) {
            updatePinSummaryFromVertices(graph);
            graph.pin.last_live_edge_rmse_m = 0.0f;
            graph.pin.last_refine_source = refine_source ? refine_source : "";
            graph.pin.optimization_passes++;
            return true;
        }
        return false;
    }

    auto evaluateCosts = [&](const std::vector<Sophus::Sim3f>& verts) {
        RefineCosts costs;
        for (const auto& factor : graph.portal_corners) {
            const auto slot_it = id_to_slot.find(factor.vertex_keyframe_id);
            if (slot_it == id_to_slot.end())
                continue;
            const Sophus::Sim3f obs =
                composeDenseObservationSim3(verts[slot_it->second], factor.Tcr);
            const Sophus::SE3f Twc_domain = twcFromSim3Vertex(obs);
            const Eigen::Vector2f projected =
                projectDomainPoint(factor.p_domain, Twc_domain, factor.fx, factor.fy, factor.cx, factor.cy);
            if (!projected.allFinite())
                continue;
            const Eigen::Vector2f residual = projected - factor.pixel;
            const float sq = residual.squaredNorm();
            const float w = factor.weight *
                            huberWeightFromSqError(sq, kCornerHuberDeltaSq, kCornerHuberWeightFloor);
            costs.corner_sq += w * sq;
            costs.corner_w += w;
        }
        for (const auto& factor : graph.prior_soft_factors) {
            const auto slot_it = id_to_slot.find(factor.vertex_keyframe_id);
            if (slot_it == id_to_slot.end())
                continue;
            const Sophus::Sim3f obs =
                composeDenseObservationSim3(verts[slot_it->second], factor.Tcr);
            const Sophus::SE3f Twc_domain = twcFromSim3Vertex(obs);
            for (const auto& obs_pt : factor.observations) {
                const Eigen::Vector2f projected = projectDomainPoint(
                    obs_pt.p_domain, Twc_domain, factor.fx, factor.fy, factor.cx, factor.cy);
                if (!projected.allFinite())
                    continue;
                const Eigen::Vector2f residual = projected - obs_pt.pixel;
                const float sq = residual.squaredNorm();
                const float w = factor.weight * cauchyWeightFromSqError(sq, kPriorSoftCauchyScaleSq);
                costs.prior_soft_sq += w * sq;
                costs.prior_soft_w += w;
            }
        }
        for (const auto& edge : rel_edges) {
            const g2o::Sim3& Siw = toG2oSim3(verts[id_to_slot.at(edge.id_i)]);
            const g2o::Sim3& Sjw = toG2oSim3(verts[id_to_slot.at(edge.id_j)]);
            const Eigen::Matrix<float, 7, 1> r = sim3LogResidual(edge.measurement, Siw, Sjw);
            const Eigen::Matrix<float, 7, 7> sqrt_info =
                relativeEdgeSqrtInformation(edge.covis_weight, edge.t_ji_norm);
            const Eigen::Matrix<float, 7, 1> rw = sqrt_info * r;
            costs.relative_sq += rw.squaredNorm();
            costs.relative_w += 7.0f;
        }
        return costs;
    };

    const RefineCosts initial_costs = evaluateCosts(trial_vertices);
    const float initial_corner_rmse = initial_costs.cornerRmsePx();
    RefineCosts last_costs = initial_costs;

    for (int iter = 0; iter < kGraphOptimizeIterations; ++iter) {
        Eigen::MatrixXf H = Eigen::MatrixXf::Zero(n_params, n_params);
        Eigen::VectorXf b = Eigen::VectorXf::Zero(n_params);

        for (const auto& factor : graph.portal_corners) {
            const auto slot_it = id_to_slot.find(factor.vertex_keyframe_id);
            if (slot_it == id_to_slot.end())
                continue;
            const int slot = slot_it->second;
            const Sophus::Sim3f obs =                 composeDenseObservationSim3(trial_vertices[slot], factor.Tcr);
            const Sophus::SE3f Twc_domain = twcFromSim3Vertex(obs);
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
                J_proj * jacobianPCamWrtSim3(trial_vertices[slot], factor.p_domain, R_cw);
            if (!J.allFinite())
                continue;
            const int base = slot * 7;
            H.block<7, 7>(base, base) += w * J.transpose() * J;
            b.segment<7>(base) += w * J.transpose() * residual;
        }

        for (const auto& factor : graph.prior_soft_factors) {
            const auto slot_it = id_to_slot.find(factor.vertex_keyframe_id);
            if (slot_it == id_to_slot.end())
                continue;
            const int slot = slot_it->second;
            const Sophus::Sim3f obs =
                composeDenseObservationSim3(trial_vertices[slot], factor.Tcr);
            const Sophus::SE3f Twc_domain = twcFromSim3Vertex(obs);
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
                    J_proj *
                    jacobianPCamWrtSim3(trial_vertices[slot], obs_pt.p_domain, R_cw);
                if (!J.allFinite())
                    continue;
                const int base = slot * 7;
                H.block<7, 7>(base, base) += w * J.transpose() * J;
                b.segment<7>(base) += w * J.transpose() * residual;
            }
        }

        for (const auto& edge : rel_edges) {
            const auto slot_i_it = id_to_slot.find(edge.id_i);
            const auto slot_j_it = id_to_slot.find(edge.id_j);
            if (slot_i_it == id_to_slot.end() || slot_j_it == id_to_slot.end())
                continue;
            const int slot_i = slot_i_it->second;
            const int slot_j = slot_j_it->second;
            const g2o::Sim3& Siw = toG2oSim3(trial_vertices[slot_i]);
            const g2o::Sim3& Sjw = toG2oSim3(trial_vertices[slot_j]);
            const Eigen::Matrix<float, 7, 1> r = sim3LogResidual(edge.measurement, Siw, Sjw);
            const Eigen::Matrix<float, 7, 7> sqrt_info =
                relativeEdgeSqrtInformation(edge.covis_weight, edge.t_ji_norm);
            const Eigen::Matrix<float, 7, 7> w_info = sqrt_info.transpose() * sqrt_info;
            // Block-diagonal Gauss-Newton approx (log(S_ji·S_iw·S_jw^-1)); avoids costly
            // numeric Sim3 Jacobians on every covis edge each LM iteration.
            const int base_i = slot_i * 7;
            const int base_j = slot_j * 7;
            H.block<7, 7>(base_i, base_i) += w_info;
            H.block<7, 7>(base_j, base_j) += w_info;
            const Eigen::Matrix<float, 7, 1> wb = w_info * r;
            b.segment<7>(base_i) += wb;
            b.segment<7>(base_j) -= wb;
        }

        if (graph.has_first_keyframe_id) {
            const auto first_it = id_to_slot.find(graph.first_keyframe_id);
            if (first_it != id_to_slot.end()) {
                const int base = first_it->second * 7;
                H.block<7, 7>(base, base).diagonal().array() += kWeakFirstKfPriorWeight;
            }
        }

        H.diagonal().array() += kRefineLmLambda * (H.diagonal().array().abs() + 1.0f);
        if (H.diagonal().maxCoeff() <= 1e-12f)
            break;

        Eigen::VectorXf delta = H.ldlt().solve(-b);
        if (!delta.allFinite())
            break;
        const float delta_norm = delta.norm();
        if (delta_norm > 1.0f)
            delta *= (1.0f / delta_norm);

        std::vector<Sophus::Sim3f> candidate = trial_vertices;
        bool accepted = false;
        float step = 1.0f;
        for (int line_search = 0; line_search < 5; ++line_search) {
            for (size_t i = 0; i < active_ids.size(); ++i) {
                Eigen::Matrix<float, 7, 1> inc = Eigen::Matrix<float, 7, 1>::Zero();
                for (int k = 0; k < 7; ++k)
                    inc(k) = step * delta(static_cast<int>(i * 7 + k));
                const Sophus::Sim3f updated = Sophus::Sim3f::exp(inc) * trial_vertices[i];
                const float s = clampScaleAbsolute(updated.scale());
                candidate[i] = Sophus::Sim3f(
                    Sophus::RxSO3f(s, Sophus::SO3f(updated.rxso3().quaternion())), updated.translation());
            }
            const RefineCosts candidate_costs = evaluateCosts(candidate);
            if (candidate_costs.totalCost() <= last_costs.totalCost() * 1.001f &&
                !(initial_corner_rmse > 0.0f &&
                  candidate_costs.cornerRmsePx() > initial_corner_rmse * 1.25f)) {
                trial_vertices = std::move(candidate);
                last_costs = candidate_costs;
                accepted = true;
                break;
            }
            step *= 0.5f;
        }
        if (!accepted)
            break;
    }

    for (size_t i = 0; i < active_ids.size(); ++i)
        graph.vertices.at(active_ids[i]).Siw = trial_vertices[i];

    updatePinSummaryFromVertices(graph);
    graph.pin.last_corner_rmse_px = last_costs.cornerRmsePx();
    graph.pin.last_live_edge_rmse_m = 0.0f;
    graph.pin.last_refine_source = refine_source ? refine_source : "";
    graph.pin.optimization_passes++;
    return true;
    } catch (const std::exception& e) {
        graph.pin.last_refine_source = std::string("exception:") + e.what();
        return false;
    } catch (...) {
        graph.pin.last_refine_source = "exception:unknown";
        return false;
    }
}

void OverlayPoseGraph::updatePinSummaryFromVertices(MapGraph& graph)
{
    if (graph.vertices.empty())
        return;
    unsigned long latest_id = graph.vertices.begin()->first;
    for (const auto& kv : graph.vertices) {
        if (!kv.second.frozen && kv.first > latest_id)
            latest_id = kv.first;
    }

    const KeyframeVertex& vtx = graph.vertices.at(latest_id);
    const Sophus::SE3f Twc = twcFromSim3Vertex(vtx.Siw);
    graph.pin.scale = clampScaleAbsolute(vtx.Siw.scale());
    graph.pin.R_domain_slam = Twc.so3().matrix();
    graph.pin.t_domain_slam = Twc.translation();
}

void OverlayPoseGraph::updatePinCounts(MapSim3Pin& pin, const MapGraph& graph)
{
    pin.portal_corner_factor_count = graph.portal_corners.size();
    pin.keyframe_vertex_count = graph.vertices.size();
    pin.frozen_keyframe_count = 0;
    for (const auto& kv : graph.vertices) {
        if (kv.second.frozen)
            pin.frozen_keyframe_count++;
    }
}

RefineResult OverlayPoseGraph::ingestPortalSight(unsigned long map_id,
                                                 const PortalSightInput& sight,
                                                 const AtlasSnapshot& atlas)
{
    std::lock_guard<std::mutex> lock(mutex_);
    RefineResult result;
    result.status = "portal_factors_accumulating";

    MapGraph& graph = graphFor(map_id);
    syncVerticesFromAtlas(graph, atlas);
    addPortalCornerFactors(map_id, sight, atlas);
    ensureFactorAnchorVertices(graph, atlas);

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
        syncVerticesFromAtlas(graph, atlas);
        result.scale_update_status = "initialized";
    }

    const MapSim3Pin pin_before_refine = pin;
    const bool refined = refinePoseGraph(graph, atlas, "portal_corner");
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
                                                      const PriorSoftEdgeFactor& factor,
                                                      const AtlasSnapshot& atlas)
{
    std::lock_guard<std::mutex> lock(mutex_);
    RefineResult result;
    MapGraph& graph = graphFor(map_id);
    if (factor.observations.empty())
        return result;

    PriorSoftEdgeFactor stored = factor;
    stored.vertex_keyframe_id = factor.vertex_keyframe_id;
    stored.Tcr = factor.Tcr;

    graph.prior_soft_factors.push_back(stored);
    if (graph.prior_soft_factors.size() > kMaxPriorSoftFactorsPerMap) {
        const size_t overflow = graph.prior_soft_factors.size() - kMaxPriorSoftFactorsPerMap;
        graph.prior_soft_factors.erase(
            graph.prior_soft_factors.begin(),
            graph.prior_soft_factors.begin() + static_cast<std::ptrdiff_t>(overflow));
    }
    if (!stored.Tcr.matrix().isApprox(Eigen::Matrix4f::Identity(), 1e-5f))
        graph.pin.dense_observation_count++;
    prior_soft_edges_accepted_++;
    last_prior_soft_edge_ = stored;

    MapSim3Pin& pin = graph.pin;
    if (pin.seeded && !graph.portal_corners.empty()) {
        if (refinePoseGraph(graph, atlas, "prior_soft")) {
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
    graphFor(map_id).pin = pin;
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

    const MapGraph& graph = it->second;
    if (graph.vertices.empty()) {
        const Eigen::Matrix3f R = graph.pin.R_domain_slam * Twc_slam.so3().matrix();
        const Eigen::Vector3f t =
            graph.pin.scale * (graph.pin.R_domain_slam * Twc_slam.translation()) +
            graph.pin.t_domain_slam;
        return Sophus::SE3f(R, t);
    }

    const Eigen::Matrix3f R = graph.pin.R_domain_slam * Twc_slam.so3().matrix();
    const Eigen::Vector3f t =
        graph.pin.scale * (graph.pin.R_domain_slam * Twc_slam.translation()) + graph.pin.t_domain_slam;
    return Sophus::SE3f(R, t);
}

Sophus::SE3f OverlayPoseGraph::applyPinToTwc(unsigned long map_id,
                                             const Sophus::SE3f& Twc_slam,
                                             uint64_t frame_id,
                                             unsigned long ref_keyframe_id,
                                             const Sophus::SE3f& Tcr,
                                             bool is_keyframe) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = maps_.find(map_id);
    if (it == maps_.end() || !it->second.pin.pinned)
        return Twc_slam;

    const MapGraph& graph = it->second;
    const unsigned long vertex_kf = is_keyframe ? frame_id : ref_keyframe_id;

    const auto vtx_it = graph.vertices.find(vertex_kf);
    if (vtx_it == graph.vertices.end()) {
        const Eigen::Matrix3f R = graph.pin.R_domain_slam * Twc_slam.so3().matrix();
        const Eigen::Vector3f t =
            graph.pin.scale * (graph.pin.R_domain_slam * Twc_slam.translation()) +
            graph.pin.t_domain_slam;
        return Sophus::SE3f(R, t);
    }

    const Sophus::SE3f local_tcr = is_keyframe ? Sophus::SE3f() : Tcr;
    const Sophus::Sim3f obs = composeDenseObservationSim3(vtx_it->second.Siw, local_tcr);
    return twcFromSim3Vertex(obs);
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

    out.Twc = applyPinToTwc(map_id, Twc_slam);
    out.pose_source = "overlay";
    return out;
}

json OverlayPoseGraph::statusJson(unsigned long active_map_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    json out;
    out["enabled"] = true;
    out["active_map_id"] = active_map_id;
    out["model"] = "per_keyframe_sim3_pose_graph";

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
