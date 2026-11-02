#include "PortalObservationStore.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

#include <Eigen/Eigenvalues>
#include <sophus/se3.hpp>

namespace ORB_SLAM3 {

namespace {

constexpr float kMinScale = 0.01f;
constexpr float kMaxScale = 100.0f;
constexpr float kRefineLmLambda = 1e-3f;
constexpr float kHessianSigmaPx = 1.0f;
constexpr int kPooledSolveIterations = 8;
constexpr float kBehindCameraZ = 1e-4f;
/** Below this (relative to max eigenvalue) H is treated as rank-deficient. */
constexpr float kMinEigenvalueRel = 1e-6f;
/** Anti-cull: protect observing KFs for the newest N stored sights per map. */
constexpr size_t kMaxProtectedSightAnchors = 64;
/** Pooled LM weight per prior-soft 3D→2D obs (below portal corners). */
constexpr float kPriorSoftPooledBaseWeight = 0.35f;
constexpr float kPriorSoftPooledCauchyScalePx = 4.0f;
constexpr float kPriorSoftPooledCauchyScaleSq =
    kPriorSoftPooledCauchyScalePx * kPriorSoftPooledCauchyScalePx;

void transformSightTwcUnderScaledRotation(PortalCornerSight& sight,
                                          const Eigen::Matrix3f& R_domain_slam,
                                          const Eigen::Vector3f& t_domain_slam,
                                          float scale)
{
    Sophus::SE3f Twc(sight.R_wc_slam, sight.t_wc_slam);
    Twc.translation() *= scale;
    const Sophus::SE3f Tyw(R_domain_slam, t_domain_slam);
    const Sophus::SE3f Tyc = Tyw * Twc;
    sight.R_wc_slam = Tyc.rotationMatrix();
    sight.t_wc_slam = Tyc.translation();
    sight.pose_in_domain_gauge = true;
}

void transformPriorSoftTwcUnderScaledRotation(PriorSoftSight& sight,
                                              const Eigen::Matrix3f& R_domain_slam,
                                              const Eigen::Vector3f& t_domain_slam,
                                              float scale)
{
    Sophus::SE3f Twc(sight.R_wc_slam, sight.t_wc_slam);
    Twc.translation() *= scale;
    const Sophus::SE3f Tyw(R_domain_slam, t_domain_slam);
    const Sophus::SE3f Tyc = Tyw * Twc;
    sight.R_wc_slam = Tyc.rotationMatrix();
    sight.t_wc_slam = Tyc.translation();
    sight.pose_in_domain_gauge = true;
}

float cauchyWeightFromSqError(float sq_error, float cauchy_scale_sq)
{
    if (!std::isfinite(sq_error) || cauchy_scale_sq <= 0.0f)
        return 0.0f;
    return cauchy_scale_sq / (cauchy_scale_sq + sq_error);
}

bool priorSoftResolvableForPooledSolve(const PriorSoftSight& sight,
                                       bool exclude_unresolved,
                                       const PortalKeyframePoseResolver& resolve_pose)
{
    if (sight.kf_id == kUnresolvedPortalKeyframeId || sight.observations.empty())
        return false;
    if (!resolve_pose)
        return true;

    Eigen::Matrix3f R_wc;
    Eigen::Vector3f t_wc;
    if (resolve_pose(sight.kf_id, R_wc, t_wc))
        return true;
    return exclude_unresolved && sight.pose_in_domain_gauge;
}

bool sightResolvableForPooledSolve(const PortalCornerSight& sight,
                                   bool exclude_unresolved,
                                   const PortalKeyframePoseResolver& resolve_pose)
{
    if (sight.kf_id == kUnresolvedPortalKeyframeId)
        return false;

    if (!resolve_pose)
        return true;

    Eigen::Matrix3f R_wc;
    Eigen::Vector3f t_wc;
    if (resolve_pose(sight.kf_id, R_wc, t_wc))
        return true;

    // Degraded: bound sight whose keyframe was culled — frozen domain snapshot only.
    return exclude_unresolved && sight.pose_in_domain_gauge;
}

static const char* kPinParamNames[] = {
    "omega_x", "omega_y", "omega_z", "log_scale", "t_x", "t_y", "t_z"};

Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& v)
{
    Eigen::Matrix3f m;
    m << 0.0f, -v.z(), v.y(), v.z(), 0.0f, -v.x(), -v.y(), v.x(), 0.0f;
    return m;
}

float clampScaleAbsolute(float scale)
{
    if (!std::isfinite(scale))
        return 1.0f;
    return std::max(kMinScale, std::min(kMaxScale, scale));
}

struct Sim3PinState {
    Eigen::Matrix3f R_domain_slam = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t_domain_slam = Eigen::Vector3f::Zero();
    float scale = 1.0f;
};

template <typename SightLike>
void twcDomainFromPinGeneric(const Sim3PinState& pin,
                             const SightLike& sight,
                             Eigen::Matrix3f& R_wc,
                             Eigen::Vector3f& t_wc)
{
    const Eigen::Matrix3f R_slam = sight.R_wc_slam;
    const Eigen::Vector3f t_slam = sight.t_wc_slam;
    R_wc = pin.R_domain_slam * R_slam;
    t_wc = pin.scale * (pin.R_domain_slam * t_slam) + pin.t_domain_slam;
}

void twcDomainFromPin(const Sim3PinState& pin,
                      const PortalCornerSight& sight,
                      Eigen::Matrix3f& R_wc,
                      Eigen::Vector3f& t_wc)
{
    twcDomainFromPinGeneric(pin, sight, R_wc, t_wc);
}

template <typename SightLike>
Eigen::Matrix<float, 3, 7> jacobianPCamWrtPinGeneric(const Sim3PinState& pin,
                                                     const SightLike& sight,
                                                     const Eigen::Vector3f& p_domain,
                                                     const Eigen::Matrix3f& R_cw)
{
    const Eigen::Matrix3f R_slam = sight.R_wc_slam;
    const Eigen::Vector3f t_slam = sight.t_wc_slam;

    Eigen::Matrix<float, 3, 7> J;
    J.setZero();
    J.block<3, 3>(0, 0) = R_cw * skewSymmetric(p_domain - pin.t_domain_slam);
    J.col(3) = -pin.scale * (R_slam.transpose() * t_slam);
    J.block<3, 3>(0, 4) = -R_cw;
    return J;
}

Eigen::Matrix<float, 3, 7> jacobianPCamWrtPinCorner(const Sim3PinState& pin,
                                                      const PortalCornerSight& sight,
                                                      const Eigen::Vector3f& p_domain,
                                                      const Eigen::Matrix3f& R_cw)
{
    return jacobianPCamWrtPinGeneric(pin, sight, p_domain, R_cw);
}

bool accumulateCornerHessianUnitWeight(const Sim3PinState& pin,
                                       const PortalCornerSight& sight,
                                       size_t corner_index,
                                       Eigen::Matrix<float, 7, 7>& H,
                                       size_t& residual_count)
{
    const Eigen::Vector3f& p_domain = sight.corners_domain[corner_index];
    Eigen::Matrix3f R_wc;
    Eigen::Vector3f t_wc;
    twcDomainFromPin(pin, sight, R_wc, t_wc);
    const Eigen::Matrix3f R_cw = R_wc.transpose();
    const Eigen::Vector3f p_cam = R_cw * (p_domain - t_wc);
    const float z = p_cam.z();
    if (!p_cam.allFinite() || z <= kBehindCameraZ)
        return false;

    Eigen::Matrix<float, 2, 3> J_proj;
    J_proj << sight.fx / z, 0.0f, -sight.fx * p_cam.x() / (z * z), 0.0f, sight.fy / z,
        -sight.fy * p_cam.y() / (z * z);
    const Eigen::Matrix<float, 2, 7> J =
        J_proj * jacobianPCamWrtPinCorner(pin, sight, p_domain, R_cw);
    if (!J.allFinite())
        return false;
    H += J.transpose() * J;
    ++residual_count;
    return true;
}

void twcDomainFromPinPriorSoft(const Sim3PinState& pin,
                               const PriorSoftSight& sight,
                               Eigen::Matrix3f& R_wc,
                               Eigen::Vector3f& t_wc)
{
    twcDomainFromPinGeneric(pin, sight, R_wc, t_wc);
}

bool accumulatePriorSoftHessianUnitWeight(const Sim3PinState& pin,
                                          const PriorSoftSight& sight,
                                          size_t obs_index,
                                          Eigen::Matrix<float, 7, 7>& H,
                                          size_t& residual_count)
{
    if (obs_index >= sight.observations.size())
        return false;
    const PriorSoftObservation3D2D& obs = sight.observations[obs_index];
    const Eigen::Vector3f& p_domain = obs.p_domain;
    Eigen::Matrix3f R_wc;
    Eigen::Vector3f t_wc;
    twcDomainFromPinPriorSoft(pin, sight, R_wc, t_wc);
    const Eigen::Matrix3f R_cw = R_wc.transpose();
    const Eigen::Vector3f p_cam = R_cw * (p_domain - t_wc);
    const float z = p_cam.z();
    if (!p_cam.allFinite() || z <= kBehindCameraZ)
        return false;

    Eigen::Matrix<float, 2, 3> J_proj;
    J_proj << sight.fx / z, 0.0f, -sight.fx * p_cam.x() / (z * z), 0.0f, sight.fy / z,
        -sight.fy * p_cam.y() / (z * z);
    const Eigen::Matrix<float, 2, 7> J =
        J_proj * jacobianPCamWrtPinGeneric(pin, sight, p_domain, R_cw);
    if (!J.allFinite())
        return false;
    H += J.transpose() * J;
    ++residual_count;
    return true;
}

bool accumulatePriorSoftFactor(const Sim3PinState& pin,
                               const PriorSoftSight& sight,
                               size_t obs_index,
                               Eigen::Matrix<float, 7, 7>& H,
                               Eigen::Matrix<float, 7, 1>& b,
                               float& obs_sq,
                               float& obs_w)
{
    if (obs_index >= sight.observations.size())
        return false;
    const PriorSoftObservation3D2D& obs = sight.observations[obs_index];
    const Eigen::Vector3f& p_domain = obs.p_domain;
    const Eigen::Vector2f& pixel = obs.pixel;
    Eigen::Matrix3f R_wc;
    Eigen::Vector3f t_wc;
    twcDomainFromPinPriorSoft(pin, sight, R_wc, t_wc);
    const Eigen::Matrix3f R_cw = R_wc.transpose();
    const Eigen::Vector3f p_cam = R_cw * (p_domain - t_wc);
    const float z = p_cam.z();
    if (!p_cam.allFinite() || z <= kBehindCameraZ)
        return false;

    Eigen::Matrix<float, 2, 3> J_proj;
    J_proj << sight.fx / z, 0.0f, -sight.fx * p_cam.x() / (z * z), 0.0f, sight.fy / z,
        -sight.fy * p_cam.y() / (z * z);
    const Eigen::Matrix<float, 2, 7> J =
        J_proj * jacobianPCamWrtPinGeneric(pin, sight, p_domain, R_cw);
    if (!J.allFinite())
        return false;

    const Eigen::Vector2f projected(sight.fx * p_cam.x() / z + sight.cx,
                                    sight.fy * p_cam.y() / z + sight.cy);
    const Eigen::Vector2f residual = projected - pixel;
    const float sq = residual.squaredNorm();
    const float base_w = sight.weight > 0.0f ? sight.weight : kPriorSoftPooledBaseWeight;
    const float w = base_w * cauchyWeightFromSqError(sq, kPriorSoftPooledCauchyScaleSq);
    if (w <= 1e-8f)
        return false;
    H += w * J.transpose() * J;
    b += w * J.transpose() * residual;
    obs_sq += w * sq;
    obs_w += w;
    return true;
}

bool accumulateCornerFactor(const Sim3PinState& pin,
                            const PortalCornerSight& sight,
                            size_t corner_index,
                            Eigen::Matrix<float, 7, 7>& H,
                            Eigen::Matrix<float, 7, 1>& b,
                            float& corner_sq,
                            float& corner_w)
{
    const Eigen::Vector3f& p_domain = sight.corners_domain[corner_index];
    const Eigen::Vector2f& pixel = sight.corners_pixel[corner_index];
    Eigen::Matrix3f R_wc;
    Eigen::Vector3f t_wc;
    twcDomainFromPin(pin, sight, R_wc, t_wc);
    const Eigen::Matrix3f R_cw = R_wc.transpose();
    const Eigen::Vector3f p_cam = R_cw * (p_domain - t_wc);
    const float z = p_cam.z();
    if (!p_cam.allFinite() || z <= kBehindCameraZ)
        return false;

    Eigen::Matrix<float, 2, 3> J_proj;
    J_proj << sight.fx / z, 0.0f, -sight.fx * p_cam.x() / (z * z), 0.0f, sight.fy / z,
        -sight.fy * p_cam.y() / (z * z);
    const Eigen::Matrix<float, 2, 7> J =
        J_proj * jacobianPCamWrtPinCorner(pin, sight, p_domain, R_cw);
    if (!J.allFinite())
        return false;

    const Eigen::Vector2f projected(sight.fx * p_cam.x() / z + sight.cx,
                                    sight.fy * p_cam.y() / z + sight.cy);
    const Eigen::Vector2f residual = projected - pixel;
    const float sq = residual.squaredNorm();
    const float w = sight.weight;
    if (w <= 1e-8f)
        return false;
    H += w * J.transpose() * J;
    b += w * J.transpose() * residual;
    corner_sq += w * sq;
    corner_w += w;
    return true;
}

float meanReprojPxAtPin(const Sim3PinState& pin,
                        const std::vector<PortalCornerSight>& portal_sights,
                        const std::vector<PriorSoftSight>& prior_soft_sights,
                        bool weighted)
{
    float sq_sum = 0.0f;
    float w_sum = 0.0f;
    for (const PortalCornerSight& sight : portal_sights) {
        for (size_t i = 0; i < 4; ++i) {
            const Eigen::Vector3f& p_domain = sight.corners_domain[i];
            const Eigen::Vector2f& pixel = sight.corners_pixel[i];
            Eigen::Matrix3f R_wc;
            Eigen::Vector3f t_wc;
            twcDomainFromPin(pin, sight, R_wc, t_wc);
            const Eigen::Matrix3f R_cw = R_wc.transpose();
            const Eigen::Vector3f p_cam = R_cw * (p_domain - t_wc);
            const float z = p_cam.z();
            if (!p_cam.allFinite() || z <= kBehindCameraZ)
                continue;
            const Eigen::Vector2f projected(sight.fx * p_cam.x() / z + sight.cx,
                                            sight.fy * p_cam.y() / z + sight.cy);
            const float sq = (projected - pixel).squaredNorm();
            if (weighted) {
                sq_sum += sight.weight * sq;
                w_sum += sight.weight;
            } else {
                sq_sum += sq;
                w_sum += 1.0f;
            }
        }
    }
    for (const PriorSoftSight& sight : prior_soft_sights) {
        const float base_w = sight.weight > 0.0f ? sight.weight : kPriorSoftPooledBaseWeight;
        for (const PriorSoftObservation3D2D& obs : sight.observations) {
            Eigen::Matrix3f R_wc;
            Eigen::Vector3f t_wc;
            twcDomainFromPinPriorSoft(pin, sight, R_wc, t_wc);
            const Eigen::Matrix3f R_cw = R_wc.transpose();
            const Eigen::Vector3f p_cam = R_cw * (obs.p_domain - t_wc);
            const float z = p_cam.z();
            if (!p_cam.allFinite() || z <= kBehindCameraZ)
                continue;
            const Eigen::Vector2f projected(sight.fx * p_cam.x() / z + sight.cx,
                                            sight.fy * p_cam.y() / z + sight.cy);
            const float sq = (projected - obs.pixel).squaredNorm();
            const float w =
                weighted ? base_w * cauchyWeightFromSqError(sq, kPriorSoftPooledCauchyScaleSq)
                         : 1.0f;
            sq_sum += w * sq;
            w_sum += w;
        }
    }
    if (w_sum <= 0.0f)
        return 0.0f;
    return std::sqrt(sq_sum / w_sum);
}

Sim3HessianObservability observabilityFromUnitHessian(const Eigen::Matrix<float, 7, 7>& H,
                                                      size_t residual_count)
{
    Sim3HessianObservability obs;
    obs.residual_count = residual_count;
    const float nan_sigma = std::numeric_limits<float>::quiet_NaN();
    obs.sigma_log_s = nan_sigma;
    for (int i = 0; i < 7; ++i)
        obs.sigma_params[i] = nan_sigma;

    if (residual_count == 0 || !H.allFinite())
        return obs;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 7, 7>> es(H);
    if (es.info() != Eigen::Success)
        return obs;

    const Eigen::Matrix<float, 7, 1> ev = es.eigenvalues();
    for (int i = 0; i < 7; ++i)
        obs.eigenvalues[i] = ev(i);

    obs.min_eigenvalue = ev.minCoeff();
    const float ev_max = ev.maxCoeff();
    obs.condition_number = ev_max / std::max(obs.min_eigenvalue, 1e-30f);

    int min_i = 0;
    ev.minCoeff(&min_i);
    const Eigen::Matrix<float, 7, 1> null_vec = es.eigenvectors().col(min_i);
    for (int i = 0; i < 7; ++i)
        obs.null_direction_loadings[i] = null_vec(i);

    int best = 0;
    float best_abs = 0.0f;
    for (int i = 0; i < 7; ++i) {
        const float a = std::abs(null_vec(i));
        if (a > best_abs) {
            best_abs = a;
            best = i;
        }
    }
    obs.null_dominant_param = kPinParamNames[best];

    const bool rank_deficient =
        residual_count < 7 ||
        obs.min_eigenvalue < kMinEigenvalueRel * std::max(ev_max, 1e-30f) ||
        obs.null_dominant_param == "log_scale";

    if (!rank_deficient) {
        const Eigen::Matrix<float, 7, 7> H_pinv =
            H.completeOrthogonalDecomposition().pseudoInverse();
        for (int i = 0; i < 7; ++i) {
            const float hinv = H_pinv(i, i);
            if (std::isfinite(hinv) && hinv > 0.0f)
                obs.sigma_params[i] = kHessianSigmaPx * std::sqrt(hinv);
        }
        const float hinv_log_s = H_pinv(3, 3);
        if (std::isfinite(hinv_log_s) && hinv_log_s > 0.0f)
            obs.sigma_log_s = kHessianSigmaPx * std::sqrt(hinv_log_s);
        obs.valid = std::isfinite(obs.sigma_log_s);
    }

    return obs;
}

Sim3HessianObservability unitHessianObservabilityAtPin(
    const Sim3PinState& pin,
    const std::vector<PortalCornerSight>& portal_sights,
    const std::vector<PriorSoftSight>& prior_soft_sights)
{
    Eigen::Matrix<float, 7, 7> H = Eigen::Matrix<float, 7, 7>::Zero();
    size_t residual_count = 0;
    for (const PortalCornerSight& sight : portal_sights) {
        for (size_t i = 0; i < 4; ++i) {
            accumulateCornerHessianUnitWeight(pin, sight, i, H, residual_count);
        }
    }
    for (const PriorSoftSight& sight : prior_soft_sights) {
        for (size_t i = 0; i < sight.observations.size(); ++i) {
            accumulatePriorSoftHessianUnitWeight(pin, sight, i, H, residual_count);
        }
    }
    return observabilityFromUnitHessian(H, residual_count);
}

Sim3PinState seedPinFromEvidence(const std::vector<PortalCornerSight>& portal_sights,
                                 const std::vector<PriorSoftSight>& prior_soft_sights)
{
    Sim3PinState pin;
    if (!portal_sights.empty()) {
        const PortalCornerSight& first = portal_sights.front();
        pin.R_domain_slam = first.R_domain_slam;
        pin.t_domain_slam = first.t_domain_slam_seed;
        pin.scale = 1.0f;
        return pin;
    }
    if (!prior_soft_sights.empty()) {
        const PriorSoftSight& first = prior_soft_sights.front();
        pin.R_domain_slam = first.R_domain_slam;
        pin.t_domain_slam = first.t_domain_slam_seed;
        pin.scale = 1.0f;
    }
    return pin;
}

Sim3PinState operatingPinFromOptions(const PooledSim3SolveOptions& options)
{
    Sim3PinState pin;
    pin.R_domain_slam = options.operating_R;
    pin.t_domain_slam = options.operating_t;
    pin.scale = options.operating_scale;
    return pin;
}

void mirrorSeedToTopLevel(PooledSim3ObservabilityReport& report)
{
    const Sim3HessianObservability& h = report.seed_pin.hessian;
    report.valid = h.valid;
    report.residual_count = h.residual_count;
    report.hessian_eigenvalues = h.eigenvalues;
    report.condition_number = h.condition_number;
    report.null_dominant_param = h.null_dominant_param;
    report.null_direction_loadings = h.null_direction_loadings;
    report.sigma_log_s = h.sigma_log_s;
    report.scale = report.seed_pin.scale;
    report.mean_reproj_px = report.seed_pin.mean_reproj_px;
}

PooledSim3ObservabilityReport solvePooledFromSights(unsigned long map_id,
                                                     const std::vector<PortalCornerSight>& portal_sights,
                                                     const std::vector<PriorSoftSight>& prior_soft_sights,
                                                     const PooledSim3SolveOptions& options)
{
    PooledSim3ObservabilityReport report;
    report.map_id = map_id;
    report.portal_sight_count = portal_sights.size();
    report.prior_soft_sight_count = prior_soft_sights.size();
    report.sight_count = portal_sights.size() + prior_soft_sights.size();
    if (portal_sights.empty() && prior_soft_sights.empty())
        return report;

    size_t prior_soft_residual_count = 0;
    for (const PriorSoftSight& sight : prior_soft_sights)
        prior_soft_residual_count += sight.observations.size();
    report.prior_soft_residual_count = prior_soft_residual_count;

    const bool incremental =
        options.gauge == PooledSim3SolveOptions::Gauge::IncrementalDomain;
    report.solve_gauge = incremental ? "incremental" : "absolute";

    const Sim3PinState seed_pin = incremental ? operatingPinFromOptions(options)
                                              : seedPinFromEvidence(portal_sights, prior_soft_sights);
    Sim3PinState pin = seed_pin;

    report.seed_pin.R_domain_slam = seed_pin.R_domain_slam;
    report.seed_pin.t_domain_slam = seed_pin.t_domain_slam;
    report.seed_pin.scale = seed_pin.scale;
    report.seed_pin.mean_reproj_px =
        meanReprojPxAtPin(seed_pin, portal_sights, prior_soft_sights, true);
    report.seed_pin.hessian =
        unitHessianObservabilityAtPin(seed_pin, portal_sights, prior_soft_sights);

    for (int iter = 0; iter < kPooledSolveIterations; ++iter) {
        Eigen::Matrix<float, 7, 7> H = Eigen::Matrix<float, 7, 7>::Zero();
        Eigen::Matrix<float, 7, 1> b = Eigen::Matrix<float, 7, 1>::Zero();
        float corner_sq = 0.0f;
        float corner_w = 0.0f;

        for (const PortalCornerSight& sight : portal_sights) {
            for (size_t i = 0; i < 4; ++i) {
                accumulateCornerFactor(pin, sight, i, H, b, corner_sq, corner_w);
            }
        }
        for (const PriorSoftSight& sight : prior_soft_sights) {
            for (size_t i = 0; i < sight.observations.size(); ++i) {
                accumulatePriorSoftFactor(pin, sight, i, H, b, corner_sq, corner_w);
            }
        }

        H.diagonal().array() += kRefineLmLambda * (H.diagonal().array().abs() + 1.0f);
        if (H.diagonal().maxCoeff() <= 1e-12f)
            break;

        Eigen::Matrix<float, 7, 1> delta = H.ldlt().solve(-b);
        if (!delta.allFinite())
            break;

        const Eigen::Matrix3f dR = Sophus::SO3f::exp(delta.segment<3>(0)).matrix();
        pin.R_domain_slam = (dR * pin.R_domain_slam).eval();
        pin.scale = clampScaleAbsolute(pin.scale * std::exp(delta(3)));
        pin.t_domain_slam += delta.segment<3>(4);
    }

    report.solved_pin.R_domain_slam = pin.R_domain_slam;
    report.solved_pin.t_domain_slam = pin.t_domain_slam;
    report.solved_pin.scale = pin.scale;
    report.solved_pin.mean_reproj_px =
        meanReprojPxAtPin(pin, portal_sights, prior_soft_sights, true);
    report.solved_pin.hessian =
        unitHessianObservabilityAtPin(pin, portal_sights, prior_soft_sights);

    mirrorSeedToTopLevel(report);
    return report;
}

void printFloatOrNan(std::ostream& os, float v)
{
    if (std::isnan(v))
        os << "nan";
    else
        os << v;
}

void logPooledSim3Report(const PooledSim3ObservabilityReport& report)
{
    const Sim3HessianObservability& seed = report.seed_pin.hessian;
    const Sim3HessianObservability& solved = report.solved_pin.hessian;
    std::cout << "[portal] pooled_sim3 map=" << report.map_id
              << " gauge=" << report.solve_gauge
              << " sights=" << report.sight_count
              << " portal_sights=" << report.portal_sight_count
              << " prior_soft_sights=" << report.prior_soft_sight_count
              << " prior_soft_residuals=" << report.prior_soft_residual_count
              << " seed_valid=" << (seed.valid ? 1 : 0)
              << " seed_residuals=" << seed.residual_count
              << " seed_cond=" << seed.condition_number
              << " seed_null=" << seed.null_dominant_param << " seed_sigma_log_s=";
    printFloatOrNan(std::cout, seed.sigma_log_s);
    std::cout << " seed_reproj_rmse_px=" << report.seed_pin.mean_reproj_px
              << " solved_scale=" << report.solved_pin.scale
              << " solved_reproj_rmse_px=" << report.solved_pin.mean_reproj_px
              << " solved_cond=" << solved.condition_number
              << " solved_null=" << solved.null_dominant_param << " solved_sigma_log_s=";
    printFloatOrNan(std::cout, solved.sigma_log_s);
    std::cout << std::endl;
}

} // namespace

PortalObservationStore& PortalObservationStore::Instance()
{
    static PortalObservationStore store;
    return store;
}

size_t PortalObservationStore::tryAppendPriorSoft(const PriorSoftSight& sight)
{
    if (sight.kf_id == kUnresolvedPortalKeyframeId || sight.observations.empty()) {
        std::cout << "[prior_soft_store] reject unbound map=" << sight.map_id
                  << " frame=" << sight.frame_id
                  << " prior_image=" << sight.prior_image_id
                  << " reason=no_observing_keyframe_or_empty" << std::endl;
        return 0;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    prior_soft_sights_.push_back(sight);
    rebuildProtectedKeyframes(sight.map_id);
    const size_t total = prior_soft_sights_.size();
    std::cout << "[prior_soft_store] append map=" << sight.map_id
              << " kf=" << sight.kf_id
              << " frame=" << sight.frame_id
              << " prior_image=" << sight.prior_image_id
              << " obs=" << sight.observations.size()
              << " weight=" << sight.weight
              << " total=" << total << std::endl;
    return total;
}

size_t PortalObservationStore::tryAppend(const PortalCornerSight& sight)
{
    if (sight.kf_id == kUnresolvedPortalKeyframeId) {
        std::cout << "[portal_store] reject unbound map=" << sight.map_id
                  << " frame=" << sight.frame_id
                  << " id=" << sight.short_id
                  << " reason=no_observing_keyframe" << std::endl;
        return 0;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    observations_.push_back(sight);
    rebuildProtectedKeyframes(sight.map_id);
    const size_t total = observations_.size();
    std::cout << "[portal_store] append map=" << sight.map_id
              << " kf=" << sight.kf_id
              << " frame=" << sight.frame_id
              << " id=" << sight.short_id
              << " weight=" << sight.weight
              << " total=" << total << std::endl;
    return total;
}

size_t PortalObservationStore::purgeUnboundSights(unsigned long map_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t before = observations_.size();
    observations_.erase(
        std::remove_if(
            observations_.begin(),
            observations_.end(),
            [map_id](const PortalCornerSight& sight) {
                return sight.map_id == map_id &&
                       sight.kf_id == kUnresolvedPortalKeyframeId;
            }),
        observations_.end());
    const size_t purged = before - observations_.size();
    if (purged > 0) {
        rebuildProtectedKeyframes(map_id);
        std::cout << "[portal_store] purged unbound map=" << map_id
                  << " n=" << purged << std::endl;
    }
    return purged;
}

void PortalObservationStore::rebuildProtectedKeyframes(unsigned long map_id)
{
    std::unordered_set<unsigned long> next;
    size_t sights_seen = 0;
    for (auto it = prior_soft_sights_.rbegin(); it != prior_soft_sights_.rend(); ++it) {
        if (it->map_id != map_id)
            continue;
        if (it->kf_id != kUnresolvedPortalKeyframeId)
            next.insert(it->kf_id);
        ++sights_seen;
        if (sights_seen >= kMaxProtectedSightAnchors)
            break;
    }
    for (auto it = observations_.rbegin(); it != observations_.rend(); ++it) {
        if (it->map_id != map_id)
            continue;
        if (it->kf_id != kUnresolvedPortalKeyframeId)
            next.insert(it->kf_id);
        ++sights_seen;
        if (sights_seen >= kMaxProtectedSightAnchors)
            break;
    }
    protected_kf_ids_[map_id] = std::move(next);
}

bool PortalObservationStore::isKeyframeAntiCulled(unsigned long map_id,
                                                unsigned long kf_id) const
{
    if (kf_id == kUnresolvedPortalKeyframeId)
        return false;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto map_it = protected_kf_ids_.find(map_id);
    if (map_it == protected_kf_ids_.end())
        return false;
    return map_it->second.find(kf_id) != map_it->second.end();
}

void PortalObservationStore::applyScaledRotationToSightPoses(unsigned long map_id,
                                                             const Eigen::Matrix3f& R_domain_slam,
                                                             const Eigen::Vector3f& t_domain_slam,
                                                             float scale)
{
    std::lock_guard<std::mutex> lock(mutex_);
    size_t updated = 0;
    for (PortalCornerSight& sight : observations_) {
        if (sight.map_id != map_id || sight.kf_id == kUnresolvedPortalKeyframeId)
            continue;
        transformSightTwcUnderScaledRotation(sight, R_domain_slam, t_domain_slam, scale);
        ++updated;
    }
    size_t prior_updated = 0;
    for (PriorSoftSight& sight : prior_soft_sights_) {
        if (sight.map_id != map_id || sight.kf_id == kUnresolvedPortalKeyframeId)
            continue;
        transformPriorSoftTwcUnderScaledRotation(sight, R_domain_slam, t_domain_slam, scale);
        ++prior_updated;
    }
    if (updated > 0 || prior_updated > 0) {
        std::cout << "[portal_store] sight_poses domain_gauge map=" << map_id
                  << " portal_updated=" << updated
                  << " prior_soft_updated=" << prior_updated << std::endl;
    }
}

PortalSightResolvability PortalObservationStore::countResolvableSights(
    unsigned long map_id,
    const PortalKeyframePoseResolver& resolve_pose,
    const PooledSim3SolveOptions& options) const
{
    PortalSightResolvability counts;
    const bool exclude_unresolved =
        options.gauge == PooledSim3SolveOptions::Gauge::IncrementalDomain;
    std::lock_guard<std::mutex> lock(mutex_);
    for (const PortalCornerSight& sight : observations_) {
        if (sight.map_id != map_id)
            continue;
        ++counts.stored;
        if (sightResolvableForPooledSolve(sight, exclude_unresolved, resolve_pose))
            ++counts.resolvable;
    }
    for (const PriorSoftSight& sight : prior_soft_sights_) {
        if (sight.map_id != map_id)
            continue;
        ++counts.stored;
        if (priorSoftResolvableForPooledSolve(sight, exclude_unresolved, resolve_pose))
            ++counts.resolvable;
    }
    return counts;
}

PortalSightResolvability PortalObservationStore::lastResolvability(unsigned long map_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = last_resolvability_.find(map_id);
    if (it != last_resolvability_.end())
        return it->second;
    PortalSightResolvability counts;
    for (const PortalCornerSight& sight : observations_) {
        if (sight.map_id == map_id)
            ++counts.stored;
    }
    return counts;
}

std::vector<PortalCornerSight> PortalObservationStore::queryByMap(unsigned long map_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<PortalCornerSight> out;
    for (const PortalCornerSight& sight : observations_) {
        if (sight.map_id == map_id)
            out.push_back(sight);
    }
    return out;
}

std::vector<PortalCornerSight> PortalObservationStore::queryByKeyframeIds(
    const std::unordered_set<unsigned long>& kf_ids) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<PortalCornerSight> out;
    for (const PortalCornerSight& sight : observations_) {
        if (kf_ids.find(sight.kf_id) != kf_ids.end())
            out.push_back(sight);
    }
    return out;
}

std::vector<PortalCornerSight> PortalObservationStore::queryByMapAndKeyframeIds(
    unsigned long map_id,
    const std::unordered_set<unsigned long>& kf_ids) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<PortalCornerSight> out;
    for (const PortalCornerSight& sight : observations_) {
        if (sight.map_id == map_id && kf_ids.find(sight.kf_id) != kf_ids.end())
            out.push_back(sight);
    }
    return out;
}

std::vector<PriorSoftSight> PortalObservationStore::queryPriorSoftByMapAndKeyframeIds(
    unsigned long map_id,
    const std::unordered_set<unsigned long>& kf_ids) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<PriorSoftSight> out;
    for (const PriorSoftSight& sight : prior_soft_sights_) {
        if (sight.map_id == map_id && kf_ids.find(sight.kf_id) != kf_ids.end())
            out.push_back(sight);
    }
    return out;
}

size_t PortalObservationStore::count() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return observations_.size();
}

size_t PortalObservationStore::countByMap(unsigned long map_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<size_t>(std::count_if(
        observations_.begin(),
        observations_.end(),
        [map_id](const PortalCornerSight& sight) { return sight.map_id == map_id; }));
}

size_t PortalObservationStore::countPriorSoftByMap(unsigned long map_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<size_t>(std::count_if(
        prior_soft_sights_.begin(),
        prior_soft_sights_.end(),
        [map_id](const PriorSoftSight& sight) { return sight.map_id == map_id; }));
}

void PortalObservationStore::clearMap(unsigned long map_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    observations_.erase(
        std::remove_if(
            observations_.begin(),
            observations_.end(),
            [map_id](const PortalCornerSight& sight) { return sight.map_id == map_id; }),
        observations_.end());
    prior_soft_sights_.erase(
        std::remove_if(
            prior_soft_sights_.begin(),
            prior_soft_sights_.end(),
            [map_id](const PriorSoftSight& sight) { return sight.map_id == map_id; }),
        prior_soft_sights_.end());
    last_pooled_reports_.erase(map_id);
    last_resolvability_.erase(map_id);
    protected_kf_ids_.erase(map_id);
}

void PortalObservationStore::clearAll()
{
    std::lock_guard<std::mutex> lock(mutex_);
    observations_.clear();
    prior_soft_sights_.clear();
    last_pooled_reports_.clear();
    last_resolvability_.clear();
    protected_kf_ids_.clear();
}

std::vector<PortalCornerSight> resolveSightsForPooledSolve(
    const std::vector<PortalCornerSight>& raw,
    const PortalKeyframePoseResolver& resolve_pose,
    bool exclude_unresolved,
    size_t& skipped_unresolved)
{
    skipped_unresolved = 0;
    if (!resolve_pose)
        return raw;

    std::vector<PortalCornerSight> resolved;
    resolved.reserve(raw.size());
    for (const PortalCornerSight& sight : raw) {
        if (sight.kf_id == kUnresolvedPortalKeyframeId) {
            ++skipped_unresolved;
            continue;
        }
        Eigen::Matrix3f R_wc;
        Eigen::Vector3f t_wc;
        const bool live = resolve_pose(sight.kf_id, R_wc, t_wc);
        if (!live) {
            if (!(exclude_unresolved && sight.pose_in_domain_gauge)) {
                ++skipped_unresolved;
                continue;
            }
        }
        PortalCornerSight out = sight;
        if (live) {
            out.R_wc_slam = R_wc;
            out.t_wc_slam = t_wc;
        }
        resolved.push_back(out);
    }
    return resolved;
}

std::vector<PriorSoftSight> resolvePriorSoftForPooledSolve(
    const std::vector<PriorSoftSight>& raw,
    const PortalKeyframePoseResolver& resolve_pose,
    bool exclude_unresolved,
    size_t& skipped_unresolved)
{
    skipped_unresolved = 0;
    if (!resolve_pose)
        return raw;

    std::vector<PriorSoftSight> resolved;
    resolved.reserve(raw.size());
    for (const PriorSoftSight& sight : raw) {
        if (sight.kf_id == kUnresolvedPortalKeyframeId) {
            ++skipped_unresolved;
            continue;
        }
        Eigen::Matrix3f R_wc;
        Eigen::Vector3f t_wc;
        const bool live = resolve_pose(sight.kf_id, R_wc, t_wc);
        if (!live) {
            if (!(exclude_unresolved && sight.pose_in_domain_gauge)) {
                ++skipped_unresolved;
                continue;
            }
        }
        // Keep append-time Twc (updated on domainize / live-scale rewrite). Pixels were
        // observed at that pose; ref_kf_id is for anti-cull only — not the camera pose.
        resolved.push_back(sight);
    }
    return resolved;
}

PooledSim3ObservabilityReport PortalObservationStore::solvePooledSim3Diagnostics(
    unsigned long map_id,
    const PortalKeyframePoseResolver& resolve_pose,
    const PooledSim3SolveOptions& options)
{
    std::vector<PortalCornerSight> portal_sights;
    std::vector<PriorSoftSight> prior_soft_sights;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const PortalCornerSight& sight : observations_) {
            if (sight.map_id == map_id)
                portal_sights.push_back(sight);
        }
        for (const PriorSoftSight& sight : prior_soft_sights_) {
            if (sight.map_id == map_id)
                prior_soft_sights.push_back(sight);
        }
    }

    const bool exclude_unresolved =
        options.gauge == PooledSim3SolveOptions::Gauge::IncrementalDomain;
    size_t skipped_unresolved = 0;
    const std::vector<PortalCornerSight> resolved_portal = resolveSightsForPooledSolve(
        portal_sights, resolve_pose, exclude_unresolved, skipped_unresolved);
    size_t skipped_prior_soft = 0;
    const std::vector<PriorSoftSight> resolved_prior_soft = resolvePriorSoftForPooledSolve(
        prior_soft_sights, resolve_pose, exclude_unresolved, skipped_prior_soft);
    const size_t skipped_total = skipped_unresolved + skipped_prior_soft;
    if (skipped_total > 0) {
        std::cout << "[portal] pooled_sim3 map=" << map_id
                  << " skipped_unresolved_sights=" << skipped_unresolved
                  << " skipped_prior_soft=" << skipped_prior_soft << std::endl;
    }

    const PooledSim3ObservabilityReport report =
        solvePooledFromSights(map_id, resolved_portal, resolved_prior_soft, options);

    PortalSightResolvability resolvability;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const PortalCornerSight& sight : observations_) {
            if (sight.map_id != map_id)
                continue;
            ++resolvability.stored;
            if (sightResolvableForPooledSolve(sight, exclude_unresolved, resolve_pose))
                ++resolvability.resolvable;
        }
        for (const PriorSoftSight& sight : prior_soft_sights_) {
            if (sight.map_id != map_id)
                continue;
            ++resolvability.stored;
            if (priorSoftResolvableForPooledSolve(sight, exclude_unresolved, resolve_pose))
                ++resolvability.resolvable;
        }
        resolvability.solved_reproj_px = report.solved_pin.mean_reproj_px;
        last_pooled_reports_[map_id] = report;
        last_resolvability_[map_id] = resolvability;
    }

    std::cout << "[portal_store] resolvable map=" << map_id
              << " stored=" << resolvability.stored
              << " resolvable=" << resolvability.resolvable
              << " solved_reproj_rmse_px=" << resolvability.solved_reproj_px << std::endl;
    logPooledSim3Report(report);
    return report;
}

PooledSim3ObservabilityReport PortalObservationStore::pooledObservability(
    unsigned long map_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = last_pooled_reports_.find(map_id);
    if (it != last_pooled_reports_.end())
        return it->second;
    return PooledSim3ObservabilityReport{};
}

} // namespace ORB_SLAM3
