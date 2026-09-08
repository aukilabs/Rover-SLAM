#include "prior_soft_edges.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <opencv2/calib3d.hpp>

#ifdef AUKI_LIGHTGLUE_AVAILABLE
#include "Matchers/LightGlueMrNerfMatcher.h"
#endif

namespace prior_soft {

namespace {

constexpr float kPriorSoftBaseWeight = 0.35f;
constexpr size_t kMaxPriorSoftObservationsPerFactor = 64;

Sophus::SE3f poseToTwc(const prior_pack::Pose& pose) {
    Eigen::Quaternionf q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
    q.normalize();
    const Eigen::Vector3f t(pose.position.x, pose.position.y, pose.position.z);
    return Sophus::SE3f(q.toRotationMatrix(), t);
}

Eigen::Vector3f cameraCenter(const Sophus::SE3f& Twc) {
    return Twc.translation();
}

float poseCenterDistanceM(const prior_pack::ImageRecord& img, const Sophus::SE3f& Twc_domain) {
    const Sophus::SE3f Twc_prior = poseToTwc(img.pose);
    return (cameraCenter(Twc_prior) - cameraCenter(Twc_domain)).norm();
}

const prior_pack::ImageRecord* imageById(const prior_pack::PriorPack& pack, int image_id) {
    for (const auto& img : pack.images) {
        if (img.id == image_id)
            return &img;
    }
    return nullptr;
}

std::vector<int> covisNeighbors(const prior_pack::PriorPack& pack, int image_id) {
    std::vector<std::pair<int, int>> neighbors;
    for (const auto& edge : pack.covis_edges) {
        if (edge.i == image_id)
            neighbors.emplace_back(edge.j, edge.shared_tracks);
        else if (edge.j == image_id)
            neighbors.emplace_back(edge.i, edge.shared_tracks);
    }
    std::sort(neighbors.begin(), neighbors.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });
    std::vector<int> out;
    for (const auto& kv : neighbors)
        out.push_back(kv.first);
    return out;
}

int findNearestPriorImageId(const prior_pack::PriorPack& pack, const Sophus::SE3f& Twc_domain) {
    int best_id = -1;
    float best_dist = std::numeric_limits<float>::infinity();
    for (const auto& img : pack.images) {
        const float d = poseCenterDistanceM(img, Twc_domain);
        if (d < best_dist) {
            best_dist = d;
            best_id = img.id;
        }
    }
    return best_id;
}

std::vector<int> selectCandidateImages(const PriorMatchInput& input, const PriorMatchConfig& config) {
    const prior_pack::PriorPack& pack = *input.pack;
    if (pack.images.empty())
        return {};

    int seed = input.seed_prior_image_id;
    if (seed < 0) {
        if (input.has_domain_pose)
            seed = findNearestPriorImageId(pack, input.Twc_domain);
        else if (!pack.images.empty())
            seed = pack.images.front().id;
    }
    if (seed < 0)
        return {};

    std::unordered_set<int> seen;
    std::vector<int> candidates;
    candidates.push_back(seed);
    seen.insert(seed);

    for (const int neighbor : covisNeighbors(pack, seed)) {
        if (seen.count(neighbor))
            continue;
        if (input.has_domain_pose) {
            const prior_pack::ImageRecord* img = imageById(pack, neighbor);
            if (!img)
                continue;
            if (poseCenterDistanceM(*img, input.Twc_domain) > config.pose_proximity_m)
                continue;
        }
        candidates.push_back(neighbor);
        seen.insert(neighbor);
        if (candidates.size() >= config.max_candidates)
            break;
    }

    if (input.has_domain_pose && candidates.size() > 1) {
        const prior_pack::ImageRecord* seed_img = imageById(pack, seed);
        if (seed_img &&
            poseCenterDistanceM(*seed_img, input.Twc_domain) > config.pose_proximity_m) {
            candidates.erase(candidates.begin());
        }
    }

    return candidates;
}

cv::Mat descriptorsToMat(const std::vector<std::vector<float>>& rows) {
    if (rows.empty())
        return {};
    const int cols = static_cast<int>(rows.front().size());
    cv::Mat out(static_cast<int>(rows.size()), cols, CV_32F);
    for (int r = 0; r < out.rows; ++r) {
        if (static_cast<int>(rows[r].size()) != cols)
            return {};
        for (int c = 0; c < cols; ++c)
            out.at<float>(r, c) = rows[r][c];
    }
    return out;
}

std::vector<cv::Point2f> keypointsToPoints(const std::vector<std::pair<float, float>>& kpts) {
    std::vector<cv::Point2f> out;
    for (const auto& kp : kpts)
        out.emplace_back(kp.first, kp.second);
    return out;
}

std::vector<cv::Point2f> keypointsToPoints(const std::vector<cv::KeyPoint>& kpts) {
    std::vector<cv::Point2f> out;
    for (const auto& kp : kpts)
        out.push_back(kp.pt);
    return out;
}

bool matchDescriptorsFixture(
    const std::vector<cv::Point2f>& live_pts,
    const cv::Mat& live_desc,
    const std::vector<cv::Point2f>& prior_pts,
    const cv::Mat& prior_desc,
    float max_l2,
    std::vector<int>& matches12) {
    if (live_desc.empty() || prior_desc.empty() || live_desc.cols != prior_desc.cols)
        return false;

    matches12.assign(live_pts.size(), -1);
    int matched = 0;
    for (int i = 0; i < live_desc.rows; ++i) {
        float best = std::numeric_limits<float>::infinity();
        int best_j = -1;
        for (int j = 0; j < prior_desc.rows; ++j) {
            float dist = 0.f;
            for (int c = 0; c < live_desc.cols; ++c) {
                const float d = live_desc.at<float>(i, c) - prior_desc.at<float>(j, c);
                dist += d * d;
            }
            if (dist < best) {
                best = dist;
                best_j = j;
            }
        }
        if (best_j >= 0 && best <= max_l2) {
            matches12[i] = best_j;
            matched++;
        }
    }
    return matched > 0;
}

struct ReprojGateDiagnostics {
    int lg_matches = 0;
    int with_track = 0;
    int pass_live = 0;
    int pass_prior = 0;
    int pass_both = 0;
    float err_live_p50 = -1.f;
    float err_prior_p50 = -1.f;
    float mean_abs_z_live = -1.f;
};

struct ReprojGateResult {
    int inliers = 0;
    float mean_reproj_px = 0.f;
    std::vector<overlay::PriorSoftObservation> observations;
    ReprojGateDiagnostics diag;
};

float percentileP50(std::vector<float> values) {
    if (values.empty())
        return -1.f;
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    return values[mid];
}

ReprojGateResult gateWithTrackReprojection(
    const prior_pack::PriorPack& pack,
    int prior_image_id,
    const prior_pack::ImageRecord& prior_img,
    const std::vector<int>& live_to_prior,
    const std::vector<cv::KeyPoint>& live_kpts,
    const Sophus::SE3f& Twc_domain,
    const std::vector<float>& intrinsics,
    float max_reproj_px) {
    ReprojGateResult out;
    if (intrinsics.size() < 4)
        return out;

    const float fx = intrinsics[0];
    const float fy = intrinsics[1];
    const float cx = intrinsics[2];
    const float cy = intrinsics[3];
    const Sophus::SE3f Tcw_live = Twc_domain.inverse();
    const Eigen::Matrix3f R_live = Tcw_live.so3().matrix();
    const Eigen::Vector3f t_live = Tcw_live.translation();

    const Sophus::SE3f Twc_prior = poseToTwc(prior_img.pose);
    const Sophus::SE3f Tcw_prior = Twc_prior.inverse();
    const Eigen::Matrix3f R_prior = Tcw_prior.so3().matrix();
    const Eigen::Vector3f t_prior = Tcw_prior.translation();
    const float pfx = prior_img.intrinsics.fx;
    const float pfy = prior_img.intrinsics.fy;
    const float pcx = prior_img.intrinsics.cx;
    const float pcy = prior_img.intrinsics.cy;

    std::unordered_map<int, const prior_pack::TrackRecord*> track_by_prior_kpt;
    for (const auto& track : pack.tracks) {
        for (const auto& obs : track.observations) {
            if (obs.image_id == prior_image_id) {
                track_by_prior_kpt[obs.keypoint_index] = &track;
                break;
            }
        }
    }

    int lg_matches = 0;
    for (const int prior_kpt_idx : live_to_prior) {
        if (prior_kpt_idx >= 0)
            lg_matches++;
    }
    out.diag.lg_matches = lg_matches;

    float reproj_sum = 0.f;
    std::vector<float> err_live_samples;
    std::vector<float> err_prior_samples;
    float abs_z_sum = 0.f;
    int abs_z_count = 0;
    for (size_t live_idx = 0; live_idx < live_to_prior.size(); ++live_idx) {
        const int prior_kpt_idx = live_to_prior[live_idx];
        if (prior_kpt_idx < 0 || live_idx >= live_kpts.size())
            continue;
        if (prior_kpt_idx >= static_cast<int>(prior_img.features.keypoints.size()))
            continue;

        const auto track_it = track_by_prior_kpt.find(prior_kpt_idx);
        if (track_it == track_by_prior_kpt.end())
            continue;

        out.diag.with_track++;
        const prior_pack::TrackRecord& track = *track_it->second;
        const Eigen::Vector3f p_world(track.position.x, track.position.y, track.position.z);

        const Eigen::Vector3f p_cam_live = R_live * p_world + t_live;
        const Eigen::Vector3f p_cam_prior = R_prior * p_world + t_prior;
        if (!p_cam_live.allFinite() || !p_cam_prior.allFinite())
            continue;
        if (std::abs(p_cam_live.z()) <= 1e-6f && std::abs(p_cam_prior.z()) <= 1e-6f)
            continue;

        abs_z_sum += std::abs(p_cam_live.z());
        abs_z_count++;

        const cv::Point2f live_obs = live_kpts[live_idx].pt;
        float err_live = std::numeric_limits<float>::infinity();
        if (std::abs(p_cam_live.z()) > 1e-6f) {
            const float u = fx * p_cam_live.x() / p_cam_live.z() + cx;
            const float v = fy * p_cam_live.y() / p_cam_live.z() + cy;
            if (std::isfinite(u) && std::isfinite(v))
                err_live = std::hypot(u - live_obs.x, v - live_obs.y);
        }

        const auto& prior_kp = prior_img.features.keypoints[static_cast<size_t>(prior_kpt_idx)];
        const cv::Point2f prior_obs(prior_kp.first, prior_kp.second);
        float err_prior = std::numeric_limits<float>::infinity();
        if (std::abs(p_cam_prior.z()) > 1e-6f) {
            const float u = pfx * p_cam_prior.x() / p_cam_prior.z() + pcx;
            const float v = pfy * p_cam_prior.y() / p_cam_prior.z() + pcy;
            if (std::isfinite(u) && std::isfinite(v))
                err_prior = std::hypot(u - prior_obs.x, v - prior_obs.y);
        }

        if (std::isfinite(err_live))
            err_live_samples.push_back(err_live);
        if (std::isfinite(err_prior))
            err_prior_samples.push_back(err_prior);

        const bool pass_live = std::isfinite(err_live) && err_live <= max_reproj_px;
        const bool pass_prior = std::isfinite(err_prior) && err_prior <= max_reproj_px;
        if (pass_live)
            out.diag.pass_live++;
        if (pass_prior)
            out.diag.pass_prior++;
        if (pass_live && pass_prior)
            out.diag.pass_both++;

        if (!pass_prior || !pass_live)
            continue;

        out.inliers++;
        reproj_sum += err_live;
        overlay::PriorSoftObservation observation;
        observation.p_domain = p_world;
        observation.pixel = Eigen::Vector2f(live_obs.x, live_obs.y);
        out.observations.push_back(observation);
    }

    out.diag.err_live_p50 = percentileP50(std::move(err_live_samples));
    out.diag.err_prior_p50 = percentileP50(std::move(err_prior_samples));
    if (abs_z_count > 0)
        out.diag.mean_abs_z_live = abs_z_sum / static_cast<float>(abs_z_count);

    if (out.inliers > 0)
        out.mean_reproj_px = reproj_sum / static_cast<float>(out.inliers);
    return out;
}

float rotationAngleDeg(const Eigen::Matrix3f& Ra, const Eigen::Matrix3f& Rb) {
    const Eigen::Matrix3f R = Ra * Rb.transpose();
    const float trace = std::min(3.0f, std::max(-1.0f, R.trace()));
    return std::acos((trace - 1.0f) * 0.5f) * (180.0f / static_cast<float>(M_PI));
}

bool disagreesWithSlamPin(const ReprojGateResult& gate,
                          const Sophus::SE3f& Twc_domain_pinned,
                          const std::vector<float>& intrinsics,
                          float max_rot_deg,
                          float max_trans_m) {
    if (gate.observations.size() < 4 || intrinsics.size() < 4)
        return false;

    const float fx = intrinsics[0];
    const float fy = intrinsics[1];
    const float cx = intrinsics[2];
    const float cy = intrinsics[3];
    cv::Mat camera_matrix =
        (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    for (const auto& obs : gate.observations) {
        object_points.emplace_back(obs.p_domain.x(), obs.p_domain.y(), obs.p_domain.z());
        image_points.emplace_back(obs.pixel.x(), obs.pixel.y());
    }

    cv::Mat rvec;
    cv::Mat tvec;
    if (!cv::solvePnP(object_points,
                      image_points,
                      camera_matrix,
                      dist_coeffs,
                      rvec,
                      tvec,
                      false,
                      cv::SOLVEPNP_ITERATIVE)) {
        return false;
    }

    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);
    Eigen::Matrix3f R_pnp;
    Eigen::Vector3f t_pnp;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c)
            R_pnp(r, c) = static_cast<float>(R_cv.at<double>(r, c));
        t_pnp(r) = static_cast<float>(tvec.at<double>(r));
    }
    const Sophus::SE3f Tcw_pnp(R_pnp, t_pnp);
    const Sophus::SE3f Twc_pnp = Tcw_pnp.inverse();

    const float rot_deg =
        rotationAngleDeg(Twc_pnp.so3().matrix(), Twc_domain_pinned.so3().matrix());
    const float trans_m = (Twc_pnp.translation() - Twc_domain_pinned.translation()).norm();
    return rot_deg > max_rot_deg || trans_m > max_trans_m;
}

}  // namespace

int PriorSoftEdgeMatcher::nearestPriorImageId(const prior_pack::PriorPack& pack,
                                              const Sophus::SE3f& Twc_domain) {
    return findNearestPriorImageId(pack, Twc_domain);
}

struct PriorSoftEdgeMatcher::MatcherImpl {
#ifdef AUKI_LIGHTGLUE_AVAILABLE
    std::unique_ptr<LightGlueMrNerfMatcher> matcher;
#endif
};

PriorSoftEdgeMatcher::PriorSoftEdgeMatcher(const PriorMatchConfig& config) : config_(config) {
    impl_ = std::make_unique<MatcherImpl>();
#ifdef AUKI_LIGHTGLUE_AVAILABLE
    impl_->matcher = std::make_unique<LightGlueMrNerfMatcher>("cuda");
    impl_->matcher->SetMatchThresh(config_.lightglue_thresh);
#endif
}

PriorSoftEdgeMatcher::~PriorSoftEdgeMatcher() = default;

PriorMatchResult PriorSoftEdgeMatcher::tryMatch(const PriorMatchInput& input) {
    PriorMatchResult result;
    result.attempted = true;

    if (!input.pack || !input.pack->loaded) {
        result.skip_reason = "prior_pack_not_loaded";
        return result;
    }
    if (input.live_desc.empty() || input.live_kpts.empty()) {
        result.skip_reason = "no_live_features";
        return result;
    }
    if (input.live_desc.rows != static_cast<int>(input.live_kpts.size())) {
        result.skip_reason = "kpt_desc_mismatch";
        return result;
    }
    if (!input.has_domain_pose) {
        result.skip_reason = "no_domain_pose";
        return result;
    }
    if (input.image_width <= 0 || input.image_height <= 0) {
        result.skip_reason = "invalid_image_size";
        return result;
    }

    const std::vector<int> candidates = selectCandidateImages(input, config_);
    if (candidates.empty()) {
        result.skip_reason = "no_candidates";
        return result;
    }

    const int seed_id = input.seed_prior_image_id >= 0 ? input.seed_prior_image_id : candidates.front();
    const prior_pack::ImageRecord* seed_img = imageById(*input.pack, seed_id);
    if (seed_img && input.has_domain_pose) {
        result.debug_twc_seed_dist_m = poseCenterDistanceM(*seed_img, input.Twc_domain);
        result.debug_seed_prior_image_id = seed_id;
    }

    const std::vector<cv::Point2f> live_pts = keypointsToPoints(input.live_kpts);
    int best_image_id = -1;
    int best_inliers = 0;
    float best_reproj = std::numeric_limits<float>::infinity();
    overlay::PriorSoftEdgeFactor best_factor;
    ReprojGateResult best_gate;
    bool saw_slam_disagreement = false;

    for (const int prior_image_id : candidates) {
        const prior_pack::ImageRecord* prior_img = imageById(*input.pack, prior_image_id);
        if (!prior_img || prior_img->features.keypoints.empty())
            continue;

        const std::vector<cv::Point2f> prior_pts =
            keypointsToPoints(prior_img->features.keypoints);
        const cv::Mat prior_desc = descriptorsToMat(prior_img->features.descriptors);
        if (prior_desc.empty())
            continue;

        std::vector<int> matches12;
        bool matched = false;

#ifdef AUKI_LIGHTGLUE_AVAILABLE
        if (impl_->matcher) {
            matches12.resize(input.live_kpts.size(), -1);
            const int prior_w =
                std::max(1, static_cast<int>(std::round(2.f * prior_img->intrinsics.cx)));
            const int prior_h =
                std::max(1, static_cast<int>(std::round(2.f * prior_img->intrinsics.cy)));
            const int n =
                impl_->matcher->Match(
                    live_pts,
                    prior_pts,
                    input.live_desc,
                    prior_desc,
                    input.image_width,
                    input.image_height,
                    prior_w,
                    prior_h,
                    matches12);
            matched = n > 0;
        }
#endif

        if (!matched && config_.fixture_smoke) {
            matched = matchDescriptorsFixture(
                live_pts, input.live_desc, prior_pts, prior_desc, 0.05f, matches12);
        }

        if (!matched) {
            continue;
        }

        int lg_matches = 0;
        for (const int m : matches12) {
            if (m >= 0)
                lg_matches++;
        }
        result.debug_lg_matches = std::max(result.debug_lg_matches, lg_matches);

        const ReprojGateResult gate =
            gateWithTrackReprojection(
                *input.pack,
                prior_image_id,
                *prior_img,
                matches12,
                input.live_kpts,
                input.Twc_domain,
                input.intrinsics,
                config_.max_reproj_px);

        result.debug_best_inliers = std::max(result.debug_best_inliers, gate.inliers);
        if (gate.diag.with_track >= result.debug_with_track) {
            result.debug_with_track = gate.diag.with_track;
            result.debug_err_live_p50 = gate.diag.err_live_p50;
            result.debug_err_prior_p50 = gate.diag.err_prior_p50;
            result.debug_pass_live = gate.diag.pass_live;
            result.debug_pass_prior = gate.diag.pass_prior;
            result.debug_pass_both = gate.diag.pass_both;
            result.debug_mean_abs_z_live = gate.diag.mean_abs_z_live;
        }

        if (gate.inliers < config_.min_inliers)
            continue;

        if (disagreesWithSlamPin(
                gate,
                input.Twc_domain,
                input.intrinsics,
                config_.max_disagree_rot_deg,
                config_.max_disagree_trans_m)) {
            saw_slam_disagreement = true;
            continue;
        }

        if (gate.inliers > best_inliers ||
            (gate.inliers == best_inliers && gate.mean_reproj_px < best_reproj)) {
            best_inliers = gate.inliers;
            best_reproj = gate.mean_reproj_px;
            best_image_id = prior_image_id;
            best_gate = gate;
            best_factor.frame_id = input.frame_id;
            best_factor.timestamp = input.timestamp;
            best_factor.prior_image_id = prior_image_id;
            best_factor.inlier_count = gate.inliers;
            best_factor.mean_reproj_px = gate.mean_reproj_px;
            best_factor.fx = input.intrinsics[0];
            best_factor.fy = input.intrinsics[1];
            best_factor.cx = input.intrinsics[2];
            best_factor.cy = input.intrinsics[3];
            best_factor.observations = gate.observations;
            if (best_factor.observations.size() > kMaxPriorSoftObservationsPerFactor) {
                best_factor.observations.resize(kMaxPriorSoftObservationsPerFactor);
            }
            best_factor.weight = kPriorSoftBaseWeight * std::sqrt(static_cast<float>(gate.inliers));
        }
    }

    if (best_image_id < 0) {
        if (saw_slam_disagreement) {
            result.skip_reason = "slam_disagreement";
        } else {
#ifdef AUKI_LIGHTGLUE_AVAILABLE
            result.skip_reason = "no_gated_match";
#else
            result.skip_reason = "lightglue_unavailable";
#endif
        }
        return result;
    }

    if (best_gate.observations.empty()) {
        result.skip_reason = "no_gated_match";
        return result;
    }

    result.accepted = true;
    result.matched_prior_image_id = best_image_id;
    result.factor = best_factor;
    result.debug_best_inliers = best_gate.inliers;
    result.debug_err_live_p50 = best_gate.diag.err_live_p50;
    result.debug_err_prior_p50 = best_gate.diag.err_prior_p50;
    result.debug_pass_live = best_gate.diag.pass_live;
    result.debug_pass_prior = best_gate.diag.pass_prior;
    result.debug_pass_both = best_gate.diag.pass_both;
    return result;
}

}  // namespace prior_soft
