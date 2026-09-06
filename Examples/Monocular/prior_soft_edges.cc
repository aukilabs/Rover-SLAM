#include "prior_soft_edges.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#ifdef AUKI_LIGHTGLUE_AVAILABLE
#include "Matchers/LightGlueMrNerfMatcher.h"
#endif

namespace prior_soft {

namespace {

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

int nearestPriorImageId(const prior_pack::PriorPack& pack, const Sophus::SE3f& Twc_domain) {
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
            seed = nearestPriorImageId(pack, input.Twc_domain);
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

struct ReprojGateResult {
    int inliers = 0;
    float mean_reproj_px = 0.f;
};

ReprojGateResult gateWithTrackReprojection(
    const prior_pack::PriorPack& pack,
    int prior_image_id,
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
    const Sophus::SE3f Tcw = Twc_domain.inverse();

    float reproj_sum = 0.f;
    for (const auto& track : pack.tracks) {
        int prior_kpt_idx = -1;
        for (const auto& obs : track.observations) {
            if (obs.image_id == prior_image_id) {
                prior_kpt_idx = obs.keypoint_index;
                break;
            }
        }
        if (prior_kpt_idx < 0)
            continue;

        int live_idx = -1;
        for (size_t i = 0; i < live_to_prior.size(); ++i) {
            if (live_to_prior[i] == prior_kpt_idx) {
                live_idx = static_cast<int>(i);
                break;
            }
        }
        if (live_idx < 0 || live_idx >= static_cast<int>(live_kpts.size()))
            continue;

        const Eigen::Vector3f p_world(track.position.x, track.position.y, track.position.z);
        const Eigen::Vector3f p_cam = Tcw * p_world;
        if (p_cam.z() <= 1e-4f)
            continue;

        const float u = fx * p_cam.x() / p_cam.z() + cx;
        const float v = fy * p_cam.y() / p_cam.z() + cy;
        const cv::Point2f proj(u, v);
        const cv::Point2f obs = live_kpts[live_idx].pt;
        const float err = std::hypot(proj.x - obs.x, proj.y - obs.y);
        if (err <= max_reproj_px) {
            out.inliers++;
            reproj_sum += err;
        }
    }

    if (out.inliers > 0)
        out.mean_reproj_px = reproj_sum / static_cast<float>(out.inliers);
    return out;
}

}  // namespace

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

    const std::vector<cv::Point2f> live_pts = keypointsToPoints(input.live_kpts);
    int best_image_id = -1;
    int best_inliers = 0;
    float best_reproj = std::numeric_limits<float>::infinity();
    overlay::PriorSoftEdgeFactor best_factor;

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
            const int n =
                impl_->matcher->Match(
                    live_pts,
                    prior_pts,
                    input.live_desc,
                    prior_desc,
                    input.image_width,
                    input.image_height,
                    input.image_width,
                    input.image_height,
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

        const ReprojGateResult gate =
            gateWithTrackReprojection(
                *input.pack,
                prior_image_id,
                matches12,
                input.live_kpts,
                input.Twc_domain,
                input.intrinsics,
                config_.max_reproj_px);

        if (gate.inliers < config_.min_inliers)
            continue;

        if (gate.inliers > best_inliers ||
            (gate.inliers == best_inliers && gate.mean_reproj_px < best_reproj)) {
            best_inliers = gate.inliers;
            best_reproj = gate.mean_reproj_px;
            best_image_id = prior_image_id;
            best_factor.frame_id = input.frame_id;
            best_factor.timestamp = input.timestamp;
            best_factor.prior_image_id = prior_image_id;
            best_factor.inlier_count = gate.inliers;
            best_factor.mean_reproj_px = gate.mean_reproj_px;
            best_factor.weight = static_cast<float>(gate.inliers);
        }
    }

    if (best_image_id < 0) {
#ifdef AUKI_LIGHTGLUE_AVAILABLE
        result.skip_reason = "no_gated_match";
#else
        result.skip_reason = "lightglue_unavailable";
#endif
        return result;
    }

    result.accepted = true;
    result.matched_prior_image_id = best_image_id;
    result.factor = best_factor;
    return result;
}

}  // namespace prior_soft
