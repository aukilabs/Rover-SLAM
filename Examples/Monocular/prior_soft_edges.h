/**
 * Prior image matching + track reprojection gate for overlay soft edges (S5).
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "overlay_pose_graph.h"
#include "prior_pack_loader.h"

namespace prior_soft {

struct PriorMatchConfig {
    size_t match_every_nth_frame = 5;
    size_t max_candidates = 8;
    float pose_proximity_m = 3.0f;
    float max_reproj_px = 12.0f;
    int min_inliers = 4;
    float lightglue_thresh = 0.2f;
    /** PnP-vs-pin rotation (deg) or translation (m) above this → slam_disagreement reject. */
    float max_disagree_rot_deg = 18.0f;
    float max_disagree_trans_m = 0.75f;
    bool fixture_smoke = false;
};

struct PriorMatchInput {
    const prior_pack::PriorPack* pack = nullptr;
    std::vector<cv::KeyPoint> live_kpts;
    cv::Mat live_desc;
    Sophus::SE3f Twc_slam;
    bool has_domain_pose = false;
    Sophus::SE3f Twc_domain;
    std::vector<float> intrinsics;
    int image_width = 0;
    int image_height = 0;
    uint64_t frame_id = 0;
    double timestamp = 0.0;
    int seed_prior_image_id = -1;
};

struct PriorMatchResult {
    bool attempted = false;
    bool accepted = false;
    std::string skip_reason;
    overlay::PriorSoftEdgeFactor factor;
    int matched_prior_image_id = -1;
    /** Diagnostics when skip_reason is no_gated_match (not a gate pass). */
    int debug_lg_matches = 0;
    int debug_best_inliers = 0;
    int debug_with_track = 0;
    float debug_err_live_p50 = -1.f;
    float debug_err_prior_p50 = -1.f;
    int debug_pass_live = 0;
    int debug_pass_prior = 0;
    int debug_pass_both = 0;
    float debug_twc_seed_dist_m = -1.f;
    float debug_mean_abs_z_live = -1.f;
    int debug_seed_prior_image_id = -1;
};

class PriorSoftEdgeMatcher {
public:
    explicit PriorSoftEdgeMatcher(const PriorMatchConfig& config);
    ~PriorSoftEdgeMatcher();

    PriorMatchResult tryMatch(const PriorMatchInput& input);

    /** Nearest prior image camera center to domain pose (for covis seed after portal PnP). */
    static int nearestPriorImageId(const prior_pack::PriorPack& pack, const Sophus::SE3f& Twc_domain);

private:
    PriorMatchConfig config_;
    struct MatcherImpl;
    std::unique_ptr<MatcherImpl> impl_;
};

}  // namespace prior_soft
