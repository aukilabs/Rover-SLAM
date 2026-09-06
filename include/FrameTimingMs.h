#ifndef FRAMETIMINGMS_H
#define FRAMETIMINGMS_H

namespace ORB_SLAM3
{

/** Always-on VO stage timings for the last processed frame (milliseconds). */
struct FrameTimingMs {
    double extract_ms = 0;
    double pose_pred_ms = 0;
    /** Accumulated time inside pose_pred sub-paths (may sum > one path if fallbacks run). */
    double pose_mm_ms = 0;
    double pose_refkf_ms = 0;
    double pose_reloc_ms = 0;
    /** Inside TrackWithMotionModel: SearchBySP (LightGlue) wall time, incl. retry. */
    double pose_mm_match_ms = 0;
    /** Inside TrackWithMotionModel: PoseOptimization wall time. */
    double pose_mm_opt_ms = 0;
    /** Accumulated LightGlueMrNerfMatcher stages (all Match calls this frame). */
    double lg_prep_ms = 0;
    double lg_fwd_ms = 0;
    double lg_drain_ms = 0;
    double lg_post_ms = 0;
    int lg_calls = 0;
    int lg_kpts0 = 0;
    int lg_kpts1 = 0;
    double local_map_ms = 0;
    double new_kf_ms = 0;
    double track_total_ms = 0;
    int local_map_points = 0;
    int matches_inliers = 0;
    int state = 0;
};

} // namespace ORB_SLAM3

#endif // FRAMETIMINGMS_H
