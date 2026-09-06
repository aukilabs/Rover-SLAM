#ifndef LIGHTGLUE_MRNERF_MATCHER_H
#define LIGHTGLUE_MRNERF_MATCHER_H

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

class LightGlueMrNerfMatcher {
public:
    explicit LightGlueMrNerfMatcher(const std::string& device = "cuda");
    ~LightGlueMrNerfMatcher();

    LightGlueMrNerfMatcher(const LightGlueMrNerfMatcher&) = delete;
    LightGlueMrNerfMatcher& operator=(const LightGlueMrNerfMatcher&) = delete;

    void SetMatchThresh(float thresh);
    float GetMatchThresh() const;

    int Match(const std::vector<cv::Point2f>& kpts0,
              const std::vector<cv::Point2f>& kpts1,
              const cv::Mat& desc0,
              const cv::Mat& desc1,
              int image_width0,
              int image_height0,
              int image_width1,
              int image_height1,
              std::vector<int>& vnMatches12);

    int Match(const std::vector<cv::KeyPoint>& kpts0,
              const std::vector<cv::KeyPoint>& kpts1,
              const cv::Mat& desc0,
              const cv::Mat& desc1,
              int image_width0,
              int image_height0,
              int image_width1,
              int image_height1,
              std::vector<int>& vnMatches12);

    int Match(const std::vector<cv::Point2f>& kpts0,
              const std::vector<cv::Point2f>& kpts1,
              float* desc0,
              float* desc1,
              int desc_cols,
              int image_width0,
              int image_height0,
              int image_width1,
              int image_height1,
              std::vector<int>& vnMatches12);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

#ifdef AUKI_LIGHTGLUE_AVAILABLE
// After extract/match: sync only by default; emptyCache if ROVER_LG_EMPTY_CACHE=1.
void MrNerfCudaSyncAfterInference();
// Sync + emptyCache before host-side g2o graph alloc/free (init Global BA).
void MrNerfCudaDrainForHostAlloc();
#endif

/** Thread-local LightGlue Match stage accumulators (instrument-only). */
struct LightGlueMatchTimingMs {
    double prep_ms = 0;
    double fwd_ms = 0;
    double drain_ms = 0;
    double post_ms = 0;
    int calls = 0;
    int last_kpts0 = 0;
    int last_kpts1 = 0;
};

void LightGlueMatchTimingReset();
LightGlueMatchTimingMs LightGlueMatchTimingSnapshot();

#endif
