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
// Drain LibTorch CUDA work before host-side g2o graph alloc/free (init Global BA).
void MrNerfCudaDrainForHostAlloc();
#endif

#endif
