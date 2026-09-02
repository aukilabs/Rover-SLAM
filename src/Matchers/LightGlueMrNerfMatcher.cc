#include "Matchers/LightGlueMrNerfMatcher.h"

#include <cstring>
#include <stdexcept>

#ifdef AUKI_LIGHTGLUE_AVAILABLE
#include <torch/torch.h>
#include "matcher/lightglue/matcher.hpp"
#endif

namespace {

#ifdef AUKI_LIGHTGLUE_AVAILABLE

torch::Dict<std::string, torch::Tensor> BuildFeatureDict(const std::vector<cv::Point2f>& kpts,
                                                         const cv::Mat& desc,
                                                         int width,
                                                         int height,
                                                         const torch::Device& device) {
    const int n = static_cast<int>(kpts.size());
    if (n == 0 || desc.empty()) {
        throw std::invalid_argument("[LightGlueMrNerfMatcher] empty keypoints or descriptors");
    }
    if (desc.rows != n) {
        throw std::invalid_argument(
            "[LightGlueMrNerfMatcher] descriptor row count must match keypoint count");
    }
    if (desc.type() != CV_32F) {
        throw std::invalid_argument("[LightGlueMrNerfMatcher] descriptors must be CV_32F");
    }

    torch::Tensor kpts_tensor = torch::empty({n, 2}, torch::kFloat32);
    float* kptr = kpts_tensor.data_ptr<float>();
    for (int i = 0; i < n; ++i) {
        kptr[i * 2] = kpts[i].x;
        kptr[i * 2 + 1] = kpts[i].y;
    }

    const int d = desc.cols;
    torch::Tensor desc_tensor =
        torch::from_blob(const_cast<float*>(desc.ptr<float>()), {n, d}, torch::kFloat32).clone();

    torch::Dict<std::string, torch::Tensor> feats;
    feats.insert("keypoints", kpts_tensor.to(device));
    feats.insert("descriptors", desc_tensor.to(device));
    feats.insert("image_size",
                 torch::tensor({static_cast<float>(width), static_cast<float>(height)},
                               torch::TensorOptions().dtype(torch::kFloat32))
                     .unsqueeze(0)
                     .to(device));
    return feats;
}

std::vector<cv::Point2f> KeyPointsToPoint2f(const std::vector<cv::KeyPoint>& kpts) {
    std::vector<cv::Point2f> out;
    out.reserve(kpts.size());
    for (const cv::KeyPoint& kp : kpts) {
        out.emplace_back(kp.pt);
    }
    return out;
}

int FillMatchesFromOutput(const torch::Dict<std::string, torch::Tensor>& match_out,
                          float match_thresh,
                          std::vector<int>& vnMatches12) {
    auto matches0 = match_out.at("matches0").to(torch::kCPU).contiguous();
    auto scores0 = match_out.at("matching_scores0").to(torch::kCPU).contiguous();

    const int m = static_cast<int>(vnMatches12.size());
    int count = 0;

    auto matches_acc = matches0.accessor<int64_t, 2>();
    auto scores_acc = scores0.accessor<float, 2>();

    for (int i = 0; i < m; ++i) {
        const int64_t j = matches_acc[0][i];
        const float score = scores_acc[0][i];
        if (j >= 0 && score > match_thresh) {
            vnMatches12[i] = static_cast<int>(j);
            ++count;
        }
    }
    return count;
}

#endif

} // namespace

#ifdef AUKI_LIGHTGLUE_AVAILABLE

struct LightGlueMrNerfMatcher::Impl {
    float match_thresh_ = 0.0f;
    torch::Device device_;
    std::shared_ptr<matcher::LightGlue> matcher_;

    explicit Impl(const std::string& device_str)
        : device_(device_str),
          matcher_(std::make_shared<matcher::LightGlue>("aliked")) {
        matcher_->to(device_);
    }

    int MatchImpl(const std::vector<cv::Point2f>& kpts0,
                  const std::vector<cv::Point2f>& kpts1,
                  const cv::Mat& desc0,
                  const cv::Mat& desc1,
                  int image_width0,
                  int image_height0,
                  int image_width1,
                  int image_height1,
                  std::vector<int>& vnMatches12) {
        vnMatches12.assign(kpts0.size(), -1);
        if (kpts0.empty() || kpts1.empty()) {
            return 0;
        }

        auto feats0 =
            BuildFeatureDict(kpts0, desc0, image_width0, image_height0, device_);
        auto feats1 =
            BuildFeatureDict(kpts1, desc1, image_width1, image_height1, device_);

        torch::Dict<std::string, torch::Tensor> match_out;
        {
            torch::NoGradGuard no_grad;
            match_out = matcher_->forward(feats0, feats1);
        }

        return FillMatchesFromOutput(match_out, match_thresh_, vnMatches12);
    }
};

LightGlueMrNerfMatcher::LightGlueMrNerfMatcher(const std::string& device)
    : impl_(std::make_unique<Impl>(device)) {}

LightGlueMrNerfMatcher::~LightGlueMrNerfMatcher() = default;

void LightGlueMrNerfMatcher::SetMatchThresh(float thresh) {
    impl_->match_thresh_ = thresh;
}

float LightGlueMrNerfMatcher::GetMatchThresh() const {
    return impl_->match_thresh_;
}

int LightGlueMrNerfMatcher::Match(const std::vector<cv::Point2f>& kpts0,
                                  const std::vector<cv::Point2f>& kpts1,
                                  const cv::Mat& desc0,
                                  const cv::Mat& desc1,
                                  int image_width0,
                                  int image_height0,
                                  int image_width1,
                                  int image_height1,
                                  std::vector<int>& vnMatches12) {
    return impl_->MatchImpl(kpts0, kpts1, desc0, desc1, image_width0, image_height0,
                            image_width1, image_height1, vnMatches12);
}

int LightGlueMrNerfMatcher::Match(const std::vector<cv::KeyPoint>& kpts0,
                                  const std::vector<cv::KeyPoint>& kpts1,
                                  const cv::Mat& desc0,
                                  const cv::Mat& desc1,
                                  int image_width0,
                                  int image_height0,
                                  int image_width1,
                                  int image_height1,
                                  std::vector<int>& vnMatches12) {
    return Match(KeyPointsToPoint2f(kpts0), KeyPointsToPoint2f(kpts1), desc0, desc1,
                 image_width0, image_height0, image_width1, image_height1, vnMatches12);
}

int LightGlueMrNerfMatcher::Match(const std::vector<cv::Point2f>& kpts0,
                                  const std::vector<cv::Point2f>& kpts1,
                                  float* desc0,
                                  float* desc1,
                                  int desc_cols,
                                  int image_width0,
                                  int image_height0,
                                  int image_width1,
                                  int image_height1,
                                  std::vector<int>& vnMatches12) {
    const int rows0 = static_cast<int>(kpts0.size());
    const int rows1 = static_cast<int>(kpts1.size());
    cv::Mat mat0(rows0, desc_cols, CV_32F, desc0);
    cv::Mat mat1(rows1, desc_cols, CV_32F, desc1);
    return Match(kpts0, kpts1, mat0, mat1, image_width0, image_height0, image_width1,
                 image_height1, vnMatches12);
}

#else

struct LightGlueMrNerfMatcher::Impl {};

LightGlueMrNerfMatcher::LightGlueMrNerfMatcher(const std::string& /*device*/)
    : impl_(std::make_unique<Impl>()) {
    throw std::runtime_error(
        "LightGlueMrNerfMatcher requires AUKI_LIGHTGLUE_AVAILABLE (set LIBTORCH_ROOT)");
}

LightGlueMrNerfMatcher::~LightGlueMrNerfMatcher() = default;

void LightGlueMrNerfMatcher::SetMatchThresh(float /*thresh*/) {}

float LightGlueMrNerfMatcher::GetMatchThresh() const {
    return 0.0f;
}

int LightGlueMrNerfMatcher::Match(const std::vector<cv::Point2f>& /*kpts0*/,
                                  const std::vector<cv::Point2f>& /*kpts1*/,
                                  const cv::Mat& /*desc0*/,
                                  const cv::Mat& /*desc1*/,
                                  int /*image_width0*/,
                                  int /*image_height0*/,
                                  int /*image_width1*/,
                                  int /*image_height1*/,
                                  std::vector<int>& /*vnMatches12*/) {
    return 0;
}

int LightGlueMrNerfMatcher::Match(const std::vector<cv::KeyPoint>& /*kpts0*/,
                                  const std::vector<cv::KeyPoint>& /*kpts1*/,
                                  const cv::Mat& /*desc0*/,
                                  const cv::Mat& /*desc1*/,
                                  int /*image_width0*/,
                                  int /*image_height0*/,
                                  int /*image_width1*/,
                                  int /*image_height1*/,
                                  std::vector<int>& /*vnMatches12*/) {
    return 0;
}

int LightGlueMrNerfMatcher::Match(const std::vector<cv::Point2f>& /*kpts0*/,
                                  const std::vector<cv::Point2f>& /*kpts1*/,
                                  float* /*desc0*/,
                                  float* /*desc1*/,
                                  int /*desc_cols*/,
                                  int /*image_width0*/,
                                  int /*image_height0*/,
                                  int /*image_width1*/,
                                  int /*image_height1*/,
                                  std::vector<int>& /*vnMatches12*/) {
    return 0;
}

#endif
