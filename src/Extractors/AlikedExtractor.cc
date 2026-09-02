#include <cstring>
#include "Extractors/AlikedExtractor.h"

#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

#ifdef AUKI_LIGHTGLUE_AVAILABLE
#include <torch/torch.h>
#include "feature/ALIKED.hpp"
#endif

namespace {

cv::Mat GrayToRgb(const cv::Mat& image_gray) {
    if (image_gray.channels() == 3) {
        return image_gray.clone();
    }
    if (image_gray.channels() == 1) {
        cv::Mat rgb;
        cv::cvtColor(image_gray, rgb, cv::COLOR_GRAY2RGB);
        return rgb;
    }
    throw std::invalid_argument("[AlikedExtractor] expected grayscale or RGB image");
}

} // namespace

#ifdef AUKI_LIGHTGLUE_AVAILABLE

struct AlikedExtractor::Impl {
    int resize_max_;
    std::shared_ptr<ALIKED> extractor_;

    Impl(const std::string& model_name,
         const std::string& device,
         int top_k,
         float scores_th,
         int n_limit,
         int resize_max)
        : resize_max_(resize_max),
          extractor_(std::make_shared<ALIKED>(model_name, device, top_k, scores_th,
                                              n_limit)) {}
};

AlikedExtractor::AlikedExtractor(const std::string& model_name,
                                 const std::string& device,
                                 int top_k,
                                 float scores_th,
                                 int n_limit,
                                 int resize_max)
    : impl_(std::make_unique<Impl>(model_name, device, top_k, scores_th, n_limit,
                                   resize_max)) {}

AlikedExtractor::~AlikedExtractor() = default;

int AlikedExtractor::Extract(const cv::Mat& image_gray,
                             std::vector<cv::KeyPoint>& keypoints,
                             cv::Mat& descriptors) {
    keypoints.clear();

    cv::Mat rgb = GrayToRgb(image_gray);
    const int orig_w = rgb.cols;
    const int orig_h = rgb.rows;

    cv::Mat process_img = rgb;
    float scale_x = 1.0f;
    float scale_y = 1.0f;

    if (impl_->resize_max_ > 0) {
        const int max_side = std::max(orig_w, orig_h);
        if (max_side > impl_->resize_max_) {
            const float scale =
                static_cast<float>(impl_->resize_max_) / static_cast<float>(max_side);
            const int new_w =
                static_cast<int>(std::lround(static_cast<float>(orig_w) * scale));
            const int new_h =
                static_cast<int>(std::lround(static_cast<float>(orig_h) * scale));
            cv::resize(rgb, process_img, cv::Size(new_w, new_h), 0, 0, cv::INTER_AREA);
            scale_x = static_cast<float>(orig_w) / static_cast<float>(new_w);
            scale_y = static_cast<float>(orig_h) / static_cast<float>(new_h);
        }
    }

    auto feats = impl_->extractor_->run(process_img);

    if (scale_x != 1.0f || scale_y != 1.0f) {
        auto kpts = feats.at("keypoints");
        const auto scales =
            torch::tensor({scale_x, scale_y},
                          torch::TensorOptions().dtype(kpts.dtype()).device(kpts.device()));
        kpts = (kpts + 0.5) * scales - 0.5;
        TORCH_CHECK(feats.erase("keypoints"),
                    "Failed to remove 'keypoints' from output dict");
        feats.insert("keypoints", kpts);
    }

    auto kpts_cpu = feats.at("keypoints").to(torch::kCPU).contiguous();
    auto desc_cpu = feats.at("descriptors").to(torch::kCPU).contiguous();
    auto scores_cpu = feats.at("scores").to(torch::kCPU).contiguous();

    const int n = static_cast<int>(kpts_cpu.size(0));
    if (n == 0) {
        descriptors.release();
        return 0;
    }

    const int desc_dim = static_cast<int>(desc_cpu.size(1));
    descriptors.create(n, desc_dim, CV_32F);

    const float* kpts_ptr = kpts_cpu.data_ptr<float>();
    const float* desc_ptr = desc_cpu.data_ptr<float>();
    const float* scores_ptr = scores_cpu.data_ptr<float>();

    keypoints.reserve(n);
    for (int i = 0; i < n; ++i) {
        cv::KeyPoint kp;
        kp.pt.x = kpts_ptr[i * 2];
        kp.pt.y = kpts_ptr[i * 2 + 1];
        kp.response = scores_ptr[i];
        kp.size = 10.0f;
        kp.octave = 0;
        keypoints.emplace_back(kp);
    }

    std::memcpy(descriptors.data, desc_ptr,
                static_cast<size_t>(n) * static_cast<size_t>(desc_dim) * sizeof(float));

    return n;
}

#else

struct AlikedExtractor::Impl {};

AlikedExtractor::AlikedExtractor(const std::string& /*model_name*/,
                                 const std::string& /*device*/,
                                 int /*top_k*/,
                                 float /*scores_th*/,
                                 int /*n_limit*/,
                                 int /*resize_max*/)
    : impl_(std::make_unique<Impl>()) {
    throw std::runtime_error(
        "AlikedExtractor requires AUKI_LIGHTGLUE_AVAILABLE (set LIBTORCH_ROOT)");
}

AlikedExtractor::~AlikedExtractor() = default;

int AlikedExtractor::Extract(const cv::Mat& /*image_gray*/,
                             std::vector<cv::KeyPoint>& /*keypoints*/,
                             cv::Mat& /*descriptors*/) {
    return 0;
}

#endif
