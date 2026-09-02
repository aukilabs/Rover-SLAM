#ifndef ALIKED_EXTRACTOR_H
#define ALIKED_EXTRACTOR_H

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

class AlikedExtractor {
public:
    AlikedExtractor(const std::string& model_name = "aliked-n16",
                    const std::string& device = "cuda",
                    int top_k = 1024,
                    float scores_th = 0.3f,
                    int n_limit = 20000,
                    int resize_max = 1024);

    ~AlikedExtractor();

    AlikedExtractor(const AlikedExtractor&) = delete;
    AlikedExtractor& operator=(const AlikedExtractor&) = delete;

    int Extract(const cv::Mat& image_gray,
                std::vector<cv::KeyPoint>& keypoints,
                cv::Mat& descriptors);

    float lastmatch = 0;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

#endif
