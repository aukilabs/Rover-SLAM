/**
 * Smoke test for ALIKED extraction + LightGlue matching (MrNeRF / LibTorch stack).
 * Built only when CMAKE finds LibTorch (AUKI_LIGHTGLUE_AVAILABLE).
 */

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#ifdef AUKI_LIGHTGLUE_AVAILABLE
#include "Extractors/AlikedExtractor.h"
#include "Matchers/LightGlueMrNerfMatcher.h"
#endif

namespace {

int Fail(const std::string& msg) {
    std::cerr << "smoke_aliked_lightglue: " << msg << std::endl;
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
#ifndef AUKI_LIGHTGLUE_AVAILABLE
    std::cerr
        << "smoke_aliked_lightglue: not built with LibTorch (AUKI_LIGHTGLUE_AVAILABLE).\n"
        << "Set LIBTORCH_ROOT and reconfigure CMake, then rebuild.\n";
    return 1;
#else
    const std::string img0_path =
        (argc > 1) ? argv[1] : "../../frame_100_original.png";
    const std::string img1_path =
        (argc > 2) ? argv[2] : "../../frame_101_original.png";
    const std::string device = (argc > 3) ? argv[3] : "cuda";

    cv::Mat img0 = cv::imread(img0_path, cv::IMREAD_GRAYSCALE);
    if (img0.empty()) {
        return Fail("failed to load image 0: " + img0_path);
    }
    cv::Mat img1 = cv::imread(img1_path, cv::IMREAD_GRAYSCALE);
    if (img1.empty()) {
        return Fail("failed to load image 1: " + img1_path);
    }

    std::cout << "smoke_aliked_lightglue\n"
              << "  image0: " << img0_path << " (" << img0.cols << "x" << img0.rows << ")\n"
              << "  image1: " << img1_path << " (" << img1.cols << "x" << img1.rows << ")\n"
              << "  device: " << device << std::endl;

    AlikedExtractor extractor("aliked-n16", device, 1024);
    LightGlueMrNerfMatcher matcher(device);

    std::vector<cv::KeyPoint> kpts0, kpts1;
    cv::Mat desc0, desc1;

    const int n0 = extractor.Extract(img0, kpts0, desc0);
    const int n1 = extractor.Extract(img1, kpts1, desc1);

    std::cout << "  keypoints0: " << n0 << " (desc " << desc0.cols << "-d)\n"
              << "  keypoints1: " << n1 << " (desc " << desc1.cols << "-d)" << std::endl;

    if (n0 == 0 || n1 == 0) {
        return Fail("extract returned zero keypoints");
    }

    std::vector<int> vnMatches12;
    const int match_count = matcher.Match(kpts0, kpts1, desc0, desc1, img0.cols, img0.rows,
                                          img1.cols, img1.rows, vnMatches12);

    std::cout << "  matches: " << match_count << std::endl;

    if (match_count == 0) {
        return Fail("matcher returned zero matches");
    }

    std::cout << "smoke_aliked_lightglue: OK" << std::endl;
    return 0;
#endif
}
