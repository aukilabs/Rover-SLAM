/**
 * Microbench: ALIKED extract time at top_k=1024 vs 512, and resize_max sweep
 * at fixed top_k=1024 (timing + keypoints).
 * Usage: bench_aliked_topk [image_or_frames_dir] [warmup] [iters] [device]
 *   If arg1 is a directory, samples several frames for keypoint stats.
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#ifdef AUKI_LIGHTGLUE_AVAILABLE
#include "Extractors/AlikedExtractor.h"
#endif

namespace fs = std::filesystem;

namespace {

double percentile(std::vector<double> v, double p) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    const double idx = p * static_cast<double>(v.size() - 1);
    const size_t lo = static_cast<size_t>(idx);
    const size_t hi = std::min(lo + 1, v.size() - 1);
    const double frac = idx - static_cast<double>(lo);
    return v[lo] * (1.0 - frac) + v[hi] * frac;
}

void print_timing_line(const std::string& label,
                       int keypoints,
                       int desc_dim,
                       int process_w,
                       int process_h,
                       const std::vector<double>& ms) {
    const double mean = std::accumulate(ms.begin(), ms.end(), 0.0) / ms.size();
    std::cout << label
              << " process=" << process_w << "x" << process_h
              << " keypoints=" << keypoints
              << " desc_dim=" << desc_dim
              << " n=" << ms.size()
              << " mean_ms=" << mean
              << " p50_ms=" << percentile(ms, 0.50)
              << " p95_ms=" << percentile(ms, 0.95)
              << " min_ms=" << *std::min_element(ms.begin(), ms.end())
              << " max_ms=" << *std::max_element(ms.begin(), ms.end())
              << std::endl;
}

cv::Size process_size_for(const cv::Mat& img, int resize_max) {
    if (resize_max <= 0) return cv::Size(img.cols, img.rows);
    const int max_side = std::max(img.cols, img.rows);
    if (max_side <= resize_max) return cv::Size(img.cols, img.rows);
    const float scale =
        static_cast<float>(resize_max) / static_cast<float>(max_side);
    return cv::Size(
        static_cast<int>(std::lround(static_cast<float>(img.cols) * scale)),
        static_cast<int>(std::lround(static_cast<float>(img.rows) * scale)));
}

void bench_config(const cv::Mat& img,
                  int top_k,
                  int resize_max,
                  int warmup,
                  int iters,
                  const std::string& device) {
    AlikedExtractor extractor("aliked-n16", device, top_k, 0.3f, 20000, resize_max);
    std::vector<cv::KeyPoint> kpts;
    cv::Mat desc;

    for (int i = 0; i < warmup; ++i) {
        extractor.Extract(img, kpts, desc);
    }

    std::vector<double> ms;
    ms.reserve(static_cast<size_t>(iters));
    int last_n = 0;
    for (int i = 0; i < iters; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        last_n = extractor.Extract(img, kpts, desc);
        const auto t1 = std::chrono::steady_clock::now();
        ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }

    const cv::Size psz = process_size_for(img, resize_max);
    const std::string label =
        "top_k=" + std::to_string(top_k) +
        " resize_max=" + (resize_max > 0 ? std::to_string(resize_max) : std::string("none"));
    print_timing_line(label, last_n, desc.cols, psz.width, psz.height, ms);
}

std::vector<fs::path> list_jpgs(const fs::path& dir) {
    std::vector<fs::path> out;
    for (const auto& e : fs::directory_iterator(dir)) {
        if (!e.is_regular_file()) continue;
        const auto ext = e.path().extension().string();
        if (ext == ".jpg" || ext == ".JPG" || ext == ".jpeg" || ext == ".JPEG" ||
            ext == ".png" || ext == ".PNG") {
            out.push_back(e.path());
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

void sample_keypoint_counts(const std::vector<cv::Mat>& frames,
                            int top_k,
                            int resize_max,
                            const std::string& device) {
    AlikedExtractor extractor("aliked-n16", device, top_k, 0.3f, 20000, resize_max);
    std::vector<cv::KeyPoint> kpts;
    cv::Mat desc;
    std::vector<int> counts;
    counts.reserve(frames.size());

    // one warmup on first frame
    if (!frames.empty()) {
        extractor.Extract(frames.front(), kpts, desc);
    }

    for (const auto& img : frames) {
        counts.push_back(extractor.Extract(img, kpts, desc));
    }

    const double mean =
        std::accumulate(counts.begin(), counts.end(), 0.0) / counts.size();
    const auto [mn_it, mx_it] = std::minmax_element(counts.begin(), counts.end());
    const cv::Size psz = process_size_for(frames.front(), resize_max);
    std::cout << "frames_kpts top_k=" << top_k
              << " resize_max=" << (resize_max > 0 ? std::to_string(resize_max) : "none")
              << " process=" << psz.width << "x" << psz.height
              << " n_frames=" << counts.size()
              << " mean_kpts=" << mean
              << " min_kpts=" << *mn_it
              << " max_kpts=" << *mx_it
              << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
#ifndef AUKI_LIGHTGLUE_AVAILABLE
    std::cerr << "bench_aliked_topk: not built with AUKI_LIGHTGLUE_AVAILABLE\n";
    return 1;
#else
    const std::string path_arg =
        (argc > 1) ? argv[1]
                   : "/mnt/d/agent-workspaces/reconstruction-server/_local/job-artifacts/"
                     "20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete/"
                     "datasets/2026-04-02_13-42-53/Frames/"
                     "Frame_2026-04-02_13-42-53_0000.jpg";
    const int warmup = (argc > 2) ? std::stoi(argv[2]) : 5;
    const int iters = (argc > 3) ? std::stoi(argv[3]) : 50;
    const std::string device = (argc > 4) ? argv[4] : "cuda";

    std::string img_path = path_arg;
    std::vector<cv::Mat> sample_frames;
    if (fs::is_directory(path_arg)) {
        auto jpgs = list_jpgs(path_arg);
        if (jpgs.empty()) {
            std::cerr << "no images in dir: " << path_arg << std::endl;
            return 1;
        }
        // ~20 evenly spaced frames for keypoint stats
        const size_t want = std::min<size_t>(20, jpgs.size());
        for (size_t i = 0; i < want; ++i) {
            const size_t idx = (want == 1) ? 0 : i * (jpgs.size() - 1) / (want - 1);
            cv::Mat im = cv::imread(jpgs[idx].string(), cv::IMREAD_GRAYSCALE);
            if (!im.empty()) sample_frames.push_back(im);
        }
        img_path = jpgs[std::min<size_t>(99, jpgs.size() - 1)].string();
    }

    cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "failed to load: " << img_path << std::endl;
        return 1;
    }
    if (sample_frames.empty()) {
        sample_frames.push_back(img);
    }

    std::cout << "bench_aliked_topk\n"
              << "  image: " << img_path << " (" << img.cols << "x" << img.rows << ")\n"
              << "  sample_frames: " << sample_frames.size() << "\n"
              << "  device: " << device
              << " warmup=" << warmup << " iters=" << iters << std::endl;

    std::cout << "\n=== top_k timing (resize_max=1024) ===\n";
    bench_config(img, 1024, 1024, warmup, iters, device);
    bench_config(img, 512, 1024, warmup, iters, device);

    // Sweep long-side cap; 0 = no resize (native 1920x1440 here).
    const std::vector<int> resize_maxes = {512, 640, 768, 1024, 1280, 0};
    std::cout << "\n=== resize_max timing (top_k=1024) ===\n";
    for (int rm : resize_maxes) {
        bench_config(img, 1024, rm, warmup, iters, device);
    }

    std::cout << "\n=== resize_max keypoint counts across frames (top_k=1024) ===\n";
    for (int rm : resize_maxes) {
        sample_keypoint_counts(sample_frames, 1024, rm, device);
    }
    return 0;
#endif
}
