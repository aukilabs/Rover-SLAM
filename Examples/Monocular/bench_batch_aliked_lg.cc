/**
 * Batch-style throughput: ALIKED extract + LightGlue match on a frame sequence.
 * Compares emptyCache-per-call (ROVER_LG_EMPTY_CACHE=1) vs default sync-only,
 * and tries stacking multiple images into one ALIKED forward (true GPU batch).
 *
 * Usage: bench_batch_aliked_lg [frames_dir] [n_frames] [device]
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#ifdef AUKI_LIGHTGLUE_AVAILABLE
#include <torch/torch.h>
#include "Extractors/AlikedExtractor.h"
#include "Matchers/LightGlueMrNerfMatcher.h"
#include "feature/ALIKED.hpp"
#endif

namespace fs = std::filesystem;

namespace {

double mean(const std::vector<double>& v) {
    if (v.empty()) return 0;
    return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

double percentile(std::vector<double> v, double p) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    const double idx = p * static_cast<double>(v.size() - 1);
    const size_t lo = static_cast<size_t>(idx);
    const size_t hi = std::min(lo + 1, v.size() - 1);
    const double frac = idx - static_cast<double>(lo);
    return v[lo] * (1.0 - frac) + v[hi] * frac;
}

std::vector<fs::path> list_images(const fs::path& dir) {
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

cv::Mat ToRgbResized(const cv::Mat& gray, int resize_max) {
    cv::Mat rgb;
    if (gray.channels() == 1) {
        cv::cvtColor(gray, rgb, cv::COLOR_GRAY2RGB);
    } else {
        rgb = gray.clone();
    }
    if (resize_max > 0) {
        const int max_side = std::max(rgb.cols, rgb.rows);
        if (max_side > resize_max) {
            const float scale =
                static_cast<float>(resize_max) / static_cast<float>(max_side);
            cv::resize(rgb, rgb,
                       cv::Size(static_cast<int>(std::lround(rgb.cols * scale)),
                                static_cast<int>(std::lround(rgb.rows * scale))),
                       0, 0, cv::INTER_AREA);
        }
    }
    return rgb;
}

}  // namespace

int main(int argc, char** argv) {
#ifndef AUKI_LIGHTGLUE_AVAILABLE
    std::cerr << "bench_batch_aliked_lg: need AUKI_LIGHTGLUE_AVAILABLE\n";
    return 1;
#else
    const std::string frames_dir =
        (argc > 1)
            ? argv[1]
            : "/mnt/d/agent-workspaces/reconstruction-server/_local/job-artifacts/"
              "20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete/"
              "datasets/2026-04-02_13-42-53/Frames";
    const int n_want = (argc > 2) ? std::stoi(argv[2]) : 60;
    const std::string device = (argc > 3) ? argv[3] : "cuda";
    constexpr int kResizeMax = 1024;
    constexpr int kTopK = 1024;

    auto paths = list_images(frames_dir);
    if (paths.empty()) {
        std::cerr << "no images in " << frames_dir << "\n";
        return 1;
    }
    const int n = std::min(n_want, static_cast<int>(paths.size()));
    // Evenly sample n frames across the sequence.
    std::vector<cv::Mat> frames;
    frames.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        const size_t idx =
            (n == 1) ? 0 : static_cast<size_t>(i) * (paths.size() - 1) / static_cast<size_t>(n - 1);
        cv::Mat im = cv::imread(paths[idx].string(), cv::IMREAD_GRAYSCALE);
        if (im.empty()) {
            std::cerr << "failed to load " << paths[idx] << "\n";
            return 1;
        }
        frames.push_back(im);
    }

    std::cout << "bench_batch_aliked_lg\n"
              << "  frames_dir=" << frames_dir << "\n"
              << "  n=" << n << " device=" << device
              << " top_k=" << kTopK << " resize_max=" << kResizeMax << "\n"
              << "  first=" << paths.front() << "\n";

    AlikedExtractor extractor("aliked-n16", device, kTopK, 0.3f, 20000, kResizeMax);
    LightGlueMrNerfMatcher matcher(device);

    auto run_pass = [&](const char* label, bool empty_cache) {
        if (empty_cache) {
            setenv("ROVER_LG_EMPTY_CACHE", "1", 1);
        } else {
            unsetenv("ROVER_LG_EMPTY_CACHE");
        }

        // Warmup
        {
            std::vector<cv::KeyPoint> k0, k1;
            cv::Mat d0, d1;
            extractor.Extract(frames[0], k0, d0);
            extractor.Extract(frames[1], k1, d1);
            std::vector<int> m;
            matcher.Match(k0, k1, d0, d1, frames[0].cols, frames[0].rows,
                          frames[1].cols, frames[1].rows, m);
        }

        std::vector<std::vector<cv::KeyPoint>> all_kpts(static_cast<size_t>(n));
        std::vector<cv::Mat> all_desc(static_cast<size_t>(n));
        std::vector<double> extract_ms;
        extract_ms.reserve(static_cast<size_t>(n));

        const auto t_all0 = std::chrono::steady_clock::now();
        for (int i = 0; i < n; ++i) {
            const auto t0 = std::chrono::steady_clock::now();
            extractor.Extract(frames[static_cast<size_t>(i)], all_kpts[static_cast<size_t>(i)],
                              all_desc[static_cast<size_t>(i)]);
            if (torch::cuda::is_available()) torch::cuda::synchronize();
            const auto t1 = std::chrono::steady_clock::now();
            extract_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        }
        const auto t_ext1 = std::chrono::steady_clock::now();

        std::vector<double> match_ms;
        match_ms.reserve(static_cast<size_t>(n - 1));
        int match_sum = 0;
        for (int i = 0; i + 1 < n; ++i) {
            LightGlueMatchTimingReset();
            std::vector<int> vn;
            const auto t0 = std::chrono::steady_clock::now();
            match_sum += matcher.Match(
                all_kpts[static_cast<size_t>(i)], all_kpts[static_cast<size_t>(i + 1)],
                all_desc[static_cast<size_t>(i)], all_desc[static_cast<size_t>(i + 1)],
                frames[static_cast<size_t>(i)].cols, frames[static_cast<size_t>(i)].rows,
                frames[static_cast<size_t>(i + 1)].cols,
                frames[static_cast<size_t>(i + 1)].rows, vn);
            const auto t1 = std::chrono::steady_clock::now();
            const auto lg = LightGlueMatchTimingSnapshot();
            match_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
            if (i == 0) {
                std::cout << "  [" << label << "] first_pair lg_prep=" << lg.prep_ms
                          << " lg_fwd=" << lg.fwd_ms << " lg_drain=" << lg.drain_ms
                          << " lg_post=" << lg.post_ms << " kpts=" << lg.last_kpts0 << "/"
                          << lg.last_kpts1 << "\n";
            }
        }
        const auto t_all1 = std::chrono::steady_clock::now();

        const double wall_ms =
            std::chrono::duration<double, std::milli>(t_all1 - t_all0).count();
        const double extract_wall =
            std::chrono::duration<double, std::milli>(t_ext1 - t_all0).count();
        const double match_wall = wall_ms - extract_wall;

        std::cout << "=== " << label << " ===\n"
                  << "  extract: mean=" << mean(extract_ms) << " p50=" << percentile(extract_ms, 0.5)
                  << " p95=" << percentile(extract_ms, 0.95)
                  << " wall_ms=" << extract_wall
                  << " per_image=" << (extract_wall / n) << "\n"
                  << "  match:   mean=" << mean(match_ms) << " p50=" << percentile(match_ms, 0.5)
                  << " p95=" << percentile(match_ms, 0.95)
                  << " wall_ms=" << match_wall
                  << " per_pair=" << (match_wall / (n - 1))
                  << " avg_matches=" << (static_cast<double>(match_sum) / (n - 1)) << "\n"
                  << "  TOTAL wall_ms=" << wall_ms
                  << " per_frame_equiv=" << (wall_ms / n)
                  << " (extract_all then match consecutive)\n";
    };

    std::cout << "\n--- pass A: emptyCache per call (ROVER_LG_EMPTY_CACHE=1) ---\n";
    run_pass("emptyCache", true);

    std::cout << "\n--- pass B: default sync-only (no emptyCache) ---\n";
    run_pass("sync_only", false);

    // True ALIKED image batch: stack B RGB tensors, one forward.
    std::cout << "\n--- pass C: ALIKED stacked image batch (B=4,8) vs sequential ---\n";
    {
        unsetenv("ROVER_LG_EMPTY_CACHE");
        auto aliked = std::make_shared<ALIKED>("aliked-n16", device, kTopK, 0.3f, 20000);
        auto rgb_one = [&](const cv::Mat& g) {
            cv::Mat rgb = ToRgbResized(g, kResizeMax);
            cv::Mat f;
            rgb.convertTo(f, CV_32F, 1.0 / 255.0);
            std::vector<cv::Mat> ch(3);
            cv::split(f, ch);
            std::vector<torch::Tensor> tc;
            for (auto& c : ch) {
                tc.push_back(torch::from_blob(c.data, {c.rows, c.cols}, torch::kFloat32).clone());
            }
            return torch::stack(tc, 0);  // 3,H,W
        };

        auto bench_batch_b = [&](int B) {
            if (B > n) return;
            std::vector<torch::Tensor> imgs;
            imgs.reserve(static_cast<size_t>(B));
            for (int i = 0; i < B; ++i) imgs.push_back(rgb_one(frames[static_cast<size_t>(i)]));
            auto batch = torch::stack(imgs, 0).to(torch::Device(device));  // B,3,H,W

            // warmup
            {
                torch::NoGradGuard ng;
                (void)aliked->forward(batch.clone());
                if (torch::cuda::is_available()) torch::cuda::synchronize();
            }

            const int reps = 10;
            std::vector<double> ms;
            for (int r = 0; r < reps; ++r) {
                auto b = batch.clone();
                const auto t0 = std::chrono::steady_clock::now();
                {
                    torch::NoGradGuard ng;
                    auto out = aliked->forward(std::move(b));
                    (void)out;
                }
                if (torch::cuda::is_available()) torch::cuda::synchronize();
                const auto t1 = std::chrono::steady_clock::now();
                ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
            }

            // sequential B singles
            std::vector<double> seq_ms;
            for (int r = 0; r < reps; ++r) {
                const auto t0 = std::chrono::steady_clock::now();
                for (int i = 0; i < B; ++i) {
                    auto one = imgs[static_cast<size_t>(i)].unsqueeze(0).to(torch::Device(device));
                    torch::NoGradGuard ng;
                    auto out = aliked->forward(std::move(one));
                    (void)out;
                }
                if (torch::cuda::is_available()) torch::cuda::synchronize();
                const auto t1 = std::chrono::steady_clock::now();
                seq_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
            }

            std::cout << "  ALIKED B=" << B
                      << " stacked_mean_ms=" << mean(ms)
                      << " per_image=" << (mean(ms) / B)
                      << " | seq_B_mean_ms=" << mean(seq_ms)
                      << " per_image=" << (mean(seq_ms) / B)
                      << " | speedup=" << (mean(seq_ms) / std::max(mean(ms), 1e-6))
                      << "\n";
        };

        try {
            bench_batch_b(4);
            bench_batch_b(8);
        } catch (const std::exception& e) {
            std::cout << "  ALIKED stacked batch failed: " << e.what() << "\n";
        }
    }

    std::cout << "BENCH_OK\n";
    return 0;
#endif
}
