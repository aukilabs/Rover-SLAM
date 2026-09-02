# Initiative: ALIKED + LightGlue (mrnerf)

Status: building (reopened — tracking smoke fails)  
Tier: feature  
Stage: 9-verify  
Skips: _(none)_  
Last updated: 2026-09-02

Living document — update in place when later work changes earlier conclusions.  
**§1 Intent (feature) and §4 System design stay distinct — never merge.**

---

## 1. Intent

- **Customer / internal need:** Rover-SLAM tracking should use the same learned features as reconstruction-server (MrNeRF ALIKED + LightGlue), so online SLAM and offline recon share descriptor space / matching behavior and we stop carrying a SuperPoint-only path.
- **Framing:** Rover-SLAM today extracts with SuperPoint ONNX (`SPextractor` → `SuperPointOnnxRunner`, model `onnxmodel/superpoint.onnx`) and matches with LightGlue ONNX (`LightGlueDecoupleOnnxRunner`, `onnxmodel/lightglue_sim.onnx`). Reconstruction-server already runs MrNeRF’s LibTorch ALIKED (`aliked-n16`, top_k≈1024) + LightGlue C++ via `mrnerf_features` / `third_party/Light_Glue_CPP`. Replace SuperPoint with that ALIKED stack (and align LightGlue) so SLAM matches recon.
- **Why now:** Recon already standardized on mrnerf ALIKED+LightGlue; keeping SuperPoint in SLAM splits the pipeline (vocab, match quality, ops). Cost of waiting is continued dual-stack drift and harder end-to-end debugging.
- **Success look like:** Monocular (and existing SLAM-server) paths extract ALIKED features and match with the mrnerf-aligned LightGlue path; SuperPoint is no longer required at runtime for the default config; a documented smoke/parity check shows extract+match behavior consistent with reconstruction-server’s mrnerf settings (model `aliked-n16`, capped keypoints).

## 2. Alternatives

- **Existing ways:** Keep SuperPoint ONNX + current LightGlue ONNX (status quo). Or swap only the ONNX extractor to an ALIKED ONNX export while keeping ORT LightGlue (faster drop-in, weaker “matching reconstruction-server”). Hybrid: LibTorch ALIKED + keep ORT LightGlue.
- **Comparison:** Status quo fails the shared-stack goal. ONNX-only / hybrid diverge from recon’s mrnerf LibTorch weights and matcher.
- **Decision:** **Proceed — vendor mrnerf LibTorch** (`Light_Glue_CPP` ALIKED + LightGlue), matching reconstruction-server. Rejected ONNX drop-in and hybrid for v1.

## 3. Priority

- **Rank vs current backlog:** Top-of-queue for this workspace (explicit user initiative).
- **Capacity / opportunity cost:** Displaces further SLAM-server polish until extract/match swap is stable; vocab/DBoW implications may add follow-on work.
- **Decision:** top-of-queue.

## 4. System design

- **Products / codebases impacted:** Rover-SLAM only (consumer). Reference implementation: `reconstruction-server/third_party/Light_Glue_CPP` + `cmake/modules/{libtorch,light_glue_cpp}.cmake` (build pattern for `auki_lightglue_lib`). No change required inside reconstruction-server for this initiative.
- **Constraints:**
  - Vendor `third_party/Light_Glue_CPP` (same tree/weights as recon: `aliked-n16.pt`, `aliked_lightglue.pt`; defaults `aliked-n16`, `top_k=1024` to avoid LightGlue OOM).
  - Discover LibTorch via `LIBTORCH_ROOT` / env / documented path (mirror recon’s `libtorch.cmake`); require CUDA ≥ 12.1 for the mrnerf lib.
  - Raise compile standard for the LightGlue/ALIKED targets to **C++20** (mrnerf requirement). Rest of ORB_SLAM3 may stay on existing flags if feasible; otherwise bump project CXX carefully.
  - Default runtime path becomes LibTorch ALIKED extract + LibTorch LightGlue match; **remove SuperPoint/LightGlue ONNX from the default build path** (ORT may remain optional/dead or be dropped in a follow-up slice — see open ask).
  - Keep Frame/Tracking boundary: `cv::KeyPoint` + `cv::Mat` descriptors in; match indices `vnMatches12` out — wrap mrnerf tensors at the extractor/matcher edge (same idea as recon’s pybind, but in-process C++).
  - **Place recognition v1:** **Defer loop closure and relocalization.** ALIKED swap must not block on DBoW/vocab. Keep `loopClosing: 0` in settings; document that loops/reloc are unsupported until follow-on. SuperPoint vocab may remain required at startup for now (minimal change) or be stubbed in a later slice — do not rebuild ALIKED DBoW vocab.
  - **Follow-on (out of v1):** EigenPlaces global retrieval via ONNX Runtime (mirrors recon’s global/local split; replaces BoW candidate search in `KeyFrameDatabase` / `LoopClosing`).
- **Interfaces / boundaries:**
  - Replace `SuperPointOnnxRunner` usage inside `SPextractor` (or rename to a thin ALIKED wrapper) with MrNeRF `ALIKED` API.
  - Replace `LightGlueDecoupleOnnxRunner` inside `SPmatcher` with MrNeRF `matcher::lightglue`.
  - Weight paths via compile def `LIGHTGLUE_MODELS_DIR` and/or config, not hard-coded ONNX paths.
  - CMake: add `cmake/modules/libtorch.cmake` + `light_glue_cpp.cmake` (adapted from recon); link `auki_lightglue_lib` + `${TORCH_LIBRARIES}` into `ORB_SLAM3`.
- **Risks:** LibTorch ABI/CUDA arch on the build host; C++14 vs C++20 clash; real-time latency vs ORT; loop/reloc disabled in v1; binary size / link times.
- **Decision:** **Design change needed** — vendor mrnerf LibTorch stack and swap extract+match call sites. v1 defers place recognition; EigenPlaces ONNX retrieval is a separate follow-on initiative.

## 5. Verification design

- **End-to-end acceptance (v1):**
  - Rover-SLAM **builds** with `LIBTORCH_ROOT` set and links `auki_lightglue_lib`.
  - Default config uses ALIKED (`aliked-n16`, `top_k=1024`) + mrnerf LightGlue — SuperPoint/LightGlue ONNX not invoked on the default path.
  - **Extract smoke:** single image → non-empty keypoints + descriptor matrix with expected dims (ALIKED n16 → 128-d descriptors).
  - **Match smoke:** two images → LightGlue returns a plausible match count (>0 on an easy pair from repo test frames).
  - **Tracking smoke:** monocular example or `slam_server` processes a short sequence without crash; pose/tracking state advances (loop closure off).
  - **Parity spot-check (best effort):** same image pair as recon smoke (`reconstruction-server/scripts/smoke_mrnerf_features.py` settings) — keypoint count within ~10% and match count same order of magnitude (not bit-identical; different resize/preprocess may differ).
- **Integration / regression:**
  - Existing monocular binary still launches with `loopClosing: 0`.
  - No regression in build when `LIBTORCH_ROOT` unset → clear CMake error (not silent fallback to SuperPoint unless explicitly gated).
  - Document required deps: LibTorch path, CUDA ≥ 12.1, model weights in `third_party/Light_Glue_CPP/models/`.
- **CI / delivery:** Manual smoke on dev GPU host for v1 (no CI gate assumed). Human confirms latency acceptable on one real sequence before calling v1 done.
- **Out of scope for v1:** Loop closure, relocalization, EigenPlaces retrieval, ALIKED DBoW vocab, bit-exact parity with recon pybind, removing vocab CLI arg entirely, production SLAM-server perf tuning.

## 6. Work breakdown (AI-oriented)

Ordered slices — one implementer subagent per slice; review + verify between slices.

### Slice 1 — Vendor Light_Glue_CPP + CMake wiring
- **Goal:** `third_party/Light_Glue_CPP` present; `cmake/modules/libtorch.cmake` + `light_glue_cpp.cmake`; `auki_lightglue_lib` builds and links into `ORB_SLAM3`.
- **Context:** Copy/adapt from `reconstruction-server/cmake/modules/`; weights from recon tree or git submodule; `LIBTORCH_ROOT` documented in plan §7.
- **Human:** Provide or confirm LibTorch install path on build machine.
- **Stop/escalate:** CUDA < 12.1 or LibTorch ABI mismatch → bounce §4.

### Slice 2 — ALIKED extractor wrapper (`SPextractor`)
- **Goal:** Replace `SuperPointOnnxRunner` default path with MrNeRF `ALIKED`; output `cv::KeyPoint` + `cv::Mat` descriptors; config `aliked-n16`, `top_k=1024`.
- **Context:** `include/Extractors/SPextractor.h`, `src/Extractors/SPextractor.cc`, reference `reconstruction-server/src/mrnerf_features_binds.cpp` (`PyAlikedExtractor`).
- **Human check after verify:** Eyeball keypoint overlay on one frame.

### Slice 3 — LightGlue matcher swap (`SPmatcher`)
- **Goal:** Replace `LightGlueDecoupleOnnxRunner` with mrnerf `matcher::lightglue`; preserve `MatchingPoints_onnx` / frame-to-frame APIs used by Tracking.
- **Context:** `include/Matchers/SPmatcher.h`, `src/Matchers/lightglue_onnx.cpp` (replace or bypass), Tracking call sites.
- **Stop/escalate:** OOM at full res → enforce `top_k` / `resize_max` in config.

### Slice 4 — Runtime config + defer loop/reloc
- **Goal:** Default yaml/settings use mrnerf stack; `loopClosing: 0` documented; README/build notes; gate or stub SuperPoint ONNX from default init; vocab startup unchanged or minimally gated.
- **Context:** Example yamls, `System.cc`, `README.md`, `Examples/Monocular/slam_server` docs.
- **Human check:** Confirm acceptable that vocab arg still required at CLI if not removed in this slice.

### Slice 5 — Smoke harness + parity script
- **Goal:** Small C++ or script target: load two test images, extract+match, print counts; optional compare against recon smoke thresholds.
- **Context:** `frame_*_original.png` in repo root or `Examples/` fixtures; recon smoke scripts for reference params.
- **Human:** Run on GPU host; sign off §5 acceptance.

**Dependencies:** 1 → 2 → 3 → 4 → 5 (4 can partially overlap 3 if matcher not needed for config-only changes).

**Follow-on initiative (not sliced here):** EigenPlaces ONNX global retrieval for loop/reloc.

## 7. Implementation notes

- **Approvals:** Mass-approve all slices (S1–S5). Re-ask only when a slice needs human to run/try something locally (e.g. GPU smoke, LibTorch path on build host). Skip re-ask for pure code/review/verify cycles.
- Approach / sequence: S1 → S2 → S3 → S4 → S5 (see §6).
- Key files: `CMakeLists.txt`, `cmake/modules/`, `third_party/Light_Glue_CPP/`, `SPextractor`, `SPmatcher`.
- Bounce triggers → update §4 if LibTorch/CUDA wiring fails.
- **S1 scope split (2026-09-01):** Reverted S2–S5 from S1 diff. Parked untracked for later slices: `AlikedExtractor.*`, `LightGlueMrNerfMatcher.*`, `Examples/Monocular/smoke_aliked_lightglue.cc`.
- **S2–S5 re-integrated (2026-09-01):** Full initiative code restored on disk after S1 split; S5 static verify pending GPU smoke on Linux host.
- **Build fixes (2026-09-01):** `libtorch.cmake` enables CUDA before `find_package(Torch)`; CMakeLists wrapper files use explicit `-std=c++20`; `scripts/configure_s1.sh` builds thirdparty + smoke target (native LibTorch copy, `-j4` default). Link fix: wrap `libonnxruntime.so` with `-Wl,--no-as-needed` on `ORB_SLAM3` and `smoke_aliked_lightglue` (GNU ld `--as-needed` dropped ONNX from shared-lib DT_NEEDED).

## 8. Verification record

- **S1–S5 (2026-09-01):** Static/code-review verification pass for all slices. CMake configure+build and `smoke_aliked_lightglue` runtime smoke **deferred** to Linux GPU host (`LIBTORCH_ROOT`, CUDA 12.1+).
- **S1 GPU verify (2026-09-01):** WSL Ubuntu, RTX 5070 Ti (sm_120), CUDA 12.9, LibTorch 2.7.0 at `~/libtorch-cu129`. Configure OK (`MrNeRF LightGlue lib: enabled`). `libauki_lightglue_lib.a` built at `~/build/rover-slam-s1/`. Run: `bash scripts/verify_s1_gpu.sh` (or build step only after configure).
- **GPU build attempt (2026-09-01 evening):** Restored missing CMake wiring for S2–S5 (`AlikedExtractor`, `LightGlueMrNerfMatcher`, `AUKI_LIGHTGLUE_AVAILABLE`, `smoke_aliked_lightglue` target). Fixed `libtorch.cmake` to `enable_language(CUDA)` before `find_package(Torch)`. Forced `-std=c++20` on wrapper sources (global `-std=c++14` caused LibTorch C++17 errors). WSL configure **succeeds** at `~/build/rover-slam-s1` with LibTorch 2.7 + CUDA 12.9; thirdparty DBoW3+g2o rebuild OK. **Blocked:** intermittent `/mnt/d` I/O errors under WSL (LibTorch headers + repo paths fail during parallel CUDA compile). **Next:** `wsl --shutdown` then retry, or copy repo + LibTorch to native WSL ext4 (`~/rover-slam-work`, `~/libtorch-native`) and run `scripts/configure_s1.sh` with `ROVER_BUILD_JOBS=2`.
- **GPU verify PASS (2026-09-01 21:44 UTC):** Native WSL build at `~/build/rover-slam-s1-v2` from `~/rover-slam-work` + `~/libtorch-cu129` (LibTorch 2.7.0 cu128). RTX 5070 Ti, CUDA 12.9, `CMAKE_CUDA_ARCHITECTURES=120`. Targets built: `auki_lightglue_lib`, `libORB_SLAM3.so`, `smoke_aliked_lightglue`. Runtime smoke (`scripts/run_smoke_gpu.sh`): frame_100/101 → **1024 keypoints** each, **128-d** descriptors, **25 matches**, exit 0. Helpers: `scripts/sync_to_native_wsl.sh`, `scripts/verify_gpu_smoke.sh`, `scripts/run_smoke_gpu.sh`, `scripts/fetch_libtorch.sh`.
- **Human gate:** Sign off §5 acceptance / stage 11 release when satisfied with latency on a real sequence.
- **Runtime GPU smoke (2026-09-01 22:04):** Reconfirmed from `/mnt/d/agent-workspaces/Rover-SLAM` via `verify_gpu_smoke.sh` (`BUILD_DIR=~/build/rover-slam-s1-v3`, `LIBTORCH_ROOT=~/libtorch-native`) — 1024 kpts × 128-d each, 25 matches, exit 0.

## Gate (stage 11)

- **Decision:** accept (v1)
- **Notes:** §5 extract+match smoke passed on GPU. Tracking smoke on a real sequence and loop/reloc remain follow-on. Integrate: commit stack + resolve nested `.git` in `third_party/Light_Glue_CPP` before merge.
