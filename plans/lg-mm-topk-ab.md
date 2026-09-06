# LightGlue MM top-k A/B

| Field | Value |
|---|---|
| Status | building |
| Tier | spike |
| Stage | 7-implement |

## Intent

Keep ALIKED extract at current max res / top_k≈1024. Feed fewer keypoints (512) into LightGlue for **TrackWithMotionModel** only; compare holdout timing and pose accuracy vs full 1024.

## Design

- Env `ROVER_LG_MM_TOPK` (unset/0 = no truncate). Applies only in `SPmatcher::SearchBySP(Frame&,Frame&)` (MM path).
- Truncate each side to strongest-K by `KeyPoint::response` (ALIKED score); remap match indices to full frame indices.
- Extract / other match paths unchanged.

## Verification

- Holdout replay @ 10 FPS: `/status` timing (`pose_mm_match`, `lg_fwd`, `lg_kpts`).
- `eval_holdout_pose_accuracy.py` metrics for baseline vs 512.

## Results (holdout, same binary; extract still 1024 / resize_max 1024)

| | full (1024→LG) | MM topk 512 |
|---|---:|---:|
| slam poses / maps | 504 / 6 | 462 / 5 |
| successful tracks | 578 | 551 |
| ATE RMSE / median (m) | 0.060 / 0.046 | 0.119 / 0.063 |
| rot_rel RMSE / median (°) | 2.27 / 1.58 | 2.39 / 1.73 |
| p50 pose_mm_match / lg_fwd (ms) | 31.8 / 27.4 | 32.2 / 29.4 |
| p95 pose_mm_match / lg_fwd (ms) | 85.6 / 79.8 | 126.7 / 120.4 |
| last-frame lg_kpts | 1024/1024 | 512/512 |

## Spike analysis @ full 1024 (holdout timing replay)

- `ROVER_LG_MM_TOPK` unset; log confirms `lg_kpts=1024/1024`, no truncate banner.
- Artifacts: `~/build/holdout_timing_1024.json`, `holdout_timing_spikes_1024.txt`, `holdout_spike_analysis_1024.json`

### Steady (all frames, p50 / p95)
- total 72 / 131 ms
- extract 34 / 44 ms
- pose_mm_match 31 / 84 ms (≈ lg_fwd 27 / 78)

### Spikes (total>100ms): 86 events, 84 after cold start
- Top stage: **pose 80/84**, extract only 4
- Within pose: **mm 82**, refkf 4, reloc 0
- Within mm: **mm_match always**; within LG: **lg_fwd always**
- Mean spike: total 118, pose 73, mm_match 69, **lg_fwd 64**, extract 40
- Frac of spike total: pose 62%, mm_match 58%, **lg_fwd 54%**, extract 34%
- Situation: **mm_match_heavy ~93%**; retries (`lg_calls=2`) only 5 but mean total 151 vs 117; clustered in frames **450–500** (38 spikes)

## Batch / offline microbench (2026-09-06)

Same holdout Frames dir, n=60, top_k=1024, resize_max=1024, cuda. Tool: `Examples/Monocular/bench_batch_aliked_lg` (+ `ROVER_LG_NO_EMPTY_CACHE`).

### Pass A vs B — SLAM drain vs offline sync-only (sequential pairs)

| | slam `emptyCache` | offline sync-only |
|---|---:|---:|
| extract p50 / per_image (ms) | 22.2 / 22.9 | 21.5 / 21.6 |
| match p50 / p95 (ms) | 26.3 / 62.5 | 21.2 / 62.9 |
| first-pair `lg_drain` (ms) | **20.4** | **0.009** |
| first-pair `lg_fwd` (ms) | 9.5 | 26.0 (warmup noise) |
| wall / frame-equiv (ms) | 3058 / 51.0 | 2845 / 47.4 |

Takeaway: **`emptyCache` per match costs ~20ms on drain** in the worst first-pair sample; median match improves ~5ms without it. **p95 match unchanged (~63ms)** — spikes are still `lg_fwd`, not the allocator. Extract in this warm microbench (~22ms) is already faster than live holdout (~34ms).

### Pass C — true ALIKED stacked batch vs sequential singles

| B | stacked per_image (ms) | seq per_image (ms) | speedup |
|---|---:|---:|---:|
| 4 | 10.7 | 13.7 | **1.28×** |
| 8 | 11.2 | 13.8 | **1.24×** |

Takeaway: stacking images helps **modestly** offline; not enough to explain recon-server vs live gap alone, and **live SLAM cannot stack future frames**. LightGlue C++ still has `TODO: Batching` (always B=1 pairs).

### Improvement directions (ranked for live)

1. **Default sync-only after extract/match** (drain only before Global BA) — done 2026-09-06; opt-in `ROVER_LG_EMPTY_CACHE=1`.
2. **Remove MM same-param LightGlue retry** (`nmatches<20` → second `SearchBySP`) — done 2026-09-06.
3. Attack **`lg_fwd` p95** (FP16/TensorRT, fewer layers, adaptive k when inliers high) — remaining spike driver.
4. Offline-only: ALIKED image batch (~1.25×) + future LightGlue multi-pair batch — irrelevant to realtime MM.

## Holdout A/B: sync-only + no MM retry (2026-09-06)

Same binary family / holdout @ 10 FPS. Baseline = pre-change binary (`emptyCache` every extract+match + MM retry). New = default sync-only + single LG call.

| | baseline (drain+retry) | sync + no-retry |
|---|---:|---:|
| successful_tracks | 578 | 578 |
| matched poses (Umeyama) | 352 | **562** |
| maps | 4 | 4 |
| ATE RMSE / median (m) | 0.129 / 0.056 | 0.155 / 0.115 |
| rot_rel RMSE / median (°) | 2.12 / 1.48 | 2.46 / 1.83 |
| total p50 / p95 (ms) | 74.2 / 149.8 | **50.8 / 100.4** |
| extract p50 / p95 (ms) | 35.7 / 46.0 | **14.4 / 24.1** |
| pose_mm_match p50 / p95 (ms) | 27.0 / 96.2 | 29.1 / **72.8** |
| lg_fwd p50 / p95 (ms) | 22.8 / 90.8 | 27.8 / **70.8** |
| lg_drain p50 / p95 (ms) | 1.84 / 3.12 | **0.009 / 0.094** |

Artifacts: `~/build/holdout_pose_eval_{baseline_drain_retry,sync_noretry}/`, `~/build/holdout_timing_*.json`.

**Read:** Wall time drops ~**32%** p50 / ~**33%** p95; extract alone ~**2.5×** faster (emptyCache was the extract tax). Match p95 improves; match p50 flat/slightly up (fwd noise). Accuracy: same track success; **more** GT-matched poses; ATE/rot_rel modestly worse on this run — treat as run variance + different matched set, not a clear regression from dropping the no-op retry. Remaining spikes still `lg_fwd`.

## Holdout A/B ×3 (2026-09-06 evening)

Same binary; baseline = `ROVER_LG_EMPTY_CACHE=1` + `ROVER_LG_MM_RETRY=1`; new = defaults (sync-only, no retry). Artifacts: `~/build/holdout_ab_repeat/` (`aggregate.txt`).

| metric | baseline mean±std [runs] | new mean±std [runs] | Δ mean |
|---|---|---|---:|
| tracks | 578 ± 0 | 578 ± 0 | 0 |
| matched | 473 ± 7 | 540 ± 27 | +67 |
| maps | 5.3 ± 0.6 | 4.3 ± 0.6 | −1.0 |
| ATE RMSE (m) | **0.103 ± 0.029** [0.123, 0.117, 0.070] | **0.217 ± 0.101** [0.117, 0.318, 0.216] | **+0.114** |
| ATE median (m) | 0.073 ± 0.016 | 0.104 ± 0.014 | +0.031 |
| rot_rel RMSE (°) | 2.39 ± 0.37 | 2.43 ± 0.01 | +0.04 |
| rot_rel median (°) | 1.57 ± 0.17 | 1.71 ± 0.06 | +0.14 |
| total p50 (ms) | 74.2 ± 2.8 | **51.5 ± 4.4** | **−22.8** |
| total p95 (ms) | 131 ± 14 | **93 ± 13** | **−38** |
| extract p50 (ms) | 35.4 ± 1.1 | **14.4 ± 0.7** | **−21** |
| mm_match p95 (ms) | 83 ± 13 | **65 ± 6** | **−19** |
| lg_drain p50 (ms) | 1.82 ± 0.13 | **0.008 ± 0.001** | −1.82 |

**Verdict:** Timing win is **stable** across 3 runs. Accuracy is **not**: new ATE RMSE mean is worse with **much higher run variance** (one outlier 0.318 m). rot_rel is essentially unchanged. Fewer maps + more matched poses on new suggests longer continuous tracks — overall Umeyama ATE can look worse even if local VO (rot_rel) is fine.

**Open question:** Is ATE noise from map topology / eval aggregation, or does skipping `emptyCache` introduce rare CUDA/caching pathologies? Next isolate: (a) sync-only extract only vs match only; (b) keep defaults but discard outlier and re-run; (c) accept timing and watch rot_rel + tracks as primary.

## Plot UIs (2026-09-06)

- **Live:** `Examples/Monocular/index.html` on slam_server port — timing + top-down trajectory (`timing.series_ms` + `/api/v1/trajectory`).
- **Offline A/B:** `out/holdout_pose_eval/` served on **:8080** via `scripts/_stage_and_serve_holdout_eval.sh` — side-by-side baseline vs sync/no-retry PNGs + metrics + `aggregate.txt`.

## Max-queue floor @ 10 FPS holdout (2026-09-06)

Sync-only / no MM-retry binary; 580 frames; `--min-buffer 1`. Metric = HTTP **503** (queue full), not `failed_tracks` (always 0).

| max-queue | zero-503 rate | notes |
|---:|---|---|
| 3–4 | 0/2 each | always a few rejects |
| 5 | ~2/3 | flaky |
| 10 | 2/2 after warm + 1 earlier single-503 | practical safe floor |

**Read:** do not go below **10** if you want no rejected frames on this holdout; **5** is the optimistic edge.

