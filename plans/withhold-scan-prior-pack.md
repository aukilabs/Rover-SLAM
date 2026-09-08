# Initiative: Withhold-scan prior pack → slam_server

Status: active  
Tier: feature  
Stage: 7-implement  
Skips: _(none)_  
Last updated: 2026-09-06

```yaml
h2k:
  initiative: withhold-scan-prior-pack
  phase: building
  awaiting_human: false
  plan: plans/withhold-scan-prior-pack.md
```

### Bounce log

- **2026-09-06:** Rejected “synthetic portal / GT Sim3 seed” pin. New truth: holdout must pin via **real QR portal detection** on `slam_server`, and map ingestion must carry **placed portals from the refined map** (global `refined_manifest.json`), not a fake seed.

### Approvals

- **2026-09-06:** Slice shape **A–E locked**. **Mass-approve all remaining slices** (A→E). Implement one slice per implementer; review/verify between slices; no re-ask until blocked or done.

- **Dynamics note (2026-09-05):** moving people / dynamic objects are **out of scope** for Rover-SLAM v1 (see `plans/hybrid-prior-pose-graph.md` §4). Holdout validation assumes relatively static scenes; do not treat foot-traffic failures as pack/bridge blockers.
- **Alt stack (2026-09-05):** possible swap to another paper/code (remembered as “*-W … SLAM”, candidate WildGS-SLAM) **deferred** — keep investigating Rover-SLAM processing spikes / VO budget first.
- **Spike instrumentation (2026-09-05):** always-on `FrameTimingMs` in Tracking (`extract` / `pose_pred` / `local_map` / `new_kf` / totals + local-map point count). `slam_server` `/status` exposes `timing` (last + p50/p95 over 300 frames, over-100ms count). Spikes `>100ms` log a one-line breakdown to stderr. **No behavior change yet.**
- **Holdout timing run (2026-09-06):** instrumented `slam_server` on `:18080`, fixture prior pack, holdout `13-42-53` Frames (580) via `send_frame.py --fps 10 --raw`. Script: `scripts/run_holdout_timing_replay.sh`. Artifacts: `~/build/holdout_timing_status.json`, `~/build/holdout_timing_spikes.txt`, `~/build/rover-slam-holdout-timing.log`.
- **pose_pred sub-instrumentation (2026-09-06):** `FrameTimingMs` adds `pose_mm_ms` / `pose_refkf_ms` / `pose_reloc_ms` (accumulated inside `TrackWithMotionModel` / `TrackReferenceKeyFrame` / `Relocalization`). `/status` timing p50/p95 + spike log include the split. Re-run holdout to identify which sub-path dominates spikes.
- **Holdout pose_pred split (2026-09-06):** re-ran `scripts/run_holdout_timing_replay.sh` after rebuild. **Dominant spike source = `TrackWithMotionModel` (mm)**, not reloc. Artifacts: `~/build/holdout_timing_status.json`, `~/build/holdout_timing_spikes.txt`.
- **MM match vs opt + LightGlue stages (2026-09-06):** instrumented `pose_mm_match_ms` / `pose_mm_opt_ms` and `lg_prep` / `lg_fwd` / `lg_drain` / `lg_post`. Re-ran holdout. **Spikes = LightGlue `forward` inside `SearchBySP`**, not PoseOptimization / CUDA drain. See §8.

### Suggested fixes (after measuring; do not threshold-fiddle)

**Measurement overturned #1.** On this holdout run, `local_map` is cheap; spikes are dominated by `pose_pred` (motion-model / reference-KF / relocalization bucket in `Tracking.cc`).

**Measurement refined #1a.** Sub-instrumentation shows **`TrackWithMotionModel` owns the pose spikes** (`p95 pose_mm ≈ pose_pred`; spike dominant **mm 46 / refkf 2 / reloc 0** among 48 spikes). Reloc is cheap here (max ≈4.6ms). RefKF is an occasional fallback (max ≈61ms).

**Measurement refined #1b.** Inside MM: **`SearchBySP` / LightGlue owns spikes** (spike dominant **match 167 / opt 0**). Inside LightGlue: **`matcher_->forward` owns spikes** (**fwd 163 / prep 4 / drain 0 / post 0**). PoseOptimization is tiny (p95 ≈1.5ms, max ≈3.9ms). CUDA drain/sync is small (p95 ≈2.9ms, max ≈5.7ms). Spike frames typically **1024/1024** keypoints; rare **lg_calls=2** when MM retries SearchBySP.

Ranked by expected payoff for “steady VO @ capture FPS without grocery/office knob wars” (**re-ranked 2026-09-06 #1b**):

1. **Cut LightGlue forward cost in `TrackWithMotionModel`** — primary behavior target. Options (design next; do not blind-threshold): lower `top_k` for tracking-only match; match a subset / every-Nth when inliers already healthy; skip second SearchBySP retry when first is close; replace full LG with cheaper temporal assoc when motion is small; avoid `emptyCache` each match if later proven harmful (not the spike here). PoseOptimization is **not** the spike.
2. **Never block track on LocalMapping** — ensure KF insertion / AcceptKeyFrames path cannot stall the track thread; mapping stays best-effort under load. (Still plausible for rare stalls; not the p95 story here.)
3. **Drop-oldest backpressure** — when queue/worker exceeds budget, drop oldest frames (phone reality) instead of under-pacing replay or growing queue → 503s.
4. **Keep loop closing off** for live nav (already `loopClosing: 0`); do not re-enable as a robustness fix.
5. **Deferred / deprioritized:** hard-cap local-map search — **rejected as primary** by holdout evidence (`p95 local_map ≈ 1.9ms` vs `p95 extract ≈ 40ms`, `p95 pose_pred ≈ 58ms`; spike dominant stage = pose on 39/47 frames). Revisit only if a denser prior-loaded map changes the profile.
6. **Optional later:** person/dynamic masks (out of scope for Rover-SLAM v1); fixed-budget VO stack (e.g. DPVO) only if Rover tails remain unacceptable after (1)–(3).
7. **Reloc budget** — not the holdout spike driver (leave alone unless LOST-heavy runs).
8. **PoseOptimization budget** — **rejected as spike driver** (max ≈3.9ms on #1b holdout).
Living document — update in place when later work changes earlier conclusions.  
**§1 Intent (feature) and §4 System design stay distinct — never merge.**

---

## Intake notes (repo ground truth)

- Fixture job artifacts (reconstruction-server, read-only consume):
  `_local/job-artifacts/20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete/`
  - `datasets/` — 4 scans (`2026-04-02_13-38-55`, `13-40-15`, `13-41-55`, `13-42-53`)
  - `refined/local/<scan>/sfm/` — per-scan COLMAP + `features.h5` / `matches.h5`
  - `refined/global/refined_sfm_combined/` — combined COLMAP bins (`points3D.bin` ~23 MB); portals in `refined_manifest.json`
- **Locked (2026-09-04):** pack from **refined only** (no intermediate/local-only map); holdout query scan **`13-42-53`**; map pack = **all three non-holdout** (`13-38-55` + `13-40-15` + `13-41-55`).
- Rover-SLAM already has prior-pack v1 + `scripts/convert_to_prior_pack.py` + `slam_server --prior-pack` (hybrid-prior-pose-graph S1–S6).
- Converter input is **recon layout** (`images.json`, `tracks.json`, `covisibility.json`, `features/{id}.npz|json`, `portals.json`, image blobs) — **not** raw COLMAP/`features.h5`.
- Gap: no COLMAP + hloc `features.h5` → recon-layout exporter yet (this initiative’s bridge).
- Hybrid initiative remains at stage 10-integrate pending human disappear/reappear demo; this initiative is the next product validation on real recon.

## 1. Intent

- **Customer / internal need:** Prove MrNeRF-refined reconstruction can feed `slam_server` as a prior map, and that a held-out scan localizes against that map.
- **Framing:** Build a prior pack from refined global outputs for the three map scans; withhold `13-42-53` frames as the query stream; run `slam_server` with the pack loaded and measure localization / prior soft-edge evidence.
- **Why now:** Refined global artifacts are on disk; prior-pack path in Rover-SLAM is ready once the COLMAP→pack bridge exists.
- **Success look like:** Validated prior pack from **global refined** (map = `13-38-55`+`13-40-15`+`13-41-55`) including **placed portals**; `slam_server` loads pack; holdout `13-42-53` **detects real portal QRs**, domain-pins, then shows prior soft-edge / ATE evidence (§5).
- **Locked:** 2026-09-04 (holdout `13-42-53`; map = all three non-holdout; refined-only). **Bounce 2026-09-06:** pin = real portal detection + pack-embedded map portals (no synthetic/GT seed).

## 2. Alternatives

- **Existing / partial:**
  - Synthetic `testdata/prior_pack_fixture` — proves loader/API, not real recon.
  - Portal-only live SLAM — domain poses after QR, no COLMAP prior landmarks.
  - Pack from local-only `refined/local/*/sfm` — rejected earlier (empty/weak points on some scans; prefer global).
  - Prior-only offline matching (no live SLAM) — rejects novelty; not product path.
- **Comparison:** Fixture is insufficient for product claim. Local-only is weaker than global refined. Prior-only conflicts with hybrid Intent.
- **Decision:** **Proceed** — export refined global (+ local ALIKED features for map images) → prior pack → `slam_server` holdout run.

## 3. Priority

- **Rank:** Top-of-queue follow-on validation after hybrid-prior S1–S6 agent work; ahead of EigenPlaces / mesh occlusion.
- **Opportunity cost:** Displaces hybrid human demo polish and release gate slightly — acceptable; this is the stated next proof on real artifacts.
- **Decision:** top-of-queue.

## 4. System design

- **Decision:** **Design change needed** (bridge + wire pack portals into live portal alignment).
- **Products / codebases:** Rover-SLAM `slam_server` + prior-pack scripts; consume reconstruction-server job artifact (read-only).
- **Pack geometry (locked):** Build from **`refined/global`** COLMAP (`refined_sfm_combined`) for the three map scans + per-scan local `features.h5` for those images → recon layout → `convert_to_prior_pack`. Exclude holdout `13-42-53` images/tracks from the pack. (Already the 2026-09-04 refined-only choice; confirm exporter does **not** fall back to local-only SfM for poses/points.)
- **Pack portals (locked 2026-09-06 bounce):** Embed **placed portals from global refinement** — source = `refined/global/refined_manifest.json` (`MQAUVO7PXZ1`, `HBORRXLUOD6` with metric `physicalSize` + domain poses). Write into recon `portals.json` → pack `portals.json`. Do **not** invent synthetic portals.
- **Live pin path (locked):** On holdout replay, run **actual portal QR detection** in `slam_server` (existing detect + PnP path). Domain pin comes from detecting holdout frames’ real QRs against the **map’s placed portal catalog**.
- **Ingestion wiring (gap today):** Pack loader already parses `portals[]`, but alignment only loads `--portal-manifest`. Change: when `--prior-pack` is loaded, **use pack portals as the portal catalog** for detection/alignment (same shape as manifest). Separate `--portal-manifest` remains optional override; for this holdout run, pack alone should suffice.
- **Holdout replay:** `slam_server --prior-pack <real_pack>` (portal detect on); feed `13-42-53/Frames`. Expect ≥1 accepted portal observation → domain pin → prior soft edges can fire.
- **ATE framing:** After portal pin, compare SLAM trajectory to holdout local COLMAP **in domain frame** (one global check — not per-map Umeyama as the success metric). Soft guide in §5.
- **Risks / unknowns:** Holdout may see portals only in early/late windows; detect-every / replay FPS must not skip those frames. Soft edges need covisible prior matches after pin. Exporter bridge (COLMAP+h5→recon) still missing.
- **Coordinate / image / QR conventions (hard — human callout 2026-09-06):** Treat frame bugs as first-class failure mode (meters-scale ATE / bad pin), not tuneables. **If unsure, copy math from reconstruction-server** (`utils/data_utils.py`, `refinement_util.py`) — do not invent transforms.
  - **Cam-from-world vs cam-to-world (locked):**
    - COLMAP / pycolmap: `image.cam_from_world` = **world→camera** (`T_cw`). Camera center = `cam_from_world.inverse().translation` (see recon-server viz / holdout eval).
    - ARKit ingest in recon-server: OpenGL pose → `convert_pose_opengl_to_colmap` → treat as **cam_to_world** → `.inverse()` → store as `cam_from_world`.
    - Prior pack schema + soft edges (`poseToTwc`): image `pose` is **camera-to-world Twc** (position = camera center in domain; rotation = R_wc). Exporter **must invert** COLMAP `cam_from_world`; writing `cam_from_world.translation` as pack position is wrong.
  - **Portal catalog frame (from recon-server write path):** `refined_manifest` portals are written as **OpenGL** after `rectify_portal_pose` in COLMAP then `convert_pose_colmap_to_opengl` (`data_utils` manifest builder). `slam_server::loadPortalManifest` loads those JSON poses **as-is** + wall→QR-plane `−π/2` about Y (API text claiming OpenGL→COLMAP on load is **stale vs code**). Pack portals from `refined_manifest` must go through that **same** load path.
  - **Pack domain consistency:** Image Twc + track XYZ + portal poses (after load) must share **one** domain. Pack exporter uses world remap `M`: `t'=M t`, `R'=M R`, `P'=M P` (left-multiply only — **not** recon-server componentwise quaternion conjugation). Portals from `refined_manifest`: position unchanged, `R_pack=R_conj@M`. Self-reproj gate on export.
  - **physicalSize (locked):** Assume **same metric QR size in SLAM PnP as in the map** — copy `physicalSize` from `refined_manifest` / scan Manifest unchanged (~0.05 m). No rescaling / guessing.
  - **Features vs pixels:** `features.h5` keypoints must align with the **exact** image pixels fed to LightGlue (no silent EXIF rotate/flip, no mismatched resize). If COLMAP names imply a rotated view vs on-disk `Frames` JPGs, fix in exporter or document the required decode path.
  - **Holdout replay images:** Same uprightness/intrinsics as during map build for QR + VO; `--raw` replay must not introduce a different orientation than COLMAP registration.
  - **QR corner order:** OpenCV `detectAndDecodeMulti` corners must stay matched to `portalObjectPoints` order `(-s,-s,0),(+s,-s,0),(+s,+s,0),(-s,+s,0)`. Wrong order → flipped/rotated PnP → bad Sim3. Prefer reproj gate + sanity check (scale ~1, portal plane normal) over silent accept.
  - **ATE compare:** Domain-pinned trajectory vs holdout COLMAP — convert GT with the **same** OpenGL/COLMAP policy as pack/portals; don’t mix `cv_to_three` orientation-only flip into position ATE; don’t Umeyama-away a frame bug. Gross error → bounce conventions, don’t loosen §5 soft bar.
- **Out of scope:** Synthetic/GT Sim3 seed; prior-PnP-without-QR bootstrap (deferred).

## 5. Verification design

- **End-to-end acceptance (holdout `13-42-53` + real prior pack):**
  1. Pack builds from **global refined** + `refined_manifest` portals; `validate_prior_pack` passes; `counts.portals ≥ 2` (MQAU…, HBOR…).
  2. `slam_server --prior-pack <real_pack>` (no synthetic seed; portal detect on) loads pack; `/status` shows `prior_pack.loaded` and non-zero portal count used for alignment.
  3. Replay holdout Frames → **`portal_observations ≥ 1`** and active map **domain-pinned** (`overlay` / alignment status).
  4. After pin: **`overlay.prior_soft_edges_accepted > 0`** (real matches, not `ROVER_PRIOR_SOFT_EDGE_FIXTURE_SMOKE`).
  5. **ATE (soft guide, 2026-09-06):** vs holdout local COLMAP in **domain frame** after pin — expect roughly **median ≤ 15 cm**, **RMSE ≤ 30 cm**. Not a hard CI fail yet; **way higher** (e.g. meters / large bias) → treat as **incorrect implementation** (often frame/corner/flip bug) and bounce, not “tune thresholds.”
  6. **Convention smoke (required before trusting ATE):** pack notes state Twc vs cam_from_world; portal pin scale sensible (~O(1)); mean portal reproj within `portal_max_reproj_px`; `physicalSize` matches manifest; spot-check one QR shortId; image size == feature image size (no EXIF skew).
- **Integration / regression:** Fixture pack still loads; `--portal-manifest` override still works if set; timing scripts unchanged unless they must pass portal args.
- **Concrete observables:** `/api/v1/status` (`prior_pack`, `overlay.prior_soft_edges_accepted`, portal/alignment fields); trajectory + `scripts/eval_holdout_pose_accuracy.py` (domain-aligned mode); pack validate CLI.
- **Out of scope for v1 verify:** prior-PnP-without-QR; dynamic-object robustness; hard ATE CI gate.

## 6. Work breakdown (AI-oriented)

_Locked 2026-09-06 (mass-approved A–E):_

| # | Slice | Boundary | Depends | Human check? |
|---|--------|----------|---------|--------------|
| A | **Wire pack portals → alignment catalog** | When `--prior-pack` loaded, feed pack `portals[]` into the **same** catalog path as `loadPortalManifest` (wall→QR Y fix, corner object model unchanged). `--portal-manifest` overrides if both set. Audit docs vs code on OpenGL↔COLMAP. No exporter yet. | — | no |
| B | **COLMAP+h5 → recon exporter** | Script: global `refined_sfm_combined` + map-scan `features.h5` + `refined_manifest` portals → recon layout; **exclude** holdout. **Math:** COLMAP `cam_from_world` → pack **Twc** via `.inverse()`; `t'=M t`, `R'=M R`, `P'=M P`; portals `R_pack=R_conj@M`; self-reproj gate. | — | no |
| C | **Build + validate real prior pack** | Run exporter → `convert_to_prior_pack` → validate; document one-liner / artifact path under `out/` or `~/build`. | B | spot-check portals/counts + image/feature size |
| D | **Holdout replay w/ real pack + portal detect** | Runner: start server with real pack, replay Frames (same orientation as map), assert pin + `prior_soft_edges_accepted > 0`; log portal reproj / scale. | A, C | yes — eyeball status |
| E | **Domain-frame ATE report** | Adapt pose eval for portal-pinned / domain trajectory; report median/RMSE vs soft guide; if meters-scale, dump frame-convention suspects before any threshold change. | D | yes — interpret ATE |

- **Stop / escalate:** Pack has 0 portals; no QR detections on holdout; soft edges stay 0 after pin; ATE meters-scale → reopen §4 (wiring/frame/export), don’t threshold-fiddle.
- **Agent vs human:** A–C agent; D–E agent run + human read of status/ATE.
- **Convention constraints:** see §4 (Twc vs cam_from_world, OpenGL portals, physicalSize, QR corners, flips).

## 7. Implementation notes

### Slice A — pack portals → alignment (e7ab1f44, 2026-09-06)

- **Done:** Review approve; verify pass. Shared `ingestPortalCatalogEntries`; pack portals load when no `--portal-manifest`; manifest path overrides. Fixture: alignment.enabled + `prior_pack_portals_loaded_waiting_for_detection`. Script: `scripts/_verify_slice_a_portals.sh`.

### Slice B — COLMAP+h5 → recon (b83774a7, 2026-09-06)

- **Done:** `scripts/export_colmap_h5_to_recon.py`. Verify: 1655 images, 2 portals, holdout excluded; `out/recon_verify_slice_b/`.

### Slice C — real prior pack (e8a690c2, 2026-09-06)

- **Done:** `out/prior_pack_map_refined` (1655 images, 2 portals); `scripts/build_holdout_prior_pack.sh` (WSL).

### Slice D — holdout + portal detect (8e2529dd, 2026-09-06) — BLOCKED on soft edges

- **Runner:** `scripts/run_holdout_prior_pack_replay.sh`; pack prep `scripts/ensure_prior_pack_for_slam_server.py` (cnpy needs uncompressed npz).
- **Portal pin: PASS** — HBORRXLUOD6 ×10; reproj ~3–8 px; scale ≈0.843 (`portal_camera_baseline`); overlay map 0 pinned.
- **Soft edges: FAIL** — `prior_soft_edges_accepted=0`; early `no_gated_match`, later `no_domain_pose` after map resets. Supporting fixes landed: full-res QR detect, cnpy loader, LightGlue build flag, prior image size to matcher.
- **Artifacts:** `~/build/holdout_prior_pack_evidence.json`, `out/holdout_prior_pack_replay/`.
- **Escalate:** do not threshold-fiddle; need decision on soft-edge investigate vs defer.

### pose_pred sub-instrumentation (slice 087781d0, 2026-09-06)

- **Source (complete, timers only):** `FrameTimingMs` adds `pose_mm_ms` / `pose_refkf_ms` / `pose_reloc_ms`; RAII `PoseSubAcc` in `TrackWithMotionModel`, `TrackReferenceKeyFrame`, `Relocalization` (`Tracking.cc`); reset via `mLastFrameTiming = FrameTimingMs()` each frame.
- **`/status`:** `StageTimingStats` records last + p50/p95 + max for sub-paths; spike log (`>100ms`) prints `mm=` / `refkf=` / `reloc=`.
- **Scripts:** `scripts/run_holdout_timing_replay.sh` prints sub-path compare; `scripts/_rebuild_pose_sub.sh` rebuilds `slam_server`.
- **Rebuild + verify (2026-09-06):** `make slam_server` OK; holdout re-run complete — see §8 “Holdout pose_pred sub-paths”.

### MM match / LightGlue stage timers (2026-09-06)

- **Source (timers only):** `FrameTimingMs` adds `pose_mm_match_ms` / `pose_mm_opt_ms` + `lg_prep_ms` / `lg_fwd_ms` / `lg_drain_ms` / `lg_post_ms` / `lg_calls` / `lg_kpts*`.
- **Tracking:** wall-time around `SearchBySP` (+ retry) and `PoseOptimization` in `TrackWithMotionModel`; copies thread-local `LightGlueMatchTimingSnapshot()` after match.
- **Matcher:** `LightGlueMrNerfMatcher::MatchImpl` stages prep / forward / `MrNerfCudaDrainForHostAlloc` / post.
- **`/status` + spike log:** include `mm_match` / `mm_opt` / `lg_*`; script parses dominant LG stage.
- **Rebuild:** `scripts/_rebuild_mm_match_timing.sh`.

## 8. Verification record

### Holdout VO timing @ 10 FPS (2026-09-06)

- **Setup:** `slam_server` + `iphoneSLAM.yaml` + `testdata/prior_pack_fixture` (isolates VO; `prior_ms=0`), replay holdout `…/2026-04-02_13-42-53/Frames` (580 jpgs) with `send_frame.py --fps 10 --raw --intrinsics 1335.984 1335.984 716.9096 963.1064`.
- **Outcome:** `total_requests=580`, `successful_tracks=578`, `failed_tracks=0`, `avg_processing_time_ms≈73`, rolling window `samples=300`.
- **p50_ms:** extract **34.1**, pose_pred **23.4**, local_map **0.58**, total **67.3**
- **p95_ms:** extract **40.0**, pose_pred **58.5**, local_map **1.9**, total **99.8**
- **Spikes:** `over_100ms_count=47`. Dominant stage among spikes: **pose 39**, extract 1, local_map **0**, other/unaccounted 7. Spike pose p50/p95/max ≈ **60.9 / 87.8 / 164** ms. `max_local_map_ms≈7.1` (local_pts max 2612). Cold-start frame 1 extract≈1091ms (ignore for steady-state).
- **Conclusion:** Do **not** implement local-map search cap as the next behavior change. Next instrument/fix target is **`pose_pred`**.

### Holdout pose_pred sub-paths @ 10 FPS (2026-09-06)

- **Setup:** rebuilt `slam_server` with `pose_mm` / `pose_refkf` / `pose_reloc` timers; same fixture pack + holdout replay (`scripts/run_holdout_timing_replay.sh`). Slice `087781d0` verified.
- **Outcome:** `total_requests=580`, `successful_tracks=578`, `failed_tracks=0`, window `samples=300`, `over_100ms_count=48`.
- **p50_ms:** pose_pred **23.3**, pose_mm **19.9**, pose_refkf **0**, pose_reloc **0**
- **p95_ms:** pose_pred **65.8**, pose_mm **63.2**, pose_refkf **0**, pose_reloc **3.6**
- **max:** pose_mm **106.2**, pose_refkf **61.2**, pose_reloc **4.6**
- **Spikes:** among 48 spikes with sub-paths, dominant = **mm 46 / refkf 2 / reloc 0**. Worst: frame **535** total **198.2**, pose **161.2** (`mm=101.4`, `refkf=59.8`).
- **Conclusion:** Next behavior target is **`TrackWithMotionModel`** (likely match + PoseOptimization). Reloc is not the spike driver on this holdout.

### Holdout MM match / LightGlue stages @ 10 FPS (2026-09-06)

- **Setup:** rebuilt `slam_server` with `pose_mm_match` / `pose_mm_opt` + `lg_prep|fwd|drain|post`; same fixture pack + holdout (`scripts/run_holdout_timing_replay.sh`, `scripts/_rebuild_mm_match_timing.sh`).
- **Outcome:** `total_requests=580`, `successful_tracks=578`, `failed_tracks=0`, `avg_processing_time_ms≈86`, window `samples=300`, `over_100ms_count=167` (higher than prior run — same path still).
- **p95_ms:** pose_pred **82.5**, pose_mm **80.6**, pose_mm_match **78.7**, pose_mm_opt **1.5**, lg_fwd **73.8**, lg_prep **3.2**, lg_drain **2.9**, lg_post **0.6**, extract **47.2**, local_map **6.6**, total **134**.
- **max:** pose_mm_match **140.1**, lg_fwd **135.2**, pose_mm_opt **3.9**, lg_drain **5.7**.
- **Spikes (167):** dominant pose subpath **mm 166 / refkf 1**; MM split **match 167 / opt 0**; LG stage **fwd 163 / prep 4**. Worst pose: frame **265** total **188**, pose/mm/match **140**, **lg_fwd 135**.
- **Conclusion:** Next behavior change must target **LightGlue `forward` cost** (and/or how often full 1024×1024 LG runs in MM). Do **not** spend effort optimizing PoseOptimization or CUDA drain for this spike class.

### Holdout pose accuracy vs local COLMAP refine (2026-09-06)

- **Setup:** `scripts/eval_holdout_pose_accuracy.py` (+ `_run_holdout_pose_eval.sh`). Replay holdout Frames with `X-Capture-Timestamp` from `Frames.csv` + `X-Frame-Id`=image index. GT = `refined/local/2026-04-02_13-42-53/sfm` COLMAP bins; camera centers → OpenGL via `convert_pose_colmap_to_opengl` (same as recon-server `data_utils.py`). SLAM = `GET /api/v1/trajectory?aligned=0&dense=1&all_maps=1` (new `all_maps` export). Match by nearest capture timestamp → image index; **Umeyama (Sim3) per map**; top-down plot with least-variance axis as up.
- **Artifacts:** `out/holdout_pose_eval/holdout_pose_topdown.png`, `holdout_pose_metrics.json`, `trajectory_all_maps.json`.
- **Counts:** GT **580** poses; SLAM export **112** poses across **5** atlas maps (82 keyframes + 30 dense on active map only). Old maps lose dense frame lists on map reset — export is KF-only for inactive maps. Gaps between map spans ≈ tracking losses / re-inits.
- **Top-down:** height axis = **Y** (least motion); plot **X–Z**.
- **ATE after per-map Umeyama (scale free):** overall matched **112**, RMSE **6.4 cm**, median **3.7 cm**, p95 **13.8 cm**, max **18.9 cm**. Per-map scales vary (**0.22–1.08**) — expected for mono VO segments without shared metric pin.
- **Caveat:** This is relative-path accuracy after Sim3, **not** absolute domain alignment (no real prior map / portal pin). Sparse SLAM samples vs GT mean the plot under-represents failed intervals.
- **Re-run note (same day):** A second replay (`~/build/holdout_pose_eval/`, `frame_id−1` index match) also saw **5** maps / sparse export (~127 poses) and the same qualitative picture: cm-level local shape on good maps, map‑1 worst, mid-scan gaps. Prefer the timestamp-matched artifacts under `out/holdout_pose_eval/` as the recorded baseline. Canonical runner: `scripts/eval_holdout_pose_accuracy.py` + `scripts/_run_holdout_pose_eval.sh`.
- **Next (locked 2026-09-06):** Dense trajectory for **every atlas map**, not KF-only inactive maps. Approach: `System::GetMapDenseTrajectory(map_id)` filter `mlRelativeFramePoses` by that map’s KF set; `GET /trajectory?dense=1&all_maps=1` uses it for inactive maps. Rejected for this pass: session-wide absolute Twc log. Then re-run holdout ATE.
- **Approvals:** Dense trajectory export for all atlas maps — **approve this slice only** (2026-09-06).
- **Dense all-maps export — slice `d03c58fa` DONE (2026-09-06):** Review approve; verify pass.
  - Code: `System::GetMapDenseTrajectory(map_id)`; inactive maps on `GET /trajectory?dense=1&all_maps=1`; `source=all_atlas_maps_dense_frames`.
  - Holdout re-run (`scripts/_run_holdout_pose_eval.sh`): **510** SLAM poses / **6** atlas maps (vs sparse baseline **112** / 5). Per-map dense: 91/174/70/74/60/41.
  - ATE Umeyama (timestamp-matched): matched **510**, RMSE **15.0 cm**, median **4.7 cm**, p95 **37.1 cm**, max **45.7 cm** (sparse baseline 6.4 / 3.7 / 13.8 / 18.9 cm). Higher aggregate tails expected — denser sampling of weak intervals; median still ~5 cm. Worst: map 1 RMSE **24.9 cm**.
  - Artifacts: `~/build/holdout_pose_eval/{trajectory_all_maps.json,holdout_pose_metrics.json,holdout_pose_topdown.png}`. Earlier same-day intermediate: 426/5 maps, RMSE 11.3 cm — superseded by this verify run.
  - **Rotation (added 2026-09-06):** same timestamp correspondences + per-map Umeyama `R` from positions. Raw abs orientation after `R_u` alone is ~**88°** median (fixed API Three.js vs GT OpenGL camera-axis convention). After per-map fixed `R_cam` Kabsch: abs median **7.0°**, p95 **17.4°**. Frame-to-frame relative (gauge-free): median **1.5°**, p95 **4.1°**, max **7.5°**. Prefer `rot_rel` / `rot_abs_camfix` over raw abs.
  - **Next behavior target (from timing §8):** cut LightGlue `forward` cost in `TrackWithMotionModel` (not PoseOpt / CUDA drain / local-map).

## Gate (stage 11)

- **Decision:** _(pending)_  
- **Notes:**
