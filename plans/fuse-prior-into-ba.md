# Initiative: Fuse prior alignment into BA

Status: active  
Tier: feature  
Stage: 6-work-breakdown (§4/§5 re-locked 2026-09-12 after slice E bounce — human pick B)  
Skips: _(none)_  
Last updated: 2026-09-12

Living document — update in place when later work changes earlier conclusions.  
**§1 Intent (feature) and §4 System design stay distinct — never merge.**

---

## 1. Intent

- **Customer / internal need:** Better live localization accuracy against the prior/domain map, with Pangolin and `/viewer` reflecting the same corrected Atlas — without paying for a second keyframe BA.
- **Framing:** Portal (and prior soft) constraints today only refine an **outside** overlay Sim(3). Atlas Local/Global BA never sees them, so mono drift stays in the map that tracking uses. A per-keyframe overlay pose graph (hybrid S10) would improve domain traj but would **optimize keyframes twice** (ORB BA + overlay BA). Goal is to fold absolute constraints into the **existing** BA pass instead.
- **Why now:** Overlay single-Sim3 is accuracy-capped (S9: VO ~3 cm vs portal-domain ~17 cm / bad global scale). User rejects “second BA” architectures.
- **Success look like:** Holdout / live domain accuracy improves vs overlay-Sim3-only; Atlas KF/points after pin are corrected by the same Local/Global BA that already runs; no parallel per-KF pose-graph optimizer; Pangolin ≈ `/viewer` without a display-only Sim3 (or Sim3 is identity after domainize).
- **Locked (2026-09-11):** Target is **Atlas BA write-back** (inject into ORB Optimizer / Local±Global BA). **Not** overlay per-KF Sim3 BA (S10 stays parked / superseded for this goal). Overlay may shrink to thin publish/gauge plumbing once Atlas owns the residuals.

## 2. Alternatives

- **Overlay single Sim(3) (current):** Cheap; no double BA; accuracy ceiling from one global scale absorbing mono drift.
- **Overlay per-KF Sim3 pose graph (S10):** Models per-KF scale; **second** keyframe optimization — rejected for this initiative.
- **Atlas BA write-back (chosen):** One BA; absolute portal (± prior) residuals in Optimizer; accuracy + single optimizer. Cost: deeper ORB coupling, frame/scale risk.
- **Decision:** Proceed with Atlas write-back.

## 3. Priority

- **Rank:** Insert after closed hybrid v1 / domain viewer; top of open follow-ons for accuracy.
- **Capacity:** Touches `Optimizer` / LocalMapping path — higher blast radius than overlay-only.
- **Decision:** top-of-queue for this accuracy track _(confirm if needed)_.

## 4. System design

- **Products / codebases:** Rover-SLAM `Optimizer` (Local ± Global BA), `Map`/`KeyFrame`/`MapPoint` (domainize write-back), `slam_server` portal detect + prior soft match plumbing, overlay shrink. Precedent: `Map::ApplyScaledRotation` (IMU gravity/scale path in `LocalMapping`).
- **Decision:** **Design change needed** — Atlas write-back + domainize-once (locked below).
- **Frame strategy (locked 2026-09-11 — human pick 1):**
  - On the first pooled pin that passes the gate (see re-lock below): apply the Sim(3) to **all KF poses + map points** of the active Atlas map (loop-correction / `ApplyScaledRotation`-style bulk rewrite).
  - After that, Atlas world = domain; Local/Global BA stays SE3 in that frame.
  - Absolute residuals (portal corners **and** prior soft, both v1 after the re-lock) enter the **same** BA as metric 3D→2D edges against fixed domain landmarks.
  - Overlay Sim(3) refine **retires** for that map (identity / publish-only); no second KF pose-graph BA.

### Re-lock 2026-09-12 — human pick **B**: pooled re-pin + live scale DOF + prior-soft promoted into v1

**Governing principle (carried over from `plans/hybrid-prior-pose-graph.md`, which learned it the expensive way):** adding constraints reduces **variance**, never **bias**. A new constraint moves the estimate only if all three hold — (i) it attaches to a parameter that is **free** in the same optimization, (ii) it adds information in a direction the existing constraints do **not** already span (baseline, not count), and (iii) it **persists** or is re-asserted instead of falling out of a window. Every accuracy failure in both initiatives so far violated at least one of these. Design below is organised around them.

- **Pin from pooled sights, not the first passing sight (replaces "Domainize trigger"):** `PortalObservationStore` already accumulates every accepted sight. Domainize solves the map→domain Sim(3) from **all** stored sights for that map (robust/weighted by reproj), not from one PnP. Gate on the **pooled** solve: residual reproj, scale in `[kMinScale, kMaxScale]`, finite R/t, **and** the observability check below. A single sight may pass only if it also passes observability. Rationale: map 0 pinned at 4.03 px on a **1-keyframe** map when 0.61 px was available one frame later.
- **Scale DOF stays live after domainize (replaces "no second bulk Sim(3) rewrite"):** the per-map domain Sim(3) remains an **estimate**, not a frozen event. On new evidence the pooled solve re-runs; if the delta exceeds its own uncertainty (hysteresis, so it cannot thrash), it is applied as a bulk `ApplyScaledRotation` correction. This is a closed-form solve plus a rewrite — **not** a second keyframe BA, so the §1 "no second BA" lock holds. Without this there is no free scale parameter at all after domainize and (i) fails by construction.
- **Prior-soft 3D→2D edges promoted from follow-on F into v1 (replaces "portal corner edges only"):** portal corners supply ~0.17 m of baseline at one location; the prior pack's 3D tracks span the ~22 m traverse. Scale information scales with baseline, so **portals cannot be the scale source** and portal-only v1 cannot satisfy "accuracy keeps improving with constraints" — condition (ii) fails. Gated prior-soft edges (Cauchy, lower weight) enter the same Local/Global BA and the same pooled pin solve.
- **Candidate-selection feedback trap (new, must be mitigated):** prior-soft candidates are chosen by nearest pack image **under the current domain pose**. When alignment is wrong by metres, selection fails away from the pin and accepts cluster near it (observed: images 605/606/609/612/1128/1135) — bad alignment starves the constraints that would fix it. Mitigation: widen candidate search radius by the current pose uncertainty rather than a fixed count.
- **Absolute-constraint keyframes are retained in the BA window (replaces "BA injection scope"):** a KF carrying portal or prior-soft edges is pulled into `LocalBundleAdjustment` (as a normal, non-fixed vertex) even once it leaves the covisibility window, subject to a bounded cap. Slice E produced **8 pose-only edges on 2 KFs** that left the window immediately — condition (iii) failed, so the BA thesis was never exercised.
- **Constraint evidence must survive keyframe culling (added 2026-09-12 after slice J).** `KeyFrameCulling` in `src/LocalMapping.cc` protects only the initial keyframe and map-point redundancy — it has **no exemption for constraint-observing keyframes**. Measured in the slice J smoke: an observing KF was culled within **≈10 frames** of its sight being stored, and by frame 28 only **1 of 3** stored sights was still resolvable (11 detections → 3 store appends → 1 usable). Constraint *count* therefore grows while usable *evidence* shrinks, which makes condition (iii) fail at the source rather than in the BA window. Fix belongs in slice **K**: keyframes referenced by an accepted portal or prior-soft sight are exempt from culling (bounded), and/or a sight carries its own frozen pose so it does not depend on a keyframe surviving. Without this, portal-only pooled re-pin cannot satisfy §5 #5 no matter how many sights arrive.
- **Portal evidence is structurally insufficient on the holdout; prior-soft is the only scalable constraint source _(measured 2026-09-12, full 580-frame holdout)_.** Portal detection runs on **every** frame for the whole scan (`portal=47.9 ms` still logged at frame 483), yet all **11** sights fall in frames **1-30**, all of the same portal, from one vantage. The pack holds 2 portals; the scan sees 1. One vantage cannot observe scale - the pooled solve reports `sights=1`, `seed_residuals=4` (below the 7 needed), `null=log_scale`, `sigma_log_s=nan`, so the map **never domainizes**. **No portal-side change can fix this - the information is not in the data.** §1's "keep improving with more constraints" must therefore be delivered by slice **L** (prior-soft), the only evidence stream present across the whole scan. _(Supersedes the withdrawn slice N hypothesis that the near-duplicate admission gate was throttling supply. That gate does compare against an arbitrary per-map SLAM gauge - measured scales 0.159 / 1.739 / 0.403 / 1.251 across four maps - which is a real defect, but not the binding one.)_
- **The domain frame is metric, so L delivers metric scale without reopening the banned ARposes path _(human confirmed 2026-09-12)_.** The prior pack's poses come from a COLMAP reconstruction seeded on ARKit, so `coordinate_frame: domain` is metric indirectly. Backed by measurement already on record: `plans/hybrid-prior-pose-graph.md` measured SLAM→AR scale **0.4760** against SLAM→GT **0.4724**, a ratio of **1.0076** over a 22 m path - the ARKit-derived frame agrees with ground truth to **0.76%**. This matters for the 2026-09-08 human lock banning a "parallel scale estimator" from `ARposes.csv`. That ban stands and is **not** reopened: prior-soft edges are not a second estimator running alongside the first, they are the *same* estimator anchored to a map that happens to be metric. Scale arrives as a property of the prior, not as an independent measurement to be fused.
- **The bad-alignment-starves-constraints trap is confirmed, with numbers _(2026-09-12)_.** Prior-soft skips report `err_prior_p50` ≈ **1.5 px** (prior geometry is excellent) against `err_live_p50` of **148-1187 px**, giving `pass_live=0` while `pass_prior` is 6-11. The live gate is a **fixed pixel threshold measured against the very alignment it exists to improve**: weak pin -> large live reprojection -> matches rejected -> pin never improves. It must instead be **widened by the current alignment uncertainty** (slice G's Hessian now supplies it) and tightened as the pin firms up, with `err_prior` carrying the correctness burden. Candidate supply is thin too (`lg_matches` 10-12), which is the other half of L's "widen candidate search".
- **Observability is instrumented and gates updates (new):** every pooled solve logs the 7×7 Hessian condition number, the dominant null direction, and σ(log s). A scale update is **rejected** when the report is not `valid` (rank-deficient, or `log_scale` still in the null direction) **and** when the pooled residual at the operating pin exceeds its bound — σ is a necessary but not sufficient signal (slice G). `scripts/_probe_sim3_conditioning.py` is the reference implementation. This makes "more constraints improve scale" a measured curve instead of an assumption.
- **Portal edge weight (locked default, unchanged):** overlay-style **high weight / inverse reproj**, **no Cauchy** on portal corners (outliers gated before accept). Prior-soft uses Cauchy.
- **Constraints:** must not double-apply overlay Sim(3) on top of a domainized Atlas. Tracking loss / new Atlas map → clear domainized flag; wait for the next portal. Re-pin corrections must be applied under the map lock and must not race LocalMapping/Global BA.
- **Interfaces / boundaries:** Catalog + detections still from `--portal-manifest` / `--prior-pack`; portal sight store readable by LocalMapping/Optimizer. Trajectory/status: after domainize, `coordinate_frame: domain_colmap` from raw Atlas Twc (identity overlay). Pangolin shows domainized Atlas.
- **Risks:** Bad pin that passes a weak gate still domainizes wrong. Local BA weight fight vs ORB reproj. Multi-map Atlas: domainize per map only. Portal-only gauge is sparse, which is why prior-soft is in v1. Repeated bulk rewrites (live scale DOF) must not fight LocalMapping. Global BA + portal edges must stay thread-safe vs LocalMapping.
- **Decision:** §4 re-locked 2026-09-12 per the pick-B block above. Prior-soft-into-BA is now **in v1** by explicit human decision, not silent scope creep.

## 5. Verification design

- **Depth (locked 2026-09-12 — human pick 3):** holdout ATE required + human glance of top-down / viewer after domainize.
- **End-to-end acceptance (v1, re-locked 2026-09-12 after slice E):**
  1. Atlas domainizes from a **pooled** sight solve that passes the quality **and** observability gate; log records how many sights fed the solve. Overlay Sim(3) for that map is identity / not re-refining.
  2. Subsequent portal **and prior-soft** sights increase absolute-edge counters in Local (and Global when run) BA, and absolute-constraint KFs are demonstrably retained in the window — not a second KF pose-graph optimizer.
  3. After domainize: trajectory `coordinate_frame: domain_colmap` from Atlas poses; Pangolin and `/viewer` agree without a separate display-only Sim(3).
  4. **Holdout replay, fair comparison (fixes the unfair metric the slice E bounce exposed):** score the new path and the S9 overlay baseline on the **same matched subset**, reporting **median, RMSE and p95 together**; median-only on disjoint subsets is not decisive (S9 wins on median 0.036 m while losing on RMSE, 4.95 m vs 0.367 m). Pass requires no meters-scale failure **and** no regression on RMSE or p95.
  5. **Monotonicity (the point of this re-lock; amended 2026-09-12 after slice G).** As constraints accumulate over the replay, **all three** must hold: (a) seed-pin **σ(log s) decreases** while it is `valid`, (b) **seed-pin pooled residual does not increase** beyond a stated bound, and (c) domain ATE does not degrade. Evidence is a per-run curve of σ(log s), pooled residual and ATE against cumulative absolute-constraint count. A run where constraints grow and σ(log s) stays flat is a **fail** — that is exactly the S9′ every-frame result (76 soft edges, ATE 0.292 vs 0.297 m, scale stuck ~0.465) this re-lock exists to avoid. **Why (b) is required:** slice G measured σ(log s) *tightening* (0.0075 → 0.0055) while seed-pin reprojection *exploded* (3.5 → 63.7 → 145.8 px) across 1→3 sights. Local log-scale curvature improves as added residuals push the weakest eigenvector off `log_scale` onto `omega_z`, even when the pin is nowhere near a sensible pooled minimum. σ alone would have scored that divergence as progress.
  6. **Human glance:** top-down + `/viewer` after domainize looks aligned to prior/portals.
  7. **Map reset / new map** (`Map::clear`): domainized cleared; no stale identity overlay on a raw SLAM map. Transient tracking loss alone must **not** clear domainized (avoids double Sim3 apply).
- **Integration / regression:** prior pack load + portal detect still work; fixture pack smoke; no overlay per-KF BA introduced; prior soft may still accept for diagnostics but must not run a second KF optimizer.
- **CI / delivery:** no hard ATE CI gate in v1 (agent report + human glance); document commands in §8 when run.
- **Out of scope for v1 verify:** EigenPlaces; multi-map simultaneous domainize polish beyond clear-on-new-map; S10.
- **Decision:** §5 locked as above.

## 6. Work breakdown (AI-oriented)

_A-E locked 2026-09-12 and shipped (see §8); A-D stand, E bounced on accuracy. Slices G-M added 2026-09-12 under the pick-B re-lock._

### v1 as shipped (A-E)

| # | Slice | Boundary | Depends | State |
|---|--------|----------|---------|--------------|
| A | **Portal observation store** | Shared store of accepted portal corner sights (map id, KF id, domain corners, pixels, intrinsics, weights) readable by Optimizer; wire detect path to append. | - | done |
| B | **Domainize-once** | Quality-gated first PnP -> bulk Sim(3) rewrite of active Atlas map (`ApplyScaledRotation`-style); per-map domainized flag; clear on map reset; overlay Sim(3) -> identity for that map. | A | done; **trigger superseded by H** |
| C | **Portal edges in Local + Global BA** | Inject 4x corner 3D->2D edges into `LocalBundleAdjustment` and `BundleAdjustment`; overlay high-weight / no Cauchy. | A, B | done; **scope extended by K** |
| D | **API/status + overlay retire** | Status `domainized` + portal BA edge counts; trajectory domain from Atlas; no parallel KF pose-graph BA. | B, C | done |
| E | **Holdout verify harness** | Replay + ATE vs overlay-Sim3 baseline; artifacts for human glance. | C, D | ran; **accuracy fail -> bounce**; metric superseded by M |

### Pick-B re-lock (G-M)

| # | Slice | Boundary | Depends | Human check? |
|---|--------|----------|---------|--------------|
| G | **Observability instrumentation** | Pooled-sight Sim(3) solve plus 7x7 Hessian diagnostics (condition number, dominant null direction, sigma(log s), sight count) logged and on `/status`. Reference: `scripts/_probe_sim3_conditioning.py`. **Read-only w.r.t. the pin - no behaviour change yet.** Includes a read-only probe reconciling the 0.9685 vs 0.4724 SLAM->GT scale figures, so we know whether drift modelling is warranted at all. | A | no |
| H | **Pooled re-pin at domainize** | Replace first-passing-sight domainize with a solve over **all** stored sights for the map; gate on pooled residual + scale range + finite R/t + G's observability check (`valid && null_dominant_param != log_scale`, **plus** a residual threshold — never σ alone, and never `solved_pin.sigma_log_s`). | G | no |
| J | **Live scale DOF** | Re-run the pooled solve as evidence arrives; apply the delta via bulk `ApplyScaledRotation` when it exceeds its own uncertainty (hysteresis against thrash). Closed-form solve plus rewrite only - **not** a second KF BA. | H | no |
| K1 | **Constraint-evidence survival** _(split from K, 2026-09-12)_ | Keyframes referenced by an accepted sight are exempt from `KeyFrameCulling` under a bounded cap; sights that never bound to a keyframe are rejected at append rather than resurrected; post-domainize resolvability requires a **live** keyframe pose. Resolvable count must be reported **with** the pooled residual. | C, J | no |
| K2 | **Absolute-constraint KF retention** | Pull constraint-observing keyframes into `LocalBundleAdjustment` as normal non-fixed vertices even after they leave the covisibility window, under a bounded cap — fixes the 8-edges-on-2-KFs dead end. | K1 | no |
| ~~N~~ | ~~**Metric admission gate**~~ **- WITHDRAWN 2026-09-12, refuted by measurement before implementation** | Hypothesis was that `kMinPortalSightBaselineSlam = 0.12` in arbitrary SLAM-gauge units was throttling portal supply. The full 580-frame holdout shows the gate is **not** the binding constraint: the portal is visible only in frames **1-30** and never again, so a looser gate would admit near-duplicate views from a single 3-second vantage - count without baseline, which by §4's own principle adds nothing. The unit bug is real but immaterial; leave it. | - | - |
| L | **Prior-soft 3D->2D edges into BA** _(now the critical path - 2026-09-12)_ | Promoted from deferred F into v1. Gated pack-track edges (Cauchy, lower weight) into Local + Global BA and into the pooled pin solve. Widen candidate search by current pose uncertainty instead of a fixed count, to break the bad-alignment-starves-constraints trap. **The full-holdout measurement makes this the only mechanism that can deliver §1 on this dataset** - portals supply 1 usable sight in 580 frames, prior-soft supplies 9 spread across the scan. The live-match gate must be conditioned on current alignment uncertainty rather than a fixed pixel threshold (`err_live_p50` 148-1187 px versus `err_prior_p50` ≈ 1.5 px, `pass_live=0`). | J, K1 | no |
| M | **Verify: fair metric + monotonicity** | Same-matched-subset comparison reporting median/RMSE/p95 (§5 #4), plus the sigma(log s)-and-ATE vs cumulative-constraint-count curve (§5 #5). | L | **yes** - glance curves/plots |

- **Ordering rationale:** G first so improvement is measurable before anything changes; H removes the ~5.3 m bias (variance work is pointless while a bias dominates); K1 before L so the distributed edges L adds actually persist; L is the slice that makes accuracy keep improving with constraints.
- **Stop / escalate:** pooled re-pin still meters-scale -> bounce §4 (frame/convention bug, not tuning); sigma(log s) flat after L -> pack tracks are not carrying scale information and the VIO/ARposes decision must be reopened; BA unstable or tracking breaks -> reopen weights or Local-only; temptation to build an S10 overlay KF BA -> reject.
- **Agent vs human:** G-L agent; M agent run plus human glance (§5 #5, #6).

## 7. Implementation notes

- **Board:** initiative `fuse-prior-into-ba` (`plans/fuse-prior-into-ba.md`). Duplicate `portal-ba-constraints` parked 2026-09-10.
- **Related:** `plans/hybrid-prior-pose-graph.md` S10 parked; this initiative **supersedes** S10 for the accuracy goal (no second KF BA).
- **Reuse:** `Map::ApplyScaledRotation` / loop CorrectLoop patterns for bulk Sim(3) apply.
- **Weight:** mirror overlay `kPortalCornerBaseWeight / max(1, reproj_px)` unless verify shows BA instability.
- **Approvals (2026-09-12):** Mass-approved all remaining v1 slices **A–E**. Implement one slice per implementer; review + verify between slices; do not re-ask until blocked or done. F deferred.
- **Approvals (2026-09-12, re-lock):** Human picked **B** — pooled re-pin + live scale DOF + prior-soft promoted into v1. Human **mass-approved G–M** the same day. One implementer per slice; fresh code-reviewer then behavior-verifier between slices; do not re-ask until blocked or done. Human glance still required at **M** (§5 #5, #6).
- **Open question carried into G:** `plans/hybrid-prior-pose-graph.md` records SLAM→GT scale as **0.9685** (S9, 2026-09-07) and **0.4724** (AR probe, 2026-09-08) — a factor of ~2.05. They come from runs with different map splits (2 maps vs 1) and monocular map scale is arbitrary per map, so they may simply not be comparable; but S10's whole premise ("overlay fits 0.5131 where truth is 0.9685") rests on the first figure, while the overlay's 0.47–0.51 agrees with the second. Reconcile read-only in G before any drift modelling is justified.
- **VIO/ARposes decision stands rejected** (human, 2026-09-08) — but it is the named escalation if σ(log s) stays flat after L. Measured there: SLAM→AR 0.4760 vs SLAM→GT 0.4724 (0.76% over a 22 m baseline) against ~0.17 m of portal baseline.

## 8. Verification record

- **A (2026-09-12):** Review approve; verify pass. Store in libORB_SLAM3; append on overlay-accepted sights; `portal_ba_store` on status. No BA/domainize yet.
- **B (2026-09-12):** Review revise→approve (tracking-loss must not clear domainized); verify pass. Domainize-once via ApplyScaledRotation; overlay identity; Map::clear clears flag. §5 #6 wording updated to map-reset.
- **C (2026-09-12):** Review approve; verify pass. Portal corner edges in Local+Global BA via EdgeSE3ProjectXYZOnlyPose when domainized; Map portal BA edge counter.
- **D (2026-09-12):** Review approve; verify pass. Status `domainized` + `portal_ba_edges` + `portal_ba_store`; trajectory raw Atlas Twc when domainized; `syncAtlasDomainizedState` after Map::clear; API/docs updated.
- **E (2026-09-12):** Harness review revise→approve (honesty); verify **bounce §4**. Plumbing OK (`domainized`, `portal_ba_edges=8`). Accuracy fail: pooled/map0 atlas domain **~5.2–5.3 m**; active map2 **33 cm** vs S9 overlay **3.6 cm**. Evidence: `out/holdout_fuse_ba_verify/` (`SLICE_E_EVIDENCE.md`, plots). Human glance + §4 bounce decision pending.

- **G (2026-09-12):** Review revise→approve; measurement-only, runtime behaviour unchanged (`[domainize]` still fires on the first gated sight at `seed_scale=1.0`). Pooled-sight Sim(3) solve + 7×7 Hessian diagnostics in `PortalObservationStore`, surfaced as `portal_pooled_sim3` on `/status`. Revise fixed two real defects: σ(log s) returned a false **0** for a singular Hessian (now NaN/`valid=false`), and the Hessian was evaluated **after** LM convergence rather than at the seed pin (now seed-pin primary, `solved_pin` reported separately). Seed-pin smoke (100 f): 1 sight cond 4.0e37 null=`log_scale` σ=nan `valid=0`; 2 sights cond 1.8e4 null=`omega_z` σ=0.0075; 3 sights cond 1.4e4 null=`omega_z` σ=0.0055. **Pooled solve wants scale ≈ 0.39–0.41 where the live pin still uses 1.0** — direct evidence for slice H. §5 #5 amended (see above). Reviewer nit carried forward: `_probe_pooled_seed_hessian_crosscheck.py` still has hardcoded job paths.
- **Scale reconciliation (2026-09-12, slice G probe):** `scripts/_probe_scale_reconciliation.py` — S9 baseline per-map Umeyama scales **0.159 / 1.739 / 0.403 / 1.251** (ratio 10.9×, 4 maps) vs S5′ single-map **0.476**, which matches the AR probe's 0.472. The 0.9685 and 0.4724 figures are **not comparable**: monocular gauge is arbitrary per map, so the headline numbers mix gauges. **S10's per-keyframe-drift premise rested on comparing unlike quantities** — it should not be revived on that evidence.

- **H (2026-09-12):** Review revise→approve; verify **pass**. Domainize now runs from a pooled solve over all stored sights, gated on seed-pin Hessian `valid` + `null_dominant_param != log_scale` + solved-pin `mean_reproj_px <= portal_max_reproj_px` + finite R/t with scale in range — never σ alone, never `solved_pin.sigma_log_s`. **Single-sight domainize is now blocked** (`sights=1 seed_valid=0 seed_null=log_scale`, no `[domainize]`), where slice E pinned on that same first sight at `reproj_px=4.03 seed_scale=1`. Domainize fires at 2 sights: `pooled_reproj_px=1.235 scale=0.4129` — the applied gauge is the pooled ~0.41, not the hardcoded 1.0. Review fix: alignment snapshot on the domainize frame now uses `setAlignmentIdentityAtlasDomainized` so `/status` cannot briefly disagree with `domainized: true`. Labelling verified before and after (never raw SLAM published as domain, including `?all_maps=1`). Evidence: `out/holdout_slice_h_verify/`.
- **The −5.3 m scare was scene geometry, not a relocated bug (2026-09-12).** A reviewer flagged `nav_pose` x ≈ −5.3 m, close to slice E's 5.33 m map-0 ATE. Independent GT check: the holdout scan **starts at the portal**, which sits near domain x ≈ −5.25 m. nav-vs-GT error is 0.183 m (frame 17), 0.203 m (18), 0.384 m (43), 2.59 m (99); over 50 matched frames median 0.603 / mean 0.702 / p95 1.08 / std 0.195 m. Slice E's signature was a near-**constant** 5.33 m (median ≈ mean ≈ p95, std ≈ 0). Different shape, so not the same failure.
- **Accepted weakness carried into L/M (2026-09-12):** the two-sight domainize threshold is **emergent, not designed**. The gate passes because the dominant null direction flips `log_scale` → `omega_z` at two sights, not because baseline carries scale — both sights are the **same portal** from viewpoints ~0.17 m apart. This is a real improvement over slice E but is **not** a scale constraint. Do not treat 2-sight domainize as proven for ATE; that is what L (prior-soft into the solve) and M (monotonicity curve) exist to settle.

- **J (2026-09-12):** Review revise→approve; verify **pass**. Per-map domain Sim(3) is now a live estimate. First attempt had a **high-severity gauge-coherence bug**: `resolveSightsForPooledSolve` mixed append-time SLAM-gauge poses (unresolved `kf_id`) with live Atlas poses that are **domain** frame after `ApplyScaledRotation`, and `twcDomainFromPin` applied a SLAM→domain pin to already-domain poses. Symptom was post-domainize pooled reprojection of **698 / 926 / 1017 px**. Worse, `S_delta = S_candidate * S_applied⁻¹` compared absolute pins, so a *correct* post-domainize solve (≈ identity) would have produced a delta ≈ `S_applied⁻¹` and bulk-rewritten the map to **undo domainize** — the smoke's apparent safety came from accidentally reusing the slice H residual gate, not from designed safety. Fixed by branching the solve: pre-domainize stays absolute SLAM→domain; post-domainize solves an **incremental ΔSim(3) about identity** from live Atlas poses only, excluding unresolvable sights, with the delta taken against the **cumulative live pin** and its own `pooledSolvePassesLiveScaleGate`. σ for hysteresis now evaluated at the operating incremental pin. Rewrite serialised per `LoopClosing::CorrectLoop` with Global BA abort and a 5 s timeout. Verified: post-domainize pooled reprojection **6.03 px** (was 1017), domainize unchanged (2 sights, 1.226 px, scale 0.4132), **zero** live corrections applied (gate correctly refuses `seed_valid=0` / 4 residuals), labelling unchanged. Evidence: `out/holdout_slice_j_verify/`. Lifecycle partial: tracking-loss preservation confirmed, `Map::clear` on live incremental state not re-exercised.
- **Sentinel defect found in slice J, with retroactive consequences (2026-09-12).** `kUnresolvedPortalKeyframeId` was **0**, but `kf_id = 0` is a **valid** initial keyframe. `AddPortalCornerBaEdges` skips sights whose `kf_id` equals the sentinel, so **every sight observed on keyframe 0 was silently dropped from BA**. Slice E logged `[portal_store] append map=0 kf=0 frame=0` and pinned map 0 on kf 0 — so the **first pin sight's four corner edges never entered BA at all**. Sentinel changed to `ULONG_MAX`. This is a real contributing cause of slice E's "8 edges on 2 KFs"; window retention (slice K) remains the larger one.
- **Evidence evaporation measured (2026-09-12, slice J verify) — the gating risk for §1.** Over the 100-frame smoke: 11 portal detections → **3** store appends → **1** resolvable at the frame-28 pooled solve. The frame-1 sight never bound to a keyframe; the frame-18 sight's `kf=9` was culled within **≈10 frames**. `KeyFrameCulling` has no constraint exemption. So usable evidence *shrinks* while constraint count grows — the §1 goal is unreachable through the pooled re-pin mechanism until slice **K1** lands. §4 and §6 K amended accordingly.

- **K1 — constraint-evidence survival (2026-09-12): approved after two revise rounds; **it reclassified the blocker**.**
  - *Shipped:* anti-cull exemption in `KeyFrameCulling` bounded to the newest 64 sights per map (O(1) set lookup, no lock inversion, exempt KFs stay normal in the covisibility graph and spanning tree); unbound sights rejected at `tryAppend`; post-domainize resolvability requires a **live** keyframe pose, with the frozen domain-gauge snapshot demoted to a culled-keyframe-only degraded mode; resolvable count co-reported with `solved_reproj_rmse_px`.
  - *Round-1 defect caught in review:* `resolvable` rose 1 → 3 while `solved_reproj_rmse_px` rose **6.03 → 372**, because a frame-1 sight captured **before mono initialisation** (init is at frame 10; that frame's timing shows `pose=0.0`) was resurrected via `pose_in_domain_gauge` and mixed with live Atlas poses. At 372 px the slice-J live-scale gate could never fire, so the fix had *entrenched* the blockage it was written to remove. Third instance of the same shape as slice G — **a headline count improving while the quantity it proxies degrades**.
  - *Round-2 defect caught in review:* an unrequested decoupling of the store from `refine.portal_corners_added` bypassed overlay's 0.12 baseline dedup and admitted a near-duplicate sight (kf 15, two frames after kf 14), giving 3 sights at 22.4 px. Reverted.
  - *Comparability lesson:* 1.77 px (absolute gauge, pre-domainize) and 22.4 px (incremental gauge, post-domainize) were **never comparable** — `PooledSolveOptionsForMap` switches gauge at the domainize boundary. Residuals must carry a gauge label.
  - *Not a bounce:* `scripts/_probe_single_sim3_sufficiency.py`'s finding stands — this was admission policy, not single-Sim(3) model error.
- **Evidence supply is the new blocker (2026-09-12, from K1) — supersedes the evaporation finding below as the gating risk for §1.** With pre-init sights correctly rejected and overlay dedup restored, the 100-frame smoke stores **one** bound sight and therefore **never domainizes** (the threshold is 2). This also means slice H's and J's domainize at scale ≈ 0.41 was **partly carried by the garbage pre-init sight**, and the absolute solve masked it because it can absorb one bad sight jointly with a good one. Honest admission exposes that 11 detections yield 1 usable observation per 100 frames. §4 and §6 amended; new slice **N** added ahead of L.

- **Full-holdout constraint-supply measurement (2026-09-12, 580 frames, post-K1) - `out/holdout_k1_supply/`.** Run on the real holdout scan with honest admission, to size the supply problem before building slice N. **It refuted slice N and re-pointed the initiative at L.**
  - *Replay:* 580 requests, 485 successful tracks, 0 failed. Prior pack: 1655 images, 184210 tracks, **2 portals**, 84807 covis edges.
  - *Portal supply:* **11** sights, **all in frames 1-30**, all portal `HBORRXLUOD6`. Frames 1-8 are pre-init (correctly rejected by K1). Frame 18 is the **only** store append (kf 9). Frames 28 and 30 are overlay-deduped near-duplicates. Frames 31-580 - **95% of the scan** - yield nothing, and this is a property of the data, not a skipped detector: `[timing]` reports `portal≈46 ms` on every sampled frame through frame 483.
  - *Consequence:* the pooled solve runs once at `sights=1`, `seed_residuals=4`, `cond=1.19e9`, `null=log_scale`, `sigma_log_s=nan` -> rank deficient -> **no domainize for the entire scan**. Final `alignment.portal_observations=0`, `alignment.scale=0.0`.
  - *Prior-soft supply, by contrast, is spread across the scan:* **9** edges accepted against pack images 619/633/639/649/1056/1116/1122/1349/1351/1405 - genuinely different vantages - with 68-368 inliers each.
  - *And the trap is visible in the skips:* `err_prior_p50` ≈ 1.5 px versus `err_live_p50` = 1187 / 250 / 148 px at frames 100 / 110 / 120, with `pass_live=0` every time while `pass_prior` is 6-11.
  - *Verdict:* the §1 goal is **unreachable through portals on this dataset** and must be delivered by **L**. Slice N withdrawn unbuilt.

## Bounce log

### 2026-09-12 — §4 reopened after slice E accuracy fail

New truth from `out/holdout_fuse_ba_verify/` (log + `compare_summary.json`). Two independent defects; neither is "BA weights need tuning".

**1. The domainize trigger pins on the worst available sight.** §4 locked "domainize on the **first** sight that passes the gate". Empirically the first detection of a portal is the most distant/oblique one, so this systematically selects the worst pin:

| Map | Pinned at | Pin reproj | Better sight available shortly after |
|---|---|---|---|
| 0 | frame 0, **kf 0** | 4.03 px | frame 1 = 0.61 px, frame 2 = 0.87 px, frame 3 = 1.79 px |
| 2 | frame 568, kf 179 | 7.48 px (gate is 8.0) | frame 572 = 2.83 px |

Map 0 pinned on a **1-keyframe map** before any geometry existed.

**2. The gate never got its scale/sanity checks.** §4 said "plus scale/sanity checks to be named in implementation notes". `portalSightPassesDomainizeGate` (`slam_server.cc`) checks only: non-empty short id, finite `reproj_px ≤ portal_max_reproj_px`, finite matrices. Seed scale is **hardcoded `1.0f`**; `LocalScaleEstimate` is computed and logged but never consulted — every sight in the run logged `local_scale_valid=0` and pinned anyway.

**Error shape confirms a bad pin, not drift or scale walk.** Map 0: median 5.33 / mean 5.55 / RMSE 5.57 / p95 6.27 m — a near-constant offset. A scale error would start near 0 at the pin and grow. The whole GT trajectory is only ~5 m across, so map 0 is entirely misplaced.

**3. The BA injection is effectively untested by this run.** `AddPortalCornerBaEdges` binds a sight only when its observing KF is in the BA window (`queryByMapAndKeyframeIds`). Across 580 frames the run produced **8 pose-only corner edges on 2 KFs per map**, and those KFs leave the local window almost immediately; Global BA only runs on loop closure. `prior_soft_edges_accepted: 0`. Pose-only `EdgeSE3ProjectXYZOnlyPose` edges on 2 early KFs **cannot** correct a global misplacement of the map regardless of weight. So the initiative's central claim — absolute priors inside the existing BA improve accuracy — was never actually exercised.

**4. The §5 #4 comparison is not apples-to-apples.** Gate compares medians across different matched subsets:

| Series | Matched | Median | RMSE | p95 |
|---|---|---|---|---|
| S9 overlay baseline | 167 | **0.036 m** | 4.95 m | 13.38 m |
| Atlas domain, active map 2 | 42 | 0.332 m | **0.367 m** | 0.50 m |

The baseline wins on median but is catastrophic in the tail; the new path is ~13× better on RMSE. Median-only on disjoint subsets cannot adjudicate this.

**Invalid sections:** §4 "Domainize trigger" (first-passing-sight + unspecified scale checks) and §4 "BA injection scope" (sparse pose-only edges as the accuracy mechanism). §5 #4 needs a fair metric. §1 Intent stands — the goal is unchanged.

**Resolved 2026-09-12 — human picked B.** §4 re-locked (see "Re-lock 2026-09-12" in §4), §5 #4 replaced with a same-subset median/RMSE/p95 comparison, §5 #5 added as an explicit **monotonicity** requirement, §6 extended with slices G-M. Stage back to 6-work-breakdown.

**Prior art that set the direction.** The overlay initiative (`plans/hybrid-prior-pose-graph.md`) already ran the "more constraints" experiment and it failed three separate times, which is why the re-lock is structural rather than a tuning pass:

| Evidence | Result |
|---|---|
| Prior-soft cadence raised to every frame (S9', 2026-09-08) | 7 -> **76** accepted edges; ATE 0.297 -> **0.292 m**; pin scale stuck ~0.465 |
| Corner factors 4 -> 8 -> 12 across sights (`_probe_sim3_conditioning.py`) | 7x7 Hessian condition ~1e18, `log_scale` in the null direction throughout |
| `_probe_single_sim3_sufficiency.py` | Best-possible single Sim(3): 8.4 mm (map0) / 29.3 mm (map1) vs published 132.7 mm / 4288.5 m |

Distilled: **constraints reduce variance, never bias**, and the third row says intra-map drift (1-3 cm) is an order of magnitude below estimator error, so the gauge must be fixed before drift modelling earns its place. Both failures in this initiative are the same shape — slice E's 5.3 m is a near-constant offset (median 5.33 / p95 6.27 m over a ~5 m trajectory), i.e. bias from a bad pin, and after domainize there is no free scale parameter for any additional edge to move.

## Gate (stage 11)

- **Decision:** _(open — blocked on §5 #4 / §4 bounce)_  
- **Notes:** Do not accept v1 until domain ATE passes after design fix.
