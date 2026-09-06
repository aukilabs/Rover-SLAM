# Initiative: Hybrid prior map + pose graph

Status: active  
Tier: feature  
Stage: 10-integrate  
Skips: _(none)_  
Last updated: 2026-09-05

Living document — update in place when later work changes earlier conclusions.  
**§1 Intent (feature) and §4 System design stay distinct — never merge.**

---

## 1. Intent

- **Customer / internal need:** Navigate indoors against a reconstruction-server prior (COLMAP + ALIKED descriptors + placed portals) while still running live SLAM so unmapped or stale regions remain usable.
- **Framing:** Pure atlas/reloc is a poor fit for ALIKED v1 (BoW deferred) and for incomplete priors. Portals are globally unique hard anchors; prior views give soft geometric constraints; live SLAM covers novelty. Treat live odometry/map edges, prior match edges, and portal absolute factors jointly as a **pose-graph** problem (not prior-only localization).
- **Why now:** ALIKED+LightGlue live stack is accepted; next product step is domain-aligned navigation with recon outputs without waiting on EigenPlaces as primary reloc.
- **Success look like:** Offline converter turns recon output into a load-cheap prior pack; `slam_server` (or equivalent) runs live ALIKED SLAM, seeds/re-anchors via portals, adds verified prior constraints when covisible matches succeed, and publishes poses in the domain frame with graceful behavior when the prior is missing or stale. **No mesh occlusion required in v1.**
- **Locked:** 2026-09-02 (human confirmed §1 draft).

## 2. Alternatives

- **Existing / partial:**
  - **Live SLAM + portal Sim(3) only** (current `slam_server`): domain poses after QR, but no prior landmarks; drift/loss between portals; no use of COLMAP+ALIKED pack.
  - **ORB-SLAM atlas `.osa` load + Localization Mode:** save/load exists; cold reloc depends on BoW — unfit for ALIKED v1 (`loopClosing: 0`).
  - **Prior-only localization** (track solely against COLMAP): fails where recon is stale/missing — rejected by product need.
  - **EigenPlaces as primary reloc:** useful soft retrieval later; false positives in similar rooms — not v1 primary.
- **Comparison:** Portal-only is too weak for “navigate in recon.” Atlas/BoW is the wrong place-recog stack. Prior-only rejects novelty. EigenPlaces alone is unsafe as commit.
- **Decision:** **Proceed** with hybrid live SLAM + offline prior pack + portal hard factors + verified prior soft edges in a pose graph. Defer EigenPlaces and mesh occlusion.

## 3. Priority

- **Rank vs backlog:** Top-of-queue follow-on after accepted ALIKED initiative (`plans/aliked-lightglue-mrnerf.md` done).
- **Capacity / opportunity cost:** Displaces merge-to-`prototyping` polish and EigenPlaces; acceptable — this is the stated next product step.
- **Decision:** top-of-queue.

## 4. System design

- **Products / codebases impacted:** Rover-SLAM (`slam_server` primary); offline converter script (recon → prior pack). No reconstruction-server changes required for v1 beyond consuming its existing COLMAP/ALIKED/portal outputs.
- **Decision (architecture):** **Design change needed — `slam_server` overlay.** Live ORB-SLAM Atlas/Tracking stays the frontend; overlay loads prior pack, runs portal + prior match factors, maintains a pose graph, publishes domain poses. Avoid deep Tracking/Optimizer rewrites in v1.
- **Constraints:**
  - Offline converter preferred (heavy covisibility/descriptor baking at convert time; cheap load).
  - No mesh occlusion in v1.
  - EigenPlaces deferred.
  - ALIKED+LightGlue live path remains default.
  - Atlas may contain **multiple live maps** after tracking loss (ORB-SLAM3 `CreateNewMap`).
- **Interfaces / boundaries:**
  - **Prior pack on disk (locked 2026-09-02 — full match-partner pack):** converter writes everything needed for covisible prior matching without recomputing COLMAP tracks at load time:
    - **Images:** id, path or blob ref, intrinsics, pose in prior/domain frame (as recon provides), optional thumbnail.
    - **Keypoints + descriptors:** per-image ALIKED kpts + desc tensors/arrays (same live stack family).
    - **Covisibility adjacency:** image–image edges (weights / shared-track counts) for candidate selection.
    - **Point tracks:** 3D points + observation lists (image id, kpt index) so the overlay can project/verify geometry, not only 2D match.
    - **Portals:** short ids + domain poses + size (may reference existing portal manifest or embed a copy).
    - **Manifest / index:** version, coordinate frame notes, file layout for mmap-friendly load.
  - Overlay pose graph: live KF/frame nodes (per map), prior relative/absolute factors, portal absolute factors, **per-map** live↔domain Sim(3) when pinned.
  - Trajectory/status API follows multi-map publish policy below; overlay nav pose vs raw SLAM for debug.
- **Risks / unknowns:**
  - Disconnected live maps keep independent gauges until portal/prior pins them to domain.
  - Prior/live disagreement in stale regions → robust kernels / gate prior edges.
  - Overlay vs internal SLAM can diverge; publish overlay as navigation pose.
  - Pack size / load time for large recon — converter may shard; v1 can require “venue-sized” packs.
  - **Dynamic scenes (people / moving objects):** classic monocular SLAM (ORB-SLAM lineage) assumes a mostly static world; live tracking can degrade when people walk through the frame. **Locked out of scope for Rover-SLAM v1 (2026-09-05):** no person masks, no dynamic-object filtering, no design change — accept as known paper/stack limitation. Prior soft edges may still favor static structure when reproj gates pass, but that is incidental, not a dynamics solution.
- **Multi-map output (locked 2026-09-02):**
  - **Active map not domain-aligned:** emit **active map only**. Withhold previously domain-aligned maps so the client demo sees them “disappear” → tracking loss is obvious.
  - **Active map domain-aligned:** emit a **merge of all maps that have a domain pin**, all in domain coordinates. Re-pinning after loss restores prior domain-aligned segments alongside the new map.
  - Never silently merge unaligned maps into domain.
- **Between portals (design consensus):** select covisible prior images via adj + pose proximity; LightGlue vs live ALIKED; gate with track reprojection; soft pose-graph edges — no whole-cloud dump, no mesh occlusion.
- **Recovery ladder (design consensus):** reject bad frames → local recover → pose-prior reloc → portal → EigenPlaces later.
- **Decision (locked 2026-09-02):** design change needed — `slam_server` overlay + multi-map publish policy + full prior pack (images/kpts/descs/covis/tracks/portals).

## 5. Verification design

- **Depth (locked 2026-09-02):** lean — static/API/schema checks by agents; **human demo** owns multi-map disappear / reappear UX and “feels navigable between portals.”
- **End-to-end acceptance (v1):**
  1. Converter produces a valid prior pack (manifest + images + kpts/descs + covis adj + tracks + portals) from a small fixture recon (or documented sample).
  2. `slam_server` loads the pack without crashing; status/API reports pack loaded (image/track counts or equivalent).
  3. Without domain pin: trajectory/status emits **active map only** (aligned history withheld).
  4. After portal (or equivalent) pin: poses publish in domain frame; when active map is aligned, **all domain-pinned maps** appear merged in domain coords.
  5. After tracking loss (new map, not yet pinned): previously aligned maps **disappear** from client feed; after re-pin they **return** with the new map — human confirms in demo app.
  6. At least one gated prior soft edge path is observable in logs/status when live↔prior LightGlue+reproj succeeds (may be off by default until slice lands — then assert field/counter > 0 on fixture).
- **Agent checks (per slice / integrate):**
  - Schema: pack files/fields present; loader rejects corrupt/missing required parts with a clear error.
  - API contract: multi-map publish rules encoded in JSON (or equivalent) match §4; unit/static review if no automated client.
  - Regression: existing portal-only / no-pack `slam_server` path still starts and tracks (no prior required).
- **Human checks:** phone/`index.html` (or equivalent) demo — lose track → history vanishes; re-scan portal → history + new segment in domain; brief walk where prior should help (qualitative only).
- **Out of scope for v1 verify:** EigenPlaces, mesh occlusion, quantitative ATE vs COLMAP, large-venue pack perf SLOs, CI GPU gate on every PR, **dynamic-object / people robustness** (no demo requirement under foot traffic).
- **Decision:** lean §5 locked 2026-09-02.

## 6. Work breakdown (AI-oriented)

- **Shape (locked 2026-09-02):** vertical thin slices S1→S6.
- **Escalate / bounce:** if loader needs Tracking rewrite, pack schema can't feed LightGlue, or publish policy fights Atlas map ids → `/bounce-plan` §4 (do not silently widen).
- **Human after verify:** S3 (disappear UX), S6 (full demo). S1–S2/S4–S5 agent-lean §5 unless blocked.

### S1 — Converter + prior-pack schema
- **Goal:** Offline tool writes full pack (manifest, images, kpts/descs, covis adj, tracks, portals) from small fixture/docs recon.
- **Deps:** none. **Agent.** Context: §4 pack schema; recon COLMAP+ALIKED layout.
- **Done when:** fixture pack validates; missing-required fails clearly (§5.1).

### S2 — Pack load in `slam_server`
- **Goal:** Optional CLI/config path loads pack; status reports counts; no-pack path unchanged.
- **Deps:** S1. **Agent.** Context: `Examples/Monocular/slam_server.cc`.
- **Done when:** §5.2 + no-pack regression.

### S3 — Multi-map publish policy
- **Goal:** Trajectory/status: unaligned active → active only; aligned active → merge all domain-pinned maps in domain coords.
- **Deps:** S2 (or reusable alignment flags). **Agent + human demo.**
- **Done when:** §5.3–5; human confirms disappear/reappear.

### S4 — Portal pin → overlay pose graph
- **Goal:** Overlay graph: per-map live nodes, portal hard factors, per-map Sim(3) pin; nav pose = overlay when pinned.
- **Deps:** S3 helpful. **Agent.** Prefer thin overlay over Atlas rewrite.
- **Done when:** portal pin → domain poses; pin state visible in API/logs.

### S5 — Prior match soft edges
- **Goal:** Covis candidate prior images → LightGlue vs live ALIKED → reproj gate → soft factors; status/counter when edges accept.
- **Deps:** S2 + S4. **Agent.** No mesh; EigenPlaces out.
- **Done when:** §5.6 on fixture (or documented skip + log path).

### S6 — Human demo polish
- **Goal:** Client/`index.html` + docs for pack path, status fields, lose/re-pin UX; ops notes.
- **Deps:** S3–S5. **Human primary**; agent polish.
- **Done when:** human sign-off on §5 human checks.

**Implement only via slice gate** (approve one | mass-approve remaining | pause).

## 7. Implementation notes

- **Approvals:** Mass-approved all remaining slices (S1–S6) 2026-09-02 — implement one slice per subagent with review/verify between; re-ask only if blocked or plan bounce.
- **Progress:** S1–S6 agent cycles complete on board. S3/S6 human disappear/reappear demo **pending** (gate).

## 8. Verification record

- **S1:** pass — fixture pack validates; missing `tracks.json` / manifest rejected clearly.
- **S2:** pass — loader + status `prior_pack` counts; WSL harness load of fixture; slam_server startup SKIP (vocab).
- **S3:** pass (agent) — `multi_map_publish` policy fields encode §4; **human disappear UX pending**.
- **S4:** pass after compile retry (`mutable alignment_mutex_`); overlay/nav_pose/pose_source in API; slam_server builds on WSL.
- **S5:** pass — `overlay.prior_soft_edges_accepted` wired; fixture smoke did not increment on synthetic geometry (`lightglue_unavailable` / reproj); increment path present.
- **S6:** pass (docs/legend); **human phone walk pending**.

## Gate (stage 11)

- **Decision:** _(pending)_  
- **Notes:**
